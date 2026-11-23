"""Teleoperated demo recording on the real robot, into a demo dataset (``real_world_rl.dataset``).

The env is the HIL actor's env (``build_hil_env(args, role="actor")``): the operator drives the
robot through its teleop source and the intervention wrapper executes the operator's actions.
While nobody intervenes the recorder holds the robot still. Every step of every episode (hold
steps too) is written to the dataset at ``--output-dir`` when the episode ends, kept and
discarded episodes alike (``meta.json`` ``status``/``discard_reason``); only the episode being
recorded is held in memory. Rerunning with an existing ``--output-dir`` appends to it (its header
must match this run's env) and its kept episodes count toward ``--num-successes``. The training
view (``real_world_rl.dataset.iter_transitions``, what ``load_demo_pkls`` loads from a dataset
directory) is the kept episodes' intervened steps, as the old recorder's pkl held them. Review
videos (``video/<key>.mp4`` per ``rgb_*`` key) are written by the dataset, at the control rate.
See ``https://real-world-rl.readthedocs.io/zh-cn/latest/en/reference/recording-demos.html``.

Episode outcome, read from the intervention wrapper's info keys so any event path works:

- the episode ends on ``terminated`` or ``truncated`` (an operator episode-end press truncates);
- it counts as a success when an explicit success outcome or ``success`` event is reported (a reward
  classifier), and no ``failure`` event fired;
- a ``rerecord`` event discards the episode at once; a new attempt starts if its budget remains;
- only successful episodes with at least one training row are kept;
- an ``operator_quit`` event (the ``q`` key) ends the episode and stops recording after it;
- an episode cut short by Ctrl-C or an error is stored as discarded (``interrupted``) when it
  has a step, after the env released the robot.

Episode boundaries (``--episode-end-behavior``, default ``confirm``): the operator at the event
device starts every episode with SPACE after a reset, and after it ends the arm holds where it is
(POST_EPISODE: o/c gripper, n resets after a gripper check, q quits; see
``real_world_rl.teleop.core.session.ConfirmSession``); only the episode's own steps are recorded.
``auto`` resets right after each episode, ``hold`` waits for a neutral-confirmed n.

The hold policy (``hold_action_fn_for``, injectable through ``run_recorder``'s
``hold_action_fn_factory``) is reset after every env reset and observes every step; on a
``cartesian_delta`` arm it holds a fixed pose (``real_world_rl.teleop.core.hold.CartesianHold``).

The stored ``reward``/``terminated`` are the env's. Without a reward classifier, pass
``--reward-from-teleop-events`` (``TeleopEventRewardWrapper`` in the ``build_hil_env`` chain): a
``success`` event then ends the episode at once and its step is stored with
``reward=--teleop-success-reward`` and ``terminated=True``; otherwise every step carries reward 0.
"""
from __future__ import annotations

import dataclasses
import json
import logging
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, Literal, Mapping, Optional, Sequence

import numpy as np
import torch

from real_world_rl.supervise import Heartbeat
from rl_garden.common.cli_args import ObservationArgs
from rl_garden.common.env_args import EnvBackendArgs, EnvRunArgs

from real_world_rl.contracts.action import (
    LEGACY_ACTION_CONTRACT,
    action_dim_of,
    demo_contract_of,
    get_env_action_contract,
    num_arms_of,
    num_grippers_of,
)
from real_world_rl.dataset import DatasetMismatchError, DatasetWriter
from real_world_rl.envs.wrappers.intervention import InterventionWrapper
from real_world_rl.loops.actor import ActorLoop
from real_world_rl.teleop.core.hold import (
    HoldPolicy,
    as_hold_policy,
    hold_action_fn_for,
)
from real_world_rl.teleop.core.contract import TeleopReading, TeleopSource
from real_world_rl.teleop.core.operator_keys import operator_quit_requested
from real_world_rl.teleop.core.composite import (
    CompositeTeleopSource,
    ToggleableTeleopSource,
)
from real_world_rl.teleop.leader.joint import TELEOP_SYNCING_EVENT
from real_world_rl.training.args import (
    HilEnvArgs,
    RealWorldFrankaArgs,
)

__all__ = [
    "HoldActionFn",
    "HoldPolicy",
    "LeaderFollowerGapError",
    "RecordDemosArgs",
    "RecordedEpisode",
    "check_leader_near_follower",
    "dataset_header_of",
    "executed_gripper_actions",
    "gripper_actions_after_reset",
    "hold_action_fn_for",
    "intervention_wrapper_of",
    "obs_spec_of",
    "open_dataset",
    "parse_args_with_preset",
    "parse_record_args",
    "record_demos",
    "run_recorder",
]

# Both real backends open the gripper on reset, and +1 means open for both.
_GRIPPER_ACTION_AFTER_RESET = 1.0
_STATE_KEY = "state"
# Config keys never copied into the dataset header (daemon credentials).
_REDACTED_CONFIG_KEYS = ("token",)

# Gripper action(s) to hold, one per arm (a number for one arm) -> the (A,) action that keeps the
# arms where they are. A ``HoldPolicy`` also receives the recorder's reset/step hooks.
HoldActionFn = Callable[[Any], torch.Tensor]
# The built env -> its hold policy (``run_recorder``'s injection point).
HoldPolicyFactory = Callable[[Any], HoldActionFn]


@dataclass
class RecordDemosArgs(HilEnvArgs, RealWorldFrankaArgs, EnvRunArgs, ObservationArgs, EnvBackendArgs):
    """Env, teleop and observation flags are the HIL training entrypoints' (``--env-backend``,
    ``--real-robot.*``, ``--teleop-device``, ``--spacemouse-mode``, ``--teleop-event-device``,
    ``--teleop-onset``, ``--obs.*``, ...): pass the ones of the run that will load the demos, so
    the recorded observations and actions match its env."""

    # The recorder never evaluates; no eval env.
    eval_freq: int = 0
    # On by default here (off for training): the operator marks every successful demo with the
    # success key anyway, and a success stored as reward 0 / done False cannot be relabelled later.
    # The success key therefore also ends the episode. ``--no-reward-from-teleop-events`` restores
    # the env's own reward.
    reward_from_teleop_events: bool = True

    # Episode boundaries (HilEnvArgs): the recorder defaults to "confirm", where the operator at
    # the event device holds the arm after each episode (o/c gripper), resets it (n, after a
    # gripper check) and starts the next episode (SPACE); "auto" resets at once.
    episode_end_behavior: Literal["auto", "hold", "confirm"] = "confirm"

    # The demo dataset directory (``real_world_rl.dataset``). Required. A new directory is
    # created; an existing dataset is appended to when its header matches this run's env.
    output_dir: Optional[str] = None
    # Stop once the dataset holds this many kept (successful) episodes, counting the ones an
    # appended-to dataset already has.
    num_successes: int = 20
    # Stop after this many completed attempts, including failures, rerecords and timeouts.
    # None keeps recording until num_successes; 1 enables manually restarted commissioning runs.
    max_episodes: Optional[int] = None
    # Episode step cap for the env: replaces --real-robot.max-episode-steps,
    # --real-robot-bimanual.max-episode-steps when set.
    max_episode_steps: Optional[int] = None
    # joint_abs: refuse to start an episode while the leader is further than this from the
    # follower in any joint (rad). Below it, --teleop-onset (ramp or arm) handles the takeover.
    # In confirm mode the start key stays locked instead (READY names the joints).
    start_tolerance_rad: float = 0.8
    # Also record the leader-arm onset catch-up steps of the ramp policy (tagged
    # ``teleop_syncing``) as ordinary supervised demo rows; by default they are left out.
    keep_syncing_steps: bool = False


@dataclass(frozen=True)
class RecordedEpisode:
    """One finished episode, already written to the dataset (its rows live on disk, not here).
    ``index`` counts this run's attempts; ``dataset_index`` is the episode's index in the dataset.
    ``discarded_reason`` says why it was not kept (``None`` when kept); ``training_rows`` is the
    number of its steps the training view selects; ``operator_quit`` whether the operator asked
    to stop recording during it."""

    index: int
    steps: int
    success: bool
    discarded_reason: Optional[str] = None
    operator_quit: bool = False
    training_rows: int = 0
    dataset_index: Optional[int] = None

    @property
    def kept(self) -> bool:
        return self.discarded_reason is None


class LeaderFollowerGapError(RuntimeError):
    """The leader arm is too far from the follower to start an episode."""


# -- hold action -------------------------------------------------------------------------


def gripper_actions_after_reset(num_grippers: int) -> Any:
    """The gripper action(s) to hold right after a reset (open): a number for one gripper, one
    entry per gripper otherwise (see ``HoldActionFn``)."""
    return _GRIPPER_ACTION_AFTER_RESET if num_grippers <= 1 else [_GRIPPER_ACTION_AFTER_RESET] * num_grippers


def executed_gripper_actions(executed: Any, num_grippers: int) -> Any:
    """The gripper action(s) of the executed action ``executed`` (its last ``num_grippers``
    dims, clipped as the env clips them), in the form ``gripper_actions_after_reset`` returns."""
    grippers = torch.as_tensor(executed).reshape(-1)[-num_grippers:].clamp(-1.0, 1.0).tolist()
    return grippers[0] if num_grippers == 1 else grippers


def _validate_episode_limit(args: RecordDemosArgs) -> None:
    from real_world_rl.teleop.core.session import (
        validate_episode_end_behavior,
        validate_gripper_fractions,
        validate_neutral_options,
    )
    validate_episode_end_behavior(args.episode_end_behavior)
    validate_neutral_options(args.neutral_threshold, args.neutral_stable_s)
    validate_gripper_fractions(args.gripper_open_fraction, args.gripper_closed_fraction)
    if args.max_episodes is not None and args.max_episodes < 1:
        raise ValueError("--max-episodes must be positive when set")


# -- episode-start sync ------------------------------------------------------------------


def check_leader_near_follower(
    env: Any, source: TeleopSource, contract: Mapping[str, Any], tolerance_rad: float
) -> np.ndarray:
    """Reads the leader once and returns ``|q_leader - q_follower|`` per joint (rad); raises
    ``LeaderFollowerGapError`` when any joint is further apart than ``tolerance_rad``.

    ``joint_abs`` only: the leader position is the source's unramped target decoded from the
    joint limits. The read goes through the motion half of a composite with the toggle released
    (``poll_with_toggle(False)`` for a leader arm), so neither the event device's latched
    buttons nor the onset policy change it. On a two-arm env every arm's leader is compared with
    its own follower: the result is ``(num_arms, num_joints)`` and the error names the arm
    furthest apart.
    """
    from real_world_rl.teleop.core.session import leader_follower_gaps

    if contract["action_mode"] != "joint_abs":
        raise ValueError(f"the leader/follower start check needs joint_abs, got {contract['action_mode']!r}")
    reading = _read_leader(source)
    if num_arms_of(contract) != 1:
        return _check_leaders_near_followers(env, reading, contract, tolerance_rad)
    error = leader_follower_gaps(env, reading, contract)[0]
    if float(error.max()) > tolerance_rad:
        raise LeaderFollowerGapError(
            f"leader arm too far from the follower to start an episode: |q_leader - q_follower| = "
            f"{np.round(error, 3).tolist()} rad, start_tolerance_rad = {tolerance_rad}; move the "
            "leader onto the follower's pose and restart"
        )
    return error


def _check_leaders_near_followers(
    env: Any, reading: TeleopReading, contract: Mapping[str, Any], tolerance_rad: float
) -> np.ndarray:
    from real_world_rl.teleop.core.session import leader_follower_gaps

    arm_names = list(contract["arm_names"])
    errors = leader_follower_gaps(env, reading, contract)
    worst = int(np.argmax(errors.max(axis=1)))
    if float(errors[worst].max()) > tolerance_rad:
        per_arm = ", ".join(f"{arm}: {np.round(error, 3).tolist()}" for arm, error in zip(arm_names, errors))
        raise LeaderFollowerGapError(
            f"leader arm {arm_names[worst]!r} too far from its follower to start an episode: "
            f"|q_leader - q_follower| = {np.round(errors[worst], 3).tolist()} rad, "
            f"start_tolerance_rad = {tolerance_rad} (per arm: {per_arm}); move every leader onto "
            "its follower's pose and restart"
        )
    return errors


def _read_leader(source: TeleopSource) -> TeleopReading:
    motion = source.motion if isinstance(source, CompositeTeleopSource) else source
    if isinstance(motion, ToggleableTeleopSource):
        return motion.poll_with_toggle(False)
    return motion.poll()


# -- recording ---------------------------------------------------------------------------


def record_demos(
    env: Any,
    args: RecordDemosArgs,
    *,
    hold_action_fn: HoldActionFn,
    writer: DatasetWriter,
    on_episode: Optional[Callable[[RecordedEpisode], None]] = None,
) -> list[RecordedEpisode]:
    """Runs episodes on ``env`` (a HIL env with the intervention wrapper), writing each one to
    ``writer`` when it ends, until the dataset holds ``args.num_successes`` kept episodes (those
    it already had count) or ``args.max_episodes`` attempts of this run ended, and returns this
    run's episodes. Failed/rerecorded/truncated attempts count as attempts too.

    Every step sends ``hold_action_fn(last executed gripper action)``; the intervention wrapper
    replaces it with the operator's action while the operator intervenes. A ``HoldPolicy``
    (``hold_action_fn_for``; a plain callable gets no-op hooks) is reset after every env reset
    and observes whether every step was intervened. Every step is written with the executed
    action (``info["intervene_action"]``) and its flags; the training view later selects the
    intervened ones (ramp-onset ``teleop_syncing`` steps only with ``args.keep_syncing_steps``).
    In ``joint_abs``, each episode starts with ``check_leader_near_follower``
    (``args.start_tolerance_rad``); with ``episode_end_behavior=confirm``
    (``_record_demos_confirm``) the start key stays locked instead. ``on_episode`` is called with every finished episode; an
    episode with the operator's quit key is the last one. An exception leaves the episode being
    recorded open in ``writer`` (``writer.active_episode``) for the caller to finish or abort.
    """
    from real_world_rl.training.runtime_paths import actor_control_hz

    _validate_episode_limit(args)
    hold = as_hold_policy(hold_action_fn)
    wrapper = intervention_wrapper_of(env)
    if args.episode_end_behavior == "confirm":
        return _record_demos_confirm(env, args, hold, wrapper, writer=writer, on_episode=on_episode)
    if args.episode_end_behavior == "hold":
        from real_world_rl.teleop.core.session import require_session_source, session_backend
        session_backend(env)  # reject unsupported backends before reset can move the arm
        require_session_source(wrapper.source)
    contract = get_env_action_contract(env) or LEGACY_ACTION_CONTRACT
    num_grippers = num_grippers_of(contract)
    control_hz = actor_control_hz(args)
    control_period_s = 1.0 / control_hz
    named_state = _named_state_reader(env)

    episodes: list[RecordedEpisode] = []
    num_kept = writer.num_kept
    index = 0
    heartbeat = Heartbeat.from_env()
    while num_kept < args.num_successes and (args.max_episodes is None or index < args.max_episodes):
        heartbeat.begin_long_operation()  # the reset homes the arm, far longer than a step
        obs, _ = env.reset(seed=args.seed if index == 0 else None)
        heartbeat.beat()
        hold.reset()
        if contract["action_mode"] == "joint_abs":
            check_leader_near_follower(env, wrapper.source, contract, args.start_tolerance_rad)
        if args.episode_end_behavior == "hold":
            from real_world_rl.teleop.core.session import wait_before_episode
            if wait_before_episode(env, wrapper.source, control_hz=control_hz,
                                   neutral_threshold=args.neutral_threshold,
                                   neutral_stable_s=args.neutral_stable_s) == "quit":
                break
            from real_world_rl.teleop.core.session import session_observe
            obs = session_observe(env)  # row 0 of the episode: the frame after the gate
        episode = _record_episode(
            env,
            obs,
            args,
            hold,
            index=index,
            num_grippers=num_grippers,
            control_period_s=control_period_s,
            writer=writer,
            named_state=named_state,
        )
        episodes.append(episode)
        num_kept += int(episode.kept)
        if on_episode is not None:
            on_episode(episode)
        index += 1
        if episode.operator_quit:
            if args.episode_end_behavior == "hold":
                from real_world_rl.teleop.core.session import notify_session_event
                notify_session_event(env, "STOPPING", "operator_quit")
            break
        if args.episode_end_behavior == "hold":
            from real_world_rl.teleop.core.session import wait_after_episode
            allow_next = num_kept < args.num_successes and (args.max_episodes is None or index < args.max_episodes)
            if wait_after_episode(env, wrapper.source, allow_next=allow_next,
                                  control_hz=control_hz, neutral_threshold=args.neutral_threshold,
                                  neutral_stable_s=args.neutral_stable_s) == "quit":
                break
    return episodes


def _record_demos_confirm(
    env: Any,
    args: RecordDemosArgs,
    hold: HoldPolicy,
    wrapper: InterventionWrapper,
    *,
    writer: DatasetWriter,
    on_episode: Optional[Callable[[RecordedEpisode], None]],
) -> list[RecordedEpisode]:
    """``record_demos`` with ``episode_end_behavior=confirm`` (``ConfirmSession``): every episode
    starts at the operator's start key after a reset and ends into ``POST_EPISODE``, where the
    operator asks for the next reset; only the steps between the start key and the episode's end
    are recorded (row 0 is a fresh observation taken at the start key). ``joint_abs`` locks the
    start key while the leader is too far from the follower instead of raising."""
    from real_world_rl.teleop.core.session import ConfirmSession, session_observe
    from real_world_rl.training.runtime_paths import actor_control_hz

    contract = get_env_action_contract(env) or LEGACY_ACTION_CONTRACT
    control_hz = actor_control_hz(args)
    session = ConfirmSession(  # checks the backend and the event device before any reset
        env, wrapper.source, control_hz=control_hz, contract=contract,
        start_tolerance_rad=args.start_tolerance_rad,
        gripper_open_fraction=args.gripper_open_fraction,
        gripper_closed_fraction=args.gripper_closed_fraction,
    )
    num_grippers = num_grippers_of(contract)
    named_state = _named_state_reader(env)
    episodes: list[RecordedEpisode] = []
    num_kept = writer.num_kept
    index = 0

    def budget_left() -> bool:
        return num_kept < args.num_successes and (args.max_episodes is None or index < args.max_episodes)

    def reset() -> None:
        env.reset(seed=args.seed if index == 0 else None)
        hold.reset()

    if not budget_left():
        return episodes
    started = session.first_episode(reset)
    while started == "start":
        obs = session_observe(env)  # row 0: the frame at the start key
        episode = _record_episode(
            env, obs, args, hold, index=index, num_grippers=num_grippers,
            control_period_s=1.0 / control_hz, writer=writer, named_state=named_state,
        )
        episodes.append(episode)
        num_kept += int(episode.kept)
        if on_episode is not None:
            on_episode(episode)
        index += 1
        if episode.operator_quit:
            session.stopping("operator_quit")
            break
        started = session.between_episodes(reset, allow_next=budget_left())
    return episodes


def _record_episode(
    env: Any,
    obs: Any,
    args: RecordDemosArgs,
    hold: HoldPolicy,
    *,
    index: int,
    num_grippers: int,
    control_period_s: float,
    writer: DatasetWriter,
    named_state: Callable[[], Optional[dict]],
) -> RecordedEpisode:
    from real_world_rl.contracts.outcomes import EpisodeOutcomeAccumulator

    episode = writer.begin_episode()
    episode.add_observation(_policy_obs(obs), named_state(), time_s=time.time(), stack_reset=True)
    gripper_action = gripper_actions_after_reset(num_grippers)
    outcome_accumulator = EpisodeOutcomeAccumulator()
    failure = False
    operator_quit = False
    rerecord = False
    training_rows = 0
    heartbeat = Heartbeat.from_env()
    while True:
        step_start = time.perf_counter()
        action = hold(gripper_action).reshape(1, -1).to(ActorLoop.env_device(obs))
        next_obs, reward, terminated, truncated, info = env.step(action)
        heartbeat.beat()
        intervened = bool(info.get("intervened", False))
        hold.observe_step(intervened)
        operator_quit = operator_quit or operator_quit_requested(info)
        executed = info.get("intervene_action", action)
        if num_grippers:
            gripper_action = executed_gripper_actions(executed, num_grippers)
        events = info.get("teleop_events") or {}
        syncing = bool(events.get(TELEOP_SYNCING_EVENT, False))
        mask = torch.full_like(torch.as_tensor(executed), intervened, dtype=torch.bool)
        episode.add_step(
            executed, info.get("policy_action", action), info.get("human_action_mask", mask),
            info.get("override_mask", mask), intervened, reward, terminated, truncated, syncing, events,
            grasp_penalty=info.get("grasp_penalty"),
        )
        episode.add_observation(_policy_obs(next_obs), named_state(), time_s=time.time())
        outcome_accumulator.observe(info)
        if events.get("rerecord", False):
            rerecord = True
            break
        failure = failure or bool(events.get("failure", False))
        if intervened and (args.keep_syncing_steps or not syncing):
            training_rows += 1
        if bool(terminated) or bool(truncated) or operator_quit:
            break
        obs = next_obs
        remaining = control_period_s - (time.perf_counter() - step_start)
        if remaining > 0:
            time.sleep(remaining)

    if rerecord:
        outcome = outcome_accumulator.finish(discarded=True)
        reason: Optional[str] = "rerecord"
        success: Optional[bool] = None
    else:
        outcome = outcome_accumulator.finish(terminated=terminated, truncated=truncated)
        success = outcome.task_success is True and not failure
        if failure:
            reason = "failure event"
        elif outcome.task_success is not True:
            reason = "no success"
        elif not training_rows:
            reason = "no intervened steps"
        else:
            reason = None
    info = episode.finish(
        status="kept" if reason is None else "discarded",
        discard_reason=reason,
        success=success,
        operator_quit=operator_quit,
        extra={"outcome": outcome.to_dict()},
    )
    return RecordedEpisode(
        index,
        episode.num_steps,
        success=bool(success),
        discarded_reason=reason,
        operator_quit=operator_quit,
        training_rows=training_rows,
        dataset_index=info.index,
    )


def _policy_obs(obs: Any) -> Mapping[str, Any]:
    # A state-only env may return its state as a bare tensor (as load_demo_pkls reads it).
    return obs if isinstance(obs, Mapping) else {_STATE_KEY: obs}


def intervention_wrapper_of(env: Any) -> InterventionWrapper:
    """The intervention wrapper in ``env``'s wrapper chain (raises ``ValueError`` without one)."""
    node, seen = env, set()
    while node is not None and id(node) not in seen:
        seen.add(id(node))
        if isinstance(node, InterventionWrapper):
            return node
        node = vars(node).get("env") if hasattr(node, "__dict__") else None
    raise ValueError(
        "recording needs an env with the intervention wrapper (build_hil_env with teleop enabled)"
    )


# -- output ------------------------------------------------------------------------------


def _env_attr(env: Any, name: str) -> Any:
    getter = getattr(env, "get_wrapper_attr", None)
    try:
        return getter(name) if getter is not None else getattr(env, name)
    except AttributeError:
        return None


def obs_spec_of(env: Any, frame_stack: int = 1) -> dict:
    """The dataset ``obs_spec`` of ``env``: one un-stacked frame per key of its
    ``single_observation_space`` (an image key's leading frame-stack dim is dropped when
    ``frame_stack > 1``)."""
    space = _env_attr(env, "single_observation_space")
    if space is None or not hasattr(space, "spaces"):
        raise ValueError("recording needs the env's single_observation_space (a Dict of state/rgb_*/depth_* keys)")
    spec = {}
    for key, entry in space.spaces.items():
        shape = [int(size) for size in entry.shape]
        if key != _STATE_KEY and frame_stack > 1 and len(shape) == 4:
            shape = shape[1:]
        spec[key] = {"shape": shape, "dtype": np.dtype(entry.dtype).name}
    return spec


def _named_state_owner(env: Any) -> Any:
    # The base RealRobotEnv / BimanualRealRobotEnv (``named_state``), walking ``.env`` down.
    node, seen = env, set()
    while node is not None and id(node) not in seen:
        seen.add(id(node))
        if isinstance(getattr(type(node), "named_state", None), property):
            return node
        node = vars(node).get("env") if hasattr(node, "__dict__") else None
    return None


def _named_state_reader(env: Any) -> Callable[[], Optional[dict]]:
    owner = _named_state_owner(env)
    return (lambda: None) if owner is None else (lambda: owner.named_state)


def _config_snapshot(args: Any) -> dict:
    def redacted(value: Any) -> Any:
        if isinstance(value, dict):
            return {key: None if key in _REDACTED_CONFIG_KEYS else redacted(item) for key, item in value.items()}
        if isinstance(value, list):
            return [redacted(item) for item in value]
        return value

    from real_world_rl.dataset.format import json_normalize

    # default=str: paths and enums become their text; the snapshot is for review, never compared.
    return redacted(json.loads(json.dumps(json_normalize(dataclasses.asdict(args)), default=str)))


def dataset_header_of(env: Any, args: RecordDemosArgs, *, obs_spec: Optional[Mapping[str, Any]] = None) -> dict:
    """The ``DatasetWriter.open`` fields of a recording on ``env`` with ``args``: ``obs_spec``
    (``obs_spec_of`` unless given), the base env's ``named_state_spec`` (empty without one),
    ``demo_contract_of(env)``, the actor's control rate, the env's task description and the
    recorder args as ``config`` (daemon tokens removed)."""
    from real_world_rl.training.runtime_paths import actor_control_hz

    owner = _named_state_owner(env)
    contract = get_env_action_contract(env) or LEGACY_ACTION_CONTRACT
    return dict(
        control_hz=float(actor_control_hz(args)),
        obs_spec=dict(obs_spec) if obs_spec is not None else obs_spec_of(env, args.obs.frame_stack),
        action_dim=action_dim_of(contract),
        action_contract=demo_contract_of(env),
        named_state_spec={} if owner is None else owner.named_state_spec,
        task_description=_env_attr(env, "task_description") or "",
        recorded_frame_stack=int(args.obs.frame_stack),
        config=_config_snapshot(args),
    )


def open_dataset(
    env: Any, args: RecordDemosArgs, root: Optional[str | Path] = None, *,
    obs_spec: Optional[Mapping[str, Any]] = None,
) -> DatasetWriter:
    """Creates the dataset at ``root`` (default ``args.output_dir``) for a recording on ``env``,
    or opens the one there to append to; a header recorded with other settings is refused with
    the differing keys named."""
    root = Path(args.output_dir if root is None else root)
    try:
        return DatasetWriter.open(root, **dataset_header_of(env, args, obs_spec=obs_spec))
    except DatasetMismatchError as error:
        raise ValueError(
            f"--output-dir {root} holds a dataset recorded with a different {', '.join(error.keys)} "
            "than this run's env; use a new --output-dir (or rerun with the settings it was recorded with)"
        ) from error


def _finish_interrupted(writer: DatasetWriter) -> Optional[int]:
    """Stores the episode an interruption left open as discarded (``interrupted``) when it has
    a complete step, else drops it. Returns its step count when stored."""
    episode = writer.active_episode
    if episode is None:
        return None
    if episode.num_steps < 1 or episode.num_observations != episode.num_steps + 1:
        episode.abort()
        return None
    episode.finish(status="discarded", discard_reason="interrupted")
    return episode.num_steps


def parse_record_args(argv: Optional[Sequence[str]] = None) -> RecordDemosArgs:
    """``RecordDemosArgs`` from the command line (``sys.argv[1:]`` by default).

    ``--config PATH`` loads a YAML preset first and flags override it, as in
    ``train_real_world.py``; settings the command line cannot express (``real_robot.joint_limits``)
    go there. The training run's own preset works: its top-level keys that are not recorder
    fields (algorithm hyperparameters) are skipped, nested ones are checked strictly.
    """
    from real_world_rl.training.env_building import resolve_teleop_input

    parsed = parse_args_with_preset(RecordDemosArgs, argv)
    resolve_teleop_input(parsed)
    return parsed


def parse_args_with_preset(args_cls: type, argv: Optional[Sequence[str]] = None) -> Any:
    """``args_cls`` (a dataclass) from the command line (``sys.argv[1:]`` by default), over the
    YAML preset of an optional ``--config PATH``: its top-level keys that are not fields of
    ``args_cls`` are skipped, nested ones are checked strictly, and flags override it."""
    import sys

    import tyro
    from rl_garden.common.effective_config import apply_strict_mapping, load_preset

    from real_world_rl.training.backend_migration import check_backend_cli

    preset_path, cli_args = _pop_config_flag(check_backend_cli(argv))
    defaults = args_cls()
    explicit_semantics = False
    if preset_path is not None:
        own_fields = {item.name for item in dataclasses.fields(args_cls)}
        values = load_preset(preset_path).values
        explicit_semantics = "cartesian_semantics" in values.get("real_robot", {})
        apply_strict_mapping(defaults, {key: value for key, value in values.items() if key in own_fields})
    parsed = tyro.cli(args_cls, default=defaults, args=cli_args)
    if getattr(parsed, "spacemouse_mode", "tracker") == "serl":
        from real_world_rl.training.env_building import resolve_spacemouse_mode

        explicit_semantics = explicit_semantics or any(
            token.split("=", 1)[0].replace("-", "_") == "__real_robot.cartesian_semantics"
            for token in cli_args
        )
        resolve_spacemouse_mode(parsed, explicit_cartesian_semantics=explicit_semantics)
    return parsed


def _pop_config_flag(cli_args: list[str]) -> tuple[Optional[str], list[str]]:
    preset_path: Optional[str] = None
    remaining: list[str] = []
    index = 0
    while index < len(cli_args):
        token = cli_args[index]
        if token == "--config" or token.startswith("--config="):
            if preset_path is not None:
                raise ValueError("only one --config preset may be given")
            if token == "--config":
                if index + 1 >= len(cli_args):
                    raise ValueError("--config needs a YAML path")
                preset_path = cli_args[index + 1]
                index += 2
                continue
            preset_path = token.split("=", 1)[1]
        else:
            remaining.append(token)
        index += 1
    return preset_path, remaining


def _dataset_dir(output_dir: Optional[str]) -> Path:
    if output_dir is None:
        raise ValueError("--output-dir is required (the demo dataset directory; an existing dataset is appended to)")
    path = Path(output_dir)
    if path.suffix == ".pkl":
        raise ValueError(f"--output-dir is a dataset directory, not a .pkl file: got {output_dir}")
    if path.exists() and not path.is_dir():
        raise ValueError(f"--output-dir {output_dir} is a file; pass a dataset directory")
    return path


def _with_episode_step_cap(args: RecordDemosArgs) -> RecordDemosArgs:
    if args.max_episode_steps is None:
        return args
    if args.env_backend == "real_robot":
        real_robot = dataclasses.replace(args.real_robot, max_episode_steps=args.max_episode_steps)
        return dataclasses.replace(args, real_robot=real_robot)
    if args.env_backend == "real_robot_bimanual":
        bimanual = dataclasses.replace(args.real_robot_bimanual, max_episode_steps=args.max_episode_steps)
        return dataclasses.replace(args, real_robot_bimanual=bimanual)
    raise ValueError(f"unsupported env backend: {args.env_backend}")


def _dataset_line(root: Path, writer: DatasetWriter) -> str:
    infos = writer.episodes()
    return f"[record] dataset {root}: {len(infos)} episodes ({sum(info.kept for info in infos)} kept)"


def run_recorder(
    args: RecordDemosArgs, *, hold_action_fn_factory: Optional[HoldPolicyFactory] = None
) -> Optional[Path]:
    """Builds the actor's HIL env and records demos into the dataset at ``args.output_dir``
    until it holds ``args.num_successes`` kept episodes (or Ctrl-C, the operator's quit key or
    ``max_episodes`` stops it). Every episode is on disk once it ends; an episode cut short is
    stored as discarded (``interrupted``) after the env is closed. Returns the dataset directory
    when it holds a kept episode, else ``None``.

    ``hold_action_fn_factory`` builds the hold policy from the built env (default
    ``hold_action_fn_for``)."""
    from real_world_rl.training import env_building

    make_hold = hold_action_fn_for if hold_action_fn_factory is None else hold_action_fn_factory
    _validate_episode_limit(args)
    output_dir = _dataset_dir(args.output_dir)
    if args.record_video_dir is not None:
        raise ValueError(
            "the demo recorder does not take --record-video-dir: the dataset writes each episode's "
            "review videos (video/<rgb key>.mp4 in the episode directory)"
        )
    if args.fwbw:
        raise ValueError("the demo recorder does not support --fwbw")
    env = env_building.build_hil_env(_with_episode_step_cap(args), role="actor")
    try:
        writer = open_dataset(env, args, output_dir)
    except BaseException:
        try:
            env.close()
        except BaseException:
            logging.getLogger(__name__).exception("environment close failed after dataset open error")
        raise
    num_kept = writer.num_kept
    print(f"{_dataset_line(output_dir, writer)}; recording until {args.num_successes} kept", flush=True)

    def report(episode: RecordedEpisode) -> None:
        nonlocal num_kept
        num_kept += int(episode.kept)
        status = "kept" if episode.kept else f"discarded ({episode.discarded_reason})"
        print(
            f"[record] episode={episode.dataset_index} steps={episode.steps} "
            f"transitions={episode.training_rows if episode.kept else 0} {status} "
            f"successes={num_kept}/{args.num_successes}",
            flush=True,
        )
        if episode.operator_quit:
            print("[record] operator quit: stopping after this episode", flush=True)

    try:
        record_demos(env, args, hold_action_fn=make_hold(env), writer=writer, on_episode=report)
    finally:
        active_failure = sys.exc_info()[0] is not None
        cleanup_failures = []

        def cleanup_failure(phase, error):
            cleanup_failures.append(error)
            logging.getLogger(__name__).error("%s failed: %s", phase, error, exc_info=True)

        try:
            Heartbeat.from_env().begin_long_operation()
        except BaseException as error:
            cleanup_failure("finalization heartbeat", error)
        # Release control before any disk work, even if heartbeat setup failed.
        try:
            env.close()
        except BaseException as error:
            cleanup_failure("environment close", error)
        try:
            index = None if writer.active_episode is None else writer.active_episode.index
            steps = _finish_interrupted(writer)
            if steps is not None:
                print(f"[record] episode={index} steps={steps} discarded (interrupted)", flush=True)
            print(_dataset_line(output_dir, writer), flush=True)
        except BaseException as error:
            cleanup_failure("interrupted episode write", error)
        if cleanup_failures and not active_failure:
            raise cleanup_failures[0]
    return output_dir if writer.num_kept else None
