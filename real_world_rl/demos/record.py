"""Teleoperated demo recording on the real robot.

The env is the HIL actor's env (``build_hil_env(args, role="actor")``): the operator drives the
robot through its teleop source and the intervention wrapper executes the operator's actions.
While nobody intervenes the recorder holds the robot still. Only intervened steps are recorded,
in the pkl transition-dict format ``load_demo_pkls`` reads (``obs``, ``next_obs``, ``action``,
``reward``, ``done``, plus ``grasp_penalty`` when ``GraspPenaltyWrapper`` is in the stack, as
the hil_serl learner's demo snapshots hold them), with the env's action-contract sidecar next to
the pkl. See ``docs/recording-demos.md``.

Episode outcome, read from the intervention wrapper's info keys so any event path works:

- the episode ends on ``terminated`` or ``truncated`` (an operator episode-end press truncates);
- it counts as a success when an explicit success outcome or ``success`` event is reported (a reward
  classifier), and no ``failure`` event fired;
- a ``rerecord`` event discards the episode at once; a new attempt starts if its budget remains;
- only successful episodes with at least one recorded transition are kept;
- an ``operator_quit`` event (the ``q`` key) ends the episode and stops recording after it; the
  kept episodes are saved as after Ctrl-C.

The hold policy (``hold_action_fn_for``, injectable through ``run_recorder``'s
``hold_action_fn_factory``) is reset after every env reset and observes every step; on a
``cartesian_delta`` arm it holds a fixed pose (``real_world_rl.teleop.core.hold.CartesianHold``).

The stored ``reward``/``done`` are the env's. Without a reward classifier, pass
``--reward-from-teleop-events`` (``TeleopEventRewardWrapper`` in the ``build_hil_env`` chain): a
``success`` event then ends the episode at once and its transition is stored with
``reward=--teleop-success-reward`` and ``done=True``; otherwise every transition carries reward 0.
"""
from __future__ import annotations

import dataclasses
import logging
import sys
import pickle
import uuid
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable, Mapping, Optional, Sequence

import numpy as np
import torch

from real_world_rl.supervise import Heartbeat
from rl_garden.common.cli_args import ObservationArgs
from rl_garden.common.env_args import EnvBackendArgs, EnvRunArgs

from real_world_rl.action_contract import (
    LEGACY_ACTION_CONTRACT,
    arm_block_slices,
    demo_contract_of,
    get_env_action_contract,
    num_arms_of,
    num_grippers_of,
    write_contract_sidecar,
    validate_transitions_against_contract,
    experiment_hash,
)
from real_world_rl.envs.wrappers.intervention import InterventionWrapper
from real_world_rl.loops.actor import ActorLoop
from real_world_rl.teleop.core.hold import (
    HoldPolicy,
    _arm_envs_of,
    as_hold_policy,
    follower_joint_pos,
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
    FrankaRealConfig,
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
    "executed_gripper_actions",
    "gripper_actions_after_reset",
    "hold_action_fn_for",
    "intervention_wrapper_of",
    "parse_args_with_preset",
    "parse_record_args",
    "record_demos",
    "run_recorder",
    "save_demos",
]

# Both real backends open the gripper on reset, and +1 means open for both.
_GRIPPER_ACTION_AFTER_RESET = 1.0
_DEMO_SUFFIX = ".pkl"

# Gripper action(s) to hold, one per arm (a number for one arm) -> the (A,) action that keeps the
# arms where they are. A ``HoldPolicy`` also receives the recorder's reset/step hooks.
HoldActionFn = Callable[[Any], torch.Tensor]
# The built env -> its hold policy (``run_recorder``'s injection point).
HoldPolicyFactory = Callable[[Any], HoldActionFn]


@dataclass
class RecordDemosArgs(HilEnvArgs, RealWorldFrankaArgs, EnvRunArgs, ObservationArgs, EnvBackendArgs):
    """Env, teleop and observation flags are the HIL training entrypoints' (``--env-backend``,
    ``--real-robot.*``/``--franka-real.*``, ``--teleop-device``, ``--spacemouse-mode``, ``--teleop-event-device``,
    ``--teleop-onset``, ``--obs.*``, ...): pass the ones of the run that will load the demos, so
    the recorded observations and actions match its env."""

    franka_real: FrankaRealConfig = field(default_factory=FrankaRealConfig)
    # The recorder never evaluates; no eval env (franka_real would open a second bridge client).
    eval_freq: int = 0
    # On by default here (off for training): the operator marks every successful demo with the
    # success key anyway, and a success stored as reward 0 / done False cannot be relabelled later.
    # The success key therefore also ends the episode. ``--no-reward-from-teleop-events`` restores
    # the env's own reward.
    reward_from_teleop_events: bool = True

    # Where the demo pkl goes (``<stem>.contract.json`` is written next to it). Required; an
    # existing file is never overwritten.
    output_path: Optional[str] = None
    # Additionally export the full SQLite archive as a portable PT file.
    export_trajectory_pt: bool = False
    # Stop after this many kept (successful) episodes.
    num_successes: int = 20
    # Stop after this many completed attempts, including failures, rerecords and timeouts.
    # None keeps recording until num_successes; 1 enables manually restarted commissioning runs.
    max_episodes: Optional[int] = None
    # Episode step cap for the env: replaces --real-robot.max-episode-steps,
    # --real-robot-bimanual.max-episode-steps or --franka-real.max-episode-steps when set.
    max_episode_steps: Optional[int] = None
    # joint_abs: refuse to start an episode while the leader is further than this from the
    # follower in any joint (rad). Below it, --teleop-onset (ramp or arm) handles the takeover.
    start_tolerance_rad: float = 0.8
    # Also record the leader-arm onset catch-up steps of the ramp policy (tagged
    # ``teleop_syncing``) as ordinary supervised demo rows; by default they are left out.
    keep_syncing_steps: bool = False


@dataclass(frozen=True)
class RecordedEpisode:
    """One finished episode. ``transitions`` holds its recorded transitions when it was kept and
    is empty otherwise; ``discarded_reason`` says why it was not kept (``None`` when kept);
    ``operator_quit`` whether the operator asked to stop recording during it."""

    index: int
    steps: int
    success: bool
    transitions: list = field(default_factory=list)
    discarded_reason: Optional[str] = None
    operator_quit: bool = False

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
    from real_world_rl.teleop.core.session import validate_neutral_options
    if args.episode_end_behavior not in ("auto", "hold"):
        raise ValueError("episode_end_behavior must be auto or hold")
    validate_neutral_options(args.neutral_threshold, args.neutral_stable_s)
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
    if contract["action_mode"] != "joint_abs":
        raise ValueError(f"the leader/follower start check needs joint_abs, got {contract['action_mode']!r}")
    reading = _read_leader(source)
    if num_arms_of(contract) != 1:
        return _check_leaders_near_followers(env, reading, contract, tolerance_rad)
    low, high = np.asarray(contract["joint_limits"], dtype=np.float64).T
    arm = np.asarray(reading.action[: len(low)], dtype=np.float64)
    leader = low + (arm + 1.0) / 2.0 * (high - low)
    error = np.abs(leader - follower_joint_pos(env))
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
    arm_names = list(contract["arm_names"])
    action = np.asarray(reading.action, dtype=np.float64)
    per_arm_limits = np.asarray(contract["joint_limits"], dtype=np.float64)
    errors = []
    for block, arm_env, limits in zip(arm_block_slices(contract), _arm_envs_of(env, contract), per_arm_limits):
        low, high = limits.T
        leader = low + (action[block] + 1.0) / 2.0 * (high - low)
        errors.append(np.abs(leader - follower_joint_pos(arm_env)))
    errors = np.stack(errors)
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
    on_episode: Optional[Callable[[RecordedEpisode], None]] = None,
) -> list[dict]:
    """Runs episodes on ``env`` (a HIL env with the intervention wrapper) until
    ``args.num_successes`` episodes were kept or ``args.max_episodes`` attempts ended, and
    returns their kept transitions, in order. Failed/rerecorded/truncated attempts count too.

    Every step sends ``hold_action_fn(last executed gripper action)``; the intervention wrapper
    replaces it with the operator's action while the operator intervenes. A ``HoldPolicy``
    (``hold_action_fn_for``; a plain callable gets no-op hooks) is reset after every env reset
    and observes whether every step was intervened. A step is recorded when
    ``info["intervened"]``, with the executed action (``info["intervene_action"]``); ramp-onset
    steps (``teleop_syncing``) only with ``args.keep_syncing_steps``. In ``joint_abs``, each
    episode starts with ``check_leader_near_follower`` (``args.start_tolerance_rad``).
    ``on_episode`` is called with every finished episode; an episode with the operator's quit
    key is the last one.
    """
    _validate_episode_limit(args)
    hold = as_hold_policy(hold_action_fn)
    wrapper = intervention_wrapper_of(env)
    if args.episode_end_behavior == "hold":
        from real_world_rl.teleop.core.session import session_backend
        session_backend(env)  # reject unsupported backends before reset can move the arm
        if (not isinstance(wrapper.source, CompositeTeleopSource)
                or not callable(getattr(wrapper.source.motion, "session_input", None))):
            raise ValueError("session hold requires SpaceMouse plus an event device")
    contract = get_env_action_contract(env) or LEGACY_ACTION_CONTRACT
    num_grippers = num_grippers_of(contract)
    control_period_s = 1.0 / args.control_hz

    from real_world_rl.trajectory import DurableArchive
    if getattr(env, "_record_archive", None) is None:
        import tempfile
        archive_path = Path(args.output_path).with_suffix(".trajectory.sqlite3") if args.output_path else Path(tempfile.mkdtemp(prefix="rl-record-")) / "trajectory.sqlite3"
        env._record_archive = DurableArchive(archive_path, run_id=args.run_id, actor_id=args.actor_id, experiment_hash=experiment_hash(env))
    kept: list[dict] = []
    num_kept = 0
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
            if wait_before_episode(env, wrapper.source, control_hz=args.control_hz,
                                   neutral_threshold=args.neutral_threshold,
                                   neutral_stable_s=args.neutral_stable_s) == "quit":
                break
            from real_world_rl.teleop.core.session import session_observe
            obs = session_observe(env)
        episode = _record_episode(
            env,
            obs,
            args,
            hold,
            index=index,
            num_grippers=num_grippers,
            control_period_s=control_period_s,
        )
        if episode.kept:
            kept.extend(episode.transitions)
            num_kept += 1
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
                                  control_hz=args.control_hz, neutral_threshold=args.neutral_threshold,
                                  neutral_stable_s=args.neutral_stable_s) == "quit":
                break
    return kept


def _record_episode(
    env: Any,
    obs: Any,
    args: RecordDemosArgs,
    hold: HoldPolicy,
    *,
    index: int,
    num_grippers: int,
    control_period_s: float,
) -> RecordedEpisode:
    from real_world_rl.trajectory import begin_episode, transition_record, end_episode
    archive = getattr(env, "_record_archive", None)
    if archive is None:
        archive = []
        env._record_archive = archive
    episode_id = uuid.uuid4().hex
    from real_world_rl.action_contract import experiment_hash
    archive.append(begin_episode(episode_id, run_id=args.run_id or "unknown", actor_id=args.actor_id, experiment_hash=experiment_hash(env), action_contract=demo_contract_of(env)))
    transitions: list[dict] = []
    gripper_action = gripper_actions_after_reset(num_grippers)
    from real_world_rl.metrics import EpisodeOutcomeAccumulator

    outcome_accumulator = EpisodeOutcomeAccumulator()
    failure = False
    operator_quit = False
    steps = 0
    heartbeat = Heartbeat.from_env()
    while True:
        step_start = time.perf_counter()
        action = hold(gripper_action).reshape(1, -1).to(ActorLoop.env_device(obs))
        next_obs, reward, terminated, truncated, info = env.step(action)
        heartbeat.beat()
        hold.observe_step(bool(info.get("intervened", False)))
        operator_quit = operator_quit or operator_quit_requested(info)
        steps += 1
        executed = info.get("intervene_action", action)
        if num_grippers:
            gripper_action = executed_gripper_actions(executed, num_grippers)
        events = info.get("teleop_events") or {}
        record = transition_record(_transition(obs, next_obs, executed, reward, terminated, info), episode_id=episode_id, step_index=steps - 1, timestamp_s=time.time(), experiment_hash=experiment_hash(env), run_id=args.run_id or "unknown", actor_id=args.actor_id, terminated=terminated, truncated=truncated, policy_action=info.get("policy_action", action), human_action_mask=info.get("human_action_mask", torch.full_like(executed, bool(info.get("intervened", False)), dtype=torch.bool)), override_mask=info.get("override_mask", torch.full_like(executed, bool(info.get("intervened", False)), dtype=torch.bool)), teleop_events=dict(events), syncing=bool(events.get(TELEOP_SYNCING_EVENT, False)), task_reward=info.get("task_reward", reward), manual_reward=info.get("manual_reward", 0.0), intervention_reward=info.get("intervention_reward", 0.0), outcome_source=info.get("task_success_source", "unknown"), reward_components=info.get("reward_components", {"task": info.get("task_reward", reward), "manual": info.get("manual_reward", 0.0), "intervention": info.get("intervention_reward", 0.0)}), task_success=info.get("task_success"))
        archive.append(record)
        record = archive[-1]
        outcome_accumulator.observe(info)
        if events.get("rerecord", False):
            archive.append(end_episode(episode_id, step_count=steps, outcome=outcome_accumulator.finish(discarded=True)))
            return RecordedEpisode(index, steps, success=False, discarded_reason="rerecord",
                                   operator_quit=operator_quit)
        failure = failure or bool(events.get("failure", False))
        syncing = bool(events.get(TELEOP_SYNCING_EVENT, False))
        if info["intervened"] and (args.keep_syncing_steps or not syncing):
            # A kept syncing step is wanted as supervision; only the archive keeps the flag.
            transitions.append(dict(record, syncing=False))
        if bool(terminated) or bool(truncated) or operator_quit:
            break
        obs = next_obs
        remaining = control_period_s - (time.perf_counter() - step_start)
        if remaining > 0:
            time.sleep(remaining)

    for row_index, row in enumerate(transitions):
        following = transitions[row_index + 1] if row_index + 1 < len(transitions) else None
        if following is None or following["step_index"] != row["step_index"] + 1:
            row["training_segment_end"] = True
    outcome = outcome_accumulator.finish(terminated=terminated, truncated=truncated)
    archive.append(end_episode(episode_id, step_count=steps, outcome=outcome))
    success = outcome.task_success is True
    if failure:
        reason: Optional[str] = "failure event"
    elif not success:
        reason = "no success"
    elif not transitions:
        reason = "no intervened steps"
    else:
        reason = None
    return RecordedEpisode(
        index,
        steps,
        success=success and not failure,
        transitions=transitions if reason is None else [],
        discarded_reason=reason,
        operator_quit=operator_quit,
    )


def _transition(obs: Any, next_obs: Any, action: torch.Tensor, reward, terminated, info: dict) -> dict:
    transition = {
        "obs": obs,
        "next_obs": next_obs,
        "action": action,
        "reward": reward,
        "done": terminated,
    }
    for key in ("grasp_penalty", "human_action_mask", "override_mask", "policy_action", "intervened"):
        if key in info:
            transition[key] = info[key]
    return transition


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


def save_demos(
    transitions: Sequence[dict], output_path: str | Path, contract: Optional[Mapping[str, Any]]
) -> Path:
    """Pickles ``transitions`` (a list of transition dicts) to ``output_path`` and writes the
    contract sidecar next to it (none for a contract-less backend)."""
    path = Path(output_path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "wb") as stream:
        pickle.dump(list(transitions), stream)
    if contract is not None:
        write_contract_sidecar(path, contract)
    return path


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

    preset_path, cli_args = _pop_config_flag(list(sys.argv[1:] if argv is None else argv))
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


def _demo_output_path(output_path: Optional[str]) -> Path:
    if output_path is None:
        raise ValueError("--output-path is required (a .pkl file)")
    path = Path(output_path)
    if path.suffix != _DEMO_SUFFIX:
        raise ValueError(f"--output-path must be a {_DEMO_SUFFIX} file, got {output_path}")
    if path.exists():
        raise ValueError(f"--output-path {output_path} exists; pick a new file")
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
    franka_real = dataclasses.replace(args.franka_real, max_episode_steps=args.max_episode_steps)
    return dataclasses.replace(args, franka_real=franka_real)


def run_recorder(
    args: RecordDemosArgs, *, hold_action_fn_factory: Optional[HoldPolicyFactory] = None
) -> Optional[Path]:
    """Builds the actor's HIL env, records demos until ``args.num_successes`` and saves the kept
    episodes to ``args.output_path`` (also when stopped early, e.g. by Ctrl-C, the operator's
    quit key or max_episodes). Returns the pkl path, or ``None`` when no episode was kept.

    ``hold_action_fn_factory`` builds the hold policy from the built env (default
    ``hold_action_fn_for``)."""
    from real_world_rl.training import env_building

    make_hold = hold_action_fn_for if hold_action_fn_factory is None else hold_action_fn_factory
    _validate_episode_limit(args)
    output_path = _demo_output_path(args.output_path)
    if args.fwbw:
        raise ValueError("the demo recorder does not support --fwbw")
    env = env_building.build_hil_env(_with_episode_step_cap(args), role="actor")
    if args.record_video_dir is not None:
        from real_world_rl.envs.wrappers.video_recorder import (
            VideoRecorderWrapper,
        )

        env = VideoRecorderWrapper(env, save_dir=args.record_video_dir)

    from real_world_rl.trajectory import DurableArchive
    try:
        env._record_archive = DurableArchive(output_path.with_suffix(".trajectory.sqlite3"), run_id=args.run_id, actor_id=args.actor_id, experiment_hash=experiment_hash(env))
        for record in env._record_archive:
            if record["record_type"] == "episode_start":
                validate_transitions_against_contract([], demo_contract_of(env), source=str(output_path), saved=record.get("action_contract"))
    except BaseException:
        try:
            env.close()
        except BaseException:
            logging.getLogger(__name__).exception("environment close failed after archive initialization error")
        archive = getattr(env, "_record_archive", None)
        if archive is not None:
            try:
                archive.close()
            except BaseException:
                logging.getLogger(__name__).exception("archive close also failed")
        raise
    kept: list[dict] = []
    num_kept = 0

    def report(episode: RecordedEpisode) -> None:
        nonlocal num_kept
        kept.extend(episode.transitions)
        num_kept += int(episode.kept)
        status = "kept" if episode.kept else f"discarded ({episode.discarded_reason})"
        print(
            f"[record] episode={episode.index} steps={episode.steps} "
            f"transitions={len(episode.transitions)} {status} "
            f"successes={num_kept}/{args.num_successes}",
            flush=True,
        )
        if episode.operator_quit:
            print("[record] operator quit: stopping after this episode", flush=True)

    try:
        record_demos(env, args, hold_action_fn=make_hold(env), on_episode=report)
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
        # Release control before exports, even if heartbeat setup failed.
        try:
            env.close()
        except BaseException as error:
            cleanup_failure("environment close", error)
        try:
            archive = getattr(env, "_record_archive", [])
            if archive:
                from real_world_rl.trajectory import end_episode
                if archive[-1]["record_type"] != "episode_end":
                    episode_id = archive[-1]["episode_id"]
                    from real_world_rl.metrics import EpisodeOutcomeAccumulator
                    interrupted_outcome = EpisodeOutcomeAccumulator()
                    for record in archive:
                        if record["record_type"] == "transition" and record["episode_id"] == episode_id:
                            interrupted_outcome.observe(record)
                    archive.append(end_episode(episode_id, step_count=interrupted_outcome.steps,
                                               outcome=interrupted_outcome.finish(interrupted=True), interrupted=True))
                from real_world_rl.metrics import EpisodeMetrics, EpisodeOutcome
                metrics = EpisodeMetrics()
                for record in archive:
                    if record["record_type"] == "episode_end":
                        outcome = record.get("outcome")
                        if outcome is not None:
                            metrics.add(EpisodeOutcome(**outcome))
                        elif record.get("interrupted"):
                            metrics.add(EpisodeOutcome(None, False, record.get("step_count", 0), 0,
                                                       completed=False, interrupted=True))
                print(f"[record] outcomes={metrics.summary()}", flush=True)
                output_path.parent.mkdir(parents=True, exist_ok=True)
                if args.export_trajectory_pt:
                    torch.save({"schema_version": 1, "records": list(archive), "action_contract": demo_contract_of(env)}, output_path.with_suffix(".trajectory.pt"))
            if kept:
                save_demos(kept, output_path, demo_contract_of(env))
                print(f"[record] saved {num_kept} episodes ({len(kept)} transitions) to {output_path}", flush=True)
            else:
                print("[record] no demo pkl exported; trajectory archive retained", flush=True)
        except BaseException as error:
            cleanup_failure("archive finalization/export", error)
        finally:
            try:
                env._record_archive.close()
            except BaseException as error:
                cleanup_failure("archive close", error)
        if cleanup_failures and not active_failure:
            raise cleanup_failures[0]
    return output_path if kept else None
