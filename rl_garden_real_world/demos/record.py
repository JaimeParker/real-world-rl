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
- it counts as a success when a ``success`` event fired or a reward was positive (a reward
  classifier), and no ``failure`` event fired;
- a ``rerecord`` event discards the episode at once and starts the next one;
- only successful episodes with at least one recorded transition are kept.
"""
from __future__ import annotations

import dataclasses
import pickle
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable, Mapping, Optional, Sequence

import numpy as np
import torch
from rl_garden.common.cli_args import ObservationArgs
from rl_garden.common.env_args import EnvBackendArgs, EnvRunArgs

from rl_garden_real_world.action_contract import (
    LEGACY_ACTION_CONTRACT,
    demo_contract_of,
    get_env_action_contract,
    write_contract_sidecar,
)
from rl_garden_real_world.actor_loop import ActorLoop
from rl_garden_real_world.envs.wrappers.intervention import InterventionWrapper
from rl_garden_real_world.teleop.base import TeleopReading, TeleopSource
from rl_garden_real_world.teleop.composite import (
    CompositeTeleopSource,
    ToggleableTeleopSource,
)
from rl_garden_real_world.teleop.leader_arm import TELEOP_SYNCING_EVENT
from rl_garden_real_world.teleop.part import _find_last_observation_owner
from rl_garden_real_world.training.args import (
    FrankaRealConfig,
    HilEnvArgs,
    RealWorldFrankaArgs,
)

__all__ = [
    "HoldActionFn",
    "LeaderFollowerGapError",
    "RecordDemosArgs",
    "RecordedEpisode",
    "check_leader_near_follower",
    "hold_action_fn_for",
    "parse_record_args",
    "record_demos",
    "run_recorder",
    "save_demos",
]

# Both real backends open the gripper on reset, and +1 means open for both.
_GRIPPER_ACTION_AFTER_RESET = 1.0
_DEMO_SUFFIX = ".pkl"

# Gripper action to hold -> the (A,) action that keeps the arm where it is.
HoldActionFn = Callable[[float], torch.Tensor]


@dataclass
class RecordDemosArgs(HilEnvArgs, RealWorldFrankaArgs, EnvRunArgs, ObservationArgs, EnvBackendArgs):
    """Env, teleop and observation flags are the HIL training entrypoints' (``--env-backend``,
    ``--real-robot.*``/``--franka-real.*``, ``--teleop-device``, ``--teleop-event-device``,
    ``--teleop-onset``, ``--obs.*``, ...): pass the ones of the run that will load the demos, so
    the recorded observations and actions match its env."""

    franka_real: FrankaRealConfig = field(default_factory=FrankaRealConfig)
    # The recorder never evaluates; no eval env (franka_real would open a second bridge client).
    eval_freq: int = 0

    # Where the demo pkl goes (``<stem>.contract.json`` is written next to it). Required; an
    # existing file is never overwritten.
    output_path: Optional[str] = None
    # Stop after this many kept (successful) episodes.
    num_successes: int = 20
    # Episode step cap for the env: replaces --real-robot.max-episode-steps or
    # --franka-real.max-episode-steps when set.
    max_episode_steps: Optional[int] = None
    # joint_abs: refuse to start an episode while the leader is further than this from the
    # follower in any joint (rad). Below it, --teleop-onset (ramp or arm) handles the takeover.
    start_tolerance_rad: float = 0.8
    # Also record the leader-arm onset steps rate-limited by the ramp policy (tagged
    # ``teleop_syncing``); by default they are left out of the demos.
    keep_syncing_steps: bool = False


@dataclass(frozen=True)
class RecordedEpisode:
    """One finished episode. ``transitions`` holds its recorded transitions when it was kept and
    is empty otherwise; ``discarded_reason`` says why it was not kept (``None`` when kept)."""

    index: int
    steps: int
    success: bool
    transitions: list = field(default_factory=list)
    discarded_reason: Optional[str] = None

    @property
    def kept(self) -> bool:
        return self.discarded_reason is None


class LeaderFollowerGapError(RuntimeError):
    """The leader arm is too far from the follower to start an episode."""


# -- hold action -------------------------------------------------------------------------


def hold_action_fn_for(env: Any) -> HoldActionFn:
    """The action that keeps ``env``'s robot still, by the env's action contract: a zero arm
    action for ``joint_delta`` and ``cartesian_delta``, the follower's current ``joint_pos``
    normalised onto ``joint_limits`` for ``joint_abs``. The gripper dimension, when the env has
    one, is the given gripper action (the last one executed), since a fixed value would open or
    close the gripper."""
    contract = get_env_action_contract(env) or LEGACY_ACTION_CONTRACT
    arm_dim = int(contract["arm_action_dim"])
    has_gripper = bool(contract["gripper_in_action"])

    if contract["action_mode"] == "joint_abs":
        if contract.get("joint_limits") is None:
            raise ValueError("a joint_abs env needs joint_limits in its action contract to hold the arm")
        low, high = np.asarray(contract["joint_limits"], dtype=np.float64).T

        def arm_action() -> np.ndarray:
            return np.clip(2.0 * (follower_joint_pos(env) - low) / (high - low) - 1.0, -1.0, 1.0)

    else:

        def arm_action() -> np.ndarray:
            return np.zeros(arm_dim)

    def hold(gripper_action: float) -> torch.Tensor:
        parts = [arm_action()]
        if has_gripper:
            parts.append(np.array([gripper_action]))
        return torch.as_tensor(np.concatenate(parts), dtype=torch.float32)

    return hold


def follower_joint_pos(env: Any) -> np.ndarray:
    """``state["joint_pos"]`` of the latest raw observation of the ``RealRobotEnv`` behind ``env``."""
    owner = _find_last_observation_owner(env)
    if owner is None:
        raise ValueError(
            "reading the follower joints needs a real_robot env (RealRobotEnv.last_observation)"
        )
    observation = owner.last_observation
    if observation is None:
        raise RuntimeError("the follower joints are read before the env was reset")
    state = observation["state"]
    if "joint_pos" not in state:
        raise ValueError(
            "the follower joints are read from the observation's joint_pos; add joint_pos to "
            "--real-robot.state-fields"
        )
    return np.asarray(state["joint_pos"], dtype=np.float64)


# -- episode-start sync ------------------------------------------------------------------


def check_leader_near_follower(
    env: Any, source: TeleopSource, contract: Mapping[str, Any], tolerance_rad: float
) -> np.ndarray:
    """Reads the leader once and returns ``|q_leader - q_follower|`` per joint (rad); raises
    ``LeaderFollowerGapError`` when any joint is further apart than ``tolerance_rad``.

    ``joint_abs`` only: the leader position is the source's unramped target decoded from the
    joint limits. The read goes through the motion half of a composite with the toggle released
    (``poll_with_toggle(False)`` for a leader arm), so neither the event device's latched
    buttons nor the onset policy change it.
    """
    if contract["action_mode"] != "joint_abs":
        raise ValueError(f"the leader/follower start check needs joint_abs, got {contract['action_mode']!r}")
    reading = _read_leader(source)
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
    ``args.num_successes`` episodes were kept and returns their transitions, in order.

    Every step sends ``hold_action_fn(last executed gripper action)``; the intervention wrapper
    replaces it with the operator's action while the operator intervenes. A step is recorded
    when ``info["intervened"]``, with the executed action (``info["intervene_action"]``);
    ramp-onset steps (``teleop_syncing``) only with ``args.keep_syncing_steps``. In
    ``joint_abs``, each episode starts with ``check_leader_near_follower``
    (``args.start_tolerance_rad``). ``on_episode`` is called with every finished episode.
    """
    wrapper = _intervention_wrapper_of(env)
    contract = get_env_action_contract(env) or LEGACY_ACTION_CONTRACT
    has_gripper = bool(contract["gripper_in_action"])
    control_period_s = 1.0 / args.control_hz

    kept: list[dict] = []
    num_kept = 0
    index = 0
    while num_kept < args.num_successes:
        obs, _ = env.reset(seed=args.seed)
        if contract["action_mode"] == "joint_abs":
            check_leader_near_follower(env, wrapper.source, contract, args.start_tolerance_rad)
        episode = _record_episode(
            env,
            obs,
            args,
            hold_action_fn,
            index=index,
            has_gripper=has_gripper,
            control_period_s=control_period_s,
        )
        if episode.kept:
            kept.extend(episode.transitions)
            num_kept += 1
        if on_episode is not None:
            on_episode(episode)
        index += 1
    return kept


def _record_episode(
    env: Any,
    obs: Any,
    args: RecordDemosArgs,
    hold_action_fn: HoldActionFn,
    *,
    index: int,
    has_gripper: bool,
    control_period_s: float,
) -> RecordedEpisode:
    transitions: list[dict] = []
    gripper_action = _GRIPPER_ACTION_AFTER_RESET
    success = failure = False
    steps = 0
    while True:
        step_start = time.perf_counter()
        action = hold_action_fn(gripper_action).reshape(1, -1).to(ActorLoop.env_device(obs))
        next_obs, reward, terminated, truncated, info = env.step(action)
        steps += 1
        executed = info.get("intervene_action", action)
        if has_gripper:
            gripper_action = float(torch.as_tensor(executed).reshape(-1)[-1].clamp(-1.0, 1.0))
        events = info.get("teleop_events") or {}
        if events.get("rerecord", False):
            return RecordedEpisode(index, steps, success=False, discarded_reason="rerecord")
        success = success or bool(events.get("success", False)) or bool((torch.as_tensor(reward) > 0).any())
        failure = failure or bool(events.get("failure", False))
        syncing = bool(events.get(TELEOP_SYNCING_EVENT, False))
        if info["intervened"] and (args.keep_syncing_steps or not syncing):
            transitions.append(_transition(obs, next_obs, executed, reward, terminated, info))
        if bool(terminated) or bool(truncated):
            break
        obs = next_obs
        remaining = control_period_s - (time.perf_counter() - step_start)
        if remaining > 0:
            time.sleep(remaining)

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
    )


def _transition(obs: Any, next_obs: Any, action: torch.Tensor, reward, terminated, info: dict) -> dict:
    transition = {
        "obs": obs,
        "next_obs": next_obs,
        "action": action,
        "reward": reward,
        "done": terminated,
    }
    if "grasp_penalty" in info:
        transition["grasp_penalty"] = info["grasp_penalty"]
    return transition


def _intervention_wrapper_of(env: Any) -> InterventionWrapper:
    node, seen = env, set()
    while node is not None and id(node) not in seen:
        seen.add(id(node))
        if isinstance(node, InterventionWrapper):
            return node
        node = vars(node).get("env") if hasattr(node, "__dict__") else None
    raise ValueError(
        "record_demos needs an env with the intervention wrapper (build_hil_env with teleop enabled)"
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
    import sys

    import tyro
    from rl_garden.common.effective_config import apply_strict_mapping, load_preset

    preset_path, cli_args = _pop_config_flag(list(sys.argv[1:] if argv is None else argv))
    defaults = RecordDemosArgs()
    if preset_path is not None:
        recorder_fields = {item.name for item in dataclasses.fields(RecordDemosArgs)}
        values = load_preset(preset_path).values
        apply_strict_mapping(defaults, {key: value for key, value in values.items() if key in recorder_fields})
    return tyro.cli(RecordDemosArgs, default=defaults, args=cli_args)


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
    franka_real = dataclasses.replace(args.franka_real, max_episode_steps=args.max_episode_steps)
    return dataclasses.replace(args, franka_real=franka_real)


def run_recorder(args: RecordDemosArgs) -> Optional[Path]:
    """Builds the actor's HIL env, records demos until ``args.num_successes`` and saves the kept
    episodes to ``args.output_path`` (also when stopped early, e.g. by Ctrl-C). Returns the pkl
    path, or ``None`` when no episode was kept."""
    from rl_garden_real_world.training import env_building

    output_path = _demo_output_path(args.output_path)
    if args.fwbw:
        raise ValueError("the demo recorder does not support --fwbw")
    env = env_building.build_hil_env(_with_episode_step_cap(args), role="actor")
    if args.record_video_dir is not None:
        from rl_garden_real_world.envs.wrappers.video_recorder import (
            VideoRecorderWrapper,
        )

        env = VideoRecorderWrapper(env, save_dir=args.record_video_dir)

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

    try:
        record_demos(env, args, hold_action_fn=hold_action_fn_for(env), on_episode=report)
    finally:
        try:
            if kept:
                save_demos(kept, output_path, demo_contract_of(env))
                print(f"[record] saved {num_kept} episodes ({len(kept)} transitions) to {output_path}", flush=True)
            else:
                print("[record] no episode kept; nothing saved", flush=True)
        finally:
            env.close()
    return output_path if kept else None
