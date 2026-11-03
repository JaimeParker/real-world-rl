"""Reward-classifier data collection on the ``real_robot`` backend (a robot_infra robot, served
locally or by its daemon), the path ``collect_data`` takes for ``--env-backend real_robot``.

The env is the HIL actor's (``build_hil_env(args, role="actor")``): the operator drives the robot
through its teleop source (``--teleop-device leader_arm`` for a GELLO behind the daemon) and labels
frames with the event device's buttons (``--teleop-event-device keyboard``: ``s`` and ``f``).
While nobody intervenes the robot holds still, as in the demo recorder.

Every step stores one frame, the classifier's image keys of the observation the step returned
(``--classifier-image-keys``, or every key but ``state``), the observation a
``RewardClassifierWrapper`` classifies. Frames are labelled failure until a ``success`` event;
from that step on they are labelled success, until a ``failure`` event or the end of the episode
(episode-end button or step cap), either of which switches back to failure. So the operator presses ``s``
when the task is achieved, moves around inside the success region to vary the frames, and
presses ``f`` when leaving it (HIL-SERL's hold-space-while-successful collection, with key
edges). A failure in the same step as a success wins. ``rerecord`` events are ignored.
The env carries no reward source: a training preset's ``--reward-from-teleop-events`` and
``--classifier-checkpoint`` are dropped (both would end the episode at the first success).

Collection stops after ``--successes-needed`` success frames; the frames collected so far are
saved also when it is stopped early (Ctrl-C), in ``collect_data``'s file format.
"""
from __future__ import annotations

import dataclasses
import time
from dataclasses import dataclass, field
from typing import Any, Callable, Literal, Optional, Sequence

from rl_garden.common.cli_args import ObservationArgs
from rl_garden.common.env_args import EnvBackendArgs, EnvRunArgs

from real_world_rl.envs.wrappers.teleop_event_reward import (
    FAILURE_EVENT,
    SUCCESS_EVENT,
)
from real_world_rl.training.args import (
    FrankaRealConfig,
    HilEnvArgs,
    RealWorldFrankaArgs,
)

__all__ = [
    "LabelledFrames",
    "RealRobotCollectArgs",
    "collect_classifier_frames",
    "parse_collect_args",
    "run_real_robot_collector",
]


@dataclass
class RealRobotCollectArgs(HilEnvArgs, RealWorldFrankaArgs, EnvRunArgs, ObservationArgs, EnvBackendArgs):
    """Env, teleop and observation flags are the HIL training entrypoints' (``--real-robot.*``,
    ``--teleop-device``, ``--teleop-event-device``, ``--obs.*``, ``--classifier-image-keys``,
    ...): pass the ones of the run that will use the classifier, so the frames match its
    observations. ``--config PATH`` loads a training preset first."""

    franka_real: FrankaRealConfig = field(default_factory=FrankaRealConfig)
    env_backend: Literal["real_robot"] = "real_robot"
    # The collector never evaluates; no eval env.
    eval_freq: int = 0

    # Where the two pkl files go. Required.
    output_dir: Optional[str] = None
    # Stop after this many success frames.
    successes_needed: int = 200
    # File name prefix of the pkl files.
    exp_name: str = "real_robot"
    # joint_abs: refuse to start an episode while the leader is further than this from the
    # follower in any joint (rad), as the demo recorder does.
    start_tolerance_rad: float = 0.8


@dataclass
class LabelledFrames:
    """``{"obs": {image_key: frame}}`` entries, by label."""

    successes: list = field(default_factory=list)
    failures: list = field(default_factory=list)


def parse_collect_args(argv: Optional[Sequence[str]] = None) -> RealRobotCollectArgs:
    """``RealRobotCollectArgs`` from the command line (``sys.argv[1:]`` by default), over an
    optional ``--config`` preset (see ``demos.record.parse_args_with_preset``)."""
    from real_world_rl.demos.record import parse_args_with_preset

    return parse_args_with_preset(RealRobotCollectArgs, argv)


def collect_classifier_frames(
    env: Any,
    args: RealRobotCollectArgs,
    *,
    hold_action_fn: Callable[[Any], Any],
    frames: Optional[LabelledFrames] = None,
) -> LabelledFrames:
    """Runs episodes on ``env`` (a HIL env with the intervention wrapper) until ``frames`` holds
    ``args.successes_needed`` success frames, labelling them as the module docstring says.

    Every step sends ``hold_action_fn(last executed gripper action)``, which the intervention
    wrapper replaces with the operator's action while the operator intervenes. ``frames`` (new
    when ``None``) is filled in place, so a caller keeps what was collected when this raises.
    """
    from real_world_rl.action_contract import (
        LEGACY_ACTION_CONTRACT,
        get_env_action_contract,
        num_grippers_of,
    )
    from real_world_rl.common.tree import index_tree
    from real_world_rl.demos.record import (
        check_leader_near_follower,
        executed_gripper_actions,
        gripper_actions_after_reset,
        intervention_wrapper_of,
    )
    from real_world_rl.loops.actor import ActorLoop
    from real_world_rl.training.env_building import classifier_image_keys_for

    frames = LabelledFrames() if frames is None else frames
    wrapper = intervention_wrapper_of(env)
    contract = get_env_action_contract(env) or LEGACY_ACTION_CONTRACT
    num_grippers = num_grippers_of(contract)
    image_keys = classifier_image_keys_for(args, env)
    control_period_s = 1.0 / args.control_hz

    episode = 0
    while len(frames.successes) < args.successes_needed:
        obs, _ = env.reset(seed=args.seed)
        if contract["action_mode"] == "joint_abs":
            check_leader_near_follower(env, wrapper.source, contract, args.start_tolerance_rad)
        gripper_action = gripper_actions_after_reset(num_grippers)
        labelling_success = False
        steps = 0
        while len(frames.successes) < args.successes_needed:
            step_start = time.perf_counter()
            action = hold_action_fn(gripper_action).reshape(1, -1).to(ActorLoop.env_device(obs))
            obs, _reward, terminated, truncated, info = env.step(action)
            steps += 1
            if num_grippers:
                gripper_action = executed_gripper_actions(info.get("intervene_action", action), num_grippers)
            events = info.get("teleop_events") or {}
            if events.get(FAILURE_EVENT, False):
                labelling_success = False
            elif events.get(SUCCESS_EVENT, False):
                labelling_success = True
            entry = {"obs": {key: index_tree(obs[key], 0) for key in image_keys}}
            (frames.successes if labelling_success else frames.failures).append(entry)
            if bool(terminated) or bool(truncated):
                break
            remaining = control_period_s - (time.perf_counter() - step_start)
            if remaining > 0:
                time.sleep(remaining)
        print(
            f"[collect] episode={episode} steps={steps} successes={len(frames.successes)}/"
            f"{args.successes_needed} failures={len(frames.failures)}",
            flush=True,
        )
        episode += 1
    return frames


def run_real_robot_collector(args: RealRobotCollectArgs) -> Optional[tuple[str, str]]:
    """Builds the actor's HIL env, collects labelled frames until ``args.successes_needed``
    success frames and saves them under ``args.output_dir`` (also when stopped early). Returns
    the ``(success, failure)`` pkl paths, or ``None`` when nothing was collected."""
    from real_world_rl.demos.record import hold_action_fn_for
    from real_world_rl.reward.collect_data import save_classifier_data
    from real_world_rl.training import env_building

    env = env_building.build_hil_env(_collection_env_args(args), role="actor")
    frames = LabelledFrames()
    paths = None
    try:
        collect_classifier_frames(env, args, hold_action_fn=hold_action_fn_for(env), frames=frames)
    finally:
        try:
            if frames.successes or frames.failures:
                paths = save_classifier_data(
                    args.output_dir, args.exp_name, len(frames.successes), frames.successes, frames.failures
                )
            else:
                print("[collect] no frame collected; nothing saved", flush=True)
        finally:
            env.close()
    return paths


def _collection_env_args(args: RealRobotCollectArgs) -> RealRobotCollectArgs:
    """``args`` checked, without the reward sources a training preset may carry: both end the
    episode on the first success, which would cut the success frames short."""
    if args.output_dir is None:
        raise ValueError("--output-dir is required")
    if args.fwbw:
        raise ValueError("the classifier data collector does not support --fwbw")
    ignored = []
    if args.reward_from_teleop_events:
        ignored.append("--reward-from-teleop-events")
    if args.classifier_checkpoint is not None:
        ignored.append("--classifier-checkpoint")
    if ignored:
        print(f"[collect] ignoring {' and '.join(ignored)}: the collector labels the frames itself", flush=True)
    return dataclasses.replace(args, reward_from_teleop_events=False, classifier_checkpoint=None)
