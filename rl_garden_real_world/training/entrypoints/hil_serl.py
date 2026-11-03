"""HIL-SERL run function.

Targets HIL-SERL's ``train_rlpd.py`` capability set (online RLPD + demo
mixing + human intervention + reward classifier), not BC or HG-DAgger's
growing-dataset problem -- see docs/hil-serl-roadmap.md.
"""
from __future__ import annotations

import dataclasses
import logging

import torch

from rl_garden_real_world.training.env_building import (
    build_hil_env,
    compose_hil_env,
    hil_env_request,
    robot_teleop_kwargs,
    tcp_pose_quat_slice,
    warn_if_no_reward_source,
)

_LOGGER = logging.getLogger(__name__)

# Names the tests and downstream code import from this module.
_tcp_pose_quat_slice = tcp_pose_quat_slice
_hil_serl_env_request = hil_env_request
_robot_teleop_kwargs = robot_teleop_kwargs


def _build_env(args, env_request):
    """Thin alias of ``env_building.compose_hil_env`` (teleop enabled)."""
    return compose_hil_env(args, env_request)


def _run_actor(args) -> None:
    """Non-FWBW: one scratch agent, one ``HilSerlActorLoop`` -- same shape as
    ``serl.py``'s ``_run_actor``. FWBW: two scratch agents (forward/backward)
    and two sync clients, driving the existing ``FWBWActorLoop`` (a single
    actor process still only ever drives the one physical robot; it's the
    *policy/sync-client pair* that's doubled, switched per step based on
    ``info["fwbw_direction"]``)."""
    from rl_garden_real_world import FWBWActorLoop
    from rl_garden_real_world.loops.hil_serl import (
        HilSerlActorLoop,
        HilSerlActorSyncClient,
    )

    env = build_hil_env(args, role="actor")
    if args.record_video_dir is not None:
        # Applied here (not inside build_hil_env) since it's actor-only --
        # build_hil_env is shared by actor/learner/eval, and only the actor
        # process ever calls env.step() against the real robot.
        from rl_garden_real_world.envs.wrappers.video_recorder import VideoRecorderWrapper

        env = VideoRecorderWrapper(env, save_dir=args.record_video_dir)

    if not args.fwbw:
        agent = _rebuild_scratch_agent_for_env(args, env)
        sync_client = HilSerlActorSyncClient(f"http://{args.sync_host}:{args.sync_port}")
        loop = HilSerlActorLoop(
            env,
            agent.policy,
            sync_client,
            control_hz=args.control_hz,
            device=agent.device,
            deterministic=args.deterministic_actor,
            seed=args.seed,
        )
        loop.run()
        return

    backward_port = args.sync_port_backward or (args.sync_port + 1)
    agents = {
        "forward": _rebuild_scratch_agent_for_env(args, env),
        "backward": _rebuild_scratch_agent_for_env(args, env),
    }
    sync_clients = {
        "forward": HilSerlActorSyncClient(f"http://{args.sync_host}:{args.sync_port}"),
        "backward": HilSerlActorSyncClient(f"http://{args.sync_host}:{backward_port}"),
    }
    loop = FWBWActorLoop(
        env,
        {k: a.policy for k, a in agents.items()},
        sync_clients,
        control_hz=args.control_hz,
        device=agents["forward"].device,
        deterministic=args.deterministic_actor,
        seed=args.seed,
    )
    loop.run()


def _gripper_kwargs(env) -> dict:
    """``num_grippers`` for ``build_rlpd_hybrid`` from ``env``'s action contract; empty for a
    single gripper, so the builder's default applies."""
    from rl_garden_real_world.action_contract import env_num_grippers

    num_grippers = env_num_grippers(env)
    return {} if num_grippers == 1 else {"num_grippers": num_grippers}


def _rebuild_scratch_agent_for_env(args, env):
    from rl_garden.training.online.entrypoints.rlpd_hybrid import build_rlpd_hybrid

    scratch_args = dataclasses.replace(
        args, buffer_size=8, load_checkpoint=None, offline_dataset=None
    )
    return build_rlpd_hybrid(
        scratch_args, env, None, logger=None, checkpoint_dir=None, **_gripper_kwargs(env)
    )


def _run_learner(args) -> None:
    """Builds a full ``RLPDHybrid`` instance and drives
    ``rl_garden_real_world.loops.hil_serl.HilSerlLearnerLoop``. In FWBW mode, a
    learner process is launched once per direction (``--fwbw_direction
    forward|backward``), each bound to its own sync port -- matching the
    SERL v1 design's "two full actor/learner pairs" for reset-free
    training. Non-FWBW: identical shape to ``serl.py``'s ``_run_learner``."""
    import os
    import time

    from rl_garden.common import Logger, seed_everything
    from rl_garden.common.resolved_config import persist_resolved_config
    from rl_garden.training.online.entrypoints.rlpd_hybrid import build_rlpd_hybrid
    from rl_garden_real_world.action_contract import demo_contract_of
    from rl_garden_real_world.loops.hil_serl import HilSerlLearnerLoop

    seed_everything(args.seed)

    direction_suffix = f"_{args.fwbw_direction}" if args.fwbw else ""
    port = (
        (args.sync_port_backward or (args.sync_port + 1))
        if args.fwbw and args.fwbw_direction == "backward"
        else args.sync_port
    )

    start_time = time.strftime("%Y%m%d_%H%M%S", time.localtime())
    run_name = (
        args.exp_name or f"{args.env_id}__hil_serl_real_world{direction_suffix}__{args.seed}__{int(time.time())}"
    )
    checkpoint_dir = args.checkpoint_dir or os.path.join(args.log_dir, run_name, "checkpoints")

    resolved_config = persist_resolved_config(
        args, training_phase="real_world", algorithm="hil_serl", run_name=run_name, log_dir=args.log_dir
    )
    logger = Logger.create(
        log_type=args.log_type,
        log_dir=args.log_dir,
        run_name=run_name,
        config=resolved_config,
        start_time=start_time,
        log_keywords=args.log_keywords,
        wandb_project=args.wandb_project,
        wandb_entity=args.wandb_entity,
        log_group=args.log_group or args.env_id,
    )

    # The learner only needs the env's spaces and action contract: no teleop device here.
    env = build_hil_env(args, role="learner", enable_teleop=False, run_name=run_name)

    agent = build_rlpd_hybrid(
        args, env, None, logger=logger, checkpoint_dir=checkpoint_dir, **_gripper_kwargs(env)
    )
    agent.init_demo_buffer(args.demo_buffer_size, args.demo_data_ratio)

    loop = HilSerlLearnerLoop(
        agent,
        host=args.sync_host,
        port=port,
        checkpoint_dir=checkpoint_dir,
        buffer_period=args.buffer_period,
        demo_dataset_paths=args.demo_dataset_paths,
        train_freq=args.train_freq,
        publish_freq=args.publish_freq,
        action_contract=demo_contract_of(env),
    )
    try:
        loop.run()
    finally:
        logger.close()


def _run_eval(args) -> None:
    """Loads a checkpoint and runs deterministic evaluation episodes directly
    on the real robot -- no sync client, no learner process, no data
    collection (item 10 of docs/hil-serl-roadmap.md, HIL-SERL's own
    ``--eval_checkpoint_step``/``--eval_n_trajs``, ``train_rlpd.py:71-110``).
    Per-episode success is read off ``terminated``: ``RewardClassifierWrapper``
    sets it True exactly when the classifier judges success
    (``rl_garden/envs/wrappers/reward_classifier.py``); a ``truncated`` episode
    (timeout) counts as a failure. FWBW is out of scope this round -- same
    "single-arm only" boundary already documented elsewhere in the real-world
    RL stack."""
    from rl_garden.training.online.entrypoints.rlpd_hybrid import build_rlpd_hybrid
    from rl_garden_real_world.loops.actor import ActorLoop

    if args.fwbw:
        raise NotImplementedError("hil_serl eval mode does not support --fwbw yet.")
    if args.load_checkpoint is None:
        raise ValueError("hil_serl eval requires --load_checkpoint.")

    env = build_hil_env(args, role="eval")

    eval_args = dataclasses.replace(args, buffer_size=8, offline_dataset=None)
    agent = build_rlpd_hybrid(
        eval_args, env, None, logger=None, checkpoint_dir=None, **_gripper_kwargs(env)
    )
    agent.policy.eval()

    successes = 0
    returns: list[float] = []
    obs, _ = env.reset(seed=args.seed)
    for episode in range(args.eval_n_trajs):
        episode_return = 0.0
        while True:
            with torch.no_grad():
                action = agent.policy.predict(agent._obs_to_policy_device(obs), deterministic=True)
            env_action = action.to(ActorLoop.env_device(obs))
            obs, reward, terminated, truncated, info = env.step(env_action)
            episode_return += float(reward.sum())
            if bool(terminated) or bool(truncated):
                success = bool(terminated)
                successes += int(success)
                returns.append(episode_return)
                print(f"[eval] episode={episode} success={success} return={episode_return:.3f}", flush=True)
                obs, _ = env.reset(seed=args.seed)
                break

    success_rate = successes / args.eval_n_trajs
    mean_return = sum(returns) / len(returns) if returns else 0.0
    print(
        f"[eval] success_rate={success_rate:.3f} mean_return={mean_return:.3f} "
        f"n_trajs={args.eval_n_trajs}",
        flush=True,
    )


def run_hil_serl(args: "HilSerlArgs") -> None:
    if args.role in ("actor", "learner"):
        warn_if_no_reward_source(args, "hil_serl")
    if args.role == "actor":
        _run_actor(args)
    elif args.role == "learner":
        _run_learner(args)
    else:
        _run_eval(args)


# ---------------------------------------------------------------------------
# Args + registration
# ---------------------------------------------------------------------------

from dataclasses import dataclass, field  # noqa: E402
from typing import Literal  # noqa: E402

from rl_garden.encoders.config import EncoderConfig  # noqa: E402
from rl_garden.training.online.entrypoints.rlpd_hybrid import RLPDHybridArgs  # noqa: E402
from rl_garden_real_world.training.args import (  # noqa: E402
    FrankaRealConfig,
    HilEnvArgs,
    RealWorldFrankaArgs,
)
from rl_garden_real_world.training.registry import registry  # noqa: E402


@dataclass
class HilSerlArgs(HilEnvArgs, RealWorldFrankaArgs, RLPDHybridArgs):
    franka_real: FrankaRealConfig = field(default_factory=FrankaRealConfig)

    # Overrides RealWorldFrankaArgs.role's Literal["actor", "learner"] with a
    # third "eval" role (item 10 of docs/hil-serl-roadmap.md) -- kept local to
    # HilSerlArgs rather than the shared RealWorldFrankaArgs so this round's
    # scope stays limited to hil_serl; serl.py is unaffected.
    role: Literal["actor", "learner", "eval"] = "actor"

    # Below: HilSerlArgs-local overrides of RLPDHybridArgs/EncoderConfig
    # defaults, matching what HIL-SERL's own real-robot recipes
    # (serl_launcher/serl_launcher/utils/launcher.py) actually run --
    # scoped here only, not touched on RLPDHybridArgs/EncoderConfig
    # themselves, so sim rlpd_hybrid training and serl.py/SerlArgs are
    # unaffected. See docs/hil-serl-roadmap.md's "Behavioral Divergences"
    # section.
    backup_entropy: bool = False
    encoder: EncoderConfig = field(
        default_factory=lambda: EncoderConfig(
            image_augmentation="random_shift",
            pooling_method="spatial_learned_embeddings",
        )
    )


registry.register("hil_serl", HilSerlArgs, run_hil_serl)
