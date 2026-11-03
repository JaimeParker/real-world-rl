"""HIL-SERL run function.

Targets HIL-SERL's ``train_rlpd.py`` capability set (online RLPD + demo
mixing + human intervention + reward classifier), not BC or HG-DAgger's
growing-dataset problem -- see docs/hil-serl-roadmap.md.
"""
from __future__ import annotations

import dataclasses

from real_world_rl.training.runtime_paths import actor_sync_kwargs, learner_experiment_kwargs, actor_control_hz
import logging


from real_world_rl.training.env_building import (
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
    from real_world_rl import FWBWActorLoop
    from real_world_rl.loops.hil_serl import (
        HilSerlActorLoop,
        HilSerlActorSyncClient,
    )

    env = build_hil_env(args, role="actor")
    loop_owns_env = False
    try:
        if args.record_video_dir is not None:
            # Applied here (not inside build_hil_env) since it's actor-only --
            # build_hil_env is shared by actor/learner/eval, and only the actor
            # process ever calls env.step() against the real robot.
            from real_world_rl.envs.wrappers.video_recorder import VideoRecorderWrapper

            env = VideoRecorderWrapper(env, save_dir=args.record_video_dir)

        if not args.fwbw:
            agent = _rebuild_scratch_agent_for_env(args, env)
            sync_client = HilSerlActorSyncClient(f"http://{args.sync_host}:{args.sync_port}", **actor_sync_kwargs(args))
            loop = HilSerlActorLoop(
                env,
                agent.policy,
                sync_client,
                control_hz=actor_control_hz(args),
                device=agent.device,
                deterministic=args.deterministic_actor,
                seed=args.seed,
            )
            loop_owns_env = True
            loop.run()
            return

        backward_port = args.sync_port_backward or (args.sync_port + 1)
        agents = {
            "forward": _rebuild_scratch_agent_for_env(args, env),
            "backward": _rebuild_scratch_agent_for_env(args, env),
        }
        sync_clients = {}
        sync_clients["forward"] = HilSerlActorSyncClient(
            f"http://{args.sync_host}:{args.sync_port}", **actor_sync_kwargs(args, direction="forward")
        )
        sync_clients["backward"] = HilSerlActorSyncClient(
            f"http://{args.sync_host}:{backward_port}", **actor_sync_kwargs(args, direction="backward")
        )
        loop = FWBWActorLoop(
            env,
            {k: a.policy for k, a in agents.items()},
            sync_clients,
            control_hz=actor_control_hz(args),
            device=agents["forward"].device,
            deterministic=args.deterministic_actor,
            seed=args.seed,
        )
        loop_owns_env = True
        loop.run()
    finally:
        if not loop_owns_env:
            from real_world_rl.training.runtime_paths import cleanup_construction
            clients = list(locals().get("sync_clients", {}).values())
            cleanup_construction(env, locals().get("sync_client"), *clients)


def build_hil_agent(args, env, eval_env, logger=None, checkpoint_dir=None, **kwargs):
    from real_world_rl.checkpoint_contract import build_agent_with_action_contract
    from real_world_rl.action_contract import env_num_grippers
    if env_num_grippers(env) == 0:
        from rl_garden.training.online.entrypoints.rlpd import build_rlpd
        from real_world_rl.algorithms.hil_continuous import ContinuousHilRLPD
        kwargs.pop("num_grippers", None)
        return build_agent_with_action_contract(build_rlpd, args, env, eval_env, logger, checkpoint_dir,
                          algorithm_cls=ContinuousHilRLPD, **kwargs)
    from rl_garden.training.online.entrypoints.rlpd_hybrid import build_rlpd_hybrid
    return build_agent_with_action_contract(build_rlpd_hybrid, args, env, eval_env, logger, checkpoint_dir, **kwargs)


def _gripper_kwargs(env) -> dict:
    """``num_grippers`` for ``build_rlpd_hybrid`` from ``env``'s action contract; empty for a
    single gripper, so the builder's default applies."""
    from real_world_rl.action_contract import env_num_grippers

    num_grippers = env_num_grippers(env)
    return {} if num_grippers == 1 else {"num_grippers": num_grippers}


def _rebuild_scratch_agent_for_env(args, env):

    scratch_args = dataclasses.replace(
        args, buffer_size=8, load_checkpoint=None, offline_dataset=None
    )
    return build_hil_agent(
        scratch_args, env, None, logger=None, checkpoint_dir=None, **_gripper_kwargs(env)
    )


def _run_learner(args) -> None:
    """Builds a full ``RLPDHybrid`` instance and drives
    ``real_world_rl.loops.hil_serl.HilSerlLearnerLoop``. In FWBW mode, a
    learner process is launched once per direction (``--fwbw_direction
    forward|backward``), each bound to its own sync port -- matching the
    SERL v1 design's "two full actor/learner pairs" for reset-free
    training. Non-FWBW: identical shape to ``serl.py``'s ``_run_learner``."""
    from real_world_rl.training.runtime_paths import resolve_control_frequency
    resolve_control_frequency(args)
    import os
    import time

    from rl_garden.common import Logger, seed_everything
    from rl_garden.common.resolved_config import persist_resolved_config
    from real_world_rl.action_contract import demo_contract_of
    from real_world_rl.loops.hil_serl import HilSerlLearnerLoop

    seed_everything(args.seed)

    direction_suffix = f"_{args.fwbw_direction}" if args.fwbw else ""
    port = (
        (args.sync_port_backward or (args.sync_port + 1))
        if args.fwbw and args.fwbw_direction == "backward"
        else args.sync_port
    )

    start_time = time.strftime("%Y%m%d_%H%M%S", time.localtime())
    from real_world_rl.training.runtime_paths import hil_learner_paths
    run_name, checkpoint_dir = hil_learner_paths(
        args, f"{args.env_id}__hil_serl_real_world{direction_suffix}__{args.seed}__{int(time.time())}"
    )

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
    loop_owns_env = False
    try:

        agent = build_hil_agent(
            dataclasses.replace(args, load_checkpoint=None), env, None, logger=logger, checkpoint_dir=checkpoint_dir, **_gripper_kwargs(env)
        )
        agent.init_demo_buffer(args.demo_buffer_size, args.demo_data_ratio)

        loop = HilSerlLearnerLoop(
            agent,
            **learner_experiment_kwargs(env, checkpoint_dir),
            admission_capacity=args.admission_capacity,
            ingest_batch_size=args.ingest_batch_size,
            resume_bundle_path=args.load_checkpoint,
            resume_replay=args.load_replay_buffer,
            journal_path=os.path.join(checkpoint_dir, "journal.sqlite3"),
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
            loop_owns_env = True
            loop.run()
        finally:
            logger.close()
    finally:
        if not loop_owns_env:
            from real_world_rl.training.runtime_paths import cleanup_construction
            clients = list(locals().get("sync_clients", {}).values())
            cleanup_construction(env, locals().get("sync_client"), *clients)
            logger.close()


def _run_eval(args) -> None:
    """Evaluate explicit task success and autonomous success with human control available."""

    if args.fwbw:
        raise NotImplementedError("hil_serl eval mode does not support --fwbw yet.")
    if args.load_checkpoint is None:
        raise ValueError("hil_serl eval requires --load_checkpoint.")

    env = build_hil_env(args, role="eval")
    try:
        from real_world_rl.training.evaluation import resolve_eval_checkpoint
        checkpoint = resolve_eval_checkpoint(args, env)

        eval_args = dataclasses.replace(args, buffer_size=8, offline_dataset=None, load_checkpoint=checkpoint, load_replay_buffer=False)
        agent = build_hil_agent(
            eval_args, env, None, logger=None, checkpoint_dir=None, **_gripper_kwargs(env)
        )
        agent.policy.eval()

    except BaseException:
        from real_world_rl.training.evaluation import close_env
        close_env(env)
        raise

    from real_world_rl.training.evaluation import evaluate_policy
    evaluate_policy(
        env, agent.policy, episodes=args.eval_n_trajs, seed=args.seed,
        predict=lambda obs: agent.policy.predict(agent._obs_to_policy_device(obs), deterministic=True),
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
from real_world_rl.training.args import (  # noqa: E402
    FrankaRealConfig,
    HilEnvArgs,
    RealWorldFrankaArgs,
)
from real_world_rl.training.registry import registry  # noqa: E402


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
