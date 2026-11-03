"""SERL run function."""
from __future__ import annotations

import dataclasses

from real_world_rl.training.runtime_paths import actor_sync_kwargs, learner_experiment_kwargs, actor_control_hz


def _serl_env_request(args, run_name):

    from real_world_rl.training.env_building import ensure_real_world_backend
    ensure_real_world_backend(args.env_backend)
    from real_world_rl.training.env_building import hil_env_request
    return hil_env_request(args, run_name)


def _run_actor(args) -> None:
    """Builds the real-robot env plus a local, inference-only RLPD instance,
    and drives ``real_world_rl.loops.serl.SerlActorLoop``.

    The local agent's optimizers and replay buffer are never touched (only
    ``agent.policy`` is used for inference) -- ``buffer_size`` is forced
    small so this doesn't allocate a full-size buffer on what is often a
    GPU-less robot control machine.
    """
    from rl_garden.envs.backend_registry import make_training_envs
    from rl_garden.training.online.entrypoints.rlpd import build_rlpd
    from real_world_rl.loops.serl import SerlActorLoop, SerlActorSyncClient

    env_request = _serl_env_request(args, run_name="real_world_actor")
    env, _ = make_training_envs(args.env_backend, env_request)
    from real_world_rl.training.env_building import compose_hil_wrappers
    from real_world_rl.training.args import HilEnvArgs
    from types import SimpleNamespace
    values = dataclasses.asdict(HilEnvArgs())
    values.update(vars(args))
    env = compose_hil_wrappers(SimpleNamespace(**values), env, env_request.backend_config, enable_teleop=False)
    loop_owns_env = False
    try:

        scratch_args = dataclasses.replace(
            args, buffer_size=8, load_checkpoint=None, offline_dataset=None
        )
        agent = build_rlpd(scratch_args, env, None, logger=None, checkpoint_dir=None)

        sync_client = SerlActorSyncClient(f"http://{args.sync_host}:{args.sync_port}", **actor_sync_kwargs(args))
        loop = SerlActorLoop(
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
    finally:
        if not loop_owns_env:
            from real_world_rl.training.runtime_paths import cleanup_construction
            clients = list(locals().get("sync_clients", {}).values())
            cleanup_construction(env, locals().get("sync_client"), *clients)


def _run_learner(args) -> None:
    """Builds a full RLPD instance (owns the replay buffer, optimizers, and
    checkpointing) and drives ``real_world_rl.loops.serl.SerlLearnerLoop``.

    The env built here is used only for its observation/action-space shape
    -- the learner never calls ``reset()``/``step()`` on it. ``FrankaRealEnv``
    doesn't make any network call until one of those is actually invoked, so
    building it is safe even when the learner machine (e.g. a remote GPU box)
    has no network route to the robot at all.
    """
    from real_world_rl.training.runtime_paths import resolve_control_frequency
    resolve_control_frequency(args)
    import os
    import time

    from rl_garden.common import Logger, seed_everything
    from rl_garden.common.resolved_config import persist_resolved_config
    from rl_garden.envs.backend_registry import make_training_envs
    from rl_garden.training.online.entrypoints.rlpd import build_rlpd
    from real_world_rl.loops.serl import SerlLearnerLoop

    seed_everything(args.seed)

    start_time = time.strftime("%Y%m%d_%H%M%S", time.localtime())
    run_name = args.exp_name or f"{args.env_id}__rlpd_real_world__{args.seed}__{int(time.time())}"
    checkpoint_dir = args.checkpoint_dir or os.path.join(args.log_dir, run_name, "checkpoints")

    resolved_config = persist_resolved_config(
        args, training_phase="real_world", algorithm="serl", run_name=run_name, log_dir=args.log_dir
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

    env_request = _serl_env_request(args, run_name=run_name)
    env, _ = make_training_envs(args.env_backend, env_request)
    from real_world_rl.training.env_building import compose_hil_wrappers
    from real_world_rl.training.args import HilEnvArgs
    from types import SimpleNamespace
    values = dataclasses.asdict(HilEnvArgs())
    values.update(vars(args))
    env = compose_hil_wrappers(SimpleNamespace(**values), env, env_request.backend_config, enable_teleop=False)
    loop_owns_env = False
    try:

        agent = build_rlpd(dataclasses.replace(args, load_checkpoint=None), env, None, logger=logger, checkpoint_dir=checkpoint_dir)

        loop = SerlLearnerLoop(
            agent,
            **learner_experiment_kwargs(env, checkpoint_dir),
            admission_capacity=args.admission_capacity,
            ingest_batch_size=args.ingest_batch_size,
            resume_bundle_path=args.load_checkpoint,
            resume_replay=args.load_replay_buffer,
            journal_path=os.path.join(checkpoint_dir, "journal.sqlite3"),
            host=args.sync_host,
            port=args.sync_port,
            train_freq=args.train_freq,
            publish_freq=args.publish_freq,
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


def run_serl(args: "SerlArgs") -> None:
    if args.role == "actor":
        _run_actor(args)
    else:
        _run_learner(args)


# ---------------------------------------------------------------------------
# Args + registration
# ---------------------------------------------------------------------------

from dataclasses import dataclass, field  # noqa: E402

from rl_garden.training.online.entrypoints.rlpd import RLPDArgs  # noqa: E402
from real_world_rl.training.args import (  # noqa: E402
    FrankaRealConfig,
    RealWorldFrankaArgs,
)
from real_world_rl.training.registry import registry  # noqa: E402


@dataclass
class SerlArgs(RealWorldFrankaArgs, RLPDArgs):
    franka_real: FrankaRealConfig = field(default_factory=FrankaRealConfig)


registry.register("serl", SerlArgs, run_serl)
