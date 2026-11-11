"""Residual HIL-SERL run function: residual SAC on top of a frozen base policy, with
human intervention, as HIL-SERL's actor / learner / eval processes.

``--residual-variant continuous`` builds ``ResidualRLPD`` (residual on every action
dim), ``hybrid`` builds ``ResidualRLPDHybrid`` (residual on the arm dims, DQN
gripper). ``--base-provider`` names the base policy (see
``real_world_rl.policies.base_provider``); only the actor and eval build it,
the learner records the spec in its checkpoints. See ``docs/residual.md``.
"""
from __future__ import annotations

import dataclasses

from real_world_rl.training.runtime_paths import actor_sync_kwargs, learner_experiment_kwargs, actor_control_hz

from real_world_rl.training.env_building import (
    build_hil_env,
    warn_if_no_reward_source,
)


def build_residual_hil_serl(args, env, eval_env, logger, checkpoint_dir):
    """The residual agent for ``args.residual_variant`` through rl-garden's RLPD
    builders (``algorithm_cls=`` + the residual keywords). The online registry's
    ``build_agent`` for ``residual_hil_serl`` and ``residual_hil_serl_hybrid``.
    ``num_grippers`` comes from ``env``'s action contract."""
    from real_world_rl.action_contract import env_num_grippers
    from real_world_rl.checkpoint_contract import build_agent_with_action_contract
    from real_world_rl.algorithms.residual import (
        ResidualRLPD,
        ResidualRLPDHybrid,
    )

    residual_kwargs = dict(
        num_grippers=env_num_grippers(env),
        residual_action_scale=args.residual_action_scale,
        residual_gripper_scale=args.residual_gripper_scale,
        residual_zero_init=args.residual_zero_init,
        residual_log_std_init=args.residual_log_std_init,
        residual_bc_weight=args.residual_bc_weight,
        base_provider_spec=args.base_provider,
        critic_only_updates=args.critic_only_updates,
        # build_rlpd_hybrid does not forward std_parameterization and build_rlpd does not
        # forward backup_entropy; pass both so the two variants read the same Args.
        std_parameterization=args.std_parameterization,
        backup_entropy=args.backup_entropy,
    )
    if args.residual_variant == "hybrid":
        if env_num_grippers(env) == 0:
            raise ValueError("hybrid residual requires a gripper; use --residual-variant continuous")
        from rl_garden.training.online.entrypoints.rlpd_hybrid import build_rlpd_hybrid

        return build_agent_with_action_contract(build_rlpd_hybrid,
            args, env, eval_env, logger, checkpoint_dir, algorithm_cls=ResidualRLPDHybrid, **residual_kwargs
        )
    from rl_garden.training.online.entrypoints.rlpd import build_rlpd

    return build_agent_with_action_contract(build_rlpd,
        args, env, eval_env, logger, checkpoint_dir, algorithm_cls=ResidualRLPD, **residual_kwargs
    )


def validate_residual_args(args) -> None:
    """Reject option combinations the residual method does not support."""
    if not args.base_provider.strip():
        raise ValueError(
            "--base-provider is required: 'zero', 'checkpoint:<path>?algo=<name>' or 'act:<path>'"
        )
    if args.fwbw:
        raise NotImplementedError("residual_hil_serl does not support --fwbw.")
    if args.memory_efficient_buffer:
        raise ValueError("residual_hil_serl does not support --memory-efficient-buffer.")
    if args.nstep != 1:
        raise ValueError(f"residual_hil_serl needs --nstep 1, got {args.nstep}.")
    if args.residual_variant == "continuous" and args.use_grasp_penalty:
        raise ValueError(
            "--use-grasp-penalty shapes the hybrid variant's DQN gripper target; use "
            "--residual-variant hybrid or drop it."
        )


def _scratch_agent(args, env):
    """An agent with an untrained policy and a token replay buffer, for the actor
    (the learner publishes the weights)."""
    scratch_args = dataclasses.replace(args, buffer_size=8, load_checkpoint=None, offline_dataset=None)
    return build_residual_hil_serl(scratch_args, env, None, logger=None, checkpoint_dir=None)


def _make_provider(args, env, device):
    from real_world_rl.policies.base_provider import make_base_provider

    return make_base_provider(
        args.base_provider,
        device=device,
        observation_space=env.single_observation_space,
        action_space=env.single_action_space,
    )


def _run_actor(args) -> None:
    from real_world_rl.loops import residual
    from real_world_rl.loops.hil_serl import HilSerlActorSyncClient

    env = build_hil_env(args, role="actor")
    loop_owns_env = False
    try:
        if args.record_video_dir is not None:
            from real_world_rl.envs.wrappers.video_recorder import (
                VideoRecorderWrapper,
            )

            env = VideoRecorderWrapper(env, save_dir=args.record_video_dir)

        agent = _scratch_agent(args, env)
        provider = _make_provider(args, env, agent.device)
        sync_client = HilSerlActorSyncClient(f"http://{args.sync_host}:{args.sync_port}", **actor_sync_kwargs(args))
        loop = residual.ResidualActorLoop(
            env,
            agent.policy,
            sync_client,
            provider,
            warmup_unit_residual=args.residual_warmup_unit,
            base_resync_on_intervention=args.residual_base_resync_on_intervention,
            control_hz=actor_control_hz(args),
            device=agent.device,
            deterministic=args.deterministic_actor,
            seed=args.seed,
        )
        if args.episode_end_behavior != "auto":
            loop.configure_episode_session(args.episode_end_behavior,
                                           neutral_threshold=args.neutral_threshold,
                                           neutral_stable_s=args.neutral_stable_s)
        loop_owns_env = True
        loop.run()
    finally:
        if not loop_owns_env:
            from real_world_rl.training.runtime_paths import cleanup_construction
            clients = list(locals().get("sync_clients", {}).values())
            cleanup_construction(env, locals().get("sync_client"), *clients)


def _run_learner(args) -> None:
    from real_world_rl.training.runtime_paths import resolve_control_frequency
    from real_world_rl.training.env_building import resolve_spacemouse_mode
    resolve_spacemouse_mode(args)
    resolve_control_frequency(args)
    import os
    import time

    from rl_garden.common import Logger, seed_everything
    from rl_garden.common.resolved_config import persist_resolved_config
    from real_world_rl.loops import residual
    from real_world_rl.action_contract import demo_contract_of

    seed_everything(args.seed)

    start_time = time.strftime("%Y%m%d_%H%M%S", time.localtime())
    run_name = args.exp_name or f"{args.env_id}__residual_hil_serl_real_world__{args.seed}__{int(time.time())}"
    checkpoint_dir = args.checkpoint_dir or os.path.join(args.log_dir, run_name, "checkpoints")

    from real_world_rl.training.env_building import resolve_teleop_input
    resolve_teleop_input(args)
    resolved_config = persist_resolved_config(
        args,
        training_phase="real_world",
        algorithm="residual_hil_serl",
        run_name=run_name,
        log_dir=args.log_dir,
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

    # Spaces and action contract only: no teleop device and no base policy on the learner.
    env = build_hil_env(args, role="learner", enable_teleop=False, run_name=run_name)
    loop_owns_env = False
    try:

        agent = build_residual_hil_serl(dataclasses.replace(args, load_checkpoint=None), env, None, logger=logger, checkpoint_dir=checkpoint_dir)
        agent.init_demo_buffer(args.demo_buffer_size, args.demo_data_ratio)

        loop = residual.ResidualLearnerLoop(
            agent,
            **learner_experiment_kwargs(env, checkpoint_dir),
            admission_capacity=args.admission_capacity,
            ingest_batch_size=args.ingest_batch_size,
            resume_bundle_path=args.load_checkpoint,
            resume_replay=args.load_replay_buffer,
            journal_path=os.path.join(checkpoint_dir, "journal.sqlite3"),
            host=args.sync_host,
            port=args.sync_port,
            checkpoint_dir=checkpoint_dir,
            buffer_period=args.buffer_period,
            demo_dataset_paths=args.demo_dataset_paths,
            train_freq=args.train_freq,
            publish_freq=args.publish_freq,
            action_contract=demo_contract_of(env),
            allow_base_less_demos=args.allow_base_less_demos,
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
    from real_world_rl.policies.base_provider import ResidualPolicyWithBase

    if args.load_checkpoint is None:
        raise ValueError("residual_hil_serl eval requires --load_checkpoint.")

    env = build_hil_env(args, role="eval")
    provider = None
    try:
        from real_world_rl.training.evaluation import resolve_eval_checkpoint
        checkpoint = resolve_eval_checkpoint(args, env)
        eval_args = dataclasses.replace(args, buffer_size=8, offline_dataset=None, load_checkpoint=checkpoint, load_replay_buffer=False)
        agent = build_residual_hil_serl(eval_args, env, None, logger=None, checkpoint_dir=None)
        provider = _make_provider(args, env, agent.device)
        policy = ResidualPolicyWithBase(agent.policy, provider).eval()

    except BaseException:
        from real_world_rl.training.evaluation import close_resources
        close_resources(env, provider)
        raise

    from real_world_rl.training.evaluation import evaluate_policy
    evaluate_policy(env, policy, episodes=args.eval_n_trajs, seed=args.seed)


def run_residual_hil_serl(args: "ResidualHilSerlArgs") -> None:
    validate_residual_args(args)
    if args.role in ("actor", "learner"):
        warn_if_no_reward_source(args, "residual_hil_serl")
    if args.role == "actor":
        _run_actor(args)
    elif args.role == "learner":
        _run_learner(args)
    else:
        _run_eval(args)


# ---------------------------------------------------------------------------
# Args + registration
# ---------------------------------------------------------------------------

from dataclasses import dataclass  # noqa: E402
from typing import Literal  # noqa: E402

from real_world_rl.training.entrypoints.hil_serl import HilSerlArgs  # noqa: E402
from real_world_rl.training.registry import registry  # noqa: E402


@dataclass
class ResidualHilSerlArgs(HilSerlArgs):
    """HIL-SERL with a residual actor on top of a frozen base policy."""

    # "continuous": ResidualRLPD, residual on every action dim (gripper scaled by
    # residual_gripper_scale). "hybrid": ResidualRLPDHybrid, residual on the arm dims and
    # the gripper from the DQN head.
    residual_variant: Literal["continuous", "hybrid"] = "continuous"
    # Executed action = clamp(base + scale * unit_residual, -1, 1), per dim.
    residual_action_scale: float = 0.1
    residual_gripper_scale: float = 0.0
    # Zero mean head at start: the first published policy acts exactly like the base.
    residual_zero_init: bool = True
    residual_log_std_init: float = -3.0
    # Weight of the BC term pulling the unit residual towards the human's implied residual
    # on intervened rows (0 disables it).
    residual_bc_weight: float = 0.0
    # Actor-side unit residual noise amplitude until the first learner weights arrive
    # (0: pure base actions).
    residual_warmup_unit: float = 0.0
    # Actor resets the base provider after every intervened step, so a stateful base (ACT)
    # replans from the robot's actual state instead of a stale chunk (no-op for zero/checkpoint).
    residual_base_resync_on_intervention: bool = True
    # Base policy spec, e.g. "zero", "checkpoint:<path>?algo=real_world_bc", "act:<path>".
    # Required (checked when the run starts).
    base_provider: str = ""
    # The first N learner train() calls update the critics only.
    critic_only_updates: int = 0
    # Load --demo-dataset-paths pkls without base_actions/next_base_actions (e.g. hil_serl or
    # teleop recordings) with zero base actions instead of failing; those rows are then
    # learned as residuals from a zero base.
    allow_base_less_demos: bool = False
    # RLPD actor log-std parameterization (RLPDArgs has it; RLPDHybridArgs does not).
    std_parameterization: Literal["exp", "uniform"] = "exp"


@dataclass
class ResidualHilSerlHybridArgs(ResidualHilSerlArgs):
    """``ResidualHilSerlArgs`` defaulting to the hybrid variant. Exists only so rl-garden's
    online registry can carry the ``ResidualRLPDHybrid`` class under its own name
    (``residual_hil_serl_hybrid``; one Args class per name); train with
    ``residual_hil_serl --residual-variant hybrid``."""

    residual_variant: Literal["continuous", "hybrid"] = "hybrid"


registry.register("residual_hil_serl", ResidualHilSerlArgs, run_residual_hil_serl)
