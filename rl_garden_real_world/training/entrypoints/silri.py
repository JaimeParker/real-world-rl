"""HIL-RL (SiLRI) run function: actor / learner / eval roles on the HIL env stack.

The actor drives the robot with ``SilriActorLoop`` (noisy policy actions, human takeovers
through the teleop wrapper, one-step delayed ``intervention_onset`` tags); the learner
trains ``SiLRI`` from the pushed transitions with ``SilriLearnerLoop`` (online + demo
buffer, behaviour-cloning pretraining, behaviour-model refreshes); eval runs the
deterministic policy from ``--load_checkpoint``. See ``docs/silri.md``.
"""
from __future__ import annotations

import dataclasses
from dataclasses import dataclass, field
from typing import Any, Literal, Optional

import torch

from rl_garden.common.cli_args import CheckpointArgs, ObservationArgs
from rl_garden.common.env_args import EnvBackendArgs, EnvRunArgs
from rl_garden.encoders.config import EncoderConfig
from rl_garden_real_world.training.args import (
    FrankaRealConfig,
    HilEnvArgs,
    RealWorldFrankaArgs,
)
from rl_garden_real_world.training.env_building import (
    build_hil_env,
    warn_if_no_reward_source,
)
from rl_garden_real_world.training.registry import registry


@dataclass
class SilriArgs(
    HilEnvArgs, RealWorldFrankaArgs, EnvRunArgs, CheckpointArgs, ObservationArgs, EnvBackendArgs
):
    """HIL-RL (SiLRI): actor-critic pulled towards a learned model of the human's
    interventions under a state-wise Lagrange multiplier. Defaults follow the reference
    Franka configuration."""

    franka_real: FrankaRealConfig = field(default_factory=FrankaRealConfig)
    role: Literal["actor", "learner", "eval"] = "actor"
    encoder: EncoderConfig = field(
        default_factory=lambda: EncoderConfig(
            image_augmentation="random_shift",
            pooling_method="spatial_learned_embeddings",
        )
    )

    buffer_size: int = 30_000
    buffer_device: str = "cuda"
    demo_buffer_size: int = 10_000
    # Online transitions / demo rows the learner waits for before training.
    learning_starts: int = 10
    demo_learning_starts: int = 10
    batch_size: int = 256
    gamma: float = 0.97
    tau: float = 0.005
    # Critic updates per gradient step (one actor and one multiplier update each).
    utd: int = 2
    actor_lr: float = 3e-4
    critic_lr: float = 3e-4
    behavior_lr: float = 3e-4
    lambda_lr: float = 3e-6
    grad_clip_norm: Optional[float] = 40.0
    # Std of the actor's Gaussian sampling noise (actor and TD target).
    actor_noise_std: float = 0.05
    # Tolerance offset: the multiplier rises where ||mu_pi - mu_b|| > sum(sigma_b) + margin.
    constraint_margin: float = 0.2
    bc_pretrain_steps: int = 500
    behavior_refresh_steps: int = 50
    # Demo-buffer additions between behaviour-model refreshes (0 disables refreshes).
    behavior_refresh_every: int = 50
    std_min: float = 1e-5
    std_max: float = 2.0
    # Also copy the transition before an intervention onset to the demo buffer.
    onset_to_demo_buffer: bool = False
    feature_dim: int = 256
    hidden_dim: int = 256
    behavior_hidden_dims: tuple[int, ...] = (256, 256)
    lambda_hidden_dims: tuple[int, ...] = (256, 256)
    # The last action dims (one gripper per arm, from the env contract) are open/hold/close
    # grippers predicted by a classification head.
    discrete_gripper: bool = True
    discrete_hidden_dim: int = 256


def build_silri(
    args,
    env,
    eval_env,
    logger,
    checkpoint_dir,
    *,
    algorithm_cls: type | None = None,
    **extra_kwargs: Any,
):
    """Build ``algorithm_cls`` (``SiLRI`` when ``None``) from ``args``; ``extra_kwargs``
    win over the keywords taken from ``args``; ``num_grippers`` comes from ``env``'s action
    contract. Loads ``args.load_checkpoint`` when set.
    The demo buffer is left to the caller (``agent.init_demo_buffer``)."""
    from rl_garden.common.cli_args import (
        resolve_critic_encoder_config,
        resolve_obs_groups_config,
    )
    from rl_garden.training.inspection import construct_agent
    from rl_garden_real_world.action_contract import (
        env_num_grippers,
        get_env_action_contract,
    )
    from rl_garden_real_world.algorithms.silri import SiLRI

    contract = get_env_action_contract(env)
    if contract is not None and args.discrete_gripper and not contract["gripper_in_action"]:
        raise ValueError(
            "--discrete-gripper needs a gripper dim in the env action (gripper_in_action=true); "
            "pass --no-discrete-gripper for an arm-only action."
        )
    agent_kwargs: dict[str, Any] = dict(
        env=env,
        eval_env=eval_env,
        buffer_size=args.buffer_size,
        buffer_device=args.buffer_device,
        learning_starts=args.learning_starts,
        demo_learning_starts=args.demo_learning_starts,
        batch_size=args.batch_size,
        gamma=args.gamma,
        tau=args.tau,
        utd=args.utd,
        actor_lr=args.actor_lr,
        critic_lr=args.critic_lr,
        behavior_lr=args.behavior_lr,
        lambda_lr=args.lambda_lr,
        grad_clip_norm=args.grad_clip_norm,
        actor_noise_std=args.actor_noise_std,
        constraint_margin=args.constraint_margin,
        bc_pretrain_steps=args.bc_pretrain_steps,
        behavior_refresh_steps=args.behavior_refresh_steps,
        behavior_refresh_every=args.behavior_refresh_every,
        std_min=args.std_min,
        std_max=args.std_max,
        onset_to_demo_buffer=args.onset_to_demo_buffer,
        feature_dim=args.feature_dim,
        hidden_dim=args.hidden_dim,
        behavior_hidden_dims=tuple(args.behavior_hidden_dims),
        lambda_hidden_dims=tuple(args.lambda_hidden_dims),
        discrete_gripper=args.discrete_gripper,
        discrete_hidden_dim=args.discrete_hidden_dim,
        num_grippers=env_num_grippers(env),
        encoder_config=args.encoder if args.obs.is_visual else None,
        obs_groups=resolve_obs_groups_config(args),
        critic_encoder_config=resolve_critic_encoder_config(args),
        image_augmentation_seed=args.seed + 1_000_003,
        seed=args.seed,
        logger=logger,
        std_log=args.std_log,
        log_freq=args.log_freq,
        num_eval_steps=args.num_eval_steps,
        checkpoint_dir=checkpoint_dir,
        checkpoint_freq=args.checkpoint_freq,
        save_replay_buffer=args.save_replay_buffer,
        save_final_checkpoint=args.save_final_checkpoint,
    )
    if args.encoder_sharing is not None:
        agent_kwargs["encoder_sharing"] = args.encoder_sharing
    agent_kwargs.update(extra_kwargs)
    agent = construct_agent(algorithm_cls or SiLRI, **agent_kwargs)
    if args.load_checkpoint is not None:
        agent.load(args.load_checkpoint, load_replay_buffer=args.load_replay_buffer)
    return agent


def _reject_unsupported_options(args) -> None:
    if args.fwbw:
        raise NotImplementedError("silri does not support --fwbw.")
    if args.use_grasp_penalty:
        raise ValueError(
            "silri has no discrete critic to consume --use-grasp-penalty; drop the flag."
        )


def _scratch_agent(args, env):
    """An agent for inference only: the actor's policy, refreshed from the learner."""
    scratch_args = dataclasses.replace(args, buffer_size=8, load_checkpoint=None)
    return build_silri(scratch_args, env, None, logger=None, checkpoint_dir=None)


def _run_actor(args) -> None:
    from rl_garden_real_world.loops.hil_serl import HilSerlActorSyncClient
    from rl_garden_real_world.loops.silri import SilriActorLoop

    env = build_hil_env(args, role="actor")
    if args.record_video_dir is not None:
        from rl_garden_real_world.envs.wrappers.video_recorder import (
            VideoRecorderWrapper,
        )

        env = VideoRecorderWrapper(env, save_dir=args.record_video_dir)
    agent = _scratch_agent(args, env)
    sync_client = HilSerlActorSyncClient(f"http://{args.sync_host}:{args.sync_port}")
    loop = SilriActorLoop(
        env,
        agent.policy,
        sync_client,
        control_hz=args.control_hz,
        device=agent.device,
        deterministic=args.deterministic_actor,
        seed=args.seed,
    )
    loop.run()


def _run_learner(args) -> None:
    import os
    import time

    from rl_garden.common import Logger, seed_everything
    from rl_garden.common.resolved_config import persist_resolved_config
    from rl_garden_real_world.action_contract import demo_contract_of
    from rl_garden_real_world.loops.silri import SilriLearnerLoop

    seed_everything(args.seed)
    start_time = time.strftime("%Y%m%d_%H%M%S", time.localtime())
    run_name = args.exp_name or f"{args.env_id}__silri_real_world__{args.seed}__{int(time.time())}"
    checkpoint_dir = args.checkpoint_dir or os.path.join(args.log_dir, run_name, "checkpoints")

    resolved_config = persist_resolved_config(
        args, training_phase="real_world", algorithm="silri", run_name=run_name, log_dir=args.log_dir
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
    agent = build_silri(args, env, None, logger=logger, checkpoint_dir=checkpoint_dir)
    agent.init_demo_buffer(args.demo_buffer_size, args.demo_data_ratio)

    loop = SilriLearnerLoop(
        agent,
        host=args.sync_host,
        port=args.sync_port,
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
    """Deterministic episodes of the ``--load_checkpoint`` policy on the robot; success is
    ``terminated`` (the reward classifier's success), a truncated episode is a failure."""
    from rl_garden_real_world.loops.actor import ActorLoop

    if args.load_checkpoint is None:
        raise ValueError("silri eval requires --load_checkpoint.")

    env = build_hil_env(args, role="eval")
    agent = build_silri(dataclasses.replace(args, buffer_size=8), env, None, logger=None, checkpoint_dir=None)
    agent.policy.eval()

    successes = 0
    returns: list[float] = []
    obs, _ = env.reset(seed=args.seed)
    for episode in range(args.eval_n_trajs):
        episode_return = 0.0
        while True:
            with torch.no_grad():
                action = agent.policy.predict(agent._obs_to_policy_device(obs), deterministic=True)
            obs, reward, terminated, truncated, _ = env.step(action.to(ActorLoop.env_device(obs)))
            episode_return += float(reward.sum())
            if bool(terminated) or bool(truncated):
                success = bool(terminated)
                successes += int(success)
                returns.append(episode_return)
                print(f"[eval] episode={episode} success={success} return={episode_return:.3f}", flush=True)
                obs, _ = env.reset(seed=args.seed)
                break

    mean_return = sum(returns) / len(returns) if returns else 0.0
    print(
        f"[eval] success_rate={successes / args.eval_n_trajs:.3f} mean_return={mean_return:.3f} "
        f"n_trajs={args.eval_n_trajs}",
        flush=True,
    )


def run_silri(args: SilriArgs) -> None:
    _reject_unsupported_options(args)
    if args.role in ("actor", "learner"):
        warn_if_no_reward_source(args, "silri")
    if args.role == "actor":
        _run_actor(args)
    elif args.role == "learner":
        _run_learner(args)
    else:
        _run_eval(args)


registry.register("silri", SilriArgs, run_silri)
