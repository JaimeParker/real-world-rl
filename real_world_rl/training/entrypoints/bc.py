"""Real-world BC: offline imitation over pre-collected demo pkls.

Structurally unlike ``serl``/``hil_serl``: BC (``rl_garden.algorithms.bc.BC``)
is an ``OfflineRLAlgorithm`` -- a static dataset, no online env stepping --
not an ``OffPolicyAlgorithm``, so there is no actor/learner HTTP split here
at all: no ``loops/sync.py``, no ``LearnerLoop``/``ActorLoop``. This reuses
``rl_garden``'s existing offline BC algorithm construction
(``rl_garden.training.offline.entrypoints.bc.build_bc``) directly against a
demo-pkl-populated buffer, rather than ``rl_garden.training.offline.runner
.run_offline`` (which expects rl-garden's own ``--offline_dataset`` h5/minari
format, not the real-world pkl-list-of-transition-dicts convention demo data
already uses -- see ``real_world_rl.training.demo_loading``).
"""
from __future__ import annotations

import os
import time



def build_real_world_bc(args, env, eval_env, logger, checkpoint_dir):
    """The BC agent on ``env``'s spaces (rl-garden's offline ``build_bc``), with the online
    builders' signature so ``deploy`` restores a real_world_bc checkpoint like any other.
    ``checkpoint_dir`` is unused: ``run_real_world_bc`` saves the checkpoints itself."""
    from rl_garden.common.cli_args import resolve_obs_groups_config
    from rl_garden.training.inspection import construct_agent
    from real_world_rl.algorithms.bc import MaskedBC

    del checkpoint_dir
    names = (
        "buffer_size", "buffer_device", "batch_size", "offline_sampling", "actor_lr",
        "weight_decay", "use_adamw", "lr_schedule", "lr_warmup_steps", "lr_decay_steps",
        "lr_min_ratio", "grad_clip_norm", "actor_use_layer_norm", "actor_use_group_norm",
        "num_groups", "actor_dropout_rate", "kernel_init", "backbone_type",
        "std_parameterization", "tanh_squash", "seed", "device", "std_log", "log_freq",
        "num_eval_steps", "save_replay_buffer",
    )
    kwargs = {name: getattr(args, name) for name in names}
    kwargs.update(env=env, logger=logger, eval_env=eval_env,
                  eval_freq=args.eval_freq if eval_env is not None else 0,
                  checkpoint_dir=None, checkpoint_freq=0, save_final_checkpoint=False,
                  encoder_config=args.encoder if args.obs.is_visual else None,
                  obs_groups=resolve_obs_groups_config(args))
    return construct_agent(MaskedBC, **kwargs)


def run_real_world_bc(args: "RealWorldBCArgs") -> None:
    from rl_garden.common import Logger, seed_everything
    from rl_garden.common.resolved_config import persist_resolved_config
    from rl_garden.envs.backend_registry import make_training_envs
    from rl_garden.training.offline.runner import _eval_env_request
    from real_world_rl.action_contract import demo_contract_of
    from real_world_rl.training.demo_loading import load_demo_pkls

    import importlib
    for backend in ("franka_real", "real_robot", "real_robot_bimanual"):
        importlib.import_module(f"real_world_rl.envs.backends.{backend}")
    seed_everything(args.seed)

    start_time = time.strftime("%Y%m%d_%H%M%S", time.localtime())
    run_name = args.exp_name or f"{args.env_id}__real_world_bc__{args.seed}__{int(time.time())}"
    checkpoint_dir = args.checkpoint_dir or os.path.join(args.log_dir, run_name, "checkpoints")

    resolved_config = persist_resolved_config(
        args, training_phase="real_world", algorithm="real_world_bc", run_name=run_name, log_dir=args.log_dir
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

    # Env is used only for its observation/action-space shape -- BC never
    # calls reset()/step() on it, and FrankaRealEnv makes no network call
    # until one of those is (mirrors serl.py's/hil_serl.py's learner-side
    # env construction). Reuses the offline-args-shaped request builder
    # (BCArgs has no online-training num_envs; _eval_env_request resolves
    # the offline spec_num_envs/num_eval_steps itself).
    import dataclasses
    from real_world_rl.training.demo_loading import read_demo_contract
    from real_world_rl.training.env_building import build_offline_experiment_env
    env = None
    try:
        env_request = _eval_env_request(args)
        original_observation = env_request.observation
        env_request.observation = dataclasses.replace(original_observation, frame_stack=1)
        env, _ = make_training_envs(args.env_backend, env_request)
        env = build_offline_experiment_env(env, env_request.backend_config, original_observation,
                                           read_demo_contract(args.demo_dataset_paths))

        agent = build_real_world_bc(args, env, None, logger, checkpoint_dir=None)
        if args.load_checkpoint is not None:
            from real_world_rl.checkpoint_contract import load_with_action_contract
            load_with_action_contract(agent, args.load_checkpoint,
                                      load_replay_buffer=args.load_replay_buffer)
        saved_contracts = []
        loaded = len(agent.replay_buffer) if args.load_checkpoint is not None and args.load_replay_buffer else 0
        if not loaded:
            loaded = load_demo_pkls(
                args.demo_dataset_paths,
                agent.replay_buffer.add,
                agent.buffer_device,
                expected_contract=demo_contract_of(env),
                on_contract=saved_contracts.append,
            )
        if loaded == 0:
            raise ValueError(
                f"No demo transitions or restored replay available from demo_dataset_paths={args.demo_dataset_paths!r}."
            )

        from pathlib import Path
        import json
        from real_world_rl.action_contract import (
            json_normalize, experiment_contract_of, effective_experiment_of, experiment_hash,
        )
        metadata_path = Path(checkpoint_dir).parent / "experiment.json"
        metadata_path.parent.mkdir(parents=True, exist_ok=True)
        source_contract = next((contract for contract in saved_contracts if contract and "experiment" in contract), None)
        requested = experiment_contract_of(env)
        effective = effective_experiment_of(env)
        if effective is not None and source_contract and "effective_control" in source_contract:
            effective = {**effective, "runtime_control": source_contract["effective_control"]}
        metadata_path.write_text(json.dumps(json_normalize(dict(experiment_hash=experiment_hash(env),
            experiment_contract=requested, effective_experiment=effective)), sort_keys=True, allow_nan=False) + "\n")

        for step in range(1, args.num_offline_steps + 1):
            metrics = agent.train(1, compute_info=(step % args.log_freq == 0))
            if step % args.log_freq == 0:
                logger.log_metrics(metrics, step)
            if args.checkpoint_freq > 0 and step % args.checkpoint_freq == 0:
                agent.save(os.path.join(checkpoint_dir, f"checkpoint_{step}.pt"), include_replay_buffer=args.save_replay_buffer)
        if args.save_final_checkpoint:
            agent.save(os.path.join(checkpoint_dir, "final.pt"), include_replay_buffer=args.save_replay_buffer)


    finally:
        from real_world_rl.training.evaluation import close_resources
        close_resources(env, logger)


# ---------------------------------------------------------------------------
# Args + registration
# ---------------------------------------------------------------------------

from dataclasses import dataclass, field  # noqa: E402
from typing import Sequence  # noqa: E402

from rl_garden.training.offline.entrypoints.bc import BCArgs  # noqa: E402
from real_world_rl.training.args import (  # noqa: E402
    FrankaRealConfig,
    RealWorldFrankaArgs,
    TeleopArgs,
)
from real_world_rl.training.registry import registry  # noqa: E402


@dataclass
class RealWorldBCArgs(TeleopArgs, RealWorldFrankaArgs, BCArgs):
    """RealWorldFrankaArgs is composed for its env-backend fields
    (franka_real, env_backend) only -- role/sync_host/sync_port/control_hz/
    deterministic_actor/train_freq/publish_freq are inherited but unused:
    BC has no actor/learner split to apply them to. TeleopArgs is included
    to share station presets; BC ignores its fields and never requires or
    constructs a teleop source."""

    franka_real: FrankaRealConfig = field(default_factory=FrankaRealConfig)
    demo_dataset_paths: Sequence[str] = field(default_factory=tuple)
    num_offline_steps: int = 100_000


registry.register("real_world_bc", RealWorldBCArgs, run_real_world_bc)
