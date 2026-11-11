"""Eval-only deployment of a trained rl-garden policy on a ``real_robot`` backend robot.

No learner, no sync server, no data collection: restore the policy from a checkpoint, then
run ``episodes`` reset/step episodes and report steps, return, RTT and robot health.

Policy restore mirrors rl-garden's ``examples/eval_checkpoint.py`` (there is no library-level
eval loader): the online registry's ``<algo>`` Args are filled from the run's ``config.json``
(``<run>/config.json`` next to ``<run>/checkpoints/``) and the checkpoint's
``metadata.hyperparameters``, the agent is rebuilt with the registry entry's ``build_agent`` against
the deploy env's spaces, and ``BaseAlgorithm.load`` restores the weights (no replay, no optimizers).
Checkpoint/space mismatches therefore surface before the robot moves (nothing connects until
the first ``reset``). ``--algo real_world_bc`` restores a real-world BC run the same way, with
the real-world registry's Args and ``build_real_world_bc`` (BC is not on the online registry).
The gripper count of an ``rlpd_hybrid`` agent comes from the deploy env's action contract, as
in hil_serl.

The training run is the source of truth for the action/observation contract: the env is built
from the run's ``real_robot`` config (``resolve_deploy_robot_config``; the CLI only supplies
connection fields and an explicit, conflicting contract field is an error) and the
training-time observation wrappers (HIL-SERL frame stacking / rotvec conversion) are re-applied.
Task deployment uses the shared classifier, intervention and reward-source composition.
Saved model contents identify reward sources; local file paths remain replaceable provenance.

A run trained on two arms deploys with ``--env-backend real_robot_bimanual``: the env is built
from the run's ``real_robot_bimanual`` config the same way (``resolve_deploy_bimanual_config``),
checking the contract fields on each arm.
"""
from __future__ import annotations

import json
import logging
import sys
from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field, fields, is_dataclass, replace
from pathlib import Path
from typing import Any, Literal, Optional

import torch

from real_world_rl.cli.errors import DEVICE_EXIT_CODE
from real_world_rl.envs.bimanual.config import BimanualRealRobotEnvConfig
from real_world_rl.envs.real_robot.config import RealRobotEnvConfig

_BACKEND = "real_robot"
_BIMANUAL_BACKEND = "real_robot_bimanual"
_DEPLOY_BACKENDS = (_BACKEND, _BIMANUAL_BACKEND)
# Inverse of rl_garden.common.effective_config.json_value's non-finite float encoding.
_JSON_FLOAT_SENTINELS = {"Infinity": float("inf"), "-Infinity": float("-inf"), "NaN": float("nan")}
# Kept for existing importers; the code itself lives in the shared exit-code table.
DEVICE_ERROR_EXIT_CODE = DEVICE_EXIT_CODE
_LOGGER = logging.getLogger(__name__)
# Fields that define what the policy's actions and observations mean: the training run decides.
# Everything else on RealRobotEnvConfig (robot, remote, token, home_joints, cameras, safety_box,
# reset_settle_s) describes this deployment's connection and stays under CLI control.
# In SERL semantics, safety_box also belongs to the compared pose limits.
_CONTRACT_FIELDS = (
    "task",
    "task_frame",
    "workspace",
    "control",
    "preprocess",
    "depth_cameras",
    "action_mode",
    "gripper_in_action",
    "num_joints",
    "joint_limits",
    "xyz_scale_m",
    "rot_scale_rad",
    "joint_scale_rad",
    "state_fields",
    "image_size",
    "control_hz",
    "max_episode_steps",
    "cartesian_semantics",
    "safety_box_rpy_low",
    "safety_box_rpy_high",
    "gripper_action_scale",
    "gripper_sleep_s",
)
# The two-arm config splits the same fields: max_episode_steps (and the arm names, which name
# the observation keys) on the two-arm config, the rest on each arm.
_BIMANUAL_CONTRACT_FIELDS = ("arm_names", "max_episode_steps", "task")
_BIMANUAL_ARM_CONTRACT_FIELDS = tuple(name for name in _CONTRACT_FIELDS if name not in ("max_episode_steps", "task"))
_BIMANUAL_ARMS = ("left", "right")
# The real-world registry's offline BC method: restored from that registry, not the online one.
_REAL_WORLD_BC = "real_world_bc"
# Online builders that take the gripper count as a keyword; the residual and SiLRI builders read
# it from the env's action contract themselves.
_GRIPPER_COUNT_KEYWORD_ALGOS = ("rlpd_hybrid",)


@dataclass
class DeployArgs:
    """Run a trained policy on a real robot (eval only)."""

    # Path to a checkpoint_<step>.pt written by a SERL/HIL-SERL learner (or any online run).
    checkpoint: str
    # Online-registry algorithm that produced the checkpoint ("rlpd" for serl,
    # "rlpd_hybrid" for hil_serl, "residual_hil_serl" or "residual_hil_serl_hybrid" for
    # residual_hil_serl), or "real_world_bc" for a real_world_bc run.
    algo: str = "rlpd"
    # Robot connection and env settings (exactly one of --real_robot.robot / .remote).
    real_robot: RealRobotEnvConfig = field(default_factory=RealRobotEnvConfig)
    # "real_robot_bimanual" for a run trained on two arms (connection settings then go to
    # --real_robot_bimanual.{left,right}.*); must match the training run's backend.
    env_backend: Literal["real_robot", "real_robot_bimanual"] = "real_robot"
    real_robot_bimanual: BimanualRealRobotEnvConfig = field(default_factory=BimanualRealRobotEnvConfig)
    # Number of reset/step episodes to run.
    episodes: int = 5
    enable_teleop: bool = True
    teleop_device: Optional[str] = None
    # None restores the training mode; without training config, tracker.
    spacemouse_mode: Optional[Literal["tracker", "serl"]] = None
    teleop_event_device: Optional[str] = None
    classifier_checkpoint: Optional[str] = None
    # Use the policy's deterministic (mean) action.
    deterministic: bool = True
    # Per-episode step cap on top of real_robot.max_episode_steps; None means no extra cap.
    max_steps_per_episode: Optional[int] = None
    # Device the restored policy runs inference on.
    device: str = "cpu"
    # Seeds rl-garden RNGs and the first reset.
    seed: int = 0
    # Training run's config.json; defaults to <checkpoint dir>/../config.json when present.
    config_path: Optional[str] = None
    # Residual checkpoints only: base action provider spec ("zero", "checkpoint:<path>?algo=...",
    # "act:<path>") replacing the one recorded at training (e.g. the base checkpoint moved).
    base_provider: Optional[str] = None


@dataclass
class EpisodeResult:
    steps: int
    episode_return: float
    truncated: bool
    rtt_ms_mean: Optional[float]
    health_ok: Optional[bool]
    outcome: Any = None


# -- policy restore -------------------------------------------------------------------


def _set_existing_attr(target: Any, key: str, value: Any) -> None:
    if not hasattr(target, key):
        return
    current = getattr(target, key)
    if isinstance(current, float) and isinstance(value, str) and value in _JSON_FLOAT_SENTINELS:
        value = _JSON_FLOAT_SENTINELS[value]
    if is_dataclass(current) and value is None:
        # Collapsed sentinel dataclass (e.g. ObsGroups()) serialized as None: keep the default.
        return
    if is_dataclass(current) and isinstance(value, Mapping):
        if type(current).__dataclass_params__.frozen:
            names = {f.name for f in fields(current)}
            setattr(target, key, replace(current, **{k: v for k, v in value.items() if k in names}))
        else:
            _apply_mapping(current, value)
        return
    setattr(target, key, value)


def _apply_mapping(target: Any, values: Mapping[str, Any]) -> None:
    for key, value in values.items():
        _set_existing_attr(target, key, value)


def _checkpoint_file(args):
    path = Path(args.checkpoint)
    if path.is_dir(): path = path / "bundle_manifest.json"
    if path.name == "bundle_manifest.json":
        manifest = json.loads(path.read_text())
        generation = path.parent / manifest["generation"]
        if manifest.get("version") != 1 or generation.parent.resolve() != path.parent.resolve():
            raise ValueError("invalid checkpoint bundle manifest")
        return generation / "agent.pt"
    return path


def _saved_experiment(args):
    file = _checkpoint_file(args)
    candidates = [file.parent / "experiment.json", _config_path(args).parent / "experiment.json"]
    for path in candidates:
        if path.is_file():
            saved = json.loads(path.read_text())
            return saved.get("effective_experiment") or saved.get("experiment_contract")
    return None


def _config_path(args: DeployArgs) -> Path:
    if args.config_path is not None:
        return Path(args.config_path)
    checkpoint = _checkpoint_file(args)
    for directory in list(checkpoint.parents)[:4]:
        candidate = directory / "config.json"
        if candidate.is_file(): return candidate
    return Path(args.checkpoint).parent.parent / "config.json"


def _training_args_cls(algo: str) -> type:
    """The Args class of ``algo``'s runs: the real-world registry's for ``real_world_bc``, the
    online registry entry's otherwise."""
    if algo == _REAL_WORLD_BC:
        from real_world_rl.training.registry import (
            registry as real_world_registry,
        )

        real_world_registry.discover()
        return real_world_registry.entries()[_REAL_WORLD_BC].args_cls
    from rl_garden.training.online.registry import registry

    registry.discover()
    entries = registry.entries()
    if algo not in entries:
        raise ValueError(
            f"unknown algo {algo!r}; online registry has {sorted(entries)} (or {_REAL_WORLD_BC!r})"
        )
    return entries[algo].args_cls


def restore_training_args(args: DeployArgs) -> Any:
    """The ``<algo>`` Args as the checkpoint's run trained them, with inference-only overrides."""
    from rl_garden.common.checkpoint import load_checkpoint_file

    training_args = _training_args_cls(args.algo)()

    saved = None
    config_path = _config_path(args)
    if config_path.exists():
        config = json.loads(config_path.read_text(encoding="utf-8"))
        saved = config.get("inputs", config.get("args", config))
        saved_backend = saved.get("env_backend")
        if saved_backend in _DEPLOY_BACKENDS and saved_backend != args.env_backend:
            raise ValueError(
                f"the training run used --env-backend {saved_backend}; deploy it with "
                f"--env-backend {saved_backend} (got {args.env_backend})"
            )
        _apply_mapping(training_args, saved)
    checkpoint = load_checkpoint_file(_checkpoint_file(args), map_location="cpu")
    hyperparameters = checkpoint.get("metadata", {}).get("hyperparameters", {})
    if isinstance(hyperparameters, Mapping):
        _apply_mapping(training_args, hyperparameters)
    if saved is not None:
        # Saved real-world fields are finalized before serialization; they own
        # the deployed env semantics, including over older checkpoint metadata.
        _restore_real_world_fields(training_args, saved)

    _apply_mapping(
        training_args,
        {
            "env_backend": args.env_backend,
            "num_envs": 1,
            "num_eval_envs": 1,
            "seed": args.seed,
            "buffer_device": "cpu",
            "buffer_size": 8,
            "offline_dataset": None,
            "load_checkpoint": None,
            "load_replay_buffer": False,
            "checkpoint_dir": None,
            "checkpoint_freq": 0,
            "save_replay_buffer": False,
            "save_final_checkpoint": False,
            "eval_freq": 0,
            "capture_video": False,
            "log_type": "none",
        },
    )
    if args.algo == _REAL_WORLD_BC:
        # BC builds on its Args' device, which the checkpoint metadata set to the training
        # device; online builders pick theirs and restore_policy moves the policy to --device.
        _apply_mapping(training_args, {"device": args.device})
    return training_args


def _restore_real_world_fields(training_args: Any, saved: Mapping[str, Any]) -> None:
    """Attach the real-world-only training fields the online ``<algo>`` Args do not declare.

    ``real_robot`` is attached only for ``env_backend == "real_robot"`` runs (franka_real runs
    persist a default, meaningless ``real_robot``), ``real_robot_bimanual`` only for
    ``env_backend == "real_robot_bimanual"`` runs; ``convert_obs_rotation`` and
    ``classifier_image_keys`` (the frame-stacked camera keys) come from hil_serl.
    """
    from real_world_rl.training.args import HilEnvArgs

    for item in fields(HilEnvArgs):
        if item.name in saved:
            setattr(training_args, item.name, saved[item.name])
    saved_robot = saved.get("real_robot")
    if saved.get("env_backend") == _BACKEND and isinstance(saved_robot, Mapping):
        training_args.real_robot = _real_robot_config_from_saved(saved_robot)
    saved_bimanual = saved.get("real_robot_bimanual")
    if saved.get("env_backend") == _BIMANUAL_BACKEND and isinstance(saved_bimanual, Mapping):
        training_args.real_robot_bimanual = _bimanual_config_from_saved(saved_bimanual)
    if "convert_obs_rotation" in saved:
        training_args.convert_obs_rotation = bool(saved["convert_obs_rotation"])
    if "classifier_image_keys" in saved:
        training_args.classifier_image_keys = tuple(saved["classifier_image_keys"] or ())


def _real_robot_config_from_saved(saved_robot: Mapping[str, Any]) -> RealRobotEnvConfig:
    names = {f.name for f in fields(RealRobotEnvConfig)}
    return RealRobotEnvConfig(**{k: v for k, v in saved_robot.items() if k in names})


def _bimanual_config_from_saved(saved_bimanual: Mapping[str, Any]) -> BimanualRealRobotEnvConfig:
    names = {f.name for f in fields(BimanualRealRobotEnvConfig)}
    values = {k: v for k, v in saved_bimanual.items() if k in names}
    for arm in _BIMANUAL_ARMS:
        if isinstance(values.get(arm), Mapping):
            values[arm] = _real_robot_config_from_saved(values[arm])
    return BimanualRealRobotEnvConfig(**values)


def _contract_conflicts(cli: Any, trained: Any, defaults: Any, names: tuple[str, ...], prefix: str = "") -> list[str]:
    """The contract fields given on the CLI (differing from ``defaults``) that contradict training."""
    from real_world_rl.action_contract import _contract_values_match, json_normalize
    return [
        f"{prefix}{name}: training={getattr(trained, name)!r}, cli={getattr(cli, name)!r}"
        for name in names
        if not _contract_values_match(json_normalize(getattr(cli, name)), json_normalize(getattr(defaults, name)))
        and not _contract_values_match(json_normalize(getattr(cli, name)), json_normalize(getattr(trained, name)))
    ]


def _training_geometry(config):
    station = config.robot or {}
    return {name: getattr(config, name) if getattr(config, name) is not None else station.get(name)
            for name in ("task_frame", "workspace")}


def resolve_deploy_robot_config(args: DeployArgs, training_args: Any) -> RealRobotEnvConfig:
    """The deploy ``RealRobotEnvConfig``: contract fields from training, connection fields from the CLI.

    A contract field counts as given on the CLI when it differs from the dataclass default; a
    given value that also differs from the training run's is a ``ValueError``. Runs without a
    ``real_robot`` config fall back to the CLI config entirely (with a warning).
    SERL restores all three safety boxes from training, rejecting different CLI limits;
    current semantics keep the existing CLI-controlled xyz safety box.
    """
    cli = args.real_robot
    trained = getattr(training_args, "real_robot", None)
    if trained is None:
        _LOGGER.warning(
            "training config has no real_robot section (older run or franka_real); using the "
            "--real_robot.* CLI config as is, so action/observation contract is NOT verified"
        )
        return cli
    # Local training configs may carry station geometry inside robot instead.
    trained = replace(trained, **_training_geometry(trained))
    contract_fields = _CONTRACT_FIELDS
    if trained.cartesian_semantics == "serl":
        contract_fields += ("safety_box",)
    conflicts = _contract_conflicts(cli, trained, RealRobotEnvConfig(), contract_fields)
    if conflicts:
        raise ValueError(
            "--real_robot.* contradicts the training run's real_robot config; the training run "
            "is the source of truth for these fields (drop them from the CLI):\n  "
            + "\n  ".join(conflicts)
        )
    values = {name: getattr(trained, name) for name in contract_fields}
    if cli.task.classifier_checkpoints:
        values["task"] = replace(trained.task, classifier_checkpoints=cli.task.classifier_checkpoints)
    return replace(cli, **values)


def resolve_deploy_bimanual_config(args: DeployArgs, training_args: Any) -> BimanualRealRobotEnvConfig:
    """``resolve_deploy_robot_config`` for two arms: ``arm_names`` and ``max_episode_steps`` from
    the training run's two-arm config, every other contract field from the training run's config
    of each arm, connection fields from ``--real_robot_bimanual.*``. Runs without a
    ``real_robot_bimanual`` config fall back to the CLI config entirely (with a warning)."""
    cli = args.real_robot_bimanual
    trained = getattr(training_args, "real_robot_bimanual", None)
    if trained is None:
        _LOGGER.warning(
            "training config has no real_robot_bimanual section; using the "
            "--real_robot_bimanual.* CLI config as is, so action/observation contract is NOT verified"
        )
        return cli
    trained = replace(trained, **{arm: replace(getattr(trained, arm),
                                              **_training_geometry(getattr(trained, arm)))
                                 for arm in _BIMANUAL_ARMS})
    conflicts = _contract_conflicts(cli, trained, BimanualRealRobotEnvConfig(), _BIMANUAL_CONTRACT_FIELDS)
    arm_defaults = RealRobotEnvConfig()
    for arm in _BIMANUAL_ARMS:
        conflicts += _contract_conflicts(
            getattr(cli, arm), getattr(trained, arm), arm_defaults, _BIMANUAL_ARM_CONTRACT_FIELDS, f"{arm}."
        )
    if conflicts:
        raise ValueError(
            "--real_robot_bimanual.* contradicts the training run's real_robot_bimanual config; "
            "the training run is the source of truth for these fields (drop them from the CLI):\n  "
            + "\n  ".join(conflicts)
        )
    arms = {
        arm: replace(
            getattr(cli, arm),
            **{name: getattr(getattr(trained, arm), name) for name in _BIMANUAL_ARM_CONTRACT_FIELDS},
        )
        for arm in _BIMANUAL_ARMS
    }
    values = {name: getattr(trained, name) for name in _BIMANUAL_CONTRACT_FIELDS}
    if cli.task.classifier_checkpoints:
        values["task"] = replace(trained.task, classifier_checkpoints=cli.task.classifier_checkpoints)
    return replace(cli, **arms, **values)


def build_deploy_env(args: DeployArgs, training_args: Any):
    """The ``args.env_backend`` env with the observation layout the policy was trained on."""
    import dataclasses

    from rl_garden.envs.backend_registry import EnvRequest, make_training_envs
    from rl_garden.observations import ObservationConfig

    from real_world_rl.training.runtime_paths import resolve_control_frequency
    if training_args is not None: resolve_control_frequency(training_args)
    if args.env_backend == _BIMANUAL_BACKEND:
        import real_world_rl.envs.backends.real_robot_bimanual  # noqa: F401  (registers the backend)

        robot_config = resolve_deploy_bimanual_config(args, training_args)
    else:
        import real_world_rl.envs.backends.real_robot  # noqa: F401  (registers the backend)

        robot_config = resolve_deploy_robot_config(args, training_args)
    observation = getattr(training_args, "obs", None) or ObservationConfig()
    request = EnvRequest(
        env_id=getattr(training_args, "env_id", "real_robot"),
        num_envs=1,
        # The backend cannot stack frames; the stacking wrapper below does.
        observation=dataclasses.replace(observation, frame_stack=1),
        control_mode="",
        render_mode="rgb_array",
        seed=args.seed,
        num_eval_envs=1,
        create_eval_env=False,
        backend_config=robot_config,
    )
    from types import SimpleNamespace
    from real_world_rl.training.args import HilEnvArgs
    from real_world_rl.training.env_building import (
        compose_hil_wrappers, motion_teleop_kwargs, resolve_spacemouse_mode,
    )
    values = dataclasses.asdict(HilEnvArgs())
    if training_args is not None: values.update(vars(training_args))
    values.update(obs=observation, env_backend=args.env_backend,
                  real_robot=robot_config if args.env_backend != _BIMANUAL_BACKEND else args.real_robot,
                  real_robot_bimanual=robot_config if args.env_backend == _BIMANUAL_BACKEND else args.real_robot_bimanual)
    saved_experiment = _saved_experiment(args)
    saved_spec = (saved_experiment or {}).get("experiment", {})
    sources = (saved_experiment or {}).get("provenance", saved_spec).get("sources", {})
    if sources:
        classifier = sources.get("classifier")
        values["classifier_checkpoint"] = classifier.get("path") if classifier else None
        for key in ("classifier_threshold", "classifier_image_keys", "reward_from_teleop_events",
                    "teleop_success_reward", "use_grasp_penalty", "grasp_penalty_value",
                    "teleop_selected_parts", "teleop_idle_behavior"):
            if key in sources: values[key] = sources[key]
        values["convert_obs_rotation"] = saved_spec.get("observation", {}).get("convert_obs_rotation", False)
    if args.classifier_checkpoint is not None: values["classifier_checkpoint"] = args.classifier_checkpoint
    if args.teleop_device is not None: values["teleop_device"] = args.teleop_device
    if args.spacemouse_mode is not None: values["spacemouse_mode"] = args.spacemouse_mode
    if args.teleop_event_device is not None: values["teleop_event_device"] = args.teleop_event_device
    wrapper_args = SimpleNamespace(**values)
    if getattr(training_args, "real_robot", None) is None and args.env_backend == _BACKEND:
        # No saved env config: finalize the CLI config as a new run would.
        resolve_spacemouse_mode(wrapper_args, request)
    elif values["spacemouse_mode"] == "serl":
        motion_teleop_kwargs(wrapper_args)
        if args.spacemouse_mode == "serl" and robot_config.cartesian_semantics == "current":
            raise ValueError("spacemouse serl override conflicts with saved cartesian_semantics='current'")
    robot_config = request.backend_config
    env, _ = make_training_envs(args.env_backend, request)
    task_active = robot_config.task.name != "none"
    # A task deployment follows the same source and intervention composition as evaluation.
    env = compose_hil_wrappers(wrapper_args, env, robot_config,
                              enable_teleop=args.enable_teleop and (task_active or args.teleop_device is not None),
                              source_config={} if saved_spec.get("sources") == {} else None)
    from real_world_rl.action_contract import validate_experiment
    try:
        validate_experiment(env, saved_experiment)
    except BaseException:
        from real_world_rl.training.evaluation import close_env
        close_env(env)
        raise
    return env


def _resolve_build_agent(algo: str):
    """The online registry entry's ``build_agent``; algorithms registered without one fall
    back to ``build_<algo>`` in their in-tree entrypoints module. ``real_world_bc`` builds with
    ``build_real_world_bc``."""
    import importlib

    if algo == _REAL_WORLD_BC:
        from real_world_rl.training.entrypoints.bc import build_real_world_bc

        return build_real_world_bc
    from rl_garden.training.online.registry import registry

    registry.discover()
    entry = registry.entries().get(algo)
    if entry is not None and entry.build_agent is not None:
        return entry.build_agent
    module = importlib.import_module(f"rl_garden.training.online.entrypoints.{algo}")
    return getattr(module, f"build_{algo}")


def _gripper_count_kwargs(algo: str, env: Any) -> dict:
    """``num_grippers`` for the builders in ``_GRIPPER_COUNT_KEYWORD_ALGOS`` from ``env``'s action
    contract, as hil_serl builds them; empty for one gripper (the builders' default) and for
    every other algorithm."""
    if algo not in _GRIPPER_COUNT_KEYWORD_ALGOS:
        return {}
    from real_world_rl.action_contract import env_num_grippers

    num_grippers = env_num_grippers(env)
    return {} if num_grippers == 1 else {"num_grippers": num_grippers}


def restore_policy(args: DeployArgs, training_args: Any, env: Any):
    """Rebuild the ``<algo>`` agent on ``env``'s spaces and load the checkpoint weights."""
    from rl_garden.common import seed_everything

    seed_everything(args.seed)
    build_agent = _resolve_build_agent(args.algo)
    from real_world_rl.action_contract import env_num_grippers
    if args.algo == "rlpd_hybrid" and env_num_grippers(env) == 0:
        from real_world_rl.training.entrypoints.hil_serl import build_hil_agent
        build_agent = build_hil_agent
    agent = build_agent(
        training_args, env, None, logger=None, checkpoint_dir=None, **_gripper_count_kwargs(args.algo, env)
    )
    from real_world_rl.checkpoint_contract import load_with_action_contract
    load_with_action_contract(agent, str(_checkpoint_file(args)), load_replay_buffer=False, load_optimizers=False)
    # Online builders construct on device "auto"; inference runs where --device says.
    policy = agent.policy.to(torch.device(args.device))
    return _compose_with_base_provider(args, agent, policy, env)


def _compose_with_base_provider(args: DeployArgs, agent: Any, policy: Any, env: Any) -> Any:
    """A residual agent's policy acts on top of its base provider: rebuild the provider from
    ``--base-provider`` or the spec the checkpoint recorded. Other policies pass through."""
    from real_world_rl.algorithms.residual import ResidualMixin

    if not isinstance(agent, ResidualMixin):
        return policy
    from real_world_rl.policies.base_provider import (
        ResidualPolicyWithBase,
        make_base_provider,
    )

    provider = make_base_provider(
        args.base_provider or agent.base_provider_spec,
        device=args.device,
        observation_space=env.single_observation_space,
        action_space=env.single_action_space,
    )
    return ResidualPolicyWithBase(policy, provider)


# -- episode loop ---------------------------------------------------------------------


def _find_robot(env: Any) -> Any:
    """``env.robot`` through any wrappers (gymnasium wrappers do not forward attributes)."""
    while env is not None:
        robot = getattr(env, "robot", None)
        if robot is not None:
            return robot
        env = getattr(env, "env", None)
    return None


def _health_ok(env: Any) -> Optional[bool]:
    robot = _find_robot(env)
    if robot is None:
        return None
    # A two-arm env's ``robot`` maps arm name to robot; healthy means every arm is.
    robots = robot.values() if isinstance(robot, Mapping) else (robot,)
    return all(bool(each.health().ok) for each in robots)


def _obs_to_device(obs: Any, device: torch.device) -> Any:
    if isinstance(obs, dict):
        return {key: value.to(device) for key, value in obs.items()}
    return obs.to(device)


def _env_device(obs: Any) -> torch.device:
    sample = next(iter(obs.values())) if isinstance(obs, dict) else obs
    return sample.device


def _run_episode(args: DeployArgs, env: Any, policy: Any, device: torch.device, seed, *, failed_outcomes=None) -> EpisodeResult:
    from real_world_rl.policies.base_provider import ResidualPolicyWithBase

    if isinstance(policy, ResidualPolicyWithBase):
        policy.reset_episode()
    obs, _ = env.reset(seed=seed)
    from real_world_rl.metrics import EpisodeOutcomeAccumulator

    accumulator = EpisodeOutcomeAccumulator()
    steps, episode_return, truncated = 0, 0.0, False
    rtts: list[float] = []
    try:
        while True:
            # Same call as ActorLoop._predict.
            with torch.no_grad():
                action = policy.predict(_obs_to_device(obs, device), deterministic=args.deterministic)
            obs, reward, terminated, env_truncated, info = env.step(action.to(_env_device(obs)))
            accumulator.observe(info)
            steps += 1
            episode_return += float(torch.as_tensor(reward).sum())
            rtt = info.get("rtt_ms") if isinstance(info, Mapping) else None
            if rtt is not None:
                rtts.append(float(rtt))
            hit_cap = args.max_steps_per_episode is not None and steps >= args.max_steps_per_episode
            if bool(torch.as_tensor(terminated).any()) or bool(torch.as_tensor(env_truncated).any()) or hit_cap:
                truncated = bool(torch.as_tensor(env_truncated).any()) or hit_cap
                break
    except BaseException:
        if failed_outcomes is not None:
            failed_outcomes.append(accumulator.finish(interrupted=True))
        raise
    return EpisodeResult(
        steps=steps,
        episode_return=episode_return,
        truncated=truncated,
        rtt_ms_mean=sum(rtts) / len(rtts) if rtts else None,
        health_ok=_health_ok(env),
        outcome=accumulator.finish(terminated=terminated, truncated=truncated),
    )


def _format_rtt(rtt_ms: Optional[float]) -> str:
    return "n/a" if rtt_ms is None else f"{rtt_ms:.1f}"


def run_deploy(args: DeployArgs, *, env: Any = None, policy: Any = None) -> list[EpisodeResult]:
    """Run ``args.episodes`` episodes; ``env``/``policy`` are injectable for tests.

    The env is always closed (which disconnects the robot), including on ``DeviceError``.
    """
    training_args = restore_training_args(args) if env is None or policy is None else None
    if env is None:
        env = build_deploy_env(args, training_args)
    results: list[EpisodeResult] = []
    failed_outcomes = []
    try:
        if policy is None:
            policy = restore_policy(args, training_args, env)
        policy.eval()
        device = torch.device(args.device)
        task = getattr(env, "task_config", None)
        episodes = task.eval_episodes if task is not None and task.eval_episodes is not None else args.episodes
        seed = task.eval_seed if task is not None and task.eval_seed is not None else args.seed
        for episode in range(episodes):
            result = _run_episode(args, env, policy, device, seed=seed if episode == 0 else None,
                                  failed_outcomes=failed_outcomes)
            results.append(result)
            print(
                f"[deploy] episode={episode} steps={result.steps} "
                f"return={result.episode_return:.3f} truncated={result.truncated} "
                f"rtt_ms={_format_rtt(result.rtt_ms_mean)} health_ok={result.health_ok}",
                flush=True,
            )
        _print_summary(results)
        return results
    except BaseException:
        if failed_outcomes:
            _print_summary(results, extra_outcomes=failed_outcomes)
        raise
    finally:
        from real_world_rl.training.evaluation import close_resources
        close_resources(env, getattr(policy, "provider", getattr(policy, "base_provider", None)))


def _print_summary(results: list[EpisodeResult], *, extra_outcomes=()) -> None:
    from real_world_rl.metrics import EpisodeMetrics

    metrics = EpisodeMetrics()
    for result in results:
        if result.outcome is not None:
            metrics.add(result.outcome)
    for outcome in extra_outcomes:
        metrics.add(outcome)
    outcome_summary = " ".join(f"{key}={value}" for key, value in metrics.summary().items())
    count = len(results)
    mean_steps = sum(r.steps for r in results) / count if count else 0.0
    mean_return = sum(r.episode_return for r in results) / count if count else 0.0
    rtts = [r.rtt_ms_mean for r in results if r.rtt_ms_mean is not None]
    all_healthy = all(r.health_ok is not False for r in results)
    print(
        f"[deploy] summary episodes={count} mean_steps={mean_steps:.1f} "
        f"mean_return={mean_return:.3f} rtt_ms={_format_rtt(sum(rtts) / len(rtts) if rtts else None)} "
        f"all_healthy={all_healthy} {outcome_summary}",
        flush=True,
    )


def main(args: DeployArgs, *, env: Any = None, policy: Any = None) -> int:
    """CLI wrapper: exit code 0, or ``DEVICE_EXIT_CODE`` (1) after printing a ``DeviceError``'s
    message and hint; usage errors stay 2 (tyro)."""
    from robot_infra.devices.errors import DeviceError

    try:
        run_deploy(args, env=env, policy=policy)
    except DeviceError as error:
        print(f"[deploy] device error: {error.message}", file=sys.stderr, flush=True)
        if error.hint:
            print(f"[deploy] hint: {error.hint}", file=sys.stderr, flush=True)
        return DEVICE_EXIT_CODE
    return 0


def run_cli(argv: Optional[Sequence[str]] = None) -> int:
    """Console entry (``real-world-rl-deploy``): parses ``DeployArgs`` and returns :func:`main`'s
    exit code."""
    import tyro

    return main(tyro.cli(DeployArgs, args=argv))


if __name__ == "__main__":
    sys.exit(run_cli())
