"""Env construction shared by every HIL-style real-world entrypoint (hil_serl,
residual, SiLRI): the wrapper stack that was ``hil_serl._build_env``.

``build_hil_env`` composes, innermost first:

1. temporal observation stacking (optional) and rotvec observation conversion (optional);
2. reward classifier (``--classifier-checkpoint``, optional): replaces the env reward with the
   classifier's and terminates on success;
3. human intervention (teleop enabled only): writes ``info["teleop_events"]``;
4. teleop-event reward (``--reward-from-teleop-events``, teleop enabled only): reads those
   events, so it sits right outside the intervention wrapper; it only raises the classifier's
   reward and ``terminated``, never lowers them;
5. gripper-flip penalty (``--use-grasp-penalty``, optional): needs ``info["intervene_action"]``;
6. reset-free forward/backward (``--fwbw``, outermost, per ``FWBWActorLoop``/
   ``FWBWResetFreeWrapper``'s documented requirement).

``args`` is any Args that carries the fields of ``HilEnvArgs`` plus ``RealWorldFrankaArgs``
and the env-request fields of ``rl_garden.common.env_args``.

``build_hil_env(args, role=..., enable_teleop=...)`` is the entry point: the actor and eval
processes keep the human-intervention wrapper (and so open the teleop device), the learner passes
``enable_teleop=False`` because it only needs the env's spaces and action contract and must not
open a HID/ZMQ device on the training box. ``hil_serl.py``, ``residual_hil_serl.py`` and
``silri.py`` all call it; ``compose_hil_env`` is the wrapper composition alone, for callers
that already hold an env request.
"""
from __future__ import annotations

import dataclasses
import logging
from typing import Any, Optional, Sequence

_LOGGER = logging.getLogger(__name__)

_FRANKA_REAL_QUAT_SLICE = (3, 7)
_CARTESIAN_ARM_DIM = 6
_BIMANUAL_BACKEND = "real_robot_bimanual"
# Backends whose env cannot stack frames itself (apply_obs_chain stacks them, as deploy does).
_ROBOT_INFRA_BACKENDS = ("real_robot", _BIMANUAL_BACKEND)
# Teleop devices that drive one arm (the robot's teleop part, a single leader arm or a single
# cartesian device); --teleop-device bimanual drives the arms of a real_robot_bimanual env.
_SINGLE_ARM_TELEOP_DEVICES = ("robot", "leader_arm", "gello", "leader_cartesian", "spacemouse", "pico")
_BIMANUAL_TELEOP_DEVICE = "bimanual"
# The forms --teleop-options-yaml may take (see teleop_options_from_yaml).
_FLAT_OPTIONS = "flat"
_STATION = "station"
_MULTI_ROBOT_STATION = "multi_robot_station"
# A station yaml's teleop: options are the device options of the local gello source.
_STATION_TELEOP_DEVICE = "gello"


def tcp_pose_quat_slice(args) -> tuple[int, int] | list[tuple[int, int]] | None:
    """FrankaRealEnv's state always starts with tcp_pose; real_robot's follows state_fields;
    real_robot_bimanual has one slice per arm (``None`` without tcp_pose)."""
    if args.env_backend == _BIMANUAL_BACKEND:
        from real_world_rl.envs.wrappers.obs_chain import tcp_pose_quat_slices

        return tcp_pose_quat_slices(args.real_robot_bimanual) or None
    if args.env_backend != "real_robot":
        return _FRANKA_REAL_QUAT_SLICE
    from real_world_rl.envs.wrappers.obs_chain import tcp_pose_quat_slice as _real_robot_slice

    return _real_robot_slice(args.real_robot)


def hil_env_request(args, run_name: str):
    from rl_garden.common.env_args import make_env_request

    env_request = make_env_request(args, run_name)
    if args.env_backend in _ROBOT_INFRA_BACKENDS:
        # The backend cannot stack frames; apply_obs_chain stacks them (as deploy does).
        env_request.observation = dataclasses.replace(env_request.observation, frame_stack=1)
    return env_request


def _reject_single_arm_teleop_on_bimanual(args) -> None:
    if args.env_backend == _BIMANUAL_BACKEND and args.teleop_device in _SINGLE_ARM_TELEOP_DEVICES:
        raise ValueError(
            f"--teleop-device {args.teleop_device} drives one arm; --env-backend "
            f"{_BIMANUAL_BACKEND} needs a bimanual teleop source: use --teleop-device "
            f"{_BIMANUAL_TELEOP_DEVICE} (one --teleop-arm-device per arm)"
        )


def robot_teleop_kwargs(args) -> dict:
    """``TeleopPartSource`` arguments for ``--teleop-device robot`` (and ``leader_arm``).

    A ``RemoteRobot`` only learns its teleop part when it connects, so the device is declared
    here (mode + gripper; the arm width follows the env) and checked against the real part on
    connect. A local robot's teleop part is checked at construction either way.
    """
    _reject_single_arm_teleop_on_bimanual(args)
    if args.env_backend != "real_robot":
        raise ValueError(f"--teleop-device {args.teleop_device} needs --env-backend real_robot")
    real_robot = args.real_robot
    kwargs = {"joint_limits": real_robot.joint_limits, "joint_scale_rad": real_robot.joint_scale_rad}
    teleop_contract = _declared_robot_teleop(args, real_robot.num_joints)
    if teleop_contract is not None:
        kwargs["teleop_contract"] = teleop_contract
    return kwargs


def _declared_robot_teleop(args, num_joints: int) -> Optional[tuple[str, int, bool]]:
    """The ``teleop_contract`` from ``--robot-teleop-action-mode``/``--robot-teleop-has-gripper``
    (``None`` when no mode is declared)."""
    mode = args.robot_teleop_action_mode
    if mode is None:
        return None
    arm_dim = _CARTESIAN_ARM_DIM if mode == "cartesian_delta" else num_joints
    return (mode, arm_dim, args.robot_teleop_has_gripper)


def teleop_onset_kwargs(args) -> dict:
    """Leader-arm onset arguments (``--teleop-onset`` and its tolerances)."""
    return {
        "onset": args.teleop_onset,
        "arm_tolerance_rad": args.teleop_arm_tolerance_rad,
        "ramp_step_rad": args.teleop_ramp_step_rad,
    }


def bimanual_teleop_kwargs(args) -> dict:
    """``BimanualTeleopSource.from_env`` arguments for ``--teleop-device bimanual``.

    Every arm's child is a ``--teleop-arm-device`` source: ``robot``/``leader_arm`` children get
    the ``--robot-teleop-action-mode`` declaration, ``leader_arm``/``gello`` children the onset
    flags. ``--teleop-options-yaml`` adds per-arm kwargs (a section per arm name, or a station
    yaml's ``robots:``) and common kwargs (its other keys; ``intervention_from`` is the
    source's own ``"any"``/``"all"``).
    """
    if args.env_backend != _BIMANUAL_BACKEND:
        raise ValueError(
            f"--teleop-device {_BIMANUAL_TELEOP_DEVICE} needs --env-backend {_BIMANUAL_BACKEND}"
        )
    arm_device = args.teleop_arm_device
    common: dict = {}
    if arm_device in ("robot", "leader_arm"):
        teleop_contract = _declared_robot_teleop(args, args.real_robot_bimanual.left.num_joints)
        if teleop_contract is not None:
            common["teleop_contract"] = teleop_contract
    if arm_device in ("leader_arm", "gello"):
        common.update(teleop_onset_kwargs(args))
    per_arm, yaml_common = _bimanual_yaml_options(args)
    common.update(yaml_common)
    kwargs = {"device_per_arm": arm_device, "per_arm_kwargs": per_arm}
    if "intervention_from" in common:
        kwargs["intervention_from"] = common.pop("intervention_from")
    kwargs["common_kwargs"] = common
    return kwargs


def teleop_options_from_yaml(path: Optional[str]) -> dict:
    """The source kwargs in ``--teleop-options-yaml`` (empty without a file, or for an empty file).

    The file is one of:

    - a mapping of source kwargs, returned as is (per-arm sections keyed by arm name included);
    - a robot_infra robot or single-robot station yaml (a ``teleop:`` section): its ``teleop``
      ``options`` plus ``hold_s``, the kwargs of a local GELLO (``--teleop-device gello``);
    - a robot_infra station yaml with ``robots:``: ``{robot name: its teleop options}``.
    """
    form, data = _load_teleop_options_yaml(path)
    if form == _STATION:
        return _station_teleop_options(path, str(data.get("name") or "robot"), data)
    if form == _MULTI_ROBOT_STATION:
        return {name: _station_teleop_options(path, name, robot) for name, robot in data["robots"].items()}
    return data


def _load_teleop_options_yaml(path: Optional[str]) -> tuple[str, dict]:
    """``(form, raw mapping)`` of ``--teleop-options-yaml`` (the forms of ``teleop_options_from_yaml``)."""
    if path is None:
        return _FLAT_OPTIONS, {}
    import yaml

    with open(path, encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    if data is None:
        return _FLAT_OPTIONS, {}
    if not isinstance(data, dict):
        raise ValueError(f"--teleop-options-yaml {path} must hold a mapping, got {type(data).__name__}")
    robots = data.get("robots")
    if isinstance(robots, dict):
        if not robots:
            raise ValueError(f"--teleop-options-yaml {path}: the station's robots: mapping is empty")
        return _MULTI_ROBOT_STATION, {**data, "robots": {str(name): robot for name, robot in robots.items()}}
    if isinstance(data.get("teleop"), dict):
        return _STATION, data
    return _FLAT_OPTIONS, data


def _station_teleop_options(path: Optional[str], robot_name: str, robot: Any) -> dict:
    """A station robot's ``teleop:`` section as local GELLO source kwargs: ``options`` plus
    ``hold_s``."""
    teleop = robot.get("teleop") if isinstance(robot, dict) else None
    if not isinstance(teleop, dict):
        raise ValueError(f"--teleop-options-yaml {path}: station robot {robot_name!r} has no teleop: section")
    backend = teleop.get("backend")
    if backend != _STATION_TELEOP_DEVICE:
        raise ValueError(
            f"--teleop-options-yaml {path}: station robot {robot_name!r} has a {backend!r} teleop; "
            f"only a {_STATION_TELEOP_DEVICE!r} section configures a local GELLO"
        )
    options = dict(teleop.get("options") or {})
    if "hold_s" in teleop:
        options["hold_s"] = teleop["hold_s"]
    return options


def _single_arm_yaml_options(args) -> dict:
    """``--teleop-options-yaml`` kwargs for a single-arm ``--teleop-device``."""
    path = args.teleop_options_yaml
    form, data = _load_teleop_options_yaml(path)
    if form == _FLAT_OPTIONS:
        return data
    if args.teleop_device != _STATION_TELEOP_DEVICE:
        raise ValueError(
            f"--teleop-options-yaml {path} is a robot_infra station yaml: its teleop: options "
            f"configure a local GELLO (--teleop-device {_STATION_TELEOP_DEVICE}), not "
            f"--teleop-device {args.teleop_device}"
        )
    if form == _STATION:
        return _station_teleop_options(path, str(data.get("name") or "robot"), data)
    robots = data["robots"]
    if len(robots) != 1:
        raise ValueError(
            f"--teleop-options-yaml {path} lists robots {sorted(robots)}; --teleop-device "
            f"{_STATION_TELEOP_DEVICE} drives one arm (use --teleop-device {_BIMANUAL_TELEOP_DEVICE} "
            f"--teleop-arm-device {_STATION_TELEOP_DEVICE} for two)"
        )
    ((name, robot),) = robots.items()
    return _station_teleop_options(path, name, robot)


def _bimanual_yaml_options(args) -> tuple[dict, dict]:
    """``(per-arm kwargs, common kwargs)`` from ``--teleop-options-yaml`` for
    ``--teleop-device bimanual``: a mapping's sections named after the env's arms are per-arm,
    its other keys common; a station's ``robots:`` named after the arms are per-arm (other
    robots are ignored)."""
    path = args.teleop_options_yaml
    arm_names = [str(arm) for arm in args.real_robot_bimanual.arm_names]
    form, data = _load_teleop_options_yaml(path)
    if form == _FLAT_OPTIONS:
        per_arm = {}
        for arm in arm_names:
            if arm not in data:
                continue
            if not isinstance(data[arm], dict):
                raise ValueError(
                    f"--teleop-options-yaml {path}: the {arm!r} section must be a mapping of that "
                    f"arm's source kwargs, got {type(data[arm]).__name__}"
                )
            per_arm[arm] = dict(data[arm])
        return per_arm, {key: value for key, value in data.items() if key not in arm_names}
    if args.teleop_arm_device != _STATION_TELEOP_DEVICE:
        raise ValueError(
            f"--teleop-options-yaml {path} is a robot_infra station yaml: its teleop: options "
            f"configure local GELLOs (--teleop-arm-device {_STATION_TELEOP_DEVICE}); with "
            f"--teleop-arm-device {args.teleop_arm_device} each arm's daemon reads its own"
        )
    if form == _STATION:
        raise ValueError(
            f"--teleop-options-yaml {path} describes one robot; --teleop-device "
            f"{_BIMANUAL_TELEOP_DEVICE} needs one GELLO per arm (a station with robots: "
            f"{arm_names}, or per-arm sections)"
        )
    robots = data["robots"]
    missing = [arm for arm in arm_names if arm not in robots]
    if missing:
        raise ValueError(
            f"--teleop-options-yaml {path}: the station's robots {sorted(robots)} do not include "
            f"the env's arms {missing} (arm_names {arm_names})"
        )
    return {arm: _station_teleop_options(path, arm, robots[arm]) for arm in arm_names}, {}


def motion_teleop_kwargs(args) -> dict:
    """Source arguments for ``--teleop-device``: the ones derived from the args (empty for devices
    configured by their defaults), then ``--teleop-options-yaml`` on top. ``bimanual`` takes
    ``bimanual_teleop_kwargs``."""
    _reject_single_arm_teleop_on_bimanual(args)
    if args.teleop_device == _BIMANUAL_TELEOP_DEVICE:
        return bimanual_teleop_kwargs(args)
    if args.teleop_device == "robot":
        derived = robot_teleop_kwargs(args)
    elif args.teleop_device == "leader_arm":
        derived = {**robot_teleop_kwargs(args), **teleop_onset_kwargs(args)}
    elif args.teleop_device == "gello":
        derived = teleop_onset_kwargs(args)
    else:
        derived = {}
    return {**derived, **_single_arm_yaml_options(args)}


def teleop_wrapper_kwargs(args) -> dict:
    """``TeleopInterventionWrapper`` arguments: ``--teleop-device`` alone, or, with
    ``--teleop-event-device``, a composite taking motion from the former and intervention plus
    events from the latter."""
    if args.teleop_event_device is None:
        return {"device": args.teleop_device, **motion_teleop_kwargs(args)}
    return {
        "device": "composite",
        "motion": args.teleop_device,
        "events": args.teleop_event_device,
        "intervention_from": "events",
        "motion_kwargs": motion_teleop_kwargs(args),
    }


def classifier_image_keys_for(args, env) -> Sequence[str]:
    """The observation keys a reward classifier reads: ``--classifier-image-keys``, or every key
    of ``env``'s observation space but ``state``.

    FrankaRealEnv's camera keys are whatever the user configured (``cfg.camera_keys``), not
    necessarily ``rgb*``/``depth*`` prefixed, so every non-``state`` key counts as an image.
    """
    return args.classifier_image_keys or tuple(
        key for key in env.single_observation_space.spaces if key != "state"
    )


_NO_REWARD_SOURCE_WARNING = (
    "%s %s: no reward source. Neither --classifier-checkpoint nor --reward-from-teleop-events is "
    "set, so every transition carries the env's own reward (0 on the real robot) and episodes "
    "end only on truncation (step cap or episode-end button): nothing tells the agent what success "
    "is. Pass --reward-from-teleop-events (the operator's success/failure keys) and/or "
    "--classifier-checkpoint."
)


def warn_if_no_reward_source(args, method: str) -> None:
    """Logs a warning when ``args`` (actor or learner role) gives the env neither a reward
    classifier nor teleop-event rewards. Never raises: a run may want neither, e.g. a dry run."""
    if args.classifier_checkpoint is None and not args.reward_from_teleop_events:
        _LOGGER.warning(_NO_REWARD_SOURCE_WARNING, method, args.role)


def compose_hil_env(args, env_request, *, enable_teleop: bool = True):
    """Builds the backend env for ``env_request`` and applies the wrapper stack
    described in the module docstring. ``enable_teleop=False`` skips the
    intervention wrapper (and so never opens a teleop device or checks the
    teleop source against the env's action contract)."""
    from rl_garden.envs.backend_registry import make_training_envs
    from rl_garden.envs.wrappers.fwbw_reset_free import FWBWResetFreeWrapper
    from rl_garden.envs.wrappers.reward_classifier import RewardClassifierWrapper
    from real_world_rl.envs.wrappers.grasp_penalty import GraspPenaltyWrapper
    from real_world_rl.envs.wrappers.obs_chain import apply_obs_chain

    env, _ = make_training_envs(args.env_backend, env_request)

    env = apply_obs_chain(
        env,
        frame_stack=args.obs.frame_stack,
        # Reuses classifier_image_keys: the same "which keys are cameras" question the
        # classifier wrapper below answers.
        image_keys=args.classifier_image_keys,
        convert_obs_rotation=args.convert_obs_rotation,
        quat_slice=tcp_pose_quat_slice(args),
    )

    if args.classifier_checkpoint is not None:
        from rl_garden.common.utils import get_device
        from rl_garden.models.reward.success.model import load_classifier_fn

        classifier_fn = load_classifier_fn(
            args.classifier_checkpoint,
            env.single_observation_space,
            classifier_image_keys_for(args, env),
            device=get_device("auto"),
        )
        env = RewardClassifierWrapper(env, classifier_fn, threshold=args.classifier_threshold)

    if enable_teleop:
        from real_world_rl.envs.wrappers.intervention import TeleopInterventionWrapper

        # Validates the teleop source's action_mode/arm count/width against the env's action
        # contract (also rejects gripper_in_action=false, which GraspPenaltyWrapper below relies on).
        env = TeleopInterventionWrapper(env, **teleop_wrapper_kwargs(args))
        if args.reward_from_teleop_events:
            from real_world_rl.envs.wrappers.teleop_event_reward import (
                TeleopEventRewardWrapper,
            )

            env = TeleopEventRewardWrapper(env, success_reward=args.teleop_success_reward)

    if args.use_grasp_penalty:
        from real_world_rl.action_contract import (
            get_env_action_contract,
            gripper_slice,
        )

        # Must wrap outside TeleopInterventionWrapper so
        # info["intervene_action"] (the actually-executed action under
        # human intervention) is already present when this runs. One gripper per
        # arm by the env's contract; a contract-less env has its gripper last.
        contract = get_env_action_contract(env)
        gripper_kwargs = {} if contract is None else {"gripper_slice": gripper_slice(contract)}
        env = GraspPenaltyWrapper(env, penalty=args.grasp_penalty_value, **gripper_kwargs)

    if args.fwbw:
        env = FWBWResetFreeWrapper(env)

    return env


def build_hil_env(
    args,
    *,
    role: str,
    enable_teleop: bool = True,
    run_name: Optional[str] = None,
) -> Any:
    """The HIL env for process ``role`` (``"actor"``, ``"learner"``, ``"eval"``).

    ``run_name`` names the env request (``real_world_<role>`` by default). The
    learner passes ``enable_teleop=False``: it only needs the env's spaces and
    action contract, and must not open a HID/ZMQ device on the training box.
    """
    env_request = hil_env_request(args, run_name or f"real_world_{role}")
    return compose_hil_env(args, env_request, enable_teleop=enable_teleop)
