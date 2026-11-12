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
from collections.abc import Mapping
from typing import Any, Optional, Sequence

_LOGGER = logging.getLogger(__name__)

_FRANKA_REAL_QUAT_SLICE = (3, 7)
_CARTESIAN_ARM_DIM = 6
_BIMANUAL_BACKEND = "real_robot_bimanual"
# Backends whose env cannot stack frames itself (apply_obs_chain stacks them, as deploy does).
_DEVICE_LAYER_BACKENDS = ("real_robot", _BIMANUAL_BACKEND)
# Teleop devices that drive one arm (the robot's teleop part, a single leader arm or a single
# cartesian device); --teleop-device bimanual drives the arms of a real_robot_bimanual env.
_SINGLE_ARM_TELEOP_DEVICES = (
    "robot", "leader_arm", "gello", "leader_cartesian", "spacemouse", "spacemouse_twist", "pico",
)
# The teleop devices that drive the follower through a CartesianTargetTracker.
_CARTESIAN_TRACKER_TELEOP_DEVICES = ("leader_cartesian", "spacemouse")
# --teleop-event-device values that were removed -> why, for old command lines and presets.
_REMOVED_TELEOP_EVENT_DEVICES = {
    "spacemouse": (
        "the SpaceMouse no longer provides the operator's labels (its button 0 takes over and "
        "hands back, button 1 toggles the gripper); use --teleop-device spacemouse "
        "--teleop-event-device stdin (or keyboard) for the s/f/r/Esc/q keys"
    ),
}
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

    resolve_spacemouse_mode(args)
    ensure_real_world_backend(args.env_backend)
    from real_world_rl.training.runtime_paths import resolve_control_frequency
    resolve_control_frequency(args)

    env_request = make_env_request(args, run_name)
    if args.env_backend in _DEVICE_LAYER_BACKENDS:
        # The backend cannot stack frames; apply_obs_chain stacks them (as deploy does).
        env_request.observation = dataclasses.replace(env_request.observation, frame_stack=1)
    return env_request


def _serl_mode(args) -> bool:
    mode = getattr(args, "spacemouse_mode", "tracker")
    if mode not in ("tracker", "serl"):
        raise ValueError("spacemouse_mode must be 'tracker' or 'serl'")
    if mode == "serl" and args.teleop_device != "spacemouse":
        raise ValueError("spacemouse_mode='serl' requires teleop_device='spacemouse'")
    return mode == "serl"


def resolve_spacemouse_mode(
    args: Any, env_request: Any = None, *, sources: Optional[Mapping[str, Any]] = None,
    explicit_cartesian_semantics: bool = False,
) -> None:
    """Select env semantics before construction, including device-free learners.

    RealRobotEnvConfig's default ``current`` is treated as unset. CLI/preset
    provenance and the standalone parsers distinguish an explicit conflict.
    Finalize this before serializing training or recording args.
    Franka's JSON mapping already preserves that distinction.
    """
    if not _serl_mode(args):
        return
    # Validate source options on learners too; no device is constructed here.
    motion_teleop_kwargs(args)
    backend = args.env_backend
    if backend not in ("real_robot", "franka_real"):
        raise ValueError("spacemouse serl mode requires real_robot or franka_real")
    config = getattr(env_request, "backend_config", None) if env_request is not None else None
    if config is None:
        config = getattr(args, backend)
    if backend == "franka_real":
        import json

        options = json.loads(config.env_kwargs_json or "{}")
        if options.get("cartesian_semantics") == "current":
            raise ValueError("spacemouse serl mode conflicts with explicit cartesian_semantics='current'")
        options["cartesian_semantics"] = "serl"
        resolved = dataclasses.replace(config, env_kwargs_json=json.dumps(options))
    else:
        explicit = explicit_cartesian_semantics
        from rl_garden.training.inspection import current_preflight, has_config_session

        if sources is None and has_config_session():
            sources = current_preflight().sources
        if sources is not None:
            source = sources.get("real_robot.cartesian_semantics")
            if source is not None:
                explicit = explicit or (source.get("kind") != "default" if isinstance(source, Mapping) else source.kind != "default")
        if explicit and config.cartesian_semantics == "current":
            raise ValueError("spacemouse serl mode conflicts with explicit cartesian_semantics='current'")
        resolved = dataclasses.replace(config, cartesian_semantics="serl")
    setattr(args, backend, resolved)
    if env_request is not None:
        env_request.backend_config = resolved


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
    - a device-layer robot or single-robot station yaml (a ``teleop:`` section): its ``teleop``
      ``options`` plus ``hold_s``, the kwargs of a local GELLO (``--teleop-device gello``);
    - a station yaml with ``robots:``: ``{robot name: its teleop options}``.
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
    if isinstance(data.get("robot"), dict):
        data = data["robot"]
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


def _single_arm_yaml_options(args, *, provenance=None) -> dict:
    """Read single-arm source options without changing the args or freezing input."""
    cached = getattr(args, "teleop_input", None)
    if args.teleop_device == "spacemouse" and cached is not None:
        import copy
        options = copy.deepcopy(cached["source_options"])
        table_file = _frozen_device_table_file(cached)
        if table_file is not None:
            options["device_table"] = table_file
        return options
    path = args.teleop_options_yaml
    form, data = _load_teleop_options_yaml(path)
    if form == _FLAT_OPTIONS:
        if getattr(args, "teleop_profile", None) is not None:
            raise ValueError("teleop_profile requires a station options YAML")
        if args.teleop_device == "spacemouse" and data and provenance is not None:
            from real_world_rl.common.provenance import portable_path
            provenance["options_file"] = portable_path(path)
        return data
    if args.teleop_device == "spacemouse":
        from real_world_rl.teleop.profiles import source_options
        from real_world_rl.common.provenance import portable_path
        if provenance is not None:
            provenance["options_file"] = portable_path(path)
        return source_options(path, getattr(args, "teleop_robot", None),
                              getattr(args, "teleop_profile", None), provenance=provenance)
    if args.teleop_device != _STATION_TELEOP_DEVICE:
        raise ValueError(
            f"--teleop-options-yaml {path} is a station yaml: its teleop: options "
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
            f"--teleop-options-yaml {path} is a station yaml: its teleop: options "
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


def motion_teleop_kwargs(args, *, source_options=None) -> dict:
    """Source arguments for ``--teleop-device``: the ones derived from the args (empty for devices
    configured by their defaults), then ``--teleop-options-yaml`` on top. ``bimanual`` takes
    ``bimanual_teleop_kwargs``."""
    serl = _serl_mode(args)
    diagnostics = {}
    axes, gain = getattr(args, "teleop_diagnostic_axes", "all"), getattr(args, "teleop_diagnostic_gain", 1.)
    if axes != "all" or gain != 1.:
        if args.teleop_device != "spacemouse":
            raise ValueError("input diagnostics require SpaceMouse")
        diagnostics = {"diagnostic_axes": axes, "diagnostic_gain": gain}
    _reject_single_arm_teleop_on_bimanual(args)
    if serl:
        from real_world_rl.teleop.spacemouse_serl import TRACKER_OPTIONS

        options = _single_arm_yaml_options(args) if source_options is None else source_options
        forbidden = sorted(set(options) & (TRACKER_OPTIONS | {"mode"}))
        if forbidden:
            raise ValueError(f"spacemouse serl mode does not accept tracker or mode keys in teleop_options_yaml: {forbidden}")
        if args.teleop_selected_parts:
            raise ValueError("spacemouse serl mode requires all arm and gripper action parts; omit teleop_selected_parts")
        return {"mode": "serl", **options, **diagnostics}
    if args.teleop_device == _BIMANUAL_TELEOP_DEVICE:
        return bimanual_teleop_kwargs(args)
    if args.teleop_device == "robot":
        derived = robot_teleop_kwargs(args)
    elif args.teleop_device == "leader_arm":
        derived = {**robot_teleop_kwargs(args), **teleop_onset_kwargs(args)}
    elif args.teleop_device == "gello":
        derived = teleop_onset_kwargs(args)
    elif args.teleop_device in _CARTESIAN_TRACKER_TELEOP_DEVICES:
        derived = cartesian_tracker_kwargs(args)
    else:
        derived = {}
    options = _single_arm_yaml_options(args) if source_options is None else source_options
    overridden = sorted(set(options) & set(_TRACKER_FLAGS)) if args.teleop_device in _CARTESIAN_TRACKER_TELEOP_DEVICES else []
    if overridden:
        flags = ", ".join(f"{key} -> {_TRACKER_FLAGS[key]}" for key in overridden)
        raise ValueError(
            f"--teleop-options-yaml {args.teleop_options_yaml} sets the command tracker ({overridden}), "
            f"which the experiment contract records from the flags; use the flags instead ({flags})"
        )
    return {**derived, **options, **diagnostics}


# leader_cartesian/spacemouse source kwarg -> the flag that sets it (the experiment's
# teleop_control block).
_TRACKER_FLAGS = {
    "lead_cap_m": "--teleop-lead-cap-m",
    "lead_cap_rad": "--teleop-lead-cap-rad",
    "target_speed_m_s": "--teleop-target-speed-m-s",
    "target_speed_rad_s": "--teleop-target-speed-rad-s",
}


def cartesian_tracker_kwargs(args) -> dict:
    """``leader_cartesian``/``spacemouse`` source arguments of the command tracker
    (``--teleop-lead-cap-*``, ``--teleop-target-speed-*``)."""
    return {
        "lead_cap_m": args.teleop_lead_cap_m,
        "lead_cap_rad": args.teleop_lead_cap_rad,
        "target_speed_m_s": args.teleop_target_speed_m_s,
        "target_speed_rad_s": args.teleop_target_speed_rad_s,
    }


def teleop_control_spec(args) -> Optional[dict]:
    """Compared control semantics from flags and resolved source options.

    Operator input rotation belongs to source provenance. Recorded actions are
    already in base coordinates, so recalibration does not change their meaning.
    Diagnostic filtering remains an explicit control semantic.
    """
    if _serl_mode(args):
        options = motion_teleop_kwargs(args)
        spec = {"version": "spacemouse_serl_v1", "mode": "serl",
                "motion_threshold": 0.001, "gripper": "serl_buttons"}
        if options.get("invert_axes"):
            spec["invert_axes"] = list(options["invert_axes"])
        from real_world_rl.teleop.input_frame import input_frame_contract
        spec.update({key: value for key, value in input_frame_contract(options).items()
                     if key not in ("input_rotation", "input_rotation_semantics")})
        return spec
    if args.teleop_device not in _CARTESIAN_TRACKER_TELEOP_DEVICES:
        return None
    from real_world_rl.teleop.cartesian_tracker import CartesianTrackerConfig

    spec = CartesianTrackerConfig(
        lead_cap_m=args.teleop_lead_cap_m,
        lead_cap_rad=args.teleop_lead_cap_rad,
        speed_m_s=args.teleop_target_speed_m_s,
        speed_rad_s=args.teleop_target_speed_rad_s,
    ).describe()
    if args.teleop_device == "spacemouse":
        from real_world_rl.teleop.input_frame import input_frame_contract
        spec.update({key: value for key, value in input_frame_contract(motion_teleop_kwargs(args)).items()
                     if key not in ("input_rotation", "input_rotation_semantics")})
    return spec


def teleop_wrapper_kwargs(args) -> dict:
    """``TeleopInterventionWrapper`` arguments: ``--teleop-device`` alone, or, with
    ``--teleop-event-device``, a composite taking motion from the former and intervention plus
    events from the latter."""
    removed = _REMOVED_TELEOP_EVENT_DEVICES.get(args.teleop_event_device)
    if removed is not None:
        raise ValueError(f"--teleop-event-device {args.teleop_event_device} was removed: {removed}")
    part_options = {}
    if args.teleop_selected_parts: part_options["selected_parts"] = args.teleop_selected_parts
    if args.teleop_idle_behavior != "hold": part_options["idle_behavior"] = args.teleop_idle_behavior
    if args.teleop_event_device is None:
        return {"device": args.teleop_device, **motion_teleop_kwargs(args), **part_options}
    return {
        "device": "composite",
        "motion": args.teleop_device,
        "events": args.teleop_event_device,
        **({"events_kwargs": {"path": args.teleop_events_file}}
           if args.teleop_event_device == "jsonl" else {}),
        "intervention_from": "motion" if _serl_mode(args) else "events",
        "motion_kwargs": motion_teleop_kwargs(args),
        **part_options,
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
    from rl_garden.envs.backend_registry import make_training_envs
    resolve_spacemouse_mode(args, env_request)
    env, _ = make_training_envs(args.env_backend, env_request)
    config = getattr(env_request, "backend_config", None)
    if config is None:
        config = getattr(args, "real_robot_bimanual" if args.env_backend == _BIMANUAL_BACKEND else "real_robot", None)
    return compose_hil_wrappers(args, env, config, enable_teleop=enable_teleop)


def compose_hil_wrappers(args, env, backend_config, *, enable_teleop=True, source_config=None):
    """One wrapper builder for train, record, evaluation and task deployment."""
    from rl_garden.envs.wrappers.fwbw_reset_free import FWBWResetFreeWrapper
    from real_world_rl.reward.outcome import ExplicitRewardClassifierWrapper
    from real_world_rl.envs.wrappers.grasp_penalty import GraspPenaltyWrapper
    from real_world_rl.envs.wrappers.obs_chain import apply_obs_chain
    try:
        env = apply_obs_chain(
            env,
            frame_stack=args.obs.frame_stack,
            # Reuses classifier_image_keys: the same "which keys are cameras" question the
            # classifier wrapper below answers.
            image_keys=args.classifier_image_keys,
            convert_obs_rotation=args.convert_obs_rotation,
            quat_slice=tcp_pose_quat_slice(args),
        )

        task = getattr(backend_config, "task", None)
        task_active = task is not None and task.name != "none"
        if task_active and task.success_source == "classifier":
            arm_names = tuple(getattr(backend_config, "arm_names", ("arm",)))
            if len(arm_names) > 1 and set(task.classifier_checkpoints) != set(arm_names):
                raise ValueError("parallel classifier task requires a classifier checkpoint for every arm")
            if len(arm_names) == 1 and not args.classifier_checkpoint and not task.classifier_checkpoints:
                raise ValueError("classifier task requires a classifier checkpoint")
        if args.classifier_checkpoint is not None:
            from rl_garden.common.utils import get_device
            from rl_garden.models.reward.success.model import load_classifier_fn

            classifier_fn = load_classifier_fn(
                args.classifier_checkpoint,
                env.single_observation_space,
                classifier_image_keys_for(args, env),
                device=get_device("auto"),
            )
            env = ExplicitRewardClassifierWrapper(env, classifier_fn, threshold=args.classifier_threshold,
                                                 terminate_on_success=not task_active)

        if task_active and task.classifier_checkpoints:
            from real_world_rl.reward.outcome import ArmClassifiersWrapper
            from rl_garden.models.reward.success.model import load_classifier_fn
            from rl_garden.common.utils import get_device
            arms = tuple(getattr(backend_config, "arm_names", ("arm",)))
            if set(task.classifier_checkpoints) != set(arms):
                raise ValueError("classifier_checkpoints must name every task arm")
            classifiers = {name: load_classifier_fn(path, env.single_observation_space,
                            task.classifier_image_keys.get(name) or classifier_image_keys_for(args, env),
                            device=get_device("auto")) for name, path in task.classifier_checkpoints.items()}
            env = ArmClassifiersWrapper(env, classifiers, args.classifier_threshold)

        if enable_teleop:
            from real_world_rl.envs.wrappers.intervention import TeleopInterventionWrapper

            # Validates the teleop source's action_mode/arm count/width against the env's action
            # contract (also rejects gripper_in_action=false, which GraspPenaltyWrapper below relies on).
            env = TeleopInterventionWrapper(env, **teleop_wrapper_kwargs(args))
            if args.reward_from_teleop_events:
                from real_world_rl.envs.wrappers.teleop_event_reward import (
                    TeleopEventRewardWrapper,
                )

                env = TeleopEventRewardWrapper(env, success_reward=args.teleop_success_reward,
                                              terminate_on_success=not task_active,
                                              terminate_on_failure=not task_active or task.terminate_on_failure)

        if args.use_grasp_penalty:
            from real_world_rl.action_contract import env_num_grippers
            if env_num_grippers(env) == 0:
                raise ValueError("grasp penalty requires a gripper in the action contract")
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

        from real_world_rl.tasks.outcome import apply_task_outcome
        from real_world_rl.envs.wrappers.experiment import apply_experiment_contract
        env = apply_task_outcome(env, backend_config)
        env = apply_experiment_contract(env, frame_stack=args.obs.frame_stack,
                                        convert_obs_rotation=args.convert_obs_rotation,
                                        source_config=reward_sources_spec(args, task) if source_config is None else source_config)
        if args.fwbw:
            env = FWBWResetFreeWrapper(env)

        return env
    except BaseException:
        from real_world_rl.training.evaluation import close_env
        close_env(env)
        raise


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


def ensure_real_world_backend(name):
    """Support source-checkout entrypoints without installed package metadata."""
    if name in ("franka_real", "real_robot", "real_robot_bimanual"):
        import importlib
        importlib.import_module(f"real_world_rl.envs.backends.{name}")


def reward_sources_spec(args, task=None):
    import hashlib
    from pathlib import Path
    def model(path):
        if path is None: return None
        file = Path(path)
        return {"path": str(path), "content_sha256": hashlib.sha256(file.read_bytes()).hexdigest() if file.is_file() else None}
    spec = {"classifier": model(args.classifier_checkpoint), "classifier_threshold": args.classifier_threshold,
            "arm_classifiers": {name: model(path) for name, path in (task.classifier_checkpoints.items() if task else ())},
            "classifier_image_keys": list(args.classifier_image_keys),
            "reward_from_teleop_events": args.reward_from_teleop_events,
            "teleop_success_reward": args.teleop_success_reward,
            "use_grasp_penalty": args.use_grasp_penalty, "grasp_penalty_value": args.grasp_penalty_value,
            "grasp_penalty_semantics": "critic_target_side_channel",
            "teleop_selected_parts": args.teleop_selected_parts, "teleop_idle_behavior": args.teleop_idle_behavior}
    # Compared, not provenance: action_contract.compared_experiment lifts it out of the sources.
    teleop_control = teleop_control_spec(args)
    if teleop_control is not None:
        spec["teleop_control"] = teleop_control
    if getattr(args, "teleop_input", None) is not None:
        spec["teleop_input"] = args.teleop_input
    return spec


def resolve_teleop_input(args):
    """Explicitly freeze a validated numeric input snapshot before config persistence."""
    if getattr(args, "teleop_device", None) != "spacemouse":
        return
    if getattr(args, "teleop_input", None) is not None:
        motion_teleop_kwargs(args)  # validate restored options without rereading files
        return
    provenance = {}
    options = _single_arm_yaml_options(args, provenance=provenance)
    motion_teleop_kwargs(args, source_options=options)
    options = _without_device_table(options, provenance)
    if options or provenance:
        import copy
        args.teleop_input = {**provenance, "source_options": copy.deepcopy(options)}


def _without_device_table(options, provenance) -> dict:
    """The SpaceMouse source options without the machine-local ``device_table`` path; a local
    table in effect is recorded as ``provenance["device_table"]`` (portable file, sha256 and
    entries), which is provenance only: decoding tables never enter ``teleop_control``."""
    from real_world_rl.devices.backends.teleops._spacemouse_hid import DEVICE_TABLE_OPTION, load_device_specs

    described = load_device_specs(options=options).provenance()
    if described is not None:
        provenance["device_table"] = described
    return {key: value for key, value in options.items() if key != DEVICE_TABLE_OPTION}


def _frozen_device_table_file(teleop_input) -> Optional[str]:
    """This machine's copy of the local device table a frozen SpaceMouse input was resolved
    with, or None: without it the device looks up the site table itself."""
    recorded = (teleop_input.get("device_table") or {}).get("file")
    if not recorded:
        return None
    from real_world_rl.devices.provenance import resolve_portable_path

    path = resolve_portable_path(recorded).resolve()
    return str(path) if path.is_file() else None


def build_offline_experiment_env(env, config, observation, saved_contract=None):
    """Build a space-only offline view without opening reward or teleop devices."""
    from types import SimpleNamespace
    from real_world_rl.envs.wrappers.obs_chain import apply_obs_chain
    from real_world_rl.envs.wrappers.experiment import apply_experiment_contract
    from real_world_rl.action_contract import demo_contract_of, validate_transitions_against_contract
    import gymnasium as gym
    spec = (saved_contract or {}).get("experiment", {})
    observed = spec.get("observation", {})
    converted = bool(observed.get("convert_obs_rotation", False))
    backend = "real_robot_bimanual" if hasattr(config, "arm_names") else "real_robot" if hasattr(config, "state_fields") else "franka_real"
    namespace = SimpleNamespace(env_backend=backend, real_robot=config, real_robot_bimanual=config)
    env = apply_obs_chain(env, frame_stack=observation.frame_stack, image_keys=observation.image_keys,
                          convert_obs_rotation=converted, quat_slice=tcp_pose_quat_slice(namespace))
    env = apply_experiment_contract(env, frame_stack=observation.frame_stack,
                                    convert_obs_rotation=converted, source_config=spec.get("sources", {}))
    expected = demo_contract_of(env)
    if saved_contract and "experiment" in saved_contract and "observation" not in saved_contract["experiment"]:
        from real_world_rl.action_contract import _requires_experiment_metadata
        import warnings
        if _requires_experiment_metadata(expected["experiment"]):
            raise ValueError("legacy metadata cannot verify task/control/crop/depth observation processing")
        from real_world_rl.action_contract import TELEOP_CONTROL_KEY
        warnings.warn("legacy raw contract has no final observation/source metadata; using configured default observation view", stacklevel=2)
        # Legacy metadata predates teleop_control: borrowing it would hide that mismatch.
        borrowed_sources = {key: value for key, value in (expected["experiment"].get("sources") or {}).items()
                            if key != TELEOP_CONTROL_KEY}
        saved_contract = {**saved_contract, "experiment": {**saved_contract["experiment"],
                           "sources": borrowed_sources,
                           "observation": expected["experiment"]["observation"]}}
    validate_transitions_against_contract([], expected, source="offline demos", saved=saved_contract)
    class OfflineSpaceOnly(gym.Wrapper):
        def __getattr__(self, name): return getattr(self.env, name)
        def reset(self, **kwargs): raise RuntimeError("offline environment supplies spaces only; deploy for live evaluation")
        def step(self, action): raise RuntimeError("offline environment supplies spaces only; deploy for live evaluation")
    return OfflineSpaceOnly(env)
