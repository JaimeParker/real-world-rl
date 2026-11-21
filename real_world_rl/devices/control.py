"""Torch-free impedance settings supported by the arm control contract.

``ArmControlConfig``: franky tracker settings. Omitted settings keep the backend/SDK defaults.
Configuration is applied while IDLE and takes effect when the next servo tracker starts.

``SERL_COMPLIANCE_PARAMS`` and the ``serl_*`` helpers: the compliance parameters of SERL's
Cartesian impedance controller (``cartesian_controller: serl``), names, defaults and ranges of
serl_franka_controllers' ``cfg/compliance_param.cfg``, merged and clamped like dynamic_reconfigure.
``franka_load``: the station ``load:`` block (libfranka ``Robot::setLoad``).
"""
from __future__ import annotations

import dataclasses
import logging
import math
from dataclasses import dataclass
from typing import Any, Mapping

logger = logging.getLogger(__name__)

# What ``rwrl setup`` writes for a new franky arm (cartesian_controller: impedance). franky's own
# default stiffness (500 N/m) with a ~1 cm teleop lead pushes the TCP with ~5 N, below the joints'
# static friction, so the arm lags and drifts off the commanded direction. These approximate SERL's
# default Cartesian impedance (compliance_param.cfg) for small errors; the translational error clip
# bounds the force (2000 N/m x 1 cm = 20 N) however far the command leads. APPROXIMATE for rotation:
# franky's rotation error is the rotation vector theta*axis, SERL's is the quaternion vector part
# sin(theta/2)*axis ~ (theta/2)*axis. So SERL's 150 Nm/rad is franky's 75, SERL's clip 0.05 (on
# sin(theta/2)) is franky's 2*asin(0.05) ~ 0.10 rad, and the damping is given explicitly (SERL's
# 89 / 7, on the measured twist in both). Equal only for small angles; cartesian_controller: serl
# runs SERL's controller itself.
FRANKY_SETUP_CONTROL = {
    "translational_stiffness": 2000.0,
    "rotational_stiffness": 75.0,
    "cartesian_damping": [89.0, 89.0, 89.0, 7.0, 7.0, 7.0],
    "translational_error_clip": [0.01, 0.01, 0.01],
    "rotational_error_clip": [0.10, 0.10, 0.10],
}

# serl_franka_controllers @ 1f140ef, cfg/compliance_param.cfg: (name, default, min, max), .cfg order.
SERL_COMPLIANCE_PARAMS: tuple[tuple[str, float, float, float], ...] = (
    ("translational_stiffness", 2000.0, 0.0, 6000.0),
    ("translational_damping", 89.0, 0.0, 400.0),
    ("rotational_stiffness", 150.0, 0.0, 300.0),
    ("rotational_damping", 7.0, 0.0, 30.0),
    ("nullspace_stiffness", 0.2, 0.0, 100.0),
    ("joint1_nullspace_stiffness", 100.0, 0.0, 100.0),
    ("translational_clip_neg_x", 0.01, 0.0, 0.1),
    ("translational_clip_neg_y", 0.01, 0.0, 0.1),
    ("translational_clip_neg_z", 0.01, 0.0, 0.1),
    ("translational_clip_x", 0.01, 0.0, 0.1),
    ("translational_clip_y", 0.01, 0.0, 0.1),
    ("translational_clip_z", 0.01, 0.0, 0.1),
    ("rotational_clip_neg_x", 0.05, 0.0, 0.1),
    ("rotational_clip_neg_y", 0.05, 0.0, 0.1),
    ("rotational_clip_neg_z", 0.05, 0.0, 0.1),
    ("rotational_clip_x", 0.05, 0.0, 0.1),
    ("rotational_clip_y", 0.05, 0.0, 0.1),
    ("rotational_clip_z", 0.05, 0.0, 0.1),
    ("translational_Ki", 0.0, 0.0, 100.0),
    ("rotational_Ki", 0.0, 0.0, 100.0),
)
SERL_PARAM_NAMES = tuple(name for name, *_ in SERL_COMPLIANCE_PARAMS)
SERL_PARAM_DEFAULTS = {name: default for name, default, _, _ in SERL_COMPLIANCE_PARAMS}
SERL_PARAM_RANGES = {name: (low, high) for name, _, low, high in SERL_COMPLIANCE_PARAMS}


def serl_param_updates(updates: Mapping[str, Any] | None, *, where: str = "serl") -> tuple[dict[str, float], dict]:
    """A partial SERL compliance update, checked and clamped like dynamic_reconfigure.

    Returns ``(values, clamped)``: ``values`` holds every given key as a float clamped into its
    .cfg range, ``clamped`` maps each key whose value was outside to ``{"requested", "applied"}``
    (SERL's dynamic_reconfigure clamps silently; HIL-SERL's ram_insertion PRECISION_PARAM sends
    rotational clips of 0.5, which run as 0.1). Unknown keys and non-finite or non-numeric values
    raise ``ValueError`` and change nothing."""
    if updates is None:
        return {}, {}
    if not isinstance(updates, Mapping):
        raise ValueError(f"{where} must be a mapping of SERL compliance parameters, got {type(updates).__name__}")
    unknown = sorted(set(updates) - set(SERL_PARAM_NAMES))
    if unknown:
        raise ValueError(f"unknown SERL compliance parameters in {where}: {unknown}; known: {list(SERL_PARAM_NAMES)}")
    values: dict[str, float] = {}
    clamped: dict[str, dict[str, float]] = {}
    for name in SERL_PARAM_NAMES:
        if name not in updates:
            continue
        value = updates[name]
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError(f"{where}.{name} must be a finite number, got {value!r}")
        low, high = SERL_PARAM_RANGES[name]
        applied = min(max(float(value), low), high)
        if applied != float(value):
            clamped[name] = {"requested": float(value), "applied": applied}
        values[name] = applied
    if clamped:
        logger.warning("%s: SERL compliance parameters outside compliance_param.cfg ranges were clamped "
                       "(as dynamic_reconfigure does): %s", where, clamped)
    return values, clamped


def resolve_serl_params(*layers: Mapping[str, Any] | None) -> dict[str, float]:
    """The full parameter set: the .cfg defaults, then each partial layer in order (e.g. the
    station ``serl:`` block, then a task profile), clamped like dynamic_reconfigure."""
    params = dict(SERL_PARAM_DEFAULTS)
    for index, layer in enumerate(layers):
        params.update(serl_param_updates(layer, where=f"serl layer {index}")[0])
    return params


def validate_serl_profiles(profiles: Mapping[str, Any] | None, *, where: str = "serl_profiles") -> dict[str, dict]:
    """``{name: partial SERL compliance parameters}``; names are nonempty strings. Returns the
    clamped profiles. Warns when profiles set different keys: a profile is a partial update on top
    of the current parameters (HIL-SERL's update_param), so a key one profile sets and another
    omits keeps whatever was applied before, and the result depends on the switching order."""
    if profiles is None:
        return {}
    if not isinstance(profiles, Mapping):
        raise ValueError(f"{where} must map profile names to SERL compliance parameters")
    result = {}
    for name, params in profiles.items():
        if not isinstance(name, str) or not name.strip():
            raise ValueError(f"{where} profile names must be nonempty strings, got {name!r}")
        if not isinstance(params, Mapping) or not params:
            raise ValueError(f"{where}.{name} must be a nonempty mapping of SERL compliance parameters")
        result[name] = serl_param_updates(params, where=f"{where}.{name}")[0]
    differing = serl_profile_key_differences(result)
    if differing:
        logger.warning("%s: profiles set different SERL parameters %s; each profile is a partial update on "
                       "top of the current values, so the keys a profile omits keep the previous profile's "
                       "values and the switching order matters", where, differing)
    return result


def serl_profile_key_differences(profiles: Mapping[str, Mapping[str, Any]]) -> dict[str, list[str]]:
    """``{profile: keys some other profile sets but this one does not}`` (empty when all agree)."""
    all_keys = set().union(*(set(params) for params in profiles.values())) if profiles else set()
    return {name: sorted(all_keys - set(params)) for name, params in profiles.items() if all_keys - set(params)}


def serl_profile_update(profiles: Mapping[str, Any] | None, name: str) -> dict[str, float]:
    """The partial update profile ``name`` posts, as HIL-SERL's ``requests.post(url + "update_param",
    json=PROFILE)``: pass it to ``arm.update_serl_params``, which merges it into the current values
    (unset keys keep their previous values). An unknown name raises ``KeyError``."""
    profiles = validate_serl_profiles(profiles)
    if name not in profiles:
        raise KeyError(f"unknown serl profile '{name}'; defined: {sorted(profiles)}")
    return dict(profiles[name])


def apply_serl_profile(current: Mapping[str, float], profiles: Mapping[str, Any] | None, name: str) -> dict[str, float]:
    """The full parameter set after switching to profile ``name`` from ``current`` (what
    ``update_serl_params(serl_profile_update(profiles, name))`` leaves on the arm)."""
    return {**resolve_serl_params(current), **serl_profile_update(profiles, name)}


def franka_load(load: Mapping[str, Any] | None, *, where: str = "load") -> dict | None:
    """The station ``load:`` block as libfranka ``Robot::setLoad`` arguments.

    ``{mass_kg, center_of_mass_m: [x, y, z] (flange frame), inertia: 3x3 or 9 values (kg m^2, about
    the center of mass)}`` -> ``{"mass_kg", "center_of_mass_m", "inertia"}`` with ``inertia`` as 9
    column-major values (a symmetric tensor reads the same either way). None stays None."""
    if load is None:
        return None
    if not isinstance(load, Mapping):
        raise ValueError(f"{where} must be a mapping {{mass_kg, center_of_mass_m, inertia}}")
    known = {"mass_kg", "center_of_mass_m", "inertia"}
    unknown = sorted(set(load) - known)
    missing = sorted(known - set(load))
    if unknown or missing:
        raise ValueError(f"{where} needs exactly {sorted(known)}; unknown {unknown}, missing {missing}")

    def numbers(value, count, name):
        if isinstance(value, Mapping) or isinstance(value, (str, bytes)) or not hasattr(value, "__len__"):
            raise ValueError(f"{where}.{name} must be a list of {count} numbers")
        flat = []
        for item in value:
            if isinstance(item, (list, tuple)):
                flat.extend(item)
            else:
                flat.append(item)
        if len(flat) != count or any(isinstance(v, bool) or not isinstance(v, (int, float))
                                     or not math.isfinite(v) for v in flat):
            raise ValueError(f"{where}.{name} must hold {count} finite numbers, got {value!r}")
        return [float(v) for v in flat]

    mass = load["mass_kg"]
    if isinstance(mass, bool) or not isinstance(mass, (int, float)) or not math.isfinite(mass) or mass < 0:
        raise ValueError(f"{where}.mass_kg must be a finite nonnegative number, got {mass!r}")
    center = numbers(load["center_of_mass_m"], 3, "center_of_mass_m")
    inertia_value = load["inertia"]
    if isinstance(inertia_value, (list, tuple)) and len(inertia_value) == 3 and all(
            isinstance(row, (list, tuple)) and len(row) == 3 for row in inertia_value):
        rows = [numbers(row, 3, "inertia") for row in inertia_value]
        inertia = [rows[row][col] for col in range(3) for row in range(3)]  # rows -> column-major
    elif isinstance(inertia_value, (list, tuple)) and all(not isinstance(v, (list, tuple)) for v in inertia_value):
        inertia = numbers(inertia_value, 9, "inertia")
    else:
        raise ValueError(f"{where}.inertia must be a 3x3 matrix or 9 numbers, got {inertia_value!r}")
    matrix = [[inertia[col * 3 + row] for col in range(3)] for row in range(3)]
    if any(abs(matrix[i][j] - matrix[j][i]) > 1e-9 for i in range(3) for j in range(3)):
        raise ValueError(f"{where}.inertia must be symmetric, got {matrix}")
    return {"mass_kg": float(mass), "center_of_mass_m": center, "inertia": inertia}


# cartesian_controller: serl. HIL-SERL's FrankaEnv POSTs /clearerr before every /pose, so a reflex
# that ended SERL's impedance controller is cleared and the controller restarts at the measured
# pose. rwrl does the same on the next servo target, at most max_per_episode times per episode
# (an episode starts when Cartesian servo is entered with set_control_mode, or at
# reset_serl_auto_recovery(), which RealRobotEnv calls on every reset).
SERL_AUTO_RECOVER_DEFAULTS = {"enabled": True, "max_per_episode": 5}


def serl_auto_recover_config(value: Mapping[str, Any] | None, *, where: str = "serl_auto_recover") -> dict:
    """The station ``serl_auto_recover: {enabled, max_per_episode}`` over the defaults."""
    if value is None:
        return dict(SERL_AUTO_RECOVER_DEFAULTS)
    if not isinstance(value, Mapping):
        raise ValueError(f"{where} must be a mapping {{enabled, max_per_episode}}")
    unknown = sorted(set(value) - set(SERL_AUTO_RECOVER_DEFAULTS))
    if unknown:
        raise ValueError(f"unknown {where} keys {unknown}; known: {sorted(SERL_AUTO_RECOVER_DEFAULTS)}")
    config = {**SERL_AUTO_RECOVER_DEFAULTS, **value}
    if not isinstance(config["enabled"], bool):
        raise ValueError(f"{where}.enabled must be true or false, got {config['enabled']!r}")
    cap = config["max_per_episode"]
    if isinstance(cap, bool) or not isinstance(cap, int) or cap < 0:
        raise ValueError(f"{where}.max_per_episode must be an integer >= 0, got {cap!r}")
    return {"enabled": config["enabled"], "max_per_episode": int(cap)}


@dataclass(frozen=True)
class ArmControlConfig:
    translational_stiffness: float | None = None
    rotational_stiffness: float | None = None
    cartesian_damping: list[float] | None = None
    joint_stiffness: list[float] | None = None
    joint_damping: list[float] | None = None
    translational_error_clip: list[float] | None = None
    rotational_error_clip: list[float] | None = None
    max_delta_tau: float | None = None
    gains_time_constant: float | None = None

    def __post_init__(self) -> None:
        vectors = {"cartesian_damping": 6, "translational_error_clip": 3, "rotational_error_clip": 3}
        for field in dataclasses.fields(self):
            value = getattr(self, field.name)
            if value is None:
                continue
            if field.name in vectors or field.name in ("joint_stiffness", "joint_damping"):
                if not isinstance(value, (list, tuple)) or not value:
                    raise ValueError(f"{field.name} must be a nonempty numeric vector")
                if field.name in vectors and len(value) != vectors[field.name]:
                    raise ValueError(f"{field.name} requires {vectors[field.name]} entries")
                values = value
            else:
                values = [value]
            positive = field.name in ("max_delta_tau", "gains_time_constant", "translational_error_clip", "rotational_error_clip")
            for item in values:
                if isinstance(item, bool) or not isinstance(item, (int, float)) or not math.isfinite(item) or (item <= 0 if positive else item < 0):
                    raise ValueError(f"{field.name} requires finite {'positive' if positive else 'nonnegative'} numbers")

    @classmethod
    def from_dict(cls, config: Mapping[str, Any] | "ArmControlConfig" | None) -> "ArmControlConfig":
        if isinstance(config, cls):
            return config
        config = dict(config or {})
        unknown = config.keys() - {field.name for field in dataclasses.fields(cls)}
        if unknown:
            raise ValueError(f"unknown arm control settings: {sorted(unknown)}")
        return cls(**config)

    def to_dict(self) -> dict[str, Any]:
        return {name: value for name, value in dataclasses.asdict(self).items() if value is not None}

    def for_mode(self, mode: str) -> dict[str, Any]:
        config = self.to_dict()
        result = {key: config[key] for key in ("max_delta_tau", "gains_time_constant") if key in config}
        if mode == "cartesian_servo":
            result.update({key: config[key] for key in ("translational_stiffness", "rotational_stiffness", "translational_error_clip", "rotational_error_clip") if key in config})
            if "cartesian_damping" in config:
                result["damping"] = config["cartesian_damping"]
        elif mode == "joint_servo":
            for source, target in (("joint_stiffness", "stiffness"), ("joint_damping", "damping")):
                if source in config:
                    result[target] = config[source]
        return result
