"""Torch-free station and RPC validation for the native joint controller."""
from collections.abc import Mapping
from copy import deepcopy

JOINT_TRAJECTORY_KEYS = (
    "stiffness", "damping", "velocity_filter_alpha", "max_velocity", "max_acceleration",
    "min_segment_duration_s", "lower_limit", "upper_limit", "limit_margin",
    "command_timeout_s", "max_delta_tau", "compensate_coriolis",
)


def joint_trajectory_updates(updates) -> dict:
    if updates is None:
        return {}
    if not isinstance(updates, Mapping):
        raise ValueError("joint_trajectory must be a mapping")
    unknown = set(updates) - set(JOINT_TRAJECTORY_KEYS)
    if unknown:
        raise ValueError(f"unknown joint_trajectory keys: {sorted(unknown)}; choose from {list(JOINT_TRAJECTORY_KEYS)}")
    return deepcopy(dict(updates))


def station_joint_trajectory(updates, joints=None) -> dict:
    """Resolve position bounds against the station; explicit bounds may only tighten."""
    import numpy as np
    values = joint_trajectory_updates(updates)
    if joints is None:
        return values
    bounds = np.asarray(joints, dtype=float)
    for key, column in (("lower_limit", 0), ("upper_limit", 1)):
        value = np.asarray(values.get(key, bounds[:, column]), dtype=float)
        if value.shape != (len(bounds),) or not np.isfinite(value).all():
            raise ValueError(f"{key} must contain {len(bounds)} finite numbers")
        if np.any(value < bounds[:, 0]) or np.any(value > bounds[:, 1]):
            raise ValueError(f"{key} may only tighten limits.joints")
        values[key] = value.tolist()
    if np.any(np.asarray(values["lower_limit"]) > np.asarray(values["upper_limit"])):
        raise ValueError("lower_limit must be <= upper_limit")
    return values
