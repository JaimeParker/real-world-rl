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
