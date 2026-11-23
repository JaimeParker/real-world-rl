"""Tighter-only environment overrides of resolved station limits."""
from dataclasses import replace
from collections.abc import Mapping

import numpy as np

from real_world_rl.devices.limits import ArmLimits, TcpBox


def validate_overrides(value, num_joints):
    if value is None:
        return
    if not isinstance(value, Mapping) or set(value) - {"joints", "tcp"}:
        raise ValueError("limits accepts only joints and tcp overrides")
    joints = value.get("joints")
    if joints is not None:
        checked = ArmLimits(joints=joints).joints
        if checked.shape != (num_joints, 2):
            raise ValueError(f"limits.joints must have shape ({num_joints}, 2)")
    tcp = value.get("tcp")
    if tcp is not None:
        if not isinstance(tcp, Mapping) or set(tcp) != {"low", "high"}:
            raise ValueError("limits.tcp requires only low and high; origin/axes come from the device")
        TcpBox(np.zeros(3), np.eye(3), tcp["low"], tcp["high"])


def merge_limits(device, overrides):
    result = ArmLimits() if device is None else ArmLimits.from_dict(device)
    overrides = overrides or {}
    joints = overrides.get("joints")
    if joints is not None:
        joints = np.asarray(joints, dtype=float)
        if result.joints is not None and (
            joints.shape != result.joints.shape
            or np.any(joints[:, 0] < result.joints[:, 0])
            or np.any(joints[:, 1] > result.joints[:, 1])
        ):
            raise ValueError("env limits.joints are looser than the robot's limits")
        result = replace(result, joints=joints)
    tcp = overrides.get("tcp")
    if tcp is not None:
        if result.tcp is None:
            raise ValueError("env limits.tcp requires a device TCP box for origin/axes")
        if np.any(np.asarray(tcp["low"]) < result.tcp.low) or np.any(np.asarray(tcp["high"]) > result.tcp.high):
            raise ValueError("env limits.tcp is looser than the robot's limits")
        result = replace(result, tcp=replace(result.tcp, low=tcp["low"], high=tcp["high"]))
    return result
