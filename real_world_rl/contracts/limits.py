"""Canonical comparison of resolved limits and legacy checkpoint geometry."""
from collections.abc import Mapping

import numpy as np

from real_world_rl.devices.limits import ArmLimits, TcpBox


def checkpoint_limits(value):
    """Translate legacy axis-aligned geometry without dropping constraints."""
    if (value.get("limits") or {}).get("version") == "station_limits_v1":
        return value["limits"]
    joints = value.get("joint_limits")
    bounds = value.get("safety_box")
    workspace = value.get("workspace") or (value.get("spatial_constraints") or {}).get("workspace")
    if workspace is not None:
        if workspace.get("frame", "base") != "base" or any(
            workspace.get(key) is not None for key in ("orientation_xyzw", "max_angle_rad")
        ):
            raise ValueError("legacy checkpoint limits are incompatible; retrain with station_limits_v1")
        other = np.asarray([*workspace["low"], *workspace["high"]], dtype=float)
        bounds = other if bounds is None else np.concatenate([
            np.maximum(bounds[:3], other[:3]), np.minimum(bounds[3:], other[3:])])
    tcp = None
    if bounds is not None:
        low, high = np.asarray(bounds, dtype=float).reshape(2, 3)
        center = (low + high) / 2
        tcp = TcpBox(center, np.eye(3), low-center, high-center)
    steps = value.get("max_delta_per_step") or {}
    return ArmLimits(joints=joints, tcp=tcp, max_delta_xyz_m=steps.get("xyz_m"),
                     max_delta_joint_rad=steps.get("joint_rad")).contract_dict()


def canonical_limits(value):
    """Compare geometry independently of the chosen origin for an identity-axis box."""
    if not isinstance(value, Mapping) or value.get("version") != "station_limits_v1":
        return value
    result = dict(value)
    box = ArmLimits.from_dict(value).tcp
    if box is not None and box.absolute_aabb() is not None:
        bounds = box.absolute_aabb()
        center = (bounds[:3] + bounds[3:]) / 2
        result["tcp"] = TcpBox(center, np.eye(3), bounds[:3]-center,
                               bounds[3:]-center, box.measured_margin_m).to_dict()
    return result


def legacy_limits_match(saved, actual):
    """Compare only legacy constraints that were recorded, preserving every such bound."""
    from real_world_rl.contracts.action import _contract_values_match
    saved, actual = canonical_limits(saved), canonical_limits(actual)
    return all(saved.get(key) is None or _contract_values_match(saved[key], actual.get(key))
               for key in ("joints", "tcp", "max_delta_per_step"))


def translate_legacy_contract(saved, actual):
    """Translate old metadata against resolved geometry before ordinary contract comparison."""
    if not isinstance(saved, Mapping) or not isinstance(actual, Mapping):
        return saved
    result = dict(saved)
    legacy = any(key in result for key in ("safety_box", "workspace", "spatial_constraints"))
    legacy = legacy or ("joint_limits" in result and "limits" not in result)
    if legacy and isinstance(actual.get("limits"), Mapping) and actual["limits"].get("version") == "station_limits_v1":
        if not legacy_limits_match(checkpoint_limits(result), actual["limits"]):
            raise ValueError("legacy checkpoint limits are incompatible with station limits; retrain")
        result["limits"] = actual["limits"]
        for key in ("safety_box", "workspace", "spatial_constraints"):
            result.pop(key, None)
        if "action_mode" not in result:
            result.pop("joint_limits", None)
    # XYZ bounds formerly duplicated in SERL semantics now live in resolved limits.
    if isinstance(result.get("serl"), Mapping):
        serl = dict(result["serl"])
        low, high = serl.pop("safety_box_low", None), serl.pop("safety_box_high", None)
        if low is not None or high is not None:
            if low is None or high is None or not legacy_limits_match(
                checkpoint_limits({"safety_box": [*low, *high]}), actual.get("limits", {})
            ):
                raise ValueError("legacy SERL checkpoint limits are incompatible; retrain")
        result["serl"] = serl
    return {key: translate_legacy_contract(value, actual.get(key)) for key, value in result.items()}
