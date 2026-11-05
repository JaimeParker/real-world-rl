"""Pure HIL-SERL Cartesian target, safety-box and binary gripper semantics.

Poses use base-frame xyz metres and xyzw quaternions. Device-layer safety
limits still apply after these targets and may impose tighter bounds.
"""
from __future__ import annotations

import math
from typing import TYPE_CHECKING, Any, Sequence

if TYPE_CHECKING:
    import numpy as np
    from numpy.typing import ArrayLike, NDArray


def validate_serl_config(
    xyz_low: Sequence[float] | None,
    xyz_high: Sequence[float] | None,
    rpy_low: Sequence[float] | None,
    rpy_high: Sequence[float] | None,
    *,
    gripper_action_scale: float,
    gripper_sleep_s: float,
    control_hz: float,
) -> None:
    """Require complete finite pose limits and valid SERL scales/timing."""
    for name, low, high in (
        ("safety_box", xyz_low, xyz_high),
        ("safety_box_rpy", rpy_low, rpy_high),
    ):
        if low is None or high is None or len(low) != 3 or len(high) != 3:
            raise ValueError(f"serl requires complete {name} low/high limits (three values each)")
        if not all(math.isfinite(value) for value in (*low, *high)):
            raise ValueError(f"{name} limits must be finite")
        if any(lo > hi for lo, hi in zip(low, high)):
            raise ValueError(f"{name} limits must satisfy low <= high")
    for name, value in (
        ("gripper_action_scale", gripper_action_scale),
        ("gripper_sleep_s", gripper_sleep_s),
        ("control_hz", control_hz),
    ):
        if not math.isfinite(value) or value < 0 or (name == "control_hz" and value == 0):
            raise ValueError(f"{name} must be finite and {'positive' if name == 'control_hz' else 'nonnegative'}")


def target_pose(
    measured_pose: ArrayLike, action: ArrayLike, xyz_scale_m: float, rot_scale_rad: float,
) -> NDArray[np.float64]:
    """Add a per-step translation and left-multiply rotation onto measured pose."""
    import numpy as np
    from scipy.spatial.transform import Rotation

    measured_pose = np.asarray(measured_pose, dtype=np.float64)
    action = np.asarray(action, dtype=np.float64)
    xyz = measured_pose[:3] + action[:3] * xyz_scale_m
    rotation = Rotation.from_rotvec(action[3:6] * rot_scale_rad) * Rotation.from_quat(
        measured_pose[3:7]
    )
    return np.concatenate([xyz, rotation.as_quat()])


def clip_safety_box(
    pose: ArrayLike,
    xyz_low: ArrayLike,
    xyz_high: ArrayLike,
    rpy_low: Sequence[float],
    rpy_high: Sequence[float],
) -> NDArray[np.float64]:
    """Return a clipped copy, preserving roll sign while clipping its magnitude."""
    import numpy as np
    from scipy.spatial.transform import Rotation

    clipped = np.array(pose, dtype=np.float64, copy=True)
    clipped[:3] = np.clip(clipped[:3], xyz_low, xyz_high)
    euler = Rotation.from_quat(clipped[3:7]).as_euler("xyz")
    euler[0] = np.sign(euler[0]) * np.clip(abs(euler[0]), rpy_low[0], rpy_high[0])
    euler[1:] = np.clip(euler[1:], rpy_low[1:], rpy_high[1:])
    clipped[3:7] = Rotation.from_euler("xyz", euler).as_quat()
    return clipped


def gripper_command(
    action: float,
    measured_width: float,
    now: float,
    last_gripper_act: float,
    *,
    gripper_action_scale: float = 1.0,
    gripper_sleep_s: float = 0.6,
) -> bool | None:
    """Return open/close or None, using strict measured-width and elapsed gates."""
    g = action * gripper_action_scale
    if now - last_gripper_act > gripper_sleep_s:
        if g <= -0.5 and measured_width > 0.85:
            return False
        if g >= 0.5 and measured_width < 0.85:
            return True
    return None


def semantics_contract(
    xyz_low: Sequence[float],
    xyz_high: Sequence[float],
    rpy_low: Sequence[float],
    rpy_high: Sequence[float],
    gripper_action_scale: float,
    gripper_sleep_s: float,
) -> dict[str, Any]:
    """Compared experiment entries, added only for the opt-in SERL branch."""
    return {
        "cartesian_semantics": "serl",
        "serl": {
            "safety_box_low": list(xyz_low),
            "safety_box_high": list(xyz_high),
            "safety_box_rpy_low": list(rpy_low),
            "safety_box_rpy_high": list(rpy_high),
            "gripper_action_scale": gripper_action_scale,
            "gripper_sleep_s": gripper_sleep_s,
        },
    }
