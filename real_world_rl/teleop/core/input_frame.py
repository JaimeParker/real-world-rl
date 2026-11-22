"""Operator input rotations into the robot base frame, before target generation.

The frame chain is base_R_input = base_R_table @ table_R_device. A configured
device rotation already contains that composition; it must not be composed again.
Controllers and policy actions do not use this operator-input transform.
"""
import numpy as np


def validate_input_rotation(value):
    if value is None:
        return None
    rotation = np.asarray(value, dtype=np.float64)
    if (rotation.shape != (3, 3) or not np.isfinite(rotation).all()
            or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6, rtol=0)
            or not np.isclose(np.linalg.det(rotation), 1.0, atol=1e-6, rtol=0)):
        raise ValueError("input_rotation must be a finite proper 3x3 rotation: base_R_input")
    return rotation.copy()


def rotate_twist(twist, rotation):
    """Rotate each twist vector, preserving direction within the max-abs action cube."""
    if rotation is None:
        return twist
    result = np.asarray(twist, dtype=np.float64).copy().reshape(6)
    # Translation and angular velocity rotate as separate vectors. There is no
    # origin translation term: these are input directions, not a spatial wrench.
    for start in (0, 3):
        vector = rotation @ result[start:start + 3]
        # Keep direction if diagonal input would exceed the action cube.
        result[start:start + 3] = vector / max(1.0, float(np.max(np.abs(vector))))
    return result


def task_frame_rotation(task_frame) -> np.ndarray | None:
    """Extract validated base_R_table from base_T_frame; None means no task frame.

    Only the rotation is used for operator input. The task origin does not move
    an anchored hand target or contribute a translation to a twist.
    """
    if task_frame is None:
        return None
    matrix = np.asarray(task_frame.get("base_T_frame"), dtype=np.float64)
    if matrix.shape != (4, 4) or not np.isfinite(matrix).all():
        raise ValueError("base_T_frame must be a finite 4x4 transform")
    if not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-8, rtol=0):
        raise ValueError("base_T_frame must be a proper rigid transform")
    return validate_input_rotation(matrix[:3, :3])


def resolve_input_rotation(*, device_rotation=None, task_frame_rotation=None):
    """Return (base_R_input, label), preferring an already composed device rotation.

    Validate both supplied rotations even when the device rotation wins. With
    neither configured, None denotes unchanged base-frame input.
    """
    task_rotation = validate_input_rotation(task_frame_rotation)
    device_rotation = validate_input_rotation(device_rotation)
    if device_rotation is not None:
        return device_rotation, "device_profile"
    if task_rotation is not None:
        return task_rotation, "task_frame"
    return None, "base"


def rotate_pose(position, rotation_matrix, rotation):
    """Apply rotation-only (R @ p, R @ Rm), or leave both unchanged for None.

    This is the operator pose convention used for anchored deltas, not an
    absolute rigid-frame transform: base_T_frame's translation is not applied.
    """
    if rotation is None:
        return position, rotation_matrix
    return rotation @ position, rotation @ rotation_matrix
