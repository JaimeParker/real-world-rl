"""Numpy rigid-transform helpers shared by the kinematics and the cartesian teleop sources.

Quaternions are ``xyzw`` (scipy's and the Franka envs' order); poses are ``xyz(3) + quat_xyzw(4)``.
"""
from __future__ import annotations

import numpy as np

_SMALL_ANGLE_SIN = 1e-12


def quat_xyzw_to_matrix(quat_xyzw: np.ndarray) -> np.ndarray:
    """``(3, 3)`` rotation matrix of a (not necessarily unit) ``xyzw`` quaternion."""
    x, y, z, w = np.asarray(quat_xyzw, dtype=np.float64) / np.linalg.norm(quat_xyzw)
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ]
    )


def matrix_to_quat_xyzw(rotation: np.ndarray) -> np.ndarray:
    """Unit ``xyzw`` quaternion of a rotation matrix, with ``w >= 0`` (Shepperd's method)."""
    m = np.asarray(rotation, dtype=np.float64)
    trace = m[0, 0] + m[1, 1] + m[2, 2]
    if trace > 0.0:
        s = 2.0 * np.sqrt(trace + 1.0)
        quat = [(m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s, 0.25 * s]
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = 2.0 * np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2])
        quat = [0.25 * s, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s, (m[2, 1] - m[1, 2]) / s]
    elif m[1, 1] > m[2, 2]:
        s = 2.0 * np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2])
        quat = [(m[0, 1] + m[1, 0]) / s, 0.25 * s, (m[1, 2] + m[2, 1]) / s, (m[0, 2] - m[2, 0]) / s]
    else:
        s = 2.0 * np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1])
        quat = [(m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, 0.25 * s, (m[1, 0] - m[0, 1]) / s]
    quat = np.asarray(quat) / np.linalg.norm(quat)
    return -quat if quat[3] < 0.0 else quat


def matrix_to_rotvec(rotation: np.ndarray) -> np.ndarray:
    """Rotation vector (axis * angle, angle in ``[0, pi]``) of a rotation matrix."""
    quat = matrix_to_quat_xyzw(rotation)
    sin_half = float(np.linalg.norm(quat[:3]))
    if sin_half < _SMALL_ANGLE_SIN:
        return 2.0 * quat[:3]
    return quat[:3] / sin_half * (2.0 * np.arctan2(sin_half, quat[3]))


def pose_to_transform(pose: np.ndarray) -> np.ndarray:
    """``(4, 4)`` homogeneous transform of an ``xyz + quat_xyzw`` pose."""
    pose = np.asarray(pose, dtype=np.float64).reshape(-1)
    if pose.shape != (7,):
        raise ValueError(f"a pose is xyz(3) + quat_xyzw(4), got shape {pose.shape}")
    transform = np.eye(4)
    transform[:3, :3] = quat_xyzw_to_matrix(pose[3:7])
    transform[:3, 3] = pose[:3]
    return transform


def transform_to_pose(transform: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """``(xyz(3), quat_xyzw(4))`` of a ``(4, 4)`` homogeneous transform."""
    transform = np.asarray(transform, dtype=np.float64)
    return transform[:3, 3].copy(), matrix_to_quat_xyzw(transform[:3, :3])


def invert_transform(transform: np.ndarray) -> np.ndarray:
    """Inverse of a rigid ``(4, 4)`` transform."""
    rotation_t = transform[:3, :3].T
    inverse = np.eye(4)
    inverse[:3, :3] = rotation_t
    inverse[:3, 3] = -rotation_t @ transform[:3, 3]
    return inverse
