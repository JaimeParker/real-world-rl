"""Single-arm RLinf RelativeFrame and FrankaEnv Cartesian conventions."""
from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation

STATE_FIELDS = ("gripper_pos", "tcp_force", "tcp_pose", "tcp_torque", "tcp_vel")
OPEN_WIDTH_M = 0.05  # RLinf's FrankyGripper.is_open threshold.


def _adjoint(pose: np.ndarray) -> np.ndarray:
    rotation = Rotation.from_quat(pose[3:]).as_matrix()
    adjoint = np.zeros((6, 6))
    adjoint[:3, :3] = adjoint[3:, 3:] = rotation
    # RLinf uses [linear, angular] vectors with this lower-left block.
    adjoint[3:, :3] = np.cross(pose[:3], rotation.T).T
    return adjoint


def policy_state(state: dict, reset_pose: np.ndarray, max_width_m: float,
                 ee_to_stiffness: Rotation) -> np.ndarray:
    pose = np.asarray(state["tcp_pose"])
    reset_rotation = Rotation.from_quat(reset_pose[3:]).inv()
    rotation = Rotation.from_quat(pose[3:])
    relative_xyz = reset_rotation.apply(pose[:3] - reset_pose[:3])
    relative_rotation = reset_rotation * rotation
    ee_vel = np.linalg.solve(_adjoint(pose), state["tcp_vel"])
    # Both wrench variants are about K. rwrl franky expresses that wrench in base
    # coordinates, while RLinf franky reports it in stiffness-frame coordinates.
    base_to_stiffness = (rotation * ee_to_stiffness).inv()
    return np.concatenate((
        state["gripper_pos"] * max_width_m,
        base_to_stiffness.apply(state["tcp_force"]),
        relative_xyz,
        relative_rotation.as_euler("xyz"),
        base_to_stiffness.apply(state["tcp_torque"]),
        ee_vel,
    )).astype(np.float32)


def target_pose(pose: np.ndarray, action: np.ndarray, xyz_scale: float,
                rot_scale: float) -> np.ndarray:
    # RelativeFrame transforms before the underlying FrankaEnv clips.
    base = np.clip(_adjoint(pose) @ action[:6], -1.0, 1.0)
    xyz = pose[:3] + base[:3] * xyz_scale
    rotation = Rotation.from_euler("xyz", base[3:] * rot_scale) * Rotation.from_quat(pose[3:])
    return np.concatenate((xyz, rotation.as_quat()))


def gripper_target(action: float, *, width_m: float, scale: float,
                   threshold: float) -> float | None:
    scaled = action * scale
    is_open = width_m >= OPEN_WIDTH_M
    if scaled <= -threshold and is_open:
        return 0.0
    if scaled >= threshold and not is_open:
        return 1.0
    return None
