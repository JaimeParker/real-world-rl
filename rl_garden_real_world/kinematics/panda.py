"""Forward kinematics of the Franka Panda (also FR3, which shares the kinematic chain).

Craig's modified Denavit-Hartenberg convention, as Franka publishes it: joint ``i`` contributes
``RotX(alpha) TransX(a) RotZ(q_i) TransZ(d)``, then the flange adds ``TransZ(0.107)``. The
end-effector frame is the flange frame moved by ``ee_offset_m`` along its z axis and rotated by
``ee_yaw_rad`` about it (``F_T_EE``; the defaults are the Franka Hand's, which is what the robot
reports as ``O_T_EE`` when the hand is configured in Desk).

Hardware verification is pending: the table matches the reference values below (the flange at
``q = 0`` is at ``(0.088, 0, 0.926)`` with its z axis pointing down; at the ready pose
``[0, -pi/4, 0, -3pi/4, 0, pi/2, pi/4]`` it is at about ``(0.307, 0, 0.590)``), but it has not
yet been compared with the robot's own ``O_T_EE`` (for example ``ee_pose(q)`` against the
bridge's ``/getstate`` pose at a few joint configurations). Numpy only; about 20 us per call.
"""
from __future__ import annotations

import math

import numpy as np

from rl_garden_real_world.kinematics.transforms import transform_to_pose

# (a, d, alpha) per joint, Craig's modified DH; joint i rotates about its own z by q_i.
PANDA_MODIFIED_DH = np.array(
    [
        [0.0, 0.333, 0.0],
        [0.0, 0.0, -math.pi / 2],
        [0.0, 0.316, math.pi / 2],
        [0.0825, 0.0, math.pi / 2],
        [-0.0825, 0.384, -math.pi / 2],
        [0.0, 0.0, math.pi / 2],
        [0.088, 0.0, math.pi / 2],
    ]
)
PANDA_FLANGE_D = 0.107
FRANKA_HAND_EE_OFFSET_M = 0.1034
FRANKA_HAND_EE_YAW_RAD = -math.pi / 4


def modified_dh_transform(a: float, d: float, alpha: float, theta: float) -> np.ndarray:
    """``RotX(alpha) TransX(a) RotZ(theta) TransZ(d)`` as a ``(4, 4)`` transform."""
    ca, sa = math.cos(alpha), math.sin(alpha)
    ct, st = math.cos(theta), math.sin(theta)
    return np.array(
        [
            [ct, -st, 0.0, a],
            [st * ca, ct * ca, -sa, -d * sa],
            [st * sa, ct * sa, ca, d * ca],
            [0.0, 0.0, 0.0, 1.0],
        ]
    )


class PandaKinematics:
    """Base-frame flange and end-effector poses of a Panda from its 7 joint angles (rad).

    Poses are ``(xyz(3) float64, quat_xyzw(4) float64)``; transforms are ``(4, 4)`` float64.
    """

    NUM_JOINTS = 7

    def __init__(
        self,
        ee_offset_m: float = FRANKA_HAND_EE_OFFSET_M,
        ee_yaw_rad: float = FRANKA_HAND_EE_YAW_RAD,
    ) -> None:
        self.ee_offset_m = float(ee_offset_m)
        self.ee_yaw_rad = float(ee_yaw_rad)
        self._flange_to_ee = modified_dh_transform(0.0, self.ee_offset_m, 0.0, self.ee_yaw_rad)
        self._last_link_to_flange = modified_dh_transform(0.0, PANDA_FLANGE_D, 0.0, 0.0)

    def _joint_vector(self, q: np.ndarray) -> np.ndarray:
        q = np.asarray(q, dtype=np.float64).reshape(-1)
        if q.shape != (self.NUM_JOINTS,):
            raise ValueError(f"PandaKinematics needs {self.NUM_JOINTS} joint angles, got shape {q.shape}")
        if not np.all(np.isfinite(q)):
            raise ValueError(f"joint angles must be finite, got {q}")
        return q

    def flange_transform(self, q: np.ndarray) -> np.ndarray:
        transform = np.eye(4)
        for (a, d, alpha), theta in zip(PANDA_MODIFIED_DH, self._joint_vector(q)):
            transform = transform @ modified_dh_transform(a, d, alpha, theta)
        return transform @ self._last_link_to_flange

    def ee_transform(self, q: np.ndarray) -> np.ndarray:
        return self.flange_transform(q) @ self._flange_to_ee

    def flange_pose(self, q: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        return transform_to_pose(self.flange_transform(q))

    def ee_pose(self, q: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        return transform_to_pose(self.ee_transform(q))
