"""PandaKinematics against Franka's published reference poses, and the numpy transform helpers
against scipy."""
from __future__ import annotations

import math

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from real_world_rl.kinematics import (
    PandaKinematics,
    invert_transform,
    matrix_to_quat_xyzw,
    matrix_to_rotvec,
    pose_to_transform,
    quat_xyzw_to_matrix,
)

READY_Q = [0.0, -math.pi / 4, 0.0, -3 * math.pi / 4, 0.0, math.pi / 2, math.pi / 4]
SAMPLE_QS = [
    [0.0] * 7,
    READY_Q,
    [0.3, -0.5, 0.2, -2.0, 0.1, 1.8, -0.4],
    [-1.2, 0.7, -0.9, -1.1, 1.3, 2.9, 2.0],
]


def test_flange_at_zero_joints_is_above_the_base_pointing_down():
    position, quat = PandaKinematics().flange_pose(np.zeros(7))
    np.testing.assert_allclose(position, [0.088, 0.0, 0.926], atol=1e-9)
    flange_z = Rotation.from_quat(quat).as_matrix()[:, 2]
    np.testing.assert_allclose(flange_z, [0.0, 0.0, -1.0], atol=1e-9)
    assert position.dtype == np.float64 and quat.shape == (4,)


def test_flange_at_the_ready_pose():
    position, _ = PandaKinematics().flange_pose(READY_Q)
    np.testing.assert_allclose(position, [0.307, 0.0, 0.590], atol=2e-3)


@pytest.mark.parametrize("q", SAMPLE_QS)
def test_rotations_are_orthonormal_and_poses_match_the_transform(q):
    kinematics = PandaKinematics()
    transform = kinematics.ee_transform(q)
    rotation = transform[:3, :3]
    np.testing.assert_allclose(rotation @ rotation.T, np.eye(3), atol=1e-12)
    assert np.linalg.det(rotation) == pytest.approx(1.0, abs=1e-12)
    np.testing.assert_allclose(transform[3], [0.0, 0.0, 0.0, 1.0])
    position, quat = kinematics.ee_pose(q)
    np.testing.assert_allclose(position, transform[:3, 3])
    np.testing.assert_allclose(Rotation.from_quat(quat).as_matrix(), rotation, atol=1e-12)


@pytest.mark.parametrize("q", SAMPLE_QS)
def test_ee_is_the_flange_moved_along_its_z_and_yawed(q):
    kinematics = PandaKinematics()
    flange = kinematics.flange_transform(q)
    ee = kinematics.ee_transform(q)
    np.testing.assert_allclose(ee[:3, 3], flange[:3, 3] + 0.1034 * flange[:3, 2], atol=1e-12)
    yaw = Rotation.from_rotvec([0.0, 0.0, -math.pi / 4]).as_matrix()
    np.testing.assert_allclose(ee[:3, :3], flange[:3, :3] @ yaw, atol=1e-12)


def test_ready_pose_ee_points_down():
    position, quat = PandaKinematics().ee_pose(READY_Q)
    np.testing.assert_allclose(position, [0.307, 0.0, 0.590 - 0.1034], atol=2e-3)
    np.testing.assert_allclose(Rotation.from_quat(quat).as_matrix()[:, 2], [0.0, 0.0, -1.0], atol=1e-9)


def test_custom_tool_offset():
    kinematics = PandaKinematics(ee_offset_m=0.0, ee_yaw_rad=0.0)
    np.testing.assert_allclose(kinematics.ee_transform(READY_Q), kinematics.flange_transform(READY_Q))


@pytest.mark.parametrize("q", [np.zeros(6), np.zeros(8), [0.0] * 6 + [float("nan")]])
def test_rejects_wrong_joint_vectors(q):
    with pytest.raises(ValueError):
        PandaKinematics().ee_pose(q)


# -- transform helpers ------------------------------------------------------------

ROTVECS = [
    [0.0, 0.0, 0.0],
    [1e-9, -2e-9, 0.0],
    [0.3, -0.2, 0.1],
    [math.pi - 1e-6, 0.0, 0.0],
    [0.0, 2.0, -1.5],
    [-2.2, 0.4, 1.0],
]


@pytest.mark.parametrize("rotvec", ROTVECS)
def test_quaternion_and_rotvec_helpers_agree_with_scipy(rotvec):
    rotation = Rotation.from_rotvec(rotvec)
    matrix = rotation.as_matrix()
    np.testing.assert_allclose(quat_xyzw_to_matrix(rotation.as_quat()), matrix, atol=1e-12)
    quat = matrix_to_quat_xyzw(matrix)
    assert quat[3] >= 0.0
    np.testing.assert_allclose(Rotation.from_quat(quat).as_matrix(), matrix, atol=1e-12)
    np.testing.assert_allclose(matrix_to_rotvec(matrix), rotation.as_rotvec(), atol=1e-9)


def test_pose_transform_round_trip_and_inverse():
    pose = np.concatenate([[0.4, -0.1, 0.3], Rotation.from_rotvec([0.2, 0.5, -0.3]).as_quat()])
    transform = pose_to_transform(pose)
    np.testing.assert_allclose(transform @ invert_transform(transform), np.eye(4), atol=1e-12)
    np.testing.assert_allclose(transform[:3, 3], pose[:3])
    with pytest.raises(ValueError, match="xyz"):
        pose_to_transform(np.zeros(6))
