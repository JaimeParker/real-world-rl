"""Pure target/clip/gripper tests; no devices, clocks or sleeps."""
from __future__ import annotations

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from real_world_rl.envs.serl_cartesian import clip_safety_box, gripper_command, target_pose


def test_serl_target_uses_latest_measured_pose():
    action = np.array([1, -0.5, 0.25, 0, 0, 0])
    measured = np.array([0.4, 0.1, 0.3, 0, 0, 0, 1.0])
    original = measured.copy()
    first = target_pose(measured, action, 0.02, 0.1)
    np.testing.assert_allclose(first[:3], [0.42, 0.09, 0.305])
    np.testing.assert_array_equal(measured, original)
    # Feedback lags the first command; the next target must follow that feedback.
    measured[0] = 0.405
    second = target_pose(measured, action, 0.02, 0.1)
    assert second[0] == pytest.approx(0.425)


@pytest.mark.parametrize("roll", [-2.9, 2.9])
def test_serl_rotation_left_multiplies_and_clips_roll_magnitude(roll):
    measured = np.r_[[0.4, 0, 0.2], Rotation.from_euler("xyz", [roll, 0.2, 0.3]).as_quat()]
    action = np.array([1, -1, 1, 0.2, -0.4, 0.6])
    target = target_pose(measured, action, 0.02, 0.1)
    expected = Rotation.from_rotvec([0.02, -0.04, 0.06]) * Rotation.from_quat(measured[3:])
    np.testing.assert_allclose(Rotation.from_quat(target[3:]).as_matrix(), expected.as_matrix())
    right_product = Rotation.from_quat(measured[3:]) * Rotation.from_rotvec([0.02, -0.04, 0.06])
    assert not np.allclose(expected.as_matrix(), right_product.as_matrix())
    original = target.copy()
    clipped = clip_safety_box(target, [0, -0.01, 0], [0.41, 0.01, 0.21],
                              [2.5, -0.1, -0.15], [2.7, 0.1, 0.15])
    np.testing.assert_allclose(clipped[:3], [0.41, -0.01, 0.21])
    np.testing.assert_allclose(Rotation.from_quat(clipped[3:]).as_euler("xyz"),
                               [np.sign(roll) * 2.7, 0.1, 0.15])
    np.testing.assert_array_equal(target, original)


@pytest.mark.parametrize("roll,expected", [(-0.1, -0.4), (0.0, 0.0), (0.1, 0.4)])
def test_serl_roll_lower_magnitude_and_zero_sign(roll, expected):
    pose = np.r_[[0, 0, 0], Rotation.from_euler("xyz", [roll, -0.5, -0.6]).as_quat()]
    clipped = clip_safety_box(pose, [-1] * 3, [1] * 3, [0.4, -0.2, -0.3], [1, 0.2, 0.3])
    np.testing.assert_allclose(Rotation.from_quat(clipped[3:]).as_euler("xyz"),
                               [expected, -0.2, -0.3], atol=1e-12)


@pytest.mark.parametrize("width", [0.0, 0.85, 1.0])
def test_serl_gripper_zero_sends_no_command(width):
    assert gripper_command(0.0, width, 10.0, 0.0) is None


@pytest.mark.parametrize("action,width,elapsed,expected", [
    (-0.5, 1.0, 0.61, False),
    (0.5, 0.0, 0.61, True),
    (-0.499999, 1.0, 0.61, None),
    (0.499999, 0.0, 0.61, None),
    (-1.0, 0.85, 0.61, None),
    (1.0, 0.85, 0.61, None),
    (-1.0, np.nextafter(0.85, 1.0), 0.61, False),
    (1.0, np.nextafter(0.85, 0.0), 0.61, True),
    (-1.0, 0.0, 0.61, None),
    (1.0, 1.0, 0.61, None),
    (-1.0, 1.0, 0.6, None),
    (1.0, 0.0, 0.6, None),
    (-1.0, 1.0, np.nextafter(0.6, 1.0), False),
    (1.0, 0.0, np.nextafter(0.6, 1.0), True),
    (-1.0, 1.0, 0.59, None),
    (1.0, 0.0, 0.0, None),
])
def test_serl_gripper_threshold_width_and_time_boundaries(action, width, elapsed, expected):
    assert gripper_command(action, width, elapsed, 0.0) is expected


@pytest.mark.parametrize("action,width,scale,expected", [
    (-1.0, 1.0, 0.49, None), (-1.0, 1.0, 0.5, False),
    (1.0, 0.0, 0.49, None), (1.0, 0.0, 0.5, True),
    (1.0, 0.0, 0.0, None),
])
def test_serl_gripper_action_scale(action, width, scale, expected):
    assert gripper_command(action, width, 1.0, 0.0, gripper_action_scale=scale) is expected
