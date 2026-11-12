import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from real_world_rl.teleop.core.targets.twist_input import validate_input_rotation, rotate_input
from real_world_rl.teleop.spacemouse.source import SpaceMouseTeleopSource
from test_spacemouse_teleop_source import _Part, NO_GRIPPER, HOME_TCP


@pytest.mark.parametrize("bad", [np.eye(2), np.diag([1, 1, -1]), np.eye(3)*2, np.full((3,3),np.nan)])
def test_reject_invalid_frame(bad):
    with pytest.raises(ValueError):
        validate_input_rotation(bad)


def test_tracker_deadzone_precedes_rotation_and_world_up_roundtrip():
    world_R_base = Rotation.from_euler("xyz", [-42, 3, 6], degrees=True).as_matrix()
    source = SpaceMouseTeleopSource(_Part(), follower_pose_getter=lambda: HOME_TCP,
                                   env_contract=NO_GRIPPER, input_rotation=world_R_base.T)
    result = source._twist([.01, -.01, .5, 0, 0, 0])
    np.testing.assert_allclose(world_R_base @ result[:3], [0, 0, .5], atol=1e-12)


def test_rotated_diagonal_is_bounded_without_direction_clipping():
    r = Rotation.from_euler("z", 30, degrees=True).as_matrix()
    result = rotate_input(np.ones(6), r)
    expected = r @ np.ones(3)
    np.testing.assert_allclose(result[:3] / np.linalg.norm(result[:3]), expected / np.linalg.norm(expected))
    assert np.max(abs(result)) <= 1


@pytest.mark.parametrize('axes', ['translation', 'rotation', 'all'])
def test_explicit_diagnostic_filter_keeps_normal_six_axis_mode(axes):
    from real_world_rl.teleop.core.targets.twist_input import diagnostic_input, input_frame_contract
    twist = np.array([.1, .2, .3, .4, .5, .6])
    np.testing.assert_array_equal(diagnostic_input(twist), twist)
    output = diagnostic_input(twist, axes, .2)
    if axes == 'translation':
        np.testing.assert_array_equal(output[3:], 0)
        np.testing.assert_allclose(output[:3], twist[:3] * .2)
    elif axes == 'rotation':
        np.testing.assert_array_equal(output[:3], 0)
        np.testing.assert_allclose(output[3:], twist[3:] * .2)
    else:
        np.testing.assert_allclose(output, twist * .2)
    assert input_frame_contract({}) == {}
    assert input_frame_contract({'diagnostic_axes': axes, 'diagnostic_gain': .2})['input_diagnostic']['gain'] == .2
