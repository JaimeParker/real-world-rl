import numpy as np
import pytest

from real_world_rl.devices.robots.franka import DATASHEET_JOINT_LIMITS, FRANKA_HOME_JOINTS
from real_world_rl.devices.robots.franka_kinematics import FLANGE_T_HAND, flange_pose, tcp_pose, tcp_position


def independent_chain(q):
    # Explicit fixed URDF joint origins followed by rotation about each joint's Z.
    origins = [(0, 0, .333, 0), (0, 0, 0, -np.pi/2), (0, -.316, 0, np.pi/2),
               (.0825, 0, 0, np.pi/2), (-.0825, .384, 0, -np.pi/2),
               (0, 0, 0, np.pi/2), (.088, 0, 0, np.pi/2)]
    result = np.eye(4)
    for angle, (x, y, z, roll) in zip(q, origins):
        fixed = np.eye(4)
        fixed[:3, 3] = [x, y, z]
        fixed[1:3, 1:3] = [[np.cos(roll), -np.sin(roll)], [np.sin(roll), np.cos(roll)]]
        turn = np.eye(4)
        turn[:2, :2] = [[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]]
        result = result @ fixed @ turn
    flange = np.eye(4)
    flange[2, 3] = .107
    return result @ flange


def test_reference_poses():
    np.testing.assert_allclose(flange_pose(np.zeros(7))[:3, 3], [.088, 0, .926], atol=1e-12)
    np.testing.assert_allclose(flange_pose(FRANKA_HOME_JOINTS)[:3, 3], [.307, 0, .590], atol=.001)
    pose = tcp_pose(FRANKA_HOME_JOINTS)
    np.testing.assert_allclose(pose[:3, 3], [.307, 0, .487], atol=.001)
    np.testing.assert_allclose(pose[:3, 2], [0, 0, -1], atol=.001)
    np.testing.assert_allclose(tcp_position(FRANKA_HOME_JOINTS), pose[:3, 3])


@pytest.mark.parametrize('model', ['panda', 'fr3'])
def test_independent_fk_on_random_datasheet_configurations(model):
    bounds = np.array(DATASHEET_JOINT_LIMITS[model])
    for q in np.random.default_rng(42).uniform(bounds[:, 0], bounds[:, 1], size=(20, 7)):
        expected = independent_chain(q)
        np.testing.assert_allclose(flange_pose(q, model), expected, atol=1e-9, rtol=0)
        np.testing.assert_allclose(tcp_pose(q, model=model), expected @ FLANGE_T_HAND, atol=1e-9, rtol=0)


@pytest.mark.parametrize('q,model', [([0]*6, 'panda'), ([np.nan]*7, 'panda'), ([0]*7, 'other')])
def test_invalid_fk_inputs(q, model):
    with pytest.raises(ValueError):
        flange_pose(q, model)


@pytest.mark.parametrize('tool', [np.eye(3), np.zeros((4, 4)), np.diag([-1, 1, 1, 1]), np.full((4, 4), np.nan)])
def test_invalid_tool(tool):
    with pytest.raises(ValueError, match='flange_T_ee'):
        tcp_pose([0]*7, tool)
