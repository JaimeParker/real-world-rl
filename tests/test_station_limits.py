
from _station_limits import station_limits, env_limits, tcp_box
from copy import deepcopy

import numpy as np
import pytest

from real_world_rl.devices import ArmLimits, TcpBox, RobotConfig, SafetyViolation
from real_world_rl.devices.backends.arms.fake import FakeArm
from real_world_rl.devices.robots.franka import FrankaConfig, DATASHEET_JOINT_LIMITS, FRANKA_HOME_JOINTS
from real_world_rl.devices.robots.franka_kinematics import tcp_position
from real_world_rl.devices.robots.piper import PiperConfig


def config_data():
    return {'limits': {'joints': [list(pair) for pair in DATASHEET_JOINT_LIMITS['panda']],
                       'tcp': {'origin': 'home', 'origin_xyz': tcp_position(FRANKA_HOME_JOINTS).tolist(),
                               'axes_R': np.eye(3).tolist(), 'low': [-.2]*3, 'high': [.2]*3}}}


def box():
    return TcpBox([0, 0, 0], np.eye(3), [-1]*3, [1]*3)


def test_schema_wire_and_read_only_convenience():
    config = FrankaConfig(**config_data())
    limits = ArmLimits.from_config(config)
    assert ArmLimits.from_dict(limits.to_dict()).to_dict() == limits.to_dict()
    assert limits.to_dict()['version'] == 'station_limits_v1'
    assert limits.to_dict()['kinematics']['model'] == 'panda'
    assert config.joint_limits == config_data()['limits']['joints']
    with pytest.raises(AttributeError):
        config.joint_limits = []
    config.joint_limits[0][0] = 10
    assert config.joint_limits[0][0] < 0
    assert ArmLimits.from_config(PiperConfig()) is None
    assert RobotConfig().kinematics() is None


@pytest.mark.parametrize('key', ['safety_box', 'workspace', 'joint_limits', 'max_delta_per_step'])
def test_migration_error_names_key_and_command(key):
    with pytest.raises(ValueError, match=key + '.*rwrl limits migrate.*docs'):
        FrankaConfig.from_dict({key: None})


@pytest.mark.parametrize('patch,match', [
    ({'joints': [[-1, 1]]*6}, '7 pairs'),
    ({'joints': [[-1, 0, 1]]*7}, 'pairs'),
    ({'joints': [[1, -1]]*7}, 'lo <= hi'),
    ({'joints': [[-1, np.inf]]*7}, 'finite'),
    ({'joints': [[False, 1]]*7}, 'finite'),
    ({'joints': [[-1, 1]]*7}, 'home_joints outside'),
    ({'bogus': 1}, 'unknown limits'),
    ({'max_delta_per_step': {'other': 1}}, 'unknown max_delta'),
    ({'max_delta_per_step': []}, 'mapping'),
])
def test_invalid_limit_schema(patch, match):
    data = config_data()
    data['limits'].update(patch)
    with pytest.raises(ValueError, match=match):
        FrankaConfig(**data)


@pytest.mark.parametrize('value', [0, -1, True, '1', np.nan, np.inf, None])
@pytest.mark.parametrize('key', ['xyz_m', 'joint_rad'])
def test_invalid_step_limit(value, key):
    data = config_data()
    data['limits']['max_delta_per_step'] = {key: value}
    with pytest.raises(ValueError):
        FrankaConfig(**data)


@pytest.mark.parametrize('patch,match', [
    ({'origin': 'other'}, 'origin'),
    ({'origin_xyz': None}, 'origin_xyz'),
    ({'origin_xyz': [0, 0, 0]}, 'home_joints changed'),
    ({'axes_R': np.diag([-1, 1, 1]).tolist()}, 'proper rotation'),
    ({'axes_R': [[0, -1, 0], [1, 0, 0], [0, 0, 1]]}, 'task_frame changed'),
    ({'axes_R': np.zeros((3, 3)).tolist()}, 'proper rotation'),
    ({'axes_R': None}, 'axes_R'),
    ({'low': [0, -1, -1]}, 'low < 0'),
    ({'high': [1, 0, 1]}, 'low < 0'),
    ({'low': [-1, -1]}, 'shape'),
    ({'high': [np.nan]*3}, 'finite'),
    ({'orientation_xyzw': [0, 0, 0, 1]}, 'unknown limits.tcp'),
    ({'origin': [0, 0, 0]}, 'must be equal'),
    ({'origin': None}, 'origin'),
])
def test_invalid_tcp_schema(patch, match):
    data = config_data()
    data['limits']['tcp'].update(patch)
    with pytest.raises(ValueError, match=match):
        FrankaConfig(**data)


@pytest.mark.parametrize('value', [-1, True, np.nan, np.inf, '0', None])
def test_invalid_margin(value):
    with pytest.raises(ValueError, match='measured_margin_m'):
        TcpBox([0]*3, np.eye(3), [-1]*3, [1]*3, value)


def test_home_requires_kinematics_and_home():
    data = config_data()
    with pytest.raises(ValueError, match='requires home_joints and kinematics'):
        FrankaConfig(home_joints=None, **data)
    data['limits'].pop('joints')
    with pytest.raises(ValueError, match='explicit origin'):
        PiperConfig(**data)
    data['limits']['tcp'].update(origin=[0, 0, 0], origin_xyz=[0, 0, 0])
    assert ArmLimits.from_config(PiperConfig(**data)).tcp.contains([0, 0, 0])
    data['limits']['tcp'].pop('origin_xyz')
    assert ArmLimits.from_config(PiperConfig(**data)).tcp.contains([0, 0, 0])


def test_frozen_axes_ignore_task_origin_and_home_attitude():
    data = config_data()
    rotation = [[0, -1, 0], [1, 0, 0], [0, 0, 1]]
    matrix = np.eye(4)
    matrix[:3, :3] = rotation
    matrix[:3, 3] = [5, 6, 7]
    data['task_frame'] = {'name': 'table', 'base_T_frame': matrix.tolist()}
    data['limits']['tcp']['axes_R'] = rotation
    tcp = ArmLimits.from_config(FrankaConfig(**data)).tcp
    np.testing.assert_allclose(tcp.origin_xyz, tcp_position(FRANKA_HOME_JOINTS))
    np.testing.assert_allclose(tcp.axes_R, rotation)
    data['home_joints'] = list(FRANKA_HOME_JOINTS)
    data['home_joints'][0] += .1
    with pytest.raises(ValueError, match='home_joints changed'):
        FrankaConfig(**data)


def test_rotated_box_math_and_clip():
    rotation = np.array([[.8, -.6, 0], [.6, .8, 0], [0, 0, 1]])
    tcp = TcpBox([2, 3, 4], rotation, [-.2, -.3, -.4], [.2, .3, .4])
    xyz = tcp.origin_xyz + rotation @ [.5, -.7, .1]
    np.testing.assert_allclose(tcp.to_box(xyz), [.5, -.7, .1])
    np.testing.assert_allclose(tcp.to_box(tcp.clip(xyz)), [.2, -.3, .1], atol=1e-15)
    assert tcp.distance(xyz) == pytest.approx(.5)
    assert not tcp.contains(xyz)
    assert tcp.absolute_aabb() is None
    assert 'box frame' in tcp.violations(xyz)[0]
    np.testing.assert_array_equal(box().absolute_aabb(), [-1]*3+[1]*3)
    limits = ArmLimits(tcp=tcp)
    pose, clipped = limits.clip_pose([*xyz, .5, .5, .5, .5])
    assert clipped
    np.testing.assert_array_equal(pose[3:], [.5]*4)
    assert not limits.clip_pose([2, 3, 4, 0, 0, 0, 1])[1]


def test_measured_margin_only_applies_to_feedback():
    limits = ArmLimits(tcp=box())
    assert limits.measured_violations([1.002, 0, 0]) == []
    assert limits.measured_violations([1.004, 0, 0])
    with pytest.raises(SafetyViolation):
        limits.require_pose_inside([1.002, 0, 0, 0, 0, 0, 1])


def test_projection_wall_and_outside_recovery():
    limits = ArmLimits(tcp=box())
    fk = lambda q: np.array([q[0], 0, 0])
    target, clipped = limits.project_joints_to_box([.5], [0], fk)
    np.testing.assert_array_equal(target, [.5])
    assert not clipped
    target, clipped = limits.project_joints_to_box([2], [0], fk)
    assert clipped and 0 <= 1-target[0] < 1e-4
    for desired in (.5, 1.5):
        target, clipped = limits.project_joints_to_box([desired], [2], fk)
        assert target[0] == desired and not clipped
    for desired in (2, 3, -3):
        target, clipped = limits.project_joints_to_box([desired], [2], fk)
        assert target[0] == 2 and clipped == (desired != 2)
    assert not limits.project_joints_to_box([2], [0], lambda q: None)[1]


def test_arm_order_projection_rejection_and_counters():
    arm = FakeArm(num_joints=1, safety=ArmLimits(joints=[[-3, 3]], tcp=box(), max_delta_joint_rad=2))
    arm.tcp_position_from_joints = lambda q: np.array([q[0], 0, 0])
    arm.connect()
    try:
        arm.servo_joints([10])
        assert 0 <= 1-arm._joint_target[0] < 1e-4
        assert arm.health().safety_clips == 1
        for q in ([2], [4]):
            with pytest.raises(SafetyViolation):
                arm.move_to_joints(q)
        with pytest.raises(SafetyViolation):
            arm.move_to_pose([2, 0, 0, 0, 0, 0, 1])
        assert arm.health().safety_rejects == 3
        assert 'move_to_joints' not in arm.call_log
        # An outside measured joint state distinguishes joint-clip -> step-limit order.
        arm.stop()
        arm._joint_pos[:] = 10
        arm._forget_state()
        arm.servo_joints([20])
        assert arm._joint_target[0] == 8
    finally:
        arm.disconnect()


def test_real_fk_projection_and_fake_hook():
    home = np.array(FRANKA_HOME_JOINTS)
    tcp = TcpBox(tcp_position(home), np.eye(3), [-.02]*3, [.02]*3)
    limits = ArmLimits(tcp=tcp)
    target = home.copy()
    target[0] += .8
    result, clipped = limits.project_joints_to_box(target, home, tcp_position)
    assert clipped and tcp.contains(tcp_position(result))
    assert np.min(tcp.high - tcp.to_box(tcp_position(result))) < 1e-4
    np.testing.assert_allclose(FakeArm().tcp_position_from_joints(home), tcp_position(home))
    assert FakeArm(num_joints=6).tcp_position_from_joints([0]*6) is None


@pytest.mark.parametrize('patch', [{'version': 'old'}, {'other': 1}, {'tcp': {'other': 1}},
                                  {'kinematics': {'model': 'other'}}, {'max_delta_per_step': {'oops': 1}}])
def test_wire_rejects_invalid_data(patch):
    data = ArmLimits(tcp=box()).to_dict()
    data.update(patch)
    with pytest.raises(ValueError):
        ArmLimits.from_dict(data)


@pytest.mark.parametrize('value', [[], 2, 'bad'])
def test_limits_requires_mapping(value):
    with pytest.raises(ValueError, match='mapping'):
        PiperConfig(limits=value)


def test_no_aliasing_of_geometry():
    origin = np.zeros(3)
    tcp = TcpBox(origin, np.eye(3), [-1]*3, [1]*3)
    origin[:] = 2
    assert tcp.contains([0, 0, 0])
    with pytest.raises(ValueError):
        tcp.low[0] = -2


def test_station_loader_validates_new_schema_and_old_keys(tmp_path):
    import yaml
    from real_world_rl.station import load_station
    path = tmp_path / 'station.yaml'
    data = {"limits": station_limits(), 'type': 'franka', 'arm_backend': 'fake', 'gripper': None, **config_data()}
    path.write_text(yaml.safe_dump(data))
    station = load_station(path)
    assert station is not None
    data['workspace'] = None
    path.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError, match='workspace.*rwrl limits migrate'):
        load_station(path)


def test_custom_tcp_transform_is_used_for_home_and_wire():
    from real_world_rl.devices.robots.franka_kinematics import FLANGE_T_HAND
    tool = FLANGE_T_HAND.copy()
    tool[2, 3] += .1
    data = config_data()
    data['flange_T_ee'] = tool.tolist()
    with pytest.raises(ValueError, match='home_joints changed'):
        FrankaConfig(**data)
    data['limits']['tcp']['origin_xyz'] = tcp_position(FRANKA_HOME_JOINTS, tool).tolist()
    config = FrankaConfig(**data)
    limits = ArmLimits.from_config(config)
    assert limits.to_dict()['kinematics']['flange_T_ee'] == tool.tolist()
    arm = FakeArm.from_config(config)
    np.testing.assert_allclose(arm.tcp_position_from_joints(FRANKA_HOME_JOINTS), limits.tcp.origin_xyz)


@pytest.mark.parametrize('offset,accepted', [(.0009, True), (.0011, False)])
def test_frozen_home_tolerance(offset, accepted):
    data = config_data()
    data['limits']['tcp']['origin_xyz'][0] += offset
    if accepted:
        FrankaConfig(**data)
    else:
        with pytest.raises(ValueError, match='home_joints changed'):
            FrankaConfig(**data)


@pytest.mark.parametrize('kwargs', [{'tcp': {}}, {'joints': [1]}, {'max_delta_xyz_m': 0},
                                   {'max_delta_joint_rad': np.inf},
                                   {'kinematics': {'model': 'panda', 'flange_T_ee': None}}])
def test_direct_limits_validation(kwargs):
    with pytest.raises(ValueError):
        ArmLimits(**kwargs)


@pytest.mark.parametrize('robot_model', ['panda', 'fr3'])
def test_franka_without_joint_limits_uses_the_datasheet(robot_model):
    from real_world_rl.devices.robots.franka import DATASHEET_JOINT_LIMITS
    for limits in ({"max_delta_per_step": {"joint_rad": 0.1}}, None):
        resolved = ArmLimits.from_config(FrankaConfig(robot_model=robot_model, limits=limits))
        np.testing.assert_array_equal(resolved.joints, np.asarray(DATASHEET_JOINT_LIMITS[robot_model]))


@pytest.mark.parametrize('value', [[np.nan]*7, [True]*7, [0]*6])
def test_home_shape_and_finiteness(value):
    with pytest.raises(ValueError, match='home_joints'):
        FrankaConfig(home_joints=value, **config_data())
