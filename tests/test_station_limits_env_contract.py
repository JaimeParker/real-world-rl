"""Resolved limits, legacy checkpoint comparison, and SERL clipping regressions."""
from copy import deepcopy

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from _station_limits import station_limits, env_limits, tcp_box
from test_real_robot_env import _Rig
from real_world_rl.contracts.limits import checkpoint_limits, legacy_limits_match
from real_world_rl.devices.limits import ArmLimits, TcpBox
from real_world_rl.envs.real_robot import RealRobotEnvConfig
from real_world_rl.envs.real_robot.limits import merge_limits
from real_world_rl.envs.serl_cartesian import clip_safety_box, target_pose


@pytest.mark.parametrize('key', ['safety_box', 'workspace', 'joint_limits'])
def test_old_env_keys_fail_with_migration_hint(key):
    with pytest.raises(ValueError, match='migrate.*limits'):
        RealRobotEnvConfig(**{key: None})


def test_override_preserves_frame_margin_steps_and_kinematics():
    box = TcpBox([.2, .3, .4], Rotation.from_euler('z', .5).as_matrix(), [-1]*3, [1]*3, .007)
    device = ArmLimits(joints=[[-2, 2]]*7, tcp=box, max_delta_xyz_m=.03).to_dict()
    result = merge_limits(device, env_limits(joints=[[-1, 1]]*7, bounds=[-.5]*3+[.5]*3))
    assert result.tcp.measured_margin_m == .007 and result.max_delta_xyz_m == .03
    np.testing.assert_array_equal(result.tcp.axes_R, box.axes_R)
    np.testing.assert_array_equal(result.tcp.origin_xyz, box.origin_xyz)
    assert result.to_dict()['version'] == 'station_limits_v1'
    with pytest.raises(ValueError, match='looser'):
        merge_limits(device, env_limits(bounds=[-2]*3+[2]*3))


@pytest.mark.parametrize('rotated', [False, True])
def test_serl_identity_bit_exact_and_rotated_box(rotated, monkeypatch, tmp_path):
    import real_world_rl.devices.claims as claims
    monkeypatch.setattr(claims, '_DEFAULT_LOCK_DIR', tmp_path / 'claims')
    axes = Rotation.from_euler('z', .6).as_matrix() if rotated else np.eye(3)
    limits = station_limits(bounds=[-.01]*3+[.01]*3)
    limits['tcp']['axes_R'] = axes.tolist()
    transform = np.eye(4)
    transform[:3, :3] = axes
    rig = _Rig('local', limits=limits, task_frame={'name': 'table', 'base_T_frame': transform.tolist()})
    try:
        low, high = [0, -.02, -.03], [.1, .02, .03]
        env = rig.env(cartesian_semantics='serl', safety_box_rpy_low=low, safety_box_rpy_high=high)
        env.reset()
        measured = env._last_raw['arm']['tcp_pose'].copy()
        action = np.array([1., 1., 1., 1., 1., 0., 0.])
        requested = target_pose(measured, action, env.config.xyz_scale_m, env.config.rot_scale_rad)
        box = env._effective_limits.tcp
        if rotated:
            xyz = box.clip(requested[:3])
            expected = clip_safety_box(requested, xyz, xyz, low, high)
        else:
            bounds = box.absolute_aabb()
            expected = clip_safety_box(requested, bounds[:3], bounds[3:], low, high)
        env.step(action)
        np.testing.assert_array_equal(rig.arm.last_cartesian_target, expected)
    finally:
        rig.close()


def test_legacy_checkpoint_boxes_and_joints_compare_in_absolute_coordinates():
    old = {'safety_box': [.1, -.2, .3, .7, .2, .9], 'joint_limits': [[-1, 1]]*7}
    translated = checkpoint_limits(old)
    box = TcpBox([.3, 0, .5], np.eye(3), [-.2, -.2, -.2], [.4, .2, .4])
    actual = ArmLimits(joints=old['joint_limits'], tcp=box, max_delta_xyz_m=.03).to_dict()
    assert translated['version'] == 'station_limits_v1'
    assert legacy_limits_match(translated, actual)
    changed = deepcopy(actual)
    changed['tcp']['high'][0] += .01
    assert not legacy_limits_match(translated, changed)


@pytest.mark.parametrize('workspace', [
    {'frame': 'table', 'low': [-1]*3, 'high': [1]*3},
    {'low': [-1]*3, 'high': [1]*3, 'orientation_xyzw': [0, 0, 0, 1], 'max_angle_rad': .1},
])
def test_nontranslatable_checkpoint_requires_retraining(workspace):
    with pytest.raises(ValueError, match='incompatible; retrain'):
        checkpoint_limits({'workspace': workspace})


def test_base_workspace_translates_and_intersects_legacy_box():
    translated = checkpoint_limits({'safety_box': [-2]*3+[2]*3,
                                    'workspace': {'frame': 'base', 'low': [-1]*3, 'high': [1]*3}})
    np.testing.assert_array_equal(ArmLimits.from_dict(translated).tcp.absolute_aabb(), [-1]*3+[1]*3)


def test_checkpoint_config_keeps_absolute_legacy_constraints_for_connect():
    from real_world_rl.deploy.checkpoint import _real_robot_config_from_saved
    config = _real_robot_config_from_saved({
        'remote': 'robot:50051', 'action_mode': 'joint_abs',
        'joint_limits': [[-1, 1]]*7, 'safety_box': [-1]*3+[1]*3,
    })
    assert config.limits == {'joints': [[-1, 1]]*7}
    assert config._checkpoint_limits['tcp']['axes_R'] == np.eye(3).tolist()


def test_legacy_experiment_contract_translates_before_comparison():
    from real_world_rl.contracts.action import validate_transitions_against_contract
    limits = ArmLimits(joints=[[-1, 1]]*7, tcp=tcp_box([-1]*3+[1]*3)).to_dict()
    actual = {'action_mode': 'cartesian_delta', 'arm_action_dim': 6, 'gripper_in_action': True,
              'joint_limits': [[-1, 1]]*7, 'limits': limits,
              'experiment': {'limits': limits}}
    old = deepcopy(actual)
    old.pop('limits')
    old['experiment'] = {'safety_box': [-1]*3+[1]*3, 'joint_limits': [[-1, 1]]*7}
    validate_transitions_against_contract([], actual, saved=old, source='legacy')
    old['experiment']['safety_box'][0] = -2
    with pytest.raises(ValueError, match='incompatible'):
        validate_transitions_against_contract([], actual, saved=old, source='legacy')


def test_remote_checkpoint_comparison_resolves_daemon_before_motion(monkeypatch, tmp_path):
    import real_world_rl.devices.claims as claims
    from real_world_rl.contracts.action import experiment_contract_of, validate_experiment
    monkeypatch.setattr(claims, '_DEFAULT_LOCK_DIR', tmp_path / 'claims')
    rig = _Rig('remote', limits=station_limits(bounds=[-1]*3+[1]*3))
    try:
        trained = rig.env()
        trained._connect_and_validate()
        saved = experiment_contract_of(trained)
        deployed = rig.env()
        validate_experiment(deployed, saved)
        assert deployed._validated
        assert not any(call.startswith(('move_', 'servo_')) for call in rig.arm.call_log)
    finally:
        rig.close()


def test_legacy_deploy_mismatch_faults_before_reset_motion(monkeypatch, tmp_path):
    from types import SimpleNamespace
    import real_world_rl.devices.claims as claims
    from real_world_rl.deploy.checkpoint import (
        DeployArgs, _real_robot_config_from_saved, build_deploy_env,
    )
    from real_world_rl.devices.errors import DeviceFault
    monkeypatch.setattr(claims, '_DEFAULT_LOCK_DIR', tmp_path / 'claims')
    trained = _real_robot_config_from_saved({'safety_box': [-1]*3+[1]*3})
    cli = RealRobotEnvConfig(robot={
        'type': 'franka', 'arm_backend': 'fake', 'gripper': {'backend': 'fake'},
        'limits': station_limits(bounds=[-2]*3+[2]*3),
    })
    env = build_deploy_env(DeployArgs(checkpoint='unused', real_robot=cli, enable_teleop=False),
                           SimpleNamespace(real_robot=trained))
    try:
        with pytest.raises(DeviceFault, match='checkpoint limits'):
            env.reset()
        backend = env
        while getattr(backend, 'env', None) is not None:
            backend = backend.env
        assert not any(call.startswith(('move_', 'servo_')) for call in backend.robot.arm.call_log)
    finally:
        env.close()
