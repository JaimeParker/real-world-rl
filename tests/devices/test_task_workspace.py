import numpy as np
import pytest

from real_world_rl.devices.workspace import Workspace, task_transform
from real_world_rl.devices.safety import ArmSafety
from real_world_rl.devices.errors import SafetyViolation


def frame():
    return {"name": "table", "base_T_frame": [[1, 0, 0, .5], [0, .8, -.6, 0],
                                              [0, .6, .8, .2], [0, 0, 0, 1]]}


def test_oriented_box_and_serialization():
    spec = {"frame": "table", "low": [-.1]*3, "high": [.1]*3}
    guard = ArmSafety(task_frame=frame(), workspace=spec)
    pose = np.array([.5, 0, .2, 0, 0, 0, 1.])
    guard.require_pose_inside(pose)
    assert not guard.is_unlimited
    clone = ArmSafety.from_dict(guard.to_dict())
    pose[:3] += task_transform(frame())[:3, :3] @ [0, 0, .2]
    with pytest.raises(SafetyViolation, match="table.z"):
        clone.clip_pose(pose)


def test_base_clipping_cannot_escape_task_box():
    guard = ArmSafety(safety_box=[.8, -1, -1, 1, 1, 1], task_frame=frame(),
                      workspace={"frame": "table", "low": [-.1]*3, "high": [.1]*3})
    with pytest.raises(SafetyViolation):
        guard.clip_pose(np.array([.5, 0, .2, 0, 0, 0, 1.]))


def test_quaternion_sign_and_rotation_limit():
    workspace = Workspace({"low": [-1]*3, "high": [1]*3,
                           "orientation_xyzw": [0, 0, 0, 1], "max_angle_rad": .2})
    assert workspace.violations([0, 0, 0, 0, 0, 0, -1]) == []
    assert workspace.violations([0, 0, 0, 0, 0, 1, 0])


def test_reflection_and_unknown_frames_rejected():
    bad = frame()
    bad["base_T_frame"][0][0] = -1
    with pytest.raises(ValueError):
        task_transform(bad)
    with pytest.raises(ValueError):
        Workspace({"frame": "missing", "low": [-1]*3, "high": [1]*3}, frame())


def test_spatial_contract_is_numeric_and_legacy_is_empty():
    from real_world_rl.devices.workspace import spatial_contract
    assert spatial_contract() == {}
    task = frame()
    spec = {'frame': 'table', 'low': [-1]*3, 'high': [1]*3,
            'orientation_xyzw': [0, 0, 0, 1], 'max_angle_rad': .4}
    contract = spatial_contract(spec, task)
    renamed = {**task, 'name': 'desk', 'source': '/different/file'}
    equivalent = {**spec, 'frame': 'desk', 'orientation_xyzw': [0, 0, 0, -1]}
    assert spatial_contract(equivalent, renamed) == contract
    assert spatial_contract({**spec, 'high': [2]*3}, task) != contract


def test_measured_margin_does_not_relax_target_checks():
    workspace = Workspace({'low': [-1]*3, 'high': [1]*3,
                           'orientation_xyzw': [0, 0, 0, 1], 'max_angle_rad': .2})
    pose = [1.001, 0, 0, 0, 0, np.sin(.21/2), np.cos(.21/2)]
    assert len(workspace.violations(pose)) == 2
    assert workspace.violations(pose, measured=True) == []
    pose[0] = 1.004
    assert workspace.violations(pose, measured=True)
    pose[0] = 0
    pose[5:] = [np.sin(.24/2), np.cos(.24/2)]
    assert workspace.violations(pose, measured=True)


@pytest.mark.parametrize('value', [-1, float('nan'), True])
@pytest.mark.parametrize('key', ['measured_margin_m', 'measured_margin_rad'])
def test_invalid_measured_margin_rejected(key, value):
    with pytest.raises(ValueError, match=key):
        Workspace({'low': [-1]*3, 'high': [1]*3, key: value})


def test_workspace_projection_uses_task_axes_and_reference_cone():
    from real_world_rl.devices.workspace import quaternion_matrix
    spec = {'frame': 'table', 'low': [-.1]*3, 'high': [.1]*3,
            'orientation_xyzw': [0, 0, 0, 1], 'max_angle_rad': .2}
    geometry = Workspace(spec, frame())
    transform = task_transform(frame())
    # A pi rotation is the difficult SO(3) branch; use a matrix and sign-independent comparison.
    pose = np.array([*(transform[:3, 3] + transform[:3, :3] @ [.4, -.3, 0]), 1, 0, 0, 0])
    limited = geometry.clip_pose(pose)
    np.testing.assert_allclose(geometry.position(limited[:3]), [.1, -.1, 0], atol=1e-12)
    relative = geometry.reference.T @ geometry.rotation.T @ quaternion_matrix(limited[3:])
    assert np.arccos(np.clip((np.trace(relative)-1)/2, -1, 1)) == pytest.approx(.2)
    assert not geometry.violations(limited)


def test_arm_servo_workspace_rejection_is_counted_and_never_sent():
    from real_world_rl.devices.backends.arms.fake import FakeArm
    arm = FakeArm(safety=ArmSafety(workspace={'low': [-1]*3, 'high': [1]*3}))
    arm.connect()
    try:
        with pytest.raises(SafetyViolation, match='workspace'):
            arm.servo_cartesian([2, 0, 0, 0, 0, 0, 1])
        assert arm.health().safety_rejects == 1
        assert 'servo_cartesian' not in arm.call_log
        assert arm.last_cartesian_target is None
    finally:
        arm.disconnect()


def test_frame_without_workspace_is_validated_but_has_no_semantics():
    from real_world_rl.devices.workspace import spatial_contract
    assert spatial_contract(task_frame=frame()) == {}
    bad = frame()
    bad['base_T_frame'][3][0] = 1
    with pytest.raises(ValueError, match='rigid transform'):
        spatial_contract(task_frame=bad)
