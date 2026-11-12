import copy

import numpy as np
import pytest

from robot_infra.devices.stop_trace import StopThresholds, evaluate_stop_trace


def trace():
    return {'schema_version': 1, 'stop_start_s': 1., 'stop_end_s': 2.,
            'control_mode': 'cartesian_servo', 'tracker_active': True,
            'workspace': {'frame': 'base', 'low': [-1]*3, 'high': [1]*3},
            'samples': [{'monotonic_s': float(t), 'device_time_s': float(t + 10),
                         'tcp_pose': [0, 0, 0, 0, 0, 0, 1], 'joint_vel': [0]*7,
                         'robot_mode': 'Move' if t <= 1 else 'Idle', 'errors': []} for t in np.linspace(.5, 4., 351)]}


def test_complete_stationary_trace_passes():
    result = evaluate_stop_trace(trace())
    assert result['status'] == 'PASS'
    assert result['metrics']['post_stop_duration_s'] == 2.


def test_endpoint_snapshots_cannot_pass():
    data = trace()
    data['samples'] = [data['samples'][50], data['samples'][-1]]
    result = evaluate_stop_trace(data)
    assert result['status'] == 'BLOCKED'
    assert 'sample coverage has gaps' in result['missing_evidence']


@pytest.mark.parametrize('offset,expected', [(.010, 'PASS'), (.0101, 'FAIL')])
def test_displacement_in_any_direction_including_mid_stop_peak(offset, expected):
    data = trace()
    data['samples'][100]['tcp_pose'][0] = offset
    assert evaluate_stop_trace(data)['status'] == expected


@pytest.mark.parametrize('degrees,expected', [(5., 'PASS'), (5.1, 'FAIL')])
def test_so3_rotation_including_quaternion_sign(degrees, expected):
    data = trace()
    radians = np.deg2rad(degrees)
    data['samples'][100]['tcp_pose'][3:] = [0, 0, -np.sin(radians/2), -np.cos(radians/2)]
    assert evaluate_stop_trace(data)['status'] == expected


def test_stale_device_time_and_missing_workspace_block():
    data = trace()
    del data['workspace']
    for row in data['samples']:
        row['device_time_s'] = 10
    result = evaluate_stop_trace(data)
    assert result['status'] == 'BLOCKED'
    assert len(result['missing_evidence']) == 2


def test_final_speed_uses_raw_peak_without_smoothing():
    data = trace()
    data['samples'][-10]['joint_vel'][3] = .0201
    assert evaluate_stop_trace(data)['status'] == 'FAIL'


def test_fault_or_workspace_breach_fail_even_with_poor_coverage():
    data = trace()
    data['samples'][100]['errors'] = ['reflex']
    data['samples'][100]['tcp_pose'][1] = 2
    del data['samples'][101:200]
    result = evaluate_stop_trace(data)
    assert result['status'] == 'FAIL'
    assert 'measured TCP outside workspace' in result['failed']
    assert result['missing_evidence']


def test_nonfinite_and_reversed_times_rejected():
    data = trace()
    data['samples'][10]['monotonic_s'] = 0
    with pytest.raises(ValueError, match='strictly increase'):
        evaluate_stop_trace(data)
    with pytest.raises(ValueError):
        StopThresholds(max_sample_gap_s=float('nan'))


def test_input_not_modified():
    data = trace()
    original = copy.deepcopy(data)
    evaluate_stop_trace(data)
    assert data == original


def test_task_vertical_displacement_uses_rotation_not_base_z():
    data = trace()
    # Tilt 90 degrees about X: task up is base -Y.
    transform = [[1, 0, 0, 2], [0, 0, -1, 3], [0, 1, 0, 4], [0, 0, 0, 1]]
    data['task_frame'] = {'name': 'table', 'base_T_frame': transform}
    data['samples'][100]['tcp_pose'][:3] = [0, .009, 0]
    data['samples'][-1]['tcp_pose'][:3] = [.002, .003, 0]
    result = evaluate_stop_trace(data)
    assert result['status'] == 'PASS'
    metric = result['metrics']['task_displacement']
    assert metric['max_downward_z_m'] == pytest.approx(.009)
    assert metric['max_upward_z_m'] == 0
    assert metric['max_horizontal_displacement_m'] == pytest.approx(.002)
    assert metric['final_xyz_m'] == pytest.approx([.002, 0, -.003])
    assert result['metrics']['max_displacement_m'] == pytest.approx(.009)


def test_missing_task_frame_does_not_claim_base_z_is_vertical():
    result = evaluate_stop_trace(trace())
    assert result['status'] == 'PASS'
    assert result['metrics']['task_displacement'] is None
    assert 'not assumed vertical' in result['metrics']['task_displacement_unavailable_reason']


def test_idle_stop_is_blocked_even_with_complete_stationary_samples():
    data = trace()
    data.update(control_mode='idle', tracker_active=False)
    for row in data['samples']:
        row['robot_mode'] = 'Idle'
    result = evaluate_stop_trace(data)
    assert result['status'] == 'BLOCKED'
    assert 'no active motion at stop' in result['missing_evidence']


def test_sample_mode_can_block_or_fail_despite_active_control_metadata():
    data = trace()
    for row in data['samples']:
        row['robot_mode'] = 'Idle'
    assert 'no active motion at stop' in evaluate_stop_trace(data)['missing_evidence']
    data['samples'][100]['robot_mode'] = 'Reflex'
    result = evaluate_stop_trace(data)
    assert result['status'] == 'FAIL'
    assert 'robot mode fault in samples' in result['failed']
