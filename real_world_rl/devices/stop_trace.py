"""Offline acceptance of sampled normal-stop evidence (never a motion command).

PASS describes the supplied samples, not a continuous-time physical guarantee.
Missing coverage/freshness cannot be repaired by interpolation or smoothing.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass
import math

import numpy as np

from .workspace import Workspace, quaternion_matrix, task_transform


@dataclass(frozen=True)
class StopThresholds:
    displacement_m: float = .010
    angle_deg: float = 5.
    joint_speed_rad_s: float = .02
    stationary_s: float = .5
    post_stop_s: float = 2.
    max_sample_gap_s: float = .1

    def __post_init__(self):
        for key, value in asdict(self).items():
            if isinstance(value, bool) or not math.isfinite(value) or value <= 0:
                raise ValueError(f"{key} must be finite and positive")


def evaluate_stop_trace(document, thresholds=None):
    """Evaluate schema-1 samples with host/device seconds, TCP xyzw, dq, errors.

    ``stop_start_s``/``stop_end_s`` bracket the actual stop call on the host
    monotonic clock. Every sample must carry ``monotonic_s``, ``device_time_s``,
    ``tcp_pose`` (7), ``joint_vel`` (all joints), ``robot_mode`` and ``errors`` (list).
    The start boundary also records ``control_mode`` and ``tracker_active``. Optional
    workspace/task_frame use the ordinary robot configuration schema. A failed
    stop or any measured limit violation yields FAIL even if coverage is missing;
    missing evidence alone yields BLOCKED. Raw samples remain the caller's record.
    """
    limits = thresholds or StopThresholds()
    if not isinstance(document, dict) or document.get('schema_version') != 1:
        raise ValueError('stop trace requires schema_version 1')
    start, end = float(document['stop_start_s']), float(document['stop_end_s'])
    if not math.isfinite(start) or not math.isfinite(end) or end < start:
        raise ValueError('invalid stop interval')
    joint_count = document.get('joint_count', 7)
    if type(joint_count) is not int or joint_count < 1:
        raise ValueError('joint_count must be a positive integer')
    samples = document.get('samples', [])
    blocked, failed = [], []
    result = {'schema_version': 1, 'thresholds': asdict(limits), 'metrics': {},
              'failed': failed, 'missing_evidence': blocked,
              'scope': 'sampled normal-stop evidence; no continuous-time guarantee'}
    if document.get('control_mode') not in ('cartesian_servo', 'joint_servo') or document.get('tracker_active') is not True:
        blocked.append('no active motion at stop')
    if document.get('sampling_errors'):
        blocked.append('sampling reported errors: ' + '; '.join(map(str, document['sampling_errors'])))
    if document.get('stop_error'):
        failed.append('stop reported an error')
    workspace = document.get('workspace')
    frame = document.get('task_frame')
    transform = task_transform(frame) if frame is not None else None
    geometry = Workspace(workspace, document.get('task_frame')) if workspace else None
    if geometry is None:
        blocked.append('no confirmed workspace supplied')
    if not samples:
        blocked.append('no samples')
        result['status'] = 'FAIL' if failed else 'BLOCKED'
        return result
    times, device_times, poses, speeds = [], [], [], []
    for row in samples:
        host = float(row['monotonic_s'])
        device = row.get('device_time_s')
        pose = np.asarray(row['tcp_pose'], dtype=float)
        dq = np.asarray(row['joint_vel'], dtype=float)
        if (not math.isfinite(host) or pose.shape != (7,) or not np.isfinite(pose).all()
                or dq.shape != (joint_count,) or not np.isfinite(dq).all()):
            raise ValueError('invalid stop sample')
        quaternion_matrix(pose[3:])
        if device is None or not math.isfinite(float(device)):
            device_times.append(float('nan'))
        else:
            device_times.append(float(device))
        errors = row.get('errors')
        if not isinstance(errors, list):
            if 'sample errors unavailable' not in blocked:
                blocked.append('sample errors unavailable')
        elif errors and 'control fault in samples' not in failed:
            failed.append('control fault in samples')
        mode = row.get('robot_mode')
        if mode not in ('Move', 'Idle', 'Reflex', 'UserStopped', 'Guiding', 'AutomaticErrorRecovery') and 'sample robot_mode unavailable' not in blocked:
            blocked.append('sample robot_mode unavailable')
        elif mode in ('Reflex', 'UserStopped', 'Guiding', 'AutomaticErrorRecovery') and 'robot mode fault in samples' not in failed:
            failed.append('robot mode fault in samples')
        if geometry and geometry.violations(pose, measured=True) and 'measured TCP outside workspace' not in failed:
            failed.append('measured TCP outside workspace')
        times.append(host)
        poses.append(pose)
        speeds.append(float(np.max(np.abs(dq))))
    before_modes = [row.get('robot_mode') for row in samples if row['monotonic_s'] <= start]
    if before_modes and before_modes[-1] == 'Idle' and 'no active motion at stop' not in blocked:
        blocked.append('no active motion at stop')
    times, poses, speeds = np.asarray(times), np.asarray(poses), np.asarray(speeds)
    intervals = np.diff(times)
    if np.any(intervals <= 0):
        raise ValueError('host sample times must strictly increase')
    device_intervals = np.diff(device_times)
    if not np.isfinite(device_times).all() or np.any(device_intervals <= 0):
        blocked.append('device timestamps missing or not advancing')
    max_gap = float(np.max(intervals)) if intervals.size else None
    if max_gap is None or max_gap > limits.max_sample_gap_s + 1e-9:
        blocked.append('sample coverage has gaps')
    before = np.flatnonzero(times <= start)
    post = np.flatnonzero(times >= end)
    if (not before.size or start - times[before[-1]] > limits.max_sample_gap_s
            or times[0] > start - limits.stationary_s + 1e-9):
        blocked.append('insufficient pre-stop coverage')
    pre_window = (times >= start - limits.stationary_s - 1e-9) & (times <= start)
    if not np.any(pre_window) or np.any(speeds[pre_window] > limits.joint_speed_rad_s + 1e-9):
        blocked.append('pre-stop stationarity not established')
    if not post.size or times[-1] < end + limits.post_stop_s - 1e-9:
        blocked.append('insufficient post-stop observation')
    final_window = (times >= times[-1] - limits.stationary_s - 1e-9) & (times >= end)
    if times[-1] - end < limits.stationary_s - 1e-9:
        blocked.append('insufficient final stationary window')
    elif np.any(speeds[final_window] > limits.joint_speed_rad_s + 1e-9):
        failed.append('final joint speed exceeds threshold')
    metrics = result['metrics']
    metrics.update(sample_count=len(samples), max_sample_gap_s=max_gap,
                   post_stop_duration_s=max(0., float(times[-1] - end)),
                   final_window_max_joint_speed_rad_s=float(np.max(speeds[final_window])) if np.any(final_window) else None)
    if before.size:
        reference = poses[before[-1]]
        measured = poses[times >= times[before[-1]]]
        displacement = float(np.max(np.linalg.norm(measured[:, :3] - reference[:3], axis=1)))
        rotation = quaternion_matrix(reference[3:])
        angles = [math.degrees(math.acos(float(np.clip((np.trace(rotation.T @ quaternion_matrix(p[3:])) - 1) / 2, -1, 1)))) for p in measured]
        angle = max(angles)
        metrics.update(max_displacement_m=displacement, max_angle_deg=angle,
                       reference_monotonic_s=float(times[before[-1]]))
        delta = measured[:, :3] - reference[:3]
        metrics['base_displacement'] = {
            'min_xyz_m': delta.min(axis=0).tolist(),
            'max_xyz_m': delta.max(axis=0).tolist(),
            'final_xyz_m': delta[-1].tolist(),
        }
        if transform is None:
            metrics['task_displacement'] = None
            metrics['task_displacement_unavailable_reason'] = 'no task_frame supplied; base Z is not assumed vertical'
        else:
            # Row vectors: inverse rotation is right multiplication by R_B_T.
            # Translation cancels for displacement relative to the reference pose.
            task_delta = delta @ transform[:3, :3]
            metrics['task_displacement'] = {
                'frame': frame['name'], 'units': 'm',
                'min_xyz_m': task_delta.min(axis=0).tolist(),
                'max_xyz_m': task_delta.max(axis=0).tolist(),
                'final_xyz_m': task_delta[-1].tolist(),
                'max_downward_z_m': float(max(0., -task_delta[:, 2].min())),
                'max_upward_z_m': float(max(0., task_delta[:, 2].max())),
                'max_horizontal_displacement_m': float(np.linalg.norm(task_delta[:, :2], axis=1).max()),
                'interpretation': 'Z of supplied task frame; physical vertical calibration requires operator verification',
            }
        if displacement > limits.displacement_m + 1e-9:
            failed.append('sampled displacement exceeds threshold')
        if angle > limits.angle_deg + 1e-9:
            failed.append('sampled rotation exceeds threshold')
    result['status'] = 'FAIL' if failed else 'BLOCKED' if blocked else 'PASS'
    return result
