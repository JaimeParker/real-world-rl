"""Read-only TCP measurements used to propose, never certify, workspace bounds and task frames."""
from __future__ import annotations

import numpy as np
from real_world_rl.devices.workspace import task_transform, quaternion_matrix


def point_set(document, task_frame):
    """Validate frame provenance and derive coordinates from original base poses."""
    frame = task_frame['name'] if task_frame else 'base'
    transform = task_transform(task_frame)
    if document is None:
        document = {'schema_version': 1, 'frame': frame, 'units': 'm',
                    'base_T_frame': transform.tolist(), 'points': {}}
    if (document.get('schema_version') != 1 or document.get('frame') != frame
            or document.get('units') != 'm'):
        raise ValueError('workspace points require schema 1, matching frame and metres')
    saved = np.asarray(document.get('base_T_frame'), dtype=float)
    if saved.shape != (4, 4) or not np.allclose(saved, transform, atol=1e-9, rtol=0):
        raise ValueError('workspace points frame changed; recapture measurements')
    points = document.get('points')
    if not isinstance(points, dict):
        raise ValueError('workspace points must be a mapping of names to measurements')
    positions = []
    for name, measurement in points.items():
        if not isinstance(name, str) or not name.strip() or not isinstance(measurement, dict):
            raise ValueError('workspace point requires a name and base_pose')
        pose = np.asarray(measurement.get('base_pose'), dtype=float)
        if pose.shape != (7,) or not np.isfinite(pose).all():
            raise ValueError('workspace point base_pose must be finite XYZ + xyzw')
        quaternion_matrix(pose[3:])
        positions.append(transform[:3, :3].T @ (pose[:3] - transform[:3, 3]))
    return document, np.asarray(positions)


def point_base_positions(document) -> dict[str, np.ndarray]:
    """Base-frame XYZ of every captured point, checked against the file's own frame record.

    Captured base poses do not depend on the task frame, so a points file stays usable for
    defining a new frame even after the station's task frame changed.
    """
    if not isinstance(document, dict):
        raise ValueError('workspace points file must be a mapping')
    frame = document.get('frame')
    recorded = None if frame == 'base' else {'name': frame, 'base_T_frame': document.get('base_T_frame')}
    point_set(document, recorded)
    return {name: np.asarray(measurement['base_pose'][:3], dtype=float)
            for name, measurement in document['points'].items()}


MIN_FRAME_AXIS_LENGTH_M = 1e-3
COLLINEAR_TOLERANCE_M = 1e-6


def _frame_points(*values):
    points = [np.asarray(value, dtype=float) for value in values]
    if any(point.shape != (3,) or not np.isfinite(point).all() for point in points):
        raise ValueError('frame points must be finite XYZ positions')
    return points


def frame_from_points(origin, x_point, y_point) -> np.ndarray:
    """base_T_frame with its origin at ``origin``, x towards ``x_point`` and y towards ``y_point``.

    x = normalize(x_point - origin), z = normalize(x cross (y_point - origin)), y = z cross x,
    so y is the component of ``y_point - origin`` orthogonal to x.
    """
    origin, x_point, y_point = _frame_points(origin, x_point, y_point)
    x_axis = x_point - origin
    length = np.linalg.norm(x_axis)
    if length < MIN_FRAME_AXIS_LENGTH_M:
        raise ValueError(f'x-axis point is {length * 1000:.3f} mm from the origin; '
                         f'it must be at least {MIN_FRAME_AXIS_LENGTH_M * 1000:g} mm away')
    x_axis = x_axis / length
    z_axis = np.cross(x_axis, y_point - origin)
    offset = np.linalg.norm(z_axis)
    if offset < COLLINEAR_TOLERANCE_M:
        raise ValueError(f'origin, x-axis and y-axis points are collinear (y-axis point is {offset:.2e} m '
                         'from the x axis); measure a y-axis point off the x axis')
    z_axis = z_axis / offset
    transform = np.eye(4)
    transform[:3, 0], transform[:3, 1], transform[:3, 2] = x_axis, np.cross(z_axis, x_axis), z_axis
    transform[:3, 3] = origin
    return transform


def frame_point_quality(origin, x_point, y_point) -> dict:
    """Measured axis lengths and the angle between the measured y direction and the final y axis."""
    transform = frame_from_points(origin, x_point, y_point)
    origin, x_point, y_point = _frame_points(origin, x_point, y_point)
    measured_y = y_point - origin
    cosine = float(np.dot(measured_y, transform[:3, 1]) / np.linalg.norm(measured_y))
    return {'axis_lengths_m': {'x': float(np.linalg.norm(x_point - origin)),
                               'y': float(np.linalg.norm(measured_y))},
            'y_axis_angle_deg': float(np.degrees(np.arccos(np.clip(cosine, -1.0, 1.0))))}


def frame_axes(task_frame) -> dict:
    """Origin and unit axes of a task frame expressed in the base frame; None without a frame."""
    if task_frame is None:
        return {'origin_base_m': None, 'axes_base': None}
    transform = task_transform(task_frame)
    return {'origin_base_m': transform[:3, 3].tolist(),
            'axes_base': {axis: transform[:3, column].tolist() for column, axis in enumerate('xyz')}}


def candidate_bounds(document, task_frame):
    document, positions = point_set(document, task_frame)
    if len(positions) < 2:
        raise ValueError('capture at least two points before proposing bounds')
    low, high = positions.min(axis=0), positions.max(axis=0)
    if np.any(high - low <= 1e-9):
        raise ValueError('captured points must span all three axes; no automatic padding')
    return {'frame': document['frame'], 'low': low.tolist(), 'high': high.tolist()}
