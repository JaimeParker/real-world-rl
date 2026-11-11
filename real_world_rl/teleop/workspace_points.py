"""Read-only TCP measurements used to propose, never certify, workspace bounds."""
from __future__ import annotations

import numpy as np
from robot_infra.devices.workspace import task_transform, quaternion_matrix


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


def candidate_bounds(document, task_frame):
    document, positions = point_set(document, task_frame)
    if len(positions) < 2:
        raise ValueError('capture at least two points before proposing bounds')
    low, high = positions.min(axis=0), positions.max(axis=0)
    if np.any(high - low <= 1e-9):
        raise ValueError('captured points must span all three axes; no automatic padding')
    return {'frame': document['frame'], 'low': low.tolist(), 'high': high.tolist()}
