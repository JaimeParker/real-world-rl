"""Task-frame transforms. Pure numpy; all public poses use xyzw quaternions."""
from __future__ import annotations

from collections.abc import Mapping

import numpy as np


def task_transform(frame: Mapping | None) -> np.ndarray:
    if frame is None:
        return np.eye(4)
    if not isinstance(frame, Mapping) or set(frame) - {"name", "base_T_frame", "source"}:
        raise ValueError("task_frame requires name and base_T_frame; optional source")
    if not isinstance(frame.get("name"), str) or not frame["name"] or frame["name"] == "base":
        raise ValueError("task_frame.name must name a non-base frame")
    matrix = np.asarray(frame.get("base_T_frame"), dtype=float)
    if matrix.shape != (4, 4) or not np.isfinite(matrix).all():
        raise ValueError("base_T_frame must be a finite 4x4 transform")
    r = matrix[:3, :3]
    if (not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-8, rtol=0)
            or not np.allclose(r.T @ r, np.eye(3), atol=1e-6, rtol=0)
            or not np.isclose(np.linalg.det(r), 1, atol=1e-6, rtol=0)):
        raise ValueError("base_T_frame must be a proper rigid transform")
    return matrix.copy()


def quaternion_matrix(value) -> np.ndarray:
    q = np.asarray(value, dtype=float)
    if q.shape != (4,) or not np.isfinite(q).all() or np.linalg.norm(q) < 1e-12:
        raise ValueError("orientation must be a nonzero finite xyzw quaternion")
    x, y, z, w = q / np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
