"""Task-frame TCP constraints. Pure numpy; all public poses use xyzw quaternions."""
from __future__ import annotations

import math
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


class Workspace:
    """An oriented position box and optional SO(3) angular radius in its frame.

    Orientation uses a reference quaternion and geodesic angular radius, avoiding
    Euler branch cuts. Devices reject violating targets; the environment may use
    clip_pose to project policy targets. Neither method commands a recovery move.
    """

    def __init__(self, specification: Mapping, task_frame: Mapping | None = None):
        if not isinstance(specification, Mapping) or set(specification) - {
            "frame", "low", "high", "orientation_xyzw", "max_angle_rad",
            "measured_margin_m", "measured_margin_rad"
        }:
            raise ValueError("unknown workspace fields")
        self.frame = specification.get("frame", "base")
        transform = task_transform(task_frame)
        if self.frame == "base":
            transform = np.eye(4)
        elif task_frame is None or self.frame != task_frame["name"]:
            raise ValueError("workspace.frame must be base or the configured task frame")
        self.rotation = transform[:3, :3]
        self.origin = transform[:3, 3]
        self.low = np.asarray(specification.get("low"), dtype=float)
        self.high = np.asarray(specification.get("high"), dtype=float)
        if (self.low.shape != (3,) or self.high.shape != (3,)
                or not np.isfinite(self.low).all() or not np.isfinite(self.high).all()
                or np.any(self.low >= self.high)):
            raise ValueError("workspace low/high must be finite ordered XYZ bounds")
        self.measured_margin_m = specification.get("measured_margin_m", .003)
        self.measured_margin_rad = specification.get("measured_margin_rad", .03)
        for name in ("measured_margin_m", "measured_margin_rad"):
            value = getattr(self, name)
            if isinstance(value, bool) or not math.isfinite(value) or value < 0:
                raise ValueError(f"{name} must be finite and nonnegative")
        self.reference = None
        self.angle = specification.get("max_angle_rad")
        if ("orientation_xyzw" in specification) != (self.angle is not None):
            raise ValueError("workspace orientation_xyzw and max_angle_rad must be supplied together")
        if self.angle is not None:
            if isinstance(self.angle, bool) or not math.isfinite(self.angle) or not 0 <= self.angle <= math.pi:
                raise ValueError("max_angle_rad must be in [0, pi]")
            self.reference = quaternion_matrix(specification["orientation_xyzw"])

    def position(self, base_xyz) -> np.ndarray:
        return self.rotation.T @ (np.asarray(base_xyz, dtype=float) - self.origin)

    def violations(self, pose, *, measured=False) -> list[str]:
        pose = np.asarray(pose, dtype=float)
        if pose.shape != (7,) or not np.isfinite(pose).all():
            raise ValueError("workspace pose must be finite XYZ + xyzw")
        margin_m = self.measured_margin_m if measured else 1e-9
        margin_rad = self.measured_margin_rad if measured else 1e-9
        position = self.position(pose[:3])
        errors = [f"{self.frame}.{axis}={value:.6f} outside [{lo}, {hi}]"
                  for axis, value, lo, hi in zip("xyz", position, self.low, self.high)
                  if value < lo - margin_m or value > hi + margin_m]
        if self.reference is not None:
            rotation = self.rotation.T @ quaternion_matrix(pose[3:])
            angle = math.acos(float(np.clip((np.trace(self.reference.T @ rotation)-1)/2, -1, 1)))
            if angle > self.angle + margin_rad:
                errors.append(f"{self.frame}.orientation angle {angle:.6f} exceeds {self.angle}")
        return errors


    def clip_pose(self, pose):
        """Project position and orientation into this workspace in its own frame."""
        # Quaternion projection uses numpy only, keeping the device layer scipy-free.
        result = np.asarray(pose, dtype=float).copy()
        self.violations(result)  # validate shape, finiteness and orientation
        result[:3] = self.rotation @ np.clip(self.position(result[:3]), self.low, self.high) + self.origin
        if self.reference is not None:
            base_reference = self.rotation @ self.reference
            # Form q_ref with a stable matrix-to-quaternion conversion, including pi.
            def quaternion(matrix):
                values, vectors = np.linalg.eigh(np.array([
                    [matrix[0,0]-matrix[1,1]-matrix[2,2], matrix[0,1]+matrix[1,0], matrix[0,2]+matrix[2,0], matrix[2,1]-matrix[1,2]],
                    [matrix[0,1]+matrix[1,0], matrix[1,1]-matrix[0,0]-matrix[2,2], matrix[1,2]+matrix[2,1], matrix[0,2]-matrix[2,0]],
                    [matrix[0,2]+matrix[2,0], matrix[1,2]+matrix[2,1], matrix[2,2]-matrix[0,0]-matrix[1,1], matrix[1,0]-matrix[0,1]],
                    [matrix[2,1]-matrix[1,2], matrix[0,2]-matrix[2,0], matrix[1,0]-matrix[0,1], np.trace(matrix)],
                ]) / 3.)
                return vectors[:, np.argmax(values)]
            relative = quaternion(base_reference.T @ quaternion_matrix(result[3:]))
            if relative[3] < 0:
                relative = -relative
            sine = np.linalg.norm(relative[:3])
            angle = 2 * math.atan2(sine, relative[3])
            if angle > self.angle + 1e-9:
                axis = relative[:3] / sine
                x, y, z = axis
                skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
                limited = np.eye(3) + math.sin(self.angle) * skew + (1-math.cos(self.angle)) * (skew @ skew)
                result[3:] = quaternion(base_reference @ limited)
        return result


def spatial_contract(workspace=None, task_frame=None):
    """Numeric geometry only: file paths, frame labels and provenance are not semantics."""
    result = {}
    if task_frame is not None:
        transform = task_transform(task_frame)
        if workspace is not None:
            result['base_T_task'] = transform.tolist()
    if workspace is not None:
        geometry = Workspace(workspace, task_frame)
        result['workspace'] = {
            'version': 'task_workspace_v1',
            'base_R_frame': geometry.rotation.tolist(),
            'base_origin': geometry.origin.tolist(),
            'low': geometry.low.tolist(), 'high': geometry.high.tolist(),
            'orientation_matrix': None if geometry.reference is None else geometry.reference.tolist(),
            'max_angle_rad': geometry.angle,
            'measured_margin_m': geometry.measured_margin_m,
            'measured_margin_rad': geometry.measured_margin_rad,
        }
    return result
