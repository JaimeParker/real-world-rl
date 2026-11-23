"""Franka Panda/FR3 forward kinematics using Craig modified DH (numpy only)."""
from __future__ import annotations

import numpy as np

_c = np.sqrt(0.5)
FLANGE_T_HAND = np.array([[_c, _c, 0, 0], [-_c, _c, 0, 0],
                          [0, 0, 1, .1034], [0, 0, 0, 1]])
FLANGE_T_HAND.setflags(write=False)


def _tool_transform(value):
    raw = np.asarray(FLANGE_T_HAND if value is None else value, dtype=object)
    if any(isinstance(x, (bool, str, complex)) or not isinstance(x, (int, float, np.number))
           for x in raw.flat):
        raise ValueError("flange_T_ee must contain finite real numbers")
    result = np.array(raw, dtype=float, copy=True)
    if (result.shape != (4, 4) or not np.isfinite(result).all()
            or not np.allclose(result[3], [0, 0, 0, 1], atol=1e-8, rtol=0)
            or not np.allclose(result[:3, :3].T @ result[:3, :3], np.eye(3), atol=1e-6, rtol=0)
            or not np.isclose(np.linalg.det(result[:3, :3]), 1, atol=1e-6, rtol=0)):
        raise ValueError("flange_T_ee must be a finite proper rigid 4x4 transform")
    return result


def flange_pose(q, model="panda") -> np.ndarray:
    if model not in ("panda", "fr3"):
        raise ValueError("model must be panda or fr3")
    q = np.asarray(q, dtype=float)
    if q.shape != (7,) or not np.isfinite(q).all():
        raise ValueError("q must contain seven finite joint angles")
    result = np.eye(4)
    table = ((0, .333, 0), (0, 0, -np.pi/2), (0, .316, np.pi/2),
             (.0825, 0, np.pi/2), (-.0825, .384, -np.pi/2),
             (0, 0, np.pi/2), (.088, 0, np.pi/2), (0, .107, 0))
    for angle, (a, d, alpha) in zip([*q, 0.], table):
        c, s = np.cos(angle), np.sin(angle)
        ca, sa = np.cos(alpha), np.sin(alpha)
        result = result @ np.array([[c, -s, 0, a], [s*ca, c*ca, -sa, -d*sa],
                                    [s*sa, c*sa, ca, d*ca], [0, 0, 0, 1]])
    return result


def tcp_pose(q, flange_T_ee=FLANGE_T_HAND, model="panda") -> np.ndarray:
    return flange_pose(q, model) @ _tool_transform(flange_T_ee)


def tcp_position(q, flange_T_ee=FLANGE_T_HAND, model="panda") -> np.ndarray:
    return tcp_pose(q, flange_T_ee, model)[:3, 3]
