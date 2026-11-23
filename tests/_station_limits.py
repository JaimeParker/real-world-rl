"""Explicit station and override builders for hardware-free env fixtures."""
import numpy as np

from real_world_rl.devices.limits import TcpBox


def tcp_box(bounds):
    low, high = np.asarray(bounds, dtype=float).reshape(2, 3)
    center = (low + high) / 2
    return TcpBox(center, np.eye(3), low-center, high-center)


def station_limits(*, joints=None, bounds=None, steps=None):
    result = {"joints": [[-10., 10.]] * 7 if joints is None else joints}
    if bounds is not None:
        box = tcp_box(bounds)
        result["tcp"] = {"origin": box.origin_xyz.tolist(), **box.to_dict()}
    if steps is not None:
        result["max_delta_per_step"] = steps
    return result


def env_limits(*, joints=None, bounds=None):
    result = {}
    if joints is not None:
        result["joints"] = joints
    if bounds is not None:
        result["tcp"] = {"low": list(bounds[:3]), "high": list(bounds[3:])}
    return result
