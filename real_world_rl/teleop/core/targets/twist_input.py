"""SpaceMouse direction calibration: input-frame vectors into robot base frame."""
import numpy as np


from real_world_rl.teleop.core.input_frame import (
    rotate_twist as rotate_input,
    validate_input_rotation,
)


AXIS_LOCKS = ("dominant", "off")


def lock_dominant_group(twist, axis_lock="dominant"):
    """``dominant``: keep only the larger of the translation and rotation groups of a device twist.

    Pushing a SpaceMouse cap also tilts it (and twisting it shifts it), so a pure push reads as
    translation plus a large rotation; the group with the larger peak axis is the one the operator
    meant and the other is zeroed. ``off`` returns the twist unchanged."""
    if axis_lock not in AXIS_LOCKS:
        raise ValueError(f"axis_lock must be one of {AXIS_LOCKS}, got {axis_lock!r}")
    result = np.asarray(twist, dtype=np.float64).copy().reshape(6)
    if axis_lock == "dominant":
        if np.max(np.abs(result[3:])) > np.max(np.abs(result[:3])):
            result[:3] = 0.0
        else:
            result[3:] = 0.0
    return result


def input_frame_contract(options):
    rotation = validate_input_rotation(options.get("input_rotation"))
    result = {} if rotation is None else {
        "input_rotation": rotation.tolist(),
        "input_rotation_semantics": "base_R_input_direction_preserving_cube_v1",
    }
    task_rotation = validate_input_rotation(options.get("task_frame_rotation"))
    if task_rotation is not None:
        result["task_frame_rotation"] = task_rotation.tolist()
    axes, gain = options.get("diagnostic_axes", "all"), options.get("diagnostic_gain", 1.)
    diagnostic_input(np.zeros(6), axes, gain)
    if axes != "all" or gain != 1.:
        result["input_diagnostic"] = {"axes": axes, "gain": gain}
    return result


def diagnostic_input(twist, axes="all", gain=1.):
    """Explicit commissioning filter; never enabled implicitly for normal control."""
    if axes not in ("all", "translation", "rotation"):
        raise ValueError("diagnostic_axes must be all, translation or rotation")
    if isinstance(gain, bool) or not np.isfinite(gain) or not 0 < gain <= 1:
        raise ValueError("diagnostic_gain must be in (0, 1]")
    result = np.asarray(twist, dtype=float).copy()
    if axes == "translation":
        result[3:6] = 0
    elif axes == "rotation":
        result[:3] = 0
    return result * gain


def input_sample_diagnostics(sample, base_input):
    """Keep the evidence attached to the exact consumed sample, never a later read."""
    import copy
    return {**copy.deepcopy(getattr(sample, "diagnostics", {})),
            "sample_seq": getattr(sample, "seq", None),
            "sample_monotonic_s": getattr(sample, "timestamp", None),
            "mapped_device_input": np.asarray(sample.action).tolist(),
            "base_input_after_filter": np.asarray(base_input).tolist()}
