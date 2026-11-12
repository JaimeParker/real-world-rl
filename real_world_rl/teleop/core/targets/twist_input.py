"""SpaceMouse direction calibration: input-frame vectors into robot base frame."""
import numpy as np


def validate_input_rotation(value):
    if value is None:
        return None
    rotation = np.asarray(value, dtype=np.float64)
    if (rotation.shape != (3, 3) or not np.isfinite(rotation).all()
            or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6, rtol=0)
            or not np.isclose(np.linalg.det(rotation), 1.0, atol=1e-6, rtol=0)):
        raise ValueError("input_rotation must be a finite proper 3x3 rotation: base_R_input")
    return rotation.copy()


def rotate_input(twist, rotation):
    if rotation is None:
        return twist
    result = np.asarray(twist, dtype=np.float64).copy().reshape(6)
    # Translation and angular velocity rotate as separate vectors. There is no
    # origin translation term: these are input directions, not a spatial wrench.
    for start in (0, 3):
        vector = rotation @ result[start:start + 3]
        # Keep direction if diagonal input would exceed the action cube.
        result[start:start + 3] = vector / max(1.0, float(np.max(np.abs(vector))))
    return result


def input_frame_contract(options):
    rotation = validate_input_rotation(options.get("input_rotation"))
    result = {} if rotation is None else {
        "input_rotation": rotation.tolist(),
        "input_rotation_semantics": "base_R_input_direction_preserving_cube_v1",
    }
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
