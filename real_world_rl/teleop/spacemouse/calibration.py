"""Device-only horizontal direction calibration; never opens a robot connection."""
from __future__ import annotations

from collections.abc import Mapping, Sequence
import math

import numpy as np

from real_world_rl.teleop.core.targets.twist_input import validate_input_rotation

_DIRECTIONS = {"+x": (1., 0.), "+y": (0., 1.), "-x": (-1., 0.), "-y": (0., -1.)}


def yaw_rotation(yaw_rad: float) -> np.ndarray:
    """Map device vectors into the operator's fixed task frame."""
    if isinstance(yaw_rad, bool) or not math.isfinite(yaw_rad):
        raise ValueError("yaw_rad must be finite")
    c, s = math.cos(yaw_rad), math.sin(yaw_rad)
    return np.array([[c, -s, 0.], [s, c, 0.], [0., 0., 1.]])


def input_rotation(base_rotation: Sequence, yaw_rad: float) -> np.ndarray:
    """Compose mounting and operator calibration exactly once."""
    base = validate_input_rotation(base_rotation)
    if base is None:
        raise ValueError("base_rotation is required")
    return base @ yaw_rotation(yaw_rad)


def sample_trials(value, label):
    """Accept legacy N x 6 rows or explicitly separated, equally weighted pushes."""
    values = value.get("trials") if isinstance(value, Mapping) else [value]
    if not isinstance(values, (list, tuple)) or not values:
        raise ValueError(f"{label}: expected one or more trials")
    trials = []
    for value in values:
        rows = np.asarray(value, dtype=float)
        if (rows.ndim != 2 or rows.shape[1] != 6 or not np.isfinite(rows).all()
                or np.any(abs(rows) > 1)):
            raise ValueError(f"{label}: expected normalized finite N x 6 samples per trial")
        trials.append(rows)
    return trials


def fit_yaw(
    samples: Mapping[str, Sequence], *, min_signal: float = 0.05,
    max_error_deg: float = 25.0,
) -> dict:
    """Fit +X/+Y, independently validate -X/-Y, and retain per-label residuals.

    Each label contains N decoded, fixed-axis-mapped six-axis samples. Trials
    have equal weight regardless of recording length or push strength.
    """
    if not math.isfinite(min_signal) or not 0 < min_signal < 1:
        raise ValueError("min_signal must be in (0, 1)")
    if not math.isfinite(max_error_deg) or not 0 < max_error_deg < 45:
        raise ValueError("max_error_deg must be in (0, 45)")
    directions = {}
    diagnostics = {}
    for label in _DIRECTIONS:
        directions[label] = []
        trials = []
        for rows in sample_trials(samples.get(label, []), label):
            norms = np.linalg.norm(rows[:, :2], axis=1)
            active = norms >= min_signal
            if not active.any():
                raise ValueError(f"{label}: insufficient horizontal signal; resample")
            mean = (rows[active, :2] / norms[active, None]).mean(axis=0)
            coherence = float(np.linalg.norm(mean))
            if coherence < 0.8:
                raise ValueError(f"{label}: inconsistent push direction; resample")
            directions[label].append(mean / coherence)
            trials.append({
                "active_samples": int(active.sum()), "coherence": coherence,
                "vertical_rms": float(np.sqrt(np.mean(rows[active, 2] ** 2))),
                "angular_rms": float(np.sqrt(np.mean(rows[active, 3:] ** 2))),
            })
        diagnostics[label] = {"trials": trials, "trial_count": len(trials),
                              "active_samples": sum(t['active_samples'] for t in trials)}
    # SO(2) fit using only positive labels; negative labels remain held out.
    cosine = sine = 0.
    for label in ("+x", "+y"):
        v = np.asarray(_DIRECTIONS[label])
        for u in directions[label]:
            cosine += float(u @ v)
            sine += float(u[0] * v[1] - u[1] * v[0])
    if math.hypot(cosine, sine) < 1e-8:
        raise ValueError("directions cannot be explained by one yaw")
    yaw = math.atan2(sine, cosine)
    rotation = yaw_rotation(yaw)
    for label, trials in directions.items():
        errors = []
        for direction, diagnostic in zip(trials, diagnostics[label]["trials"]):
            mapped = rotation[:2, :2] @ direction
            error = math.degrees(math.acos(float(np.clip(mapped @ _DIRECTIONS[label], -1, 1))))
            diagnostic["error_deg"] = error
            errors.append(error)
            if error > max_error_deg:
                raise ValueError(f"{label}: direction error {error:.1f} degrees; resample or inspect axis mapping")
        diagnostics[label]["error_deg"] = max(errors)
    return {"schema_version": 1, "yaw_rad": yaw, "directions": diagnostics,
            "max_error_deg": max_error_deg, "min_signal": min_signal,
            "horizontal_validated": True, "rotation_validated": False}


def validate_axes(samples: Mapping[str, Sequence], yaw_rad: float, *,
                  min_signal: float = .05, max_error_deg: float = 25.) -> dict:
    """Validate vertical translation and signed rotations independently of yaw fitting."""
    if not math.isfinite(min_signal) or not 0 < min_signal < 1:
        raise ValueError("min_signal must be in (0, 1)")
    if not math.isfinite(max_error_deg) or not 0 < max_error_deg < 45:
        raise ValueError("max_error_deg must be in (0, 45)")
    rotation = yaw_rotation(yaw_rad)
    residuals = {}
    for label in ("+z", "-z", "+rx", "-rx", "+ry", "-ry", "+rz", "-rz"):
        if label not in samples:
            continue
        if isinstance(samples[label], Mapping):
            errors = [validate_axes({label: rows}, yaw_rad, min_signal=min_signal,
                                   max_error_deg=max_error_deg)["axis_errors_deg"][label]
                      for rows in sample_trials(samples[label], label)]
            residuals[label] = max(errors)
            continue
        rows = np.asarray(samples[label], dtype=float)
        if rows.ndim != 2 or rows.shape[1] != 6 or not np.isfinite(rows).all() or np.any(abs(rows) > 1):
            raise ValueError(f"{label}: expected normalized finite N x 6 samples")
        start = 3 if 'r' in label else 0
        vectors = rows[:, start:start+3]
        norms = np.linalg.norm(vectors, axis=1)
        active = norms >= min_signal
        if not active.any():
            raise ValueError(f"{label}: insufficient signal")
        mean = (vectors[active] / norms[active, None]).mean(axis=0)
        if np.linalg.norm(mean) < .8:
            raise ValueError(f"{label}: inconsistent direction")
        actual = rotation @ (mean / np.linalg.norm(mean))
        expected = np.zeros(3)
        expected['xyz'.index(label[-1])] = 1 if label[0] == '+' else -1
        error = math.degrees(math.acos(float(np.clip(actual @ expected, -1, 1))))
        if error > max_error_deg:
            raise ValueError(f"{label}: direction error {error:.1f}; inspect rotation mapping")
        residuals[label] = error
    return {"axis_errors_deg": residuals,
            "vertical_validated": all(label in residuals for label in ('+z', '-z')),
            "rotation_validated": all(label in residuals for label in
                                      ('+rx', '-rx', '+ry', '-ry', '+rz', '-rz'))}
