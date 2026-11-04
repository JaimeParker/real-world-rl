"""Persistent cartesian command target for teleop on ``cartesian_delta`` envs ("control v2").

``RealRobotEnv`` in ``cartesian_delta`` sends ``measured_tcp + action * scale``, and an impedance
backend (Franky's ``CartesianImpedanceTracker``) treats that pose as its equilibrium. A zero
action therefore re-anchors the equilibrium wherever the arm drifted, and a twist recomputed
from the measured pose every step never holds still. ``CartesianTargetTracker`` keeps the
target instead, per arm, independent of any device or env:

- a hold pose, sampled once (``reset`` at the env reset, ``hold_at`` at a hand-back, a workspace
  rejection, an input loss or an episode end) and then fixed while the operator does not drive;
- a command pose that, while the operator drives (``take_over``), moves towards the requested
  pose at most ``speed_m_s``/``speed_rad_s`` and never leads the measured pose by more than
  ``lead_cap_m``/``lead_cap_rad`` (the excess is dropped, not queued);
- in hold, ``reach_for_hold(measured)`` makes the command the hold pose pulled back to within
  the lead cap of the measured pose, so an arm pushed far off the hold returns at most one capped
  step at a time and the command converges onto the fixed hold pose;
- the env action that lands the arm on the command: ``(command - measured)`` as
  ``[xyz / xyz_scale_m, rotvec(R_cmd R_meas^-1) / rot_scale_rad]`` (``action_for``), which
  inverts the env's own update. It raises instead of clipping an action it cannot represent;
  with ``xyz_scale_m >= lead_cap_m`` and ``rot_scale_rad >= lead_cap_rad`` it never has to.

A gap of more than ``max_dt_s`` between two advances (a stalled control loop) is not an error:
the tracker goes to hold at the measured pose and ``advance`` reports the gap
(``TrackerUpdate.missed_deadline_s``); the caller takes over again to continue.

Poses are ``xyz + quat_xyzw`` in the robot base frame. Pure numpy and scipy.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Sequence

import numpy as np

__all__ = [
    "TRACKER_VERSION",
    "CartesianTargetTracker",
    "CartesianTrackerConfig",
    "CommandOutOfRange",
    "MissedControlDeadline",
    "TrackerUpdate",
    "workspace_violations",
]

TRACKER_VERSION = "cartesian_tracker_v1"
_POSE_DIM = 7
_SAFETY_BOX_LENGTH = 6
# Slack for floating-point error on an action the lead cap already bounds by 1.
_ACTION_TOLERANCE = 1e-6
# Slack on the inclusive max_dt_s bound: a difference of two clock readings carries rounding
# error (0.9 - 0.6 is 0.30000000000000004), which must not count as a stalled loop.
_DEADLINE_SLACK_S = 1e-9


class MissedControlDeadline(RuntimeError):
    """The time since the previous advance is negative or not finite (a broken clock)."""


class CommandOutOfRange(RuntimeError):
    """The command is further from the measured pose than one env step can express."""


@dataclass(frozen=True)
class CartesianTrackerConfig:
    """``lead_cap_m``/``lead_cap_rad``: how far the command may lead the measured pose.
    ``speed_m_s``/``speed_rad_s``: how fast the command advances towards the requested pose.
    ``max_dt_s``: the longest gap between two advances before the control loop counts as stalled
    (the tracker then holds at the measured pose).
    """

    lead_cap_m: float = 0.05
    lead_cap_rad: float = 0.3
    speed_m_s: float = 0.05
    speed_rad_s: float = 0.3
    max_dt_s: float = 0.3

    def __post_init__(self) -> None:
        for name in ("lead_cap_m", "lead_cap_rad", "speed_m_s", "speed_rad_s", "max_dt_s"):
            value = getattr(self, name)
            if not (isinstance(value, (int, float)) and math.isfinite(value) and value > 0):
                raise ValueError(f"CartesianTrackerConfig.{name} must be positive and finite, got {value!r}")

    def describe(self) -> dict:
        """The experiment-contract block (``teleop_control``) of a run using this tracker."""
        return {
            "version": TRACKER_VERSION,
            "lead_cap_m": float(self.lead_cap_m),
            "lead_cap_rad": float(self.lead_cap_rad),
            "speed_m_s": float(self.speed_m_s),
            "speed_rad_s": float(self.speed_rad_s),
        }


@dataclass(frozen=True)
class TrackerUpdate:
    """Result of ``advance``: the new command; the axes of the requested pose outside the
    workspace (empty unless the request was rejected); the gap since the previous advance when it
    exceeded ``max_dt_s`` (None otherwise). On a rejection or a missed deadline the tracker went
    to hold at the measured pose."""

    command: np.ndarray
    workspace_violations: tuple[str, ...] = ()
    missed_deadline_s: Optional[float] = None

    @property
    def held(self) -> bool:
        """Whether this advance put the tracker in hold instead of following."""
        return bool(self.workspace_violations) or self.missed_deadline_s is not None


def workspace_violations(pose: np.ndarray, safety_box: Optional[Sequence[float]]) -> tuple[str, ...]:
    """The position axes of ``pose`` outside ``safety_box`` (``[xmin, ymin, zmin, xmax, ymax, zmax]``)."""
    if safety_box is None:
        return ()
    box = np.asarray(safety_box, dtype=np.float64)
    return tuple(
        f"{axis}={value:.4f} outside [{low:.4f}, {high:.4f}] m"
        for axis, value, low, high in zip("xyz", pose[:3], box[:3], box[3:])
        if value < low or value > high
    )


class CartesianTargetTracker:
    """Hold pose and command pose of one arm (module docstring). ``safety_box`` bounds the
    requested position; ``None`` leaves the workspace to the env and device layer."""

    def __init__(
        self,
        config: Optional[CartesianTrackerConfig] = None,
        *,
        safety_box: Optional[Sequence[float]] = None,
    ) -> None:
        self.config = CartesianTrackerConfig() if config is None else config
        if safety_box is not None:
            box = np.asarray(safety_box, dtype=np.float64)
            if box.shape != (_SAFETY_BOX_LENGTH,) or not np.all(np.isfinite(box)) or np.any(box[:3] > box[3:]):
                raise ValueError(
                    f"safety_box must be finite [xmin, ymin, zmin, xmax, ymax, zmax] with low <= high, got {safety_box}"
                )
            safety_box = box
        self.safety_box = safety_box
        self._hold_pose: Optional[np.ndarray] = None
        self._command: Optional[np.ndarray] = None
        self._following = False

    # -- state -------------------------------------------------------------------------

    @property
    def initialized(self) -> bool:
        """Whether a hold pose was sampled (``reset`` or ``hold_at``)."""
        return self._hold_pose is not None

    @property
    def following(self) -> bool:
        """Whether the operator drives the command (``take_over`` until the next ``hold_at``)."""
        return self._following

    @property
    def hold_pose(self) -> np.ndarray:
        return self._require_initialized(self._hold_pose).copy()

    @property
    def command(self) -> np.ndarray:
        return self._require_initialized(self._command).copy()

    def describe(self) -> dict:
        return self.config.describe()

    # -- transitions -------------------------------------------------------------------

    def reset(self, measured: np.ndarray) -> None:
        """After an env reset: hold and command become ``measured``."""
        self.hold_at(measured)

    def hold_at(self, measured: np.ndarray) -> None:
        """Stops following: hold and command become ``measured`` and stay there."""
        pose = _as_pose(measured)
        self._hold_pose = pose.copy()
        self._command = pose.copy()
        self._following = False

    def take_over(self) -> None:
        """The operator starts driving from the current command (no jump)."""
        self._require_initialized(self._command)
        self._following = True

    def advance(self, requested: np.ndarray, measured: np.ndarray, dt: float) -> TrackerUpdate:
        """Moves the command towards ``requested`` by at most ``speed * dt``, then pulls it back
        to within the lead cap of ``measured``. A ``requested`` position outside the safety box,
        or ``dt`` above ``max_dt_s``, puts the tracker in hold at ``measured`` and is reported in
        the result; ``take_over`` is needed to follow again. A negative or non-finite ``dt``
        raises ``MissedControlDeadline``."""
        if not self._following:
            raise RuntimeError("CartesianTargetTracker.advance needs take_over() first")
        if not (math.isfinite(dt) and dt >= 0.0):
            raise MissedControlDeadline(f"invalid time since the previous command: {dt} s (clock error)")
        requested, measured = _as_pose(requested), _as_pose(measured)
        if dt > self.config.max_dt_s + _DEADLINE_SLACK_S:
            self.hold_at(measured)
            return TrackerUpdate(self.command, missed_deadline_s=float(dt))
        violations = workspace_violations(requested, self.safety_box)
        if violations:
            self.hold_at(measured)
            return TrackerUpdate(self.command, violations)
        config = self.config
        advanced = _limited_towards(self._command, requested, config.speed_m_s * dt, config.speed_rad_s * dt)
        # The lead cap: the advanced command, pulled back to within reach of the measured pose.
        self._command = _limited_towards(measured, advanced, config.lead_cap_m, config.lead_cap_rad)
        return TrackerUpdate(self.command)

    def reach_for_hold(self, measured: np.ndarray) -> np.ndarray:
        """In hold: the command becomes the hold pose pulled back to within the lead cap of
        ``measured`` (the hold pose itself when it is within reach). Returns the command."""
        if self._following:
            raise RuntimeError("CartesianTargetTracker.reach_for_hold is for the hold, not while following")
        hold = self._require_initialized(self._hold_pose)
        self._command = _limited_towards(_as_pose(measured), hold, self.config.lead_cap_m, self.config.lead_cap_rad)
        return self.command

    def action_for(self, measured: np.ndarray, xyz_scale_m: float, rot_scale_rad: float) -> np.ndarray:
        """The 6-D ``cartesian_delta`` arm action that takes ``measured`` onto the command.
        Raises ``CommandOutOfRange`` when a component would leave ``[-1, 1]``."""
        command = self._require_initialized(self._command)
        measured = _as_pose(measured)
        from scipy.spatial.transform import Rotation

        translation = (command[:3] - measured[:3]) / xyz_scale_m
        rotation = (Rotation.from_quat(command[3:]) * Rotation.from_quat(measured[3:]).inv()).as_rotvec()
        action = np.concatenate([translation, rotation / rot_scale_rad])
        if not np.all(np.isfinite(action)) or np.max(np.abs(action)) > 1.0 + _ACTION_TOLERANCE:
            raise CommandOutOfRange(
                f"the command is {np.linalg.norm(command[:3] - measured[:3]):.4f} m / "
                f"{np.linalg.norm(rotation):.4f} rad from the measured pose, beyond one env step "
                f"(xyz_scale_m={xyz_scale_m}, rot_scale_rad={rot_scale_rad}); refusing to clip"
            )
        return np.clip(action, -1.0, 1.0)

    @staticmethod
    def _require_initialized(pose: Optional[np.ndarray]) -> np.ndarray:
        if pose is None:
            raise RuntimeError("CartesianTargetTracker used before reset(measured)")
        return pose


def _as_pose(pose: np.ndarray) -> np.ndarray:
    pose = np.asarray(pose, dtype=np.float64).reshape(-1)
    if pose.shape != (_POSE_DIM,) or not np.all(np.isfinite(pose)):
        raise ValueError(f"a pose is a finite xyz + quat_xyzw (7,), got {pose}")
    return pose


# Distances below this count as already within a limit, so a pose that is reached is kept
# bit-for-bit instead of being recomposed (which would perturb the last bits of the quaternion).
_REACHED = 1e-12


def _limited_towards(
    start: np.ndarray, goal: np.ndarray, max_m: float, max_rad: float
) -> np.ndarray:
    """``goal``, or the pose ``max_m``/``max_rad`` from ``start`` towards it (translation along
    the offset vector, rotation along the rotation vector of ``R_goal R_start^-1``)."""
    from scipy.spatial.transform import Rotation

    result = goal.copy()
    offset = goal[:3] - start[:3]
    distance = float(np.linalg.norm(offset))
    if distance > max_m + _REACHED:
        result[:3] = start[:3] + offset * (max_m / distance)
    start_rotation = Rotation.from_quat(start[3:])
    rotation = (Rotation.from_quat(goal[3:]) * start_rotation.inv()).as_rotvec()
    angle = float(np.linalg.norm(rotation))
    if angle > max_rad + _REACHED:
        result[3:] = (Rotation.from_rotvec(rotation * (max_rad / angle)) * start_rotation).as_quat()
    return result
