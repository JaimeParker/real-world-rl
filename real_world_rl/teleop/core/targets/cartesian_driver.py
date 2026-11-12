"""What a ``TRACKS_CARTESIAN_TARGET`` teleop source needs around its ``CartesianTargetTracker``.

The tracker owns the persistent command, the lead cap and the speed limit. Driving it from a
device also takes the follower's measured pose, the env's per-step scales and a clock for the
advances; ``CartesianTrackerDriver`` holds those for a source (by composition, it is not a base
class). How a source requests poses, and what it does after a workspace rejection or a stalled
control loop, stays in the source.

``validated_tracker_config`` checks a source's tracker kwargs against the env scales, and
``tracker_control_spec`` is the ``teleop_control`` block those kwargs record.
"""
from __future__ import annotations

from typing import Callable, Mapping, Optional

import numpy as np

from real_world_rl.teleop.core.targets.cartesian_tracker import (
    CartesianTargetTracker,
    CartesianTrackerConfig,
)
from real_world_rl.teleop.core.targets.follower import PoseGetter

__all__ = ["CartesianTrackerDriver", "tracker_control_spec", "validated_tracker_config"]


def validated_tracker_config(
    source_name: str,
    *,
    pos_scale_m: float,
    rot_scale_rad: float,
    lead_cap_m: float,
    lead_cap_rad: float,
    target_speed_m_s: float,
    target_speed_rad_s: float,
    max_dt_s: float,
) -> CartesianTrackerConfig:
    """The tracker config of source ``source_name``. The env scales must be positive and at least
    the lead caps, so one env step reaches every command the lead cap allows."""
    if not (pos_scale_m > 0 and rot_scale_rad > 0):
        raise ValueError(
            f"pos_scale_m and rot_scale_rad must be positive, got {pos_scale_m}, {rot_scale_rad}"
        )
    config = CartesianTrackerConfig(
        lead_cap_m=lead_cap_m,
        lead_cap_rad=lead_cap_rad,
        speed_m_s=target_speed_m_s,
        speed_rad_s=target_speed_rad_s,
        max_dt_s=max_dt_s,
    )
    if pos_scale_m < lead_cap_m or rot_scale_rad < lead_cap_rad:
        raise ValueError(
            f"{source_name} needs xyz_scale_m >= lead_cap_m and rot_scale_rad >= lead_cap_rad, "
            "so one env step reaches every command the lead cap allows; got "
            f"xyz_scale_m={pos_scale_m}, lead_cap_m={lead_cap_m}, rot_scale_rad={rot_scale_rad}, "
            f"lead_cap_rad={lead_cap_rad}. Raise --real-robot.xyz-scale-m/--real-robot.rot-scale-rad "
            "or lower --teleop-lead-cap-m/--teleop-lead-cap-rad"
        )
    return config


def tracker_control_spec(tracker_options: Mapping[str, float]) -> dict:
    """The ``teleop_control`` block of the tracker kwargs ``lead_cap_m``/``lead_cap_rad``/
    ``target_speed_m_s``/``target_speed_rad_s``."""
    return CartesianTrackerConfig(
        lead_cap_m=tracker_options["lead_cap_m"],
        lead_cap_rad=tracker_options["lead_cap_rad"],
        speed_m_s=tracker_options["target_speed_m_s"],
        speed_rad_s=tracker_options["target_speed_rad_s"],
    ).describe()


class CartesianTrackerDriver:
    """``tracker`` with the follower it drives: ``follower_pose_getter`` (the measured TCP pose,
    None before the env's first reset), the env's per-step ``pos_scale_m``/``rot_scale_rad`` and
    ``clock`` (monotonic seconds) timing the tracker's advances. ``source_name`` names the source
    in errors."""

    def __init__(
        self,
        tracker: CartesianTargetTracker,
        *,
        source_name: str,
        follower_pose_getter: PoseGetter,
        pos_scale_m: float,
        rot_scale_rad: float,
        clock: Callable[[], float],
    ) -> None:
        self._tracker = tracker
        self._source_name = source_name
        self._follower_pose = follower_pose_getter
        self._pos_scale_m = float(pos_scale_m)
        self._rot_scale_rad = float(rot_scale_rad)
        self._clock = clock
        self._last_advance_s: Optional[float] = None

    @property
    def tracker(self) -> CartesianTargetTracker:
        return self._tracker

    def reset(self) -> None:
        """After the env reset: the next advance starts a new interval, and the tracker holds the
        follower's pose when there is one yet."""
        self._last_advance_s = None
        pose = self._follower_pose()
        if pose is not None:
            self._tracker.reset(pose)

    def restart_clock(self) -> None:
        """The next ``elapsed_s()`` is 0 (a take-over starts a new interval)."""
        self._last_advance_s = None

    def elapsed_s(self) -> float:
        """Seconds since the previous advance; 0 for the first one after ``restart_clock``."""
        now = float(self._clock())
        elapsed = 0.0 if self._last_advance_s is None else now - self._last_advance_s
        self._last_advance_s = now
        return elapsed

    def measured_pose(self) -> np.ndarray:
        pose = self._follower_pose()
        if pose is None:
            raise RuntimeError(f"{self._source_name} polled before the env was reset (no follower pose yet)")
        return np.asarray(pose, dtype=np.float64)

    def action_for(self, measured: np.ndarray) -> np.ndarray:
        """The env action landing the follower on the tracker's command."""
        return self._tracker.action_for(measured, self._pos_scale_m, self._rot_scale_rad)

    def hold_after_failure(self) -> bool:
        """Input or tracking failure: a following tracker holds the measured pose (when there is
        one). Returns whether the tracker was following."""
        if not self._tracker.following:
            return False
        pose = self._follower_pose()
        if pose is not None:
            self._tracker.hold_at(pose)
        return True
