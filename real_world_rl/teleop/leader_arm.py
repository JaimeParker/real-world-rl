"""Joint-space leader arms (GELLO and alike) as a teleop source (registered as
``device="leader_arm"``, over the teleop part of the ``real_robot`` env's robot).

A leader arm's part reports absolute leader joint positions (``joint_abs``). The source emits
the env's joint action mode, mapped as ``TeleopPartSource`` does: ``joint_abs`` normalises the
leader joints onto ``joint_limits``, ``joint_delta`` is ``(q_leader - q_follower) /
joint_scale_rad`` (saturating at ``joint_scale_rad`` per step). ``q_follower`` is the
``joint_pos`` of the env's latest observation.

When the operator takes over, leader and follower are usually apart. The ``onset`` policy
decides what happens then:

- ``ramp`` (default): in ``joint_abs`` the emitted target is
  ``q_follower + clip(q_leader - q_follower, +-ramp_step_rad)``, normalised onto the joint
  limits, so the recorded action is the executed command; readings that were rate-limited carry
  ``events["teleop_syncing"] = True``. ``joint_delta`` already saturates at
  ``joint_scale_rad``, so the command is unchanged; only the catch-up readings carry
  ``teleop_syncing``: from the takeover until the first reading with
  ``|q_leader - q_follower|_inf <= joint_scale_rad``. Later readings of the same takeover are
  not syncing even when the operator outpaces the follower.
- ``arm``: the toggle only counts once ``|q_leader - q_follower|_inf <= arm_tolerance_rad``;
  until then ``intervened`` is false and the per-joint error is logged (at most once per second).
  Once armed, the source stays armed until the toggle releases.
"""
from __future__ import annotations

import logging
import time
from typing import Any, Mapping, Optional, Sequence, Tuple, Union

import numpy as np

from real_world_rl.teleop.base import TeleopReading
from real_world_rl.teleop.composite import ToggleableTeleopSource
from real_world_rl.teleop.part import (
    _FORWARDED_EVENTS,
    TeleopContract,
    TeleopPartSource,
    TeleopResolver,
    teleop_contract_of,
)
from real_world_rl.teleop.registry import register_teleop_source

__all__ = [
    "DEFAULT_ARM_TOLERANCE_RAD",
    "DEFAULT_RAMP_STEP_RAD",
    "LeaderArmPartSource",
    "ONSET_POLICIES",
    "TELEOP_SYNCING_EVENT",
]

ONSET_POLICIES = ("ramp", "arm")
DEFAULT_RAMP_STEP_RAD = 0.05
DEFAULT_ARM_TOLERANCE_RAD = 0.1
TELEOP_SYNCING_EVENT = "teleop_syncing"
_JOINT_MODES = ("joint_abs", "joint_delta")
_UNARMED_LOG_PERIOD_S = 1.0

logger = logging.getLogger(__name__)

# Module-level clock so tests can drive the log rate limit without sleeping.
_now = time.monotonic


def _same_leader(declared: TeleopContract, actual: TeleopContract) -> bool:
    """Both report leader joint positions (``joint_abs`` and ``joint_delta`` parts both do) for
    the same joints and gripper channel."""
    return (
        declared.action_mode in _JOINT_MODES
        and actual.action_mode in _JOINT_MODES
        and declared.arm_action_dim == actual.arm_action_dim
        and declared.has_gripper == actual.has_gripper
    )


@register_teleop_source("leader_arm")
class LeaderArmPartSource(TeleopPartSource, ToggleableTeleopSource):
    """``TeleopPartSource`` for a joint-space leader part (a local ``Teleop``, a ``RemoteTeleop``
    or a resolver yielding one) with the ``onset`` policy of the module docstring.

    ``action_mode`` is the env's (``joint_abs`` or ``joint_delta``), whatever joint space the part
    declares. The declaration check is loose: ``teleop_contract`` (declaring the part before a
    remote robot connects) and the part itself only have to agree on the joint count and the
    gripper channel and both name a joint space; the declared joint space may differ from the
    env's joint mode, and the env's mode wins. So ``--robot-teleop-action-mode joint_delta``
    declares a ``joint_abs`` GELLO for a ``joint_delta`` env. ``joint_abs`` needs
    ``last_observation_getter`` (both onset policies compare against the follower).
    """

    def __init__(
        self,
        teleop: Union[Any, TeleopResolver],
        *,
        env_contract: Mapping[str, Any],
        teleop_contract: Optional[Sequence] = None,
        onset: str = "ramp",
        arm_tolerance_rad: float = DEFAULT_ARM_TOLERANCE_RAD,
        ramp_step_rad: float = DEFAULT_RAMP_STEP_RAD,
        **part_kwargs: Any,
    ) -> None:
        if onset not in ONSET_POLICIES:
            raise ValueError(f"onset must be one of {list(ONSET_POLICIES)}, got {onset!r}")
        if arm_tolerance_rad <= 0 or ramp_step_rad <= 0:
            raise ValueError(
                f"arm_tolerance_rad and ramp_step_rad must be > 0, got {arm_tolerance_rad} and "
                f"{ramp_step_rad}"
            )
        env_mode = env_contract.get("action_mode")
        # None on a non-joint env: action_mode then reports the part's mode and the wrapper
        # rejects the mismatch, naming both sides.
        self._emitted_mode: Optional[str] = env_mode if env_mode in _JOINT_MODES else None
        self._onset = onset
        self._arm_tolerance_rad = float(arm_tolerance_rad)
        self._ramp_step_rad = float(ramp_step_rad)
        self._armed = False
        # joint_delta ramp: whether this takeover has reached the follower yet.
        self._caught_up = False
        self._last_unarmed_log_time: Optional[float] = None

        resolved = teleop() if callable(teleop) else teleop
        declared = None if teleop_contract is None else TeleopContract(*teleop_contract)
        part_contract = declared if resolved is None else teleop_contract_of(resolved)
        if part_contract is not None and part_contract.action_mode not in _JOINT_MODES:
            raise ValueError(
                f"a leader arm reports joint positions, but the teleop part declares {part_contract}"
            )
        if resolved is not None and declared is not None:
            if not _same_leader(declared, part_contract):
                raise ValueError(f"declared teleop {declared} does not match the leader part {part_contract}")
            teleop_contract = None  # the parent reads the declaration from the part itself
        super().__init__(teleop, env_contract=env_contract, teleop_contract=teleop_contract, **part_kwargs)

        if self._emitted_mode == "joint_abs":
            if self._last_observation is None:
                raise ValueError(
                    "a joint_abs leader arm needs last_observation_getter (the follower's joint_pos) "
                    f"for its {onset!r} onset"
                )
            self._require_state_field(
                tuple(env_contract.get("state_fields") or ()), "joint_pos", f"the leader-arm {onset} onset"
            )

    @property
    def action_mode(self) -> str:
        return self._emitted_mode or self._contract.action_mode

    # -- source protocol -----------------------------------------------------------

    def poll(self) -> TeleopReading:
        """One leader read; the part's own ``intervening`` is the operator toggle."""
        return self._poll(toggle=None)

    def poll_with_toggle(self, toggle: bool) -> TeleopReading:
        return self._poll(toggle=bool(toggle))

    def reset(self) -> None:
        """Also disarms: every episode starts by syncing the leader again."""
        super().reset()
        self._armed = False
        self._caught_up = False

    # -- onset ---------------------------------------------------------------------

    def _poll(self, toggle: Optional[bool]) -> TeleopReading:
        teleop, sample = self._read_leader()
        leader = np.asarray(sample.action, dtype=np.float64)
        requested = bool(sample.intervening) if toggle is None else toggle
        intervened, target, syncing = self._apply_onset(leader, requested)
        parts = [np.clip(self._map_arm(target), -1.0, 1.0)]
        if self._env_gripper:
            parts.append(np.array([self._gripper_action(sample, teleop)]))
        events = {key: True for key in _FORWARDED_EVENTS if sample.events.get(key, False)}
        if syncing:
            events[TELEOP_SYNCING_EVENT] = True
        return TeleopReading(
            action=np.concatenate(parts).astype(np.float32),
            intervened=intervened,
            episode_end=bool(sample.events.get("episode_end", False)),
            events=events,
            timestamp=float(getattr(sample, "timestamp", 0.0)),
            human_action_mask=np.asarray([True] * self.arm_action_dim + ([bool(self._contract.has_gripper)] if self._env_gripper else []), dtype=bool),
        )

    def _read_leader(self) -> Tuple[Any, Any]:
        from real_world_rl.devices.errors import DeviceFault

        teleop = self._resolve()
        if teleop is None:
            raise DeviceFault(
                "the robot has no teleop part (not connected, or no teleop: section in its yaml)",
                hint="add a teleop: section to the robot yaml and reset the env before stepping",
                device="teleop",
            )
        self._check_declaration(teleop)
        sample = teleop.read(self._read_timeout_s)
        if sample.action_space not in _JOINT_MODES:
            raise DeviceFault(
                f"leader sample has action_space {sample.action_space!r}, expected a joint space",
                device=getattr(teleop, "name", "teleop"),
            )
        return teleop, sample

    def _apply_onset(self, leader: np.ndarray, requested: bool) -> Tuple[bool, np.ndarray, bool]:
        """``(intervened, leader joint target to map, syncing)`` for one read."""
        if not requested:
            self._armed = False
            self._caught_up = False
            return False, leader, False
        if self._onset == "arm":
            if not self._armed:
                error = np.abs(leader - self._follower_joints())
                if float(error.max()) > self._arm_tolerance_rad:
                    self._log_unarmed(error)
                    return False, leader, False
                self._armed = True
            return True, leader, False
        if self.action_mode != "joint_abs":
            # joint_delta saturates at joint_scale_rad by itself. Only the catch-up after the
            # takeover is syncing; once aligned, a gap from the operator outpacing the follower
            # is ordinary teaching.
            if not self._caught_up:
                gap = float(np.abs(leader - self._follower_joints()).max())
                self._caught_up = gap <= float(self._joint_scale_rad) + 1e-6
            return True, leader, not self._caught_up
        follower = self._follower_joints()
        gap = leader - follower
        syncing = float(np.abs(gap).max()) > self._ramp_step_rad
        target = follower + np.clip(gap, -self._ramp_step_rad, self._ramp_step_rad)
        return True, target, syncing

    def _follower_joints(self) -> np.ndarray:
        return np.asarray(self._latest_state()["joint_pos"], dtype=np.float64)

    def _log_unarmed(self, error: np.ndarray) -> None:
        now = _now()
        last = self._last_unarmed_log_time
        if last is not None and now - last < _UNARMED_LOG_PERIOD_S:
            return
        self._last_unarmed_log_time = now
        logger.warning(
            "leader arm not armed: |q_leader - q_follower| = %s rad, arm_tolerance_rad = %.3f; "
            "move the leader onto the follower",
            np.round(error, 3).tolist(),
            self._arm_tolerance_rad,
        )

    def _check_declaration(self, teleop: Any) -> None:
        if teleop is self._verified:
            return
        from real_world_rl.devices.errors import DeviceFault

        actual = teleop_contract_of(teleop)
        if not _same_leader(self._contract, actual):
            raise DeviceFault(
                f"the robot's teleop part declares {actual}, but this run was built for a leader "
                f"arm {self._contract}",
                hint="fix the robot yaml's teleop: section or --robot-teleop-action-mode/"
                "--robot-teleop-has-gripper",
                device=getattr(teleop, "name", "teleop"),
            )
        self._verified = teleop
