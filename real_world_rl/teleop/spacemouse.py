"""3Dconnexion SpaceMouse as a velocity input on a ``cartesian_delta`` env, through the same
``CartesianTargetTracker`` as ``leader_cartesian`` (registered as ``device="spacemouse"``).

The device layer's ``SpaceMouseTeleop`` (a poll thread feeding ``read()``) gives the puck as a
twist in ``[-1, 1]^6`` with axis map ``[-y, x, z, -roll, -pitch, -yaw]`` of the pyspacemouse
state (``invert_axes`` flips the sign of the listed output indices 0-5), the button levels, and
every button press latched until the next read. This source gives them their meaning:

- **Button 0** toggles the take-over: one press takes over, the next hands back. Moving the puck
  alone never intervenes (a nudge must not take over). A press counts once: the device's latched
  press event, or a rising level for a device without it. A button held through an env reset
  does not count.
- **Button 1** toggles the gripper between open (``+1``) and closed (``-1``) while the operator
  drives. At every take-over the gripper action starts as the env's last commanded gripper action
  (``last_gripper_action_getter``, which the intervention wrapper passes), kept as is until the
  first press, so taking over never opens a closed gripper; a press switches to the other state.
- **The puck is a velocity.** Each intervened poll requests the tracker's command moved by
  ``twist * target_speed * dt`` (``dt`` the time since the previous intervened poll, 0 at a
  take-over; translation ``twist[:3] * target_speed_m_s * dt`` in the base frame, rotation vector
  ``twist[3:] * target_speed_rad_s * dt`` left-multiplied, as the env's own step). The tracker
  then limits the step to the target speed, keeps the command within ``lead_cap_m``/
  ``lead_cap_rad`` of the measured pose (the excess is dropped), rejects a requested position
  outside the safety box, and the action is ``(command - measured)`` in the env's units. The
  request starts from the command, not from the previous request, so a lagging robot never lets
  it run ahead: once the puck is released (every axis within ``deadzone``) the command stays
  where it is, at most one lead cap ahead of the measured pose, and the robot stops there.
- **Hand-back** puts the tracker in hold at the measured pose (``hold_at``); the recorder's hold
  policy (``real_world_rl.teleop.hold``) shares the tracker and keeps the follower there. A
  workspace rejection (``WORKSPACE HOLD: <axes>``, logged) or an input or tracking failure hands
  back too: the next button-0 press takes over again. A stalled control loop (more than
  ``max_dt_s`` between two intervened polls) holds at the measured pose and the next poll takes
  over again from there.

Under ``CompositeTeleopSource`` (``--teleop-event-device stdin`` or ``keyboard``) the outcome
labels (``s``/``f``/``r``), episode end (Esc) and quit (``q``) come from the event device; the
SpaceMouse raises none. Button 0 stays the only take-over toggle: the event device's take-over
state (``t``/``h``) is ignored, and a change of it logs a warning saying so. (Its keys only set a
state, which can disagree with a take-over made by button 0, so they could not hand back reliably;
a released puck already stops the robot.)

Needs ``pyspacemouse>=2,<3``, imported when ``start()`` opens the device (the intervention wrapper
calls it), so building the source works without the SDK and a missing SDK raises ``SdkMissing``
there. ``close()`` disconnects the device.

``SpaceMouseTwistTeleopSource`` (``device="spacemouse_twist"``) keeps the old raw-twist semantics
for the ManiSkill sim examples only; see its docstring.
"""
from __future__ import annotations

import logging
import time
from typing import Any, Callable, Mapping, Optional, Sequence

import numpy as np

from real_world_rl.teleop.input_frame import rotate_input, diagnostic_input, input_sample_diagnostics

from robot_infra.devices.backends.teleops import (
    spacemouse as _spacemouse_backend,  # registers "spacemouse"
)
from robot_infra.devices.teleop import Teleop, TeleopConfig

from real_world_rl.action_contract import (
    LEGACY_ACTION_CONTRACT,
    get_env_action_contract,
)
from real_world_rl.teleop.base import TeleopReading, TeleopSource, held_gripper_action
from real_world_rl.teleop.cartesian_tracker import (
    CartesianTargetTracker,
    CartesianTrackerConfig,
)
from real_world_rl.teleop.composite import ToggleableTeleopSource
from real_world_rl.teleop.part import (
    DEFAULT_READ_TIMEOUT_S,
    TeleopPartSource,
    teleop_contract_of,
)
from real_world_rl.teleop.registry import register_teleop_source

__all__ = ["DEFAULT_DEADZONE", "SpaceMouseTeleopSource", "SpaceMouseTwistTeleopSource"]

_CARTESIAN_ACTION_MODE = "cartesian_delta"
_CARTESIAN_ARM_DIM = 6
TAKE_OVER_BUTTON = 0
GRIPPER_BUTTON = 1
_OPEN_GRIPPER, _CLOSED_GRIPPER = 1.0, -1.0
# Per axis, of full scale: the SpaceMouse Compact's rest noise stays below it.
DEFAULT_DEADZONE = 0.05

PoseGetter = Callable[[], Optional[np.ndarray]]

logger = logging.getLogger(__name__)


@register_teleop_source("spacemouse")
class SpaceMouseTeleopSource(ToggleableTeleopSource):
    """A SpaceMouse driving the follower's command tracker (module docstring).

    The device is the SpaceMouse on this machine (``device_index``, ``deadzone``,
    ``invert_axes``, ``poll_interval_s`` and ``device_table``, the local device table, configure
    it; ``start()`` connects it and ``close()`` disconnects it), or ``teleop``: a ``cartesian_delta`` teleop part the caller owns (its
    ``buttons``/``events`` named ``button_0``/``button_1``).

    ``follower_pose_getter`` returns the follower's measured TCP pose ``xyz + quat_xyzw`` in the
    base frame (None before the env's first reset). ``pos_scale_m``/``rot_scale_rad`` are the
    env's per-step scales and must be at least ``lead_cap_m``/``lead_cap_rad``. ``deadzone``
    zeroes each twist axis whose magnitude is at most that fraction of full scale. ``clock`` (a
    monotonic clock in seconds) times the tracker's advances. On an env with a gripper,
    ``last_gripper_action_getter`` is required. Read failures propagate; nothing ever turns into
    an idle reading.
    """

    def __init__(
        self,
        teleop: Optional[Any] = None,
        *,
        follower_pose_getter: PoseGetter,
        last_gripper_action_getter: Optional[Callable[[], Any]] = None,
        env_contract: Optional[Mapping[str, Any]] = None,
        device_index: int = 0,
        deadzone: float = DEFAULT_DEADZONE,
        invert_axes: Sequence[int] = (),
        input_rotation=None,
        expected_device_identity=None,
        diagnostic_axes="all",
        diagnostic_gain=1.,
        poll_interval_s: float = 0.005,
        device_table: Optional[str] = None,
        pos_scale_m: float = 0.05,
        rot_scale_rad: float = 0.3,
        lead_cap_m: float = 0.05,
        lead_cap_rad: float = 0.3,
        target_speed_m_s: float = 0.05,
        target_speed_rad_s: float = 0.3,
        max_dt_s: float = 0.3,
        safety_box: Optional[Sequence[float]] = None,
        workspace=None,
        task_frame=None,
        read_timeout_s: float = DEFAULT_READ_TIMEOUT_S,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        if not 0.0 <= deadzone < 1.0:
            raise ValueError(f"deadzone must be in [0, 1) of full scale, got {deadzone}")
        if not (pos_scale_m > 0 and rot_scale_rad > 0):
            raise ValueError(
                f"pos_scale_m and rot_scale_rad must be positive, got {pos_scale_m}, {rot_scale_rad}"
            )
        tracker_config = CartesianTrackerConfig(
            lead_cap_m=lead_cap_m,
            lead_cap_rad=lead_cap_rad,
            speed_m_s=target_speed_m_s,
            speed_rad_s=target_speed_rad_s,
            max_dt_s=max_dt_s,
        )
        if pos_scale_m < lead_cap_m or rot_scale_rad < lead_cap_rad:
            raise ValueError(
                "spacemouse needs xyz_scale_m >= lead_cap_m and rot_scale_rad >= lead_cap_rad, "
                "so one env step reaches every command the lead cap allows; got "
                f"xyz_scale_m={pos_scale_m}, lead_cap_m={lead_cap_m}, rot_scale_rad={rot_scale_rad}, "
                f"lead_cap_rad={lead_cap_rad}. Raise --real-robot.xyz-scale-m/--real-robot.rot-scale-rad "
                "or lower --teleop-lead-cap-m/--teleop-lead-cap-rad"
            )
        contract = LEGACY_ACTION_CONTRACT if env_contract is None else env_contract
        self._env_gripper = bool(contract["gripper_in_action"])
        if self._env_gripper and last_gripper_action_getter is None:
            raise ValueError(
                "spacemouse on an env with gripper_in_action starts every take-over from the last "
                "sent gripper action, so it needs last_gripper_action_getter (the intervention "
                "wrapper passes it)"
            )
        self._tracker = CartesianTargetTracker(tracker_config, safety_box=safety_box,
                                               workspace=workspace, task_frame=task_frame)
        self._pos_scale_m = float(pos_scale_m)
        self._rot_scale_rad = float(rot_scale_rad)
        self._deadzone = float(deadzone)
        from real_world_rl.teleop.input_frame import validate_input_rotation
        self._input_rotation = validate_input_rotation(input_rotation)
        diagnostic_input(np.zeros(6), diagnostic_axes, diagnostic_gain)
        self._diagnostic_axes, self._diagnostic_gain = diagnostic_axes, diagnostic_gain
        from real_world_rl.teleop.profiles import validate_device_identity
        self._expected_device_identity = validate_device_identity(expected_device_identity)
        self._follower_pose = follower_pose_getter
        self._last_gripper_action = last_gripper_action_getter
        self._read_timeout_s = read_timeout_s
        self._clock = clock

        self._owns_part = teleop is None
        if teleop is None:
            config = TeleopConfig(
                name="spacemouse",
                backend="spacemouse",
                options={
                    "device_index": device_index,
                    "deadzone": deadzone,
                    "invert_axes": list(invert_axes),
                    "poll_interval_s": poll_interval_s,
                    **({} if device_table is None else {"device_table": str(device_table)}),
                },
            )
            teleop = Teleop.backend("spacemouse")(config, read_timeout_s=read_timeout_s)
        else:
            declared = teleop_contract_of(teleop)
            if (declared.action_mode, declared.arm_action_dim) != (_CARTESIAN_ACTION_MODE, _CARTESIAN_ARM_DIM):
                raise ValueError(
                    f"spacemouse needs a {_CARTESIAN_ACTION_MODE} teleop part with "
                    f"{_CARTESIAN_ARM_DIM} axes, got {declared}"
                )
        self._part = teleop

        self._driving = False
        self._previously_driving = False
        self._previous_buttons: Optional[tuple[bool, ...]] = None  # None: the next read is the baseline
        self._previous_event_toggle = False
        self._last_advance_s: Optional[float] = None
        self._gripper_action = _OPEN_GRIPPER

    @property
    def action_mode(self) -> str:
        return _CARTESIAN_ACTION_MODE

    @property
    def arm_action_dim(self) -> int:
        return _CARTESIAN_ARM_DIM

    @property
    def has_gripper(self) -> bool:
        return self._env_gripper

    @property
    def cartesian_tracker(self) -> CartesianTargetTracker:
        """The follower's command tracker, shared with the hold policy."""
        return self._tracker

    @classmethod
    def from_env(cls, env: Any, **kwargs: Any) -> "TeleopSource":
        """Source for ``env``. Unless passed, the follower pose, the scales and the safety box
        come from the env as for ``leader_cartesian`` (``follower_pose_from_env``): a
        ``FrankaRealEnv`` down the wrapper chain, else a ``real_robot`` env in ``cartesian_delta``
        mode with ``tcp_pose`` in its state."""
        mode = kwargs.pop("mode", "tracker")
        if mode == "serl":
            from real_world_rl.teleop.spacemouse_serl import SpaceMouseSerlTeleopSource

            return SpaceMouseSerlTeleopSource.from_env(env, **kwargs)
        if mode != "tracker":
            raise ValueError("spacemouse mode must be 'tracker' or 'serl'")
        from real_world_rl.teleop.leader_cartesian import (
            _safety_box_from_env,
            follower_pose_from_env,
        )

        contract = get_env_action_contract(env)
        kwargs.setdefault("env_contract", LEGACY_ACTION_CONTRACT if contract is None else contract)
        if "follower_pose_getter" not in kwargs:
            getter, (pos_scale_m, rot_scale_rad) = follower_pose_from_env(env, contract)
            kwargs["follower_pose_getter"] = getter
            kwargs.setdefault("pos_scale_m", pos_scale_m)
            kwargs.setdefault("rot_scale_rad", rot_scale_rad)
        kwargs.setdefault("safety_box", _safety_box_from_env(env, contract))
        from real_world_rl.teleop.leader_cartesian import _find_in_wrapper_chain
        from real_world_rl.envs.real_robot.env import RealRobotEnv
        backend = _find_in_wrapper_chain(env, RealRobotEnv)
        if backend is not None:
            config = backend.config
            local = config.robot or {}
            kwargs.setdefault("workspace", config.workspace if config.workspace is not None else local.get("workspace"))
            kwargs.setdefault("task_frame", config.task_frame if config.task_frame is not None else local.get("task_frame"))
        return cls(**kwargs)

    # -- source protocol -----------------------------------------------------------

    def start(self) -> None:
        from real_world_rl.teleop.profiles import start_profiled_source
        start_profiled_source(self)

    def session_input(self):
        """Consume fresh device input without advancing the motion target."""
        sample = self._part.read(timeout_s=self._read_timeout_s)
        return sample.action, sample.buttons

    def reset(self) -> None:
        """After the env reset: handed back, the tracker holds the follower's pose, the button
        levels of the next read are the baseline, and the device's latched presses are dropped."""
        from robot_infra.devices.errors import DeviceNotConnected

        self._driving = False
        self._previously_driving = False
        self._previous_buttons = None
        self._previous_event_toggle = False
        self._last_advance_s = None
        pose = self._follower_pose()
        if pose is not None:
            self._tracker.reset(pose)
        try:
            self._part.reset()
        except DeviceNotConnected:
            return

    def poll(self) -> TeleopReading:
        """One device read; button 0 is the operator's take-over toggle."""
        return self._poll()

    def poll_with_toggle(self, toggle: bool) -> TeleopReading:
        """``poll()``: the event device's take-over state is not used (button 0 is the toggle);
        a change of it logs a warning."""
        toggle = bool(toggle)
        if toggle != self._previous_event_toggle:
            logger.warning(
                "the SpaceMouse ignores the event device's take-over/hand-back keys (t/h); "
                "press SpaceMouse button 0 to take over or hand back"
            )
        self._previous_event_toggle = toggle
        return self._poll()

    def close(self) -> None:
        if self._owns_part:
            self._part.disconnect()

    # -- polling -------------------------------------------------------------------

    def _poll(self) -> TeleopReading:
        try:
            sample = self._part.read(self._read_timeout_s)
            if sample.action_space != _CARTESIAN_ACTION_MODE:
                raise self._device_fault(
                    f"spacemouse sample has action_space {sample.action_space!r}, expected "
                    f"{_CARTESIAN_ACTION_MODE!r}"
                )
        except BaseException:
            self._hand_back_after_failure()
            raise
        presses = self._button_presses(sample)
        if presses[TAKE_OVER_BUTTON]:
            self._driving = not self._driving
        measured = self._measured_pose()
        if not self._tracker.initialized:
            self._tracker.reset(measured)
        takeover = self._driving and not self._previously_driving
        base_input = self._twist(sample.action)
        intervened, arm = self._arm_action(base_input, measured, takeover)
        self._previously_driving = self._driving
        parts = [arm]
        if self._env_gripper:
            parts.append(np.array([self._gripper(intervened, presses[GRIPPER_BUTTON])]))
        return TeleopReading(
            diagnostics=input_sample_diagnostics(sample, base_input),
            action=np.concatenate(parts).astype(np.float32),
            intervened=intervened,
            timestamp=float(getattr(sample, "timestamp", 0.0)),
        )

    def _button_presses(self, sample: Any) -> tuple[bool, ...]:
        """Per button, whether it was pressed since the previous read: its latched press event,
        or its level rising. The first read after a reset only sets the level baseline."""
        names = _spacemouse_backend.BUTTON_NAMES
        levels = tuple(bool(sample.buttons.get(name, False)) for name in names)
        previous = levels if self._previous_buttons is None else self._previous_buttons
        self._previous_buttons = levels
        return tuple(
            bool(sample.events.get(name, False)) or (level and not before)
            for name, level, before in zip(names, levels, previous)
        )

    def _twist(self, action: Any) -> np.ndarray:
        twist = np.clip(np.asarray(action, dtype=np.float64).reshape(_CARTESIAN_ARM_DIM), -1.0, 1.0)
        twist[np.abs(twist) <= self._deadzone] = 0.0
        return diagnostic_input(rotate_input(twist, self._input_rotation),
                                self._diagnostic_axes, self._diagnostic_gain)

    def _arm_action(self, twist: np.ndarray, measured: np.ndarray, takeover: bool) -> tuple[bool, np.ndarray]:
        """``(intervened, arm action)``; the arm action only matters while intervened."""
        idle = np.zeros(_CARTESIAN_ARM_DIM)
        tracker = self._tracker
        if not self._driving:
            if tracker.following:
                tracker.hold_at(measured)  # hand-back
            return False, idle
        if not tracker.following:  # a take-over, or resuming after a stalled loop
            self._take_over(start_gripper=takeover)
        try:
            dt = self._elapsed_s()
            update = tracker.advance(self._requested_pose(twist, dt), measured, dt)
            if update.workspace_violations:
                self._driving = False
                logger.warning(
                    "WORKSPACE HOLD: %s; the follower holds its pose and the SpaceMouse handed "
                    "back, press button 0 to take over again",
                    ", ".join(update.workspace_violations),
                )
                return False, idle
            if update.missed_deadline_s is not None:
                logger.warning(
                    "control loop stalled: %.3f s between two SpaceMouse commands (max %.3f s); "
                    "the follower holds its pose and the next poll continues from there",
                    update.missed_deadline_s,
                    tracker.config.max_dt_s,
                )
            return True, tracker.action_for(measured, self._pos_scale_m, self._rot_scale_rad)
        except BaseException:
            self._hand_back_after_failure()
            raise

    def _take_over(self, *, start_gripper: bool) -> None:
        self._tracker.take_over()
        self._last_advance_s = None
        if start_gripper and self._env_gripper:
            self._gripper_action = held_gripper_action(self._last_gripper_action)

    def _requested_pose(self, twist: np.ndarray, dt: float) -> np.ndarray:
        """The tracker's command moved by ``twist * target_speed * dt``."""
        config = self._tracker.config
        requested = self._tracker.command
        requested[:3] += twist[:3] * config.speed_m_s * dt
        if np.any(twist[3:]):
            from scipy.spatial.transform import Rotation

            rotation = Rotation.from_rotvec(twist[3:] * config.speed_rad_s * dt)
            requested[3:] = (rotation * Rotation.from_quat(requested[3:])).as_quat()
        return requested

    def _elapsed_s(self) -> float:
        """Seconds since the previous advance; 0 for the first one of a take-over."""
        now = float(self._clock())
        elapsed = 0.0 if self._last_advance_s is None else now - self._last_advance_s
        self._last_advance_s = now
        return elapsed

    def _measured_pose(self) -> np.ndarray:
        pose = self._follower_pose()
        if pose is None:
            raise RuntimeError("spacemouse polled before the env was reset (no follower pose yet)")
        return np.asarray(pose, dtype=np.float64)

    def _hand_back_after_failure(self) -> None:
        """Input or tracking failure: hand back and hold the measured pose."""
        self._driving = False
        self._previously_driving = False
        if not self._tracker.following:
            return
        pose = self._follower_pose()
        if pose is not None:
            self._tracker.hold_at(pose)

    def _gripper(self, intervened: bool, pressed: bool) -> float:
        if not intervened:
            return held_gripper_action(self._last_gripper_action)
        if pressed:
            self._gripper_action = _CLOSED_GRIPPER if self._gripper_action > 0.0 else _OPEN_GRIPPER
        return self._gripper_action

    @staticmethod
    def _device_fault(message: str) -> Exception:
        from robot_infra.devices.errors import DeviceFault

        return DeviceFault(
            message,
            hint="the spacemouse source reads a cartesian_delta teleop (the device layer's spacemouse backend)",
            device="spacemouse",
        )


# -- sim only: the old raw-twist source ------------------------------------------------------

_EPISODE_END_BUTTON = 1


class _EpisodeEndOnButtonPress:
    """The device part, with the latched press of ``_EPISODE_END_BUTTON`` read as ``episode_end``;
    every other attribute is the part's."""

    def __init__(self, part: Any) -> None:
        self._part = part

    def __getattr__(self, name: str) -> Any:
        return getattr(self._part, name)

    def read(self, timeout_s: Optional[float] = None) -> Any:
        sample = self._part.read(timeout_s)
        if sample.events.get(_spacemouse_backend.BUTTON_NAMES[_EPISODE_END_BUTTON], False):
            sample.events["episode_end"] = True
        return sample


@register_teleop_source("spacemouse_twist")
class SpaceMouseTwistTeleopSource(TeleopSource):
    """Sim only (the ManiSkill teleop examples): raw twist, motion counts as intervention, button 0
    gripper, button 1 episode end, ``hold_s``. Not for real robots: a nudge of the puck takes over
    and every reading is the raw twist, with no command tracker; use ``device="spacemouse"`` there.

    A SpaceMouse on this machine, read by the device layer's ``SpaceMouseTeleop`` and mapped into
    the env action space by ``TeleopPartSource``. The twist is the raw puck reading (axis map
    ``[-y, x, z, -roll, -pitch, -yaw]`` of the pyspacemouse state, clipped to ``[-1, 1]``;
    ``invert_axes`` flips the sign of the listed output indices 0-5). The operator counts as
    intervening while the twist norm exceeds ``deadzone`` or a button is held, and ``hold_s``
    keeps it intervening that long after the last activity. Button 0 toggles the device's sticky
    gripper (open at start and after every ``reset()``, sent as ``+1`` open / ``-1`` closed); a
    press of button 1 raises ``episode_end``.

    Needs ``pyspacemouse>=2,<3``, imported when ``start()`` opens the device. ``close()``
    disconnects the device.
    """

    def __init__(
        self,
        *,
        env_contract: Optional[Mapping[str, Any]] = None,
        device_index: int = 0,
        deadzone: float = 1e-3,
        invert_axes: Sequence[int] = (),
        poll_interval_s: float = 0.005,
        hold_s: float = 0.0,
        read_timeout_s: float = DEFAULT_READ_TIMEOUT_S,
        device_table: Optional[str] = None,
    ) -> None:
        config = TeleopConfig(
            name="spacemouse",
            backend="spacemouse",
            hold_s=hold_s,
            options={
                "device_index": device_index,
                "deadzone": deadzone,
                "invert_axes": list(invert_axes),
                "poll_interval_s": poll_interval_s,
                **({} if device_table is None else {"device_table": str(device_table)}),
            },
        )
        self._part = Teleop.backend("spacemouse")(config, read_timeout_s=read_timeout_s)
        self._mapped = TeleopPartSource(
            _EpisodeEndOnButtonPress(self._part),
            env_contract=LEGACY_ACTION_CONTRACT if env_contract is None else env_contract,
            read_timeout_s=read_timeout_s,
        )

    @classmethod
    def from_env(cls, env: Any, **kwargs: Any) -> "SpaceMouseTwistTeleopSource":
        kwargs.setdefault("env_contract", get_env_action_contract(env))
        return cls(**kwargs)

    @property
    def action_mode(self) -> str:
        return self._mapped.action_mode

    @property
    def arm_action_dim(self) -> int:
        return self._mapped.arm_action_dim

    @property
    def has_gripper(self) -> bool:
        return self._mapped.has_gripper

    def start(self) -> None:
        self._part.connect()

    def poll(self) -> TeleopReading:
        return self._mapped.poll()

    def reset(self) -> None:
        self._mapped.reset()

    def close(self) -> None:
        self._part.disconnect()
