"""The Pico VR controller as a pose device on a ``cartesian_delta`` env, through the same
``CartesianTargetTracker`` as ``leader_cartesian`` and ``spacemouse`` (registered as
``device="pico"``).

The device layer's ``PicoTeleop`` (``real_world_rl.teleop.pico.backend``) gives the hand pose
(``diagnostics["hand_pose"]``, Pico tracking frame), the trigger as a width target, and the bind
and episode-end presses latched until the next read. This source gives them their meaning:

- **Bind takes over and hands back.** One bind press takes over, the next hands back (the
  device's latched ``bind`` event, so a press between two control steps is not lost). Moving the
  hand alone never intervenes.
- **Anchored mapping** (``pico_anchored_v1``), as ``leader_cartesian``'s ``anchored``: the hand
  pose is first put in the robot base frame with ``input_rotation`` (``base_R_input``, default
  identity): ``p_l = B p_hand``, ``R_l = B R_hand``. At the take-over the tracker's current
  command ``(p_c, R_c)`` (the hold pose: the follower does not jump) and the hand pose
  ``(p_l0, R_l0)`` are stored; then ``p* = p_c + (p_l - p_l0)``, ``R* = (R_l R_l0^T) R_c``. A
  hand rotation turns the follower in place. The tracker limits the command to the target speed
  and to ``lead_cap_m``/``lead_cap_rad`` ahead of the measured pose; the action is
  ``(command - measured)`` in the env's units.
- **Hand-back** puts the tracker in hold at the measured pose (``hold_at``); the recorder's hold
  policy (``real_world_rl.teleop.core.hold``) shares the tracker. A requested position outside
  the safety box or the station workspace (``WORKSPACE HOLD: <axes>``, logged), an input or
  tracking failure and an episode end hand back too: the next bind press takes over again, from
  wherever the hand is then. A stalled control loop (more than ``max_dt_s`` between two
  intervened polls) holds at the measured pose and the next poll re-anchors the hand there.
- **Gripper**: the trigger's width target ``w`` (1 = open) gives ``+1`` if
  ``w > gripper_binarize_threshold`` else ``-1`` (default 0.5; ``None`` emits ``2w - 1``), as
  ``leader_cartesian``. With ``last_gripper_action_getter`` (the intervention wrapper passes it),
  a trigger that disagrees at the take-over with the gripper's last commanded state is ignored
  until it crosses to that state, so taking over never drops a grasped object.
- **Episode end**: the Pico's episode-end press ends the episode.

Under ``CompositeTeleopSource`` (``--teleop-event-device stdin``/``keyboard``) the event device
adds the outcome labels, episode end and quit; bind stays the only take-over, and a change of the
event device's take-over state (``t``/``h``) logs a warning saying so.

The device is a Pico subscriber built here (``address``, ``topic``, ``hand``, ``bind_button``,
``poll_interval_s``, ``hold_s``; ``start()`` connects it, ``close()`` disconnects it), a
caller-owned teleop part or resolver (``teleop``), the teleop part of the ``real_robot`` env's own
robot (``from_env(robot_teleop=True)``: a robot yaml ``teleop: {backend: pico}``, local or served
by the robot daemon), or the teleop part of another robot daemon (``from_env(remote_address=...)``,
connected in ``start()``). ``pyzmq`` is imported when the part connects, so building the source
works without it.

``device="pico_twist"`` (``twist_sim``) keeps the former raw-twist semantics for the ManiSkill sim
examples only.
"""
from __future__ import annotations

import logging
import time
from typing import Any, Callable, Mapping, Optional, Sequence, Union

import numpy as np

from real_world_rl.devices.teleop import Teleop, TeleopConfig
from real_world_rl.kinematics.transforms import matrix_to_quat_xyzw, pose_to_transform, quat_xyzw_to_matrix
from real_world_rl.teleop.core.composite import ToggleableTeleopSource
from real_world_rl.teleop.core.contract import TeleopReading, held_gripper_action
from real_world_rl.teleop.core.part import (
    DEFAULT_READ_TIMEOUT_S,
    TeleopPartSource,
    TeleopResolver,
)
from real_world_rl.teleop.core.registry import register_teleop_source
from real_world_rl.teleop.core.targets.cartesian_driver import (
    CartesianTrackerDriver,
    tracker_control_spec,
    validated_tracker_config,
)
from real_world_rl.teleop.core.targets.cartesian_tracker import CartesianTargetTracker
from real_world_rl.teleop.core.targets.follower import PoseGetter
from real_world_rl.teleop.core.targets.twist_input import validate_input_rotation
from real_world_rl.teleop.pico import backend as _pico_backend  # registers "pico"

__all__ = ["PICO_CONTRACT", "PicoTeleopSource", "TARGET_MAPPING"]

#: The mapping version a reading's diagnostics name (``target_mapping``). Provenance only: the
#: compared ``teleop_control`` block is the tracker's (``cartesian_tracker_v1``).
TARGET_MAPPING = "pico_anchored_v1"
#: What every Pico part declares: a ``cartesian_delta`` sample with a gripper channel.
PICO_CONTRACT = ("cartesian_delta", 6, True)
_CARTESIAN_ARM_DIM = 6
_FORWARDED_EVENTS = ("success", "failure", "rerecord")
_DEFAULT_REMOTE_NAME = "pico"

logger = logging.getLogger(__name__)


@register_teleop_source("pico")
class PicoTeleopSource(TeleopPartSource, ToggleableTeleopSource):
    """A Pico hand pose driving the follower's command tracker (module docstring).

    ``follower_pose_getter`` returns the follower's measured TCP pose ``xyz + quat_xyzw`` in the
    base frame (None before the env's first reset). ``pos_scale_m``/``rot_scale_rad`` are the
    env's per-step scales and must be at least ``lead_cap_m``/``lead_cap_rad``. ``clock`` (a
    monotonic clock in seconds) times the tracker's advances. Read failures propagate; nothing
    ever turns into an idle reading.
    """

    TRACKS_CARTESIAN_TARGET = True

    @classmethod
    def control_spec(cls, *, tracker_options, source_options) -> dict:
        """The tracker's block: the Pico's own options are not compared."""
        return tracker_control_spec(tracker_options)

    def __init__(
        self,
        teleop: Union[Any, TeleopResolver, None] = None,
        *,
        follower_pose_getter: PoseGetter,
        teleop_robot: Optional[Any] = None,
        env_contract: Optional[Mapping[str, Any]] = None,
        last_gripper_action_getter: Optional[Callable[[], Any]] = None,
        address: str = _pico_backend.DEFAULT_ADDRESS,
        topic: str = "",
        hand: str = "right",
        bind_button: str = "bind",
        poll_interval_s: float = 0.005,
        hold_s: float = 0.0,
        input_rotation=None,
        gripper_binarize_threshold: Optional[float] = 0.5,
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
        from real_world_rl.action_contract import LEGACY_ACTION_CONTRACT

        if teleop is not None and teleop_robot is not None:
            raise ValueError("pass at most one Pico: teleop=<part or resolver> or teleop_robot=<robot>")
        tracker_config = validated_tracker_config(
            "pico",
            pos_scale_m=pos_scale_m,
            rot_scale_rad=rot_scale_rad,
            lead_cap_m=lead_cap_m,
            lead_cap_rad=lead_cap_rad,
            target_speed_m_s=target_speed_m_s,
            target_speed_rad_s=target_speed_rad_s,
            max_dt_s=max_dt_s,
        )
        self._tracker = CartesianTargetTracker(tracker_config, safety_box=safety_box,
                                               workspace=workspace, task_frame=task_frame)
        self._driver = CartesianTrackerDriver(
            self._tracker, source_name="pico", follower_pose_getter=follower_pose_getter,
            pos_scale_m=pos_scale_m, rot_scale_rad=rot_scale_rad, clock=clock,
        )
        self._input_rotation = validate_input_rotation(input_rotation)
        if gripper_binarize_threshold is not None and not 0.0 <= gripper_binarize_threshold <= 1.0:
            raise ValueError(f"gripper_binarize_threshold must be in [0, 1], got {gripper_binarize_threshold}")
        self._gripper_binarize_threshold = gripper_binarize_threshold

        self._owned_part = None
        self._teleop_robot = teleop_robot
        if teleop_robot is not None:
            teleop = lambda: teleop_robot.teleop  # noqa: E731  (known once the robot connects)
        elif teleop is None:
            config = TeleopConfig(
                name="pico",
                backend="pico",
                hold_s=hold_s,
                options={"address": address, "topic": topic, "hand": hand, "bind_button": bind_button,
                         "poll_interval_s": poll_interval_s},
            )
            self._owned_part = Teleop.backend("pico")(config, read_timeout_s=read_timeout_s)
            teleop = self._owned_part
        super().__init__(
            teleop,
            env_contract=LEGACY_ACTION_CONTRACT if env_contract is None else env_contract,
            teleop_contract=PICO_CONTRACT,
            last_gripper_action_getter=last_gripper_action_getter,
            read_timeout_s=read_timeout_s,
        )

        self._driving = False
        self._previously_driving = False
        self._previous_event_toggle = False
        self._anchor: Optional[tuple[np.ndarray, np.ndarray]] = None  # (p_c0 - p_l0, R_l0^T R_c0)
        self._gripper_latched = False

    @property
    def cartesian_tracker(self) -> CartesianTargetTracker:
        """The follower's command tracker, shared with the hold policy."""
        return self._tracker

    @property
    def teleop_robot(self) -> Optional[Any]:
        """The robot this source owns (connected by ``start()``), or None."""
        return self._teleop_robot

    @classmethod
    def from_env(
        cls,
        env: Any,
        *,
        robot_teleop: bool = False,
        remote_address: Optional[str] = None,
        remote_token: Optional[str] = None,
        remote_name: str = _DEFAULT_REMOTE_NAME,
        **kwargs: Any,
    ) -> "PicoTeleopSource":
        """Source for ``env``. Unless passed, the follower pose, the scales, the safety box and the
        station workspace/task frame come from the env, as for ``spacemouse``
        (``follower_kwargs_from_env``, ``workspace_from_env``). The Pico is, in this order:
        ``teleop`` (a part the caller owns); with ``robot_teleop=True`` the teleop part of the
        robot behind the ``real_robot`` env (its robot yaml's ``teleop: {backend: pico}``, local or
        served by the robot daemon; the robot owns it); with ``remote_address`` (``host:port``) the
        teleop part of another robot daemon (a ``RemoteRobot`` connected in ``start()``, as for
        ``leader_cartesian`` on the ``franka_real`` stack); else a Pico subscriber built here."""
        from real_world_rl.teleop.core.targets.follower import (
            follower_kwargs_from_env,
            workspace_from_env,
        )

        if robot_teleop and remote_address is not None:
            raise ValueError("pico: pass robot_teleop=True or remote_address, not both")
        follower_kwargs_from_env(env, kwargs)
        workspace_from_env(env, kwargs)
        if robot_teleop:
            from real_world_rl.teleop.core.part import _find_last_observation_owner

            robot = getattr(_find_last_observation_owner(env), "robot", None)
            if robot is None:
                raise ValueError(
                    "pico robot_teleop=True reads the teleop part of the real_robot env's robot; "
                    "this env has no device-layer robot (use address=... or remote_address=...)"
                )
            kwargs["teleop"] = lambda: robot.teleop
        elif remote_address is not None:
            from real_world_rl.server import RemoteRobot, RemoteRobotConfig

            kwargs["teleop_robot"] = RemoteRobot(
                RemoteRobotConfig(name=remote_name, address=remote_address, token=remote_token)
            )
        return cls(**kwargs)

    # -- source protocol -----------------------------------------------------------

    def start(self) -> None:
        """Connects the Pico part this source built, or the robot it owns (and checks its part)."""
        if self._owned_part is not None:
            self._owned_part.connect()
        elif self._teleop_robot is not None:
            self._teleop_robot.connect()
            try:
                self._require_part()
            except BaseException:
                self._teleop_robot.disconnect()
                raise

    def reset(self) -> None:
        """After the env reset: handed back, the tracker holds the follower's pose, and the
        part's latched presses are dropped."""
        self._driving = False
        self._previously_driving = False
        self._previous_event_toggle = False
        self._anchor = None
        self._gripper_latched = False
        self._driver.reset()
        super().reset()

    def poll(self) -> TeleopReading:
        """One Pico read; the bind press is the operator's take-over toggle."""
        return self._poll()

    def poll_with_toggle(self, toggle: bool) -> TeleopReading:
        """``poll()``: the event device's take-over state is not used (bind is the toggle); a
        change of it logs a warning."""
        toggle = bool(toggle)
        if toggle != self._previous_event_toggle:
            logger.warning(
                "the Pico ignores the event device's take-over/hand-back keys (t/h); press the "
                "Pico's bind button to take over or hand back"
            )
        self._previous_event_toggle = toggle
        return self._poll()

    def close(self) -> None:
        if self._owned_part is not None:
            self._owned_part.disconnect()
        elif self._teleop_robot is not None:
            self._teleop_robot.disconnect()

    # -- polling -------------------------------------------------------------------

    def _poll(self) -> TeleopReading:
        try:
            teleop = self._require_part()
            sample = teleop.read(self._read_timeout_s)
            if sample.action_space != PICO_CONTRACT[0]:
                raise self._device_fault(
                    f"pico sample has action_space {sample.action_space!r}, expected {PICO_CONTRACT[0]!r}", teleop)
            try:
                hand_pose = _pico_backend.hand_pose_of(sample)
            except ValueError as error:
                raise self._device_fault(str(error), teleop) from None
        except BaseException:
            self._hand_back_after_failure()
            raise
        if sample.events.get(_pico_backend.BIND_EVENT, False):
            self._driving = not self._driving
        episode_end = bool(sample.events.get(_pico_backend.EPISODE_END_EVENT, False))
        measured = self._driver.measured_pose()
        if not self._tracker.initialized:
            self._tracker.reset(measured)
        takeover = self._driving and not self._previously_driving
        hand = self._base_frame_pose(hand_pose)
        intervened, arm = self._arm_action(hand, measured)
        driving = self._driving
        if episode_end:
            if self._tracker.following:
                self._tracker.hold_at(measured)
                arm = self._driver.action_for(measured)
            self._driving = False
        self._previously_driving = self._driving
        parts = [arm]
        if self._env_gripper:
            parts.append(np.array([self._gripper_action(sample, teleop, takeover, driving)]))
        return TeleopReading(
            action=np.concatenate(parts).astype(np.float32),
            intervened=intervened,
            episode_end=episode_end,
            events={key: True for key in _FORWARDED_EVENTS if sample.events.get(key, False)},
            timestamp=float(getattr(sample, "timestamp", 0.0)),
            diagnostics={"target_mapping": TARGET_MAPPING, "hand_pose": hand_pose.tolist()},
        )

    def _base_frame_pose(self, hand_pose: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """``(p_l, R_l)``: the hand pose with ``input_rotation`` applied (``B p``, ``B R``)."""
        position, rotation = hand_pose[:3], quat_xyzw_to_matrix(hand_pose[3:])
        if self._input_rotation is not None:
            position, rotation = self._input_rotation @ position, self._input_rotation @ rotation
        return position, rotation

    def _arm_action(self, hand: tuple[np.ndarray, np.ndarray], measured: np.ndarray) -> tuple[bool, np.ndarray]:
        """``(intervened, arm action)``; the arm action only matters while intervened."""
        idle = np.zeros(_CARTESIAN_ARM_DIM)
        tracker = self._tracker
        if not self._driving:
            if tracker.following:
                tracker.hold_at(measured)  # hand-back
            return False, idle
        if not tracker.following:  # a take-over, or resuming after a stalled loop
            self._take_over(hand)
        try:
            update = tracker.advance(self._requested_pose(hand), measured, self._driver.elapsed_s())
            if update.workspace_violations:
                self._driving = False
                logger.warning(
                    "WORKSPACE HOLD: %s; the follower holds its pose and the Pico handed back, "
                    "press bind to take over again",
                    ", ".join(update.workspace_violations),
                )
                return False, idle
            if update.missed_deadline_s is not None:
                logger.warning(
                    "control loop stalled: %.3f s between two Pico commands (max %.3f s); the "
                    "follower holds its pose and the hand is re-anchored at the next poll",
                    update.missed_deadline_s,
                    tracker.config.max_dt_s,
                )
            return True, self._driver.action_for(measured)
        except BaseException:
            self._hand_back_after_failure()
            raise

    def _take_over(self, hand: tuple[np.ndarray, np.ndarray]) -> None:
        self._tracker.take_over()
        self._driver.restart_clock()
        position, rotation = hand
        command = pose_to_transform(self._tracker.command)
        # p_c0 - p_l0 and R_l0^T R_c0: p* = p_l + (p_c0 - p_l0), R* = R_l (R_l0^T R_c0).
        self._anchor = (command[:3, 3] - position, rotation.T @ command[:3, :3])

    def _requested_pose(self, hand: tuple[np.ndarray, np.ndarray]) -> np.ndarray:
        position, rotation = hand
        position_offset, rotation_offset = self._anchor
        return np.concatenate([position + position_offset, matrix_to_quat_xyzw(rotation @ rotation_offset)])

    def _hand_back_after_failure(self) -> None:
        """Input or tracking failure: hand back and hold the measured pose."""
        self._driving = False
        self._previously_driving = False
        self._driver.hold_after_failure()

    # -- gripper -------------------------------------------------------------------

    def _gripper_action(self, sample: Any, teleop: Any, takeover: bool, driving: bool) -> float:
        if sample.gripper is None:
            raise self._device_fault("pico sample carries no gripper (trigger) value", teleop)
        trigger = self._trigger_action(float(sample.gripper))
        if self._last_gripper_action is None:
            return trigger
        if not driving:
            self._gripper_latched = False
            return trigger
        held = held_gripper_action(self._last_gripper_action)
        if takeover and _gripper_open(trigger) != _gripper_open(held):
            self._gripper_latched = True
            logger.info(
                "Pico trigger disagrees with the gripper (%s); holding it until the trigger is %s too",
                "open" if _gripper_open(held) else "closed",
                "open" if _gripper_open(held) else "closed",
            )
        if self._gripper_latched:
            if _gripper_open(trigger) != _gripper_open(held):
                return held
            self._gripper_latched = False
        return trigger

    def _trigger_action(self, width: float) -> float:
        if self._gripper_binarize_threshold is None:
            return float(np.clip(2.0 * width - 1.0, -1.0, 1.0))
        return 1.0 if width > self._gripper_binarize_threshold else -1.0

    # -- part checks ---------------------------------------------------------------

    def _require_part(self) -> Any:
        teleop = self._resolve()
        if teleop is None:
            raise self._device_fault("the Pico robot has no teleop part (not connected, or no teleop: section)")
        self._check_declaration(teleop)
        return teleop

    @staticmethod
    def _device_fault(message: str, teleop: Any = None) -> Exception:
        from real_world_rl.devices.errors import DeviceFault

        return DeviceFault(
            message,
            hint="the pico source reads the device layer's pico teleop (backend pico: ZeroMQ "
            "address, hand, bind_button); check that the Pico publisher is running and reachable",
            device=getattr(teleop, "name", "pico"),
        )


def _gripper_open(action: float) -> bool:
    return action > 0.0
