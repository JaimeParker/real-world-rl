"""Pico VR controller over ZeroMQ: one hand's pose and buttons as a device-layer teleop part.

The Pico side publishes JSON arrays (``real_world_rl.teleop.pico.bridge``: one 13-value hand
record, or both hands). The poll thread subscribes to ``address`` and, every
``poll_interval_s``, reads every message queued since the previous poll: button presses are
taken from all of them (a press shorter than the poll interval is not lost), the pose from the
newest. A reading carries:

- ``diagnostics["hand_pose"]``: the hand pose ``xyz + quat_xyzw`` in the Pico tracking frame
  (``hand_pose_of`` reads it back, also from a sample served by the robot daemon);
- ``action``: the hand's displacement since the previous reading in the same frame
  (``[dp, rotvec]``, unscaled, clipped to ``[-1, 1]``), so the part keeps the
  ``cartesian_delta`` sample shape of every teleop part; the ``pico`` source does not read it;
- ``gripper``: the trigger as a width target in ``[0, 1]``, 1 = open (trigger released);
- ``buttons``: the levels ``bind`` and ``episode_end``; ``events``: their presses, latched until
  the next ``read()``; ``active``: the bind button is held.

The first message after a connect or ``reset()`` only sets the button baseline, so a button held
through it does not count as a press. ``pyzmq`` is imported when the device opens.
"""
from __future__ import annotations

import json
import threading
import time
from dataclasses import dataclass
from types import ModuleType
from typing import Any, Optional

import numpy as np

from real_world_rl.devices.errors import DeviceFault, SdkMissing
from real_world_rl.devices.teleop import RawReading, Teleop, TeleopConfig
from real_world_rl.kinematics.transforms import matrix_to_rotvec, quat_xyzw_to_matrix
from real_world_rl.teleop.pico.bridge import (
    BIND_INDEX,
    EPISODE_END_INDEX,
    HANDS,
    POSITION_SLICE,
    QUATERNION_SLICE,
    TRIGGER_INDEX,
    hand_record,
)

__all__ = [
    "BIND_EVENT",
    "DEFAULT_ADDRESS",
    "EPISODE_END_EVENT",
    "HAND_POSE_KEY",
    "PYZMQ_REQUIREMENT",
    "PicoTeleop",
    "hand_pose_of",
]

DEFAULT_ADDRESS = "tcp://localhost:7777"
PYZMQ_REQUIREMENT = "pyzmq"
HAND_POSE_KEY = "hand_pose"
BIND_EVENT = "bind"
EPISODE_END_EVENT = "episode_end"
# ``bind_button`` -> (record index of the take-over button, record index of the episode-end button).
BIND_BUTTONS = {
    "bind": (BIND_INDEX, EPISODE_END_INDEX),
    "episode_end": (EPISODE_END_INDEX, BIND_INDEX),
}
_TRIGGER_PRESSED = 0.5
_OPTION_KEYS = {"address", "topic", "hand", "bind_button", "poll_interval_s"}
_SDK_HINT = f"pip install {PYZMQ_REQUIREMENT} (or, from the repository root: pip install -e '.[pico]')"
_sleep = time.sleep  # module-level so tests can observe the poll cadence


def hand_pose_of(sample: Any) -> np.ndarray:
    """The hand pose ``xyz + quat_xyzw`` (Pico tracking frame) a Pico part put in ``sample``'s
    diagnostics. Raises ``ValueError`` for a sample without one (not a Pico part)."""
    pose = (getattr(sample, "diagnostics", None) or {}).get(HAND_POSE_KEY)
    if pose is None:
        raise ValueError(f"the teleop sample carries no diagnostics[{HAND_POSE_KEY!r}] (not a Pico part)")
    pose = np.asarray(pose, dtype=np.float64).reshape(-1)
    if pose.shape != (7,) or not np.isfinite(pose).all():
        raise ValueError(f"diagnostics[{HAND_POSE_KEY!r}] must be 7 finite numbers, got {pose.tolist()}")
    return pose


@dataclass
class _PicoHandle:
    zmq: ModuleType
    context: Any
    socket: Any


@Teleop.register("pico")
class PicoTeleop(Teleop):
    """Options (``TeleopConfig.options``): ``address`` (the Pico publisher, default
    ``tcp://localhost:7777``), ``topic`` (subscription prefix, default ``""``: every message;
    with a topic the publisher sends ``[topic, json]`` frames), ``hand`` (``right`` | ``left``,
    which record of a two-hand message), ``bind_button`` (``bind`` | ``episode_end``: which of the
    two record buttons takes over; the other ends the episode), ``poll_interval_s`` (default
    0.005). ``TeleopConfig.hold_s`` extends ``intervening`` past the bind button's release.
    ``discover()`` finds nothing: a Pico is a network publisher, not a local device."""

    SDK = "zmq"
    ACTION_SPACE = "cartesian_delta"
    HAS_GRIPPER = True
    PIP_REQUIREMENTS = (PYZMQ_REQUIREMENT,)

    @classmethod
    def require_sdk(cls) -> ModuleType:
        try:
            return super().require_sdk()
        except SdkMissing as error:
            raise SdkMissing(error.message, hint=_SDK_HINT, device=error.device) from None

    def __init__(self, config: TeleopConfig, **kwargs: Any) -> None:
        super().__init__(config, **kwargs)
        options = dict(config.options)
        for key in options:
            if key not in _OPTION_KEYS:
                raise ValueError(f"unknown pico option '{key}'; known: {sorted(_OPTION_KEYS)}")
        self._address = str(options.get("address", DEFAULT_ADDRESS))
        self._topic = str(options.get("topic", ""))
        self._hand = str(options.get("hand", "right"))
        if self._hand not in HANDS:
            raise ValueError(f"pico hand must be one of {list(HANDS)}, got {self._hand!r}")
        bind_button = str(options.get("bind_button", "bind"))
        if bind_button not in BIND_BUTTONS:
            raise ValueError(f"pico bind_button must be one of {list(BIND_BUTTONS)}, got {bind_button!r}")
        self._bind_index, self._episode_end_index = BIND_BUTTONS[bind_button]
        self._poll_interval_s = float(options.get("poll_interval_s", 0.005))
        if self._poll_interval_s < 0:
            raise ValueError(f"poll_interval_s must be >= 0, got {self._poll_interval_s}")
        self._state_lock = threading.Lock()
        self._previous_levels: Optional[tuple[bool, bool]] = None  # None: the next message is the baseline
        self._previous_pose: Optional[np.ndarray] = None

    @property
    def address(self) -> str:
        return self._address

    # -- device ----------------------------------------------------------------

    def _open_device(self) -> _PicoHandle:
        zmq = self.require_sdk()
        context = zmq.Context()
        socket = context.socket(zmq.SUB)
        try:
            socket.setsockopt(zmq.LINGER, 0)
            socket.connect(self._address)
            socket.setsockopt_string(zmq.SUBSCRIBE, self._topic)
        except Exception as error:
            socket.close()
            context.term()
            raise DeviceFault(
                f"pico subscriber could not connect to {self._address}: {error}",
                hint="check the pico address option (tcp://<publisher-host>:<port>)",
                device=self.name,
            ) from error
        with self._state_lock:
            self._reset_state()
        return _PicoHandle(zmq=zmq, context=context, socket=socket)

    def _close_device(self, handle: _PicoHandle) -> None:
        handle.socket.close()
        handle.context.term()

    def _reset_state(self) -> None:
        self._previous_levels = None
        self._previous_pose = None

    def _reset_device(self, handle: Any) -> None:
        with self._state_lock:
            self._reset_state()

    def _poll_device(self, handle: _PicoHandle) -> Optional[RawReading]:
        _sleep(self._poll_interval_s)
        records = [hand_record(_decode(frames[-1]), self._hand) for frames in _drain(handle)]
        if not records:
            return None
        pressed = {BIND_EVENT: False, EPISODE_END_EVENT: False}
        with self._state_lock:
            for record in records:
                levels = (bool(record[self._bind_index]), bool(record[self._episode_end_index]))
                previous = levels if self._previous_levels is None else self._previous_levels
                pressed[BIND_EVENT] |= levels[0] and not previous[0]
                pressed[EPISODE_END_EVENT] |= levels[1] and not previous[1]
                self._previous_levels = levels
            pose = _pose(records[-1])
            displacement = _displacement(self._previous_pose, pose)
            self._previous_pose = pose
            bind_level, episode_end_level = self._previous_levels
        trigger = float(np.clip(records[-1][TRIGGER_INDEX], 0.0, 1.0))
        return RawReading(
            action=np.clip(displacement, -1.0, 1.0),
            gripper=1.0 - trigger,
            buttons={BIND_EVENT: bind_level, EPISODE_END_EVENT: episode_end_level,
                     "trigger": trigger >= _TRIGGER_PRESSED},
            events_now=pressed,
            active=bind_level,
            diagnostics={HAND_POSE_KEY: pose.tolist(), "messages": len(records)},
        )


def _drain(handle: _PicoHandle) -> list:
    """Every message queued on the subscriber, oldest first."""
    messages = []
    while True:
        try:
            messages.append(handle.socket.recv_multipart(flags=handle.zmq.NOBLOCK))
        except handle.zmq.Again:
            return messages


def _decode(payload: Any) -> np.ndarray:
    if isinstance(payload, (bytes, bytearray, memoryview)):
        payload = bytes(payload).decode("utf-8")
    values = np.asarray(json.loads(payload), dtype=np.float64).reshape(-1)
    if not np.isfinite(values).all():
        raise ValueError("pico message holds non-finite values")
    return values


def _pose(record: np.ndarray) -> np.ndarray:
    quat = np.asarray(record[QUATERNION_SLICE], dtype=np.float64)
    norm = np.linalg.norm(quat)
    if norm < 1e-8:
        raise ValueError("pico hand quaternion has zero norm")
    return np.concatenate([np.asarray(record[POSITION_SLICE], dtype=np.float64), quat / norm])


def _displacement(previous: Optional[np.ndarray], pose: np.ndarray) -> np.ndarray:
    if previous is None:
        return np.zeros(6)
    rotation = quat_xyzw_to_matrix(pose[3:]) @ quat_xyzw_to_matrix(previous[3:]).T
    return np.concatenate([pose[:3] - previous[:3], matrix_to_rotvec(rotation)])
