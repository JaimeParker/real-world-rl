"""The Pico controller's ZeroMQ message format, and the legacy twist bridge.

The Pico side publishes JSON arrays: one hand record (13 values) or both (26, left first). A
hand record holds the position ``[0:3]``, the orientation quaternion ``xyzw`` ``[6:10]``, the
trigger ``[10]``, the bind button ``[11]`` and the episode-end button ``[12]``.
``hand_record`` selects one hand's record from a message; ``device_sample`` reads it. The
device layer's ``PicoTeleop`` (``real_world_rl.teleop.pico.backend``) uses both.

``EETwistTeleOpWrapper`` is the legacy client: it opens its ZMQ subscriber at construction and
turns hand-pose differences into a clipped EE twist (``HandPoseToEETwist``). Only the ManiSkill
sim examples (``device="pico_twist"``) and ``examples/run_twist_input_inspector.py`` use it; a
real robot takes ``device="pico"`` (the anchored pose tracker).
"""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

import numpy as np

HAND_RECORD_SIZE = 13
POSITION_SLICE = slice(0, 3)
QUATERNION_SLICE = slice(6, 10)
TRIGGER_INDEX = 10
BIND_INDEX = 11
EPISODE_END_INDEX = 12
HANDS = ("left", "right")

TRACKING_RATIO = 0.39
TRACKING_COMPENSATION = 1.0 / TRACKING_RATIO
DEFAULT_TWIST_LIMIT = 0.1


RIGHT_ALIGN = (
    np.array([[0.0, 0.0, 1.0], [0.0, 1.0, 0.0], [-1.0, 0.0, 0.0]])
    @ np.array([[1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, -1.0, 0.0]])
)
LEFT_ALIGN = (
    np.array([[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]])
    @ RIGHT_ALIGN
)


def _skew(v: np.ndarray) -> np.ndarray:
    return np.array(
        [[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]], [-v[1], v[0], 0.0]],
        dtype=np.float64,
    )


def _se3_log(omega: np.ndarray, pos: np.ndarray) -> np.ndarray:
    theta = np.linalg.norm(omega)
    omega_hat = _skew(omega)
    if theta < 1e-8:
        v_inv = np.eye(3) - 0.5 * omega_hat + (omega_hat @ omega_hat) / 12.0
    else:
        a = 1.0 / theta**2 - (1.0 + np.cos(theta)) / (2.0 * theta * np.sin(theta))
        v_inv = np.eye(3) - 0.5 * omega_hat + a * (omega_hat @ omega_hat)
    return np.concatenate([v_inv @ pos, omega]).astype(np.float32)


def _quat(quat_xyzw: np.ndarray) -> np.ndarray:
    quat = np.asarray(quat_xyzw, dtype=np.float64).reshape(4)
    norm = np.linalg.norm(quat)
    if norm < 1e-8:
        raise ValueError("Invalid hand quaternion.")
    return quat / norm


def _quat_conj(q: np.ndarray) -> np.ndarray:
    return np.array([-q[0], -q[1], -q[2], q[3]], dtype=np.float64)


def _quat_mul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return np.array(
        [
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz,
        ],
        dtype=np.float64,
    )


def _quat_rotate(q: np.ndarray, v: np.ndarray) -> np.ndarray:
    xyz = q[:3]
    w = q[3]
    return (
        2.0 * np.dot(xyz, v) * xyz
        + (w * w - np.dot(xyz, xyz)) * v
        + 2.0 * w * np.cross(xyz, v)
    )


def _quat_to_rotvec(q: np.ndarray) -> np.ndarray:
    q = _quat(q)
    if q[3] < 0.0:
        q = -q
    vec = q[:3]
    sin_half = np.linalg.norm(vec)
    if sin_half < 1e-8:
        return 2.0 * vec
    angle = 2.0 * np.arctan2(sin_half, q[3])
    return vec * (angle / sin_half)


class ButtonStatus(Enum):
    RELEASED = 0
    JUST_PRESSED = 1
    PRESSED = 2
    JUST_RELEASED = 3


@dataclass(frozen=True)
class DeviceSample:
    hand_pos: np.ndarray
    hand_quat: np.ndarray
    gripper_cmd: float
    bind_pressed: bool
    episode_end_pressed: bool


def hand_record(data: np.ndarray, hand: str) -> np.ndarray:
    """The 13-value record of ``hand`` in a Pico message (13 values: the only hand; 26 or more:
    left first, then right)."""
    if hand not in HANDS:
        raise ValueError(f"hand must be one of {list(HANDS)}, got {hand!r}")
    data = np.asarray(data).reshape(-1)
    if data.size == HAND_RECORD_SIZE:
        return data
    if data.size >= 2 * HAND_RECORD_SIZE:
        return data[:HAND_RECORD_SIZE] if hand == "left" else data[HAND_RECORD_SIZE:2 * HAND_RECORD_SIZE]
    raise ValueError(
        f"Pico data must contain {HAND_RECORD_SIZE} or {2 * HAND_RECORD_SIZE} values, got {data.size}."
    )


def device_sample(record: np.ndarray) -> DeviceSample:
    """One hand record read as positions, quaternion and button levels."""
    return DeviceSample(
        hand_pos=record[POSITION_SLICE],
        hand_quat=record[QUATERNION_SLICE],
        gripper_cmd=float(record[TRIGGER_INDEX]),
        bind_pressed=bool(record[BIND_INDEX]),
        episode_end_pressed=bool(record[EPISODE_END_INDEX]),
    )


@dataclass(frozen=True)
class TeleOpSample:
    action: np.ndarray
    twist: np.ndarray
    gripper: float
    bind_pressed: bool
    episode_end: bool
    intervened: bool


@dataclass
class HandPoseToEETwist:
    hand: str = "right"
    pos_scale: float = 1.0
    rot_scale: float = 1.0
    twist_limit: float = 0.1

    def __post_init__(self):
        if self.hand not in ("left", "right"):
            raise ValueError("hand must be 'left' or 'right'.")
        self.align = LEFT_ALIGN if self.hand == "left" else RIGHT_ALIGN
        self.bound = False
        self._last_pos: np.ndarray | None = None
        self._last_quat: np.ndarray | None = None

    def bind(self, hand_pos: np.ndarray, hand_quat: np.ndarray) -> None:
        self._last_pos = np.asarray(hand_pos, dtype=np.float64).reshape(3).copy()
        self._last_quat = _quat(hand_quat)
        self.bound = True

    def reset(self) -> None:
        self.bound = False
        self._last_pos = None
        self._last_quat = None

    def compute(self, hand_pos: np.ndarray, hand_quat: np.ndarray):
        pos = np.asarray(hand_pos, dtype=np.float64).reshape(3)
        quat = _quat(hand_quat)
        if not self.bound or self._last_pos is None or self._last_quat is None:
            return np.zeros(6, dtype=np.float32)

        last_inv = _quat_conj(self._last_quat)
        delta_pos_hand = _quat_rotate(last_inv, pos - self._last_pos)
        delta_rotvec_hand = _quat_to_rotvec(_quat_mul(last_inv, quat))
        self._last_pos = pos.copy()
        self._last_quat = quat

        delta_pos_ee = self.align @ delta_pos_hand
        delta_rotvec_ee = self.align @ delta_rotvec_hand
        twist = _se3_log(delta_rotvec_ee, delta_pos_ee)
        twist[:3] *= self.pos_scale
        twist[3:] *= self.rot_scale
        return np.clip(twist, -self.twist_limit, self.twist_limit).astype(np.float32)


class EETwistTeleOpWrapper:
    """Legacy: a ZMQ subscriber opened at construction, polled into a clipped EE twist (module
    docstring). For the sim examples and the inspector only; a real robot uses ``device="pico"``."""

    def __init__(
        self,
        zmq_url: str = "tcp://localhost:7777",
        hand: str = "right",
        device: str = "pico",
        pos_scale: float = TRACKING_COMPENSATION,
        rot_scale: float = TRACKING_COMPENSATION,
        twist_limit: float = DEFAULT_TWIST_LIMIT,
        intervention_threshold: float = 1e-4,
    ):
        if device not in ("pico", "spacemouse"):
            raise ValueError("device must be 'pico' or 'spacemouse'.")
        if device == "spacemouse":
            raise NotImplementedError(
                "SpaceMouse teleoperation parser is not implemented yet."
            )

        import zmq

        self.zmq = zmq
        self.device = device
        self.hand = hand
        self.mapper = HandPoseToEETwist(hand, pos_scale, rot_scale, twist_limit)
        self.intervention_threshold = float(intervention_threshold)
        self.reset()
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.SUB)
        self.socket.connect(zmq_url)
        self.socket.setsockopt_string(zmq.SUBSCRIBE, "")

    def reset(self, *, episode_end_pressed: bool = False) -> None:
        self.mapper.reset()
        self.button_state = ButtonStatus.PRESSED
        self.episode_button_state = (
            ButtonStatus.PRESSED if episode_end_pressed else ButtonStatus.RELEASED
        )
        self.last_gripper = 1.0
        self.last_received = False

    @staticmethod
    def _gripper_action(cmd: float) -> float:
        return -1.0 if cmd == 1 else 1.0

    @staticmethod
    def _update_button_state(pressed: bool, button_state: ButtonStatus):
        if pressed and button_state in [
            ButtonStatus.RELEASED,
            ButtonStatus.JUST_RELEASED,
        ]:
            return ButtonStatus.JUST_PRESSED
        if pressed and button_state in [ButtonStatus.JUST_PRESSED, ButtonStatus.PRESSED]:
            return ButtonStatus.PRESSED
        if not pressed and button_state in [
            ButtonStatus.PRESSED,
            ButtonStatus.JUST_PRESSED,
        ]:
            return ButtonStatus.JUST_RELEASED
        if not pressed and button_state in [
            ButtonStatus.JUST_RELEASED,
            ButtonStatus.RELEASED,
        ]:
            return ButtonStatus.RELEASED
        return button_state

    def _recv_latest(self) -> np.ndarray | None:
        latest = None
        while True:
            try:
                latest = self.socket.recv_json(flags=self.zmq.NOBLOCK)
            except self.zmq.Again:
                break
        return None if latest is None else np.asarray(latest, dtype=np.float32)

    def _parse_pico(self, data: np.ndarray) -> DeviceSample:
        return device_sample(hand_record(data, self.hand))

    def _parse_device(self, data: np.ndarray) -> DeviceSample:
        if self.device == "pico":
            return self._parse_pico(data)
        raise NotImplementedError(
            "SpaceMouse teleoperation parser is not implemented yet."
        )

    def _fallback_sample(self) -> TeleOpSample:
        self.last_received = False
        twist = np.zeros(6, dtype=np.float32)
        return TeleOpSample(
            action=np.concatenate([twist, [self.last_gripper]]).astype(np.float32),
            twist=twist,
            gripper=self.last_gripper,
            bind_pressed=False,
            episode_end=False,
            intervened=False,
        )

    def poll(self) -> TeleOpSample:
        data = self._recv_latest()
        if data is None:
            return self._fallback_sample()
        self.last_received = True

        parsed = self._parse_device(data)
        prev_gripper = self.last_gripper

        self.button_state = self._update_button_state(
            parsed.bind_pressed, self.button_state
        )
        self.episode_button_state = self._update_button_state(
            parsed.episode_end_pressed, self.episode_button_state
        )

        if self.button_state in [ButtonStatus.JUST_PRESSED, ButtonStatus.PRESSED]:
            self.mapper.bind(parsed.hand_pos, parsed.hand_quat)

        twist = self.mapper.compute(parsed.hand_pos, parsed.hand_quat)
        if parsed.bind_pressed:
            twist = np.zeros(6, dtype=np.float32)

        self.last_gripper = self._gripper_action(parsed.gripper_cmd)
        episode_end = self.episode_button_state == ButtonStatus.JUST_PRESSED
        intervened = (
            np.linalg.norm(twist) > self.intervention_threshold
            or self.last_gripper != prev_gripper
        )
        action = np.concatenate([twist, [self.last_gripper]]).astype(np.float32)
        return TeleOpSample(
            action=action,
            twist=twist,
            gripper=self.last_gripper,
            bind_pressed=parsed.bind_pressed,
            episode_end=episode_end,
            intervened=bool(intervened),
        )

    def get_ee_twist(self) -> np.ndarray:
        return self.poll().twist

    def get_action(self) -> np.ndarray:
        return self.poll().action

    def close(self) -> None:
        self.socket.close()
        self.context.term()
