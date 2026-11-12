"""GELLO leader arm: passive Dynamixel servos read over Protocol 2.0 as absolute joint targets.

The hardware design and the Dynamixel protocol usage originate from GELLO (MIT license,
Philipp Wu, https://github.com/wuphilipp/gello_software). This backend is a re-implementation
on the ROBOTIS ``dynamixel_sdk``; no GELLO code is copied.

Decoding per joint: ``q = wrap((raw_rad - offset) * sign)`` into ``[c - pi, c + pi)`` around
``wrap_centers_rad[i]``, so no start pose is needed at connect as long as every joint range is
shorter than 2 pi. The gripper trigger maps to ``clip((raw - closed) / (open - closed), 0, 1)``,
1 = open. Torque stays off: the leader is moved by hand only."""
from __future__ import annotations

import glob
import importlib
import threading
import time
from dataclasses import dataclass, field
from types import ModuleType
from typing import Any, Mapping, Sequence

import numpy as np

from real_world_rl.devices.errors import DeviceFault, DeviceNotFound, SdkMissing
from real_world_rl.devices.teleop import EVENT_KEYS, RawReading, Teleop, TeleopConfig

DYNAMIXEL_SDK_REQUIREMENT = "dynamixel-sdk>=3.7.31,<3.9"
EVDEV_SDK = "evdev"
PROTOCOL_VERSION = 2.0
# X-series control table (XL330 and siblings).
ADDR_TORQUE_ENABLE = 64
ADDR_PRESENT_VELOCITY = 128
ADDR_PRESENT_POSITION = 132
LEN_PRESENT_POSITION = 4
SYNC_READ_LENGTH = 8  # present_velocity (4) + present_position (4) in one transaction
TORQUE_DISABLED = 0
TICKS_PER_HALF_TURN = 2048
SERIAL_BY_ID_GLOBS = ("/dev/serial/by-id/usb-FTDI_*", "/dev/serial/by-id/usb-ROBOTIS_*")
MAX_DYNAMIXEL_ID = 252
EVDEV_KEY_DOWN = 1
TOGGLE_ACTION = "toggle"
INTERVENTION_MODES = ("always", "toggle")
DEFAULT_EVENT_KEYS = {
    TOGGLE_ACTION: "KEY_SPACE",
    "episode_end": "KEY_ESC",
    "success": "KEY_S",
    "failure": "KEY_F",
    "rerecord": "KEY_R",
}
_KEY_ACTIONS = (TOGGLE_ACTION, *EVENT_KEYS)
_OPTION_KEYS = {
    "port",
    "baudrate",
    "joint_ids",
    "joint_offsets_rad",
    "joint_signs",
    "wrap_centers_rad",
    "gripper_id",
    "gripper_open_rad",
    "gripper_closed_rad",
    "gripper_binarize_threshold",
    "intervention",
    "event_device",
    "event_keys",
    "poll_interval_s",
}
_REQUIRED_OPTIONS = ("port", "joint_ids", "joint_offsets_rad", "joint_signs")
_GRIPPER_RANGE_OPTIONS = ("gripper_open_rad", "gripper_closed_rad")
_sleep = time.sleep  # module-level so tests can observe the poll cadence
_SDK_HINT = (
    f"pip install '{DYNAMIXEL_SDK_REQUIREMENT}' (or, from the repository root: pip install -e '.[gello]')"
)
_EVDEV_HINT = "pip install evdev (or, from the repository root: pip install -e '.[gello-events]')"
_PORT_HINT = (
    "check the USB cable and use the stable /dev/serial/by-id/... path; the user needs the "
    "'dialout' group (`sudo usermod -aG dialout $USER`, then log in again) or a udev rule; "
    "inside Docker pass the node with `docker run --device=<port>`; "
    "run `rwrl doctor`"
)


# -- decoding (pure) -----------------------------------------------------------


def signed_int32(value: int) -> int:
    """Two's complement of an unsigned 32-bit register value."""
    value = int(value) & 0xFFFFFFFF
    return value - 0x1_0000_0000 if value > 0x7FFFFFFF else value


def ticks_to_rad(ticks: Any) -> np.ndarray:
    return np.asarray(ticks, dtype=np.float64) / TICKS_PER_HALF_TURN * np.pi


def wrap_to_center(angle_rad: Any, center_rad: Any) -> np.ndarray:
    """Wrap into ``[center - pi, center + pi)``."""
    low = np.asarray(center_rad, dtype=np.float64) - np.pi
    return low + np.mod(np.asarray(angle_rad, dtype=np.float64) - low, 2 * np.pi)


def joint_positions_from_raw(
    raw_rad: Any, offsets_rad: Any, signs: Any, wrap_centers_rad: Any
) -> np.ndarray:
    raw = np.asarray(raw_rad, dtype=np.float64)
    unwrapped = (raw - np.asarray(offsets_rad, dtype=np.float64)) * np.asarray(signs, dtype=np.float64)
    return wrap_to_center(unwrapped, wrap_centers_rad)


def gripper_width_from_raw(
    raw_rad: float, open_rad: float, closed_rad: float, binarize_threshold: float | None = None
) -> float:
    """Normalized width target in [0, 1], 1 = open, whichever way the trigger turns."""
    width = float(np.clip((raw_rad - closed_rad) / (open_rad - closed_rad), 0.0, 1.0))
    if binarize_threshold is not None:
        width = 1.0 if width > binarize_threshold else 0.0
    return width


# -- bus -----------------------------------------------------------------------


class DynamixelBus:
    """One serial port with a set of Protocol 2.0 servos: ping, torque off, sync-read the
    present position. Shared by the teleop backend and the calibration CLI."""

    def __init__(
        self, sdk: ModuleType, port: str, baudrate: int, servo_ids: Sequence[int], *, device: str
    ) -> None:
        self._sdk = sdk
        self.port = port
        self.baudrate = baudrate
        self.servo_ids = [int(servo_id) for servo_id in servo_ids]
        self._device = device
        self._port_handler: Any = None
        self._packet_handler: Any = None
        self._sync_read: Any = None
        self._lock = threading.Lock()

    @property
    def is_open(self) -> bool:
        return self._port_handler is not None

    def open(self) -> None:
        """Open the port, check every id answers a ping, turn torque off and prepare the
        sync read. Never kills a process holding the port and never changes permissions."""
        sdk = self._sdk
        port_handler = sdk.PortHandler(self.port)
        try:
            opened = port_handler.openPort()
        except Exception as error:
            raise DeviceNotFound(
                f"cannot open Dynamixel port {self.port}: {error}", hint=_PORT_HINT, device=self._device
            ) from error
        if not opened:
            raise DeviceNotFound(f"cannot open Dynamixel port {self.port}", hint=_PORT_HINT, device=self._device)
        self._port_handler = port_handler
        self._packet_handler = sdk.PacketHandler(PROTOCOL_VERSION)
        try:
            self._set_baudrate()
            self._ping_all()
            self.disable_torque(strict=True)
            self._sync_read = sdk.GroupSyncRead(
                port_handler, self._packet_handler, ADDR_PRESENT_VELOCITY, SYNC_READ_LENGTH
            )
            for servo_id in self.servo_ids:
                if not self._sync_read.addParam(servo_id):
                    raise DeviceFault(
                        f"cannot add Dynamixel id {servo_id} to the sync read on {self.port}",
                        hint="servo ids must be unique",
                        device=self._device,
                    )
        except BaseException:
            self._close_port()
            raise

    def broadcast_ping(self) -> dict[int, int]:
        """Open the port at ``baudrate``, broadcast-ping and close it again: servo id -> model
        number of every responder (``servo_ids`` is ignored). Read-only: no torque write. Raises
        ``DeviceNotFound`` when the port cannot be opened or the baud rate is refused."""
        if self.is_open:
            raise RuntimeError(f"broadcast_ping needs a closed bus; {self.port} is open")
        sdk = self._sdk
        port_handler = sdk.PortHandler(self.port)
        try:
            opened = port_handler.openPort()
        except Exception as error:
            raise DeviceNotFound(
                f"cannot open Dynamixel port {self.port}: {error}", hint=_PORT_HINT, device=self._device
            ) from error
        if not opened:
            raise DeviceNotFound(f"cannot open Dynamixel port {self.port}", hint=_PORT_HINT, device=self._device)
        self._port_handler = port_handler
        self._packet_handler = sdk.PacketHandler(PROTOCOL_VERSION)
        try:
            self._set_baudrate()
            # Responders are kept even when the tail of the reply is corrupt or times out.
            responders, _comm_result = self._packet_handler.broadcastPing(port_handler)
        finally:
            self._close_port()
        return {int(servo_id): int(info[0]) for servo_id, info in dict(responders or {}).items()}

    def _set_baudrate(self) -> None:
        try:
            accepted = self._port_handler.setBaudRate(self.baudrate)
        except Exception as error:
            accepted, detail = False, f": {error}"
        else:
            detail = ""
        if not accepted:
            raise DeviceNotFound(
                f"cannot set baud rate {self.baudrate} on {self.port}{detail}",
                hint="use a baud rate the adapter supports (57600 factory default, 1000000 after a "
                "one-time change in Dynamixel Wizard); " + _PORT_HINT,
                device=self._device,
            )

    def _ping_all(self) -> None:
        missing = []
        for servo_id in self.servo_ids:
            _model, comm_result, _error = self._packet_handler.ping(self._port_handler, servo_id)
            if comm_result != self._sdk.COMM_SUCCESS:
                missing.append(servo_id)
        if missing:
            raise DeviceNotFound(
                f"Dynamixel ids {missing} on {self.port} do not answer a ping at {self.baudrate} baud "
                f"(expected {self.servo_ids})",
                hint="check the servo power supply and daisy chain, the configured ids and the baud "
                "rate (Dynamixel Wizard scans both)",
                device=self._device,
            )

    def disable_torque(self, *, strict: bool) -> None:
        """Write torque_enable = 0 to every servo; ``strict`` raises on the first failure,
        otherwise failures are ignored (used on close)."""
        with self._lock:
            for servo_id in self.servo_ids:
                try:
                    comm_result, _error = self._packet_handler.write1ByteTxRx(
                        self._port_handler, servo_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLED
                    )
                except Exception:
                    if strict:
                        raise
                    continue
                if strict and comm_result != self._sdk.COMM_SUCCESS:
                    raise DeviceFault(
                        f"cannot turn torque off on Dynamixel id {servo_id} on {self.port}: "
                        f"{self._comm_text(comm_result)}",
                        hint="check the servo power supply and cabling, then reconnect",
                        device=self._device,
                    )

    def read_positions_rad(self) -> np.ndarray:
        """One sync read; raw multi-turn angles in rad, ordered like ``servo_ids``. Raises
        ``DeviceFault`` on a communication failure or a servo missing from the reply."""
        with self._lock:
            comm_result = self._sync_read.txRxPacket()
            if comm_result != self._sdk.COMM_SUCCESS:
                raise DeviceFault(
                    f"Dynamixel sync read on {self.port} failed: {self._comm_text(comm_result)}",
                    hint="check the servo power supply and cabling",
                    device=self._device,
                )
            missing = [
                servo_id
                for servo_id in self.servo_ids
                if not self._sync_read.isAvailable(servo_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
            ]
            if missing:
                raise DeviceFault(
                    f"Dynamixel ids {missing} missing from the sync read on {self.port}",
                    hint="check the servo power supply and cabling",
                    device=self._device,
                )
            ticks = [
                signed_int32(self._sync_read.getData(servo_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION))
                for servo_id in self.servo_ids
            ]
        return ticks_to_rad(ticks)

    def close(self) -> None:
        """Best-effort torque off, then release the port."""
        if self._port_handler is None:
            return
        try:
            self.disable_torque(strict=False)
        finally:
            self._close_port()

    def _close_port(self) -> None:
        port_handler, self._port_handler = self._port_handler, None
        self._sync_read = None
        if port_handler is not None:
            port_handler.closePort()

    def _comm_text(self, comm_result: int) -> str:
        describe = getattr(self._packet_handler, "getTxRxResult", None)
        return str(describe(comm_result)) if callable(describe) else f"comm result {comm_result}"


# -- teleop --------------------------------------------------------------------


@dataclass
class _GelloHandle:
    bus: DynamixelBus
    event_device: Any = None
    event_key_codes: dict[int, str] = field(default_factory=dict)
    event_type_key: int | None = None


@Teleop.register("gello")
class GelloTeleop(Teleop):
    """Options (``TeleopConfig.options``): ``port`` (required, ``/dev/serial/by-id/...``),
    ``baudrate`` (57600, never written to EEPROM), ``joint_ids``, ``joint_offsets_rad``,
    ``joint_signs`` (required, one per joint), ``wrap_centers_rad`` (zeros), ``gripper_id``
    (None = no gripper channel), ``gripper_open_rad``/``gripper_closed_rad`` (required with
    ``gripper_id``), ``gripper_binarize_threshold`` (None), ``intervention`` (``always`` |
    ``toggle``), ``event_device`` (an evdev path; None = no events and no toggle input),
    ``event_keys`` (action -> evdev key name, merged over the defaults; None unbinds an action),
    ``poll_interval_s`` (0.0, sleep before each bus transaction)."""

    SDK = "dynamixel_sdk"
    ACTION_SPACE = "joint_abs"

    @classmethod
    def require_sdk(cls) -> ModuleType:
        try:
            return super().require_sdk()
        except SdkMissing as error:
            raise SdkMissing(error.message, hint=_SDK_HINT, device=error.device) from None

    def __init__(self, config: TeleopConfig, **kwargs: Any) -> None:
        options = dict(config.options)
        for key in options:
            if key not in _OPTION_KEYS:
                raise ValueError(f"unknown gello option '{key}'; known: {sorted(_OPTION_KEYS)}")
        for key in _REQUIRED_OPTIONS:
            if options.get(key) is None:
                raise ValueError(f"gello option '{key}' is required; known: {sorted(_OPTION_KEYS)}")
        joint_ids = _servo_ids("joint_ids", options["joint_ids"])
        if not joint_ids:
            raise ValueError("gello option 'joint_ids' must list at least one servo id")
        gripper_id = options.get("gripper_id")
        # Instance attributes shadow the ClassVars: the shape follows the configured servos.
        self.NUM_JOINTS = len(joint_ids)
        self.HAS_GRIPPER = gripper_id is not None
        super().__init__(config, **kwargs)
        self._port = str(options["port"])
        self._baudrate = int(options.get("baudrate", 57600))
        if self._baudrate <= 0:
            raise ValueError(f"gello baudrate must be positive, got {self._baudrate}")
        self._joint_ids = joint_ids
        num_joints = len(joint_ids)
        self._joint_offsets_rad = _per_joint("joint_offsets_rad", options["joint_offsets_rad"], num_joints)
        self._joint_signs = _per_joint("joint_signs", options["joint_signs"], num_joints)
        if any(sign not in (1.0, -1.0) for sign in self._joint_signs):
            raise ValueError(f"gello joint_signs entries must be 1 or -1, got {list(self._joint_signs)}")
        self._wrap_centers_rad = _per_joint(
            "wrap_centers_rad", options.get("wrap_centers_rad", [0.0] * num_joints), num_joints
        )
        self._gripper_id, self._gripper_open_rad, self._gripper_closed_rad = self._gripper_options(
            options, gripper_id
        )
        threshold = options.get("gripper_binarize_threshold")
        if threshold is not None:
            threshold = float(threshold)
            if not 0.0 <= threshold <= 1.0:
                raise ValueError(f"gripper_binarize_threshold must be in [0, 1], got {threshold}")
        self._gripper_binarize_threshold = threshold
        self._intervention = options.get("intervention", "always")
        if self._intervention not in INTERVENTION_MODES:
            raise ValueError(
                f"gello intervention '{self._intervention}' invalid; choose from {list(INTERVENTION_MODES)}"
            )
        event_device = options.get("event_device")
        self._event_device_path = None if event_device is None else str(event_device)
        self._event_keys = _event_key_names(options.get("event_keys"))
        if self._intervention == "toggle" and (
            self._event_device_path is None or TOGGLE_ACTION not in self._event_keys
        ):
            raise ValueError(
                "gello intervention 'toggle' needs event_device and an event_keys.toggle binding "
                "(the toggle key is read from that input device)"
            )
        self._poll_interval_s = float(options.get("poll_interval_s", 0.0))
        if self._poll_interval_s < 0:
            raise ValueError(f"poll_interval_s must be >= 0, got {self._poll_interval_s}")
        self._state_lock = threading.Lock()
        self._toggle_on = False

    def _gripper_options(
        self, options: Mapping[str, Any], gripper_id: Any
    ) -> tuple[int | None, float | None, float | None]:
        if gripper_id is None:
            gripper_only = (*_GRIPPER_RANGE_OPTIONS, "gripper_binarize_threshold")
            given = [key for key in gripper_only if options.get(key) is not None]
            if given:
                raise ValueError(f"gello options {given} need gripper_id")
            return None, None, None
        (servo_id,) = _servo_ids("gripper_id", [gripper_id])
        if servo_id in self._joint_ids:
            raise ValueError(f"gello gripper_id {servo_id} is also in joint_ids {self._joint_ids}")
        missing = [key for key in _GRIPPER_RANGE_OPTIONS if options.get(key) is None]
        if missing:
            raise ValueError(f"gello options {missing} are required with gripper_id")
        open_rad, closed_rad = float(options["gripper_open_rad"]), float(options["gripper_closed_rad"])
        if open_rad == closed_rad:
            raise ValueError(f"gripper_open_rad and gripper_closed_rad must differ, both are {open_rad}")
        return servo_id, open_rad, closed_rad

    @property
    def servo_ids(self) -> list[int]:
        return self._joint_ids + ([] if self._gripper_id is None else [self._gripper_id])

    @classmethod
    def discover(cls) -> set[str]:
        """Stable serial paths of U2D2 (FTDI) and OpenRB-150 (ROBOTIS) adapters."""
        found: set[str] = set()
        for pattern in SERIAL_BY_ID_GLOBS:
            found.update(glob.glob(pattern))
        return found

    # -- device ----------------------------------------------------------------

    def _open_device(self) -> _GelloHandle:
        sdk = self.require_sdk()
        evdev = self._require_evdev() if self._event_device_path is not None else None
        bus = DynamixelBus(sdk, self._port, self._baudrate, self.servo_ids, device=self.name)
        bus.open()
        handle = _GelloHandle(bus=bus)
        try:
            if evdev is not None:
                self._open_event_device(evdev, handle)
        except BaseException:
            bus.close()
            raise
        with self._state_lock:
            self._toggle_on = False
        return handle

    def _require_evdev(self) -> ModuleType:
        try:
            return importlib.import_module(EVDEV_SDK)
        except ImportError:
            raise SdkMissing(
                f"{type(self).__name__} with event_device needs the '{EVDEV_SDK}' package",
                hint=_EVDEV_HINT,
                device=self.name,
            ) from None

    def _open_event_device(self, evdev: ModuleType, handle: _GelloHandle) -> None:
        codes: dict[int, str] = {}
        for action, key_name in self._event_keys.items():
            code = getattr(evdev.ecodes, key_name, None)
            if not isinstance(code, int):
                raise ValueError(f"gello event_keys.{action}: unknown evdev key name '{key_name}'")
            if code in codes:
                raise ValueError(
                    f"gello event_keys: '{key_name}' is bound to both {codes[code]} and {action}"
                )
            codes[code] = action
        try:
            device = evdev.InputDevice(self._event_device_path)
        except OSError as error:
            raise DeviceNotFound(
                f"cannot open input device {self._event_device_path}: {error}",
                hint="use a /dev/input/by-id/... path; the user needs the 'input' group "
                "(`sudo usermod -aG input $USER`, then log in again); inside Docker pass the node "
                "with `docker run --device=<path>`",
                device=self.name,
            ) from error
        handle.event_device = device
        handle.event_key_codes = codes
        handle.event_type_key = evdev.ecodes.EV_KEY

    def _close_device(self, handle: _GelloHandle) -> None:
        try:
            if handle.event_device is not None:
                try:
                    handle.event_device.close()
                except Exception:
                    pass
        finally:
            handle.bus.close()

    def _reset_device(self, handle: _GelloHandle) -> None:
        with self._state_lock:
            self._toggle_on = False

    def _poll_device(self, handle: _GelloHandle) -> RawReading:
        if self._poll_interval_s > 0:
            _sleep(self._poll_interval_s)
        raw_rad = handle.bus.read_positions_rad()
        num_joints = len(self._joint_ids)
        joints = joint_positions_from_raw(
            raw_rad[:num_joints], self._joint_offsets_rad, self._joint_signs, self._wrap_centers_rad
        )
        gripper = None
        if self._gripper_id is not None:
            gripper = gripper_width_from_raw(
                float(raw_rad[num_joints]),
                self._gripper_open_rad,
                self._gripper_closed_rad,
                self._gripper_binarize_threshold,
            )
        pressed = self._drain_key_downs(handle)
        with self._state_lock:
            if pressed.count(TOGGLE_ACTION) % 2:
                self._toggle_on = not self._toggle_on
            toggle_on = self._toggle_on
        return RawReading(
            action=joints.astype(np.float32),
            gripper=gripper,
            buttons={TOGGLE_ACTION: toggle_on},
            events_now={key: key in pressed for key in EVENT_KEYS},
            active=True if self._intervention == "always" else toggle_on,
        )

    def _drain_key_downs(self, handle: _GelloHandle) -> list[str]:
        """Actions whose key went down since the last poll (non-blocking)."""
        device = handle.event_device
        if device is None:
            return []
        pressed = []
        while True:
            event = device.read_one()
            if event is None:
                return pressed
            if event.type == handle.event_type_key and event.value == EVDEV_KEY_DOWN:
                action = handle.event_key_codes.get(event.code)
                if action is not None:
                    pressed.append(action)


def _servo_ids(option: str, values: Any) -> list[int]:
    ids = []
    for value in list(values):
        if isinstance(value, bool) or not isinstance(value, (int, np.integer)):
            raise ValueError(f"gello {option} entries must be integers, got {value!r}")
        if not 0 <= int(value) <= MAX_DYNAMIXEL_ID:
            raise ValueError(f"gello {option} entries must be in [0, {MAX_DYNAMIXEL_ID}], got {value}")
        ids.append(int(value))
    if len(set(ids)) != len(ids):
        raise ValueError(f"gello {option} has duplicate ids: {ids}")
    return ids


def _per_joint(option: str, values: Any, num_joints: int) -> np.ndarray:
    array = np.asarray(values, dtype=np.float64)
    if array.shape != (num_joints,):
        raise ValueError(f"gello {option} needs {num_joints} entries (one per joint id), got {values!r}")
    return array


def _event_key_names(overrides: Mapping[str, Any] | None) -> dict[str, str]:
    names = dict(DEFAULT_EVENT_KEYS)
    for action, key_name in dict(overrides or {}).items():
        if action not in _KEY_ACTIONS:
            raise ValueError(f"unknown gello event_keys action '{action}'; known: {list(_KEY_ACTIONS)}")
        if key_name is None:
            names.pop(action, None)
        else:
            names[action] = str(key_name)
    return names
