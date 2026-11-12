"""3Dconnexion SpaceMouse through the in-tree hidapi reader: the puck drives a cartesian twist; every
button press is latched as an event named after the button (``button_0``, ``button_1``), so a
press shorter than the consumer's read period is not lost. Neither button ends the episode.

Button 0 also toggles a sticky gripper target here (``gripper``), and ``active`` is the puck or a
held button; both are kept for the robot-yaml route (a ``Teleop`` part read by the robot).
``real_world_rl``'s ``SpaceMouseTeleopSource`` reads neither: it gives the buttons their own
meaning (button 0 take-over / hand-back, button 1 gripper)."""
from __future__ import annotations

import glob
import threading
import time
from types import ModuleType
from typing import Any

import numpy as np

from real_world_rl.devices.backends.teleops import _spacemouse_hid
from real_world_rl.devices.errors import DeviceError
from real_world_rl.devices.teleop import RawReading, Teleop, TeleopConfig

HIDRAW_GLOB = "/dev/hidraw*"
GRIPPER_BUTTON = 0
_OPEN_GRIPPER, _CLOSED_GRIPPER = 1.0, 0.0
_NUM_BUTTONS = 2
# Each button's ``buttons`` level key and latched press event, by index.
BUTTON_NAMES = tuple(f"button_{index}" for index in range(_NUM_BUTTONS))
_OPTION_KEYS = {"device_index", "deadzone", "invert_axes", "poll_interval_s", _spacemouse_hid.DEVICE_TABLE_OPTION}
_sleep = time.sleep  # module-level so tests can observe the poll cadence


def map_spacemouse_axes(decoded, invert_axes=()):
    """Map decoded x/y/z/roll/pitch/yaw into the shared HIL-SERL input axes."""
    raw = np.asarray(decoded, dtype=np.float32)
    if raw.shape != (6,) or not np.isfinite(raw).all():
        raise ValueError("decoded SpaceMouse axes must be six finite numbers")
    axes = list(invert_axes)
    if any(type(axis) is not int or axis not in range(6) for axis in axes):
        raise ValueError("invert_axes must contain integer axes 0..5")
    result = raw[[1, 0, 2, 3, 4, 5]] * np.array([-1, 1, 1, -1, -1, -1], dtype=np.float32)
    result[axes] *= -1
    return result


@Teleop.register("spacemouse")
class SpaceMouseTeleop(Teleop):
    """Options: ``device_index`` (default 0), ``deadzone`` (default 1e-3, twist norm below which
    the puck counts as idle), ``invert_axes`` (indices 0-5 of the output twist whose sign is
    flipped), ``poll_interval_s`` (default 0.005 = 200 Hz; device poll cadence so a non-blocking
    ``read()`` does not spin a core, may be 0 once ``read()`` is confirmed to block),
    ``device_table`` (a local device table that extends the built-in one; without it the
    site's ``spacemouse-devices.yaml``, see ``_spacemouse_hid.load_device_specs``; loaded and
    validated here, at construction).
    The twist uses fixed device input axes ``[-y, x, z, -roll, -pitch, -yaw]``
    from the HID state, as in HIL-SERL. Mounting and operator orientation are
    applied by the higher-level teleop source, not this device reader."""

    SDK = "hid"
    ACTION_SPACE = "cartesian_delta"
    HAS_GRIPPER = True

    @classmethod
    def require_sdk(cls) -> ModuleType:
        return _spacemouse_hid.require_sdk()

    def __init__(self, config: TeleopConfig, *, claim: bool = True, **kwargs: Any) -> None:
        super().__init__(config, **kwargs)
        self._claim = claim
        options = dict(config.options)
        for key in options:
            if key not in _OPTION_KEYS:
                raise ValueError(f"unknown spacemouse option '{key}'; known: {sorted(_OPTION_KEYS)}")
        self._device_index = int(options.get("device_index", 0))
        self._deadzone = float(options.get("deadzone", 1e-3))
        if self._deadzone < 0:
            raise ValueError(f"deadzone must be >= 0, got {self._deadzone}")
        self._poll_interval_s = float(options.get("poll_interval_s", 0.005))
        if self._poll_interval_s < 0:
            raise ValueError(f"poll_interval_s must be >= 0, got {self._poll_interval_s}")
        invert_axes = [int(axis) for axis in options.get("invert_axes", [])]
        for axis in invert_axes:
            if not 0 <= axis < self.action_dim:
                raise ValueError(f"invert_axes entries must be in [0, {self.action_dim}), got {axis}")
        self._invert_axes = invert_axes
        self._device_table = _spacemouse_hid.load_device_specs(options=options)
        self._state_lock = threading.Lock()
        self._gripper = _OPEN_GRIPPER
        self._previous_buttons = [False] * _NUM_BUTTONS

    @property
    def device_identity(self) -> dict:
        """Identity of the currently open HID device; no SDK access or polling."""
        self._require_connected()
        return dict(getattr(self._live_device, "identity", {}))

    @property
    def device_spec(self) -> dict:
        """The device-table entry decoding the open device (name, hid_id, source builtin|local
        and, for a local entry, the table's sha256); no SDK access or polling."""
        self._require_connected()
        return dict(getattr(self._live_device, "device_spec", {}))

    @classmethod
    def discover(cls, table: _spacemouse_hid.DeviceTable | None = None) -> set[str]:
        """Device ids: ``vid:pid:path`` of every HID interface whose exact vendor and product id
        is in ``table`` (default: ``load_device_specs()``), via hidapi when installed; else every
        ``/dev/hidraw*`` (which also lists non-SpaceMouse HID devices). Matching the vendor alone
        is wrong: older 3Dconnexion units enumerate under Logitech 0x046d, which also makes
        keyboard and mouse receivers."""
        try:
            import hid
        except ImportError:
            return set(glob.glob(HIDRAW_GLOB))
        table = _spacemouse_hid.load_device_specs() if table is None else table
        found = set()
        for info in hid.enumerate():
            if (info["vendor_id"], info["product_id"]) in table.specs:
                path = info["path"]
                path = path.decode(errors="replace") if isinstance(path, bytes) else str(path)
                found.add(f"{info['vendor_id']:04x}:{info['product_id']:04x}:{path}")
        return found

    # -- device ----------------------------------------------------------------

    def _open_device(self) -> Any:
        try:
            device = _spacemouse_hid.open(device_index=self._device_index, table=self._device_table,
                                          **({"claim": False} if not self._claim else {}))
        except DeviceError as error:
            error.device = self.name
            raise
        with self._state_lock:
            self._reset_state()
        return device

    def _close_device(self, handle: Any) -> None:
        # The poll thread is already stopped. The HID handle must be released, otherwise the
        # next open() cannot get it.
        close = getattr(handle, "close", None)
        if callable(close):
            close()

    def _reset_state(self) -> None:
        self._gripper = _OPEN_GRIPPER
        self._previous_buttons = [False] * _NUM_BUTTONS

    def _reset_device(self, handle: Any) -> None:
        with self._state_lock:
            self._reset_state()

    def _poll_device(self, handle: Any) -> RawReading:
        _sleep(self._poll_interval_s)
        state = handle.read()
        twist = map_spacemouse_axes(
            [state.x, state.y, state.z, state.roll, state.pitch, state.yaw], self._invert_axes)
        pressed = [bool(b) for b in list(state.buttons)[:_NUM_BUTTONS]]
        pressed += [False] * (_NUM_BUTTONS - len(pressed))
        with self._state_lock:
            rising = [now and not before for now, before in zip(pressed, self._previous_buttons)]
            self._previous_buttons = pressed
            if rising[GRIPPER_BUTTON]:
                self._gripper = _CLOSED_GRIPPER if self._gripper == _OPEN_GRIPPER else _OPEN_GRIPPER
            gripper = self._gripper
        return RawReading(
            action=twist,
            gripper=gripper,
            buttons=dict(zip(BUTTON_NAMES, pressed)),
            events_now=dict(zip(BUTTON_NAMES, rising)),
            active=bool(np.linalg.norm(twist) > self._deadzone or any(pressed)),
            diagnostics={"decoded_axes": [state.x, state.y, state.z, state.roll, state.pitch, state.yaw],
                         "decoded_axis_order": ["x", "y", "z", "roll", "pitch", "yaw"],
                         "hid_report_count": getattr(handle, "report_count", None),
                         "hid_last_report_monotonic_s": getattr(handle, "last_report_monotonic", None)},
        )
