"""Nonblocking SpaceMouse HID reader using trezor/cython-hidapi (distribution ``hidapi``).

Device table and decoder derived from PySpaceMouse 1.0.3, the HIL-SERL
vendored version, plus one entry added in this repository: SpaceMouse Wireless BT
(0x256F:0xC63A), the Bluetooth edition cabled over USB (report 1: six signed 16-bit
axes; report 3: two button bits), checked against its HID descriptor on 2026-11-05.
Derived from PySpaceMouse by John Harrison et al., MIT.
No separate PySpaceMouse license was present in the reference tree.
The HIL-SERL LICENSE carries this notice (copied verbatim):

   Copyright [2024] [Jianlan Luo, Charles Xu, Jeffrey Wu]

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.

Modified for robot_infra: replace easyhid with hidapi, omit callbacks, add optional advisory device claims,
guard short reports, and close failed opens. Apart from the added entry, the device table is
unchanged from 1.0.3.
"""
from __future__ import annotations

import importlib
from dataclasses import dataclass
from types import ModuleType
from typing import Any, NamedTuple, Sequence

from robot_infra.devices.errors import DeviceNotFound, SdkMissing, VersionMismatch

HIDAPI_REQUIREMENT = "hidapi==0.14.0.post4"
SDK_HINT = "pip install -e '.[spacemouse]'"
UDEV_HINT = (
    "on the NUC, repository root (no venv required): "
    "sudo cp robot_infra/robot_infra/setup/99-spacemouse.rules /etc/udev/rules.d/; "
    'sudo usermod -aG plugdev "$USER"; '
    "sudo udevadm control --reload-rules && sudo udevadm trigger --subsystem-match=hidraw; "
    "log in again and replug the SpaceMouse"
)


class AxisSpec(NamedTuple):
    channel: int
    byte1: int
    byte2: int
    scale: int


class ButtonSpec(NamedTuple):
    channel: int | None
    byte: int | None
    bit: int | None


@dataclass(frozen=True)
class DeviceSpec:
    name: str
    hid_id: list[int]
    led_id: list[int] | None
    mappings: dict[str, AxisSpec]
    button_mapping: list[ButtonSpec]
    axis_scale: float = 350.0


class State(NamedTuple):
    x: float
    y: float
    z: float
    roll: float
    pitch: float
    yaw: float
    buttons: list[int]


def to_int16(low: int, high: int) -> int:
    value = low | (high << 8)
    return value - 65536 if value >= 32768 else value


device_specs = {
    "SpaceExplorer": DeviceSpec(
        name="SpaceExplorer",
        # vendor ID and product ID
        hid_id=[0x46D, 0xc627],
        # LED HID usage code pair
        led_id=[0x8, 0x4B],
        mappings={
            "x": AxisSpec(channel=1, byte1=1, byte2=2, scale=1),
            "y": AxisSpec(channel=1, byte1=3, byte2=4, scale=-1),
            "z": AxisSpec(channel=1, byte1=5, byte2=6, scale=-1),
            "pitch": AxisSpec(channel=2, byte1=1, byte2=2, scale=-1),
            "roll": AxisSpec(channel=2, byte1=3, byte2=4, scale=-1),
            "yaw": AxisSpec(channel=2, byte1=5, byte2=6, scale=1),
        },
        button_mapping=[
            ButtonSpec(channel=3, byte=2, bit=0),  # SHIFT
            ButtonSpec(channel=3, byte=1, bit=6),  # ESC
            ButtonSpec(channel=3, byte=2, bit=1),  # CTRL
            ButtonSpec(channel=3, byte=1, bit=7),  # ALT
            ButtonSpec(channel=3, byte=1, bit=0),  # 1
            ButtonSpec(channel=3, byte=1, bit=1),  # 2
            ButtonSpec(channel=3, byte=2, bit=3),  # PANEL
            ButtonSpec(channel=3, byte=2, bit=2),  # FIT
            ButtonSpec(channel=3, byte=2, bit=5),  # -
            ButtonSpec(channel=3, byte=2, bit=4),  # +
            ButtonSpec(channel=3, byte=1, bit=2),  # T
            ButtonSpec(channel=3, byte=1, bit=3),  # L
            ButtonSpec(channel=3, byte=1, bit=5),  # F
            ButtonSpec(channel=3, byte=2, bit=6),  # 2D
            ButtonSpec(channel=3, byte=1, bit=4),  # R
        ],
        axis_scale=350.0,
    ),
    "SpaceNavigator": DeviceSpec(
        name="SpaceNavigator",
        # vendor ID and product ID
        hid_id=[0x46D, 0xC626],
        # LED HID usage code pair
        led_id=[0x8, 0x4B],
        mappings={
            "x": AxisSpec(channel=1, byte1=1, byte2=2, scale=1),
            "y": AxisSpec(channel=1, byte1=3, byte2=4, scale=-1),
            "z": AxisSpec(channel=1, byte1=5, byte2=6, scale=-1),
            "pitch": AxisSpec(channel=2, byte1=1, byte2=2, scale=-1),
            "roll": AxisSpec(channel=2, byte1=3, byte2=4, scale=-1),
            "yaw": AxisSpec(channel=2, byte1=5, byte2=6, scale=1),
        },
        button_mapping=[
            ButtonSpec(channel=3, byte=1, bit=0),  # LEFT
            ButtonSpec(channel=3, byte=1, bit=1),  # RIGHT
        ],
        axis_scale=350.0,
    ),
    "SpaceMouse USB": DeviceSpec(
        name="SpaceMouseUSB",
        # vendor ID and product ID
        hid_id=[0x256f, 0xc641],
        # LED HID usage code pair
        led_id=None,
        mappings={
            "x": AxisSpec(channel=1, byte1=1, byte2=2, scale=1),
            "y": AxisSpec(channel=1, byte1=3, byte2=4, scale=-1),
            "z": AxisSpec(channel=1, byte1=5, byte2=6, scale=-1),
            "pitch": AxisSpec(channel=2, byte1=1, byte2=2, scale=-1),
            "roll": AxisSpec(channel=2, byte1=3, byte2=4, scale=-1),
            "yaw": AxisSpec(channel=2, byte1=5, byte2=6, scale=1),
        },
        button_mapping=[
            ButtonSpec(channel=None, byte=None, bit=None),  # No buttons
            ],
        axis_scale=350.0,
    ),
    "SpaceMouse Compact": DeviceSpec(
        name="SpaceMouse Compact",
        # vendor ID and product ID
        hid_id=[0x256F, 0xC635],
        # LED HID usage code pair
        led_id=[0x8, 0x4B],
        mappings={
            "x": AxisSpec(channel=1, byte1=1, byte2=2, scale=1),
            "y": AxisSpec(channel=1, byte1=3, byte2=4, scale=-1),
            "z": AxisSpec(channel=1, byte1=5, byte2=6, scale=-1),
            "pitch": AxisSpec(channel=2, byte1=1, byte2=2, scale=-1),
            "roll": AxisSpec(channel=2, byte1=3, byte2=4, scale=-1),
            "yaw": AxisSpec(channel=2, byte1=5, byte2=6, scale=1),
        },
        button_mapping=[
            ButtonSpec(channel=3, byte=1, bit=0),  # LEFT
            ButtonSpec(channel=3, byte=1, bit=1),  # RIGHT
        ],
        axis_scale=350.0,
    ),
    "SpaceMouse Pro Wireless": DeviceSpec(
        name="SpaceMouse Pro Wireless",
        # vendor ID and product ID
        hid_id=[0x256F, 0xC632],
        # LED HID usage code pair
        led_id=[0x8, 0x4B],
        mappings={
            "x": AxisSpec(channel=1, byte1=1, byte2=2, scale=1),
            "y": AxisSpec(channel=1, byte1=3, byte2=4, scale=-1),
            "z": AxisSpec(channel=1, byte1=5, byte2=6, scale=-1),
            "pitch": AxisSpec(channel=1, byte1=7, byte2=8, scale=-1),
            "roll": AxisSpec(channel=1, byte1=9, byte2=10, scale=-1),
            "yaw": AxisSpec(channel=1, byte1=11, byte2=12, scale=1),
        },
        button_mapping=[
            ButtonSpec(channel=3, byte=1, bit=0),  # MENU
            ButtonSpec(channel=3, byte=3, bit=7),  # ALT
            ButtonSpec(channel=3, byte=4, bit=1),  # CTRL
            ButtonSpec(channel=3, byte=4, bit=0),  # SHIFT
            ButtonSpec(channel=3, byte=3, bit=6),  # ESC
            ButtonSpec(channel=3, byte=2, bit=4),  # 1
            ButtonSpec(channel=3, byte=2, bit=5),  # 2
            ButtonSpec(channel=3, byte=2, bit=6),  # 3
            ButtonSpec(channel=3, byte=2, bit=7),  # 4
            ButtonSpec(channel=3, byte=2, bit=0),  # ROLL CLOCKWISE
            ButtonSpec(channel=3, byte=1, bit=2),  # TOP
            ButtonSpec(channel=3, byte=4, bit=2),  # ROTATION
            ButtonSpec(channel=3, byte=1, bit=5),  # FRONT
            ButtonSpec(channel=3, byte=1, bit=4),  # REAR
            ButtonSpec(channel=3, byte=1, bit=1),  # FIT
        ],
        axis_scale=350.0,
    ),
    "SpaceMouse Pro": DeviceSpec(
        name="SpaceMouse Pro",
        # vendor ID and product ID
        hid_id=[0x46D, 0xC62b],
        led_id=[0x8, 0x4B],
        mappings={
            "x": AxisSpec(channel=1, byte1=1, byte2=2, scale=1),
            "y": AxisSpec(channel=1, byte1=3, byte2=4, scale=-1),
            "z": AxisSpec(channel=1, byte1=5, byte2=6, scale=-1),
            "pitch": AxisSpec(channel=2, byte1=1, byte2=2, scale=-1),
            "roll": AxisSpec(channel=2, byte1=3, byte2=4, scale=-1),
            "yaw": AxisSpec(channel=2, byte1=5, byte2=6, scale=1),
        },
        button_mapping=[
            ButtonSpec(channel=3, byte=1, bit=0),  # MENU
            ButtonSpec(channel=3, byte=3, bit=7),  # ALT
            ButtonSpec(channel=3, byte=4, bit=1),  # CTRL
            ButtonSpec(channel=3, byte=4, bit=0),  # SHIFT
            ButtonSpec(channel=3, byte=3, bit=6),  # ESC
            ButtonSpec(channel=3, byte=2, bit=4),  # 1
            ButtonSpec(channel=3, byte=2, bit=5),  # 2
            ButtonSpec(channel=3, byte=2, bit=6),  # 3
            ButtonSpec(channel=3, byte=2, bit=7),  # 4
            ButtonSpec(channel=3, byte=2, bit=0),  # ROLL CLOCKWISE
            ButtonSpec(channel=3, byte=1, bit=2),  # TOP
            ButtonSpec(channel=3, byte=4, bit=2),  # ROTATION
            ButtonSpec(channel=3, byte=1, bit=5),  # FRONT
            ButtonSpec(channel=3, byte=1, bit=4),  # REAR
            ButtonSpec(channel=3, byte=1, bit=1),  # FIT
        ],
        axis_scale=350.0,
    ),
    "SpaceMouse Wireless": DeviceSpec(
        name="SpaceMouse Wireless",
        # vendor ID and product ID
        hid_id=[0x256F, 0xC62E],
        # LED HID usage code pair
        led_id=[0x8, 0x4B],
        mappings={
            "x": AxisSpec(channel=1, byte1=1, byte2=2, scale=1),
            "y": AxisSpec(channel=1, byte1=3, byte2=4, scale=-1),
            "z": AxisSpec(channel=1, byte1=5, byte2=6, scale=-1),
            "pitch": AxisSpec(channel=1, byte1=7, byte2=8, scale=-1),
            "roll": AxisSpec(channel=1, byte1=9, byte2=10, scale=-1),
            "yaw": AxisSpec(channel=1, byte1=11, byte2=12, scale=1),
        },
        button_mapping=[
            ButtonSpec(channel=3, byte=1, bit=0),  # LEFT
            ButtonSpec(channel=3, byte=1, bit=1),  # RIGHT
        ],  # FIT
        axis_scale=350.0,
    ),
    # USB-cabled BT edition: report 1 has six signed 16-bit axes;
    # report 3 has two button bits (verified against its HID descriptor).
    "SpaceMouse Wireless BT": DeviceSpec(
        name="SpaceMouse Wireless BT",
        hid_id=[0x256F, 0xC63A],
        led_id=[0x8, 0x4B],
        mappings={
            "x": AxisSpec(channel=1, byte1=1, byte2=2, scale=1),
            "y": AxisSpec(channel=1, byte1=3, byte2=4, scale=-1),
            "z": AxisSpec(channel=1, byte1=5, byte2=6, scale=-1),
            "pitch": AxisSpec(channel=1, byte1=7, byte2=8, scale=-1),
            "roll": AxisSpec(channel=1, byte1=9, byte2=10, scale=-1),
            "yaw": AxisSpec(channel=1, byte1=11, byte2=12, scale=1),
        },
        button_mapping=[
            ButtonSpec(channel=3, byte=1, bit=0),
            ButtonSpec(channel=3, byte=1, bit=1),
        ],
        axis_scale=350.0,
    ),
    "3Dconnexion Universal Receiver": DeviceSpec(
        name="3Dconnexion Universal Receiver",
        # vendor ID and product ID
        hid_id=[0x256F, 0xC652],
        # LED HID usage code pair
        led_id=[0x8, 0x4B],
        mappings={
            "x": AxisSpec(channel=1, byte1=1, byte2=2, scale=1),
            "y": AxisSpec(channel=1, byte1=3, byte2=4, scale=-1),
            "z": AxisSpec(channel=1, byte1=5, byte2=6, scale=-1),
            "pitch": AxisSpec(channel=1, byte1=7, byte2=8, scale=-1),
            "roll": AxisSpec(channel=1, byte1=9, byte2=10, scale=-1),
            "yaw": AxisSpec(channel=1, byte1=11, byte2=12, scale=1),
        },
        button_mapping=[
            ButtonSpec(channel=3, byte=1, bit=0),  # MENU
            ButtonSpec(channel=3, byte=3, bit=7),  # ALT
            ButtonSpec(channel=3, byte=4, bit=1),  # CTRL
            ButtonSpec(channel=3, byte=4, bit=0),  # SHIFT
            ButtonSpec(channel=3, byte=3, bit=6),  # ESC
            ButtonSpec(channel=3, byte=2, bit=4),  # 1
            ButtonSpec(channel=3, byte=2, bit=5),  # 2
            ButtonSpec(channel=3, byte=2, bit=6),  # 3
            ButtonSpec(channel=3, byte=2, bit=7),  # 4
            ButtonSpec(channel=3, byte=2, bit=0),  # ROLL CLOCKWISE
            ButtonSpec(channel=3, byte=1, bit=2),  # TOP
            ButtonSpec(channel=3, byte=4, bit=2),  # ROTATION
            ButtonSpec(channel=3, byte=1, bit=5),  # FRONT
            ButtonSpec(channel=3, byte=1, bit=4),  # REAR
            ButtonSpec(channel=3, byte=1, bit=1),  # FIT
        ],
        axis_scale=350.0,
    ),
    "SpacePilot": DeviceSpec(
        name="SpacePilot",
        # vendor ID and product ID
        hid_id=[0x46D, 0xC625],
        # LED HID usage code pair
        led_id=None,
        mappings={
            "x": AxisSpec(channel=1, byte1=1, byte2=2, scale=1),
            "y": AxisSpec(channel=1, byte1=3, byte2=4, scale=-1),
            "z": AxisSpec(channel=1, byte1=5, byte2=6, scale=-1),
            "pitch": AxisSpec(channel=2, byte1=1, byte2=2, scale=-1),
            "roll": AxisSpec(channel=2, byte1=3, byte2=4, scale=-1),
            "yaw": AxisSpec(channel=2, byte1=5, byte2=6, scale=1),
        },
        button_mapping=[
            ButtonSpec(channel=3, byte=1, bit=0), # 1
            ButtonSpec(channel=3, byte=1, bit=1), # 2
            ButtonSpec(channel=3, byte=1, bit=2), # 3
            ButtonSpec(channel=3, byte=1, bit=3), # 4
            ButtonSpec(channel=3, byte=1, bit=4), # 5
            ButtonSpec(channel=3, byte=1, bit=5), # 6
            ButtonSpec(channel=3, byte=1, bit=6), # T
            ButtonSpec(channel=3, byte=1, bit=7), # L
            ButtonSpec(channel=3, byte=2, bit=0), # R
            ButtonSpec(channel=3, byte=2, bit=1), # F
            ButtonSpec(channel=3, byte=2, bit=2), # Esc
            ButtonSpec(channel=3, byte=2, bit=3), # Alt
            ButtonSpec(channel=3, byte=2, bit=4), # Shift
            ButtonSpec(channel=3, byte=2, bit=5), # Ctrl
            ButtonSpec(channel=3, byte=2, bit=6), # Fit
            ButtonSpec(channel=3, byte=2, bit=7), # Panel
            ButtonSpec(channel=3, byte=3, bit=0), # Zoom -
            ButtonSpec(channel=3, byte=3, bit=1), # Zoom +
            ButtonSpec(channel=3, byte=3, bit=2), # Dom
            ButtonSpec(channel=3, byte=3, bit=3), # 3D Lock
            ButtonSpec(channel=3, byte=3, bit=4), # Config
        ],
        axis_scale=350.0,
    ),
    "SpacePilot Pro": DeviceSpec(
        name="SpacePilot Pro",
        # vendor ID and product ID
        hid_id=[0x46D, 0xC629],
        # LED HID usage code pair
        led_id=[0x8, 0x4B],
        mappings={
            "x": AxisSpec(channel=1, byte1=1, byte2=2, scale=1),
            "y": AxisSpec(channel=1, byte1=3, byte2=4, scale=-1),
            "z": AxisSpec(channel=1, byte1=5, byte2=6, scale=-1),
            "pitch": AxisSpec(channel=2, byte1=1, byte2=2, scale=-1),
            "roll": AxisSpec(channel=2, byte1=3, byte2=4, scale=-1),
            "yaw": AxisSpec(channel=2, byte1=5, byte2=6, scale=1),
        },
        button_mapping=[
            ButtonSpec(channel=3, byte=4, bit=0),  # SHIFT
            ButtonSpec(channel=3, byte=3, bit=6),  # ESC
            ButtonSpec(channel=3, byte=4, bit=1),  # CTRL
            ButtonSpec(channel=3, byte=3, bit=7),  # ALT
            ButtonSpec(channel=3, byte=3, bit=1),  # 1
            ButtonSpec(channel=3, byte=3, bit=2),  # 2
            ButtonSpec(channel=3, byte=2, bit=6),  # 3
            ButtonSpec(channel=3, byte=2, bit=7),  # 4
            ButtonSpec(channel=3, byte=3, bit=0),  # 5
            ButtonSpec(channel=3, byte=1, bit=0),  # MENU
            ButtonSpec(channel=3, byte=4, bit=6),  # -
            ButtonSpec(channel=3, byte=4, bit=5),  # +
            ButtonSpec(channel=3, byte=4, bit=4),  # DOMINANT
            ButtonSpec(channel=3, byte=4, bit=3),  # PAN/ZOOM
            ButtonSpec(channel=3, byte=4, bit=2),  # ROTATION
            ButtonSpec(channel=3, byte=2, bit=0),  # ROLL CLOCKWISE
            ButtonSpec(channel=3, byte=1, bit=2),  # TOP
            ButtonSpec(channel=3, byte=1, bit=5),  # FRONT
            ButtonSpec(channel=3, byte=1, bit=4),  # REAR
            ButtonSpec(channel=3, byte=2, bit=2),  # ISO
            ButtonSpec(channel=3, byte=1, bit=1),  # FIT
        ],
        axis_scale=350.0,
    ),
}


def require_sdk() -> ModuleType:
    """Import only for an explicit SDK check or device open, never at module import."""
    try:
        sdk = importlib.import_module("hid")
    except ImportError as error:
        raise SdkMissing(
            f"SpaceMouse needs the '{HIDAPI_REQUIREMENT}' distribution (import hid): {error}",
            hint=SDK_HINT,
        ) from error
    except OSError as error:
        raise SdkMissing(
            f"cannot load hidapi (import hid): {error}",
            hint=f"{SDK_HINT}; check the hidapi wheel for this platform",
        ) from error
    if not callable(getattr(sdk, "device", None)):
        raise VersionMismatch(
            "wrong distribution 'hid' installed; SpaceMouse needs trezor hidapi with hid.device()",
            hint=f"remove the distribution 'hid', then {SDK_HINT} (distribution hidapi, import hid)",
        )
    return sdk


class _HidReader:
    def __init__(self, handle: Any, spec: DeviceSpec) -> None:
        self._handle = handle
        self._claim = None
        self.identity = {}
        self.report_count = 0
        self.last_report_monotonic = None
        self._spec = spec
        self._state = State(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, [0] * len(spec.button_mapping))
        self._report_size = 1 + max(
            [max(axis.byte1, axis.byte2) for axis in spec.mappings.values()]
            + [button.byte for button in spec.button_mapping if button.byte is not None]
        )

    def read(self) -> State:
        if self._handle is None:
            raise OSError("read from a closed SpaceMouse")
        # One nonblocking report per poll, as in the native reader. Split reports
        # update their axes only; an empty read must preserve the last state.
        report = self._handle.read(self._report_size)
        if report:
            import time
            self.report_count += 1
            self.last_report_monotonic = time.monotonic()
            self._process(report)
        return self._state

    def _process(self, report: Sequence[int]) -> None:
        updates: dict[str, Any] = {}
        for name, (channel, byte1, byte2, flip) in self._spec.mappings.items():
            if report[0] == channel and max(byte1, byte2) < len(report):
                updates[name] = flip * to_int16(report[byte1], report[byte2]) / self._spec.axis_scale
        buttons = self._state.buttons.copy()
        for index, (channel, byte, bit) in enumerate(self._spec.button_mapping):
            if channel == report[0] and byte is not None and bit is not None and byte < len(report):
                buttons[index] = int(bool(report[byte] & (1 << bit)))
                updates["buttons"] = buttons
        if updates:
            self._state = self._state._replace(**updates)

    def close(self) -> None:
        handle, self._handle = self._handle, None
        try:
            if handle is not None:
                handle.close()
        finally:
            if self._claim is not None:
                self._claim.release()
                self._claim = None


def open(device_index: int = 0, *, claim: bool = True, claims_dir=None) -> _HidReader:
    """Open the indexed supported HID path, in hid.enumerate() order."""
    sdk = require_sdk()
    by_id = {tuple(spec.hid_id): spec for spec in device_specs.values()}
    try:
        devices = [
            (info, by_id[(info.get("vendor_id"), info.get("product_id"))])
            for info in sdk.enumerate()
            if (info.get("vendor_id"), info.get("product_id")) in by_id
        ]
        if not 0 <= device_index < len(devices):
            raise DeviceNotFound(
                f"no supported SpaceMouse #{device_index} ({len(devices)} found)",
                hint=f"plug it in; {UDEV_HINT}",
            )
        info, spec = devices[device_index]
        from robot_infra.devices.claims import DeviceClaim
        import hashlib
        path = info["path"]
        key = hashlib.sha256(path if isinstance(path, bytes) else str(path).encode()).hexdigest()
        claim = DeviceClaim("spacemouse:" + key, lock_dir=claims_dir) if claim else None
        if claim is not None:
            claim.acquire()
        handle = None
        try:
            handle = sdk.device()
            handle.open_path(info["path"])
            handle.set_nonblocking(1)
            reader = _HidReader(handle, spec)
            reader._claim = claim
            reader.identity = {"vendor_id": info["vendor_id"], "product_id": info["product_id"],
                               "serial_number": info.get("serial_number") or "", "model": spec.name}
            return reader
        except BaseException:
            try:
                if handle is not None:
                    handle.close()
            finally:
                if claim is not None:
                    claim.release()
            raise
    except (OSError, RuntimeError) as error:
        from robot_infra.devices.errors import DeviceBusy
        if isinstance(error, (DeviceNotFound, DeviceBusy)):
            raise
        raise DeviceNotFound(
            f"cannot open SpaceMouse #{device_index}: {error}", hint=UDEV_HINT
        ) from error
