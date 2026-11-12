"""Nonblocking SpaceMouse HID reader using trezor/cython-hidapi (distribution ``hidapi``).

Device table and decoder derived from PySpaceMouse 1.0.3, the HIL-SERL
vendored version. Derived from PySpaceMouse by John Harrison et al., MIT.
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

Modified for the real-world-rl device layer (then robot_infra): replace easyhid with hidapi, omit callbacks, add optional advisory device claims,
guard short reports, and close failed opens. The built-in device table is unchanged from 1.0.3.

Models a site meets beyond that table (a new product id, a firmware variant) are added or
overridden by the site's local device table, ``spacemouse-devices.yaml``, never in this file:
``load_device_specs`` merges it over the built-in table and is the one entry point to the
runtime table. Format and location: docs/spacemouse-commissioning.md, "Site device table".
"""
from __future__ import annotations

import importlib
import math
import os
import re
from dataclasses import dataclass, field
from pathlib import Path
from types import MappingProxyType, ModuleType
from typing import Any, Mapping, NamedTuple, Sequence

from real_world_rl.devices.errors import DeviceNotFound, SdkMissing, VersionMismatch

HIDAPI_REQUIREMENT = "hidapi==0.14.0.post4"
SDK_HINT = "pip install -e '.[spacemouse]'"
UDEV_HINT = (
    "on the NUC, repository root (no venv required): "
    "sudo cp real_world_rl/setup/99-spacemouse.rules /etc/udev/rules.d/; "
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


# The PySpaceMouse 1.0.3 table, by entry name. Readers use the merged runtime table from
# load_device_specs; a local table refers to these entries by name in ``layout:``.
_BUILTIN_DEVICE_SPECS = {
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
BUILTIN_DEVICE_SPECS: Mapping[str, DeviceSpec] = MappingProxyType(_BUILTIN_DEVICE_SPECS)

# -- local device table ----------------------------------------------------------------------

DEVICE_TABLE_FILENAME = "spacemouse-devices.yaml"
DEVICE_TABLE_OPTION = "device_table"  # teleop option: an explicit local table
DEVICE_TABLE_SCHEMA_VERSION = 1
SITE_TABLE_DOC = "docs/spacemouse-commissioning.md#site-device-table"
AXIS_NAMES = ("x", "y", "z", "roll", "pitch", "yaw")
_ENTRY_KEYS = frozenset({"hid_id", "layout", "led_id", "mappings", "button_mapping", "axis_scale", "evidence"})
# Without ``layout:`` an entry spells out its decoding; ``led_id`` defaults to None (no LED).
_LAYOUT_KEYS = ("mappings", "button_mapping", "axis_scale")
_AXIS_KEYS = frozenset({"channel", "byte1", "byte2", "scale"})
_BUTTON_KEYS = frozenset({"channel", "byte", "bit"})
_MAX_REPORT_BYTE = 63
_HID_ID_TEXT = re.compile(r"[0-9a-fA-F]{4}:[0-9a-fA-F]{4}")

THREEDCONNEXION_VENDOR_ID = 0x256F
# Older 3Dconnexion units enumerate under Logitech, which also makes keyboard and mouse
# receivers: an unlisted Logitech device is reported only when it describes itself as a
# multi-axis controller.
LOGITECH_VENDOR_ID = 0x046D
# HID report descriptor start: Usage Page (Generic Desktop), Usage (Multi-axis Controller).
MULTI_AXIS_DESCRIPTOR_PREFIX = bytes((0x05, 0x01, 0x09, 0x08))
_GENERIC_DESKTOP_PAGE, _MULTI_AXIS_USAGE = 0x01, 0x08


def format_hid_id(hid_id: Sequence[int]) -> str:
    return f"{int(hid_id[0]):04x}:{int(hid_id[1]):04x}"


def _hid_key(spec: DeviceSpec) -> tuple[int, int]:
    return int(spec.hid_id[0]), int(spec.hid_id[1])


@dataclass(frozen=True)
class DeviceTable:
    """The runtime device table, by ``(vendor_id, product_id)``: the built-in entries, with the
    local table's entries added, or replacing the built-in entry of the same ``hid_id``.

    ``path``/``sha256`` identify the local file (None: the built-in table alone);
    ``overrides`` maps a local entry name to the built-in entry it replaced."""

    specs: Mapping[tuple[int, int], DeviceSpec]
    local_names: frozenset[str] = frozenset()
    path: Path | None = None
    sha256: str | None = None
    overrides: Mapping[str, str] = field(default_factory=dict)

    @property
    def hid_ids(self) -> frozenset[tuple[int, int]]:
        return frozenset(self.specs)

    def source_of(self, spec: DeviceSpec) -> str:
        return "local" if spec.name in self.local_names else "builtin"

    def spec_provenance(self, spec: DeviceSpec) -> dict:
        """Which entry decodes a device: name, hid_id, source and, for a local entry, the
        table's sha256."""
        record = {"name": spec.name, "hid_id": format_hid_id(spec.hid_id), "source": self.source_of(spec)}
        if record["source"] == "local":
            record["table_sha256"] = self.sha256
        return record

    def provenance(self) -> dict | None:
        """The local table in effect (portable file path, sha256 and its entries), or None for
        the built-in table alone."""
        if self.path is None:
            return None
        from real_world_rl.devices.provenance import portable_path

        entries = []
        for spec in self.specs.values():
            if spec.name in self.local_names:
                entry = {"name": spec.name, "hid_id": format_hid_id(spec.hid_id)}
                if spec.name in self.overrides:
                    entry["overrides"] = self.overrides[spec.name]
                entries.append(entry)
        return {"source": "local", "file": portable_path(self.path), "sha256": self.sha256, "entries": entries}


def builtin_device_table() -> DeviceTable:
    return DeviceTable(specs={_hid_key(spec): spec for spec in _BUILTIN_DEVICE_SPECS.values()})


def resolve_device_table_path(
    station_dir: str | Path | None = None,
    options: Mapping[str, Any] | None = None,
    *,
    cwd: str | Path | None = None,
    env: Mapping[str, str] | None = None,
) -> Path | None:
    """The local table that applies, or None for the built-in table alone, in this order:

    1. ``options["device_table"]``; a relative path is relative to ``station_dir`` (the
       directory of the station file), else to the working directory.
    2. With a station: ``<station_dir>/spacemouse-devices.yaml`` when it exists.
    3. Without a station: the first existing ``spacemouse-devices.yaml`` of the station scopes,
       ``<cwd>/local/``, ``$XDG_CONFIG_HOME/rwrl/``, then the pre-0.1.0 ``$XDG_CONFIG_HOME/robot_infra/``.
    """
    base = Path.cwd() if cwd is None else Path(cwd)
    explicit = (options or {}).get(DEVICE_TABLE_OPTION)
    if explicit is not None:
        if not isinstance(explicit, (str, os.PathLike)) or not str(explicit):
            raise ValueError(f"spacemouse option '{DEVICE_TABLE_OPTION}' must be a nonempty path, got {explicit!r}")
        path = Path(explicit).expanduser()
        return path if path.is_absolute() else (base if station_dir is None else Path(station_dir)) / path
    if station_dir is not None:
        candidate = Path(station_dir) / DEVICE_TABLE_FILENAME
        return candidate if candidate.is_file() else None
    from real_world_rl.common.station_paths import station_lookup_dirs

    for directory in station_lookup_dirs(cwd=base, env=env):
        candidate = directory / DEVICE_TABLE_FILENAME
        if candidate.is_file():
            return candidate
    return None


def load_device_specs(
    station_dir: str | Path | None = None,
    options: Mapping[str, Any] | None = None,
    *,
    cwd: str | Path | None = None,
    env: Mapping[str, str] | None = None,
) -> DeviceTable:
    """The runtime device table: the built-in table merged with the local table that
    ``resolve_device_table_path`` selects. A local table that fails validation is rejected as
    a whole (``ValueError`` naming the file and the entry)."""
    path = resolve_device_table_path(station_dir, options, cwd=cwd, env=env)
    return builtin_device_table() if path is None else parse_device_table(path)


def station_teleop_options(options: Mapping[str, Any], station_dir: str | Path) -> dict:
    """``options`` of a station's spacemouse teleop with ``device_table`` as the absolute path of
    the local table that applies (unchanged when only the built-in table applies)."""
    path = resolve_device_table_path(station_dir, options)
    return dict(options) if path is None else {**options, DEVICE_TABLE_OPTION: str(path)}


def parse_device_table(path: str | Path) -> DeviceTable:
    import hashlib

    import yaml

    path = Path(path)

    def invalid(message: str, entry: str | None = None) -> ValueError:
        where = f"SpaceMouse device table {path}" + ("" if entry is None else f", device {entry!r}")
        return ValueError(f"{where}: {message}")

    try:
        raw = path.read_bytes()
    except OSError as error:
        raise invalid(f"cannot be read ({error})") from error
    try:
        document = yaml.safe_load(raw)
    except yaml.YAMLError as error:
        raise invalid(f"invalid YAML: {error}") from error
    if not isinstance(document, Mapping) or set(document) != {"schema_version", "devices"}:
        raise invalid("must be a mapping with exactly the keys 'schema_version' and 'devices'")
    version = document["schema_version"]
    if type(version) is not int or version != DEVICE_TABLE_SCHEMA_VERSION:
        raise invalid(f"schema_version must be {DEVICE_TABLE_SCHEMA_VERSION}, got {version!r}")
    devices = document["devices"]
    if not isinstance(devices, Mapping) or not devices:
        raise invalid("'devices' must map at least one device name to its entry")

    builtin_ids = {}
    for key, spec in _BUILTIN_DEVICE_SPECS.items():
        builtin_ids[key] = builtin_ids[spec.name] = _hid_key(spec)
    specs = {_hid_key(spec): spec for spec in _BUILTIN_DEVICE_SPECS.values()}
    local_ids: dict[tuple[int, int], str] = {}
    overrides = {}
    for name, entry in devices.items():
        if not isinstance(name, str) or not name.strip():
            raise invalid(f"device names must be nonempty text, got {name!r}")
        spec = _local_spec(name, entry, lambda message: invalid(message, name))
        key = _hid_key(spec)
        if key in local_ids:
            raise invalid(f"hid_id {format_hid_id(key)} is also used by {local_ids[key]!r}", name)
        if name in builtin_ids and builtin_ids[name] != key:
            raise invalid(
                f"reuses the name of the built-in entry for {format_hid_id(builtin_ids[name])} with another "
                "hid_id; give it its own name (or the same hid_id to override that entry)", name)
        local_ids[key] = name
        if key in specs:
            overrides[name] = specs[key].name
        specs[key] = spec
    return DeviceTable(
        specs=specs, local_names=frozenset(local_ids.values()), path=path,
        sha256=hashlib.sha256(raw).hexdigest(), overrides=overrides,
    )


def _local_spec(name: str, entry: Any, invalid) -> DeviceSpec:
    if not isinstance(entry, Mapping):
        raise invalid("the entry must be a mapping")
    unknown = sorted(set(entry) - _ENTRY_KEYS)
    if unknown:
        raise invalid(f"unknown keys {unknown}; known: {sorted(_ENTRY_KEYS)}")
    if "hid_id" not in entry:
        raise invalid("hid_id is required")
    hid_id = _parse_hid_id(entry["hid_id"], invalid)
    layout = entry.get("layout")
    if layout is not None:
        if not isinstance(layout, str) or layout not in _BUILTIN_DEVICE_SPECS:
            raise invalid(f"layout {layout!r} is not a built-in entry; built-in: {sorted(_BUILTIN_DEVICE_SPECS)}")
        inherited = _BUILTIN_DEVICE_SPECS[layout]
        fields = {"led_id": inherited.led_id, "mappings": dict(inherited.mappings),
                  "button_mapping": list(inherited.button_mapping), "axis_scale": inherited.axis_scale}
    else:
        missing = [key for key in _LAYOUT_KEYS if key not in entry]
        if missing:
            raise invalid(f"without 'layout', {missing} are required")
        fields = {"led_id": None}
    if "led_id" in entry:
        fields["led_id"] = _parse_led_id(entry["led_id"], invalid)
    if "mappings" in entry:
        fields["mappings"] = _parse_mappings(entry["mappings"], invalid)
    if "button_mapping" in entry:
        fields["button_mapping"] = _parse_buttons(entry["button_mapping"], invalid)
    if "axis_scale" in entry:
        fields["axis_scale"] = _parse_axis_scale(entry["axis_scale"], invalid)
    if "evidence" in entry and not isinstance(entry["evidence"], Mapping):
        raise invalid("evidence must be a mapping (for example probe, checked, note)")
    return DeviceSpec(name=name, hid_id=list(hid_id), **fields)


def _is_int(value: Any, low: int, high: int) -> bool:
    return type(value) is int and low <= value <= high


def _parse_hid_id(value: Any, invalid) -> tuple[int, int]:
    if isinstance(value, str) and _HID_ID_TEXT.fullmatch(value):
        vendor, product = value.split(":")
        return int(vendor, 16), int(product, 16)
    if isinstance(value, (list, tuple)) and len(value) == 2 and all(_is_int(v, 0, 0xFFFF) for v in value):
        return int(value[0]), int(value[1])
    raise invalid(f"hid_id must be two integers 0..0xFFFF ([0x256F, 0xC62E]) or 'vvvv:pppp', got {value!r}")


def _parse_led_id(value: Any, invalid) -> list[int] | None:
    if value is None:
        return None
    if isinstance(value, (list, tuple)) and len(value) == 2 and all(_is_int(v, 0, 0xFFFF) for v in value):
        return [int(v) for v in value]
    raise invalid(f"led_id must be null or two integers, got {value!r}")


def _parse_mappings(value: Any, invalid) -> dict[str, AxisSpec]:
    if not isinstance(value, Mapping) or set(value) != set(AXIS_NAMES):
        raise invalid(f"mappings must map exactly the axes {list(AXIS_NAMES)}")
    mappings = {}
    for axis, spec in value.items():
        if not isinstance(spec, Mapping) or set(spec) != _AXIS_KEYS:
            raise invalid(f"mappings.{axis} must have exactly the keys {sorted(_AXIS_KEYS)}")
        if not (_is_int(spec["channel"], 0, 255) and _is_int(spec["byte1"], 1, _MAX_REPORT_BYTE)
                and _is_int(spec["byte2"], 1, _MAX_REPORT_BYTE) and spec["scale"] in (-1, 1)
                and type(spec["scale"]) is int):
            raise invalid(f"mappings.{axis} needs channel 0..255, byte1/byte2 1..{_MAX_REPORT_BYTE} and "
                          f"scale -1 or 1, got {dict(spec)}")
        mappings[axis] = AxisSpec(spec["channel"], spec["byte1"], spec["byte2"], spec["scale"])
    return mappings


def _parse_buttons(value: Any, invalid) -> list[ButtonSpec]:
    if not isinstance(value, (list, tuple)) or not value:
        raise invalid("button_mapping must be a nonempty list (button_0 first)")
    buttons = []
    for index, spec in enumerate(value):
        if not isinstance(spec, Mapping) or set(spec) != _BUTTON_KEYS:
            raise invalid(f"button_mapping[{index}] must have exactly the keys {sorted(_BUTTON_KEYS)}")
        channel, byte, bit = spec["channel"], spec["byte"], spec["bit"]
        no_button = channel is None and byte is None and bit is None
        if not no_button and not (_is_int(channel, 0, 255) and _is_int(byte, 1, _MAX_REPORT_BYTE)
                                  and _is_int(bit, 0, 7)):
            raise invalid(f"button_mapping[{index}] needs channel 0..255, byte 1..{_MAX_REPORT_BYTE} and "
                          f"bit 0..7, or all three null, got {dict(spec)}")
        buttons.append(ButtonSpec(channel, byte, bit))
    return buttons


def _parse_axis_scale(value: Any, invalid) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
        raise invalid(f"axis_scale must be a finite number > 0, got {value!r}")
    return float(value)


# -- unknown devices -------------------------------------------------------------------------


def unknown_device_message(vendor_id: int, product_id: int) -> str:
    return (f"unknown 3Dconnexion-vendor HID device {vendor_id:04x}:{product_id:04x}; "
            f"add it to {DEVICE_TABLE_FILENAME} (see {SITE_TABLE_DOC})")


def describes_multi_axis_controller(descriptor: bytes | Sequence[int]) -> bool:
    return bytes(descriptor[:len(MULTI_AXIS_DESCRIPTOR_PREFIX)]) == MULTI_AXIS_DESCRIPTOR_PREFIX


def is_unknown_spacemouse(vendor_id: int, product_id: int, table: DeviceTable, *, multi_axis: bool = False) -> bool:
    """A HID device that looks like a SpaceMouse but is not in ``table``: any unlisted 3Dconnexion
    product, or an unlisted Logitech one that is a multi-axis controller."""
    if (vendor_id, product_id) in table.specs:
        return False
    return vendor_id == THREEDCONNEXION_VENDOR_ID or (vendor_id == LOGITECH_VENDOR_ID and multi_axis)


def _path_text(path: Any) -> str:
    return path.decode(errors="replace") if isinstance(path, bytes) else str(path)


def _read_bytes(path: Path) -> bytes:
    try:
        return path.read_bytes()
    except OSError:
        return b""


def _hidraw_describes_multi_axis(node: str, sysfs: Path) -> bool:
    name = Path(node).name
    return name.startswith("hidraw") and describes_multi_axis_controller(
        _read_bytes(sysfs / "class/hidraw" / name / "device/report_descriptor"))


def _unknown_entry(vendor_id: int, product_id: int, product: str, path: str) -> dict:
    return {"vid": f"{vendor_id:04x}", "pid": f"{product_id:04x}", "product": product, "path": path,
            "message": unknown_device_message(vendor_id, product_id)}


def unknown_hid_devices(table: DeviceTable, *, sysfs: str | Path = "/sys", dev: str | Path = "/dev") -> list[dict]:
    """HID interfaces that look like a SpaceMouse but are not in ``table``
    (``is_unknown_spacemouse``), each ``{vid, pid, product, path, message}``: from hidapi's
    enumeration when it is importable, else from the sysfs hidraw nodes. Read-only."""
    sysfs, dev = Path(sysfs), Path(dev)
    try:
        enumerate_devices = getattr(importlib.import_module("hid"), "enumerate", None)
    except (ImportError, OSError):
        enumerate_devices = None
    infos = None
    if callable(enumerate_devices):
        try:
            infos = list(enumerate_devices())
        except (OSError, RuntimeError):
            infos = None
    found: dict[tuple[int, int, str], dict] = {}
    if infos is not None:
        for info in infos:
            vendor_id, product_id = info.get("vendor_id"), info.get("product_id")
            if type(vendor_id) is not int or type(product_id) is not int:
                continue
            path = _path_text(info.get("path", ""))
            multi_axis = vendor_id == LOGITECH_VENDOR_ID and (
                (info.get("usage_page"), info.get("usage")) == (_GENERIC_DESKTOP_PAGE, _MULTI_AXIS_USAGE)
                or _hidraw_describes_multi_axis(path, sysfs))
            if is_unknown_spacemouse(vendor_id, product_id, table, multi_axis=multi_axis):
                found[(vendor_id, product_id, path)] = _unknown_entry(
                    vendor_id, product_id, str(info.get("product_string") or ""), path)
        return list(found.values())
    for node in sorted((sysfs / "class/hidraw").glob("hidraw*")):
        uevent = _read_bytes(node / "device/uevent").decode(errors="replace")
        match = re.search(r"^HID_ID=[0-9A-Fa-f]+:([0-9A-Fa-f]+):([0-9A-Fa-f]+)$", uevent, re.MULTILINE)
        if match is None:
            continue
        vendor_id, product_id = int(match.group(1), 16), int(match.group(2), 16)
        multi_axis = describes_multi_axis_controller(_read_bytes(node / "device/report_descriptor"))
        if is_unknown_spacemouse(vendor_id, product_id, table, multi_axis=multi_axis):
            name = re.search(r"^HID_NAME=(.*)$", uevent, re.MULTILINE)
            path = str(dev / node.name)
            found[(vendor_id, product_id, path)] = _unknown_entry(
                vendor_id, product_id, name.group(1) if name else "", path)
    return list(found.values())


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
        self.device_spec = {}  # DeviceTable.spec_provenance of the decoding entry
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


def open(
    device_index: int = 0, *, table: DeviceTable | None = None, claim: bool = True, claims_dir=None,
) -> _HidReader:
    """Open the indexed HID path whose exact (vendor, product) id is in ``table`` (default:
    ``load_device_specs()``), in hid.enumerate() order."""
    sdk = require_sdk()
    table = load_device_specs() if table is None else table
    try:
        infos = list(sdk.enumerate())
        devices = [
            (info, table.specs[(info.get("vendor_id"), info.get("product_id"))])
            for info in infos
            if (info.get("vendor_id"), info.get("product_id")) in table.specs
        ]
        if not 0 <= device_index < len(devices):
            unknown = sorted({
                unknown_device_message(info["vendor_id"], info["product_id"]) for info in infos
                if type(info.get("vendor_id")) is int and type(info.get("product_id")) is int
                and is_unknown_spacemouse(info["vendor_id"], info["product_id"], table, multi_axis=(
                    info.get("usage_page"), info.get("usage")) == (_GENERIC_DESKTOP_PAGE, _MULTI_AXIS_USAGE))
            })
            raise DeviceNotFound(
                f"no supported SpaceMouse #{device_index} ({len(devices)} found)",
                hint="; ".join(unknown) if unknown else f"plug it in; {UDEV_HINT}",
            )
        info, spec = devices[device_index]
        from real_world_rl.devices.claims import DeviceClaim
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
            reader.device_spec = table.spec_provenance(spec)
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
        from real_world_rl.devices.errors import DeviceBusy
        if isinstance(error, (DeviceNotFound, DeviceBusy)):
            raise
        raise DeviceNotFound(
            f"cannot open SpaceMouse #{device_index}: {error}", hint=UDEV_HINT
        ) from error
