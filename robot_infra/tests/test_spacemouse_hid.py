"""In-tree HID decoder and trezor transport contracts, without hardware or hidapi."""
from __future__ import annotations

from collections import deque
import hashlib
from importlib.resources import files
from pathlib import Path
import subprocess
import sys
import types

import pytest

from robot_infra.devices import DeviceNotFound, VersionMismatch
from robot_infra.devices.backends.teleops import _spacemouse_hid as reader


@pytest.fixture(autouse=True)
def isolated_hid_claims(tmp_path, monkeypatch):
    monkeypatch.setattr("robot_infra.devices.claims._DEFAULT_LOCK_DIR", tmp_path / "claims")


@pytest.fixture(autouse=True)
def isolated_site_tables(tmp_path, monkeypatch):
    """No local device table of this machine (./local or the user config directory) applies."""
    monkeypatch.chdir(tmp_path)
    monkeypatch.setenv("XDG_CONFIG_HOME", str(tmp_path / "xdg"))


WIRELESS_BT = (0x256F, 0xC63A)
# The documented example: 0xC63A declared with the decoding of the built-in SpaceMouse Wireless.
WIRELESS_BT_TABLE = """\
schema_version: 1
devices:
  SpaceMouse Wireless BT (USB cable):
    hid_id: [0x256F, 0xC63A]
    layout: SpaceMouse Wireless
    evidence:
      descriptor: evidence/256f-c63a/report-descriptor.txt
      checked: 2026-11-05
"""
PYSPACEMOUSE_1_0_3_IDS = {
    (0x046D, 0xC627), (0x046D, 0xC626), (0x256F, 0xC641), (0x256F, 0xC635), (0x256F, 0xC632),
    (0x046D, 0xC62B), (0x256F, 0xC62E), (0x256F, 0xC652), (0x046D, 0xC625), (0x046D, 0xC629),
}


def write_table(directory, text=WIRELESS_BT_TABLE):
    directory.mkdir(parents=True, exist_ok=True)
    path = directory / reader.DEVICE_TABLE_FILENAME
    path.write_text(text, encoding="utf-8")
    return path


def words(channel, *values):
    report = [channel]
    for value in values:
        word = value & 0xFFFF
        report.extend((word & 0xFF, word >> 8))
    return report


class Handle:
    def __init__(self, reports=()):
        self.reports = deque(reports)
        self.path = None
        self.nonblocking = None
        self.closed = False
        self.sizes = []

    def open_path(self, path):
        self.path = path

    def set_nonblocking(self, enabled):
        self.nonblocking = enabled

    def read(self, size):
        if self.closed:
            raise OSError("device disconnected")
        self.sizes.append(size)
        return self.reports.popleft() if self.reports else []

    def close(self):
        self.closed = True


def fake_hid(monkeypatch, entries, handle):
    sdk = types.ModuleType("hid")
    sdk.enumerate = lambda: entries
    sdk.device = lambda: handle
    monkeypatch.setitem(sys.modules, "hid", sdk)
    return sdk


@pytest.mark.parametrize("model", ["SpaceNavigator", "SpaceMouse Compact"])
def test_hid_report_matches_native_axis_and_button_mapping(model):
    handle = Handle([words(1, 350, -175, -70), words(2, 105, -140, 210), [3, 1], [3, 2]])
    device = reader._HidReader(handle, reader.BUILTIN_DEVICE_SPECS[model])
    device.read()
    state = device.read()
    assert state[:6] == pytest.approx((1.0, 0.5, 0.2, 0.4, -0.3, 0.6))
    assert device.read().buttons == [1, 0]
    assert device.read().buttons == [0, 1]
    # Six-axis mapping consumed by SpaceMouseTeleop, in base frame.
    assert [-state.y, state.x, state.z, -state.roll, -state.pitch, -state.yaw] == pytest.approx(
        [-0.5, 1.0, 0.2, -0.4, 0.3, -0.6]
    )
    assert handle.sizes == [7] * 4


def test_builtin_table_is_frozen_to_pyspacemouse_1_0_3():
    assert {tuple(spec.hid_id) for spec in reader.BUILTIN_DEVICE_SPECS.values()} == PYSPACEMOUSE_1_0_3_IDS
    table = reader.load_device_specs()  # no local table on this (isolated) host
    assert table.hid_ids == PYSPACEMOUSE_1_0_3_IDS and table.path is None and table.provenance() is None


def test_a_local_layout_entry_enumerates_decodes_and_releases_like_its_builtin_layout(monkeypatch, tmp_path):
    write_table(tmp_path / "site")
    table = reader.load_device_specs(tmp_path / "site")
    spec, inherited = table.specs[WIRELESS_BT], reader.BUILTIN_DEVICE_SPECS["SpaceMouse Wireless"]
    assert spec.name == "SpaceMouse Wireless BT (USB cable)" and spec.hid_id == list(WIRELESS_BT)
    assert (spec.led_id, spec.mappings, spec.button_mapping, spec.axis_scale) == (
        inherited.led_id, inherited.mappings, inherited.button_mapping, inherited.axis_scale)
    handle = Handle([
        words(1, 350, -175, -70, 105, -140, 210),
        [3, 1, 0], [3, 2, 0], [3, 0, 0], words(1, 0, 0, 0, 0, 0, 0),
    ])
    fake_hid(monkeypatch, [
        {"vendor_id": 0x256F, "product_id": 0xC63A, "path": b"bt-usb"},
    ], handle)
    device = reader.open(table=table)
    assert handle.path == b"bt-usb"
    assert device.identity["model"] == "SpaceMouse Wireless BT (USB cable)"
    assert device.device_spec == {"name": "SpaceMouse Wireless BT (USB cable)", "hid_id": "256f:c63a",
                                  "source": "local", "table_sha256": table.sha256}
    assert device.read()[:6] == pytest.approx((1, .5, .2, .4, -.3, .6))
    assert device.read().buttons == [1, 0]
    assert device.read().buttons == [0, 1]
    assert device.read().buttons == [0, 0]
    assert device.read()[:6] == (0,) * 6
    device.close()
    assert handle.closed


def test_without_a_local_table_an_unlisted_3dconnexion_product_is_refused_with_the_table_hint(monkeypatch):
    fake_hid(monkeypatch, [
        {"vendor_id": 0x256F, "product_id": 0xC63A, "path": b"bt-usb"},
    ], Handle())
    with pytest.raises(DeviceNotFound, match="no supported SpaceMouse") as error:
        reader.open()
    assert error.value.hint == reader.unknown_device_message(0x256F, 0xC63A)


def test_a_builtin_device_reports_its_builtin_entry(monkeypatch):
    fake_hid(monkeypatch, [{"vendor_id": 0x256F, "product_id": 0xC635, "path": b"compact"}], Handle())
    device = reader.open()
    assert device.device_spec == {"name": "SpaceMouse Compact", "hid_id": "256f:c635", "source": "builtin"}
    device.close()


def test_a_vendor_match_with_an_unlisted_product_is_not_supported(monkeypatch):
    assert (0x256F, 0xFFFF) not in reader.load_device_specs().specs
    fake_hid(monkeypatch, [
        {"vendor_id": 0x256F, "product_id": 0xFFFF, "path": b"unlisted"},
    ], Handle())
    with pytest.raises(DeviceNotFound, match="no supported SpaceMouse"):
        reader.open()


def test_unknown_device_message_names_the_table_and_the_doc_anchor():
    assert reader.unknown_device_message(0x256F, 0xC63A) == (
        "unknown 3Dconnexion-vendor HID device 256f:c63a; add it to spacemouse-devices.yaml "
        "(see docs/spacemouse-commissioning.md#site-device-table)")


# -- local table: lookup, merge, validation ------------------------------------------------


def test_device_table_lookup_order(tmp_path):
    station, cwd = tmp_path / "station", tmp_path / "cwd"
    env = {"XDG_CONFIG_HOME": str(tmp_path / "config")}

    def lookup(station_dir=None, options=None):
        return reader.resolve_device_table_path(station_dir, options, cwd=cwd, env=env)

    assert lookup() is None and lookup(station) is None
    user = write_table(tmp_path / "config/robot_infra")
    assert lookup() == user
    local = write_table(cwd / "local")
    assert lookup() == local  # the local scope comes first, as for station.yaml
    assert lookup(station) is None  # a station's table is the one next to it
    beside = write_table(station)
    assert lookup(station) == beside
    # An explicit device_table wins; a relative one is relative to the station directory.
    assert lookup(station, {"device_table": "tables/sm.yaml"}) == station / "tables/sm.yaml"
    assert lookup(None, {"device_table": "tables/sm.yaml"}) == cwd / "tables/sm.yaml"
    assert lookup(station, {"device_table": str(user)}) == user
    with pytest.raises(ValueError, match="cannot be read"):
        reader.load_device_specs(station, {"device_table": "tables/sm.yaml"})
    with pytest.raises(ValueError, match="nonempty path"):
        lookup(station, {"device_table": 3})


def test_a_local_entry_with_a_builtin_hid_id_overrides_it_and_others_are_added(tmp_path):
    from robot_infra.devices.provenance import portable_path

    path = write_table(tmp_path, """\
schema_version: 1
devices:
  Compact (site firmware):
    hid_id: "256f:c635"
    layout: SpaceMouse Compact
    axis_scale: 700
  SpaceMouse Wireless BT (USB cable):
    hid_id: [0x256F, 0xC63A]
    layout: SpaceMouse Wireless
""")
    table = reader.load_device_specs(tmp_path)
    compact = table.specs[(0x256F, 0xC635)]
    assert compact.name == "Compact (site firmware)" and compact.axis_scale == 700.0
    assert compact.mappings == reader.BUILTIN_DEVICE_SPECS["SpaceMouse Compact"].mappings
    assert table.hid_ids == PYSPACEMOUSE_1_0_3_IDS | {WIRELESS_BT}
    assert table.overrides == {"Compact (site firmware)": "SpaceMouse Compact"}
    assert table.source_of(table.specs[(0x256F, 0xC62E)]) == "builtin"
    assert table.source_of(compact) == "local"
    assert table.provenance() == {
        "source": "local", "file": portable_path(path),
        "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
        "entries": [
            {"name": "Compact (site firmware)", "hid_id": "256f:c635", "overrides": "SpaceMouse Compact"},
            {"name": "SpaceMouse Wireless BT (USB cable)", "hid_id": "256f:c63a"},
        ],
    }


def test_an_explicit_layout_entry_decodes_its_own_report_layout(tmp_path):
    write_table(tmp_path, """\
schema_version: 1
devices:
  My SpaceMouse:
    hid_id: "256f:c6ff"
    led_id: null
    axis_scale: 350.0
    mappings:
      x:     {channel: 1, byte1: 1,  byte2: 2,  scale: 1}
      y:     {channel: 1, byte1: 3,  byte2: 4,  scale: -1}
      z:     {channel: 1, byte1: 5,  byte2: 6,  scale: -1}
      pitch: {channel: 1, byte1: 7,  byte2: 8,  scale: -1}
      roll:  {channel: 1, byte1: 9,  byte2: 10, scale: -1}
      yaw:   {channel: 1, byte1: 11, byte2: 12, scale: 1}
    button_mapping:
      - {channel: 3, byte: 1, bit: 0}
      - {channel: 3, byte: 1, bit: 1}
""")
    spec = reader.load_device_specs(tmp_path).specs[(0x256F, 0xC6FF)]
    assert spec.led_id is None
    device = reader._HidReader(Handle([words(1, 350, -175, -70, 105, -140, 210), [3, 2]]), spec)
    assert device.read()[:6] == pytest.approx((1, .5, .2, .4, -.3, .6))
    assert device.read().buttons == [0, 1]


_ENTRY = "{hid_id: [0x256F, 0xC6FF], layout: SpaceNavigator}"


@pytest.mark.parametrize("text, entry, match", [
    ("- not a mapping", None, "exactly the keys"),
    (f"schema_version: 1\ndevices: {{a: {_ENTRY}}}\nextra: 1", None, "exactly the keys"),
    (f"schema_version: 2\ndevices: {{a: {_ENTRY}}}", None, "schema_version must be 1"),
    ("schema_version: true\ndevices: {a: {hid_id: [1, 2], layout: SpaceNavigator}}", None, "schema_version must be 1"),
    ("schema_version: 1\ndevices: {}", None, "at least one device"),
    ("schema_version: 1\ndevices: [a]", None, "at least one device"),
    ("schema_version: 1\ndevices: {a: {hid_id: [1, 2], layout: SpaceNavigator, colour: red}}", "a",
     r"unknown keys \['colour'\]"),
    ("schema_version: 1\ndevices: {a: {layout: SpaceNavigator}}", "a", "hid_id is required"),
    ("schema_version: 1\ndevices: {a: {hid_id: [1], layout: SpaceNavigator}}", "a", "hid_id must be"),
    ("schema_version: 1\ndevices: {a: {hid_id: [0x10000, 1], layout: SpaceNavigator}}", "a", "hid_id must be"),
    ("schema_version: 1\ndevices: {a: {hid_id: [true, 1], layout: SpaceNavigator}}", "a", "hid_id must be"),
    ("schema_version: 1\ndevices: {a: {hid_id: '256f-c6ff', layout: SpaceNavigator}}", "a", "hid_id must be"),
    ("schema_version: 1\ndevices: {a: {hid_id: [1, 2], layout: Nope}}", "a", "is not a built-in entry"),
    ("schema_version: 1\ndevices: {a: {hid_id: [1, 2], layout: SpaceNavigator}, "
     "b: {hid_id: [1, 2], layout: SpaceNavigator}}", "b", "is also used by 'a'"),
    ("schema_version: 1\ndevices: {a: {hid_id: [1, 2], axis_scale: 350}}", "a", "without 'layout'"),
    ("schema_version: 1\ndevices: {a: {hid_id: [1, 2], layout: SpaceNavigator, mappings: "
     "{x: {channel: 1, byte1: 1, byte2: 2, scale: 1}}}}", "a", "exactly the axes"),
    ("schema_version: 1\ndevices: {a: {hid_id: [1, 2], layout: SpaceNavigator, mappings: {"
     + ", ".join(f"{axis}: {{channel: 1, byte1: 1, byte2: 2, scale: 2}}" for axis in reader.AXIS_NAMES)
     + "}}}", "a", "scale -1 or 1"),
    ("schema_version: 1\ndevices: {a: {hid_id: [1, 2], layout: SpaceNavigator, mappings: {"
     + ", ".join(f"{axis}: {{channel: 1, byte1: 64, byte2: 2, scale: 1}}" for axis in reader.AXIS_NAMES)
     + "}}}", "a", "byte1/byte2 1..63"),
    ("schema_version: 1\ndevices: {a: {hid_id: [1, 2], layout: SpaceNavigator, "
     "button_mapping: [{channel: 3, byte: 1, bit: 8}]}}", "a", "bit 0..7"),
    ("schema_version: 1\ndevices: {a: {hid_id: [1, 2], layout: SpaceNavigator, "
     "button_mapping: [{channel: 3, byte: null, bit: null}]}}", "a", "or all three null"),
    ("schema_version: 1\ndevices: {a: {hid_id: [1, 2], layout: SpaceNavigator, button_mapping: []}}", "a",
     "nonempty list"),
    ("schema_version: 1\ndevices: {a: {hid_id: [1, 2], layout: SpaceNavigator, axis_scale: 0}}", "a",
     "axis_scale must be"),
    ("schema_version: 1\ndevices: {a: {hid_id: [1, 2], layout: SpaceNavigator, axis_scale: .inf}}", "a",
     "axis_scale must be"),
    ("schema_version: 1\ndevices: {a: {hid_id: [1, 2], layout: SpaceNavigator, led_id: x}}", "a", "led_id must be"),
    ("schema_version: 1\ndevices: {a: {hid_id: [1, 2], layout: SpaceNavigator, evidence: probe}}", "a",
     "evidence must be a mapping"),
    ("schema_version: 1\ndevices: {SpaceNavigator: {hid_id: [1, 2], layout: SpaceNavigator}}", "SpaceNavigator",
     "reuses the name of the built-in entry for 046d:c626"),
    ("schema_version: 1\ndevices: {a: {hid_id: [1, 2]", None, "invalid YAML"),
])
def test_an_invalid_local_table_is_rejected_naming_the_file_and_the_entry(tmp_path, text, entry, match):
    path = write_table(tmp_path, text)
    with pytest.raises(ValueError, match=match) as error:
        reader.load_device_specs(tmp_path)
    assert str(path) in str(error.value)
    if entry is not None:
        assert f"device {entry!r}" in str(error.value)


def test_a_local_entry_may_reuse_a_builtin_name_with_that_entrys_hid_id(tmp_path):
    write_table(tmp_path, "schema_version: 1\ndevices:\n  SpaceNavigator: {hid_id: [0x046D, 0xC626], "
                          "layout: SpaceNavigator, axis_scale: 500}\n")
    table = reader.load_device_specs(tmp_path)
    assert table.specs[(0x046D, 0xC626)].axis_scale == 500.0
    assert table.overrides == {"SpaceNavigator": "SpaceNavigator"}


def test_load_station_resolves_the_spacemouse_device_table_against_the_station_directory(tmp_path):
    import yaml

    from robot_infra.discovery.autoconfig import load_station

    station = tmp_path / "site/station.yaml"
    station.parent.mkdir()
    station.write_text(yaml.safe_dump({"robots": {
        "left": {"type": "franka", "arm_backend": "fake", "teleop": {"backend": "spacemouse"}},
        "right": {"type": "franka", "arm_backend": "fake",
                  "teleop": {"backend": "spacemouse", "options": {"device_table": "tables/sm.yaml"}}},
        "gello": {"type": "franka", "arm_backend": "fake", "teleop": {"backend": "fake"}},
    }}))
    loaded = load_station(station)
    assert "device_table" not in loaded.robots["left"].config.teleop.options  # built-in table only
    assert loaded.robots["right"].config.teleop.options["device_table"] == str(
        station.parent.resolve() / "tables/sm.yaml")
    assert loaded.robots["gello"].config.teleop.options == {}
    beside = write_table(station.parent)
    assert load_station(station).robots["left"].config.teleop.options["device_table"] == str(beside.resolve())


# -- unknown devices -----------------------------------------------------------------------


def hidraw_node(sysfs, name, descriptor, hid_id="", hid_name=""):
    device = sysfs / "class/hidraw" / name / "device"
    device.mkdir(parents=True)
    (device / "report_descriptor").write_bytes(bytes(descriptor))
    if hid_id:
        (device / "uevent").write_text(f"DRIVER=hid-generic\nHID_ID={hid_id}\nHID_NAME={hid_name}\n")


MULTI_AXIS = [0x05, 0x01, 0x09, 0x08, 0xA1, 0x01]
KEYBOARD = [0x05, 0x01, 0x09, 0x06, 0xA1, 0x01]


def test_unknown_hid_devices_report_unlisted_3dconnexion_and_multi_axis_logitech_only(monkeypatch, tmp_path):
    sysfs = tmp_path / "sys"
    hidraw_node(sysfs, "hidraw2", MULTI_AXIS)
    hidraw_node(sysfs, "hidraw0", KEYBOARD)
    fake_hid(monkeypatch, [
        {"vendor_id": 0x256F, "product_id": 0xC63A, "path": b"/dev/hidraw6", "product_string": "SpaceMouse BT"},
        {"vendor_id": 0x256F, "product_id": 0xC635, "path": b"/dev/hidraw3"},  # in the table
        {"vendor_id": 0x046D, "product_id": 0xC542, "path": b"/dev/hidraw0"},  # keyboard receiver
        {"vendor_id": 0x046D, "product_id": 0xC6FF, "path": b"/dev/hidraw1", "usage_page": 1, "usage": 8},
        {"vendor_id": 0x046D, "product_id": 0xC6FE, "path": b"/dev/hidraw2"},  # multi-axis descriptor
        {"vendor_id": 0x1234, "product_id": 0x0001, "path": b"/dev/hidraw5"},
    ], Handle())
    found = reader.unknown_hid_devices(reader.load_device_specs(), sysfs=sysfs)
    assert [(d["vid"], d["pid"], d["path"]) for d in found] == [
        ("256f", "c63a", "/dev/hidraw6"), ("046d", "c6ff", "/dev/hidraw1"), ("046d", "c6fe", "/dev/hidraw2")]
    assert found[0] == {"vid": "256f", "pid": "c63a", "product": "SpaceMouse BT", "path": "/dev/hidraw6",
                        "message": reader.unknown_device_message(0x256F, 0xC63A)}
    write_table(tmp_path / "site")
    found = reader.unknown_hid_devices(reader.load_device_specs(tmp_path / "site"), sysfs=sysfs)
    assert [d["pid"] for d in found] == ["c6ff", "c6fe"]


def test_unknown_hid_devices_fall_back_to_sysfs_without_hidapi(monkeypatch, tmp_path):
    monkeypatch.setitem(sys.modules, "hid", None)
    sysfs = tmp_path / "sys"
    hidraw_node(sysfs, "hidraw0", KEYBOARD, "0003:0000046D:0000C542", "Logitech USB Receiver")
    hidraw_node(sysfs, "hidraw1", MULTI_AXIS, "0003:0000256F:0000C63A", "3Dconnexion SpaceMouse BT")
    hidraw_node(sysfs, "hidraw2", MULTI_AXIS, "0003:0000256F:0000C635", "3Dconnexion SpaceMouse Compact")
    hidraw_node(sysfs, "hidraw3", MULTI_AXIS, "0003:0000046D:0000C6FF", "3Dconnexion SpaceMouse")
    found = reader.unknown_hid_devices(reader.load_device_specs(), sysfs=sysfs, dev=Path("/dev"))
    assert found == [
        {"vid": "256f", "pid": "c63a", "product": "3Dconnexion SpaceMouse BT", "path": "/dev/hidraw1",
         "message": reader.unknown_device_message(0x256F, 0xC63A)},
        {"vid": "046d", "pid": "c6ff", "product": "3Dconnexion SpaceMouse", "path": "/dev/hidraw3",
         "message": reader.unknown_device_message(0x046D, 0xC6FF)},
    ]


def test_loading_the_table_needs_no_hid_sdk(monkeypatch, tmp_path):
    monkeypatch.setitem(sys.modules, "hid", None)
    write_table(tmp_path / "local")
    assert WIRELESS_BT in reader.load_device_specs().specs  # ./local of the working directory


def test_split_reports_keep_other_axes():
    handle = Handle([words(1, 35, -70, -105), words(2, -140, -175, 210), words(1, 350, 0, 0)])
    device = reader._HidReader(handle, reader.BUILTIN_DEVICE_SPECS["SpaceNavigator"])
    assert device.read()[:6] == pytest.approx((0.1, 0.2, 0.3, 0, 0, 0))
    assert device.read()[:6] == pytest.approx((0.1, 0.2, 0.3, 0.5, 0.4, 0.6))
    assert device.read()[:6] == pytest.approx((1, 0, 0, 0.5, 0.4, 0.6))


def test_empty_report_preserves_last_state():
    handle = Handle([words(1, -350, 350, 175), [3, 3]])
    device = reader._HidReader(handle, reader.BUILTIN_DEVICE_SPECS["SpaceMouse Compact"])
    device.read()
    state = device.read()
    assert device.read() is state
    assert device.read() is state
    assert state.buttons == [1, 1]


def test_hid_open_by_index_and_close_releases_handle(monkeypatch):
    entries = [
        {"vendor_id": 0x1234, "product_id": 1, "path": b"ignored"},
        {"vendor_id": 0x046D, "product_id": 0xC626, "path": b"first"},
        {"vendor_id": 0x256F, "product_id": 0xC635, "path": b"second"},
    ]
    handle = Handle()
    fake_hid(monkeypatch, entries, handle)
    device = reader.open(device_index=1)
    assert handle.path == b"second" and handle.nonblocking == 1
    assert device.read() == reader.State(0, 0, 0, 0, 0, 0, [0, 0])
    device.close()
    device.close()
    assert handle.closed
    with pytest.raises(OSError, match="closed"):
        device.read()
    for index in (-1, 2):
        with pytest.raises(DeviceNotFound):
            reader.open(device_index=index)


@pytest.mark.parametrize("failure", ["open_path", "set_nonblocking"])
def test_failed_open_releases_handle_and_reports_udev(monkeypatch, failure):
    handle = Handle()
    fake_hid(monkeypatch, [
        {"vendor_id": 0x256F, "product_id": 0xC635, "path": b"denied"},
    ], handle)

    def fail(*args):
        raise OSError("permission denied")

    monkeypatch.setattr(handle, failure, fail)
    with pytest.raises(DeviceNotFound) as error:
        reader.open()
    assert handle.closed
    assert "99-spacemouse.rules" in error.value.hint and "sudo cp" in error.value.hint
    assert "pip install" not in error.value.hint


def test_short_or_unknown_report_is_handled():
    handle = Handle([[99, 255], [1], [1, 0], [3], words(1, -32768)])
    device = reader._HidReader(handle, reader.BUILTIN_DEVICE_SPECS["SpaceNavigator"])
    state = device.read()
    for _ in range(3):
        assert device.read() is state
    assert device.read().x == -32768 / 350
    assert device.read().y == 0


def test_sdk_import_is_lazy_and_missing_hidapi_hint_is_exact():
    code = '''import sys
sys.modules["hid"] = None
sys.modules["pyspacemouse"] = None
sys.modules["easyhid"] = None
from robot_infra.devices.backends.teleops import _spacemouse_hid as reader
from robot_infra.devices import SdkMissing
try:
    reader.open()
except SdkMissing as error:
    assert "hidapi==0.14.0.post4" in error.message
    assert error.hint == "pip install -e '.[spacemouse]'"
else:
    raise AssertionError("opening must require hidapi")
'''
    result = subprocess.run([sys.executable, "-B", "-c", code], capture_output=True, text=True)
    assert result.returncode == 0, result.stderr


def test_wrong_hid_distribution_is_reported(monkeypatch):
    monkeypatch.setitem(sys.modules, "hid", types.SimpleNamespace(Device=Handle))
    with pytest.raises(VersionMismatch, match="wrong distribution 'hid'") as error:
        reader.open()
    assert "trezor hidapi" in error.value.message
    assert "pip install -e '.[spacemouse]'" in error.value.hint


def test_udev_rule_is_packaged():
    rules = files("robot_infra.setup").joinpath("99-spacemouse.rules").read_text()
    assert rules.splitlines() == [
        'SUBSYSTEM=="hidraw", ATTRS{idVendor}=="256f", MODE="0664", GROUP="plugdev"',
        'SUBSYSTEM=="hidraw", ATTRS{idVendor}=="046d", MODE="0664", GROUP="plugdev"',
    ]
    metadata = (Path(__file__).resolve().parents[1] / "pyproject.toml").read_text()
    assert '"setup/*.rules"' in metadata


@pytest.mark.parametrize("model", list(reader.BUILTIN_DEVICE_SPECS))
def test_every_model_decodes_combined_or_split_axes_and_all_button_bits(model):
    spec = reader.BUILTIN_DEVICE_SPECS[model]
    reports = {}
    for number, axis in enumerate(spec.mappings.values(), 1):
        report = reports.setdefault(axis.channel, [axis.channel])
        report.extend([0] * (max(axis.byte1, axis.byte2) + 1 - len(report)))
        value = number * 35
        report[axis.byte1], report[axis.byte2] = value & 0xFF, value >> 8
    handle = Handle(reports.values())
    device = reader._HidReader(handle, spec)
    for _ in reports:
        state = device.read()
    for number, (name, axis) in enumerate(spec.mappings.items(), 1):
        assert getattr(state, name) == pytest.approx(number * 0.1 * axis.scale)
    for index, button in enumerate(spec.button_mapping):
        if button.channel is None:  # SpaceMouse USB's native no-buttons placeholder.
            assert state.buttons == [0]
            continue
        report = [button.channel] + [0] * max(b.byte for b in spec.button_mapping)
        report[button.byte] = 1 << button.bit
        handle.reports.append(report)
        assert device.read().buttons == [int(i == index) for i in range(len(spec.button_mapping))]


def test_read_error_propagates_to_the_polling_backend():
    handle = Handle()
    device = reader._HidReader(handle, reader.BUILTIN_DEVICE_SPECS["SpaceNavigator"])
    handle.closed = True
    with pytest.raises(OSError, match="disconnected"):
        device.read()


def test_open_claim_is_exclusive_and_released(monkeypatch, tmp_path):
    from robot_infra.devices.errors import DeviceBusy
    handle = Handle([])
    fake_hid(monkeypatch, [
        {"vendor_id": 0x256F, "product_id": 0xC635, "path": b"exclusive-test"},
    ], handle)
    first = reader.open(claims_dir=tmp_path / "claims")
    try:
        with pytest.raises(DeviceBusy):
            reader.open(claims_dir=tmp_path / "claims")
    finally:
        first.close()
    second = reader.open(claims_dir=tmp_path / "claims")
    second.close()


def test_claim_opt_out_is_forwarded_by_robot_config(monkeypatch):
    from robot_infra.devices import TeleopConfig
    from robot_infra.devices.robots.franka import FrankaConfig, FrankaRobot
    robot = FrankaRobot(FrankaConfig(arm_backend='fake', gripper=None, claim=False,
                                    teleop=TeleopConfig(backend='spacemouse')))
    assert robot.teleop._claim is False


def test_claim_can_be_explicitly_disabled(tmp_path, monkeypatch):
    fake_hid(monkeypatch, [
        {'vendor_id': 0x256F, 'product_id': 0xC635, 'path': b'unclaimed'},
    ], Handle())
    first = reader.open(claim=False, claims_dir=tmp_path / 'claims')
    second = reader.open(claim=False, claims_dir=tmp_path / 'claims')
    first.close()
    second.close()
    assert not (tmp_path / 'claims').exists()
