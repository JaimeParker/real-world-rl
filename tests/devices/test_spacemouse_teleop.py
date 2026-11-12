"""SpaceMouse teleop contract against a mocked reader, without HID quantization or hardware."""
from __future__ import annotations

import sys
import threading
import time
import types

import numpy as np
import pytest

from robot_infra.devices import DeviceNotFound, SdkMissing, Teleop, TeleopConfig, VersionMismatch
from robot_infra.devices.backends.teleops import spacemouse as spacemouse_backend
from robot_infra.devices.backends.teleops.spacemouse import SpaceMouseTeleop
from robot_infra.discovery import enumerate as probes
from robot_infra.doctor import checks


def make_hid() -> types.ModuleType:
    fake = types.ModuleType("hid")
    fake.twist = (0.0,) * 6
    fake.buttons = (0, 0)
    fake.devices = []

    class Device:
        def __init__(self):
            self.closed = False

        def read(self):
            if self.closed:
                raise OSError("read from a closed SpaceMouse")
            time.sleep(0.001)
            x, y, z, roll, pitch, yaw = fake.twist
            return types.SimpleNamespace(
                x=x, y=y, z=z, roll=roll, pitch=pitch, yaw=yaw, buttons=list(fake.buttons)
            )

        def close(self):
            self.closed = True

    def open_device(**kwargs):
        device = Device()
        device.kwargs = kwargs
        fake.devices.append(device)
        return device

    fake.open = open_device
    return fake


@pytest.fixture(autouse=True)
def isolated_site_tables(tmp_path, monkeypatch):
    """No local device table of this machine (./local or the user config directory) applies."""
    monkeypatch.chdir(tmp_path)
    monkeypatch.setenv("XDG_CONFIG_HOME", str(tmp_path / "xdg"))


WIRELESS_BT_TABLE = """\
schema_version: 1
devices:
  SpaceMouse Wireless BT (USB cable):
    hid_id: [0x256F, 0xC63A]
    layout: SpaceMouse Wireless
"""


def write_table(directory, text=WIRELESS_BT_TABLE):
    directory.mkdir(parents=True, exist_ok=True)
    path = directory / spacemouse_backend._spacemouse_hid.DEVICE_TABLE_FILENAME
    path.write_text(text, encoding="utf-8")
    return path


@pytest.fixture
def hid_sdk(monkeypatch):
    fake = make_hid()
    fake.device = lambda: None
    monkeypatch.setitem(sys.modules, "hid", fake)
    monkeypatch.setattr(spacemouse_backend._spacemouse_hid, "open", lambda **kw: fake.open(**kw))
    return fake


@pytest.fixture
def spacemouse(hid_sdk):
    created = []

    def build(**options):
        teleop = Teleop.backend("spacemouse")(TeleopConfig(backend="spacemouse", options=options))
        teleop.connect()
        created.append(teleop)
        return teleop

    yield build
    for teleop in created:
        teleop.disconnect()


def settled_read(teleop, predicate, attempts=200):
    """Read until the mocked state has propagated through the poll thread."""
    for _ in range(attempts):
        sample = teleop.read()
        if predicate(sample):
            return sample
    raise AssertionError("mocked state never reached the sample")


def test_registered_and_contract():
    assert Teleop.backend("spacemouse") is SpaceMouseTeleop
    assert SpaceMouseTeleop.ACTION_SPACE == "cartesian_delta" and SpaceMouseTeleop.HAS_GRIPPER


def test_missing_sdk_raises_sdk_missing_with_install_hint(monkeypatch):
    monkeypatch.setitem(sys.modules, "hid", None)  # makes `import hid` fail
    teleop = SpaceMouseTeleop(TeleopConfig(backend="spacemouse"))
    with pytest.raises(SdkMissing) as info:
        teleop.connect()
    assert info.value.hint == "pip install -e '.[spacemouse]'"
    assert not teleop.is_connected


def test_wrong_hid_api_is_version_mismatch(monkeypatch):
    legacy = types.ModuleType("hid")
    legacy.Device = lambda: None
    monkeypatch.setitem(sys.modules, "hid", legacy)
    with pytest.raises(VersionMismatch) as info:
        SpaceMouseTeleop(TeleopConfig(backend="spacemouse")).connect()
    assert "wrong distribution 'hid'" in info.value.message
    assert "pip install -e '.[spacemouse]'" in info.value.hint


def test_open_failure_is_device_not_found(monkeypatch):
    fake = types.ModuleType("hid")
    fake.device = lambda: None
    fake.enumerate = lambda: []
    monkeypatch.setitem(sys.modules, "hid", fake)
    with pytest.raises(DeviceNotFound):
        SpaceMouseTeleop(TeleopConfig(backend="spacemouse")).connect()


def test_device_index_is_forwarded(spacemouse, hid_sdk):
    spacemouse(device_index=2)
    kwargs = hid_sdk.devices[0].kwargs
    assert set(kwargs) == {"device_index", "table"} and kwargs["device_index"] == 2
    assert kwargs["table"].path is None  # the built-in table alone


def test_the_device_table_option_is_loaded_at_construction_and_used_to_open(spacemouse, hid_sdk, tmp_path):
    path = write_table(tmp_path / "tables")
    spacemouse(device_table=str(path))
    table = hid_sdk.devices[0].kwargs["table"]
    assert table.path == path and (0x256F, 0xC63A) in table.specs
    path.write_text("schema_version: 1\ndevices: {a: {hid_id: [1, 2], layout: Nope}}\n", encoding="utf-8")
    with pytest.raises(ValueError, match="is not a built-in entry"):
        SpaceMouseTeleop(TeleopConfig(backend="spacemouse", options={"device_table": str(path)}))
    with pytest.raises(ValueError, match="cannot be read"):
        SpaceMouseTeleop(TeleopConfig(backend="spacemouse", options={"device_table": str(tmp_path / "absent.yaml")}))


def test_without_the_option_the_site_table_of_the_working_directory_applies(spacemouse, hid_sdk, tmp_path):
    path = write_table(tmp_path / "local")
    spacemouse()
    assert hid_sdk.devices[0].kwargs["table"].path.resolve() == path.resolve()


def test_device_spec_names_the_decoding_entry_of_the_open_device(monkeypatch, tmp_path):
    from robot_infra.devices.backends.teleops import _spacemouse_hid

    class Handle:
        def open_path(self, path):
            pass

        def set_nonblocking(self, enabled):
            pass

        def read(self, size):
            return []

        def close(self):
            pass

    fake = types.ModuleType("hid")
    fake.device = Handle
    fake.enumerate = lambda: [{"vendor_id": 0x256F, "product_id": 0xC63A, "path": b"bt"}]
    monkeypatch.setitem(sys.modules, "hid", fake)
    path = write_table(tmp_path / "tables")
    teleop = SpaceMouseTeleop(TeleopConfig(backend="spacemouse", options={"device_table": str(path)}), claim=False)
    teleop.connect()
    try:
        assert teleop.device_spec == {"name": "SpaceMouse Wireless BT (USB cable)", "hid_id": "256f:c63a",
                                      "source": "local", "table_sha256": _spacemouse_hid.parse_device_table(path).sha256}
        assert teleop.device_identity["model"] == "SpaceMouse Wireless BT (USB cable)"
    finally:
        teleop.disconnect()


def test_axis_mapping_signs_and_order(spacemouse, hid_sdk):
    teleop = spacemouse()
    hid_sdk.twist = (0.1, 0.2, 0.3, 0.4, 0.5, 0.6)  # x, y, z, roll, pitch, yaw
    sample = settled_read(teleop, lambda s: s.action[0] != 0)
    np.testing.assert_allclose(sample.action, [-0.2, 0.1, 0.3, -0.4, -0.5, -0.6], atol=1e-6)
    assert sample.action.dtype == np.float32 and sample.intervening


def test_invert_axes_flips_selected_outputs(spacemouse, hid_sdk):
    teleop = spacemouse(invert_axes=[0, 5])
    hid_sdk.twist = (0.1, 0.2, 0.3, 0.4, 0.5, 0.6)
    sample = settled_read(teleop, lambda s: s.action[0] != 0)
    np.testing.assert_allclose(sample.action, [0.2, 0.1, 0.3, -0.4, -0.5, 0.6], atol=1e-6)


def test_invalid_options_rejected():
    with pytest.raises(ValueError, match="unknown spacemouse option"):
        SpaceMouseTeleop(TeleopConfig(backend="spacemouse", options={"bogus": 1}))
    with pytest.raises(ValueError, match="invert_axes"):
        SpaceMouseTeleop(TeleopConfig(backend="spacemouse", options={"invert_axes": [6]}))


def test_below_deadzone_is_not_intervening_and_action_stays_raw(spacemouse, hid_sdk):
    teleop = spacemouse(deadzone=0.05)
    hid_sdk.twist = (0.0, 0.0, 0.01, 0.0, 0.0, 0.0)
    sample = settled_read(teleop, lambda s: s.action[2] != 0)
    assert not sample.intervening
    assert sample.action[2] == pytest.approx(0.01)  # legacy semantics: raw twist, flag only
    hid_sdk.twist = (0.0, 0.0, 0.2, 0.0, 0.0, 0.0)
    assert settled_read(teleop, lambda s: s.action[2] > 0.1).intervening


def test_idle_sample_has_zero_action_and_open_gripper(spacemouse):
    sample = spacemouse().read()
    assert not sample.intervening and not sample.action.any() and sample.gripper == 1.0


def press(hid_sdk, teleop, buttons):
    hid_sdk.buttons = buttons
    return settled_read(teleop, lambda s: [s.buttons["button_0"], s.buttons["button_1"]] == [bool(b) for b in buttons])


def test_button_zero_toggles_sticky_gripper_and_reset_reopens(spacemouse, hid_sdk):
    teleop = spacemouse()
    sample = press(hid_sdk, teleop, (1, 0))
    assert sample.intervening  # a pressed button alone counts as activity
    assert settled_read(teleop, lambda s: s.gripper == 0.0).gripper == 0.0
    press(hid_sdk, teleop, (0, 0))
    assert teleop.read().gripper == 0.0  # sticky after release
    press(hid_sdk, teleop, (1, 0))
    assert settled_read(teleop, lambda s: s.gripper == 1.0).gripper == 1.0
    press(hid_sdk, teleop, (0, 0))
    press(hid_sdk, teleop, (1, 0))
    settled_read(teleop, lambda s: s.gripper == 0.0)
    press(hid_sdk, teleop, (0, 0))
    teleop.reset()
    assert settled_read(teleop, lambda s: not s.buttons["button_0"] and s.gripper == 1.0).gripper == 1.0


def test_held_button_toggles_only_once(spacemouse, hid_sdk):
    teleop = spacemouse()
    press(hid_sdk, teleop, (1, 0))
    time.sleep(0.05)  # many polls with the button still held
    assert teleop.read().gripper == 0.0


@pytest.mark.parametrize("button", [0, 1])
def test_a_held_button_latches_one_press_event_and_never_ends_the_episode(spacemouse, hid_sdk, button):
    teleop = spacemouse()
    name = f"button_{button}"
    hid_sdk.buttons = tuple(int(index == button) for index in range(2))
    seen = []
    for _ in range(200):
        sample = teleop.read()
        seen.append(sample.events.get(name, False))
        assert sample.events["episode_end"] is False
        if sample.buttons[name] and len(seen) > 1 and any(seen):
            break
    hid_sdk.buttons = (0, 0)
    time.sleep(0.05)
    assert seen.count(True) == 1  # the held button fires one edge
    for _ in range(2):
        sample = teleop.read()
        assert sample.events.get(name, False) is False and sample.events["episode_end"] is False


def settled_latest(teleop, predicate, attempts=400):
    """Wait, without consuming events, until the poll thread published a matching sample."""
    for _ in range(attempts):
        sample = teleop.latest()
        if sample is not None and predicate(sample):
            return sample
        time.sleep(0.001)
    raise AssertionError("mocked state never reached the sample")


def test_a_press_released_between_two_reads_is_still_latched(spacemouse, hid_sdk):
    teleop = spacemouse()
    teleop.read()
    hid_sdk.buttons = (0, 1)
    settled_latest(teleop, lambda s: s.buttons["button_1"])
    hid_sdk.buttons = (0, 0)
    settled_latest(teleop, lambda s: not s.buttons["button_1"])
    sample = teleop.read()
    assert not sample.buttons["button_1"] and sample.events["button_1"] is True
    assert sample.events.get("button_0", False) is False


def test_disconnect_closes_device_and_stops_thread(spacemouse, hid_sdk):
    teleop = spacemouse()
    teleop.read()
    device = hid_sdk.devices[0]
    assert any(t.name == f"teleop-{teleop.name}" for t in threading.enumerate())
    teleop.disconnect()
    assert device.closed
    assert not any(t.name == f"teleop-{teleop.name}" for t in threading.enumerate())


def test_reconnect_opens_a_fresh_device(spacemouse, hid_sdk):
    teleop = spacemouse()
    teleop.disconnect()
    teleop.connect()
    assert len(hid_sdk.devices) == 2 and hid_sdk.devices[0].closed
    assert not hid_sdk.devices[1].closed
    teleop.disconnect()


# -- discovery ---------------------------------------------------------------


def fake_hid(entries):
    module = types.ModuleType("hid")
    module.enumerate = lambda: entries
    return module


DISCOVER_ENTRIES = [
    {"vendor_id": 0x256F, "product_id": 0xC635, "path": b"/dev/hidraw3"},
    {"vendor_id": 0x046D, "product_id": 0xC626, "path": b"/dev/hidraw4"},
    {"vendor_id": 0x256F, "product_id": 0xC63A, "path": b"/dev/hidraw6"},
    {"vendor_id": 0x1234, "product_id": 0x0001, "path": b"/dev/hidraw5"},
]


def test_discover_matches_vendor_and_product_pairs_of_the_builtin_table(monkeypatch):
    monkeypatch.setitem(sys.modules, "hid", fake_hid(DISCOVER_ENTRIES))
    assert SpaceMouseTeleop.discover() == {"256f:c635:/dev/hidraw3", "046d:c626:/dev/hidraw4"}


def test_discover_also_matches_the_local_table(monkeypatch, tmp_path):
    from robot_infra.devices.backends.teleops._spacemouse_hid import load_device_specs

    monkeypatch.setitem(sys.modules, "hid", fake_hid(DISCOVER_ENTRIES))
    write_table(tmp_path / "site")
    expected = {"256f:c635:/dev/hidraw3", "046d:c626:/dev/hidraw4", "256f:c63a:/dev/hidraw6"}
    assert SpaceMouseTeleop.discover(table=load_device_specs(tmp_path / "site")) == expected
    write_table(tmp_path / "local")  # the default lookup without a station: ./local
    assert SpaceMouseTeleop.discover() == expected


def test_discover_ignores_logitech_receiver_and_unlisted_3dconnexion_products(monkeypatch):
    from robot_infra.devices.backends.teleops._spacemouse_hid import BUILTIN_DEVICE_SPECS

    listed = {tuple(spec.hid_id) for spec in BUILTIN_DEVICE_SPECS.values()}
    assert (0x046D, 0xC542) not in listed and (0x256F, 0xFFFF) not in listed
    entries = [
        {"vendor_id": 0x046D, "product_id": 0xC542, "path": b"/dev/hidraw0"},
        {"vendor_id": 0x256F, "product_id": 0xFFFF, "path": b"/dev/hidraw1"},
    ]
    monkeypatch.setitem(sys.modules, "hid", fake_hid(entries))
    assert SpaceMouseTeleop.discover() == set()


def test_discover_falls_back_to_hidraw_nodes_without_hidapi(monkeypatch, tmp_path):
    for name in ("hidraw0", "hidraw1"):
        (tmp_path / name).touch()
    monkeypatch.setitem(sys.modules, "hid", None)
    monkeypatch.setattr(spacemouse_backend, "HIDRAW_GLOB", str(tmp_path / "hidraw*"))
    assert SpaceMouseTeleop.discover() == {str(tmp_path / "hidraw0"), str(tmp_path / "hidraw1")}
    for path in tmp_path.iterdir():
        path.unlink()
    assert SpaceMouseTeleop.discover() == set()


def test_discover_teleops_includes_spacemouse(monkeypatch):
    monkeypatch.setitem(sys.modules, "hid", fake_hid([]))
    assert probes.discover_teleops(hardware_only=True)["spacemouse"] == set()


def test_discover_teleops_passes_backend_discover_kwargs(monkeypatch, tmp_path):
    from robot_infra.devices.backends.teleops._spacemouse_hid import load_device_specs

    monkeypatch.setitem(sys.modules, "hid", fake_hid(DISCOVER_ENTRIES))
    write_table(tmp_path / "site")
    found = probes.discover_teleops(
        hardware_only=True, discover_kwargs={"spacemouse": {"table": load_device_specs(tmp_path / "site")}})
    assert "256f:c63a:/dev/hidraw6" in found["spacemouse"]


# -- doctor os:teleop for a spacemouse ---------------------------------------

BT_ENTRY = {"vendor_id": 0x256F, "product_id": 0xC63A, "path": b"/dev/hidraw6"}
COMPACT_ENTRY = {"vendor_id": 0x256F, "product_id": 0xC635, "path": b"/dev/hidraw3"}


def run_teleop_check(monkeypatch, entries, **options):
    monkeypatch.setitem(sys.modules, "hid", fake_hid(entries))
    teleop = TeleopConfig(name="sm", backend="spacemouse", options=options)
    return checks._make_check(2, "os:teleop:sm", checks._spacemouse_outcome(teleop, "os:teleop:sm"))()


def test_teleop_check_fails_naming_an_unknown_3dconnexion_device_without_a_supported_one(monkeypatch):
    result = run_teleop_check(monkeypatch, [BT_ENTRY])
    assert not result.ok
    assert ("unknown 3Dconnexion-vendor HID device 256f:c63a; add it to spacemouse-devices.yaml "
            "(see docs/spacemouse-commissioning.md#site-device-table) (/dev/hidraw6)") in result.message
    assert "site-device-table" in result.hint


def test_teleop_check_warns_about_an_unknown_device_beside_a_supported_one(monkeypatch):
    result = run_teleop_check(monkeypatch, [COMPACT_ENTRY, BT_ENTRY])
    assert result.ok and result.warning and "256f:c635:/dev/hidraw3" in result.message
    assert "256f:c63a" in result.hint


def test_teleop_check_passes_with_the_device_in_the_local_table(monkeypatch, tmp_path):
    path = write_table(tmp_path / "tables")
    result = run_teleop_check(monkeypatch, [BT_ENTRY], device_table=str(path))
    assert result.ok and not result.warning and "256f:c63a:/dev/hidraw6" in result.message


def test_teleop_check_reports_an_invalid_local_table(monkeypatch, tmp_path):
    path = write_table(tmp_path / "tables", "schema_version: 2\ndevices: {}\n")
    result = run_teleop_check(monkeypatch, [BT_ENTRY], device_table=str(path))
    assert not result.ok and str(path) in result.message and "site-device-table" in result.hint


def test_teleop_check_without_any_device_keeps_the_plug_in_hint(monkeypatch):
    result = run_teleop_check(monkeypatch, [])
    assert not result.ok and result.message == "no 'spacemouse' teleop device found"
    assert "rwrl discover" in result.hint


# -- doctor os:hid -----------------------------------------------------------


def run_hid(pattern):
    return checks._make_check(2, "os:hid", checks._hid_outcome(pattern))()


def test_hid_check_ok_when_nodes_are_readable_and_writable(tmp_path):
    (tmp_path / "hidraw0").touch()
    result = run_hid(str(tmp_path / "hidraw*"))
    assert result.ok and "hidraw0" in result.message


def test_hid_check_reports_udev_hint_when_access_denied(tmp_path, monkeypatch):
    (tmp_path / "hidraw0").touch()
    monkeypatch.setattr(checks.os, "access", lambda path, mode: False)
    result = run_hid(str(tmp_path / "hidraw*"))
    assert not result.ok and "hidraw0" in result.message
    assert "sudo cp robot_infra/setup/99-spacemouse.rules " in result.hint
    assert "udevadm control --reload-rules" in result.hint and "plugdev" in result.hint
    assert "--device=/dev/hidrawN" in result.hint


def test_hid_check_reports_missing_devices(tmp_path):
    result = run_hid(str(tmp_path / "hidraw*"))
    assert not result.ok and "no HID device" in result.message and "USB" in result.hint


def test_os_hid_check_only_for_spacemouse_backend(monkeypatch):
    monkeypatch.setattr(checks, "_HIDRAW_GLOB", "/nonexistent/hidraw*")
    from robot_infra.devices.robots.piper import PiperConfig

    def names(backend):
        config = PiperConfig(arm_backend="fake", teleop=TeleopConfig(backend=backend))
        return {name for name, _ in checks._os_checks(config)}

    assert "os:hid" in names("spacemouse")
    assert "os:hid" not in names("fake")


def test_poll_interval_sleeps_once_per_poll(spacemouse, monkeypatch):
    sleeps = []
    monkeypatch.setattr(spacemouse_backend, "_sleep", lambda seconds: sleeps.append(seconds))
    teleop = spacemouse(poll_interval_s=0.02)
    for _ in range(3):
        teleop.read()
    teleop.disconnect()
    assert len(sleeps) >= 3 and set(sleeps) == {0.02}


def test_poll_interval_defaults_to_200hz_and_rejects_negative():
    assert SpaceMouseTeleop(TeleopConfig(backend="spacemouse"))._poll_interval_s == 0.005
    with pytest.raises(ValueError, match="poll_interval_s"):
        SpaceMouseTeleop(TeleopConfig(backend="spacemouse", options={"poll_interval_s": -1}))


def test_consumed_sample_keeps_decoded_input_metadata(hid_sdk):
    hid_sdk.twist = (.1, .2, .3, .4, .5, .6)
    teleop = SpaceMouseTeleop(TeleopConfig(backend='spacemouse'))
    teleop.connect()
    try:
        sample = teleop.read()
        assert sample.diagnostics['decoded_axes'] == list(hid_sdk.twist)
        np.testing.assert_allclose(sample.action, [-.2, .1, .3, -.4, -.5, -.6])
        assert sample.diagnostics['hid_report_count'] is None  # fake reader has no HID evidence
        sample.diagnostics['decoded_axes'][0] = 99
        assert teleop.latest().diagnostics['decoded_axes'][0] == .1
    finally:
        teleop.disconnect()


def test_shared_axis_mapping_and_claim_opt_out(hid_sdk):
    from robot_infra.devices.backends.teleops.spacemouse import map_spacemouse_axes
    from robot_infra.devices.teleop import TeleopConfig
    raw = [1, 2, 3, 4, 5, 6]
    np.testing.assert_array_equal(map_spacemouse_axes(raw, [1, 4]), [-2, -1, 3, -4, 5, -6])
    part = spacemouse_backend.SpaceMouseTeleop(TeleopConfig(backend='spacemouse'), claim=False)
    part.connect()
    try:
        assert hid_sdk.devices[0].kwargs['claim'] is False
    finally:
        part.disconnect()
