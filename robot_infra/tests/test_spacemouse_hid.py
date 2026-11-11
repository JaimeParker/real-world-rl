"""In-tree HID decoder and trezor transport contracts, without hardware or hidapi."""
from __future__ import annotations

from collections import deque
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
    device = reader._HidReader(handle, reader.device_specs[model])
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


def test_unverified_wireless_bt_id_is_not_supported(monkeypatch):
    assert (0x256F, 0xC63A) not in {tuple(spec.hid_id) for spec in reader.device_specs.values()}
    fake_hid(monkeypatch, [
        {"vendor_id": 0x256F, "product_id": 0xC63A, "path": b"unverified"},
    ], Handle())
    with pytest.raises(DeviceNotFound, match="no supported SpaceMouse"):
        reader.open()


def test_split_reports_keep_other_axes():
    handle = Handle([words(1, 35, -70, -105), words(2, -140, -175, 210), words(1, 350, 0, 0)])
    device = reader._HidReader(handle, reader.device_specs["SpaceNavigator"])
    assert device.read()[:6] == pytest.approx((0.1, 0.2, 0.3, 0, 0, 0))
    assert device.read()[:6] == pytest.approx((0.1, 0.2, 0.3, 0.5, 0.4, 0.6))
    assert device.read()[:6] == pytest.approx((1, 0, 0, 0.5, 0.4, 0.6))


def test_empty_report_preserves_last_state():
    handle = Handle([words(1, -350, 350, 175), [3, 3]])
    device = reader._HidReader(handle, reader.device_specs["SpaceMouse Compact"])
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
    device = reader._HidReader(handle, reader.device_specs["SpaceNavigator"])
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


@pytest.mark.parametrize("model", list(reader.device_specs))
def test_every_model_decodes_combined_or_split_axes_and_all_button_bits(model):
    spec = reader.device_specs[model]
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
    device = reader._HidReader(handle, reader.device_specs["SpaceNavigator"])
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
