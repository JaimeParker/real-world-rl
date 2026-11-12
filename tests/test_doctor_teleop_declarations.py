"""The host doctor reads its teleop SDK, USB and serial facts from the teleop backend classes."""
from __future__ import annotations

import os
from pathlib import Path
from types import SimpleNamespace

import pytest

from real_world_rl.devices import Teleop
from real_world_rl.devices.backends.teleops.fake import FakeTeleop
from real_world_rl.doctor import host as doctor_host
from real_world_rl.doctor.host import Host
from real_world_rl.doctor.repair import Report
from real_world_rl.teleop.gello.backend import GelloTeleop
from real_world_rl.teleop.spacemouse.backend import SpaceMouseTeleop
from real_world_rl.teleop.spacemouse.hid import BUILTIN_DEVICE_SPECS

# The host tables before the backends declared their part (doctor output must not change).
SDK_TABLE = {
    "franky": ("franky", ["franky-control"]),
    "spacemouse": ("hid", ["hidapi==0.14.0.post4"]),
    "gello": ("dynamixel_sdk", ["dynamixel-sdk>=3.7.31,<3.9"]),
    "gello-events": ("evdev", ["evdev"]),
    "realsense": ("pyrealsense2", ["pyrealsense2"]),
    "grpc": ("grpc", ["grpcio>=1.60"]),
    "jpeg": ("cv2", ["pillow", "opencv-python-headless"]),
}
TREZOR_CHECK = "assert callable(getattr(m,'device',None)), 'wrong hid distribution; trezor hidapi required'"


@pytest.fixture
def host(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    monkeypatch.setenv("XDG_CONFIG_HOME", str(tmp_path / "xdg"))
    monkeypatch.setenv("RWRL_DOCTOR_REPORT_DIR", str(tmp_path / "reports"))
    return Host(tmp_path, Report(tmp_path), sysfs=tmp_path / "sys", dev=tmp_path / "dev", etc=tmp_path / "etc")


def test_sdk_checks_equal_the_former_host_tables():
    checks = doctor_host.sdk_checks()
    assert {extra: (module, requirements) for extra, (module, _, requirements) in checks.items()} == SDK_TABLE
    assert checks["spacemouse"][1] == TREZOR_CHECK
    assert {checks[extra][1] for extra in checks if extra != "spacemouse"} == {"print('import OK')"}


def test_backend_declarations():
    assert SpaceMouseTeleop.UDEV_SUBSYSTEMS == ("usb", "hidraw") and SpaceMouseTeleop.SERIAL_GROUP is None
    assert GelloTeleop.SERIAL_GROUP == "dialout" and not GelloTeleop.UDEV_SUBSYSTEMS
    assert GelloTeleop.OPTIONAL_SDKS["gello-events"].option == "event_device"
    assert FakeTeleop.usb_ids() == frozenset() and FakeTeleop.udev_rules({(1, 2)}) == []
    assert not FakeTeleop.SERIAL_GROUP and not FakeTeleop.PIP_REQUIREMENTS


def test_spacemouse_udev_rules_keep_their_lines():
    assert SpaceMouseTeleop.udev_rules({(0x256F, 0xC635)}) == [
        'SUBSYSTEM=="usb", ATTRS{idVendor}=="256f", ATTRS{idProduct}=="c635", MODE="0660", GROUP="plugdev"',
        'SUBSYSTEM=="hidraw", ATTRS{idVendor}=="256f", ATTRS{idProduct}=="c635", MODE="0660", GROUP="plugdev"',
    ]


def test_host_takes_the_usb_ids_from_the_backends(host):
    backend, ids = host.teleop_usb["spacemouse"]
    assert backend is SpaceMouseTeleop
    assert ids == {tuple(spec.hid_id) for spec in BUILTIN_DEVICE_SPECS.values()}
    assert set(host.teleop_usb) == {name for name, cls in Teleop.backends().items() if cls.UDEV_SUBSYSTEMS}


def test_a_serial_teleop_uses_the_group_its_backend_declares(host, monkeypatch, tmp_path):
    class SerialPad(FakeTeleop):
        SERIAL_GROUP = "rwrl-test-serial"

    monkeypatch.setitem(Teleop._BACKENDS, "serial-pad", SerialPad)
    port = tmp_path / "ttyUSB0"
    port.touch()
    real_access = os.access
    monkeypatch.setattr(os, "access", lambda path, mode, **kw: Path(path) != port.resolve() and real_access(path, mode, **kw))
    monkeypatch.setattr(host, "command", lambda *a, **kw: (1, ""))
    host.inspect_gello([SimpleNamespace(teleop=SimpleNamespace(backend="serial-pad", options={"port": str(port)}))])
    assert "serial-pad" in host.expected
    assert next(f for f in host.findings if f.id == "gello.0").status == "FAIL"
    assert any(action.id == "member:rwrl-test-serial" for action in host.actions)


def test_a_teleop_without_a_serial_group_is_not_a_serial_candidate(host):
    host.inspect_gello([SimpleNamespace(teleop=SimpleNamespace(backend="spacemouse", options={"port": "/dev/x"}))])
    assert not any(f.id.startswith("gello.") and f.id != "gello.candidates" for f in host.findings)
    assert "spacemouse" not in host.expected
