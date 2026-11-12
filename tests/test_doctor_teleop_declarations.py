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

# The host tables before the backends declared their part (doctor output must not change), plus
# the pico extra its backend declares since the Pico became a device-layer part.
SDK_TABLE = {
    "franky": ("franky", ["franky-control"]),
    "spacemouse": ("hid", ["hidapi==0.14.0.post4"]),
    "gello": ("dynamixel_sdk", ["dynamixel-sdk>=3.7.31,<3.9"]),
    "gello-events": ("evdev", ["evdev"]),
    "realsense": ("pyrealsense2", ["pyrealsense2"]),
    "grpc": ("grpc", ["grpcio>=1.60"]),
    "jpeg": ("cv2", ["pillow", "opencv-python-headless"]),
    "pico": ("zmq", ["pyzmq"]),
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


def test_new_declaration_defaults_and_overrides():
    assert FakeTeleop.site_device_table() is None
    assert FakeTeleop.unknown_device_finding(1, 2, Path('/unused')) is None
    assert FakeTeleop.EVENT_DEVICE_GROUP is None
    assert FakeTeleop.EVENT_DEVICE_OPTION == 'event_device'
    assert FakeTeleop.DEVICE_TABLE_ERROR_HINT == FakeTeleop.DEVICE_TABLE_DOC == ''
    assert FakeTeleop.HID_PERMISSION_HINT == ''
    assert FakeTeleop.PRESET_ALIASES == ()
    assert FakeTeleop.DOCTOR_COVERAGE is None and FakeTeleop.USB_PROBE_NAME is None
    assert GelloTeleop.site_device_table() is None
    assert GelloTeleop.EVENT_DEVICE_GROUP == 'input'
    assert GelloTeleop.PRESET_ALIASES == ('leader_arm', 'leader_cartesian')
    assert GelloTeleop.DOCTOR_COVERAGE == (2, 'GELLO')
    assert SpaceMouseTeleop.DOCTOR_COVERAGE == (1, 'SpaceMouse')
    assert SpaceMouseTeleop.USB_PROBE_NAME == 'doctor-space'
    assert SpaceMouseTeleop.DEVICE_TABLE_DOC.endswith('#site-device-table')
    assert SpaceMouseTeleop.HID_PERMISSION_HINT


def test_backend_unknown_finding_uses_the_supplied_table(tmp_path):
    from real_world_rl.teleop.spacemouse import hid

    table = hid.builtin_device_table()
    assert SpaceMouseTeleop.unknown_device_finding(0x256F, 0xC635, tmp_path, table=table) is None
    assert SpaceMouseTeleop.unknown_device_finding(0x256F, 0xC6FF, tmp_path) is None
    message, hint = SpaceMouseTeleop.unknown_device_finding(0x256F, 0xC6FF, tmp_path, table=table)
    assert message == hid.unknown_device_message(0x256F, 0xC6FF)
    assert 'local/spacemouse-devices.yaml' in hint
    assert SpaceMouseTeleop.unknown_device_finding(0x046D, 0xC6FF, tmp_path, table=table) is None
    interface = tmp_path / 'interface'
    interface.mkdir()
    (interface / 'report_descriptor').write_bytes(bytes([0x05, 0x01, 0x09, 0x08]))
    assert SpaceMouseTeleop.unknown_device_finding(
        0x046D, 0xC6FF, tmp_path, table=table, hid_interfaces=[interface]) is not None


def test_host_delegates_tables_options_unknown_devices_and_errors(tmp_path, monkeypatch):
    calls = []
    table = object()

    class TablePad(FakeTeleop):
        UDEV_SUBSYSTEMS = ('hidraw',)
        USB_IDS_SOURCE = 'test device table'
        DEVICE_TABLE_ERROR_HINT = 'repair test table'

        @classmethod
        def site_device_table(cls, station_dir=None, options=None):
            calls.append((station_dir, options))
            if options and options.get('broken'):
                raise ValueError('bad table')
            return table

        @classmethod
        def usb_ids(cls, station_dir=None, options=None):
            return frozenset()

        @classmethod
        def unknown_device_finding(cls, vid, pid, path, **kwargs):
            assert kwargs['table'] is table
            return 'unknown pad', 'add test entry'

    monkeypatch.setattr(doctor_host, 'teleop_backends', lambda: {'table-pad': TablePad})
    usb = tmp_path / 'sys/bus/usb/devices/1-1'
    usb.mkdir(parents=True)
    for name, value in {'idVendor': '1234', 'idProduct': '5678', 'product': 'Pad'}.items():
        (usb / name).write_text(value)
    monkeypatch.setenv('RWRL_DOCTOR_REPORT_DIR', str(tmp_path / 'reports'))
    kwargs = dict(sysfs=tmp_path / 'sys', dev=tmp_path / 'dev', etc=tmp_path / 'etc', station_dir=tmp_path)
    host = Host(tmp_path, Report(tmp_path), teleop_options={'table-pad': {'selection': 3}}, **kwargs)
    assert calls == [(tmp_path, {'selection': 3})]
    assert host.teleop_tables == {'table-pad': table}
    host.inspect_usb()
    assert [(f.id, f.status, f.message, f.hint) for f in host.findings] == [
        ('usb.table-pad.unknown.1234:5678', 'WARN', 'unknown pad', 'product "Pad"; add test entry')]
    failed = Host(tmp_path, Report(tmp_path), teleop_options={'table-pad': {'broken': True}}, **kwargs)
    assert not failed.teleop_tables
    assert [(f.id, f.status, f.message, f.hint) for f in failed.findings] == [
        ('usb.table-pad.table', 'FAIL', 'cannot load the test device table: bad table', 'repair test table')]


def test_os_checks_use_custom_hid_and_serial_declarations(monkeypatch):
    from real_world_rl.devices import TeleopConfig
    from real_world_rl.devices.robots.franka import FrankaConfig
    from real_world_rl.doctor import checks

    class CustomPad(FakeTeleop):
        SDK = 'os'
        UDEV_SUBSYSTEMS = ('hidraw',)
        SERIAL_GROUP = 'pad-serial'
        EVENT_DEVICE_GROUP = 'pad-events'
        EVENT_DEVICE_OPTION = 'buttons'

    monkeypatch.setitem(Teleop._BACKENDS, 'custom-pad', CustomPad)
    seen = []
    monkeypatch.setattr(checks, '_group_membership_result', lambda groups: seen.append(groups) or checks.CheckResult(2, 'os:groups', True, 'ok'))
    config = FrankaConfig(arm_backend='fake', gripper=None, teleop=TeleopConfig(
        backend='custom-pad', options={'port': '/unused/tty', 'buttons': '/unused/event'}))
    named = dict(checks._os_checks(config))
    assert {'os:hid', 'os:serial', 'os:groups', 'os:brltty', 'os:latency_timer'} <= named.keys()
    named['os:groups']()
    assert seen == [{'pad-serial': '/unused/tty', 'pad-events': '/unused/event'}]


def test_dependencies_follow_late_registered_backend_declarations(monkeypatch):
    from real_world_rl.devices import TeleopConfig
    from real_world_rl.devices.teleop import OptionalSdk
    from real_world_rl.devices.robots.franka import FrankaConfig
    from real_world_rl.doctor import deps

    class CustomPad(FakeTeleop):
        SDK = 'pad_sdk'
        PIP_REQUIREMENTS = ('pad-sdk',)
        OPTIONAL_SDKS = {'pad-buttons': OptionalSdk('buttons', 'pad_events', ('pad-events',))}

    monkeypatch.setitem(Teleop._BACKENDS, 'custom-pad', CustomPad)
    config = FrankaConfig(arm_backend='fake', gripper=None, teleop=TeleopConfig(backend='custom-pad'))
    assert deps.required_extras(config) == ['custom-pad']
    config.teleop.options['buttons'] = '/unused/event'
    assert deps.required_extras(config, serves_daemon=True) == ['custom-pad', 'pad-buttons', 'grpc']
    monkeypatch.setattr(deps.importlib.util, 'find_spec', lambda name: object() if name == 'pad_events' else None)
    assert not deps.extra_installed('custom-pad')
    assert deps.extra_installed('pad-buttons')


def test_table_inputs_collect_every_backend_and_keep_first_options(tmp_path, monkeypatch):
    from real_world_rl.doctor import report
    from real_world_rl import station
    from real_world_rl.discovery import autoconfig

    robots = {
        name: SimpleNamespace(config=SimpleNamespace(teleop=SimpleNamespace(backend=backend, options=options)))
        for name, backend, options in (
            ('first', 'table-pad', {'selection': 1}),
            ('second', 'serial-pad', {'port': '/unused/tty'}),
            ('third', 'table-pad', {'selection': 2}),
        )
    }
    monkeypatch.setattr(report, 'station_configured', lambda args: True)
    monkeypatch.setattr(station, 'load_station', lambda path: SimpleNamespace(robots=robots))
    monkeypatch.setattr(autoconfig, 'resolve_station_config_path', lambda path: tmp_path / 'station.yaml')
    args = SimpleNamespace(config='unused', robot=None)
    assert report.teleop_table_inputs(args) == (
        tmp_path, {'table-pad': {'selection': 1}, 'serial-pad': {'port': '/unused/tty'}})
    args.robot = 'third'
    assert report.teleop_table_inputs(args) == (tmp_path, {'table-pad': {'selection': 2}})
