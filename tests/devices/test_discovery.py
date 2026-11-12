"""Hardware-free tests for discovery probes and env-based auto-fill."""
from __future__ import annotations

import pytest
import yaml

import real_world_rl.discovery.report as discovery_cli
from real_world_rl.devices import CameraConfig
from real_world_rl.devices.backends.cameras.fake import FakeCamera
from real_world_rl.devices.backends.cameras.realsense import RealSenseCamera
from real_world_rl.devices.robots.franka import FrankaConfig
from real_world_rl.devices.robots.piper import PiperConfig
from real_world_rl.discovery import enumerate as enum
from real_world_rl.discovery.autoconfig import fill_from_env


@pytest.fixture
def probed_hosts(monkeypatch):
    calls: list[list[str]] = []
    monkeypatch.setattr(
        discovery_cli, "discover_all",
        lambda host_ips, spacemouse_table=None: calls.append(list(host_ips)) or enum.DiscoveryReport(),
    )
    return calls


def test_cli_without_config_probes_every_robot_of_the_station(tmp_path, monkeypatch, probed_hosts, capsys):
    robots = {
        name: {"type": "franka", "arm_backend": "fake", "robot_ip": ip}
        for name, ip in (("left", "10.0.0.1"), ("right", "10.0.0.2"), ("spare", "10.0.0.1"))
    }
    path = tmp_path / "station.yaml"
    path.write_text(yaml.safe_dump({"robots": robots}))
    monkeypatch.setenv("RWRL_STATION", str(path))
    assert discovery_cli.main([]) == 0
    assert probed_hosts == [["10.0.0.1", "10.0.0.2"]]
    assert "hosts:" in capsys.readouterr().out


def test_cli_without_config_or_station_probes_no_hosts(tmp_path, monkeypatch, probed_hosts):
    monkeypatch.delenv("RWRL_STATION", raising=False)
    monkeypatch.setenv("XDG_CONFIG_HOME", str(tmp_path / "xdg"))
    monkeypatch.chdir(tmp_path)
    assert discovery_cli.main([]) == 0
    assert probed_hosts == [[]]


def test_cli_with_a_single_robot_config(tmp_path, probed_hosts):
    path = tmp_path / "robot.yaml"
    path.write_text(yaml.safe_dump({"type": "franka", "arm_backend": "fake", "robot_ip": "10.0.0.7"}))
    assert discovery_cli.main(["--station", str(path)]) == 0
    assert probed_hosts == [["10.0.0.7"]]


def write_iface(root, name, iface_type, operstate):
    iface = root / name
    iface.mkdir()
    (iface / "type").write_text(f"{iface_type}\n")
    (iface / "operstate").write_text(f"{operstate}\n")


def test_list_can_interfaces_keeps_only_can(tmp_path, monkeypatch):
    write_iface(tmp_path, "can0", 280, "up")
    write_iface(tmp_path, "eth0", 1, "up")
    monkeypatch.setattr(enum, "can_bitrate", lambda name: 1_000_000)
    interfaces = enum.list_can_interfaces(tmp_path)
    assert interfaces == {"can0": enum.CanInfo("can0", "up", 1_000_000)}


def test_list_can_interfaces_missing_root(tmp_path):
    assert enum.list_can_interfaces(tmp_path / "absent") == {}


def test_ping_without_binary_is_false(monkeypatch):
    def missing(*args, **kwargs):
        raise FileNotFoundError

    monkeypatch.setattr(enum.subprocess, "run", missing)
    assert enum.ping("10.0.0.1") is False
    assert enum.can_bitrate("can0") is None


def test_discover_all_and_format(monkeypatch):
    monkeypatch.setattr(enum, "ping", lambda ip, timeout_s=1.0: True)
    monkeypatch.setattr(enum, "tcp_open", lambda ip, port, timeout_s=0.5: port == 1337)
    monkeypatch.setattr(
        enum, "list_can_interfaces", lambda: {"can7": enum.CanInfo("can7", "up", None)}
    )
    monkeypatch.setattr(RealSenseCamera, "discover", classmethod(lambda cls: {"RS123"}))
    report = enum.discover_all(["172.16.0.2"])
    assert "fake" not in report.cameras
    assert report.cameras["realsense"] == {"RS123"}
    assert report.hosts["172.16.0.2"] == enum.HostInfo("172.16.0.2", True, [1337])
    text = report.format()
    assert "can7" in text and "172.16.0.2" in text and "RS123" in text
    assert "FAKE0" not in text and "fake" not in text


def test_empty_report_formats_none_lines():
    assert enum.DiscoveryReport().format().count("none") == 5


UNKNOWN_BT = {
    "vid": "256f", "pid": "c63a", "product": "", "path": "/dev/hidraw6",
    "message": "unknown 3Dconnexion-vendor HID device 256f:c63a; add it to spacemouse-devices.yaml "
               "(see docs/spacemouse-commissioning.md#site-device-table)",
}


def test_unknown_hid_devices_are_formatted_as_warnings_only_when_present():
    text = enum.DiscoveryReport(unknown_hid=[UNKNOWN_BT]).format()
    assert text.endswith("warnings:\n  WARN " + UNKNOWN_BT["message"] + " (/dev/hidraw6)")
    assert "warnings:" not in enum.DiscoveryReport().format()


def test_discover_all_matches_hid_devices_against_the_given_table(monkeypatch):
    from real_world_rl.devices.backends.teleops import _spacemouse_hid

    monkeypatch.setattr(enum, "list_can_interfaces", lambda: {})
    table = _spacemouse_hid.builtin_device_table()
    seen = []
    monkeypatch.setattr(_spacemouse_hid, "unknown_hid_devices", lambda given: seen.append(given) or [UNKNOWN_BT])
    report = enum.discover_all([], spacemouse_table=table)
    assert seen[0] is table and report.unknown_hid == [UNKNOWN_BT]


def test_cli_matches_hid_devices_against_the_station_device_table(tmp_path, monkeypatch):
    received = []
    monkeypatch.setattr(
        discovery_cli, "discover_all",
        lambda host_ips, spacemouse_table=None: received.append(spacemouse_table) or enum.DiscoveryReport(),
    )
    path = tmp_path / "station.yaml"
    path.write_text(yaml.safe_dump({"type": "franka", "arm_backend": "fake", "teleop": {"backend": "spacemouse"}}))
    (tmp_path / "spacemouse-devices.yaml").write_text(
        "schema_version: 1\ndevices:\n  BT: {hid_id: [0x256F, 0xC63A], layout: SpaceMouse Wireless}\n")
    assert discovery_cli.main(["--station", str(path)]) == 0
    assert (0x256F, 0xC63A) in received[0].specs and received[0].path == (tmp_path / "spacemouse-devices.yaml").resolve()


def test_discover_teleops_and_format():
    assert enum.discover_teleops()["fake"] == {"fake"}
    assert "fake" not in enum.discover_teleops(hardware_only=True)
    text = enum.DiscoveryReport(teleops={"spacemouse": {"hid-0"}}).format()
    assert "teleops:\n  spacemouse: hid-0" in text


def test_camera_discover_exception_gives_empty_set(monkeypatch):
    def boom(cls):
        raise RuntimeError("usb")

    monkeypatch.setattr(FakeCamera, "discover", classmethod(boom))
    assert enum.discover_cameras()["fake"] == set()


def test_hosts_from_config():
    assert enum.hosts_from_config(FrankaConfig(robot_ip="1.2.3.4")) == ["1.2.3.4"]
    assert enum.hosts_from_config(PiperConfig()) == []


def test_fill_from_env_casts_and_fills_only_empty():
    franka = fill_from_env(
        FrankaConfig(arm_backend="fake", robot_ip=""), {"ROBOT_IP": "10.0.0.5"}
    )
    assert franka.robot_ip == "10.0.0.5"
    kept = fill_from_env(FrankaConfig(robot_ip="1.1.1.1"), {"ROBOT_IP": "10.0.0.5"})
    assert kept.robot_ip == "1.1.1.1"
    piper = fill_from_env(PiperConfig(can_interface=""), {"CAN_INTERFACE": "can3", "BITRATE": "5"})
    assert piper.can_interface == "can3" and piper.bitrate == 1_000_000
    assert piper.arm_backend == "pyagxarm"


@pytest.mark.parametrize(
    "serials, expected", [({"ONLY"}, "ONLY"), ({"A", "B"}, "")], ids=["single", "ambiguous"]
)
def test_fill_from_env_camera_serial(monkeypatch, serials, expected):
    monkeypatch.setattr(FakeCamera, "discover", classmethod(lambda cls: set(serials)))
    config = FrankaConfig(
        arm_backend="fake", cameras=[CameraConfig(name="wrist", backend="fake")]
    )
    assert fill_from_env(config, {}).cameras[0].serial == expected
