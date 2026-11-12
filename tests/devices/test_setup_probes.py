"""Setup wizard probes against monkeypatched host probes and fake SDK modules (no hardware)."""
from __future__ import annotations

import sys
import types

import numpy as np
import pytest

from real_world_rl.devices.backends.cameras.realsense import RealSenseCamera
from real_world_rl.devices.backends.teleops import gello_calibrate
from real_world_rl.devices.backends.teleops.gello import DynamixelBus, GelloTeleop
from real_world_rl.devices.backends.teleops.spacemouse import SpaceMouseTeleop
from real_world_rl.discovery import enumerate as host_probes
from real_world_rl.setup import probes

GELLO_PORT = "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FAKE-if00-port0"
DEAD_PORT = "/dev/ttyUSB9"


# -- franka hosts ---------------------------------------------------------------------------


def test_franka_candidates_defaults_env_and_flags_without_duplicates():
    candidates = probes.franka_candidates(["10.0.0.9", "172.16.0.2"], env={"ROBOT_IP": "10.0.0.8"})
    assert candidates == ["172.16.0.2", "172.16.1.2", "10.0.0.8", "10.0.0.9"]
    assert probes.franka_candidates(env={}) == list(probes.DEFAULT_FRANKA_IPS)


def test_subnet_hosts_and_the_cap():
    assert probes.subnet_hosts("10.0.0.0/30") == ["10.0.0.1", "10.0.0.2"]
    assert len(probes.subnet_hosts("10.0.0.0/24")) == 254
    with pytest.raises(ValueError, match="more than 256"):
        probes.subnet_hosts("10.0.0.0/23")
    with pytest.raises(ValueError):
        probes.subnet_hosts("not-a-subnet")


def test_probe_franka_reports_open_ports_per_candidate(monkeypatch):
    open_pairs = {("172.16.0.2", 22), ("172.16.0.2", 1337)}
    monkeypatch.setattr(host_probes, "tcp_open", lambda ip, port, timeout_s=0.5: (ip, port) in open_pairs)
    hosts = probes.probe_franka(["172.16.0.2", "172.16.1.2"])
    assert hosts == [probes.HostProbe("172.16.0.2", [22, 1337]), probes.HostProbe("172.16.1.2", [])]
    assert hosts[0].reachable and not hosts[1].reachable


def test_open_ports_by_host_keeps_order_and_handles_no_hosts(monkeypatch):
    monkeypatch.setattr(host_probes, "tcp_open", lambda ip, port, timeout_s=0.5: port == 443)
    assert host_probes.open_ports_by_host(["b", "a", "b"], (22, 443)) == {"b": [443], "a": [443]}
    assert host_probes.open_ports_by_host([], (22,)) == {}


# -- cameras ------------------------------------------------------------------------------------


def fake_realsense(devices: dict[str, str]) -> types.ModuleType:
    rs = types.ModuleType("pyrealsense2")
    rs.camera_info = types.SimpleNamespace(serial_number="serial", product_line="line")

    class Device:
        def __init__(self, serial, line):
            self.info = {"serial": serial, "line": line}

        def get_info(self, key):
            return self.info[key]

    rs.context = lambda: types.SimpleNamespace(
        query_devices=lambda: [Device(serial, line) for serial, line in devices.items()]
    )
    return rs


def test_probe_realsense_with_the_sdk(monkeypatch):
    monkeypatch.setitem(sys.modules, "pyrealsense2", fake_realsense({"B2": "D400", "A1": "L500"}))
    assert probes.probe_realsense() == [probes.CameraProbe("A1", "L500"), probes.CameraProbe("B2", "D400")]


def test_probe_realsense_without_the_sdk_or_on_errors(monkeypatch):
    monkeypatch.setitem(sys.modules, "pyrealsense2", None)
    assert probes.probe_realsense() == []

    def boom(cls):
        raise RuntimeError("usb")

    monkeypatch.setattr(RealSenseCamera, "discover", classmethod(boom))
    assert probes.probe_realsense() == []


# -- gello ----------------------------------------------------------------------------------------


class FakePortHandler:
    instances: list["FakePortHandler"] = []

    def __init__(self, port):
        self.port, self.baudrate, self.closed = port, None, False
        FakePortHandler.instances.append(self)

    def openPort(self):
        return self.port != DEAD_PORT

    def setBaudRate(self, baudrate):
        self.baudrate = baudrate
        return baudrate != 2_000_000  # an adapter that refuses one rate

    def closePort(self):
        self.closed = True


class FakePacketHandler:
    """Servos 1-3 answer at 1 Mbaud only; the reply tail is reported as a timeout."""

    def __init__(self, protocol_version):
        self.protocol_version = protocol_version

    def broadcastPing(self, port_handler):
        if port_handler.baudrate == 1_000_000:
            return {3: [1200, 52], 1: [1200, 52], 2: [1060, 50]}, -3001
        return {}, -3001

    def write1ByteTxRx(self, *args):
        raise AssertionError("a probe must not write to the servos")


@pytest.fixture
def fake_dynamixel(monkeypatch):
    FakePortHandler.instances = []
    sdk = types.ModuleType("dynamixel_sdk")
    sdk.PortHandler, sdk.PacketHandler, sdk.COMM_SUCCESS = FakePortHandler, FakePacketHandler, 0
    monkeypatch.setitem(sys.modules, "dynamixel_sdk", sdk)
    monkeypatch.setattr(GelloTeleop, "discover", classmethod(lambda cls: {GELLO_PORT}))
    return sdk


def test_probe_gello_ports_scans_baud_rates_until_servos_answer(fake_dynamixel):
    buses = probes.probe_gello_ports([DEAD_PORT])
    assert [bus.port for bus in buses] == [GELLO_PORT, DEAD_PORT]
    found, dead = buses
    assert (found.baudrate, found.servo_ids, found.model_numbers) == (1_000_000, [1, 2, 3], [1200, 1060, 1200])
    assert dead.baudrate is None and dead.servo_ids == [] and "cannot open" in dead.note
    tried = [handler.baudrate for handler in FakePortHandler.instances if handler.port == GELLO_PORT]
    assert tried == [57600, 1_000_000]
    assert all(handler.closed for handler in FakePortHandler.instances if handler.baudrate is not None)


def test_probe_gello_ports_reports_a_refused_baud_rate_and_silent_servos(fake_dynamixel):
    (bus,) = probes.probe_gello_ports(baudrates=(57600, 2_000_000))
    assert bus.servo_ids == [] and "cannot set baud rate 2000000" in bus.note


def test_probe_gello_ports_without_dynamixel_sdk_lists_ports_with_a_note(monkeypatch):
    monkeypatch.setitem(sys.modules, "dynamixel_sdk", None)
    monkeypatch.setattr(GelloTeleop, "discover", classmethod(lambda cls: {GELLO_PORT}))
    (bus,) = probes.probe_gello_ports()
    assert bus.port == GELLO_PORT and bus.servo_ids == [] and "dynamixel-sdk" in bus.note


def test_broadcast_ping_refuses_an_open_bus(fake_dynamixel):
    bus = DynamixelBus(fake_dynamixel, GELLO_PORT, 1_000_000, [], device="test")
    assert bus.broadcast_ping() == {1: 1200, 2: 1060, 3: 1200} and not bus.is_open
    bus._port_handler = object()
    with pytest.raises(RuntimeError, match="closed bus"):
        bus.broadcast_ping()


# -- spacemouse, station, mock -------------------------------------------------------------


def test_probe_spacemouse(monkeypatch):
    monkeypatch.setattr(SpaceMouseTeleop, "discover", classmethod(lambda cls: {"256f:c635:/dev/hidraw3"}))
    assert probes.probe_spacemouse() == ["256f:c635:/dev/hidraw3"]

    def boom(cls):
        raise OSError("hid")

    monkeypatch.setattr(SpaceMouseTeleop, "discover", classmethod(boom))
    assert probes.probe_spacemouse() == []


def test_discover_station_with_a_subnet_scan(monkeypatch):
    monkeypatch.setattr(host_probes, "tcp_open", lambda ip, port, timeout_s=0.5: ip == "10.0.0.2" and port == 22)
    monkeypatch.setitem(sys.modules, "pyrealsense2", None)
    monkeypatch.setitem(sys.modules, "dynamixel_sdk", None)
    monkeypatch.setattr(GelloTeleop, "discover", classmethod(lambda cls: set()))
    monkeypatch.setattr(SpaceMouseTeleop, "discover", classmethod(lambda cls: set()))
    probe = probes.discover_station(probes.ProbeOptions(scan_subnet="10.0.0.0/29", env={}))
    assert [host.ip for host in probe.franka_hosts] == ["172.16.0.2", "172.16.1.2", "10.0.0.2"]
    assert [host.ip for host in probe.reachable_hosts] == ["10.0.0.2"]
    assert probe.cameras == [] and probe.gello_buses == [] and probe.spacemice == []
    text = probe.format()
    assert "10.0.0.2: open ports 22" in text and "no answer on TCP 22/443/1337: 172.16.0.2, 172.16.1.2" in text


def test_mock_probe_and_its_report():
    probe = probes.mock_station_probe()
    assert probe.mock and [host.ip for host in probe.reachable_hosts] == list(probes.MOCK_FRANKA_IPS)
    assert len(probe.cameras) == 3 and len(probe.gello_buses) == 2
    assert all(bus.servo_ids == list(probes.MOCK_GELLO_SERVO_IDS) for bus in probe.gello_buses)
    text = probe.format()
    assert "--mock" in text and "servo ids [1, 2, 3, 4, 5, 6, 7, 8]" in text and "spacemouse devices:\n  none" in text


def test_simulated_bus_calibrates_to_non_zero_offsets():
    bus = probes.SimulatedGelloBus(gello_calibrate.FRANKA_CALIBRATION_POSE, has_gripper=True)
    options = gello_calibrate.calibrate_gello(
        port="mock", joint_ids=[1, 2, 3, 4, 5, 6, 7], gripper_id=8, skip_sign_check=True,
        joint_limits=[[-1.0, 1.0]] * 7, ask=lambda prompt: "", echo=lambda line: None, bus=bus,
    )
    np.testing.assert_allclose(options["joint_offsets_rad"], probes.SimulatedGelloBus.HORN_OFFSETS_RAD, atol=1e-4)
    assert {options["gripper_open_rad"], options["gripper_closed_rad"]} == set(probes.SimulatedGelloBus.TRIGGER_ANGLES_RAD)
    assert not bus.is_open
