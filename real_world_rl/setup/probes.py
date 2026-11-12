"""Read-only hardware probes for the setup wizard. Each returns plain dataclasses and never raises
because a vendor SDK is missing; ``mock_station_probe`` is the canned result of ``--mock`` and
``SimulatedGelloBus`` stands in for the GELLO bus during a ``--mock`` calibration."""
from __future__ import annotations

import importlib
import ipaddress
import itertools
import os
from dataclasses import dataclass, field
from typing import Iterable, Mapping, Sequence

import numpy as np

from real_world_rl.devices import Camera, Teleop
from real_world_rl.devices.backends.teleops.gello import (
    DYNAMIXEL_SDK_REQUIREMENT,
    DynamixelBus,
    GelloTeleop,
)
from real_world_rl.discovery import enumerate as host_probes

DEFAULT_FRANKA_IPS = ("172.16.0.2", "172.16.1.2")
FRANKA_PROBE_PORTS = (22, 443, 1337)
GELLO_BAUDRATES = (57600, 1_000_000, 2_000_000, 3_000_000, 4_000_000)
MAX_SUBNET_HOSTS = 256
SUBNET_PROBE_TIMEOUT_S = 0.2
MOCK_FRANKA_IPS = DEFAULT_FRANKA_IPS
MOCK_CAMERA_SERIALS = ("123422070001", "123422070002", "123422070003")
MOCK_GELLO_PORTS = (
    "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_MOCK0-if00-port0",
    "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_MOCK1-if00-port0",
)
MOCK_GELLO_SERVO_IDS = (1, 2, 3, 4, 5, 6, 7, 8)
_XL330_MODEL_NUMBER = 1200
_REALSENSE_SDK = "pyrealsense2"


@dataclass
class HostProbe:
    ip: str
    open_ports: list[int] = field(default_factory=list)

    @property
    def reachable(self) -> bool:
        return bool(self.open_ports)


@dataclass
class CameraProbe:
    serial: str
    product_line: str = ""


@dataclass
class SerialBusProbe:
    """A serial port and the Dynamixel servos that answered a broadcast ping on it."""

    port: str
    baudrate: int | None = None
    servo_ids: list[int] = field(default_factory=list)
    model_numbers: list[int] = field(default_factory=list)
    note: str = ""


@dataclass
class ProbeOptions:
    robot_ips: list[str] = field(default_factory=list)
    scan_subnet: str | None = None
    serial_ports: list[str] = field(default_factory=list)
    timeout_s: float = 0.5
    env: Mapping[str, str] | None = None


@dataclass
class StationProbe:
    franka_hosts: list[HostProbe] = field(default_factory=list)
    cameras: list[CameraProbe] = field(default_factory=list)
    gello_buses: list[SerialBusProbe] = field(default_factory=list)
    spacemice: list[str] = field(default_factory=list)
    mock: bool = False

    @property
    def reachable_hosts(self) -> list[HostProbe]:
        return [host for host in self.franka_hosts if host.reachable]

    def format(self) -> str:
        lines = ["hardware probe" + (" (--mock: canned results)" if self.mock else "") + ":", "franka hosts:"]
        reachable = self.reachable_hosts
        lines.extend(
            f"  {host.ip}: open ports {', '.join(str(port) for port in host.open_ports)}" for host in reachable
        )
        silent = [host.ip for host in self.franka_hosts if not host.reachable]
        if silent:
            lines.append(f"  no answer on TCP {'/'.join(map(str, FRANKA_PROBE_PORTS))}: {', '.join(silent)}")
        if not self.franka_hosts:
            lines.append("  none probed")
        lines.append("realsense cameras:")
        lines.extend(
            f"  {camera.serial}" + (f" ({camera.product_line})" if camera.product_line else "")
            for camera in self.cameras
        )
        if not self.cameras:
            lines.append("  none")
        lines.append("gello / dynamixel buses:")
        for bus in self.gello_buses:
            if bus.servo_ids:
                lines.append(f"  {bus.port}: {bus.baudrate} baud, servo ids {bus.servo_ids}")
            else:
                lines.append(f"  {bus.port}: no servo answered" + (f" ({bus.note})" if bus.note else ""))
        if not self.gello_buses:
            lines.append("  none")
        lines.append("spacemouse devices:")
        lines.extend(f"  {device}" for device in self.spacemice or ["none"])
        return "\n".join(lines)


# -- franka hosts ------------------------------------------------------------------


def franka_candidates(robot_ips: Iterable[str] = (), env: Mapping[str, str] | None = None) -> list[str]:
    """The default FCI addresses, ``$ROBOT_IP`` and the given IPs, without duplicates."""
    env = os.environ if env is None else env
    from_env = [env["ROBOT_IP"]] if env.get("ROBOT_IP") else []
    return list(dict.fromkeys([*DEFAULT_FRANKA_IPS, *from_env, *robot_ips]))


def subnet_hosts(cidr: str) -> list[str]:
    """Every host address of ``cidr``; at most ``MAX_SUBNET_HOSTS``."""
    network = ipaddress.ip_network(cidr, strict=False)
    hosts = [str(ip) for ip in itertools.islice(network.hosts(), MAX_SUBNET_HOSTS + 1)]
    if len(hosts) > MAX_SUBNET_HOSTS:
        raise ValueError(f"subnet {cidr} has more than {MAX_SUBNET_HOSTS} hosts; scan a /24 or smaller")
    return hosts


def probe_franka(candidates: Sequence[str], timeout_s: float = 0.5) -> list[HostProbe]:
    """TCP 22/443/1337 of every candidate (never connects a robot), in candidate order."""
    open_ports = host_probes.open_ports_by_host(candidates, FRANKA_PROBE_PORTS, timeout_s)
    return [HostProbe(ip, ports) for ip, ports in open_ports.items()]


# -- cameras ---------------------------------------------------------------------------


def _realsense_product_lines() -> dict[str, str]:
    try:
        rs = importlib.import_module(_REALSENSE_SDK)
        lines = {}
        for device in rs.context().query_devices():
            serial = device.get_info(rs.camera_info.serial_number)
            try:
                lines[serial] = str(device.get_info(rs.camera_info.product_line))
            except Exception:
                lines[serial] = ""
        return lines
    except Exception:
        return {}


def probe_realsense() -> list[CameraProbe]:
    try:
        serials = Camera.backend("realsense").discover()
    except Exception:
        return []
    product_lines = _realsense_product_lines() if serials else {}
    return [CameraProbe(serial, product_lines.get(serial, "")) for serial in sorted(serials)]


# -- gello -----------------------------------------------------------------------------


def _scan_serial_port(sdk, port: str, baudrates: Sequence[int]) -> SerialBusProbe:
    note = ""
    for baudrate in baudrates:
        try:
            responders = DynamixelBus(sdk, port, baudrate, [], device="setup").broadcast_ping()
        except Exception as error:
            note = getattr(error, "message", None) or str(error)
            continue
        if responders:
            servo_ids = sorted(responders)
            return SerialBusProbe(port, baudrate, servo_ids, [responders[servo_id] for servo_id in servo_ids])
    return SerialBusProbe(port, note=note or f"tried {', '.join(str(baudrate) for baudrate in baudrates)} baud")


def probe_gello_ports(
    extra_ports: Iterable[str] = (), baudrates: Sequence[int] = GELLO_BAUDRATES
) -> list[SerialBusProbe]:
    """Broadcast-ping every U2D2 / OpenRB-150 port (plus ``extra_ports``) at each baud rate until
    servos answer. Without dynamixel-sdk the ports are listed with empty ids and a note."""
    try:
        discovered = sorted(GelloTeleop.discover())
    except Exception:
        discovered = []
    ports = list(dict.fromkeys([*discovered, *extra_ports]))
    try:
        sdk = importlib.import_module(GelloTeleop.SDK)
    except ImportError:
        note = f"dynamixel-sdk not installed (pip install '{DYNAMIXEL_SDK_REQUIREMENT}'); servos not scanned"
        return [SerialBusProbe(port, note=note) for port in ports]
    return [_scan_serial_port(sdk, port, baudrates) for port in ports]


# -- spacemouse --------------------------------------------------------------------------


def probe_spacemouse() -> list[str]:
    try:
        return sorted(Teleop.backend("spacemouse").discover())
    except Exception:
        return []


# -- station -----------------------------------------------------------------------------


def discover_station(options: ProbeOptions | None = None) -> StationProbe:
    options = ProbeOptions() if options is None else options
    hosts = probe_franka(franka_candidates(options.robot_ips, options.env), options.timeout_s)
    if options.scan_subnet:
        known = {host.ip for host in hosts}
        scanned = probe_franka(
            [ip for ip in subnet_hosts(options.scan_subnet) if ip not in known], SUBNET_PROBE_TIMEOUT_S
        )
        hosts += [host for host in scanned if host.reachable]
    return StationProbe(
        franka_hosts=hosts,
        cameras=probe_realsense(),
        gello_buses=probe_gello_ports(options.serial_ports),
        spacemice=probe_spacemouse(),
    )


def mock_station_probe() -> StationProbe:
    """Two Frankas, three RealSense cameras and two 8-servo GELLOs; nothing is touched."""
    return StationProbe(
        franka_hosts=[HostProbe(ip, list(FRANKA_PROBE_PORTS)) for ip in MOCK_FRANKA_IPS],
        cameras=[CameraProbe(serial, "D400") for serial in MOCK_CAMERA_SERIALS],
        gello_buses=[
            SerialBusProbe(port, 57600, list(MOCK_GELLO_SERVO_IDS), [_XL330_MODEL_NUMBER] * len(MOCK_GELLO_SERVO_IDS))
            for port in MOCK_GELLO_PORTS
        ],
        mock=True,
    )


class SimulatedGelloBus:
    """Calibration stand-in for ``--mock``: the joints rest in ``pose`` with servo horns mounted
    ``HORN_OFFSETS_RAD`` off (so the solved offsets are non-zero, +1 signs), and the gripper
    trigger reading alternates between two angles on every read, so the open/closed capture
    always sees travel."""

    HORN_OFFSETS_RAD = tuple(np.pi / 2 * np.array([0, 0, 2, 2, 2, 3, 0]))
    TRIGGER_ANGLES_RAD = (3.5, 4.4)

    def __init__(self, pose: Sequence[float], has_gripper: bool) -> None:
        offsets = itertools.cycle(self.HORN_OFFSETS_RAD)
        self._pose = [float(angle) + float(next(offsets)) for angle in pose]
        self._has_gripper = has_gripper
        self._reads = 0
        self.is_open = False

    def open(self) -> None:
        self.is_open = True

    def read_positions_rad(self) -> np.ndarray:
        self._reads += 1
        trigger = [self.TRIGGER_ANGLES_RAD[self._reads % 2]] if self._has_gripper else []
        return np.array(self._pose + trigger, dtype=np.float64)

    def close(self) -> None:
        self.is_open = False
