"""Read-only probes: cameras, teleop devices, CAN interfaces, reachable hosts and HID devices."""
from __future__ import annotations

import glob
import re
import socket
import subprocess
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Iterable, Mapping

from real_world_rl.devices import Camera, RobotConfig, Teleop

_ARPHRD_CAN = "280"
_DEFAULT_PORTS = (22, 80, 443, 1337, 50051)


@dataclass
class CanInfo:
    name: str
    operstate: str
    bitrate: int | None


@dataclass
class HostInfo:
    ip: str
    ping_ok: bool
    open_ports: list[int]


@dataclass
class DiscoveryReport:
    cameras: dict[str, set[str]] = field(default_factory=dict)
    teleops: dict[str, set[str]] = field(default_factory=dict)
    can_interfaces: dict[str, CanInfo] = field(default_factory=dict)
    hosts: dict[str, HostInfo] = field(default_factory=dict)
    hid_devices: list[str] = field(default_factory=list)
    # HID devices that look like a SpaceMouse but are missing from the device table:
    # {vid, pid, product, path, message} (``_spacemouse_hid.unknown_hid_devices``).
    unknown_hid: list[dict] = field(default_factory=list)

    def format(self) -> str:
        lines = ["cameras:"]
        if any(self.cameras.values()):
            for backend, serials in sorted(self.cameras.items()):
                if serials:
                    lines.append(f"  {backend}: {', '.join(sorted(serials))}")
        else:
            lines.append("  none")
        lines.append("teleops:")
        if any(self.teleops.values()):
            for backend, devices in sorted(self.teleops.items()):
                if devices:
                    lines.append(f"  {backend}: {', '.join(sorted(devices))}")
        else:
            lines.append("  none")
        lines.append("can interfaces:")
        if self.can_interfaces:
            for info in self.can_interfaces.values():
                bitrate = "unknown" if info.bitrate is None else str(info.bitrate)
                lines.append(f"  {info.name}: operstate={info.operstate} bitrate={bitrate}")
        else:
            lines.append("  none")
        lines.append("hosts:")
        if self.hosts:
            for host in self.hosts.values():
                ports = ",".join(str(port) for port in host.open_ports) or "none"
                lines.append(
                    f"  {host.ip}: ping={'ok' if host.ping_ok else 'no reply'} open ports={ports}"
                )
        else:
            lines.append("  none")
        lines.append("hid devices:")
        lines.extend(f"  {path}" for path in self.hid_devices or ["none"])
        if self.unknown_hid:
            lines.append("warnings:")
            lines.extend(f"  WARN {device['message']} ({device['path']})" for device in self.unknown_hid)
        return "\n".join(lines)


def can_bitrate(name: str) -> int | None:
    try:
        output = subprocess.run(
            ["ip", "-details", "link", "show", name],
            capture_output=True,
            text=True,
            timeout=2.0,
            check=True,
        ).stdout
        match = re.search(r"bitrate (\d+)", output)
        return int(match.group(1)) if match else None
    except Exception:
        return None


def list_can_interfaces(sys_class_net: Path = Path("/sys/class/net")) -> dict[str, CanInfo]:
    interfaces: dict[str, CanInfo] = {}
    if not sys_class_net.is_dir():
        return interfaces
    for iface in sorted(sys_class_net.iterdir()):
        try:
            if (iface / "type").read_text().strip() != _ARPHRD_CAN:
                continue
            operstate = (iface / "operstate").read_text().strip()
        except OSError:
            continue
        interfaces[iface.name] = CanInfo(iface.name, operstate, can_bitrate(iface.name))
    return interfaces


def ping(ip: str, timeout_s: float = 1.0) -> bool:
    try:
        result = subprocess.run(
            ["ping", "-c", "1", "-W", str(int(timeout_s)), ip],
            capture_output=True,
            timeout=timeout_s + 2.0,
        )
    except FileNotFoundError:
        return False
    except subprocess.TimeoutExpired:
        return False
    return result.returncode == 0


def tcp_open(ip: str, port: int, timeout_s: float = 0.5) -> bool:
    try:
        with socket.create_connection((ip, port), timeout=timeout_s):
            return True
    except OSError:
        return False


def open_ports_by_host(
    host_ips: Iterable[str], ports: Iterable[int], timeout_s: float = 0.5, max_workers: int = 32
) -> dict[str, list[int]]:
    """TCP-connect every (host, port) pair in a thread pool; the open ports per host, in input order."""
    host_ips, ports = list(dict.fromkeys(host_ips)), list(ports)
    pairs = [(ip, port) for ip in host_ips for port in ports]
    if not pairs:
        return {ip: [] for ip in host_ips}
    with ThreadPoolExecutor(max_workers=min(max_workers, len(pairs))) as pool:
        states = list(pool.map(lambda pair: tcp_open(pair[0], pair[1], timeout_s), pairs))
    open_ports: dict[str, list[int]] = {ip: [] for ip in host_ips}
    for (ip, port), is_open in zip(pairs, states):
        if is_open:
            open_ports[ip].append(port)
    return open_ports


def discover_cameras(hardware_only: bool = False) -> dict[str, set[str]]:
    """Serials per camera backend; ``hardware_only`` skips backends without a vendor SDK (the fake)."""
    cameras: dict[str, set[str]] = {}
    for name, backend_cls in sorted(Camera.backends().items()):
        if hardware_only and backend_cls.SDK is None:
            continue
        try:
            cameras[name] = set(backend_cls.discover())
        except Exception:
            cameras[name] = set()
    return cameras


def discover_teleops(
    hardware_only: bool = False, *, discover_kwargs: Mapping[str, Mapping[str, Any]] | None = None,
) -> dict[str, set[str]]:
    """Device ids per teleop backend; ``hardware_only`` skips backends without a vendor SDK (the
    fake). ``discover_kwargs`` maps a backend name to keyword arguments of its ``discover``."""
    teleops: dict[str, set[str]] = {}
    for name, backend_cls in sorted(Teleop.backends().items()):
        if hardware_only and backend_cls.SDK is None:
            continue
        try:
            teleops[name] = set(backend_cls.discover(**dict((discover_kwargs or {}).get(name, {}))))
        except Exception:
            teleops[name] = set()
    return teleops


def discover_all(
    host_ips: Iterable[str] = (), ports: Iterable[int] = _DEFAULT_PORTS, *, spacemouse_table=None,
) -> DiscoveryReport:
    """Everything visible from this host. ``spacemouse_table`` is the SpaceMouse device table
    the HID devices are matched against (default: ``_spacemouse_hid.load_device_specs()``)."""
    from real_world_rl.devices.backends.teleops import _spacemouse_hid

    table = _spacemouse_hid.load_device_specs() if spacemouse_table is None else spacemouse_table
    ports = list(ports)
    hosts = {
        ip: HostInfo(ip, ping(ip), [port for port in ports if tcp_open(ip, port)])
        for ip in host_ips
    }
    return DiscoveryReport(
        cameras=discover_cameras(hardware_only=True),
        teleops=discover_teleops(hardware_only=True, discover_kwargs={"spacemouse": {"table": table}}),
        can_interfaces=list_can_interfaces(),
        hosts=hosts,
        hid_devices=sorted(glob.glob("/dev/hidraw*")),
        unknown_hid=_spacemouse_hid.unknown_hid_devices(table),
    )


def hosts_from_config(config: RobotConfig) -> list[str]:
    ip = getattr(config, "robot_ip", None)
    return [ip] if ip else []
