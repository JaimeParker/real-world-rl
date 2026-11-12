"""``rwrl discover [--station robot.yaml]``: print what hardware is visible."""
from __future__ import annotations

import dataclasses
from real_world_rl.cli.cmd_discover import build_parser
from pathlib import Path

from real_world_rl.discovery.autoconfig import (
    fill_from_env,
    resolve_station_config_path,
)
from real_world_rl.discovery.enumerate import DiscoveryReport, discover_all, hosts_from_config


def load_station(path: str | Path):
    """The station with its upper-layer sections (``teleop_profiles``) validated, as every rwrl
    command loads it; imported on use so this module stays light."""
    from real_world_rl.station import load_station as load_with_sections

    return load_with_sections(path)


def _station_path(explicit: str | None) -> Path | None:
    """The given config, else the resolved station file, else None (no hosts are probed)."""
    if explicit:
        return Path(explicit)
    try:
        return resolve_station_config_path()
    except FileNotFoundError:
        return None


def host_ips_from_station(path: str | Path) -> list[str]:
    """Every robot's ``robot_ip`` (environment applied), in first-occurrence order."""
    ips: list[str] = []
    for robot in load_station(path).robots.values():
        ips.extend(hosts_from_config(fill_from_env(robot.config)))
    return list(dict.fromkeys(ips))




def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    return run(parser.parse_args(argv), None)


def station_spacemouse_table(path: str | Path):
    """The SpaceMouse device table of the station: its first spacemouse teleop's
    ``device_table``, else the station's ``spacemouse-devices.yaml``, else the built-in table."""
    from real_world_rl.devices.backends.teleops._spacemouse_hid import load_device_specs

    options = next(
        (robot.config.teleop.options for robot in load_station(path).robots.values()
         if robot.config.teleop is not None and robot.config.teleop.backend == "spacemouse"),
        {},
    )
    return load_device_specs(Path(path).resolve().parent, options)


def collect(args) -> DiscoveryReport:
    path = _station_path(args.config)
    if path is None:
        return discover_all([], spacemouse_table=None)
    return discover_all(host_ips_from_station(path), spacemouse_table=station_spacemouse_table(path))


def report_payload(report: DiscoveryReport) -> dict:
    """The JSON form of a report: one list per category (empty when nothing was found), sorted."""
    return {
        "schema_version": 1,
        "cameras": [
            {"backend": backend, "serial": serial}
            for backend, serials in sorted(report.cameras.items()) for serial in sorted(serials)
        ],
        "teleops": [
            {"backend": backend, "device": device}
            for backend, devices in sorted(report.teleops.items()) for device in sorted(devices)
        ],
        "can_interfaces": [dataclasses.asdict(info) for info in report.can_interfaces.values()],
        "hosts": [dataclasses.asdict(host) for host in report.hosts.values()],
        "hid_devices": list(report.hid_devices),
        "unknown_hid": [
            dict(device) for device in sorted(report.unknown_hid, key=lambda d: (d["vid"], d["pid"], d["path"]))
        ],
    }


def run(args, ctx) -> int:
    print(collect(args).format())
    return 0
