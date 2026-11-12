"""``python -m robot_infra.discovery [--config robot.yaml]``: print what hardware is visible."""
from __future__ import annotations

import dataclasses
from robot_infra.cli.cmd_discover import build_parser
from pathlib import Path

from robot_infra.discovery.autoconfig import (
    fill_from_env,
    load_station,
    resolve_station_config_path,
)
from robot_infra.discovery.enumerate import DiscoveryReport, discover_all, hosts_from_config


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


def collect(args) -> DiscoveryReport:
    path = _station_path(args.config)
    host_ips = host_ips_from_station(path) if path is not None else []
    return discover_all(host_ips)


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
    }


def run(args, ctx) -> int:
    print(collect(args).format())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
