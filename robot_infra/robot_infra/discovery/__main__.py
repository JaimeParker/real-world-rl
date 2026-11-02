"""``python -m robot_infra.discovery [--config robot.yaml]``: print what hardware is visible."""
from __future__ import annotations

import argparse
from pathlib import Path

from robot_infra.discovery.autoconfig import (
    fill_from_env,
    load_station,
    resolve_station_config_path,
)
from robot_infra.discovery.enumerate import discover_all, hosts_from_config


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
    parser = argparse.ArgumentParser(prog="robot_infra.discovery", description=__doc__)
    parser.add_argument(
        "--config",
        help="robot or station yaml; its robot IPs are probed (never connected). Default: the "
        "station file the daemon would use; none found means no hosts",
    )
    args = parser.parse_args(argv)
    path = _station_path(args.config)
    host_ips = host_ips_from_station(path) if path is not None else []
    print(discover_all(host_ips).format())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
