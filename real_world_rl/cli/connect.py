"""Resolve daemon versus direct ownership and pair every connect with disconnect."""
from __future__ import annotations

from contextlib import contextmanager
from dataclasses import dataclass
import socket
import sys
from typing import Any, Iterator

from .errors import CliError, USAGE_EXIT_CODE


def client_address(bind: str) -> str:
    host, separator, port = bind.rpartition(":")
    if not separator or not host or not port.isdigit() or not 0 <= int(port) <= 65535:
        raise CliError(f"invalid daemon bind {bind!r}", code=USAGE_EXIT_CODE)
    return f"{'127.0.0.1' if host == '0.0.0.0' else host}:{port}"


def probe_daemon(address: str, timeout_s: float = 0.5) -> bool:
    """Small TCP probe; replace this function for hardware-free daemon tests."""
    host, _, port = address.rpartition(":")
    try:
        with socket.create_connection((host.strip("[]"), int(port)), timeout=timeout_s):
            return True
    except OSError:
        return False


@dataclass
class RobotConnection:
    robot: Any
    station_config: Any
    path: str
    remote: bool

    def __getattr__(self, name: str) -> Any:
        return getattr(self.robot, name)


@contextmanager
def open_robot(args) -> Iterator[RobotConnection]:
    from real_world_rl.station import load_station
    from robot_infra.devices import Robot
    from robot_infra.discovery.autoconfig import fill_from_env, resolve_station_config_path

    try:
        station_path = resolve_station_config_path(args.station)
        if not station_path.is_file():
            raise FileNotFoundError(f"station config not found: {station_path}")
        station = load_station(station_path)
        selected = station.select(args.robot)
    except (FileNotFoundError, ValueError, KeyError) as error:
        raise CliError(str(error), code=USAGE_EXIT_CODE, hint="run python -m robot_infra.setup") from error
    remote = False
    if selected.daemon_declared:
        address = client_address(selected.bind)
        reachable = probe_daemon(address)
        if args.direct and reachable:
            raise CliError("the daemon owns the arm", code=USAGE_EXIT_CODE, hint="omit --direct")
        if not args.direct:
            if not reachable:
                raise CliError(f"daemon declared at {selected.bind} but not reachable",
                               hint="start it (rwrl daemon) or pass --direct")
            from robot_infra.server.remote import RemoteRobot, RemoteRobotConfig

            robot = RemoteRobot(RemoteRobotConfig(address=address, token=selected.token))
            path = f"daemon {address}"
            remote = True
    if not remote:
        config = selected.config
        if args.mock:
            from robot_infra.doctor.checks import mocked_config

            config = mocked_config(config)
        robot = Robot.from_config(fill_from_env(config))
        path = f"direct {config.arm_backend}"
    try:
        robot.connect()
        yield RobotConnection(robot, selected.config, path, remote)
    finally:
        # Cleanup must not replace the useful failure that caused it.
        failing = sys.exc_info()[0] is not None
        try:
            robot.disconnect()
        except Exception:
            if not failing:
                raise
