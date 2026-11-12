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


@dataclass
class TeleopConnection:
    teleop: Any
    station_config: Any
    path: str
    remote: bool
    follower: Any = None  # the daemon's arm when requested; never opened on the direct route


FOLLOWER_NEEDS_DAEMON_HINT = ("follower comparison needs the daemon route "
                              "(direct access would take the FCI control connection)")


def _selected_robot(args):
    from real_world_rl.station import load_station
    from real_world_rl.discovery.autoconfig import resolve_station_config_path

    try:
        station_path = resolve_station_config_path(args.station)
        if not station_path.is_file():
            raise FileNotFoundError(f"station config not found: {station_path}")
        return load_station(station_path).select(args.robot)
    except (FileNotFoundError, ValueError, KeyError) as error:
        raise CliError(str(error), code=USAGE_EXIT_CODE, hint="run rwrl setup") from error


def _daemon_address(args, selected) -> str | None:
    """Address of the daemon that owns the selected robot; None selects the direct route."""
    if not selected.daemon_declared:
        return None
    address = client_address(selected.bind)
    reachable = probe_daemon(address)
    if args.direct and reachable:
        raise CliError("the daemon owns the arm", code=USAGE_EXIT_CODE, hint="omit --direct")
    if args.direct:
        return None
    if not reachable:
        raise CliError(f"daemon declared at {selected.bind} but not reachable",
                       hint="start it (rwrl daemon) or pass --direct")
    return address


def _remote_robot(address: str, selected):
    from real_world_rl.server.remote import RemoteRobot, RemoteRobotConfig

    return RemoteRobot(RemoteRobotConfig(address=address, token=selected.token))


def _direct_robot(args, selected):
    """An unconnected robot built from the station; ``--mock`` swaps in the fake backends."""
    from real_world_rl.devices import Robot
    from real_world_rl.discovery.autoconfig import fill_from_env

    config = selected.config
    if args.mock:
        from real_world_rl.doctor.checks import mocked_config

        config = mocked_config(config)
    return Robot.from_config(fill_from_env(config)), config


@contextmanager
def _connected(part) -> Iterator[None]:
    try:
        part.connect()
        yield
    finally:
        # Cleanup must not replace the useful failure that caused it.
        failing = sys.exc_info()[0] is not None
        try:
            part.disconnect()
        except Exception:
            if not failing:
                raise


@contextmanager
def open_robot(args) -> Iterator[RobotConnection]:
    selected = _selected_robot(args)
    address = _daemon_address(args, selected)
    if address is not None:
        robot = _remote_robot(address, selected)
        path = f"daemon {address}"
    else:
        robot, config = _direct_robot(args, selected)
        path = f"direct {config.arm_backend}"
    with _connected(robot):
        yield RobotConnection(robot, selected.config, path, address is not None)


@contextmanager
def open_teleop(args, *, with_follower: bool = False) -> Iterator[TeleopConnection]:
    """Connect only the station's teleop part, or the daemon that serves it.

    The direct route builds the robot without connecting it and opens its teleop alone, so the
    arm, gripper and cameras stay free. ``with_follower`` exposes the daemon's arm for reading
    and is refused on the direct route."""
    selected = _selected_robot(args)
    if selected.config.teleop is None:
        raise CliError("station has no teleop", code=USAGE_EXIT_CODE,
                       hint="add a teleop: block to the station robot")
    address = _daemon_address(args, selected)
    if address is None:
        if with_follower:
            raise CliError("--with-follower is not available on the direct route",
                           code=USAGE_EXIT_CODE, hint=FOLLOWER_NEEDS_DAEMON_HINT)
        robot, config = _direct_robot(args, selected)
        teleop = robot.teleop
        with _connected(teleop):
            yield TeleopConnection(teleop, selected.config, f"direct {config.teleop.backend}", False)
        return
    robot = _remote_robot(address, selected)
    with _connected(robot):
        if robot.teleop is None:
            raise CliError(f"daemon at {address} serves no teleop", code=USAGE_EXIT_CODE,
                           hint="restart the daemon after adding the station teleop")
        yield TeleopConnection(robot.teleop, selected.config, f"daemon {address}", True,
                               robot.arm if with_follower else None)
