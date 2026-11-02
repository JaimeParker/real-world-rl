"""Robot daemon: ``python -m robot_infra.server [--config robot.yaml]`` serves one robot over a
transport until SIGINT/SIGTERM, then disconnects it (never powers it off). Without ``--config``
the station file is resolved (``$ROBOT_INFRA_CONFIG``, ``./local/station.yaml``,
``~/.config/robot_infra/station.yaml``); a station with several robots starts one daemon process
per robot (or serves only ``--robot NAME``)."""
from __future__ import annotations

import argparse
import os
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path
from typing import Sequence

from robot_infra.devices import Robot, RobotConfig
from robot_infra.discovery.autoconfig import (
    DEFAULT_DAEMON_BIND,
    Station,
    fill_from_env,
    load_config_with_env,
    load_station,
    resolve_station_config_path,
)
from robot_infra.doctor.checks import exit_code, run_doctor
from robot_infra.doctor.cli import format_text
from robot_infra.server.service import RobotService
from robot_infra.server.transport.base import Transport

_TRANSPORT_NAMES = ("grpc",)
_CHILD_POLL_INTERVAL_S = 0.2
_FORWARDED_SIGNALS = (signal.SIGINT, signal.SIGTERM)
# Module-level so tests can replace the process launcher and the wait cadence.
_launch_daemon = subprocess.Popen
_sleep = time.sleep


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(prog="robot_infra.server", description="Serve one robot over the network")
    parser.add_argument("--config", default=None, help="robot or station yaml; default: $ROBOT_INFRA_CONFIG, "
        "./local/station.yaml, then ~/.config/robot_infra/station.yaml")
    parser.add_argument("--robot", default=None, help="serve only this robot of a station (default: every robot, "
        "one daemon process each)")
    parser.add_argument("--bind", default=None, help="host:port to listen on (default: the robot's daemon.bind, "
        f"else {DEFAULT_DAEMON_BIND}); single robot only")
    parser.add_argument("--token", default=None, help="shared secret clients must send (default: the robot's "
        "daemon.token); single robot only")
    parser.add_argument("--no-preflight", action="store_true", help="skip the doctor before serving")
    parser.add_argument("--watchdog-s", type=float, default=0.5, help="idle a servoing arm after this long without actions")
    parser.add_argument("--jpeg-quality", type=int, default=None, help="lossy-JPEG the color image of every camera frame at this quality (1-100); depth stays raw. "
        "Train and deploy with the same value; default sends raw frames")
    parser.add_argument("--transport", choices=_TRANSPORT_NAMES, default="grpc")
    return parser.parse_args(argv)


def _make_transport(name: str) -> Transport:
    if name == "grpc":
        from robot_infra.server.transport.grpc_transport import GrpcTransport

        return GrpcTransport()
    raise ValueError(f"unknown transport '{name}'; choose from {list(_TRANSPORT_NAMES)}")


def load_daemon_config(args: argparse.Namespace) -> RobotConfig:
    return load_config_with_env(resolve_station_config_path(args.config), robot=getattr(args, "robot", None))


def preflight_passed(config: RobotConfig) -> bool:
    results = run_doctor(config)
    print(format_text(results))
    if exit_code(results):
        print("preflight failed; fix the items above or start with --no-preflight")
        return False
    return True


def build_daemon(
    args: Sequence[str] | argparse.Namespace, *, config: RobotConfig | None = None
) -> tuple[RobotService, Transport]:
    """Build the service and an unstarted transport without serving."""
    if not isinstance(args, argparse.Namespace):
        args = parse_args(args)
    config = load_daemon_config(args) if config is None else config
    service = RobotService(Robot.from_config(config), watchdog_s=args.watchdog_s, jpeg_quality=args.jpeg_quality)
    return service, _make_transport(args.transport)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        path = resolve_station_config_path(args.config)
    except FileNotFoundError as error:
        print(error)
        return 2
    station = load_station(path)
    if args.robot is None and len(station.robots) > 1:
        return serve_station(args, path, station)
    selected = station.select(args.robot)
    bind = args.bind if args.bind is not None else selected.bind
    token = args.token if args.token is not None else selected.token
    return serve_robot(args, fill_from_env(selected.config), bind=bind, token=token)


def serve_robot(args: argparse.Namespace, config: RobotConfig, *, bind: str, token: str | None) -> int:
    """Preflight, then serve ``config`` on ``bind`` until SIGINT/SIGTERM."""
    if not args.no_preflight and not preflight_passed(config):
        return 1
    service, transport = build_daemon(args, config=config)
    robot = service.robot
    print(robot.describe_text())
    print(f"serving {robot.ROBOT_TYPE} '{robot.name}' on {bind}" + (" (token required)" if token else ""))

    def request_stop(signum: int, frame: object) -> None:
        transport.stop()

    previous_handlers = {}
    if threading.current_thread() is threading.main_thread():
        for signum in _FORWARDED_SIGNALS:
            previous_handlers[signum] = signal.signal(signum, request_stop)
    try:
        transport.serve(service, bind, token=token)
    finally:
        for signum, handler in previous_handlers.items():
            signal.signal(signum, handler)
        transport.stop()
        service.close()
    return 0


# -- multi-robot station ---------------------------------------------------------


def station_daemon_argv(config_path: str | Path, robot: str, args: argparse.Namespace) -> list[str]:
    """Command line of the daemon process serving ``robot``; the shared flags are passed on."""
    argv = [
        sys.executable, "-m", "robot_infra.server",
        "--config", os.path.abspath(config_path),
        "--robot", robot,
        "--watchdog-s", str(args.watchdog_s),
        "--transport", args.transport,
    ]
    if args.no_preflight:
        argv.append("--no-preflight")
    if args.jpeg_quality is not None:
        argv += ["--jpeg-quality", str(args.jpeg_quality)]
    return argv


def _station_conflicts(args: argparse.Namespace, station: Station) -> list[str]:
    problems = []
    if args.bind is not None or args.token is not None:
        problems.append(
            "--bind/--token apply to one robot; set daemon.bind/daemon.token per robot in the station "
            "file, or pass --robot <name>"
        )
    owners: dict[str, str] = {}
    for name, robot in station.robots.items():
        if robot.bind in owners:
            problems.append(f"robots '{owners[robot.bind]}' and '{name}' both bind {robot.bind}")
        owners.setdefault(robot.bind, name)
    return problems


def _shell_exit_code(returncode: int) -> int:
    """A child killed by signal N reports -N; report it the way a shell does (128 + N)."""
    return returncode if returncode >= 0 else 128 - returncode


def serve_station(args: argparse.Namespace, config_path: str | Path, station: Station) -> int:
    """Start one daemon process per robot, forward SIGINT/SIGTERM to them and wait for all.
    Exit code: the first non-zero child exit code in robot order, else 0."""
    problems = _station_conflicts(args, station)
    if problems:
        for problem in problems:
            print(f"error: {problem}")
        return 2
    print(f"starting {len(station.robots)} daemons from {config_path}: " + ", ".join(
        f"'{name}' on {robot.bind}" for name, robot in station.robots.items()
    ))
    children: dict[str, subprocess.Popen] = {}

    def forward(signum: int, frame: object) -> None:
        for child in children.values():
            if child.poll() is None:
                child.send_signal(signum)

    previous_handlers = {}
    if threading.current_thread() is threading.main_thread():
        for signum in _FORWARDED_SIGNALS:
            previous_handlers[signum] = signal.signal(signum, forward)
    try:
        for name in station.robots:
            # Own session: a terminal Ctrl-C reaches only this supervisor, which forwards it once.
            children[name] = _launch_daemon(station_daemon_argv(config_path, name, args), start_new_session=True)
        exit_codes = _wait_for_children(children)
    except BaseException:
        for child in children.values():
            if child.poll() is None:
                child.send_signal(signal.SIGTERM)
        for child in children.values():
            child.wait()
        raise
    finally:
        for signum, handler in previous_handlers.items():
            signal.signal(signum, handler)
    return next((code for code in exit_codes.values() if code), 0)


def _wait_for_children(children: dict[str, subprocess.Popen]) -> dict[str, int]:
    exit_codes: dict[str, int] = {}
    while True:
        for name, child in children.items():
            if name in exit_codes:
                continue
            returncode = child.poll()
            if returncode is not None:
                exit_codes[name] = _shell_exit_code(returncode)
                print(f"daemon '{name}' exited with code {exit_codes[name]}")
        if len(exit_codes) == len(children):
            return {name: exit_codes[name] for name in children}
        _sleep(_CHILD_POLL_INTERVAL_S)
