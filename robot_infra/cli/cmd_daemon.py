"""Lightweight daemon registration; device imports wait until run."""
from __future__ import annotations

import argparse

from .options import add_station_argument

DEFAULT_DAEMON_BIND = "0.0.0.0:50051"
_TRANSPORT_NAMES = ("grpc",)


def build_parser(parser=None):
    parser = parser if parser is not None else argparse.ArgumentParser(prog="rwrl daemon", description="Serve one robot over the network")
    add_station_argument(parser, alias="--config", dest="config", help="robot or station yaml; default: $ROBOT_INFRA_CONFIG, "
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
    return parser



def run(args, ctx):
    from robot_infra.server.daemon import run as implementation
    return implementation(args, ctx)
