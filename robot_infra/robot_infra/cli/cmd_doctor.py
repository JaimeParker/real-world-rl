"""Lightweight doctor registration; device imports wait until run."""
from __future__ import annotations

import argparse

from .options import add_station_argument



def build_parser(parser=None):
    parser = parser if parser is not None else argparse.ArgumentParser(prog="robot_infra.doctor", description="Layered robot diagnostic")
    add_station_argument(
        parser, alias="--config", dest="config",
        help="robot or station yaml; default: $ROBOT_INFRA_CONFIG, ./local/station.yaml, then "
        "~/.config/robot_infra/station.yaml",
    )
    parser.add_argument("--robot", default=None, help="check only this robot of a station (default: all)")
    parser.add_argument("--mock", action="store_true", help="swap every backend for its fake")
    parser.add_argument("--fix", action="store_true", help="run autofixes for failed checks")
    parser.add_argument("--json", action="store_true", help="machine-readable output")
    parser.add_argument(
        "--install-sdk",
        action="store_true",
        help="after a libfranka version mismatch, offer to pip install the franky wheel for the robot server "
        "version (asks first; with --json only reports the command)",
    )
    parser.add_argument("--yes", action="store_true", help="skip confirmation; preflight checks still run")
    return parser



def run(args, ctx):
    from robot_infra.doctor.cli import run as implementation
    return implementation(args, ctx)
