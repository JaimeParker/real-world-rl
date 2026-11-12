"""Lightweight discover registration; device imports wait until run."""
from __future__ import annotations

import argparse

from .options import add_station_argument



def build_parser(parser=None):
    parser = parser if parser is not None else argparse.ArgumentParser(prog="robot_infra.discovery", description='``python -m robot_infra.discovery [--station robot.yaml]``: print what hardware is visible.')
    add_station_argument(
        parser, alias="--config", dest="config",
        help="robot or station yaml; its robot IPs are probed (never connected). Default: the "
        "station file the daemon would use; none found means no hosts",
    )
    return parser



def run(args, ctx):
    from robot_infra.discovery.__main__ import run as implementation
    return implementation(args, ctx)


def run_json(args, ctx):
    from robot_infra.discovery.__main__ import collect, report_payload
    from .command import Result
    report = collect(args)
    return Result(report_payload(report), report.format())
