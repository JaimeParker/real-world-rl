"""Lightweight discover registration; device imports wait until run."""
from __future__ import annotations

import argparse

from .options import add_station_argument



def build_parser(parser=None):
    parser = parser if parser is not None else argparse.ArgumentParser(prog="rwrl discover", description='``rwrl discover [--station robot.yaml]``: print what hardware is visible.')
    add_station_argument(
        parser, dest="config",
        help="robot or station yaml; its robot IPs are probed (never connected). Default: the "
        "station file the daemon would use; none found means no hosts",
    )
    return parser



def run(args, ctx):
    from real_world_rl.discovery.report import run as implementation
    return implementation(args, ctx)


def run_json(args, ctx):
    from real_world_rl.discovery.report import collect, report_payload
    from .command import Result
    report = collect(args)
    return Result(report_payload(report), report.format())
