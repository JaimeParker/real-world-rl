"""Lightweight supervise registration; device imports wait until run."""
from __future__ import annotations

import argparse



def build_parser(parser=None):
    parser = parser if parser is not None else argparse.ArgumentParser(
        prog="rwrl supervise",
        description="Run a robot process under a separate heartbeat watchdog; on silence escalate "
        "SIGINT -> SIGTERM -> SIGKILL to its process group.",
        usage="rwrl supervise [options] -- <command...>",
    )
    parser.add_argument("--heartbeat-timeout-s", type=float, default=2.0,
                        help="escalate after this long without a heartbeat")
    parser.add_argument("--grace-s", type=float, default=3.0,
                        help="wait this long after SIGINT before SIGTERM")
    parser.add_argument("--kill-s", type=float, default=2.0,
                        help="wait this long after SIGTERM before SIGKILL")
    parser.add_argument("--startup-s", type=float, default=30.0,
                        help="no heartbeat is required until the first one arrives or this elapses")
    parser.add_argument("--long-op-s", type=float, default=60.0,
                        help="silence allowed after the child announces a long operation (an env reset)")
    parser.add_argument("--log", default=None, help="append a jsonl event log to this path")
    parser.add_argument("child_command", nargs=argparse.REMAINDER, metavar="command", help="the supervised command, after --")
    return parser



def run(args, ctx):
    from real_world_rl.supervise.supervisor import run as implementation
    return implementation(args, ctx)
