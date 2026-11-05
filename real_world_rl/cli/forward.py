"""Thin argv forwarding without importing executable __main__ side effects."""
from __future__ import annotations

from importlib import import_module


def forward(args, remaining: list[str]) -> int:
    modules = {
        "doctor": "real_world_rl.cli.doctor",
        "setup": "robot_infra.setup.__main__",
        "daemon": "robot_infra.server.daemon",
        "supervise": "robot_infra.supervise.supervisor",
    }
    prefix: list[str] = []
    if args.command in ("doctor", "daemon"):
        if args.station is not None:
            prefix += ["--config", args.station]
        if args.robot is not None:
            prefix += ["--robot", args.robot]
    elif args.command == "setup" and args.station is not None:
        prefix += ["--from", args.station]
    if args.command in ("doctor", "setup") and args.mock:
        prefix.append("--mock")
    if args.command == "doctor":
        if args.preset is not None:
            prefix += ["--preset", args.preset]
        if args.json:
            prefix.append("--json")
        if args.yes:
            prefix.append("--yes")
    return int(import_module(modules[args.command]).main([*prefix, *remaining]) or 0)
