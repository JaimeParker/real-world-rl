"""Station hardware primitives. Device imports happen only when a command runs."""
from __future__ import annotations

import argparse
from contextlib import nullcontext, redirect_stdout
import json
import os
import sys
import traceback

from .errors import CliError

FORWARDED_COMMANDS = ("doctor", "setup", "daemon", "supervise")


class _Parser(argparse.ArgumentParser):
    def error(self, message: str) -> None:
        prefix = "the following arguments are required: "
        required = message[len(prefix):].split(", ") if message.startswith(prefix) else ()
        raise CliError(message, code=2, hint="run rwrl --help", required_fields=required)


def build_parser() -> argparse.ArgumentParser:
    parser = _Parser(prog="rwrl", description="Robot station hardware primitives",
                     epilog="Place global options before the command.", allow_abbrev=False)
    parser.add_argument("--station", metavar="PATH", help="station yaml; default: station config lookup")
    parser.add_argument("--robot", metavar="NAME", help="robot name in a multi-robot station")
    parser.add_argument("--preset", metavar="PATH", help="env preset yaml for home and reset gripper")
    parser.add_argument("--direct", action="store_true", help="connect directly if no daemon owns the arm")
    parser.add_argument("--mock", action="store_true", help="use doctor fake backends for direct connections")
    parser.add_argument("--json", action="store_true", help="one JSON result on stdout")
    parser.add_argument("--yes", action="store_true", help="skip motion or doctor repair confirmation (preflight checks still apply)")
    commands = parser.add_subparsers(dest="command", required=True)
    for name in ("status", "q", "tcp", "stop", "recover"):
        commands.add_parser(name, allow_abbrev=False)
    home = commands.add_parser("home", help="show, save, or move to home (home phase only)")
    homes = home.add_subparsers(dest="action", required=True)
    for name in ("show", "set", "go"):
        homes.add_parser(name, allow_abbrev=False)
    gripper = commands.add_parser("gripper", help="read or command the gripper")
    grippers = gripper.add_subparsers(dest="action", required=True)
    for name in ("show", "open", "close", "move"):
        sub = grippers.add_parser(name, allow_abbrev=False)
        if name == "move":
            sub.add_argument("fraction", type=float, metavar="F")
    for name in FORWARDED_COMMANDS:
        # Even --help belongs to the existing CLI, not this wrapper.
        commands.add_parser(name, add_help=False, allow_abbrev=False)
    from .teleop_config import add_commands
    add_commands(commands)
    return parser


def _report_error(error: Exception, as_json: bool) -> int:
    message = str(getattr(error, "message", str(error))).replace("\n", " ")
    hint = str(getattr(error, "hint", "")).replace("\n", " ")
    print(f"error: {message}" + (f" (hint: {hint})" if hint else ""), file=sys.stderr)
    if as_json:
        payload = {"error": message, "hint": hint}
        required = getattr(error, "required_fields", ())
        if required:
            payload["required_fields"] = list(required)
        print(json.dumps(payload, allow_nan=False))
    if os.environ.get("RWRL_DEBUG") == "1":
        traceback.print_exc(file=sys.stderr)
    return error.code if isinstance(error, CliError) else 1


def main(argv: list[str] | None = None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    as_json = "--json" in argv
    try:
        parser = build_parser()
        args, remaining = parser.parse_known_args(argv)
        as_json = args.json
        if args.command in FORWARDED_COMMANDS:
            from .forward import forward

            return forward(args, remaining)
        if remaining:
            parser.error("unrecognized arguments: " + " ".join(remaining))
        if args.command in ("teleop", "workspace"):
            from .teleop_config import execute
        else:
            from .commands import execute

        # Keep SDK chatter and confirmation text out of the JSON result stream.
        with redirect_stdout(sys.stderr) if as_json else nullcontext():
            payload, output = execute(args)
        print(json.dumps(payload, allow_nan=False) if as_json else output)
        if args.command == "teleop" and args.teleop_command == "inspect-stop":
            from .errors import STOP_TRACE_FAIL_EXIT_CODE, STOP_TRACE_BLOCKED_EXIT_CODE
            return {"PASS": 0, "FAIL": STOP_TRACE_FAIL_EXIT_CODE,
                    "BLOCKED": STOP_TRACE_BLOCKED_EXIT_CODE}[payload["status"]]
        if args.command == "workspace" and args.action == "validate" and not payload["valid"]:
            from .errors import WORKSPACE_INVALID_EXIT_CODE
            return 0 if as_json else WORKSPACE_INVALID_EXIT_CODE
        return 0
    except CliError as error:
        return _report_error(error, as_json)
    except SystemExit as error:
        return int(error.code or 0)
    except Exception as error:
        return _report_error(error, as_json)
