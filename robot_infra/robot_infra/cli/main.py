"""Shared parser, dispatch and result rendering for explicit command registries."""
from __future__ import annotations

import argparse
from contextlib import nullcontext, redirect_stdout
import json
import os
import sys
import traceback

from .errors import CliError, FAILURE_EXIT_CODE, SUCCESS_EXIT_CODE, USAGE_EXIT_CODE
from .command import Context, render


class _Parser(argparse.ArgumentParser):
    def error(self, message: str) -> None:
        if getattr(self, "error_factory", None) is not None:
            raise self.error_factory(message)
        prefix = "the following arguments are required: "
        required = message[len(prefix):].split(", ") if message.startswith(prefix) else ()
        raise CliError(message, code=USAGE_EXIT_CODE, hint=f"run {self.prog.split()[0]} --help", required_fields=required)


GLOBAL_OPTIONS = (
    ("station", dict(metavar="PATH", help="station yaml; default: station config lookup")),
    ("robot", dict(metavar="NAME", help="robot name in a multi-robot station")),
    ("preset", dict(metavar="PATH", help="env preset yaml for home and reset gripper")),
    ("direct", dict(action="store_true", help="connect directly if no daemon owns the arm")),
    ("mock", dict(action="store_true", help="use doctor fake backends for direct connections")),
    ("json", dict(action="store_true", help="one JSON result on stdout")),
    ("yes", dict(action="store_true", help="skip motion or doctor repair confirmation (preflight checks still apply)")),
)


def _add_global_options(parser, names=None, *, trailing=False):
    for name, options in GLOBAL_OPTIONS:
        if names is None or name in names:
            kwargs = dict(options)
            if trailing:
                # A trailing option overrides the root value only when supplied.
                kwargs["default"] = argparse.SUPPRESS
            parser.add_argument("--" + name, **kwargs)


def build_parser(*, commands, prog, description, global_options=None) -> argparse.ArgumentParser:
    parser = _Parser(prog=prog, description=description,
                     epilog="Place global options before the command.", allow_abbrev=False)
    _add_global_options(parser, global_options)
    subcommands = parser.add_subparsers(dest="command", required=True)
    for command in commands:
        options = dict(getattr(command, "parser_options", {}))
        if command.help:
            options["help"] = command.help
        subparser = subcommands.add_parser(command.name, **options)
        _add_global_options(subparser, getattr(command, "trailing_globals", ()), trailing=True)
        command.register(subparser)
        root_options = {"--" + name for name, _ in GLOBAL_OPTIONS}
        for action in subparser._actions:
            if root_options.intersection(action.option_strings):
                action.default = argparse.SUPPRESS
        subparser.set_defaults(_command=command, _parser=subparser)
    return parser


def _report_error(error: Exception, as_json: bool) -> int:
    message = str(getattr(error, "message", str(error))).replace("\n", " ")
    hint = str(getattr(error, "hint", "")).replace("\n", " ")
    diagnostic = getattr(error, "diagnostic", None)
    if diagnostic is not None:
        print(diagnostic, file=sys.stderr, end="")
    else:
        print(f"error: {message}" + (f" (hint: {hint})" if hint else ""), file=sys.stderr)
    if as_json:
        payload = getattr(error, "payload", None)
        if payload is None:
            payload = {"error": message, "hint": hint}
        required = getattr(error, "required_fields", ())
        if required:
            payload["required_fields"] = list(required)
        print(json.dumps(payload, allow_nan=False))
    if os.environ.get("RWRL_DEBUG") == "1":
        traceback.print_exc(file=sys.stderr)
    return error.code if isinstance(error, CliError) else FAILURE_EXIT_CODE


def main(argv: list[str] | None = None, *, commands, prog, description,
         station_loader=None, connection_factory=None, global_options=None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    as_json = "--json" in argv
    try:
        parser = build_parser(commands=commands, prog=prog, description=description, global_options=global_options)
        args, remaining = parser.parse_known_args(argv)
        as_json = args.json
        command = args._command
        if remaining and not getattr(command, "accepts_remaining", False):
            args._parser.error("unrecognized arguments: " + " ".join(remaining))
        if as_json and not getattr(command, "supports_json", True):
            raise CliError(f"{command.name} has no JSON output", code=USAGE_EXIT_CODE)
        ctx = Context(args, remaining, station_loader=station_loader, connection_factory=connection_factory)
        # Keep SDK chatter and confirmation text out of the JSON result stream.
        with redirect_stdout(sys.stderr) if as_json else nullcontext():
            result = command.run(args, ctx)
        render(result, ctx)
        return result.exit_code
    except CliError as error:
        return _report_error(error, as_json)
    except SystemExit as error:
        if isinstance(error.code, str):
            # sys.exit("message"): print the message as Python would, but report a usage failure.
            print(error.code, file=sys.stderr)
            return USAGE_EXIT_CODE
        return int(error.code or SUCCESS_EXIT_CODE)
    except Exception as error:
        return _report_error(error, as_json)
