"""Shared command contract, invocation context and confirmation policy."""
from __future__ import annotations

import argparse
from contextlib import redirect_stdout
from dataclasses import dataclass
import json
import sys
from typing import Protocol

from .errors import CliError, SUCCESS_EXIT_CODE, USAGE_EXIT_CODE


@dataclass
class Result:
    payload: dict
    text: str
    exit_code: int = SUCCESS_EXIT_CODE
    # Native commands may own their output; never render a second result.
    rendered: bool = False
    ensure_ascii: bool = True


class Context:
    def __init__(self, args: argparse.Namespace, remaining=(), *, station_loader=None, connection_factory=None):
        self.args = args
        self.remaining = list(remaining)
        self.stdout = sys.stdout
        self.stderr = sys.stderr
        self._station = None
        self.station_loader = station_loader
        self.connection_factory = connection_factory

    def open_robot(self):
        if self.connection_factory is None:
            raise CliError("this command has no connection factory", code=USAGE_EXIT_CODE)
        return self.connection_factory(self.args)

    def station(self):
        if self._station is None:
            from robot_infra.discovery.autoconfig import load_station
            from robot_infra.discovery.autoconfig import resolve_station_config_path
            self._station = (self.station_loader or load_station)(resolve_station_config_path(self.args.station))
        return self._station

    def confirm(self, question: str, *, detail: str, saving: bool = False) -> None:
        return confirm(self, question, detail=detail, saving=saving)


def affirmative(answer: str) -> bool:
    """Only an explicit y confirms an action."""
    return answer.strip().lower() == "y"


def doctor_envelope(*, code: int, checks, repairs, **extra) -> dict:
    """Use the host doctor's field names without altering device result records."""
    return {
        "schema_version": 1,
        "status": "PASS" if code == SUCCESS_EXIT_CODE else "FAIL",
        "checks": checks,
        "repairs": repairs,
        **extra,
    }


def confirm(ctx: Context, question: str, *, detail: str, saving: bool = False) -> None:
    """Preserve motion/save prompts while sharing the TTY, yes and EOF policy."""
    stream = ctx.stderr if saving or ctx.args.json else ctx.stdout
    print(detail, file=stream)
    if ctx.args.yes:
        return
    if not sys.stdin.isatty():
        message = "saving requires --yes without a terminal" if saving else "refusing to move without --yes"
        raise CliError(message, code=USAGE_EXIT_CODE)
    try:
        if saving:
            print(question, file=stream, end="", flush=True)
            answer = sys.stdin.readline()
        else:
            with redirect_stdout(stream):
                answer = input(question)
    except EOFError:
        answer = ""
    if not affirmative(answer):
        raise CliError("cancelled", code=USAGE_EXIT_CODE)


def render(result: Result, ctx: Context) -> None:
    if not result.rendered:
        print(json.dumps(result.payload, allow_nan=False, ensure_ascii=result.ensure_ascii) if ctx.args.json else result.text, file=ctx.stdout)


class Command(Protocol):
    name: str
    help: str

    def register(self, parser: argparse.ArgumentParser) -> None: ...

    def run(self, args: argparse.Namespace, ctx: Context) -> Result: ...

