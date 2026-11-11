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
    # Forwarded CLIs own their output (including help); never render a second result.
    rendered: bool = False
    ensure_ascii: bool = True


class Context:
    def __init__(self, args: argparse.Namespace, remaining=()):
        self.args = args
        self.remaining = list(remaining)
        self.stdout = sys.stdout
        self.stderr = sys.stderr
        self._station = None

    def open_robot(self):
        from .connect import open_robot
        return open_robot(self.args)

    def station(self):
        if self._station is None:
            from real_world_rl.station import load_station
            from robot_infra.discovery.autoconfig import resolve_station_config_path
            self._station = load_station(resolve_station_config_path(self.args.station))
        return self._station

    def confirm(self, question: str, *, detail: str, saving: bool = False) -> None:
        """Preserve motion/save prompts while sharing the TTY, yes and EOF policy."""
        stream = self.stderr if saving or self.args.json else self.stdout
        print(detail, file=stream)
        if self.args.yes:
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
        if answer.strip().lower() != "y":
            raise CliError("cancelled", code=USAGE_EXIT_CODE)


def render(result: Result, ctx: Context) -> None:
    if not result.rendered:
        print(json.dumps(result.payload, allow_nan=False, ensure_ascii=result.ensure_ascii) if ctx.args.json else result.text, file=ctx.stdout)


class Command(Protocol):
    name: str
    help: str

    def register(self, parser: argparse.ArgumentParser) -> None: ...

    def run(self, args: argparse.Namespace, ctx: Context) -> Result: ...


# Populated explicitly by cli.__init__; command modules do not self-register.
COMMANDS: list[Command] = []
