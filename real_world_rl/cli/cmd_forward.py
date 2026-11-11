"""Declarative argv translation for CLIs that still own parsing and output."""
from contextlib import redirect_stdout
from dataclasses import dataclass
from importlib import import_module

from .command import Result
from .errors import SUCCESS_EXIT_CODE


@dataclass(frozen=True)
class ForwardCommand:
    name: str
    module: str
    # (global attribute, downstream flag, flag-only) in historical argv order.
    options: tuple[tuple[str, str, bool], ...] = ()
    help: str = ""
    accepts_remaining = True
    parser_options = {"add_help": False, "allow_abbrev": False}

    def register(self, parser):
        # Even --help belongs to the downstream CLI.
        pass

    def run(self, args, ctx):
        prefix = []
        for attribute, flag, flag_only in self.options:
            value = getattr(args, attribute)
            if flag_only:
                if value:
                    prefix.append(flag)
            elif value is not None:
                prefix.extend((flag, value))
        with redirect_stdout(ctx.stdout):
            code = import_module(self.module).main([*prefix, *ctx.remaining])
        return Result({}, "", int(code or SUCCESS_EXIT_CODE), rendered=True)


SETUP = ForwardCommand("setup", "robot_infra.setup.__main__", (
    ("station", "--from", False), ("mock", "--mock", True),
))
DAEMON = ForwardCommand("daemon", "robot_infra.server.daemon", (
    ("station", "--config", False), ("robot", "--robot", False),
))
SUPERVISE = ForwardCommand("supervise", "robot_infra.supervise.supervisor")
