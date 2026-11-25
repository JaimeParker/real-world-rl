"""``rwrl shell``: interactive terminal on one device profile; its UI loads only when run."""
from __future__ import annotations

import os
import sys

from .command import Result
from .errors import CliError, USAGE_EXIT_CODE

NOT_APPLICABLE_OPTIONS = ("station", "robot", "preset", "direct", "mock", "yes")
INSTALL_HINT = "pip install -e '.[shell]' (from the repository root)"


class ShellCommand:
    name = "shell"
    help = "interactive terminal: pick a device profile, then run commands without `rwrl`"
    supports_json = False
    uses_selection = False

    def register(self, parser):
        parser.description = (
            "Pick a device profile (local/sites/<name>/ with station.yaml, task.yaml and tasks/*.yaml), then type "
            "commands without the rwrl prefix. The selection applies to this terminal only.")
        parser.add_argument("profile", nargs="?", help="profile to select (default: choose from a list)")
        parser.add_argument("--task", help="task to select (default: the last used one)")

    def run(self, args, ctx):
        from real_world_rl.common.station_paths import SHELL_ENV
        for option in NOT_APPLICABLE_OPTIONS:
            if getattr(args, option, None) not in (None, False):
                raise CliError(f"--{option} not applicable to rwrl shell", code=USAGE_EXIT_CODE)
        if args.task and not args.profile:
            raise CliError("--task needs a profile", code=USAGE_EXIT_CODE)
        if os.environ.get(SHELL_ENV):
            raise CliError("already inside rwrl shell", code=USAGE_EXIT_CODE)
        if not (sys.stdin.isatty() and sys.stdout.isatty()):
            raise CliError("rwrl shell needs a terminal", code=USAGE_EXIT_CODE,
                           hint="scripts and agents use the plain rwrl commands")
        try:
            import prompt_toolkit  # noqa: F401
            import rich  # noqa: F401
        except ModuleNotFoundError as error:
            raise CliError("rwrl shell needs the shell extra", code=USAGE_EXIT_CODE, hint=INSTALL_HINT) from error
        from real_world_rl.shell.app import run
        return Result({}, "", run(args.profile, args.task), rendered=True)


SHELL = ShellCommand()
