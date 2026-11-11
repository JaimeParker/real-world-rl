"""Lazy adapters; learning entrypoints retain ownership of their arguments."""
from dataclasses import dataclass
from importlib import import_module
import sys

from .command import Result
from .errors import CliError, SUCCESS_EXIT_CODE, USAGE_EXIT_CODE


@dataclass(frozen=True)
class LearningCommand:
    name: str
    help: str
    target: str
    accepts_remaining = True
    parser_options = {"add_help": False, "allow_abbrev": False}

    def register(self, parser):
        # Even --help belongs to the entrypoint, rather than this argparse layer.
        pass

    def entrypoint_argv(self, args, remaining):
        for option in ("station", "robot", "direct", "mock", "json", "yes"):
            value = getattr(args, option, None)
            supplied = value is not None if option in ("station", "robot") else bool(value)
            if supplied:
                raise CliError(
                    f"--{option} not applicable to rwrl {self.name}; pass the entrypoint's own flags",
                    code=USAGE_EXIT_CODE,
                )
        if args.preset is not None:
            if any(token == "--config" or token.startswith("--config=") for token in remaining):
                raise CliError("--preset conflicts with the entrypoint's --config", code=USAGE_EXIT_CODE)
            return ["--config", args.preset, *remaining]
        return remaining

    def run(self, args, ctx):
        argv = self.entrypoint_argv(args, ctx.remaining)
        module, function = self.target.split(":", 1)
        try:
            code = getattr(import_module(module), function)(argv)
        except SystemExit as error:
            if isinstance(error.code, str) or error.code == USAGE_EXIT_CODE:
                message = error.code if isinstance(error.code, str) else "invalid entrypoint arguments"
                raise CliError(message, code=USAGE_EXIT_CODE) from error
            raise
        except Exception as error:
            # The entrypoint loads rl-garden if it needs it. Never import it merely
            # to discover the error type (in particular while listing commands).
            config = sys.modules.get("rl_garden.common.effective_config")
            if config is not None and isinstance(error, getattr(config, "ConfigError", ())):
                raise CliError(str(error), code=USAGE_EXIT_CODE) from error
            raise
        return Result({}, "", int(code or SUCCESS_EXIT_CODE), rendered=True)


COMMANDS = [
    LearningCommand("train", "Train a real-world policy", "real_world_rl.training.__main__:main"),
    LearningCommand("record", "Record teleoperated demonstrations", "real_world_rl.demos.__main__:main"),
    LearningCommand("deploy", "Deploy a trained policy", "real_world_rl.deploy:main_cli"),
    LearningCommand("collect-reward", "Collect reward-classifier data", "real_world_rl.reward.collect_data:main_cli"),
]
