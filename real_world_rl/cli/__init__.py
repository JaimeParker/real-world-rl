"""``rwrl``: robot station commands composed with the torch-free device commands."""
from robot_infra.cli import COMMANDS as DEVICE_COMMANDS
from robot_infra.cli.main import build_parser as _build_parser, main as _main
from .command import Context, Result  # noqa: F401
from .cmd_primitives import STATUS, Q, TCP, STOP, RECOVER
from .cmd_home import HOME
from .cmd_gripper import GRIPPER
from .cmd_camera import CAMERA
from .cmd_doctor import DOCTOR
from .cmd_teleop import TELEOP
from .cmd_workspace import WORKSPACE
from .cmd_archive import ARCHIVE
from .cmd_learning import COMMANDS as LEARNING_COMMANDS


class DeprecatedAlias:
    """A device command kept under its name with the same parser and implementation; only the help
    names its replacement."""

    def __init__(self, target, replacement):
        self.target = target
        self.name = target.name
        self.help = f"alias of {replacement} (will be removed)"
        self.parser_options = {"description": self.help}
        self.supports_json = getattr(target, "supports_json", True)

    def register(self, parser):
        self.target.register(parser)

    def run(self, args, ctx):
        return self.target.run(args, ctx)


# rwrl names these device commands by their teleop subcommand; the old names remain as aliases.
DEVICE_COMMAND_REPLACEMENTS = {"gello-calibrate": "teleop calibrate --backend gello"}


def _station_command(command):
    replacement = DEVICE_COMMAND_REPLACEMENTS.get(command.name)
    return command if replacement is None else DeprecatedAlias(command, replacement)


COMMANDS = [STATUS, Q, TCP, STOP, RECOVER, HOME, GRIPPER, CAMERA, DOCTOR,
            *(_station_command(command) for command in DEVICE_COMMANDS),
            TELEOP, WORKSPACE, ARCHIVE, *LEARNING_COMMANDS]


def build_parser():
    return _build_parser(commands=COMMANDS, prog="rwrl", description="Robot station hardware primitives")


def main(argv=None):
    def load_station(path):
        from real_world_rl.station import load_station as implementation
        return implementation(path)

    def open_robot(args):
        from .connect import open_robot as implementation
        return implementation(args)

    return _main(argv, commands=COMMANDS, prog="rwrl", description="Robot station hardware primitives",
                 station_loader=load_station, connection_factory=open_robot)
