"""Robot station commands composed with the torch-free device CLI."""
from robot_infra.cli import COMMANDS as DEVICE_COMMANDS
from robot_infra.cli.main import build_parser as _build_parser, main as _main
from .command import Context, Result  # noqa: F401
from .cmd_primitives import STATUS, Q, TCP, STOP, RECOVER
from .cmd_home import HOME
from .cmd_gripper import GRIPPER
from .cmd_doctor import DOCTOR
from .cmd_teleop import TELEOP
from .cmd_workspace import WORKSPACE
from .cmd_learning import COMMANDS as LEARNING_COMMANDS

COMMANDS = [STATUS, Q, TCP, STOP, RECOVER, HOME, GRIPPER, DOCTOR,
            *(command for command in DEVICE_COMMANDS if command.name != "doctor"),
            TELEOP, WORKSPACE, *LEARNING_COMMANDS]


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
