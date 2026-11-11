"""Native device commands, usable without real_world_rl or torch."""
from contextlib import redirect_stdout
from dataclasses import dataclass
from importlib import import_module

from .command import Result
from .errors import SUCCESS_EXIT_CODE
from .main import main as invoke


@dataclass(frozen=True)
class DeviceCommand:
    name: str
    module: str
    help: str
    supports_json: bool = False
    station_attribute: str | None = None

    def register(self, parser):
        import_module(f"{__package__}.cmd_{self.name.replace('-', '_')}").build_parser(parser)

    def run(self, args, ctx):
        # A command-local station value takes precedence over the root option.
        if self.station_attribute and getattr(args, self.station_attribute, None) is None:
            setattr(args, self.station_attribute, args.station)
        with redirect_stdout(ctx.stdout):
            code = import_module(f"{__package__}.cmd_{self.name.replace('-', '_')}").run(args, ctx)
        return Result({}, "", int(code or SUCCESS_EXIT_CODE), rendered=True)


COMMANDS = [
    DeviceCommand("setup", "robot_infra.setup.__main__", "Discover and configure a station", station_attribute="from_path"),
    DeviceCommand("daemon", "robot_infra.server.daemon", "Serve robots over the network", station_attribute="config"),
    DeviceCommand("supervise", "robot_infra.supervise.supervisor", "Supervise a robot process"),
    DeviceCommand("discover", "robot_infra.discovery.__main__", "Discover visible hardware", station_attribute="config"),
    DeviceCommand("doctor", "robot_infra.doctor.cli", "Layered robot diagnostic", supports_json=True, station_attribute="config"),
    DeviceCommand("gello-calibrate", "robot_infra.devices.backends.teleops.gello_calibrate", "Calibrate a GELLO leader arm"),
]


def main(argv=None):
    return invoke(argv, commands=COMMANDS, prog="robot-infra", description="Robot device infrastructure",
                  global_options=("station", "robot", "mock", "json", "yes"))
