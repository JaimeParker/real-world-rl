"""Native device commands, composed into ``rwrl`` by ``real_world_rl.cli``; importing them
needs neither torch nor the learning side."""
from contextlib import redirect_stdout
from dataclasses import dataclass
from importlib import import_module

from .command import Result
from .errors import SUCCESS_EXIT_CODE


@dataclass(frozen=True)
class DeviceCommand:
    name: str
    module: str
    help: str
    supports_json: bool = False
    station_attribute: str | None = None
    # Name of a function in the command module that returns a Result for --json, instead of printing.
    json_result: str | None = None

    def register(self, parser):
        import_module(f"{__package__}.cmd_{self.name.replace('-', '_')}").build_parser(parser)

    def run(self, args, ctx):
        # A command-local station value takes precedence over the root option.
        if self.station_attribute and getattr(args, self.station_attribute, None) is None:
            setattr(args, self.station_attribute, args.station)
        module = import_module(f"{__package__}.cmd_{self.name.replace('-', '_')}")
        if self.json_result and args.json:
            return getattr(module, self.json_result)(args, ctx)
        with redirect_stdout(ctx.stdout):
            code = module.run(args, ctx)
        return Result({}, "", int(code or SUCCESS_EXIT_CODE), rendered=True)


COMMANDS = [
    DeviceCommand("setup", "robot_infra.setup.command", "Discover and configure a station", station_attribute="from_path"),
    DeviceCommand("daemon", "robot_infra.server.daemon", "Serve robots over the network", station_attribute="config"),
    DeviceCommand("supervise", "robot_infra.supervise.supervisor", "Supervise a robot process"),
    DeviceCommand("discover", "robot_infra.discovery.report", "Discover visible hardware", supports_json=True, station_attribute="config",
                  json_result="run_json"),
    DeviceCommand("gello-calibrate", "robot_infra.devices.backends.teleops.gello_calibrate", "Calibrate a GELLO leader arm"),
]
