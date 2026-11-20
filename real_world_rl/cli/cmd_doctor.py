"""Doctor command registration; the implementation is real_world_rl.doctor, imported on use."""
from pathlib import Path

from .command import doctor_envelope

from .errors import CliError, USAGE_EXIT_CODE


class DoctorCommand:
    name = "doctor"
    help = "Inspect the host, project environment and devices"
    trailing_globals = ("station", "robot", "preset", "mock", "json", "yes")
    parser_options = {
        "allow_abbrev": False,
        "description": "Robot-host diagnostics: inspect, preview, confirm, repair, then inspect again.",
    }

    def register(self, parser):
        parser.add_argument("--fix", action="store_true")
        parser.add_argument(
            "--install-sdk", action="store_true",
            help="repair only a handshake-confirmed Franky mismatch",
        )
        parser.add_argument(
            "--report-dir", type=Path,
            help="report base directory (overrides RWRL_DOCTOR_REPORT_DIR)",
        )
        parser.add_argument("--link", metavar="HOST:PORT", help="probe the client route to the robot daemon")
        parser.add_argument("--ssh", metavar="[USER@]HOST", help="NUC ssh destination (USER@HOST or a ~/.ssh/config alias): probe the SSH host and local forwarding")
        parser.add_argument("--remote-bind", metavar="HOST:PORT",
                            help="daemon address as seen on the NUC, for --ssh forwarding checks (default 127.0.0.1:50051)")
        parser.add_argument("--policy", metavar="HOST:PORT", help="connect to openpi and read metadata")
        parser.add_argument("--token", metavar="TOKEN", help="authenticate the daemon link probes")
        parser.error_factory = lambda message: self.usage_error(parser, message)

    @staticmethod
    def usage_error(parser, message, report_directory=None):
        from real_world_rl.doctor.repair import redact
        message = redact(message)
        return CliError(
            message, code=USAGE_EXIT_CODE,
            payload={"schema_version": 1, "error": message, "report_directory": report_directory},
            diagnostic=parser.format_usage() + f"{parser.prog}: error: {message}\n",
        )

    def run(self, args, ctx):
        from real_world_rl.doctor.report import run
        result = run(ctx, lambda message, directory=None: self.usage_error(args._parser, message, directory))
        result.payload = doctor_envelope(code=result.exit_code, **result.payload)
        return result


DOCTOR = DoctorCommand()
