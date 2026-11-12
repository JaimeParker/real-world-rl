"""Offline recorder-archive evidence: ``rwrl archive inspect`` (no station, no device)."""
import math

from real_world_rl.demos.archive_args import add_inspect_arguments
from .command import Result
from .errors import (CliError, SUCCESS_EXIT_CODE, USAGE_EXIT_CODE,
                     STOP_TRACE_FAIL_EXIT_CODE, STOP_TRACE_BLOCKED_EXIT_CODE)

# Global options that select or move a robot; reading an archive uses none of them.
NOT_APPLICABLE_OPTIONS = ("station", "robot", "preset", "direct", "mock", "yes")
STATUS_EXIT_CODES = {"REPORT": SUCCESS_EXIT_CODE, "PASS": SUCCESS_EXIT_CODE,
                     "FAIL": STOP_TRACE_FAIL_EXIT_CODE, "BLOCKED": STOP_TRACE_BLOCKED_EXIT_CODE}


def settle_status(settles) -> str:
    """``REPORT`` without stop times; otherwise ``BLOCKED`` when a stop time has no TCP episode
    spanning it, else ``FAIL`` when one never settled, else ``PASS``."""
    if not settles:
        return "REPORT"
    if any(entry["episode_id"] is None for entry in settles):
        return "BLOCKED"
    if not all(entry["settled"] for entry in settles):
        return "FAIL"
    return "PASS"


def finite_or_none(value):
    """``value`` with every NaN or infinite float replaced by ``None`` (strict JSON)."""
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, dict):
        return {key: finite_or_none(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [finite_or_none(item) for item in value]
    return value


def inspect(args, ctx):
    for option in NOT_APPLICABLE_OPTIONS:
        if getattr(args, option, None) not in (None, False):
            raise CliError(f"--{option} not applicable to rwrl archive inspect", code=USAGE_EXIT_CODE)
    import sqlite3
    from .cmd_learning import learning_dependencies
    # Torch loads here, never while the parser is built.
    with learning_dependencies("archive inspect"):
        from real_world_rl.demos.archive_inspect import format_summary, inspect_archive
    try:
        summary = inspect_archive(args.archive, args.csv, measurements_path=args.measurements,
                                  stop_times_s=args.stop_time_s, drift_window_s=args.drift_window_s,
                                  settle_speed_mm_s=args.settle_speed_mm_s, settle_hold_s=args.settle_hold_s)
    except (OSError, ValueError, sqlite3.Error) as error:
        raise CliError(f"cannot inspect {args.archive}: {error}", code=USAGE_EXIT_CODE) from error
    status = settle_status(summary["settle"])
    payload = {**finite_or_none(summary), "status": status}
    return Result(payload, format_summary(summary), STATUS_EXIT_CODES[status], ensure_ascii=False)


class ArchiveCommand:
    name = "archive"
    help = "inspect recorder archives offline (no station or robot)"

    def register(self, parser):
        sub = parser.add_subparsers(dest="archive_command", required=True)
        inspect_parser = sub.add_parser(
            "inspect", help="hold drift, take-over jump and settle evidence from a recorder SQLite archive",
            description="Read a recorder SQLite archive read-only. With --stop-time-s the exit code is "
                        f"{SUCCESS_EXIT_CODE} when every stop settled, {STOP_TRACE_FAIL_EXIT_CODE} when one "
                        f"did not, {STOP_TRACE_BLOCKED_EXIT_CODE} when one lacks a spanning TCP episode.")
        add_inspect_arguments(inspect_parser)

    def run(self, args, ctx):
        operations = {"inspect": inspect}
        return operations[args.archive_command](args, ctx)


ARCHIVE = ArchiveCommand()
