"""The device doctor as a library: run the layered checks on every robot of a station and format
the results. ``rwrl doctor`` runs the same checks through ``real_world_rl.cli.doctor_probe``;
``rwrl setup`` and the daemon preflight use the formatting here."""
from __future__ import annotations

import argparse
from contextlib import nullcontext, redirect_stdout
import dataclasses
import json
import shlex
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path

from real_world_rl.cli.command import affirmative, doctor_envelope
from real_world_rl.cli.errors import FAILURE_EXIT_CODE, SUCCESS_EXIT_CODE
from real_world_rl.cli.options import add_station_argument
from real_world_rl.cli.render import BOLD, BOLD_GREEN, BOLD_RED, paint, status_style, supports_color
from real_world_rl.discovery.autoconfig import (
    fill_from_env,
    resolve_station_config_path,
)
from real_world_rl.devices.backends.arms.franky import server_version_cross_check
from real_world_rl.doctor.checks import (
    LIBFRANKA_LIBRARY_VERSION,
    LIBFRANKA_SERVER_VERSION,
    CheckResult,
    exit_code,
    run_doctor,
    summarize,
)
from real_world_rl.doctor.deps import franky_wheel_install_argv

_LAYER_COUNT = 5
_JSON_EXCLUDED_FIELDS = ("autofix", "details")
_INSTALL_PROMPT = "Install now? [y/N] "


def _status(result: CheckResult) -> str:
    if result.skipped:
        return "SKIP"
    if not result.ok:
        return "FAIL"
    return "WARN" if result.warning else "OK"


def format_text(results: list[CheckResult], *, color: bool = False) -> str:
    lines = []
    for result in results:
        status = _status(result)
        layer = paint(f"[{result.layer}/{_LAYER_COUNT}]", BOLD, enabled=color)
        painted = paint(status, status_style(status), enabled=color)
        lines.append(f"{layer} {result.name} ... {painted}  {result.message}")
        if status in ("FAIL", "WARN") and result.hint:
            lines.append(f"    hint: {result.hint}")
    ok, failed, skipped = summarize(results)
    warnings = sum(_status(result) == "WARN" for result in results)
    warned = f" ({warnings} with warnings)" if warnings else ""
    summary = f"summary: {ok} ok{warned}, {failed} failed, {skipped} skipped"
    lines.append(paint(summary, BOLD_RED if failed else BOLD_GREEN, enabled=color))
    return "\n".join(lines)


def _result_dicts(results: list[CheckResult]) -> list[dict]:
    return [
        {f.name: getattr(result, f.name) for f in dataclasses.fields(result) if f.name not in _JSON_EXCLUDED_FIELDS}
        for result in results
    ]


def format_json(results: list[CheckResult]) -> str:
    return format_station_json({"robot": results})


def load_station(path: str | Path):
    """The station with its upper-layer sections (``teleop_profiles``) validated, as every rwrl
    command loads it; imported on use so this module stays light."""
    from real_world_rl.station import load_station as load_with_sections

    return load_with_sections(path)


def doctor_station(
    path: str | Path, *, robot: str | None = None, mock: bool = False, fix: bool = False
) -> dict[str, list[CheckResult]]:
    """Run the doctor on ``robot`` of a station yaml, or on every robot in order. A robot with a
    ``daemon:`` block needs grpc."""
    station = load_station(path)
    names = [robot] if robot is not None else list(station.robots)
    results_by_robot = {}
    for name in names:
        station_robot = station.select(name)
        results_by_robot[name] = run_doctor(
            fill_from_env(station_robot.config),
            mock=mock,
            fix=fix,
            config_path=str(path),
            serves_daemon=station_robot.daemon_declared,
        )
    return results_by_robot


def station_exit_code(results_by_robot: dict[str, list[CheckResult]]) -> int:
    return max((exit_code(results) for results in results_by_robot.values()), default=SUCCESS_EXIT_CODE)


def format_station_text(results_by_robot: dict[str, list[CheckResult]], *, color: bool = False) -> str:
    """One robot: exactly ``format_text``. Several: a header and summary per robot, then an overall line.
    ``color`` adds terminal styling only; the text without escape codes is the same either way."""
    if len(results_by_robot) == 1:
        return format_text(next(iter(results_by_robot.values())), color=color)
    blocks = [
        paint(f"== robot '{name}' ==", BOLD, enabled=color) + "\n" + format_text(results, color=color)
        for name, results in results_by_robot.items()
    ]
    failing = [name for name, results in results_by_robot.items() if exit_code(results)]
    overall = f"overall: {len(results_by_robot)} robots, " + (
        f"failures on {failing}" if failing else "all passed"
    )
    return "\n".join([*blocks, paint(overall, BOLD_RED if failing else BOLD_GREEN, enabled=color)])


def _station_payload(results_by_robot: dict[str, list[CheckResult]]) -> list | dict:
    if len(results_by_robot) == 1:
        return _result_dicts(next(iter(results_by_robot.values())))
    return {name: _result_dicts(results) for name, results in results_by_robot.items()}


def format_station_json(results_by_robot: dict[str, list[CheckResult]]) -> str:
    """Envelope around the original one-robot list or multi-robot map."""
    return json.dumps(doctor_envelope(
        code=station_exit_code(results_by_robot), checks=[], repairs=[],
        results=_station_payload(results_by_robot),
    ))


# -- --install-sdk ---------------------------------------------------------------------------


@dataclass
class LibfrankaMismatch:
    robot: str
    server_version: int
    library_version: int

    @property
    def install_argv(self) -> list[str]:
        return franky_wheel_install_argv(self.server_version)


def libfranka_mismatches(results_by_robot: dict[str, list[CheckResult]]) -> list[LibfrankaMismatch]:
    """Every failed handshake that named the robot server and library versions."""
    return [
        LibfrankaMismatch(name, result.details[LIBFRANKA_SERVER_VERSION], result.details[LIBFRANKA_LIBRARY_VERSION])
        for name, results in results_by_robot.items()
        for result in results
        if result.name == "handshake" and not result.ok and LIBFRANKA_SERVER_VERSION in result.details
    ]


def format_install_sdk_json(results_by_robot: dict[str, list[CheckResult]]) -> str:
    """Envelope around the original results/install_sdk report object."""
    install = [
        {
            "robot": mismatch.robot,
            "server_version": mismatch.server_version,
            "library_version": mismatch.library_version,
            "command": shlex.join(mismatch.install_argv),
        }
        for mismatch in libfranka_mismatches(results_by_robot)
    ]
    return json.dumps(doctor_envelope(
        code=station_exit_code(results_by_robot), checks=[], repairs=[],
        results={"results": _station_payload(results_by_robot), "install_sdk": install},
    ))


def _confirmed(prompt: str) -> bool:
    try:
        answer = input(prompt)
    except EOFError:
        return False
    return affirmative(answer)


def doctor_rerun_argv(path: str | Path, robot: str) -> list[str]:
    return [sys.executable, "-m", "real_world_rl.cli", "doctor", "--station", str(path), "--robot", robot]


def install_franky_wheel(
    path: str | Path, results_by_robot: dict[str, list[CheckResult]], *, assume_yes: bool = False
) -> int:
    """After a libfranka mismatch: show the server version, where to verify it and the pip command,
    install on confirmation (or ``assume_yes``), then re-run the doctor on those robots in a fresh
    interpreter, because this one keeps the old libfranka loaded. Returns the exit code."""
    code = station_exit_code(results_by_robot)
    mismatches = libfranka_mismatches(results_by_robot)
    if not mismatches:
        print("--install-sdk: no libfranka version mismatch found; nothing to install")
        return code
    server_versions = sorted({mismatch.server_version for mismatch in mismatches})
    if len(server_versions) > 1:
        print(
            f"--install-sdk: the robots report different robot server versions {server_versions}; one Python "
            "environment holds one franky wheel, so install on each robot PC by hand:"
        )
        for mismatch in mismatches:
            print(f"  {mismatch.robot}: {shlex.join(mismatch.install_argv)}")
        return code
    first = mismatches[0]
    robots = ", ".join(mismatch.robot for mismatch in mismatches)
    print(
        f"--install-sdk: robot server version {first.server_version} detected on {robots} "
        f"(installed library version {first.library_version})"
    )
    print(f"{server_version_cross_check(first.server_version)} before installing")
    print(f"command: {shlex.join(first.install_argv)}")
    if not assume_yes and not _confirmed(_INSTALL_PROMPT):
        print("not installed")
        return code
    completed = subprocess.run(first.install_argv)
    if completed.returncode != 0:
        print(f"pip exited with code {completed.returncode}; the handshake was not re-checked", file=sys.stderr)
        return FAILURE_EXIT_CODE
    print("installed; re-running the doctor in a fresh interpreter")
    rerun_codes = [subprocess.run(doctor_rerun_argv(path, mismatch.robot)).returncode for mismatch in mismatches]
    mismatched = {mismatch.robot for mismatch in mismatches}
    other_codes = [exit_code(results) for name, results in results_by_robot.items() if name not in mismatched]
    return max([*rerun_codes, *other_codes], default=SUCCESS_EXIT_CODE)




def build_parser() -> argparse.ArgumentParser:
    """Arguments of ``main``, the library entry of the device doctor (it is no command)."""
    parser = argparse.ArgumentParser(prog="device doctor", description="Layered robot diagnostic")
    add_station_argument(
        parser, dest="config",
        help="robot or station yaml; default: $RWRL_STATION, ./local/station.yaml, then "
        "~/.config/rwrl/station.yaml",
    )
    parser.add_argument("--robot", default=None, help="check only this robot of a station (default: all)")
    parser.add_argument("--mock", action="store_true", help="swap every backend for its fake")
    parser.add_argument("--fix", action="store_true", help="run autofixes for failed checks")
    parser.add_argument("--json", action="store_true", help="machine-readable output")
    parser.add_argument(
        "--install-sdk",
        action="store_true",
        help="after a libfranka version mismatch, offer to pip install the franky wheel for the robot server "
        "version (asks first; with --json only reports the command)",
    )
    parser.add_argument("--yes", action="store_true", help="skip confirmation; preflight checks still run")
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    return run(parser.parse_args(argv), None)


def run(args, ctx) -> int:
    parser = getattr(args, "_parser", None) or build_parser()
    try:
        path = resolve_station_config_path(args.config)
    except FileNotFoundError as error:
        parser.error(str(error))
    with redirect_stdout(sys.stderr) if args.json else nullcontext():
        results_by_robot = doctor_station(path, robot=args.robot, mock=args.mock, fix=args.fix)
    if args.json:
        print(format_install_sdk_json(results_by_robot) if args.install_sdk else format_station_json(results_by_robot))
        return station_exit_code(results_by_robot)
    print(format_station_text(results_by_robot, color=supports_color(sys.stdout)))
    if args.install_sdk:
        return install_franky_wheel(path, results_by_robot, assume_yes=args.yes)
    return station_exit_code(results_by_robot)
