from __future__ import annotations

import argparse
import dataclasses
import json
from pathlib import Path

from robot_infra.discovery.autoconfig import (
    fill_from_env,
    load_station,
    resolve_station_config_path,
)
from robot_infra.doctor.checks import CheckResult, exit_code, run_doctor, summarize

_LAYER_COUNT = 5


def _status(result: CheckResult) -> str:
    if result.skipped:
        return "SKIP"
    return "OK" if result.ok else "FAIL"


def format_text(results: list[CheckResult]) -> str:
    lines = []
    for result in results:
        lines.append(
            f"[{result.layer}/{_LAYER_COUNT}] {result.name} ... {_status(result)}  {result.message}"
        )
        if _status(result) == "FAIL" and result.hint:
            lines.append(f"    hint: {result.hint}")
    ok, failed, skipped = summarize(results)
    lines.append(f"summary: {ok} ok, {failed} failed, {skipped} skipped")
    return "\n".join(lines)


def _result_dicts(results: list[CheckResult]) -> list[dict]:
    return [
        {f.name: getattr(result, f.name) for f in dataclasses.fields(result) if f.name != "autofix"}
        for result in results
    ]


def format_json(results: list[CheckResult]) -> str:
    return json.dumps(_result_dicts(results))


def doctor_station(
    path: str | Path, *, robot: str | None = None, mock: bool = False, fix: bool = False
) -> dict[str, list[CheckResult]]:
    """Run the doctor on ``robot`` of a station yaml, or on every robot in order."""
    station = load_station(path)
    names = [robot] if robot is not None else list(station.robots)
    return {
        name: run_doctor(fill_from_env(station.select(name).config), mock=mock, fix=fix) for name in names
    }


def station_exit_code(results_by_robot: dict[str, list[CheckResult]]) -> int:
    return max((exit_code(results) for results in results_by_robot.values()), default=0)


def format_station_text(results_by_robot: dict[str, list[CheckResult]]) -> str:
    """One robot: exactly ``format_text``. Several: a header and summary per robot, then an overall line."""
    if len(results_by_robot) == 1:
        return format_text(next(iter(results_by_robot.values())))
    blocks = [f"== robot '{name}' ==\n{format_text(results)}" for name, results in results_by_robot.items()]
    failing = [name for name, results in results_by_robot.items() if exit_code(results)]
    overall = f"overall: {len(results_by_robot)} robots, " + (
        f"failures on {failing}" if failing else "all passed"
    )
    return "\n".join([*blocks, overall])


def format_station_json(results_by_robot: dict[str, list[CheckResult]]) -> str:
    """One robot: exactly ``format_json``. Several: ``{robot name: [result, ...]}``."""
    if len(results_by_robot) == 1:
        return format_json(next(iter(results_by_robot.values())))
    return json.dumps({name: _result_dicts(results) for name, results in results_by_robot.items()})


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="robot_infra.doctor", description="Layered robot diagnostic")
    parser.add_argument(
        "--config",
        default=None,
        help="robot or station yaml; default: $ROBOT_INFRA_CONFIG, ./local/station.yaml, then "
        "~/.config/robot_infra/station.yaml",
    )
    parser.add_argument("--robot", default=None, help="check only this robot of a station (default: all)")
    parser.add_argument("--mock", action="store_true", help="swap every backend for its fake")
    parser.add_argument("--fix", action="store_true", help="run autofixes for failed checks")
    parser.add_argument("--json", action="store_true", help="machine-readable output")
    args = parser.parse_args(argv)
    try:
        path = resolve_station_config_path(args.config)
    except FileNotFoundError as error:
        parser.error(str(error))
    results_by_robot = doctor_station(path, robot=args.robot, mock=args.mock, fix=args.fix)
    print(format_station_json(results_by_robot) if args.json else format_station_text(results_by_robot))
    return station_exit_code(results_by_robot)
