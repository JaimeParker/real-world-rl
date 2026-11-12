"""``rwrl archive inspect``: the archive inspector as a registry command, with stop-settle exit codes."""
from __future__ import annotations

import json
import math
import os
from pathlib import Path
import subprocess
import sys

import pytest

import test_inspect_teleop_archive as scripted_archive
from test_inspect_teleop_archive import (CARTESIAN_CONTRACT, JOINT_CONTRACT, LEADER_STOP_STEP, NUM_STEPS,
                                         START_S, step_time, write_archive)
from real_world_rl.cli import main
from real_world_rl.cli.errors import (SUCCESS_EXIT_CODE, USAGE_EXIT_CODE, STOP_TRACE_FAIL_EXIT_CODE,
                                      STOP_TRACE_BLOCKED_EXIT_CODE)
from real_world_rl.demos import archive_inspect

ROOT = Path(__file__).resolve().parents[1]
STOP_S = step_time(LEADER_STOP_STEP)
OUTSIDE_S = START_S - 1.0


def moving_offsets_mm() -> list[tuple[float, float, float]]:
    return [(10.0 * step, 0.0, 0.0) for step in range(NUM_STEPS)]  # 100 mm/s to the last sample


def archive_for(trajectory: str, tmp_path: Path, monkeypatch) -> Path:
    if trajectory == "moving":
        monkeypatch.setattr(scripted_archive, "scripted_offsets_mm", moving_offsets_mm)
    contract = JOINT_CONTRACT if trajectory == "joint-space" else CARTESIAN_CONTRACT
    return write_archive(tmp_path / "demo.trajectory.sqlite3", contract=contract)


def json_result(capsys) -> dict:
    (line,) = capsys.readouterr().out.splitlines()
    return json.loads(line)


def test_text_output_is_the_example_script_output(tmp_path, capsys):
    archive = write_archive(tmp_path / "demo.trajectory.sqlite3", contract=CARTESIAN_CONTRACT)
    archive_inspect.main([str(archive), "--stop-time-s", repr(STOP_S)])
    expected = capsys.readouterr().out
    assert main(["archive", "inspect", str(archive), "--stop-time-s", repr(STOP_S)]) == SUCCESS_EXIT_CODE
    assert capsys.readouterr().out == expected
    assert "status" not in json.loads(expected)


def test_json_output_is_the_summary_with_a_status(tmp_path, capsys):
    archive = write_archive(tmp_path / "demo.trajectory.sqlite3", contract=CARTESIAN_CONTRACT)
    assert main(["--json", "archive", "inspect", str(archive), "--drift-window-s", "0.85"]) == SUCCESS_EXIT_CODE
    payload = json_result(capsys)
    assert payload.pop("status") == "REPORT"
    summary = archive_inspect.inspect_archive(archive, drift_window_s=0.85)
    assert payload == json.loads(json.dumps(summary))
    assert payload["settle"] == [] and payload["metric_settings"]["drift_window_s"] == 0.85


def test_json_output_replaces_non_finite_numbers(monkeypatch, capsys):
    summary = {"settle": [], "episodes": [{"segments": [{"max_drift_mm": math.nan, "jump_mm": math.inf}]}]}
    monkeypatch.setattr(archive_inspect, "inspect_archive", lambda *args, **kwargs: summary)
    assert main(["--json", "archive", "inspect", "unused.sqlite3"]) == SUCCESS_EXIT_CODE
    segment = json_result(capsys)["episodes"][0]["segments"][0]
    assert segment == {"max_drift_mm": None, "jump_mm": None}
    assert main(["archive", "inspect", "unused.sqlite3"]) == SUCCESS_EXIT_CODE
    assert "NaN" in capsys.readouterr().out  # text mode prints what the example script prints


@pytest.mark.parametrize("trajectory,stop_times_s,code,status", [
    ("settling", [STOP_S], SUCCESS_EXIT_CODE, "PASS"),
    ("moving", [STOP_S], STOP_TRACE_FAIL_EXIT_CODE, "FAIL"),
    ("settling", [OUTSIDE_S], STOP_TRACE_BLOCKED_EXIT_CODE, "BLOCKED"),
    ("joint-space", [STOP_S], STOP_TRACE_BLOCKED_EXIT_CODE, "BLOCKED"),
    ("moving", [STOP_S, OUTSIDE_S], STOP_TRACE_BLOCKED_EXIT_CODE, "BLOCKED"),
])
def test_stop_times_set_the_exit_code(tmp_path, monkeypatch, capsys, trajectory, stop_times_s, code, status):
    archive = archive_for(trajectory, tmp_path, monkeypatch)
    stops = [token for stop_s in stop_times_s for token in ("--stop-time-s", repr(stop_s))]
    assert main(["--json", "archive", "inspect", str(archive), *stops]) == code
    payload = json_result(capsys)
    assert payload["status"] == status
    assert len(payload["settle"]) == len(stop_times_s)
    if status == "FAIL":
        (settle,) = payload["settle"]
        assert settle["episode_id"] == "episode-0" and settle["settled"] is False


def test_usage_errors_exit_with_the_usage_code(tmp_path, capsys):
    archive = write_archive(tmp_path / "demo.trajectory.sqlite3", contract=CARTESIAN_CONTRACT)
    not_sqlite = tmp_path / "notes.sqlite3"
    not_sqlite.write_text("not a database\n" * 100)
    existing = tmp_path / "measurements.csv"
    existing.write_text("earlier evidence\n")
    for argv in (["archive", "inspect"],
                 ["archive", "inspect", str(tmp_path / "missing.sqlite3")],
                 ["archive", "inspect", str(not_sqlite)],
                 ["archive", "inspect", str(archive), "--measurements", str(existing)],
                 ["archive", "inspect", str(archive), "--csv", str(tmp_path / "steps.txt")],
                 ["archive", "inspect", str(archive), "--settle-hold-s", "0"]):
        assert main(["--json", *argv]) == USAGE_EXIT_CODE, argv
        assert "error" in json_result(capsys)
    assert existing.read_text() == "earlier evidence\n"


@pytest.mark.parametrize("option", [["--station", "x"], ["--robot", "left"], ["--preset", "p.yaml"],
                                    ["--direct"], ["--mock"], ["--yes"]])
def test_robot_options_are_not_applicable(tmp_path, capsys, option):
    assert main([*option, "archive", "inspect", str(tmp_path / "unused.sqlite3")]) == USAGE_EXIT_CODE
    assert f"{option[0]} not applicable to rwrl archive inspect" in capsys.readouterr().err


def test_the_example_script_remains_an_alias():
    path = os.pathsep.join(filter(None, (str(ROOT), os.environ.get("PYTHONPATH"))))
    result = subprocess.run([sys.executable, str(ROOT / "examples/inspect_teleop_archive.py"), "--help"],
                            cwd=ROOT, env={**os.environ, "PYTHONPATH": path},
                            capture_output=True, text=True, check=False)
    assert result.returncode == 0, result.stderr
    assert "--stop-time-s" in result.stdout and "Sampling assumption" in result.stdout
