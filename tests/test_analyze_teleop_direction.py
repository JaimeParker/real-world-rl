"""scripts/analyze_teleop_direction.py on synthetic diagnostics logs."""
from __future__ import annotations

import importlib.util
import json
import math
import sys
from pathlib import Path

import pytest

SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "analyze_teleop_direction.py"
spec = importlib.util.spec_from_file_location("analyze_teleop_direction", SCRIPT)
analyze = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = analyze  # dataclasses resolve their module
spec.loader.exec_module(analyze)

QUAT = [0.0, 0.0, 0.0, 1.0]


def _row(t, kind, measured, accepted=None, command=None, step=0):
    teleop = None
    if command is not None:
        teleop = {"intervened": True, "input": {"base_input_after_filter": list(command)}}
    return {"schema_version": "1", "monotonic_s": t, "kind": kind, "step": step,
            "measured_base_pose": list(measured) + QUAT,
            "device_accepted_pose": list(accepted or measured) + QUAT, "teleop": teleop}


def _session(direction_deg=0.0, lag_m=0.008, coast_m=0.001, command=(1, 0, 0, 0, 0, 0), steps=10, dt=0.1):
    """A +x command for ``steps`` steps; the arm moves 10 mm/step at ``direction_deg`` off x,
    trailing the accepted target by ``lag_m``, then coasts ``coast_m`` over hold rows."""
    rows, pos, t = [], [0.5, 0.0, 0.3], 0.0
    rows.append(_row(t, "step", pos, step=0))
    heading = math.radians(direction_deg)
    for i in range(1, steps + 1):
        t += dt
        pos = [pos[0] + 0.01 * math.cos(heading), pos[1] + 0.01 * math.sin(heading), pos[2]]
        rows.append(_row(t, "step", pos, accepted=[pos[0] + lag_m, pos[1], pos[2]], command=command, step=i))
    end = pos
    for k in range(1, 6):
        t += 0.1
        rows.append(_row(t, "hold", [end[0] + coast_m * k / 5, end[1], end[2]]))
    t += 0.1
    rows.append(_row(t, "step", [end[0] + coast_m, end[1], end[2]], step=steps + 1))
    return rows


def test_straight_segment_passes_with_metrics():
    (segment,) = analyze.analyze_rows(_session())
    assert segment.axis == "+x" and segment.steps == 10 and segment.passed
    assert segment.angle_deg == pytest.approx(0.0, abs=1e-6)
    assert segment.speed_mm_s == pytest.approx(100.0)
    assert segment.lead_at_release_mm == pytest.approx(8.0)
    assert segment.coast_mm == pytest.approx(1.0)


def test_direction_error_and_coast_fail():
    (bent,) = analyze.analyze_rows(_session(direction_deg=15.0))
    assert bent.angle_deg == pytest.approx(15.0) and not bent.passed
    (coasting,) = analyze.analyze_rows(_session(coast_m=0.005))
    assert coasting.coast_mm == pytest.approx(5.0) and not coasting.passed


def test_rotation_and_short_segments_are_not_translation_isolated():
    assert analyze.analyze_rows(_session(command=(1, 0, 0, 0.2, 0, 0))) == []
    assert analyze.analyze_rows(_session(steps=3)) == []


def test_axis_change_splits_segments():
    rows = _session()
    for row in rows[6:11]:
        row["teleop"]["input"]["base_input_after_filter"] = [0, -1, 0, 0, 0, 0]
    segments = analyze.analyze_rows(rows)
    assert [s.axis for s in segments] == ["+x", "-y"]


def test_cli_reads_a_session_directory(tmp_path, capsys):
    session = tmp_path / "translation-isolated-01.session-abc"
    session.mkdir()
    (session / "diagnostics-1.jsonl").write_text("\n".join(json.dumps(r) for r in _session()) + "\n")
    (session / "resolved-config.yaml").write_text("spacemouse_mode: serl\n")
    assert analyze.main([str(session)]) == 0
    out = capsys.readouterr().out
    assert "spacemouse_mode=serl" in out and "PASS" in out and "1 segments, 0 failed" in out
    assert analyze.main([str(session), "--max-lead-mm", "0.5"]) == 1
    empty = tmp_path / "empty.jsonl"
    empty.write_text("")
    assert analyze.main([str(empty)]) == 2
