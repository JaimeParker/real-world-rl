#!/usr/bin/env python3
"""Direction error and stopping lead of translation-isolated SpaceMouse teleop sessions.

Reads the control diagnostics a ``rwrl teleop`` / env session writes
(``<name>.session-<id>/diagnostics-*.jsonl``, one JSON row per control tick: ``kind`` step or
hold, ``monotonic_s``, ``device_accepted_pose`` and ``measured_base_pose`` as base-frame
``[x, y, z, qx, qy, qz, qw]``, ``teleop.intervened`` and ``teleop.input.base_input_after_filter``)
and splits the intervened steps into segments of one commanded translation direction: the base
frame command's translation has norm >= ``--min-input``, its rotation part is at most
``--max-rotation-input`` (translation-isolated) and its dominant signed axis does not change.
For every segment of at least ``--min-steps`` steps it reports:

- ``angle_deg``: angle between the mean commanded direction and the measured TCP displacement
  from the row before the segment to its last step;
- ``speed_mm_s``: that displacement along the commanded direction, per second;
- ``lead_at_release_mm``: distance from the last accepted target to the measured TCP at the
  segment's last step (how far the command leads the arm when the operator lets go);
- ``coast_mm``: how far the measured TCP still travels after the segment's last step, over the
  following ``--settle-s`` seconds of rows (step and hold): the stop the operator feels.

A segment passes when ``angle_deg < --max-angle-deg`` (10) and ``coast_mm < --max-lead-mm`` (3).
Exit status 0 when every reported segment passes, 1 otherwise, 2 when no segment was found.

    python scripts/analyze_teleop_direction.py local/sites/<site>/translation-isolated-01.session-*/
    python scripts/analyze_teleop_direction.py --json run.diagnostics.jsonl
"""

from __future__ import annotations

import argparse
import json
import math
import re
import sys
from collections.abc import Iterable, Sequence
from dataclasses import asdict, dataclass
from pathlib import Path

AXES = "xyz"


@dataclass
class Segment:
    source: str
    axis: str
    first_step: int
    last_step: int
    steps: int
    duration_s: float
    command_dir: list[float]
    displacement_mm: list[float]
    angle_deg: float
    speed_mm_s: float
    lead_at_release_mm: float
    coast_mm: float
    passed: bool


def _norm(v: Sequence[float]) -> float:
    return math.sqrt(sum(x * x for x in v))


def _sub(a: Sequence[float], b: Sequence[float]) -> list[float]:
    return [x - y for x, y in zip(a, b)]


def _angle_deg(a: Sequence[float], b: Sequence[float]) -> float:
    na, nb = _norm(a), _norm(b)
    if na < 1e-12 or nb < 1e-12:
        return float("nan")
    cos = sum(x * y for x, y in zip(a, b)) / (na * nb)
    return math.degrees(math.acos(max(-1.0, min(1.0, cos))))


def diagnostics_files(paths: Iterable[str | Path]) -> list[Path]:
    """The diagnostics jsonl files of the given session directories or files."""
    files: list[Path] = []
    for raw in paths:
        path = Path(raw)
        if path.is_dir():
            found = sorted(path.glob("diagnostics-*.jsonl"))
            if not found:
                raise FileNotFoundError(f"{path}: no diagnostics-*.jsonl")
            files += found
        elif path.is_file():
            files.append(path)
        else:
            raise FileNotFoundError(str(path))
    return files


def read_rows(path: Path) -> list[dict]:
    rows = []
    with path.open() as handle:
        for line in handle:
            line = line.strip()
            if line:
                rows.append(json.loads(line))
    return rows


def _command(row: dict) -> list[float] | None:
    teleop = row.get("teleop") or {}
    if not teleop.get("intervened"):
        return None
    command = (teleop.get("input") or {}).get("base_input_after_filter")
    if not command or len(command) < 3:
        return None
    return [float(x) for x in command]


def _isolated_axis(command: list[float], min_input: float, max_rotation_input: float) -> str | None:
    translation, rotation = command[:3], command[3:6]
    if _norm(translation) < min_input:
        return None
    if rotation and max(abs(x) for x in rotation) > max_rotation_input:
        return None
    index = max(range(3), key=lambda i: abs(translation[i]))
    return ("+" if translation[index] > 0 else "-") + AXES[index]


def analyze_rows(rows: Sequence[dict], *, source: str = "", min_input: float = 0.3,
                 max_rotation_input: float = 0.05, min_steps: int = 4, settle_s: float = 1.0,
                 max_angle_deg: float = 10.0, max_lead_mm: float = 3.0) -> list[Segment]:
    """Translation-isolated segments of one session's diagnostics rows (in file order)."""
    rows = [r for r in rows if r.get("measured_base_pose") and r.get("monotonic_s") is not None]
    steps = [i for i, r in enumerate(rows) if r.get("kind") == "step"]
    runs: list[tuple[str, list[int]]] = []
    current: tuple[str, list[int]] | None = None
    previous_step = None
    for index in steps:
        command = _command(rows[index])
        axis = None if command is None else _isolated_axis(command, min_input, max_rotation_input)
        contiguous = current is not None and previous_step is not None and current[1][-1] == previous_step
        if axis is not None and contiguous and current[0] == axis:
            current[1].append(index)
        else:
            if current is not None:
                runs.append(current)
            current = (axis, [index]) if axis is not None else None
        previous_step = index
    if current is not None:
        runs.append(current)

    segments = []
    for axis, members in runs:
        if len(members) < min_steps:
            continue
        first, last = members[0], members[-1]
        before = rows[first - 1] if first > 0 else rows[first]
        start_pos = before["measured_base_pose"][:3]
        end_row = rows[last]
        end_pos = end_row["measured_base_pose"][:3]
        displacement = _sub(end_pos, start_pos)
        commands = [_command(rows[i])[:3] for i in members]
        mean = [sum(c[k] for c in commands) / len(commands) for k in range(3)]
        direction = [x / _norm(mean) for x in mean]
        duration = float(end_row["monotonic_s"]) - float(before["monotonic_s"])
        along = sum(d * u for d, u in zip(displacement, direction))
        accepted = end_row.get("device_accepted_pose") or end_row.get("requested_pose") or end_pos
        lead = _norm(_sub(accepted[:3], end_pos))
        settle_end = float(end_row["monotonic_s"]) + settle_s
        coast_pos = end_pos
        for row in rows[last + 1:]:
            if float(row["monotonic_s"]) > settle_end:
                break
            coast_pos = row["measured_base_pose"][:3]
        coast = _norm(_sub(coast_pos, end_pos))
        angle = _angle_deg(displacement, direction)
        passed = bool(angle < max_angle_deg and coast * 1000.0 < max_lead_mm)
        segments.append(Segment(
            source=source, axis=axis, first_step=int(rows[first].get("step", first)),
            last_step=int(end_row.get("step", last)), steps=len(members), duration_s=duration,
            command_dir=direction, displacement_mm=[x * 1000.0 for x in displacement],
            angle_deg=angle, speed_mm_s=(along / duration * 1000.0) if duration > 0 else float("nan"),
            lead_at_release_mm=lead * 1000.0, coast_mm=coast * 1000.0, passed=passed,
        ))
    return segments


def session_label(path: Path) -> str:
    """The session directory name, with spacemouse_mode / cartesian_controller when recorded."""
    session = path.parent
    label = session.name
    config = session / "resolved-config.yaml"
    if config.is_file():
        text = config.read_text(errors="replace")
        for key in ("cartesian_controller", "spacemouse_mode"):
            match = re.search(rf"^\s*{key}:\s*([\w-]+)", text, re.MULTILINE)
            if match:
                label += f" {key}={match.group(1)}"
    return label


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("paths", nargs="+", help="session directories or diagnostics jsonl files")
    parser.add_argument("--min-input", type=float, default=0.3, help="min |translation command| (normalised)")
    parser.add_argument("--max-rotation-input", type=float, default=0.05, help="max |rotation command| per axis")
    parser.add_argument("--min-steps", type=int, default=4)
    parser.add_argument("--settle-s", type=float, default=1.0, help="window after release for coast_mm")
    parser.add_argument("--max-angle-deg", type=float, default=10.0)
    parser.add_argument("--max-lead-mm", type=float, default=3.0, help="pass bound of coast_mm")
    parser.add_argument("--json", action="store_true", help="one JSON object per segment")
    args = parser.parse_args(argv)
    segments: list[Segment] = []
    for path in diagnostics_files(args.paths):
        segments += analyze_rows(read_rows(path), source=session_label(path), min_input=args.min_input,
                                 max_rotation_input=args.max_rotation_input, min_steps=args.min_steps,
                                 settle_s=args.settle_s, max_angle_deg=args.max_angle_deg,
                                 max_lead_mm=args.max_lead_mm)
    if not segments:
        print("no translation-isolated segments found", file=sys.stderr)
        return 2
    for segment in segments:
        if args.json:
            print(json.dumps(asdict(segment)))
        else:
            print(f"{segment.source} {segment.axis:>2s} steps {segment.first_step}-{segment.last_step} "
                  f"({segment.duration_s:.2f} s): angle {segment.angle_deg:5.1f} deg, "
                  f"speed {segment.speed_mm_s:6.1f} mm/s, lead at release {segment.lead_at_release_mm:5.1f} mm, "
                  f"coast {segment.coast_mm:5.1f} mm  {'PASS' if segment.passed else 'FAIL'}")
    failed = sum(not s.passed for s in segments)
    if not args.json:
        print(f"{len(segments)} segments, {failed} failed "
              f"(pass: angle < {args.max_angle_deg} deg and coast < {args.max_lead_mm} mm)")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
