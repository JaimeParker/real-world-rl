"""Inspect a recorder SQLite archive without modifying it; optionally export step evidence.

Besides outcome and mask counts, each episode whose state carries the TCP position (one arm,
``tcp_pose`` in the recorded ``state_fields``) is split into segments, the maximal runs of
transitions with an identical ``human_action_mask``. A segment is ``hold`` when its mask is all
zero and ``intervene`` otherwise. From the TCP xyz (metres, reported in mm) of each transition's
``obs``:

- ``max_drift_mm``: the largest distance from the segment's first xyz; a ``hold`` segment also
  gets ``max_drift_window_mm`` over its first ``--drift-window-s`` seconds (``window_end_s`` is
  the last sample inside the window);
- ``residual_mm``: the distance between the segment's first and last xyz;
- ``jump_mm`` (``intervene`` only): the distance from the previous segment's last xyz to this
  segment's first xyz one step later (its first transition's ``next_obs``), so a jump at
  take-over shows.

``--stop-time-s T`` (the leader-stop wall time, ``date +%s.%N`` on the recording machine) adds a
settle entry: within the episode whose samples span ``T``, ``settled_at_s`` is the first sample
time ``t >= T`` from which the TCP speed stays below ``--settle-speed-mm-s`` for
``--settle-hold-s`` seconds; ``settle_s = settled_at_s - T`` and ``residual_after_stop_mm`` is
the distance from the last xyz at or before ``T`` to the xyz at ``settled_at_s``. The archive does
not record the teleop command target, so the operator's stop time is the only stop source.

Sampling assumption: one transition is one position sample. Its xyz is the observation taken
before the step and its ``timestamp_s`` is the recorder's wall clock after the step, so sample
times trail the measurement by one control period (constant at a steady control rate: it cancels
in durations and only offsets times compared with ``--stop-time-s``). TCP speed is the finite
difference of xyz over consecutive transitions of the same episode.

``--measurements PATH`` writes the rows of the test pages' ``measurements.csv``: one per segment,
one ``hold_window`` row per ``hold`` segment and one ``settle`` row per stop time. Episodes
without a TCP position (joint-space or two-arm archives) report segments without metrics and add
no rows.
"""
from __future__ import annotations

import argparse
from collections import Counter
from contextlib import ExitStack, closing
import csv
import json
import math
from pathlib import Path
import sqlite3
from types import SimpleNamespace
from typing import Any, Mapping, NamedTuple, Optional, Sequence

import torch

from real_world_rl.action_contract import num_arms_of
from real_world_rl.demos.archive_args import (DRIFT_WINDOW_S, SETTLE_HOLD_S, SETTLE_SPEED_MM_S,
                                              add_inspect_arguments)
from real_world_rl.envs.wrappers.obs_chain import tcp_pose_quat_slice
from real_world_rl.loops.durable import decode

MEASUREMENT_FIELDS = ("phase", "start_s", "end_s", "max_drift_mm", "residual_mm", "settle_s",
                      "stop_to_exit_s", "stop_to_robot_stop_s", "evidence")
MEASUREMENT_EVIDENCE = "inspect_teleop_archive"
# tcp_pose is xyz followed by the quaternion, so x sits this many dims before the quaternion.
_TCP_XYZ_BEFORE_QUAT = 3


class PositionSample(NamedTuple):
    """One transition's evidence for the TCP metrics (xyz in metres, ``None`` when absent)."""

    timestamp_s: float
    mask: str
    xyz: Optional[tuple[float, ...]]
    next_xyz: Optional[tuple[float, ...]]


def json_value(value):
    if isinstance(value, torch.Tensor):
        return value.detach().cpu().tolist()
    raise TypeError(f"unsupported archived value {type(value).__name__}")


def packed(value):
    return json.dumps(value, default=json_value, ensure_ascii=False, separators=(",", ":"))


def mask_bits(value):
    return "".join("1" if bit else "0" for bit in torch.as_tensor(value).reshape(-1).tolist())


def tcp_xyz_offset(contract: Any) -> Optional[int]:
    """Index of TCP x in a single arm's flat ``state``, or ``None`` when the episode's contract
    cannot place it (no contract, several arms, no ``tcp_pose``, or a joint field before it
    without a recorded ``num_joints``)."""
    if not isinstance(contract, Mapping) or num_arms_of(contract) != 1:
        return None
    fields = list(contract.get("state_fields") or ())
    if "tcp_pose" not in fields:
        return None
    layout = SimpleNamespace(state_fields=fields, num_joints=contract.get("num_joints"))
    try:
        quat_start, _ = tcp_pose_quat_slice(layout)
    except TypeError:  # a joint-sized field precedes tcp_pose and num_joints is unknown
        return None
    return quat_start - _TCP_XYZ_BEFORE_QUAT


def tcp_xyz(observation: Any, xyz_offset: Optional[int]) -> Optional[tuple[float, ...]]:
    """TCP xyz in metres from ``observation["state"]``: a ``{"tcp_pose": ...}`` mapping or the
    flat ``(1, D)`` state read at ``xyz_offset``."""
    state = observation.get("state") if isinstance(observation, Mapping) else None
    if isinstance(state, Mapping):
        if state.get("tcp_pose") is None:
            return None
        values = torch.as_tensor(state["tcp_pose"], dtype=torch.float64).reshape(-1)[:3]
    elif state is None or xyz_offset is None:
        return None
    else:
        values = torch.as_tensor(state, dtype=torch.float64).reshape(-1)[xyz_offset:xyz_offset + 3]
    return tuple(values.tolist()) if values.numel() == 3 else None


def distance_mm(first, second) -> float:
    return math.dist(first, second) * 1000.0


def speed_mm_s(earlier: PositionSample, later: PositionSample) -> float:
    moved = distance_mm(earlier.xyz, later.xyz)
    elapsed = later.timestamp_s - earlier.timestamp_s
    if elapsed > 0:
        return moved / elapsed
    return 0.0 if moved == 0 else math.inf


def segment_bounds(samples: Sequence[PositionSample]) -> list[tuple[int, int]]:
    """``(start, stop)`` index ranges of the maximal runs of identical masks."""
    bounds, start = [], 0
    for index in range(1, len(samples) + 1):
        if index == len(samples) or samples[index].mask != samples[start].mask:
            bounds.append((start, index))
            start = index
    return bounds


def segment_metrics(samples: Sequence[PositionSample], *, with_tcp: bool, drift_window_s: float) -> list[dict]:
    segments = []
    for index, (start, stop) in enumerate(segment_bounds(samples)):
        run = samples[start:stop]
        phase = "intervene" if "1" in run[0].mask else "hold"
        segment = {"index": index, "phase": phase, "human_action_mask": run[0].mask, "steps": len(run),
                   "start_s": run[0].timestamp_s, "end_s": run[-1].timestamp_s,
                   "duration_s": run[-1].timestamp_s - run[0].timestamp_s,
                   "max_drift_mm": None, "max_drift_window_mm": None, "window_end_s": None,
                   "residual_mm": None, "jump_mm": None}
        if with_tcp:
            origin = run[0].xyz
            segment["max_drift_mm"] = max(distance_mm(origin, sample.xyz) for sample in run)
            segment["residual_mm"] = distance_mm(origin, run[-1].xyz)
            if phase == "hold":
                window = [sample for sample in run if sample.timestamp_s - run[0].timestamp_s <= drift_window_s]
                segment["max_drift_window_mm"] = max(distance_mm(origin, sample.xyz) for sample in window)
                segment["window_end_s"] = window[-1].timestamp_s
            elif start > 0:
                segment["jump_mm"] = distance_mm(samples[start - 1].xyz, run[0].next_xyz)
        segments.append(segment)
    return segments


def settle_after_stop(samples: Sequence[PositionSample], stop_time_s: float, *, speed_limit_mm_s: float,
                      hold_s: float) -> dict:
    """Settle fields for one stop time inside one episode's samples (which span it)."""
    reference = max(index for index, sample in enumerate(samples) if sample.timestamp_s <= stop_time_s)
    for candidate in range(reference, len(samples)):
        settled_at_s = samples[candidate].timestamp_s
        if settled_at_s < stop_time_s:
            continue
        window_end_s = settled_at_s + hold_s
        if samples[-1].timestamp_s < window_end_s:
            break  # the episode ends before the hold could be confirmed
        overlapping = (index for index in range(candidate + 1, len(samples))
                       if samples[index - 1].timestamp_s < window_end_s)
        if all(speed_mm_s(samples[index - 1], samples[index]) < speed_limit_mm_s for index in overlapping):
            return {"settled": True, "settled_at_s": settled_at_s, "settle_s": settled_at_s - stop_time_s,
                    "residual_after_stop_mm": distance_mm(samples[reference].xyz, samples[candidate].xyz),
                    "problem": None}
    return {"settled": False, "settled_at_s": None, "settle_s": None, "residual_after_stop_mm": None,
            "problem": "the TCP did not settle before the episode ended"}


def settle_entry(episodes: Sequence[dict], samples_by_episode: Mapping[str, list[PositionSample]],
                 stop_time_s: float, *, speed_limit_mm_s: float, hold_s: float) -> dict:
    """The settle entry for ``stop_time_s`` in the first TCP episode whose samples span it."""
    entry = {"stop_time_s": stop_time_s, "episode_id": None, "segment_index": None}
    for episode in episodes:
        samples = samples_by_episode.get(episode["episode_id"], [])
        if not episode["tcp_metrics"] or not samples[0].timestamp_s <= stop_time_s <= samples[-1].timestamp_s:
            continue
        started = [segment for segment in episode["segments"] if segment["start_s"] <= stop_time_s]
        entry.update(episode_id=episode["episode_id"], segment_index=started[-1]["index"])
        entry.update(settle_after_stop(samples, stop_time_s, speed_limit_mm_s=speed_limit_mm_s, hold_s=hold_s))
        return entry
    entry.update(settled=False, settled_at_s=None, settle_s=None, residual_after_stop_mm=None,
                 problem="the stop time is outside every episode with a TCP position")
    return entry


def episode_summaries(order: Sequence[str], samples_by_episode: Mapping[str, list[PositionSample]],
                      drift_window_s: float) -> list[dict]:
    episodes = []
    for episode_id in order:
        samples = samples_by_episode.get(episode_id, [])
        with_tcp = bool(samples) and all(sample.xyz is not None and sample.next_xyz is not None
                                         for sample in samples)
        episodes.append({"episode_id": episode_id, "tcp_metrics": with_tcp,
                         "segments": segment_metrics(samples, with_tcp=with_tcp, drift_window_s=drift_window_s)})
    return episodes


def measurement_rows(episodes: Sequence[dict], settles: Sequence[dict]) -> list[dict]:
    def number(value, digits):
        return "" if value is None else f"{value:.{digits}f}"

    rows = []
    for episode in episodes:
        if not episode["tcp_metrics"]:
            continue
        for segment in episode["segments"]:
            rows.append({"phase": segment["phase"], "start_s": number(segment["start_s"], 6),
                         "end_s": number(segment["end_s"], 6), "max_drift_mm": number(segment["max_drift_mm"], 3),
                         "residual_mm": number(segment["residual_mm"], 3)})
            if segment["phase"] == "hold":
                rows.append({"phase": "hold_window", "start_s": number(segment["start_s"], 6),
                             "end_s": number(segment["window_end_s"], 6),
                             "max_drift_mm": number(segment["max_drift_window_mm"], 3)})
    for settle in settles:
        rows.append({"phase": "settle", "start_s": number(settle["stop_time_s"], 6),
                     "end_s": number(settle["settled_at_s"], 6),
                     "residual_mm": number(settle["residual_after_stop_mm"], 3),
                     "settle_s": number(settle["settle_s"], 3)})
    return [{**dict.fromkeys(MEASUREMENT_FIELDS, ""), **row, "evidence": MEASUREMENT_EVIDENCE} for row in rows]


def inspect_archive(path: Path, csv_path: Path | None = None, *, measurements_path: Path | None = None,
                    stop_times_s: Sequence[float] = (), drift_window_s: float = DRIFT_WINDOW_S,
                    settle_speed_mm_s: float = SETTLE_SPEED_MM_S, settle_hold_s: float = SETTLE_HOLD_S):
    path = path.resolve(strict=True)
    for export, name in ((csv_path, "step export"), (measurements_path, "measurements export")):
        if export is not None and export.suffix.lower() != ".csv":
            raise ValueError(f"{name} must use a new .csv path")
    for value, flag in ((drift_window_s, "--drift-window-s"), (settle_speed_mm_s, "--settle-speed-mm-s"),
                        (settle_hold_s, "--settle-hold-s")):
        if not value > 0:
            raise ValueError(f"{flag} must be positive, got {value}")
    fields = ["episode_id", "step_index", "timestamp_s", "run_id", "human_action_mask", "override_mask",
              "action", "policy_action", "state", "next_state", "teleop_events", "terminated", "truncated"]
    summary = {"archive": str(path), "records": 0, "episode_starts": 0, "episode_ends": 0,
               "transitions": 0, "human_mask_counts": Counter(), "outcomes": []}
    contracts: dict[str, Any] = {}  # insertion order is the episodes' archive order
    xyz_offsets: dict[str, Optional[int]] = {}
    samples_by_episode: dict[str, list[PositionSample]] = {}
    with ExitStack() as exits:
        # mode=ro prevents schema creation and interrupted-episode recovery writes.
        connection = exits.enter_context(closing(sqlite3.connect(path.as_uri() + "?mode=ro", uri=True)))
        connection.execute("PRAGMA query_only=ON")
        cursor = connection.execute("SELECT payload FROM messages ORDER BY seq")
        writer = None
        if csv_path is not None:
            stream = exits.enter_context(csv_path.open("x", encoding="utf-8-sig", newline=""))
            writer = csv.DictWriter(stream, fieldnames=fields)
            writer.writeheader()
        measurements = None
        if measurements_path is not None:
            measurements = exits.enter_context(measurements_path.open("x", encoding="utf-8", newline=""))
        for (payload,) in cursor:
            record = decode(payload)
            summary["records"] += 1
            kind = record["record_type"]
            episode_id = record["episode_id"]
            contracts.setdefault(episode_id, None)
            if kind == "episode_start":
                summary["episode_starts"] += 1
                contracts[episode_id] = record.get("action_contract")
            elif kind == "episode_end":
                summary["episode_ends"] += 1
                summary["outcomes"].append(record.get("outcome"))
            elif kind == "transition":
                summary["transitions"] += 1
                bits = mask_bits(record["human_action_mask"])
                summary["human_mask_counts"][bits] += 1
                if episode_id not in xyz_offsets:
                    xyz_offsets[episode_id] = tcp_xyz_offset(contracts[episode_id])
                offset = xyz_offsets[episode_id]
                samples_by_episode.setdefault(episode_id, []).append(PositionSample(
                    float(record["timestamp_s"]), bits, tcp_xyz(record["obs"], offset),
                    tcp_xyz(record["next_obs"], offset)))
                if writer is not None:
                    row = {key: packed(record.get(key)) for key in fields}
                    row.update(human_action_mask=bits, override_mask=mask_bits(record["override_mask"]),
                               state=packed(record["obs"].get("state")),
                               next_state=packed(record["next_obs"].get("state")))
                    writer.writerow(row)
        episodes = episode_summaries(list(contracts), samples_by_episode, drift_window_s)
        settles = [settle_entry(episodes, samples_by_episode, float(stop_time_s),
                                speed_limit_mm_s=settle_speed_mm_s, hold_s=settle_hold_s)
                   for stop_time_s in stop_times_s]
        if measurements is not None:
            measurement_writer = csv.DictWriter(measurements, fieldnames=MEASUREMENT_FIELDS, lineterminator="\n")
            measurement_writer.writeheader()
            measurement_writer.writerows(measurement_rows(episodes, settles))
    summary["human_mask_counts"] = dict(summary["human_mask_counts"])
    summary["metric_settings"] = {"drift_window_s": drift_window_s, "settle_speed_mm_s": settle_speed_mm_s,
                                  "settle_hold_s": settle_hold_s}
    summary["episodes"] = episodes
    summary["settle"] = settles
    return summary


def format_summary(summary: Mapping[str, Any]) -> str:
    """The summary as this inspector prints it (indented JSON, non-ASCII kept)."""
    return json.dumps(summary, ensure_ascii=False, indent=2)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    add_inspect_arguments(parser)
    args = parser.parse_args(argv)
    summary = inspect_archive(args.archive, args.csv, measurements_path=args.measurements,
                              stop_times_s=args.stop_time_s, drift_window_s=args.drift_window_s,
                              settle_speed_mm_s=args.settle_speed_mm_s, settle_hold_s=args.settle_hold_s)
    print(format_summary(summary))
