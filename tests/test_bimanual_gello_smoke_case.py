"""The commissioning preset records locally and its evidence inspector stays read-only."""
from __future__ import annotations

import csv
import hashlib
from pathlib import Path

from test_bimanual_station_smoke import station_daemons  # noqa: F401 (shared fake gRPC fixture)
from real_world_rl.demos import archive_inspect
from real_world_rl.demos.record import parse_record_args, run_recorder
from real_world_rl.trajectory import DurableArchive, begin_episode

ROOT = Path(__file__).resolve().parents[1]


def test_smoke_preset_records_one_attempt_on_two_local_fake_daemons(tmp_path, station_daemons):  # noqa: F811 (fixture injected by name)
    output = tmp_path / "left.pkl"
    args = parse_record_args([
        "--config", str(ROOT / "examples/presets/bimanual_gello_smoke.yaml"),
        "--output-path", str(output), "--max-episodes", "1",
    ])
    assert args.teleop_event_device is None  # headless: events arrive from device-side inputs
    assert not args.obs.rgb and args.teleop_onset == "arm"
    for name, daemon in station_daemons.items():
        config = getattr(args.real_robot_bimanual, name)
        config.remote, config.token = daemon.address, daemon.token
        config.control_hz = 100
        config.reset_settle_s = 0
    args.control_hz = 100
    args.real_robot_bimanual.reset_settle_s = 0
    args.real_robot_bimanual.max_episode_steps = 10
    # Fake leaders are deliberately 0.1 rad off home; hardware uses verified alignment.
    args.teleop_arm_tolerance_rad = 0.2
    assert run_recorder(args) == output
    assert all(not daemon.service.robot.is_connected for daemon in station_daemons.values())
    archive = output.with_suffix(".trajectory.sqlite3")
    digest = hashlib.sha256(archive.read_bytes()).hexdigest()
    step_csv = tmp_path / "steps.csv"
    summary = archive_inspect.inspect_archive(archive, step_csv)
    assert summary["episode_starts"] == summary["episode_ends"] == 1
    assert summary["outcomes"][0]["task_success"] is True
    assert summary["human_mask_counts"]["1111111000000010"] >= 1
    assert hashlib.sha256(archive.read_bytes()).hexdigest() == digest
    with step_csv.open(encoding="utf-8-sig", newline="") as stream:
        rows = list(csv.DictReader(stream))
    assert len(rows) == summary["transitions"]
    assert {"state", "next_state", "policy_action", "override_mask"} <= rows[0].keys()


def test_inspection_does_not_finalize_an_unfinished_episode(tmp_path):
    archive_path = tmp_path / "unfinished.trajectory.sqlite3"
    archive = DurableArchive(archive_path)
    try:
        archive.append(begin_episode("unfinished"))
    finally:
        archive.close()
    digest = hashlib.sha256(archive_path.read_bytes()).hexdigest()
    summary = archive_inspect.inspect_archive(archive_path)
    assert summary["episode_starts"] == 1 and summary["episode_ends"] == 0
    assert summary["outcomes"] == []
    assert hashlib.sha256(archive_path.read_bytes()).hexdigest() == digest
