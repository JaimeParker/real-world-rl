"""The archive inspector's hold drift, take-over jump and settle metrics on a scripted TCP path."""
from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path

import pytest
import torch

from real_world_rl.demos import archive_inspect
from real_world_rl.trajectory import DurableArchive, begin_episode, end_episode, transition_record

CARTESIAN_CONTRACT = {"action_mode": "cartesian_delta", "arm_action_dim": 6, "gripper_in_action": True,
                      "num_joints": 7, "state_fields": ["joint_pos", "tcp_pose"]}
JOINT_CONTRACT = {"action_mode": "joint_delta", "arm_action_dim": 7, "gripper_in_action": True,
                  "num_joints": 7, "state_fields": ["joint_pos", "gripper_pos"]}
BASE_XYZ_M = (0.5, 0.0, 0.3)
START_S, STEP_S = 1000.0, 0.1
HOLD_MASK = [False] * 7
INTERVENE_MASK = [True] * 6 + [False]
# Hold wobble along x (mm): at most 2 mm in the first 0.85 s (steps 0-8), 3 mm at step 12.
HOLD_WOBBLE_X_MM = [0, 1, -1, 2, 0, 1, -2, 1, 0, 1, 0, -1, 3, 1, 0, 1]
TAKEOVER_JUMP_MM = (12.0, 16.0, 0.0)  # 20 mm between step 15 and step 17
LEADER_STOP_STEP = 24
# After the leader stops at step 24 the arm keeps closing in: 30 mm more, still from step 32.
CATCH_UP_X_MM = [8, 15, 20, 24, 27, 28.5, 29.5, 30.0]
SETTLED_STEP = LEADER_STOP_STEP + len(CATCH_UP_X_MM)
INTERVENE_STOP = 41  # steps 16-40 intervene, 41-45 hold again
NUM_STEPS = 46
DRIFT_WINDOW_S = 0.85


def step_time(step: int) -> float:
    return START_S + STEP_S * step


def scripted_offsets_mm() -> list[tuple[float, float, float]]:
    offsets = [(float(x), 0.0, 0.0) for x in HOLD_WOBBLE_X_MM]
    held = offsets[-1]
    offsets.append(held)  # step 16: the first intervened observation, before its action acts
    jumped = tuple(a + b for a, b in zip(held, TAKEOVER_JUMP_MM))
    offsets.extend((jumped[0] + 10.0 * k, jumped[1], jumped[2]) for k in range(LEADER_STOP_STEP - 16))
    stopped = offsets[LEADER_STOP_STEP]
    offsets.extend((stopped[0] + x, stopped[1], stopped[2]) for x in CATCH_UP_X_MM)
    offsets.extend([offsets[-1]] * (NUM_STEPS - len(offsets)))
    return offsets


def state_of(offset_mm, layout: str):
    xyz = [base + delta / 1000.0 for base, delta in zip(BASE_XYZ_M, offset_mm)]
    pose = torch.tensor(xyz + [0.0, 0.0, 0.0, 1.0], dtype=torch.float64)
    if layout == "dict":
        return {"tcp_pose": pose}
    return torch.cat([torch.zeros(7, dtype=torch.float64), pose]).reshape(1, -1)


def write_archive(path: Path, *, contract: dict, layout: str = "flat", episode_id: str = "episode-0") -> Path:
    offsets = scripted_offsets_mm()
    archive = DurableArchive(path)
    try:
        archive.append(begin_episode(episode_id, action_contract=contract))
        for step in range(NUM_STEPS):
            following = offsets[min(step + 1, NUM_STEPS - 1)]
            mask = INTERVENE_MASK if 16 <= step < INTERVENE_STOP else HOLD_MASK
            transition = {"obs": {"state": state_of(offsets[step], layout)},
                          "next_obs": {"state": state_of(following, layout)},
                          "action": torch.zeros(1, 7), "reward": 0.0, "done": False}
            archive.append(transition_record(
                transition, episode_id=episode_id, step_index=step, timestamp_s=step_time(step),
                terminated=False, truncated=step == NUM_STEPS - 1,
                human_action_mask=torch.tensor([mask]), override_mask=torch.tensor([mask])))
        archive.append(end_episode(episode_id, step_count=NUM_STEPS))
    finally:
        archive.close()
    return path


def fingerprint(path: Path):
    stat = path.stat()
    return stat.st_size, stat.st_mtime_ns, hashlib.sha256(path.read_bytes()).hexdigest()


@pytest.mark.parametrize("layout", ["flat", "dict"])
def test_segments_report_hold_drift_takeover_jump_and_settle(tmp_path, layout):
    archive = write_archive(tmp_path / "demo.trajectory.sqlite3", contract=CARTESIAN_CONTRACT, layout=layout)
    before = fingerprint(archive)
    summary = archive_inspect.inspect_archive(archive, stop_times_s=[step_time(LEADER_STOP_STEP)],
                                              drift_window_s=DRIFT_WINDOW_S)
    assert fingerprint(archive) == before
    (episode,) = summary["episodes"]
    assert episode["episode_id"] == "episode-0" and episode["tcp_metrics"] is True
    first_hold, intervene, last_hold = episode["segments"]

    assert [segment["phase"] for segment in episode["segments"]] == ["hold", "intervene", "hold"]
    assert (first_hold["steps"], intervene["steps"], last_hold["steps"]) == (16, 25, 5)
    assert intervene["human_action_mask"] == "1111110"
    assert intervene["start_s"] == pytest.approx(step_time(16), abs=1e-6)
    assert intervene["end_s"] == pytest.approx(step_time(40), abs=1e-6)
    assert intervene["duration_s"] == pytest.approx(2.4, abs=1e-6)

    assert first_hold["max_drift_mm"] == pytest.approx(3.0, abs=1e-6)
    assert first_hold["max_drift_window_mm"] == pytest.approx(2.0, abs=1e-6)
    assert first_hold["window_end_s"] == pytest.approx(step_time(8), abs=1e-6)
    assert first_hold["residual_mm"] == pytest.approx(1.0, abs=1e-6)
    assert first_hold["jump_mm"] is None

    assert intervene["jump_mm"] == pytest.approx(20.0, abs=1e-6)
    assert intervene["max_drift_mm"] == pytest.approx(math.hypot(112.0, 16.0), abs=1e-6)
    assert intervene["residual_mm"] == pytest.approx(math.hypot(112.0, 16.0), abs=1e-6)
    assert intervene["max_drift_window_mm"] is None

    assert last_hold["max_drift_mm"] == pytest.approx(0.0, abs=1e-6)
    assert last_hold["max_drift_window_mm"] == pytest.approx(0.0, abs=1e-6)

    (settle,) = summary["settle"]
    assert settle["settled"] is True and settle["problem"] is None
    assert settle["episode_id"] == "episode-0" and settle["segment_index"] == 1
    assert settle["settled_at_s"] == pytest.approx(step_time(SETTLED_STEP), abs=1e-6)
    assert settle["settle_s"] == pytest.approx(0.8, abs=1e-6)
    assert settle["residual_after_stop_mm"] == pytest.approx(30.0, abs=1e-6)


def test_a_stricter_settle_speed_moves_the_settle_time(tmp_path):
    archive = write_archive(tmp_path / "demo.trajectory.sqlite3", contract=CARTESIAN_CONTRACT)
    # The last catch-up step moves 0.5 mm in 0.1 s (5 mm/s): a 6 mm/s limit settles one step earlier.
    (settle,) = archive_inspect.inspect_archive(archive, stop_times_s=[step_time(LEADER_STOP_STEP)],
                                                settle_speed_mm_s=6.0)["settle"]
    assert settle["settle_s"] == pytest.approx(0.7, abs=1e-6)
    assert settle["residual_after_stop_mm"] == pytest.approx(29.5, abs=1e-6)


def test_a_stop_time_outside_the_episodes_reports_a_problem(tmp_path):
    archive = write_archive(tmp_path / "demo.trajectory.sqlite3", contract=CARTESIAN_CONTRACT)
    (settle,) = archive_inspect.inspect_archive(archive, stop_times_s=[START_S - 1.0])["settle"]
    assert settle["settled"] is False and settle["episode_id"] is None and settle["settle_s"] is None
    assert "outside" in settle["problem"]


def test_cli_writes_the_test_page_measurements(tmp_path, capsys):
    archive = write_archive(tmp_path / "demo.trajectory.sqlite3", contract=CARTESIAN_CONTRACT)
    before = fingerprint(archive)
    measurements = tmp_path / "measurements.csv"
    argv = [str(archive), "--measurements", str(measurements), "--stop-time-s", repr(step_time(LEADER_STOP_STEP)),
            "--drift-window-s", str(DRIFT_WINDOW_S)]
    archive_inspect.main(argv)
    summary = json.loads(capsys.readouterr().out)
    assert summary["metric_settings"] == {"drift_window_s": DRIFT_WINDOW_S, "settle_speed_mm_s": 2.0,
                                          "settle_hold_s": 0.5}
    assert summary["settle"][0]["settle_s"] == pytest.approx(0.8, abs=1e-6)
    assert measurements.read_text(encoding="utf-8").splitlines() == [
        "phase,start_s,end_s,max_drift_mm,residual_mm,settle_s,stop_to_exit_s,stop_to_robot_stop_s,evidence",
        "hold,1000.000000,1001.500000,3.000,1.000,,,,inspect_teleop_archive",
        "hold_window,1000.000000,1000.800000,2.000,,,,,inspect_teleop_archive",
        "intervene,1001.600000,1004.000000,113.137,113.137,,,,inspect_teleop_archive",
        "hold,1004.100000,1004.500000,0.000,0.000,,,,inspect_teleop_archive",
        "hold_window,1004.100000,1004.500000,0.000,,,,,inspect_teleop_archive",
        "settle,1002.400000,1003.200000,,30.000,0.800,,,inspect_teleop_archive",
    ]
    with pytest.raises(FileExistsError):
        archive_inspect.main(argv)  # exports never overwrite earlier evidence
    assert fingerprint(archive) == before


def test_joint_space_archive_has_segments_without_tcp_metrics(tmp_path):
    archive = write_archive(tmp_path / "demo.trajectory.sqlite3", contract=JOINT_CONTRACT)
    before = fingerprint(archive)
    measurements = tmp_path / "measurements.csv"
    summary = archive_inspect.inspect_archive(archive, measurements_path=measurements,
                                              stop_times_s=[step_time(LEADER_STOP_STEP)])
    assert fingerprint(archive) == before
    (episode,) = summary["episodes"]
    assert episode["tcp_metrics"] is False
    assert [segment["phase"] for segment in episode["segments"]] == ["hold", "intervene", "hold"]
    assert all(segment[key] is None for segment in episode["segments"]
               for key in ("max_drift_mm", "max_drift_window_mm", "residual_mm", "jump_mm"))
    assert summary["settle"][0]["settled"] is False
    assert measurements.read_text(encoding="utf-8").splitlines() == [
        "phase,start_s,end_s,max_drift_mm,residual_mm,settle_s,stop_to_exit_s,stop_to_robot_stop_s,evidence",
        "settle,1002.400000,,,,,,,inspect_teleop_archive",
    ]


def test_measurements_must_be_a_csv_path(tmp_path):
    archive = write_archive(tmp_path / "demo.trajectory.sqlite3", contract=CARTESIAN_CONTRACT)
    with pytest.raises(ValueError, match="measurements export"):
        archive_inspect.inspect_archive(archive, measurements_path=tmp_path / "measurements.txt")
