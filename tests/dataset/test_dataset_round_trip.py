"""Episodes written by ``EpisodeWriter`` read back bit-for-bit, with the meta of the spec; review
videos for RGB keys, and none (but the episode kept) when ffmpeg fails."""
from __future__ import annotations

import json
import shutil
import subprocess
from pathlib import Path

import numpy as np
import pytest
import torch
from _dataset_fakes import ACTION_DIM, NAMED_STATE_SPEC, OBS_SPEC, open_writer, random_obs, write_episode

from real_world_rl.dataset import Dataset, SCHEMA_VERSION, iter_transitions
from real_world_rl.dataset import video as dataset_video

INTERVENED = [False, False, True, True, True, False, True, True]
EVENTS = {4: {"success": False, "teleop_syncing": True}, 7: {"success": True}}


def _ffmpeg_works() -> bool:
    executable = dataset_video.ffmpeg_executable()
    return shutil.which(executable) is not None or Path(executable).is_file()


def _write_three(root):
    rng = np.random.default_rng(0)
    writer = open_writer(root)
    episodes = [
        write_episode(writer, rng, INTERVENED, events=EVENTS, grasp_penalty=True),
        write_episode(writer, rng, [True, False, True], status="discarded", discard_reason="rerecord",
                      success=False, events={1: {"rerecord": True}}),
        write_episode(writer, rng, [True] * 4),
    ]
    return writer, episodes


def test_round_trip_is_bit_exact_with_the_spec_meta(tmp_path):
    root = tmp_path / "peg"
    _, written = _write_three(root)
    dataset = Dataset(root)

    header = dataset.header
    assert (header["format"], header["schema_version"], header["next_index"]) == ("rwrl-dataset", SCHEMA_VERSION, 3)
    assert header["obs_spec"] == OBS_SPEC and header["named_state_spec"] == NAMED_STATE_SPEC
    assert header["action_dim"] == ACTION_DIM and header["control_hz"] == 10.0
    assert [info.index for info in dataset.episodes()] == [0, 1, 2]
    assert [info.status for info in dataset.episodes()] == ["kept", "discarded", "kept"]
    assert sorted(p.name for p in root.iterdir()) == ["dataset.json", "episode_000000", "episode_000001", "episode_000002"]

    for index, (record, _) in enumerate(written):
        episode = dataset.episode(index)
        data = episode.data
        steps = len(record.steps)
        np.testing.assert_array_equal(data["obs_time_s"], np.asarray(record.times))
        assert data["obs_time_s"].dtype == np.float64
        np.testing.assert_array_equal(data["state"], np.stack([obs["state"] for obs in record.obs]))
        for key in ("rgb_wrist", "rgb_front", "depth_front"):
            frames = episode.frames(key)
            assert frames.dtype == np.dtype(OBS_SPEC[key]["dtype"]) and frames.shape == (steps + 1, *OBS_SPEC[key]["shape"])
            np.testing.assert_array_equal(frames, np.stack([obs[key] for obs in record.obs]))
        np.testing.assert_array_equal(data["named.joint_pos"], np.stack([n["joint_pos"] for n in record.named]).astype(np.float32))
        np.testing.assert_array_equal(data["named.gripper_pos"],
                                      np.asarray([[n["gripper_pos"]] for n in record.named], dtype=np.float32))
        assert data["stack_reset"].tolist() == [True] + [False] * steps
        for name in ("action", "policy_action", "human_action_mask", "override_mask"):
            np.testing.assert_array_equal(data[name], np.stack([step[name] for step in record.steps]))
        for name, dtype in (("intervened", bool), ("reward", np.float32), ("terminated", bool),
                            ("truncated", bool), ("syncing", bool)):
            assert data[name].dtype == dtype and data[name].shape == (steps,)
            np.testing.assert_array_equal(data[name], np.asarray([step[name] for step in record.steps], dtype=dtype))
        assert data["action"].dtype == np.float32 and data["human_action_mask"].dtype == bool

    first = dataset.episode(0)
    np.testing.assert_array_equal(first.data["grasp_penalty"],
                                  np.asarray([s["grasp_penalty"] for s in written[0][0].steps], dtype=np.float32))
    assert first.data["event.success"].tolist() == [False] * 7 + [True]
    assert first.data["event.teleop_syncing"].tolist() == [False] * 4 + [True] + [False] * 3
    meta = first.meta
    assert meta["status"] == "kept" and meta["discard_reason"] is None and meta["recorded_discard_reason"] is None
    assert meta["steps"] == 8 and meta["intervened_steps"] == 5 and meta["success"] is True
    assert meta["segments"] == [{"start": 0, "end": 2, "kind": "hold"}, {"start": 2, "end": 5, "kind": "intervene"},
                                {"start": 5, "end": 6, "kind": "hold"}, {"start": 6, "end": 8, "kind": "intervene"}]
    assert meta["events"] == [{"step": 4, "name": "teleop_syncing"}, {"step": 7, "name": "success"}]
    assert meta["duration_s"] == pytest.approx(0.8) and meta["operator_quit"] is False
    assert meta["notes"] == [] and meta["history"] == [] and meta["extra"] == {}
    assert len(meta["episode_id"]) == 32 and meta["index"] == 0

    discarded = dataset.episode(1).meta
    assert (discarded["status"], discarded["discard_reason"], discarded["recorded_discard_reason"]) == (
        "discarded", "rerecord", "rerecord")
    assert discarded["success"] is False and discarded["events"] == [{"step": 1, "name": "rerecord"}]
    assert "grasp_penalty" not in dataset.episode(1).data
    assert dataset.episode(2).meta["segments"] == [{"start": 0, "end": 4, "kind": "intervene"}]


@pytest.mark.skipif(not _ffmpeg_works(), reason="no ffmpeg executable")
def test_rgb_keys_get_a_review_video_of_every_frame(tmp_path):
    root = tmp_path / "peg"
    _write_three(root)
    episode = Dataset(root).episode(0)
    assert episode.meta["videos"] == {"rgb_front": "video/rgb_front.mp4", "rgb_wrist": "video/rgb_wrist.mp4"}
    assert episode.video_path("depth_front") is None
    for key, (height, width) in (("rgb_wrist", (9, 15)), ("rgb_front", (7, 11))):
        path = episode.video_path(key)
        assert path.is_file() and path.stat().st_size > 0
        probe = subprocess.run([dataset_video.ffmpeg_executable(), "-i", str(path), "-f", "null", "-"],
                               capture_output=True, text=True)
        assert "Video: h264" in probe.stderr and "yuv420p" in probe.stderr
        # Odd sizes are padded to even.
        assert f"{width + 1}x{height + 1}" in probe.stderr
        assert "frame=    9" in probe.stderr or "frame=9" in probe.stderr.replace(" ", "")


@pytest.mark.parametrize("executable", ["missing", "failing"])
def test_a_failing_ffmpeg_keeps_the_episode_without_videos(tmp_path, monkeypatch, caplog, executable):
    path = tmp_path / "ffmpeg"
    if executable == "failing":
        path.write_text("#!/bin/sh\ncat > /dev/null\necho broken >&2\nexit 1\n")
        path.chmod(0o755)
    monkeypatch.setattr(dataset_video, "ffmpeg_executable", lambda: str(path))
    root = tmp_path / "peg"
    rng = np.random.default_rng(1)
    record, info = write_episode(open_writer(root), rng, [True, True])

    episode = Dataset(root).episode(0)
    assert info.kept and episode.meta["status"] == "kept" and episode.meta["videos"] == {}
    assert episode.video_path("rgb_wrist") is None
    assert not list((episode.path / "video").glob("*.mp4"))
    np.testing.assert_array_equal(episode.frames("rgb_wrist"), np.stack([obs["rgb_wrist"] for obs in record.obs]))
    assert "no review video" in caplog.text


def test_a_stacked_image_stores_its_newest_frame(tmp_path):
    rng = np.random.default_rng(2)
    episode = open_writer(tmp_path / "peg").begin_episode()
    rows = []
    for _ in range(2):
        obs = random_obs(rng)
        history = {key: np.stack([random_obs(rng)[key], value]) for key, value in obs.items() if key != "state"}
        stacked = {"state": torch.from_numpy(obs["state"])[None], **{k: torch.from_numpy(v)[None] for k, v in history.items()}}
        episode.add_observation(stacked, {"joint_pos": np.zeros(7), "gripper_pos": [0.0]}, time_s=0.0)
        rows.append(obs)
        if len(rows) == 1:
            episode.add_step(np.zeros(ACTION_DIM), np.zeros(ACTION_DIM), np.ones(ACTION_DIM, bool),
                             np.ones(ACTION_DIM, bool), True, 0.0, False, False, False, {})
    episode.finish(status="kept", success=True)
    stored = Dataset(tmp_path / "peg").episode(0)
    for key in ("rgb_wrist", "rgb_front", "depth_front"):
        np.testing.assert_array_equal(stored.frames(key), np.stack([obs[key] for obs in rows]))


def test_finish_needs_one_more_observation_than_steps(tmp_path):
    writer = open_writer(tmp_path / "peg")
    episode = writer.begin_episode()
    with pytest.raises(RuntimeError, match="add_observation first"):
        episode.add_step(np.zeros(ACTION_DIM), np.zeros(ACTION_DIM), np.ones(ACTION_DIM, bool),
                         np.ones(ACTION_DIM, bool), True, 0.0, False, False, False, {})
    with pytest.raises(RuntimeError, match="needs 1 observations"):
        episode.finish(status="kept")
    obs = {key: value[None] for key, value in random_obs(np.random.default_rng(3)).items()}
    named = {"joint_pos": np.zeros(7), "gripper_pos": [0.0]}
    episode.add_observation(obs, named, time_s=0.0)
    with pytest.raises(RuntimeError, match="twice"):
        episode.add_observation(obs, named, time_s=0.0)


def test_observations_are_checked_against_the_spec(tmp_path):
    episode = open_writer(tmp_path / "peg").begin_episode()
    obs = {key: value[None] for key, value in random_obs(np.random.default_rng(4)).items()}
    named = {"joint_pos": np.zeros(7), "gripper_pos": [0.0]}
    with pytest.raises(ValueError, match="'rgb_wrist' has shape"):
        episode.add_observation({**obs, "rgb_wrist": obs["rgb_wrist"][:, :4]}, named, time_s=0.0)
    with pytest.raises(ValueError, match="dtype"):
        episode.add_observation({**obs, "state": obs["state"].astype(np.float64)}, named, time_s=0.0)
    with pytest.raises(ValueError, match="observation keys"):
        episode.add_observation({k: v for k, v in obs.items() if k != "depth_front"}, named, time_s=0.0)
    with pytest.raises(ValueError, match="named state fields"):
        episode.add_observation(obs, {"joint_pos": np.zeros(7)}, time_s=0.0)
    with pytest.raises(ValueError, match="named state 'joint_pos'"):
        episode.add_observation(obs, {"joint_pos": np.zeros(6), "gripper_pos": [0.0]}, time_s=0.0)
    episode.add_observation(obs, named, time_s=0.0)
    with pytest.raises(ValueError, match="action has shape"):
        episode.add_step(np.zeros(ACTION_DIM + 1), np.zeros(ACTION_DIM), np.ones(ACTION_DIM, bool),
                         np.ones(ACTION_DIM, bool), True, 0.0, False, False, False, {})
    with pytest.raises(ValueError, match="discard_reason must be one of"):
        episode.finish(status="discarded", discard_reason="bored")


def test_dataset_json_is_plain_json(tmp_path):
    open_writer(tmp_path / "peg")
    header = json.loads((tmp_path / "peg" / "dataset.json").read_text())
    assert header["next_index"] == 0 and header["config"] == {"seed": 1} and header["extra"] == {}
    assert header["recorded_frame_stack"] == 1 and header["task_description"] == "peg"


def test_the_training_view_reads_kept_episodes_only_with_their_columns(tmp_path):
    root = tmp_path / "peg"
    _, written = _write_three(root)
    dataset = Dataset(root)
    rows = list(iter_transitions(dataset, frame_stack=2))
    ids = [dataset.episode(index).meta["episode_id"] for index in (0, 2)]
    assert [(row["episode_id"], row["step_index"]) for row in rows] == (
        [(ids[0], step) for step in (2, 3, 4, 6, 7)] + [(ids[1], step) for step in range(4)])
    first = rows[0]
    assert first["obs"]["state"].shape == (1, 5) and first["obs"]["rgb_wrist"].shape == (1, 2, 9, 15, 3)
    assert first["obs"]["depth_front"].dtype == torch.float32 and first["obs"]["rgb_front"].dtype == torch.uint8
    assert first["grasp_penalty"].dtype == torch.float32 and first["grasp_penalty"].shape == (1,)
    assert "grasp_penalty" not in rows[-1]
    assert first["action"].shape == (1, ACTION_DIM) and first["reward"].shape == (1,)
    assert first["done"].dtype == torch.bool and first["intervened"] is True and first["syncing"] is False
    # The hold step 5 splits the first episode into two training segments.
    assert [row.get("training_segment_end", False) for row in rows] == [False, False, True, False, True,
                                                                        False, False, False, True]
    np.testing.assert_array_equal(rows[1]["obs"]["rgb_wrist"][0].numpy(),
                                  np.stack([written[0][0].obs[row]["rgb_wrist"] for row in (2, 3)]))
