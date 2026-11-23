"""Atomic episode writes, resuming a dataset, header mismatches, and drop/undrop/prune."""
from __future__ import annotations

import logging

import numpy as np
import pytest
from _dataset_fakes import OBS_SPEC, open_writer, random_obs, write_episode

from real_world_rl.dataset import Dataset, DatasetMismatchError


def _start_unfinished(writer, rng):
    episode = writer.begin_episode()
    episode.add_observation({key: value[None] for key, value in random_obs(rng).items()},
                            {"joint_pos": np.zeros(7), "gripper_pos": [0.0]}, time_s=0.0)
    return episode


def test_an_aborted_or_crashed_episode_is_invisible_and_cleaned_on_open(tmp_path, caplog):
    root = tmp_path / "peg"
    rng = np.random.default_rng(0)
    writer = open_writer(root)
    write_episode(writer, rng, [True, True])
    aborted = _start_unfinished(writer, rng)
    aborted.abort()
    assert not list(root.glob(".tmp-*"))
    # A crash: the episode is never finished, its .tmp- directory stays behind.
    _start_unfinished(writer, rng)
    assert len(list(root.glob(".tmp-episode_000001-*"))) == 1
    assert [info.index for info in Dataset(root).episodes()] == [0]

    with caplog.at_level(logging.WARNING):
        resumed = open_writer(root)
    assert not list(root.glob(".tmp-*")) and "removing the unfinished episode write" in caplog.text
    assert resumed.next_index == 1 and resumed.num_kept == 1
    _, info = write_episode(resumed, rng, [True])
    assert info.index == 1


def test_appending_continues_the_indices_and_counts_earlier_kept_episodes(tmp_path):
    root = tmp_path / "peg"
    rng = np.random.default_rng(1)
    writer = open_writer(root)
    write_episode(writer, rng, [True])
    write_episode(writer, rng, [True], status="discarded", discard_reason="no success", success=False)
    created_at = Dataset(root).header["created_at"]

    resumed = open_writer(root, task_description="ignored on append")
    assert resumed.next_index == 2 and resumed.num_kept == 1
    _, info = write_episode(resumed, rng, [True, False])
    assert info.index == 2 and resumed.num_kept == 2
    header = Dataset(root).header
    assert header["next_index"] == 3 and header["created_at"] == created_at and header["task_description"] == "peg"


def test_only_one_episode_is_open_at_a_time(tmp_path):
    writer = open_writer(tmp_path / "peg")
    writer.begin_episode()
    with pytest.raises(RuntimeError, match="still open"):
        writer.begin_episode()


@pytest.mark.parametrize("override, key", [
    ({"control_hz": 15.0}, "control_hz"),
    ({"action_dim": 5}, "action_dim"),
    ({"named_state_spec": {"joint_pos": [7]}}, "named_state_spec"),
    ({"obs_spec": {**OBS_SPEC, "state": {"shape": [6], "dtype": "float32"}}}, "obs_spec"),
    ({"action_contract": {"action_mode": "joint_delta"}}, "action_contract"),
])
def test_a_header_mismatch_names_the_differing_key(tmp_path, override, key):
    open_writer(tmp_path / "peg")
    with pytest.raises(DatasetMismatchError, match=key) as raised:
        open_writer(tmp_path / "peg", **override)
    assert raised.value.keys == [key]


def test_headers_compare_after_json_normalization(tmp_path):
    open_writer(tmp_path / "peg")
    spec = {key: {"shape": tuple(entry["shape"]), "dtype": np.dtype(entry["dtype"])} for key, entry in OBS_SPEC.items()}
    open_writer(tmp_path / "peg", obs_spec=spec, control_hz=10, named_state_spec={"gripper_pos": (1,), "joint_pos": (7,)})


def test_a_non_dataset_directory_is_refused(tmp_path):
    (tmp_path / "peg").mkdir()
    (tmp_path / "peg" / "peg.pkl").write_bytes(b"")
    with pytest.raises(ValueError, match="no dataset.json"):
        open_writer(tmp_path / "peg")


def test_prune_never_lets_an_index_be_reused(tmp_path):
    root = tmp_path / "peg"
    rng = np.random.default_rng(2)
    writer = open_writer(root)
    write_episode(writer, rng, [True])
    write_episode(writer, rng, [True], status="discarded", discard_reason="rerecord", success=False)
    dataset = Dataset(root)
    assert dataset.prune() == [1]
    assert [info.index for info in dataset.episodes()] == [0] and not list(root.glob(".tmp-*"))

    resumed = open_writer(root)
    assert resumed.next_index == 2
    _, info = write_episode(resumed, rng, [True])
    assert info.index == 2 and (root / "episode_000002").is_dir() and not (root / "episode_000001").exists()


def test_a_header_left_behind_by_a_crash_after_the_rename_is_reconciled(tmp_path):
    root = tmp_path / "peg"
    rng = np.random.default_rng(3)
    writer = open_writer(root)
    write_episode(writer, rng, [True])
    writer._set_next_index(0)  # the rename happened, the header bump did not
    assert open_writer(root).next_index == 1


def test_drop_undrop_and_prune(tmp_path):
    root = tmp_path / "peg"
    rng = np.random.default_rng(4)
    writer = open_writer(root)
    for _ in range(2):
        write_episode(writer, rng, [True])
    write_episode(writer, rng, [True], status="discarded", discard_reason="failure event", success=False)
    dataset = Dataset(root)

    dataset.drop(1, note="gripper slipped")
    meta = dataset.episode(1).meta
    assert (meta["status"], meta["discard_reason"], meta["recorded_discard_reason"]) == ("discarded", "manual", None)
    assert [(h["action"], h["note"]) for h in meta["history"]] == [("drop", "gripper slipped")]
    with pytest.raises(ValueError, match="already discarded"):
        dataset.drop(1)

    dataset.undrop(1)
    meta = dataset.episode(1).meta
    assert (meta["status"], meta["discard_reason"]) == ("kept", None)
    assert [h["action"] for h in meta["history"]] == ["drop", "undrop"]
    with pytest.raises(ValueError, match="not dropped"):
        dataset.undrop(1)
    # Discarded when recorded: cannot be undropped.
    with pytest.raises(ValueError, match="discarded when recorded"):
        dataset.undrop(2)
    with pytest.raises(KeyError):
        dataset.drop(7)

    dataset.drop(0)
    assert dataset.prune() == [0, 2]
    assert [(info.index, info.status) for info in dataset.episodes()] == [(1, "kept")]
    assert dataset.prune() == []
