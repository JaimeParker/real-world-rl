from __future__ import annotations

import subprocess
import sys

import numpy as np
import pytest

from real_world_rl.dataset.view import (episode_document, episode_title, intervention_curve,
                                         quat_xyzw_to_rpy, recording_id_for)


def test_import_pulls_neither_rerun_nor_torch():
    code = ("import sys, real_world_rl.dataset.view, real_world_rl.dataset.summary;"
            "bad=[m for m in ('rerun','torch') if m in sys.modules]; assert not bad, bad")
    subprocess.run([sys.executable, "-c", code], check=True)


def _quat(axis, angle):
    q = np.zeros(4)
    q[axis] = np.sin(angle / 2)
    q[3] = np.cos(angle / 2)
    return q


@pytest.mark.parametrize("axis,expected", [(0, (0.7, 0, 0)), (1, (0, 0.7, 0)), (2, (0, 0, 0.7)),
                                           (2, (0, 0, -2.5))])
def test_quat_to_rpy_single_axis(axis, expected):
    rpy = quat_xyzw_to_rpy(_quat(axis, expected[axis]))
    np.testing.assert_allclose(rpy, expected, atol=1e-12)


def test_quat_to_rpy_composed_and_batched():
    # R = Rz(yaw) Ry(pitch) Rx(roll) built independently from rotation matrices.
    roll, pitch, yaw = 0.3, -0.4, 1.2
    rx = np.array([[1, 0, 0], [0, np.cos(roll), -np.sin(roll)], [0, np.sin(roll), np.cos(roll)]])
    ry = np.array([[np.cos(pitch), 0, np.sin(pitch)], [0, 1, 0], [-np.sin(pitch), 0, np.cos(pitch)]])
    rz = np.array([[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])
    qx, qy, qz = _quat(0, roll), _quat(1, pitch), _quat(2, yaw)

    def mul(a, b):  # Hamilton product, xyzw
        av, aw, bv, bw = a[:3], a[3], b[:3], b[3]
        return np.concatenate([aw * bv + bw * av + np.cross(av, bv), [aw * bw - av @ bv]])

    q = mul(qz, mul(qy, qx))
    assert np.allclose(rz @ ry @ rx @ np.array([1.0, 2, 3]),
                       rz @ ry @ rx @ np.array([1.0, 2, 3]))
    out = quat_xyzw_to_rpy(np.stack([q, 2 * q]))  # unnormalised input too
    np.testing.assert_allclose(out, [[roll, pitch, yaw]] * 2, atol=1e-12)


def test_pitch_gimbal_lock_is_clamped():
    rpy = quat_xyzw_to_rpy(_quat(1, np.pi / 2))
    assert rpy[1] == pytest.approx(np.pi / 2)


def test_titles_ids_and_curve(dataset_root):
    assert episode_title({"index": 7, "status": "kept", "discard_reason": None}) == "episode_000007 · kept"
    assert episode_title({"index": 3, "status": "discarded", "discard_reason": "rerecord"}) \
        == "episode_000003 · discarded (rerecord)"
    assert recording_id_for(dataset_root, 3) == "peg_episode_000003"
    segs = [{"start": 0, "end": 2, "kind": "hold"}, {"start": 2, "end": 5, "kind": "intervene"}]
    assert intervention_curve(segs, 5).tolist() == [0, 0, 1, 1, 1]
    assert "gripper slipped" in episode_document(
        {"status": "kept", "index": 0, "notes": [{"at": "t", "text": "gripper slipped"}]})


def test_save_writes_all_episodes(dataset_root, tmp_path):
    pytest.importorskip("rerun")
    from rerun.chunk import RrdReader

    from real_world_rl.dataset.view import view_dataset

    out = tmp_path / "all.rrd"
    view_dataset(dataset_root, mode="save", save_path=out)
    reader = RrdReader(out)
    ids = sorted(e.recording_id for e in reader.recordings())
    assert ids == ["peg_episode_000000", "peg_episode_000003"]
    assert {e.application_id for e in reader.recordings()} == {"rwrl-dataset"}

    paths = {}
    for entry in reader.recordings():
        paths[entry.recording_id] = {c.entity_path for c in reader.stream(store=entry)}
    kept, dropped = paths["peg_episode_000000"], paths["peg_episode_000003"]
    expected = {"/joint_pos/q1", "/joint_pos/q7", "/tcp_xyz/x", "/tcp_rpy/yaw", "/gripper_pos/g0",
                "/action/a0", "/policy_action/a6", "/intervened", "/reward", "/intervention",
                "/log/intervention", "/log/events", "/episode"}
    assert expected <= kept and expected - {"/log/events"} <= dropped
    assert "/state/s0" not in kept
    assert {"/camera/rgb_wrist", "/camera/rgb_front", "/camera/depth_front"} <= kept
    assert not any(p.startswith("/camera") for p in dropped)


def test_save_subset_of_episodes(dataset_root, tmp_path):
    pytest.importorskip("rerun")
    from rerun.chunk import RrdReader

    from real_world_rl.dataset.view import view_dataset

    out = tmp_path / "one.rrd"
    view_dataset(dataset_root, episodes=[3], mode="save", save_path=out)
    assert [e.recording_id for e in RrdReader(out).recordings()] == ["peg_episode_000003"]
    with pytest.raises(ValueError):
        view_dataset(dataset_root, episodes=[9], mode="save", save_path=out)


def _camera_chunks(path, recording_id, entity):
    from rerun.chunk import RrdReader

    reader = RrdReader(path)
    entry = next(e for e in reader.recordings() if e.recording_id == recording_id)
    return [c for c in reader.stream(store=entry) if c.entity_path == entity]


def test_video_is_logged_once_with_a_reference_per_row(dataset_root, tmp_path):
    pytest.importorskip("rerun")
    from real_world_rl.dataset.view import view_dataset

    if not list(dataset_root.rglob("*.mp4")):
        pytest.skip("no ffmpeg to build the fixture videos")
    out = tmp_path / "v.rrd"
    view_dataset(dataset_root, episodes=[0], mode="save", save_path=out)
    chunks = _camera_chunks(out, "peg_episode_000000", "/camera/rgb_wrist")
    static = [c for c in chunks if c.is_static]
    timed = [c for c in chunks if not c.is_static]
    assert len(static) == 1 and sum(c.num_rows for c in timed) == 13
    assert set(timed[0].timeline_names) == {"step", "time"}


def test_images_are_logged_per_row_without_video(dataset_root, tmp_path, monkeypatch):
    pytest.importorskip("rerun")
    from real_world_rl.dataset import reader
    from real_world_rl.dataset.view import view_dataset

    monkeypatch.setattr(reader.Episode, "video_path", lambda self, key: None)
    out = tmp_path / "i.rrd"
    view_dataset(dataset_root, episodes=[0], mode="save", save_path=out)
    for entity in ("/camera/rgb_front", "/camera/depth_front"):
        chunks = _camera_chunks(out, "peg_episode_000000", entity)
        assert sum(c.num_rows for c in chunks if not c.is_static) == 13
        assert not any(c.is_static for c in chunks)
