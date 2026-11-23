"""LeRobot v3.0 export of the kept episodes (needs lerobot 0.7.0; skipped without it)."""
from __future__ import annotations

import numpy as np
import pytest
from _cli_fakes import EPISODES, build_dataset

pytest.importorskip("lerobot.datasets.lerobot_dataset")

from real_world_rl.cli import main  # noqa: E402
from real_world_rl.cli.errors import SUCCESS_EXIT_CODE, USAGE_EXIT_CODE  # noqa: E402
from real_world_rl.dataset import Dataset  # noqa: E402
from real_world_rl.dataset.lerobot_export import export_lerobot, lerobot_features  # noqa: E402


@pytest.fixture
def exported(tmp_path):
    root = tmp_path / "peg"
    written = build_dataset(root)
    summary = export_lerobot(Dataset(root), tmp_path / "lerobot", repo_id="rwrl/peg")
    return root, written, summary, tmp_path / "lerobot"


def test_round_trip(exported):
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    root, written, summary, out = exported
    kept = [index for index, (_, status, _) in enumerate(EPISODES) if status == "kept"]
    assert summary["episode_indices"] == kept and summary["skipped_depth_keys"] == ["depth_front"]
    loaded = LeRobotDataset("rwrl/peg", root=out)
    assert loaded.num_episodes == len(kept)
    assert loaded.num_frames == sum(EPISODES[i][0] for i in kept) == summary["frames"]
    assert loaded.fps == 10
    assert loaded.meta.video_keys == ["observation.images.wrist"]
    assert "observation.images.depth_front" not in loaded.meta.features
    first = loaded[0]
    expected = written[kept[0]]
    np.testing.assert_allclose(first["observation.state"].numpy(), expected["state"][0])
    np.testing.assert_allclose(first["observation.joint_pos"].numpy(), expected["joint_pos"][0], atol=1e-6)
    np.testing.assert_allclose(first["action"].numpy(), expected["action"][0])
    assert first["task"] == "insert the peg"
    assert first["observation.images.wrist"].shape == (3, 12, 16)
    # Frame index 6 is the first frame of the second kept episode (episode 2 of the dataset).
    second = loaded[EPISODES[0][0]]
    assert second["episode_index"] == 1
    np.testing.assert_allclose(second["action"].numpy(), written[kept[1]]["action"][0])
    steps = EPISODES[0][0]
    flags = [bool(loaded[i]["intervened"]) for i in range(steps)]
    assert flags == list(expected["intervened"])
    np.testing.assert_array_equal(loaded[steps - 1]["human_action_mask"].numpy(), expected["human_action_mask"][-1])


def test_features_skip_depth_and_reject_odd_frames():
    header = {"obs_spec": {"state": {"shape": [3], "dtype": "float32"},
                           "rgb_a": {"shape": [4, 6, 3], "dtype": "uint8"},
                           "depth_b": {"shape": [4, 6, 1], "dtype": "float32"}},
              "named_state_spec": {"tcp_pose": [7]}, "action_dim": 2}
    features, rgb, skipped = lerobot_features(header)
    assert rgb == ["rgb_a"] and skipped == ["depth_b"]
    assert features["observation.tcp_pose"]["shape"] == (7,)
    header["obs_spec"]["rgb_a"]["shape"] = [5, 6, 3]
    with pytest.raises(ValueError, match="even"):
        lerobot_features(header)


def test_cli_export_and_refusal(tmp_path, capsys):
    root = tmp_path / "peg"
    build_dataset(root)
    out = tmp_path / "lr"
    assert main(["dataset", "export", str(root), "--format", "lerobot", "--out", str(out),
                 "--repo-id", "rwrl/peg"]) == SUCCESS_EXIT_CODE
    assert "2 episodes, 10 frames, 10 fps" in capsys.readouterr().out
    assert (out / "meta" / "info.json").is_file()
    assert main(["dataset", "export", str(root), "--format", "lerobot", "--out", str(out)]) == USAGE_EXIT_CODE
