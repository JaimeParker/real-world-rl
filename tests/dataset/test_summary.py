from __future__ import annotations

from real_world_rl.dataset.reader import Dataset
from real_world_rl.dataset.summary import episode_rows, format_episode_table


def test_episode_rows(dataset_root):
    rows = episode_rows(Dataset(dataset_root))
    assert [r["index"] for r in rows] == [0, 3]
    assert rows[0]["status"] == "kept" and rows[0]["success"] is True
    assert rows[0]["intervened_steps"] == 6 and rows[0]["intervened_pct"] == 50.0
    assert rows[1]["discard_reason"] == "rerecord" and rows[1]["notes"] == 1


def test_table_text(dataset_root):
    lines = format_episode_table(Dataset(dataset_root)).splitlines()
    assert lines[0].split() == ["episode", "status", "reason", "success", "steps", "intervened",
                                "duration_s", "notes"]
    assert lines[2].split() == ["000000", "kept", "-", "yes", "12", "50%", "1.2", "1"]
    assert lines[3].split() == ["000003", "discarded", "rerecord", "-", "8", "50%", "0.8", "1"]
    assert lines[-1] == "episodes: 2  kept: 1  discarded: 1  kept transitions: 6"


def test_table_scales_to_1000_episodes(tmp_path):
    class Info:
        def __init__(self, i):
            self.index = i
            self.meta = {"status": "kept" if i % 2 else "discarded", "steps": 10,
                         "intervened_steps": 4, "duration_s": 1.0}

    class Fake:
        def episodes(self):
            return [Info(i) for i in range(1000)]

    text = format_episode_table(Fake())
    assert len(text.splitlines()) == 1000 + 4
    assert text.splitlines()[-1].endswith("kept transitions: 2000")
