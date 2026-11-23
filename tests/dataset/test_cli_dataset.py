"""``rwrl dataset``: list, view, drop, undrop, prune and export (pkl) on a synthetic dataset."""
from __future__ import annotations

import json
from pathlib import Path

import pytest
from _cli_fakes import build_dataset

from real_world_rl.cli import main
from real_world_rl.cli.errors import SUCCESS_EXIT_CODE, USAGE_EXIT_CODE
from real_world_rl.dataset import Dataset


@pytest.fixture
def root(tmp_path) -> Path:
    path = tmp_path / "peg"
    build_dataset(path)
    return path


def statuses(root) -> dict[int, str]:
    return {info.index: info.status for info in Dataset(root).episodes()}


def test_list_text_and_json(root, capsys):
    assert main(["dataset", "list", str(root)]) == SUCCESS_EXIT_CODE
    text = capsys.readouterr().out
    assert "000001" in text and "rerecord" in text
    assert "episodes: 3  kept: 2  discarded: 1" in text
    assert main(["--json", "dataset", "list", str(root)]) == SUCCESS_EXIT_CODE
    payload = json.loads(capsys.readouterr().out)
    assert [row["index"] for row in payload["episodes"]] == [0, 1, 2]
    assert payload["totals"] == {"episodes": 3, "kept": 2, "discarded": 1, "kept_transitions": 3 + 2}


def test_not_a_dataset_is_a_usage_error(tmp_path, capsys):
    assert main(["dataset", "list", str(tmp_path)]) == USAGE_EXIT_CODE
    assert "not a dataset" in capsys.readouterr().err


@pytest.mark.parametrize("option", ["--station", "--robot", "--preset"])
def test_station_options_are_not_applicable(root, option, capsys):
    assert main([option, "x", "dataset", "list", str(root)]) == USAGE_EXIT_CODE
    assert "not applicable" in capsys.readouterr().err


def test_drop_and_undrop(root, capsys):
    assert main(["dataset", "drop", str(root), "0", "--note", "shaky"]) == SUCCESS_EXIT_CODE
    assert statuses(root)[0] == "discarded"
    assert Dataset(root).episode(0).meta["history"][-1]["note"] == "shaky"
    assert main(["dataset", "undrop", str(root), "0"]) == SUCCESS_EXIT_CODE
    assert statuses(root)[0] == "kept"
    capsys.readouterr()
    # Already discarded at record time: cannot be undropped; unknown episode; double drop.
    assert main(["dataset", "undrop", str(root), "1"]) == USAGE_EXIT_CODE
    assert "cannot be undropped" in capsys.readouterr().err
    assert main(["dataset", "drop", str(root), "9"]) == USAGE_EXIT_CODE
    assert "no episode 9" in capsys.readouterr().err
    assert main(["dataset", "drop", str(root), "1"]) == USAGE_EXIT_CODE
    assert "already discarded" in capsys.readouterr().err


def test_prune_needs_yes(root, capsys):
    assert main(["dataset", "drop", str(root), "2"]) == SUCCESS_EXIT_CODE
    capsys.readouterr()
    assert main(["dataset", "prune", str(root)]) == USAGE_EXIT_CODE
    err = capsys.readouterr().err
    assert "episode 000001" in err and "episode 000002" in err and "--yes" in err
    assert set(statuses(root)) == {0, 1, 2}
    assert main(["dataset", "prune", str(root), "--yes"]) == SUCCESS_EXIT_CODE
    assert "removed episodes 1, 2" in capsys.readouterr().out
    assert set(statuses(root)) == {0}


def test_prune_accepts_the_global_yes_and_json(root, capsys):
    assert main(["--yes", "--json", "dataset", "prune", str(root)]) == SUCCESS_EXIT_CODE
    assert json.loads(capsys.readouterr().out)["removed"] == [1]
    assert main(["dataset", "prune", str(root)]) == SUCCESS_EXIT_CODE  # nothing left to delete
    assert "no discarded episodes" in capsys.readouterr().out


def test_export_pkl_loads_with_the_demo_loader(root, tmp_path, capsys):
    torch = pytest.importorskip("torch")
    from real_world_rl.training.demo_loading import load_demo_pkls

    out = tmp_path / "out" / "peg.pkl"
    assert main(["dataset", "export", str(root), "--format", "pkl", "--out", str(out)]) == SUCCESS_EXIT_CODE
    rows = []
    added = load_demo_pkls([str(out)], lambda obs, next_obs, action, reward, done, **extra: rows.append(obs),
                           torch.device("cpu"))
    assert added == len(rows) == 3 + 2  # intervened steps of the two kept episodes
    assert rows[0]["rgb_wrist"].shape == (1, 12, 16, 3)
    assert main(["dataset", "export", str(root), "--format", "pkl", "--out", str(out),
                 "--frame-stack", "2"]) == SUCCESS_EXIT_CODE
    stacked = []
    load_demo_pkls([str(out)], lambda obs, *a, **k: stacked.append(obs), torch.device("cpu"))
    assert stacked[0]["rgb_wrist"].shape == (1, 2, 12, 16, 3)


def test_view_save_writes_an_rrd(root, tmp_path, capsys):
    pytest.importorskip("rerun")
    out = tmp_path / "peg.rrd"
    assert main(["dataset", "view", str(root), "--save", str(out), "-e", "0", "-e", "2"]) == SUCCESS_EXIT_CODE
    assert out.stat().st_size > 0
    assert "recording list" in capsys.readouterr().out
    assert main(["dataset", "view", str(root), "--save", str(out), "-e", "7"]) == USAGE_EXIT_CODE


def test_view_without_rerun_hints_the_extra(root, tmp_path, monkeypatch, capsys):
    monkeypatch.setitem(__import__("sys").modules, "rerun", None)  # makes `import rerun` fail
    assert main(["dataset", "view", str(root), "--save", str(tmp_path / "x.rrd")]) == USAGE_EXIT_CODE
    err = capsys.readouterr().err
    assert "pip install -e '.[viewer]'" in err
