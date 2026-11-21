"""``rwrl use``: remembered station and preset, their lookup order and where they never apply."""
from __future__ import annotations

import argparse
import json
import shutil
from pathlib import Path

import pytest

from real_world_rl import cli
from real_world_rl.cli import registry
from real_world_rl.cli.cmd_learning import LearningCommand
from real_world_rl.cli.command import Result
from real_world_rl.cli.errors import CliError
from real_world_rl.common import station_paths
from real_world_rl.discovery.autoconfig import resolve_station_config_path

EXAMPLES = Path(__file__).resolve().parents[1] / "site" / "examples"


@pytest.fixture
def workdir(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    monkeypatch.setenv("XDG_CONFIG_HOME", str(tmp_path / "xdg"))
    (tmp_path / "configs").mkdir()
    shutil.copy(EXAMPLES / "mock-station.yaml", tmp_path / "configs" / "lab.yaml")
    shutil.copy(EXAMPLES / "task.yaml", tmp_path / "configs" / "pick.yaml")
    return tmp_path


def rwrl(*argv):
    return cli.main(list(argv))


def test_use_remembers_station_and_preset_relative_to_the_directory(workdir, capsys):
    assert rwrl("use", "station", "configs/lab.yaml") == 0
    assert rwrl("use", "preset", "./configs/pick.yaml") == 0
    stored = json.loads((workdir / "local" / ".rwrl-use.json").read_text())
    assert stored == {"station": "configs/lab.yaml", "preset": "configs/pick.yaml"}
    capsys.readouterr()
    assert rwrl("--json", "use") == 0
    shown = json.loads(capsys.readouterr().out)
    assert shown["station"] == {"path": "configs/lab.yaml", "source": "rwrl use"}
    assert shown["preset"] == {"path": "configs/pick.yaml", "source": "rwrl use"}


def test_use_refuses_files_that_do_not_load(workdir, capsys):
    (workdir / "configs" / "broken.yaml").write_text("robots: [\n")
    assert rwrl("use", "station", "configs/missing.yaml") != 0
    assert rwrl("use", "station", "configs/broken.yaml") != 0
    (workdir / "configs" / "list.yaml").write_text("- 1\n")
    assert rwrl("use", "preset", "configs/list.yaml") != 0
    assert not (workdir / "local" / ".rwrl-use.json").exists()
    assert "does not" in capsys.readouterr().err


def test_use_without_path_is_a_usage_error(workdir):
    assert rwrl("use", "preset") == 2


def test_clear_forgets_one_kind_or_both(workdir):
    rwrl("use", "station", "configs/lab.yaml")
    rwrl("use", "preset", "configs/pick.yaml")
    assert rwrl("use", "--clear", "preset") == 0
    assert station_paths.read_selection() == {"station": "configs/lab.yaml"}
    assert rwrl("use", "--clear") == 0
    assert not (workdir / "local" / ".rwrl-use.json").exists()


def test_clear_repairs_a_damaged_selection_file(workdir):
    (workdir / "local").mkdir()
    (workdir / "local" / ".rwrl-use.json").write_text("{not json")
    with pytest.raises(ValueError, match="rwrl use --clear"):
        station_paths.read_selection()
    assert rwrl("use", "--clear") == 0
    assert station_paths.read_selection() == {}


def test_station_lookup_order(workdir, monkeypatch, capsys):
    (workdir / "local").mkdir()
    shutil.copy(EXAMPLES / "mock-station.yaml", workdir / "local" / "station.yaml")
    assert resolve_station_config_path().resolve() == (workdir / "local" / "station.yaml").resolve()

    station_paths.write_selection({"station": "configs/lab.yaml"})
    assert resolve_station_config_path() == Path("configs/lab.yaml")
    assert capsys.readouterr().err.count("using station configs/lab.yaml (from rwrl use)") == 1
    resolve_station_config_path()
    assert capsys.readouterr().err == ""  # announced once per process

    assert resolve_station_config_path("other.yaml") == Path("other.yaml")
    monkeypatch.setenv("RWRL_STATION", "env.yaml")
    assert resolve_station_config_path() == Path("env.yaml")


def test_a_stale_station_selection_is_an_error_not_a_silent_fallback(workdir):
    (workdir / "local").mkdir()
    shutil.copy(EXAMPLES / "mock-station.yaml", workdir / "local" / "station.yaml")
    station_paths.write_selection({"station": "configs/gone.yaml"})
    with pytest.raises(FileNotFoundError, match="rwrl use --clear station"):
        resolve_station_config_path()


class Echo:
    name = "echo"
    help = "report the preset"

    def register(self, parser):
        pass

    def run(self, args, ctx):
        return Result({"preset": args.preset}, f"preset={args.preset}")


def echo(*argv):
    return registry.main(list(argv), commands=[Echo()], prog="rwrl", description="test")


def test_a_command_falls_back_to_the_remembered_preset(workdir, capsys):
    station_paths.write_selection({"preset": "configs/pick.yaml"})
    assert echo("echo") == 0
    captured = capsys.readouterr()
    assert captured.out.strip() == "preset=configs/pick.yaml"
    assert "using preset configs/pick.yaml (from rwrl use)" in captured.err

    station_paths._announced.clear()
    assert echo("--json", "echo") == 0
    payload = json.loads(capsys.readouterr().out)
    assert payload == {"preset": "configs/pick.yaml", "from_rwrl_use": {"preset": "configs/pick.yaml"}}


def test_an_explicit_preset_wins_and_is_not_announced(workdir, capsys):
    station_paths.write_selection({"preset": "configs/pick.yaml"})
    assert echo("--preset", "mine.yaml", "echo") == 0
    captured = capsys.readouterr()
    assert captured.out.strip() == "preset=mine.yaml"
    assert captured.err == ""


def test_a_stale_preset_selection_is_a_usage_error(workdir, capsys):
    station_paths.write_selection({"preset": "configs/gone.yaml"})
    assert echo("echo") == 2
    assert "rwrl use --clear preset" in capsys.readouterr().err


def test_learning_commands_never_receive_the_remembered_preset(workdir):
    station_paths.write_selection({"preset": "configs/pick.yaml"})
    command = LearningCommand("train", "Train", "real_world_rl.training.__main__:main")
    args = argparse.Namespace(preset=None)
    registry.fill_selected_preset(args, command)
    assert args.preset is None
    for option in ("station", "robot", "direct", "mock", "json", "yes"):
        setattr(args, option, None)
    assert command.entrypoint_argv(args, ["--seed", "1"]) == ["--seed", "1"]


def test_teleop_run_without_a_preset_points_at_rwrl_use(workdir, capsys):
    shutil.copy(EXAMPLES / "mock-station.yaml", workdir / "station.yaml")
    assert rwrl("--station", "station.yaml", "teleop", "run", "--backend", "spacemouse", "--record", "demo.pkl") == 2
    assert "rwrl use preset" in capsys.readouterr().err


def test_fill_skips_parsers_without_a_preset_option():
    args = argparse.Namespace()
    registry.fill_selected_preset(args, Echo())
    assert not hasattr(args, "preset")


def test_the_selection_error_reaches_the_user_as_a_cli_error(workdir):
    (workdir / "local").mkdir()
    (workdir / "local" / ".rwrl-use.json").write_text("[]")
    with pytest.raises(CliError, match="JSON object"):
        registry.fill_selected_preset(argparse.Namespace(preset=None), Echo())
