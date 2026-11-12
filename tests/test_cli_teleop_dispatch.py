"""`rwrl teleop` operations dispatched to the device slices: station defaults, --backend, the
refusal of an unimplemented operation, help grouped by backend, lazily imported slices."""
from __future__ import annotations

import argparse
import json
import subprocess
import sys
import types
from pathlib import Path

import pytest
import yaml

from real_world_rl.cli import main
from real_world_rl.cli import teleop_dispatch
from real_world_rl.cli.command import Result

ROOT = Path(__file__).resolve().parents[1]
HANDLERS = {"spacemouse": ("calibrate", "diagnose", "run_session", "profiles", "probe_hid"), "gello": ("calibrate",)}


def station(tmp_path, backend):
    config = {"type": "franka", "name": "left", "arm_backend": "fake"}
    if backend is not None:
        config["teleop"] = {"backend": backend}
    path = tmp_path / "station.yaml"
    path.write_text(yaml.safe_dump(config))
    return path


@pytest.fixture
def calls(monkeypatch):
    """Replaces every slice handler by one that records ``(backend, handler)``."""
    recorded = []
    for backend, names in HANDLERS.items():
        for name in names:
            def handler(args, ctx, backend=backend, name=name):
                recorded.append((backend, name))
                return Result({"backend": backend, "handler": name}, "")
            monkeypatch.setattr(f"real_world_rl.teleop.{backend}.cli.{name}", handler)
    return recorded


def error_of(capsys):
    return json.loads(capsys.readouterr().out)


def test_each_operation_lists_the_slices_that_implement_it():
    assert teleop_dispatch.implementers("calibrate") == ("spacemouse", "gello")
    for operation in ("diagnose", "run", "profiles", "probe-hid"):
        assert teleop_dispatch.implementers(operation) == ("spacemouse",)


@pytest.mark.parametrize("backend", ["spacemouse", "gello"])
def test_calibrate_defaults_to_the_station_teleop_backend(tmp_path, calls, backend):
    assert main(["--json", "--station", str(station(tmp_path, backend)), "teleop", "calibrate"]) == 0
    assert calls == [(backend, "calibrate")]


def test_backend_flag_overrides_the_station_teleop(tmp_path, calls):
    assert main(["--json", "--station", str(station(tmp_path, "gello")), "teleop", "calibrate",
                 "--backend", "spacemouse"]) == 0
    assert calls == [("spacemouse", "calibrate")]


@pytest.mark.parametrize("backend,command", [
    ("fake", ["calibrate"]),
    ("fake", ["diagnose"]),
    ("gello", ["diagnose"]),
    ("gello", ["profiles", "list"]),
    ("gello", ["run", "--record", "unused.pkl"]),
])
def test_an_unimplemented_operation_is_one_usage_error(tmp_path, calls, capsys, backend, command):
    assert main(["--json", "--station", str(station(tmp_path, backend)), "teleop", *command]) == 2
    error = error_of(capsys)
    assert error["error"] == f"backend '{backend}' does not implement teleop {command[0]}"
    assert error["hint"] == "implemented by: " + ", ".join(teleop_dispatch.implementers(command[0]))
    assert calls == []


def test_a_station_without_teleop_needs_backend_only_where_there_is_a_choice(tmp_path, calls, capsys):
    path = station(tmp_path, None)
    assert main(["--json", "--station", str(path), "teleop", "calibrate"]) == 2
    error = error_of(capsys)
    assert error["error"] == "station has no teleop; pass --backend" and error["required_fields"] == ["--backend"]
    assert main(["--json", "--station", str(path), "teleop", "diagnose"]) == 0
    assert calls == [("spacemouse", "diagnose")]


def test_flags_of_another_backend_are_refused_before_its_handler(tmp_path, calls, capsys):
    path = station(tmp_path, "spacemouse")
    assert main(["--json", "--station", str(path), "teleop", "calibrate", "--backend", "gello",
                 "--profile", "front", "--save"]) == 2
    assert error_of(capsys)["error"] == "--profile, --save not applicable to --backend gello"
    assert calls == []


def test_probe_hid_runs_whatever_the_station_teleop_is(tmp_path, calls):
    assert main(["--json", "--station", str(station(tmp_path, "gello")), "teleop", "probe-hid"]) == 0
    assert calls == [("spacemouse", "probe_hid")]


def test_operation_help_groups_the_flags_by_backend(capsys):
    assert main(["teleop", "calibrate", "--help"]) == 0
    text = capsys.readouterr().out
    spacemouse, gello = text.index("\nspacemouse:\n"), text.index("\ngello:\n")
    assert "--backend {spacemouse,gello}" in text[:spacemouse] and "--use" in text[:spacemouse]
    assert "--profile" in text[spacemouse:gello] and "--port" not in text[spacemouse:gello]
    assert "--port" in text[gello:] and "--profile" not in text[gello:]
    assert main(["teleop", "diagnose", "--help"]) == 0
    text = capsys.readouterr().out
    assert "\nspacemouse:\n" in text and "\ngello:\n" not in text and "--backend" not in text


def test_two_slices_cannot_declare_the_same_flag(monkeypatch):
    clashing = types.ModuleType("clashing_teleop_cli")
    clashing.calibrate = lambda args, ctx: None
    clashing.add_calibrate_arguments = lambda parser: parser.add_argument("--port")
    monkeypatch.setitem(sys.modules, clashing.__name__, clashing)
    monkeypatch.setitem(teleop_dispatch.TELEOP_SLICES, "clashing", clashing.__name__)
    with pytest.raises(argparse.ArgumentError):
        teleop_dispatch.add_operation_arguments(argparse.ArgumentParser(), "calibrate")


LOADED_SLICES = '''
import sys
from real_world_rl.cli import main
main(sys.argv[1:])
print("SLICES=" + ",".join(sorted(name for name in sys.modules if name.startswith("real_world_rl.teleop"))),
      file=sys.stderr)
'''


@pytest.mark.parametrize("argv,loaded", [
    (["--help"], False),
    (["teleop", "--help"], False),
    (["teleop", "calibrate", "--help"], True),
])
def test_slices_load_only_for_the_selected_operation(argv, loaded):
    result = subprocess.run([sys.executable, "-c", LOADED_SLICES, *argv], cwd=ROOT, capture_output=True,
                            text=True, timeout=120)
    assert result.returncode == 0, result.stderr
    line = next(row for row in result.stderr.splitlines() if row.startswith("SLICES="))
    modules = set(filter(None, line[len("SLICES="):].split(",")))
    if loaded:
        assert {"real_world_rl.teleop.spacemouse.cli", "real_world_rl.teleop.gello.cli"} <= modules
    else:
        assert not modules, modules
