"""Preset precedence, exact backups, limits, and home-only reset behavior."""
from __future__ import annotations

import json
from pathlib import Path
import sys

import numpy as np
import pytest
import yaml

from real_world_rl.cli import main
from real_world_rl.cli.preset_edit import Preset


@pytest.fixture
def station(tmp_path, monkeypatch):
    monkeypatch.setattr(sys.stdin, "isatty", lambda: False)
    path = tmp_path / "station.yaml"
    path.write_text(yaml.safe_dump({"type": "franka", "arm_backend": "fake", "claim": False,
                                    "gripper": {"backend": "fake"}, "home_joints": [0.3] * 7}))
    return path


def call(station, preset, capsys, action, *, code=0, yes=False):
    args = ["--station", str(station), "--mock", "--json"]
    if preset is not None:
        args += ["--preset", str(preset)]
    if yes:
        args += ["--yes"]
    assert main([*args, "home", action]) == code
    captured = capsys.readouterr()
    return json.loads(captured.out)


@pytest.mark.parametrize("wrapped", [True, False])
def test_home_set_roundtrip_and_backup(station, tmp_path, capsys, wrapped):
    source = Path(__file__).resolve().parents[1] / "examples/presets/pick_and_place_fake.yaml"
    original = source.read_text(encoding="utf-8")
    original = original.replace('    name: pick_and_place', '    name: "pick_and_place" # task label\n')
    original = original.replace('    reset:\n', '    reset:\n      home_joints: [0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2]\n')
    if not wrapped:
        original = "# bare env preset\n" + yaml.safe_dump(yaml.safe_load(original)["real_robot"], sort_keys=False)
    preset = tmp_path / "preset.yaml"
    preset.write_text(original, encoding="utf-8")
    backup = Path(str(preset) + ".bak")
    backup.write_text("stale backup")
    result = call(station, preset, capsys, "set")
    backup = Path(result["backup"])
    assert backup.read_bytes() == original.encode("utf-8")
    loaded = yaml.safe_load(preset.read_text())
    env = loaded["real_robot"] if wrapped else loaded
    assert env["task"]["reset"]["home_joints"] == [0.0] * 7
    assert all(type(value) is float for value in env["task"]["reset"]["home_joints"])
    assert env["task"]["reset"]["xyz_jitter_m"] == [0.01, 0.01, 0.0]
    assert "warning" not in result
    from test_yaml_edit import assert_only_key_changed
    key = ('real_robot', 'task', 'reset', 'home_joints') if wrapped else ('task', 'reset', 'home_joints')
    assert_only_key_changed(original, preset.read_text(), key)
    shown = call(station, preset, capsys, "show")
    assert shown["source"] == "preset task.reset.home_joints"
    assert shown["home_joints"] == [0.0] * 7


@pytest.mark.parametrize("document", [{}, {"task": {}}, {"task": {"reset": {}}},
                                      {"real_robot": {"task": {"reset": None}}}])
def test_home_set_creates_missing_layers(station, tmp_path, capsys, document):
    preset = tmp_path / "preset.yaml"
    preset.write_text(yaml.safe_dump(document))
    call(station, preset, capsys, "set")
    data = yaml.safe_load(preset.read_text())
    assert data.get("real_robot", data)["task"]["reset"]["home_joints"] == [0.0] * 7


def test_home_set_rounds_measured_q(station, tmp_path, capsys, monkeypatch):
    from real_world_rl.devices.backends.arms.fake import FakeArm

    read = FakeArm.read_state
    measured = [0.123456, -0.234567, 0.00001, 0.345678, -0.456789, 0.567891, 0.678912]

    def state(self):
        return {**read(self), "joint_pos": np.array(measured)}

    monkeypatch.setattr(FakeArm, "read_state", state)
    preset = tmp_path / "preset.yaml"
    preset.write_text("{}\n")
    result = call(station, preset, capsys, "set")
    assert result["home_joints"] == [round(value, 4) for value in measured]


def test_home_set_six_joints(tmp_path):
    preset_path = tmp_path / "preset.yaml"
    preset_path.write_text("{}\n", encoding="utf-8")
    current = [0.123456, -0.234567, 0.345678, -0.456789, 0.567891, -0.678912]
    result = Preset.load(str(preset_path)).save_home(current, [], num_joints=6)
    written = yaml.safe_load(preset_path.read_text(encoding="utf-8"))["task"]["reset"]["home_joints"]
    assert len(written) == 6
    assert written == [round(value, 4) for value in current]
    assert result["home_joints"] == written


@pytest.mark.parametrize("where", ["station", "preset", "preset_robot"])
def test_home_set_outside_limits_leaves_files_untouched(station, tmp_path, capsys, where):
    preset = tmp_path / "preset.yaml"
    data = {"task": {"reset": {"home_joints": [0.3] * 7}}}
    limits = [[0.1, 0.6]] * 7  # Station home is valid; fake's current q=0 is outside.
    if where == "station":
        station_data = yaml.safe_load(station.read_text())
        station_data["joint_limits"] = limits
        station.write_text(yaml.safe_dump(station_data))
    elif where == "preset":
        data["joint_limits"] = limits
    else:
        data["robot"] = {"joint_limits": limits}
    preset.write_text(yaml.safe_dump(data))
    original = preset.read_bytes()
    backup = Path(str(preset) + ".bak")
    backup.write_text("keep this backup")
    result = call(station, preset, capsys, "set", code=1)
    assert "outside" in result["error"]
    assert preset.read_bytes() == original
    assert backup.read_text() == "keep this backup"


@pytest.mark.parametrize("document,expected,source", [
    ({"home_joints": [0.2] * 7, "task": {"reset": {"home_joints": [0.4] * 7}}},
     [0.4] * 7, "preset task.reset.home_joints"),
    ({"home_joints": [0.2] * 7, "task": {"reset": {"home_joints": None}}},
     [0.2] * 7, "preset home_joints"),
    ({}, [0.3] * 7, "station home_joints"),
])
def test_home_precedence(station, tmp_path, capsys, document, expected, source):
    preset = tmp_path / "preset.yaml"
    preset.write_text(yaml.safe_dump(document))
    result = call(station, preset, capsys, "show")
    assert result["source"] == source
    assert result["home_joints"] == expected


@pytest.mark.parametrize("gripper", ["open", "close", "keep"])
def test_home_go_only_home_phase(station, tmp_path, capsys, monkeypatch, gripper):
    from real_world_rl.devices import Robot

    built = []
    original = Robot.from_config

    def create(cls, config):
        robot = original(config)
        built.append(robot)
        return robot

    monkeypatch.setattr(Robot, "from_config", classmethod(create))
    preset = tmp_path / "preset.yaml"
    preset.write_text(yaml.safe_dump({"task": {"reset": {
        "home_joints": [0.5] * 7, "gripper": gripper, "lift_m": 0.1,
        "waypoints": [[0.3, 0, 0.2, 0, 0, 0, 1]], "xyz_jitter_m": [0.01] * 3, "settle_s": 100,
    }}}))
    result = call(station, preset, capsys, "go", yes=True)
    assert result["max_delta"] < 1e-6
    robot = built[-1]
    assert robot.arm.call_log == ["move_to_joints"]
    assert ("open" in robot.gripper.call_log) == (gripper == "open")
    assert ("close" in robot.gripper.call_log) == (gripper == "close")


def test_home_set_needs_preset(station, capsys):
    assert "--preset" in call(station, None, capsys, "set", code=2)["error"]


def test_home_set_prints_diff_and_backup(station, tmp_path, capsys):
    preset = tmp_path / "preset.yaml"
    preset.write_text("# keep old text in backup\ntask:\n  reset:\n    home_joints: [0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2]\n")
    assert main(["--station", str(station), "--preset", str(preset), "home", "set"]) == 0
    text = capsys.readouterr().out
    assert " -> [0.0000," in text
    assert f"Previous file kept as {preset}." in text
    assert "# keep old text in backup" in preset.read_text()
