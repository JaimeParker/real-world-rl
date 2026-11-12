"""`rwrl teleop calibrate --backend gello`: station defaults, flag overrides and the --use write."""
from __future__ import annotations

import json
import sys

import pytest
import yaml
from robot_infra.devices.backends.teleops.gello_calibrate import FRANKA_CALIBRATION_POSE, format_yaml_block

from real_world_rl.cli import main

LIMITS = [[-2.8, 2.8], [-1.7, 1.7], [-2.8, 2.8], [-3.0, -0.1], [-2.8, 2.8], [0.0, 3.7], [-2.8, 2.8]]
CALIBRATED = {
    "port": "/dev/serial/by-id/leader", "baudrate": 1000000, "joint_ids": [1, 2, 3, 4, 5, 6, 7],
    "joint_offsets_rad": [0.0, 1.5708, 0.0, -1.5708, 0.0, 3.1416, 0.0], "joint_signs": [1, -1, 1, -1, 1, 1, 1],
    "wrap_centers_rad": [0.0, 0.0, 0.0, -1.55, 0.0, 1.85, 0.0], "gripper_id": 8,
    "gripper_open_rad": 3.0, "gripper_closed_rad": 2.2,
}
STATION = """\
# Station notes
robot:
  type: franka
  name:  "left"  # operator label
  arm_backend: fake
  claim: false
  gripper: {backend: fake}
  joint_limits: [[-2.8, 2.8], [-1.7, 1.7], [-2.8, 2.8], [-3.0, -0.1], [-2.8, 2.8], [0.0, 3.7], [-2.8, 2.8]]
  teleop:
    name: gello
    backend: gello
    options:
      port: /dev/serial/by-id/leader  # U2D2
      baudrate: 1000000
      joint_ids: [1, 2, 3, 4, 5, 6, 7]
      joint_signs: [1, -1, 1, -1, 1, 1, 1]
      gripper_id: 8
      intervention: always  # keep

# End of station
"""


@pytest.fixture
def calibrations(monkeypatch):
    """Replaces the guided procedure; records the arguments it was called with."""
    calls = []

    def calibrate_gello(**kwargs):
        assert callable(kwargs.pop("ask")) and callable(kwargs.pop("echo"))
        calls.append(kwargs)
        return dict(CALIBRATED)

    monkeypatch.setattr("robot_infra.devices.backends.teleops.gello_calibrate.calibrate_gello", calibrate_gello)
    monkeypatch.setattr(sys.stdin, "isatty", lambda: True)
    return calls


def write(tmp_path, text=STATION):
    path = tmp_path / "station.yaml"
    path.write_text(text, encoding="utf-8")
    return path


def without_teleop(tmp_path, teleop=None):
    data = yaml.safe_load(STATION)
    del data["robot"]["teleop"]
    if teleop is not None:
        data["robot"]["teleop"] = teleop
    return write(tmp_path, yaml.safe_dump(data))


def test_station_options_are_the_defaults(tmp_path, capsys, calibrations):
    path = write(tmp_path)
    assert main(["--station", str(path), "teleop", "calibrate"]) == 0
    captured = capsys.readouterr()
    assert captured.out == format_yaml_block(**CALIBRATED)
    (call,) = calibrations
    assert [list(row) for row in call.pop("joint_limits")] == LIMITS
    assert call == {"port": "/dev/serial/by-id/leader", "joint_ids": [1, 2, 3, 4, 5, 6, 7], "gripper_id": 8,
                    "baudrate": 1000000, "calibration_pose": list(FRANKA_CALIBRATION_POSE),
                    "joint_signs": [1, -1, 1, -1, 1, 1, 1], "skip_sign_check": False}
    assert path.read_text() == STATION


def test_flags_override_the_station(tmp_path, capsys, calibrations):
    path = write(tmp_path)
    limits = tmp_path / "arm.yaml"
    limits.write_text(yaml.safe_dump({"robot": {"joint_limits": [[-1.0, 1.0], [-2.0, 2.0], [0.0, 3.0]]}}))
    assert main(["--station", str(path), "teleop", "calibrate", "--backend", "gello", "--port", "/dev/other",
                 "--joint-ids", "1", "2", "3", "--gripper-id", "9", "--baudrate", "57600",
                 "--calibration-pose", "0", "0.5", "1", "--joint-signs", "1", "1", "-1",
                 "--joint-limits-yaml", str(limits), "--skip-sign-check"]) == 0
    capsys.readouterr()
    assert calibrations == [{"port": "/dev/other", "joint_ids": [1, 2, 3], "gripper_id": 9, "baudrate": 57600,
                             "calibration_pose": [0.0, 0.5, 1.0], "joint_signs": [1, 1, -1],
                             "joint_limits": [[-1.0, 1.0], [-2.0, 2.0], [0.0, 3.0]], "skip_sign_check": True}]


def test_new_joint_set_needs_matching_limits_and_restarts_signs(tmp_path, capsys, calibrations):
    path = without_teleop(tmp_path)
    assert main(["--station", str(path), "teleop", "calibrate", "--backend", "gello", "--port", "/dev/x",
                 "--joint-ids", "1", "2", "3", "--calibration-pose", "0", "0", "0"]) == 2
    assert "joint_limits rows" in capsys.readouterr().err
    path = write(tmp_path)
    limits = tmp_path / "arm.yaml"
    limits.write_text(yaml.safe_dump({"joint_limits": [[-1.0, 1.0]] * 3}))
    assert main(["--station", str(path), "teleop", "calibrate", "--joint-ids", "1", "2", "3",
                 "--calibration-pose", "0", "0", "0", "--joint-limits-yaml", str(limits)]) == 0
    capsys.readouterr()
    assert calibrations[-1]["joint_signs"] == [1, 1, 1]


def test_json_result(tmp_path, capsys, calibrations):
    path = write(tmp_path)
    assert main(["--station", str(path), "--json", "teleop", "calibrate"]) == 0
    payload = json.loads(capsys.readouterr().out)
    assert payload == {"backend": "gello", "options": CALIBRATED, "yaml_block": format_yaml_block(**CALIBRATED)}


def test_use_writes_station_teleop_options_and_keeps_comments(tmp_path, capsys, calibrations):
    from test_yaml_edit import assert_only_key_changed

    path = write(tmp_path)
    assert main(["--station", str(path), "--json", "--yes", "teleop", "calibrate", "--use"]) == 0
    payload = json.loads(capsys.readouterr().out)
    assert payload["station_saved"]["backup"]
    after = path.read_text()
    assert_only_key_changed(STATION, after, ("robot", "teleop"))
    teleop = yaml.safe_load(after)["robot"]["teleop"]
    assert teleop["backend"] == "gello"
    assert teleop["options"] == {**yaml.safe_load(STATION)["robot"]["teleop"]["options"], **CALIBRATED}
    assert "port: /dev/serial/by-id/leader  # U2D2" in after
    assert "intervention: always  # keep" in after
    assert '  name:  "left"  # operator label\n' in after
    assert after.endswith("\n# End of station\n")


def test_use_without_gripper_removes_station_gripper_options(tmp_path, capsys, calibrations, monkeypatch):
    gripperless = {**CALIBRATED, "gripper_id": None, "gripper_open_rad": None, "gripper_closed_rad": None}
    monkeypatch.setattr("robot_infra.devices.backends.teleops.gello_calibrate.calibrate_gello",
                        lambda **kwargs: dict(gripperless))
    station = STATION.replace("      gripper_id: 8\n", "      gripper_id: 8\n      gripper_open_rad: 3.0\n"
                              "      gripper_closed_rad: 2.2\n      gripper_binarize_threshold: 0.5\n")
    path = write(tmp_path, station)
    assert main(["--station", str(path), "--json", "--yes", "teleop", "calibrate", "--use"]) == 0
    capsys.readouterr()
    after = path.read_text()
    options = yaml.safe_load(after)["robot"]["teleop"]["options"]
    gripper_keys = {"gripper_id", "gripper_open_rad", "gripper_closed_rad", "gripper_binarize_threshold"}
    assert not gripper_keys & set(options)
    expected = {**yaml.safe_load(station)["robot"]["teleop"]["options"], **gripperless}
    assert options == {key: value for key, value in expected.items() if key not in gripper_keys}
    assert "intervention: always  # keep" in after
    assert after.endswith("\n# End of station\n")


def test_use_adds_a_gello_teleop_to_a_station_without_one(tmp_path, capsys, calibrations):
    path = without_teleop(tmp_path)
    assert main(["--station", str(path), "--json", "--yes", "teleop", "calibrate", "--backend", "gello",
                 "--port", "/dev/serial/by-id/leader", "--joint-ids", "1", "2", "3", "4", "5", "6", "7",
                 "--use"]) == 0
    capsys.readouterr()
    teleop = yaml.safe_load(path.read_text())["robot"]["teleop"]
    assert teleop == {"name": "gello", "backend": "gello", "options": CALIBRATED}


def test_use_refuses_to_replace_another_backend(tmp_path, capsys, calibrations):
    path = without_teleop(tmp_path, {"backend": "spacemouse"})
    before = path.read_text()
    assert main(["--station", str(path), "--json", "--yes", "teleop", "calibrate", "--backend", "gello",
                 "--port", "/dev/x", "--joint-ids", "1", "2", "3", "4", "5", "6", "7", "--use"]) == 2
    assert "spacemouse" in json.loads(capsys.readouterr().out)["error"]
    assert calibrations == [] and path.read_text() == before


@pytest.mark.parametrize("argv,flag", [
    (["--profile", "rear"], "--profile"),
    (["--backend", "gello", "--save"], "--save"),
    (["--output", "profiles.yaml", "--frame", "table"], "--frame, --output"),
])
def test_spacemouse_flags_are_refused(tmp_path, capsys, calibrations, argv, flag):
    path = write(tmp_path)
    assert main(["--station", str(path), "--json", "teleop", "calibrate", *argv]) == 2
    assert json.loads(capsys.readouterr().out)["error"] == f"{flag} not applicable to --backend gello"
    assert calibrations == []


def test_missing_port_and_joint_ids_are_structured(tmp_path, capsys, calibrations):
    path = without_teleop(tmp_path)
    assert main(["--station", str(path), "--json", "teleop", "calibrate", "--backend", "gello"]) == 2
    assert json.loads(capsys.readouterr().out)["required_fields"] == ["--port", "--joint-ids"]
    assert calibrations == []


def test_station_without_teleop_needs_a_backend(tmp_path, capsys, calibrations):
    path = without_teleop(tmp_path)
    assert main(["--station", str(path), "--json", "teleop", "calibrate"]) == 2
    assert json.loads(capsys.readouterr().out)["required_fields"] == ["--backend"]


def test_calibration_needs_a_terminal(tmp_path, capsys, calibrations, monkeypatch):
    monkeypatch.setattr(sys.stdin, "isatty", lambda: False)
    path = write(tmp_path)
    assert main(["--station", str(path), "--json", "teleop", "calibrate"]) == 2
    assert "terminal" in json.loads(capsys.readouterr().out)["error"]
    assert calibrations == []


def test_help_groups_flags_by_backend(capsys):
    assert main(["teleop", "calibrate", "--help"]) == 0
    text = capsys.readouterr().out
    spacemouse, gello = text.index("\nspacemouse:\n"), text.index("\ngello:\n")
    assert spacemouse < gello
    shared, spacemouse_flags, gello_flags = text[:spacemouse], text[spacemouse:gello], text[gello:]
    assert "\n  --backend {spacemouse,gello}" in shared and "\n  --use " in shared
    assert "\n  --profile PROFILE" in spacemouse_flags and "--profile" not in gello_flags
    assert "\n  --port PORT" in gello_flags and "--port" not in spacemouse_flags


def test_rwrl_gello_calibrate_is_an_alias(capsys):
    from robot_infra.cli import COMMANDS as DEVICE_COMMANDS

    assert main(["gello-calibrate", "--help"]) == 0
    text = " ".join(capsys.readouterr().out.split())
    assert "alias of teleop calibrate --backend gello (will be removed)" in text
    assert "--joint-ids" in text
    device = next(command for command in DEVICE_COMMANDS if command.name == "gello-calibrate")
    assert device.help == "Calibrate a GELLO leader arm"
