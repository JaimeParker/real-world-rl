"""GELLO calibration helpers on synthetic readings, the printed yaml block, and the CLI flow
over a scripted bus (no hardware)."""
from __future__ import annotations

import numpy as np
import pytest
import yaml

from real_world_rl.devices import TeleopConfig
from real_world_rl.teleop.gello import backend as gello_backend
from real_world_rl.teleop.gello import calibrate
from real_world_rl.teleop.gello.backend import (
    GelloTeleop,
    joint_positions_from_raw,
)

FRANKA_POSE = np.array(calibrate.FRANKA_CALIBRATION_POSE)
QUARTER = np.pi / 2
THREE_JOINTS_AT_ZERO_POSE = (
    "--port", "/dev/ttyUSB0", "--joint-ids", "1", "2", "3", "--calibration-pose", "0", "0", "0", "--skip-sign-check"
)


def raw_for(pose, offsets, signs, turns=None):
    """Raw multi-turn angles that decode to ``pose``: ``raw = offset + sign * pose + 2 pi k``."""
    turns = np.zeros(len(pose)) if turns is None else np.asarray(turns)
    return np.asarray(offsets) + np.asarray(signs) * np.asarray(pose) + 2 * np.pi * turns


def scripted(values):
    queue = list(values)
    return lambda *args: queue.pop(0)


# -- solve_offsets ------------------------------------------------------------------


def test_solve_offsets_recovers_quarter_turn_offsets_with_negative_signs():
    true_offsets = QUARTER * np.array([0, -1, 2, 3, -4, 5, -7])
    signs = np.array([1, -1, 1, -1, 1, 1, -1])
    noise = np.array([0.05, -0.04, 0.0, 0.03, -0.05, 0.02, -0.01])
    raw = raw_for(FRANKA_POSE, true_offsets, signs) + noise
    offsets = calibrate.solve_offsets(raw, FRANKA_POSE, signs)
    np.testing.assert_allclose(offsets, true_offsets, atol=1e-9)
    np.testing.assert_allclose(np.mod(offsets / QUARTER + 0.5, 1.0), 0.5, atol=1e-9)  # on the pi/2 grid
    decoded = joint_positions_from_raw(raw, offsets, signs, np.zeros(7))
    np.testing.assert_allclose(decoded, FRANKA_POSE + signs * noise, atol=1e-9)
    assert np.all(calibrate.offset_residuals(raw, FRANKA_POSE, signs, offsets) <= 0.05 + 1e-9)


def test_solve_offsets_stays_on_the_grid_bounds():
    signs = np.ones(7)
    raw = raw_for(FRANKA_POSE, QUARTER * np.array([16, -16, 0, 0, 0, 0, 0]), signs)
    offsets = calibrate.solve_offsets(raw, FRANKA_POSE, signs)
    assert offsets[0] == pytest.approx(8 * np.pi) and offsets[1] == pytest.approx(-8 * np.pi)
    assert np.all(np.abs(offsets) <= 8 * np.pi + 1e-9)


def test_solve_offsets_rejects_mismatched_lengths():
    with pytest.raises(ValueError, match="one entry per joint"):
        calibrate.solve_offsets([0.0] * 7, [0.0] * 6, [1] * 7)


# -- sign check and gripper capture ---------------------------------------------------


def test_observed_joint_sign():
    before = np.zeros(3)
    assert calibrate.observed_joint_sign(before, [0.0, 0.5, 0.0], 1) == 1
    assert calibrate.observed_joint_sign(before, [0.0, -0.5, 0.0], 1) == -1
    assert calibrate.observed_joint_sign(before, [0.0, 0.05, 0.0], 1) is None


def test_check_joint_signs_accepts_declines_and_keeps_on_no_motion():
    rest = np.zeros(3)
    reads = [
        rest, np.array([-0.4, 0.0, 0.0]),  # joint 1 moved negative: observed -1, accepted
        rest, np.array([0.0, -0.4, 0.0]),  # joint 2 observed -1, declined
        rest, np.array([0.0, 0.0, 0.02]),  # joint 3 barely moved: kept
    ]
    answers = ["", "", "", "n", ""]
    prompts, messages = [], []

    def ask(prompt):
        prompts.append(prompt)
        return answers.pop(0)

    signs = calibrate.check_joint_signs(scripted(reads), [1, 1, 1], ask, messages.append)
    assert signs == [-1, 1, 1]
    assert "joint 1" in prompts[0] and "positive" in prompts[0]
    assert "observed sign -1" in prompts[1]
    assert len(messages) == 1 and "joint 3" in messages[0]
    assert answers == []


def test_capture_gripper_range_in_both_directions():
    asked = []
    open_rad, closed_rad = calibrate.capture_gripper_range(
        scripted([np.array([0.0, 3.22]), np.array([0.0, 2.0])]), 1, asked.append
    )
    assert (open_rad, closed_rad) == (3.22, 2.0)
    assert "OPEN" in asked[0] and "CLOSED" in asked[1]
    assert calibrate.capture_gripper_range(scripted([np.array([1.0]), np.array([1.9])]), 0, lambda p: "") == (1.0, 1.9)


def test_capture_gripper_range_rejects_a_trigger_that_did_not_move():
    with pytest.raises(ValueError, match="moved only"):
        calibrate.capture_gripper_range(scripted([np.array([2.0]), np.array([2.01])]), 0, lambda p: "")


# -- wrap centres ---------------------------------------------------------------------


def test_wrap_centers_from_joint_limits():
    limits = [[-2.8973, 2.8973], [-3.0718, -0.0698], [-0.0175, 3.7525]]
    np.testing.assert_allclose(calibrate.wrap_centers_from_joint_limits(limits), [0.0, -1.5708, 1.8675])


def test_load_joint_limits_top_level_nested_and_unset(tmp_path):
    limits = [[-1.0, 1.0], [0.0, 2.0]]
    top = tmp_path / "top.yaml"
    top.write_text(yaml.safe_dump({"type": "franka", "joint_limits": limits}))
    nested = tmp_path / "nested.yaml"
    nested.write_text(yaml.safe_dump({"robot": {"type": "franka", "joint_limits": limits}}))
    unset = tmp_path / "unset.yaml"
    unset.write_text(yaml.safe_dump({"type": "franka"}))
    assert calibrate.load_joint_limits(top) == limits
    assert calibrate.load_joint_limits(nested) == limits
    assert calibrate.load_joint_limits(unset) is None


# -- yaml block -------------------------------------------------------------------------


def test_format_yaml_block_has_every_option_and_builds_a_teleop():
    offsets = QUARTER * np.array([0, 0, 2, 2, 2, 3, 0])
    text = calibrate.format_yaml_block(
        port="/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT1-if00-port0",
        joint_ids=[1, 2, 3, 4, 5, 6, 7],
        joint_offsets_rad=offsets,
        joint_signs=[1, -1, 1, -1, 1, 1, 1],
        wrap_centers_rad=[0.0, 0.0, 0.0, -1.5708, 0.0, 1.8675, 0.0],
        baudrate=1_000_000,
        gripper_id=8,
        gripper_open_rad=3.2214,
        gripper_closed_rad=2.0,
    )
    assert text.startswith("teleop:\n") and text.endswith("\n")
    block = yaml.safe_load(text)["teleop"]
    assert block["backend"] == "gello" and block["name"] == "gello" and block["hold_s"] == 0.0
    options = block["options"]
    assert set(options) == gello_backend._OPTION_KEYS
    assert options["port"] == "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT1-if00-port0"
    assert options["baudrate"] == 1_000_000 and options["joint_ids"] == [1, 2, 3, 4, 5, 6, 7]
    np.testing.assert_allclose(options["joint_offsets_rad"], offsets, atol=1e-4)
    assert options["joint_signs"] == [1, -1, 1, -1, 1, 1, 1]
    assert options["wrap_centers_rad"][5] == pytest.approx(1.8675)
    assert options["gripper_id"] == 8 and options["gripper_open_rad"] == pytest.approx(3.2214)
    assert options["gripper_closed_rad"] == 2.0 and options["gripper_binarize_threshold"] is None
    assert options["intervention"] == "always" and options["event_device"] is None
    assert options["event_keys"] == gello_backend.DEFAULT_EVENT_KEYS
    assert options["poll_interval_s"] == 0.0
    teleop = GelloTeleop(TeleopConfig.from_dict(block))
    assert teleop.action_dim == 7 and teleop.has_gripper


def test_format_yaml_block_without_gripper_and_with_zero_centres():
    text = calibrate.format_yaml_block(
        port="/dev/ttyUSB0", joint_ids=[1, 2, 3], joint_offsets_rad=[-0.0, np.pi, -np.pi / 2], joint_signs=[1, 1, -1]
    )
    options = yaml.safe_load(text)["teleop"]["options"]
    assert options["gripper_id"] is None and options["gripper_open_rad"] is None
    assert options["wrap_centers_rad"] == [0.0, 0.0, 0.0] and options["joint_offsets_rad"][0] == 0.0
    teleop = GelloTeleop(TeleopConfig.from_dict(yaml.safe_load(text)["teleop"]))
    assert teleop.action_dim == 3 and not teleop.has_gripper


# -- CLI flow ---------------------------------------------------------------------------------


class ScriptedBus:
    """Stands in for ``DynamixelBus``: each read returns the next scripted reading."""

    instances: list["ScriptedBus"] = []
    readings: list[np.ndarray] = []

    def __init__(self, sdk, port, baudrate, servo_ids, *, device):
        self.port, self.baudrate, self.servo_ids = port, baudrate, list(servo_ids)
        self.opened = self.closed = False
        ScriptedBus.instances.append(self)

    def open(self):
        self.opened = True

    def read_positions_rad(self):
        return ScriptedBus.readings.pop(0)

    def close(self):
        self.closed = True


@pytest.fixture
def scripted_bus(monkeypatch):
    ScriptedBus.instances = []
    monkeypatch.setattr(calibrate, "DynamixelBus", ScriptedBus)
    monkeypatch.setattr(GelloTeleop, "require_sdk", classmethod(lambda cls: object()))
    return ScriptedBus


def test_main_prints_a_calibrated_block(scripted_bus, tmp_path):
    true_offsets = QUARTER * np.array([0, 0, 2, 2, 2, 3, 0])
    signs = np.array([1, -1, 1, -1, 1, 1, 1])
    turns = np.array([0, 1, -1, 0, 2, 0, -3])
    pose_raw = raw_for(FRANKA_POSE, true_offsets, signs, turns=turns)
    scripted_bus.readings = [
        np.append(pose_raw, 0.0),  # calibration pose (gripper reading unused here)
        np.append(pose_raw, 3.2),  # trigger open
        np.append(pose_raw, 2.1),  # trigger closed
    ]
    robot_yaml = tmp_path / "robot.yaml"
    limits = [[-2.9, 2.9], [-1.76, 1.76], [-2.9, 2.9], [-3.07, -0.07], [-2.9, 2.9], [-0.02, 3.75], [-2.9, 2.9]]
    robot_yaml.write_text(yaml.safe_dump({"robot": {"type": "franka", "joint_limits": limits}}))
    printed = []
    argv = [
        "--port", "/dev/ttyUSB0", "--joint-ids", "1", "2", "3", "4", "5", "6", "7", "--gripper-id", "8",
        "--joint-signs", "1", "-1", "1", "-1", "1", "1", "1", "--skip-sign-check",
        "--joint-limits-yaml", str(robot_yaml),
    ]
    assert calibrate.main(argv, ask=lambda prompt: "", echo=printed.append) == 0
    bus = scripted_bus.instances[0]
    assert bus.opened and bus.closed and bus.servo_ids == [1, 2, 3, 4, 5, 6, 7, 8] and bus.baudrate == 57600
    assert scripted_bus.readings == []
    options = yaml.safe_load(printed[-1])["teleop"]["options"]
    # Whole turns of the raw reading at calibration land in the offset; decoding wraps them away.
    np.testing.assert_allclose(options["joint_offsets_rad"], true_offsets + 2 * np.pi * turns, atol=1e-4)
    assert options["joint_signs"] == signs.tolist()
    np.testing.assert_allclose(options["wrap_centers_rad"], [(lo + hi) / 2 for lo, hi in limits], atol=1e-4)
    assert (options["gripper_open_rad"], options["gripper_closed_rad"]) == (3.2, 2.1)


def test_main_warns_without_joint_limits_and_closes_the_bus_on_error(scripted_bus):
    scripted_bus.readings = [np.zeros(3)]
    printed = []
    argv = [*THREE_JOINTS_AT_ZERO_POSE]
    assert calibrate.main(argv, ask=lambda prompt: "", echo=printed.append) == 0
    assert any("wrap_centers_rad are zeros" in line for line in printed)
    assert yaml.safe_load(printed[-1])["teleop"]["options"]["wrap_centers_rad"] == [0.0, 0.0, 0.0]

    scripted_bus.readings = [np.zeros(4), np.full(4, 2.0), np.full(4, 2.01)]  # trigger barely moves
    argv = [*THREE_JOINTS_AT_ZERO_POSE, "--gripper-id", "4"]
    with pytest.raises(ValueError):
        calibrate.main(argv, ask=lambda prompt: "", echo=printed.append)
    assert scripted_bus.instances[-1].closed


def test_main_requires_a_pose_for_non_franka_joint_counts(scripted_bus, capsys):
    with pytest.raises(SystemExit):
        calibrate.main(["--port", "/dev/ttyUSB0", "--joint-ids", "1", "2", "3"], ask=lambda prompt: "")
    assert "--calibration-pose is required" in capsys.readouterr().err


# -- calibrate_gello (shared by the CLI and the setup wizard) ----------------------------------


def test_calibrate_gello_returns_the_options_main_prints(scripted_bus, tmp_path):
    true_offsets = QUARTER * np.array([0, 0, 2, 2, 2, 3, 0])
    signs = [1, -1, 1, -1, 1, 1, 1]
    pose_raw = raw_for(FRANKA_POSE, true_offsets, signs)
    readings = [np.append(pose_raw, 0.0), np.append(pose_raw, 3.2), np.append(pose_raw, 2.1)]
    limits = [[-2.9, 2.9]] * 7
    scripted_bus.readings = list(readings)
    options = calibrate.calibrate_gello(
        port="/dev/ttyUSB0", joint_ids=[1, 2, 3, 4, 5, 6, 7], gripper_id=8, joint_signs=signs,
        joint_limits=limits, skip_sign_check=True, ask=lambda prompt: "", echo=lambda line: None,
    )
    assert set(options) == {
        "port", "baudrate", "joint_ids", "joint_offsets_rad", "joint_signs", "wrap_centers_rad",
        "gripper_id", "gripper_open_rad", "gripper_closed_rad",
    }
    assert options["joint_offsets_rad"] == [0.0, 0.0, 3.1416, 3.1416, 3.1416, 4.7124, 0.0]
    assert options["joint_signs"] == signs and options["wrap_centers_rad"] == [0.0] * 7
    assert (options["gripper_open_rad"], options["gripper_closed_rad"]) == (3.2, 2.1)
    assert scripted_bus.instances[-1].servo_ids == [1, 2, 3, 4, 5, 6, 7, 8]

    robot_yaml_limits = {"robot": {"type": "franka", "joint_limits": limits}}
    scripted_bus.readings = list(readings)
    printed = []
    argv = [
        "--port", "/dev/ttyUSB0", "--joint-ids", "1", "2", "3", "4", "5", "6", "7", "--gripper-id", "8",
        "--joint-signs", "1", "-1", "1", "-1", "1", "1", "1", "--skip-sign-check",
    ]
    limits_path = tmp_path / "robot.yaml"
    limits_path.write_text(yaml.safe_dump(robot_yaml_limits))
    assert calibrate.main([*argv, "--joint-limits-yaml", str(limits_path)], ask=lambda prompt: "", echo=printed.append) == 0
    assert printed[-1] == calibrate.format_yaml_block(**options)


def test_calibrate_gello_uses_an_injected_bus_without_the_sdk(monkeypatch):
    monkeypatch.setattr(GelloTeleop, "require_sdk", classmethod(lambda cls: pytest.fail("SDK required")))
    monkeypatch.setattr(ScriptedBus, "instances", [])
    monkeypatch.setattr(ScriptedBus, "readings", [np.zeros(3)])
    bus = ScriptedBus(None, "/dev/ttyUSB0", 57600, [1, 2, 3], device="test")
    options = calibrate.calibrate_gello(
        port="/dev/ttyUSB0", joint_ids=[1, 2, 3], calibration_pose=[0.0, 0.0, 0.0], skip_sign_check=True,
        ask=lambda prompt: "", echo=lambda line: None, bus=bus,
    )
    assert bus.opened and bus.closed
    assert options["joint_offsets_rad"] == [0.0, 0.0, 0.0] and options["gripper_id"] is None
    assert options["gripper_open_rad"] is None and options["gripper_closed_rad"] is None


@pytest.mark.parametrize(
    "kwargs, match",
    [
        ({"joint_ids": [1, 2, 3]}, "calibration_pose is required for 3 joints"),
        ({"joint_ids": [1, 2, 3], "calibration_pose": [0.0, 0.0]}, "calibration_pose needs 3 values"),
        ({"joint_ids": [1, 2, 3], "calibration_pose": [0.0] * 3, "joint_signs": [1]}, "joint_signs needs 3 values"),
    ],
)
def test_calibrate_gello_validates_before_touching_the_bus(kwargs, match):
    with pytest.raises(ValueError, match=match):
        calibrate.calibrate_gello(port="/dev/ttyUSB0", ask=lambda prompt: "", bus=object(), **kwargs)
