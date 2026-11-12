"""Setup wizard end to end under ``--mock`` with scripted answers: one and two arms, flags and
``--non-interactive``, ``--station`` round trips, ``--only``, backups, rendering and the doctor."""
from __future__ import annotations

from datetime import datetime, timezone
from pathlib import Path

import numpy as np
import pytest
import yaml

import real_world_rl.devices.claims as claims
from real_world_rl.devices import ArmSafety, CameraConfig, DeviceNotFound
from real_world_rl.devices.backends.teleops import gello as gello_backend
from real_world_rl.devices.robots.franka import FrankaConfig
from real_world_rl.discovery.autoconfig import STATION_CONFIG_ENV, load_station
from real_world_rl.doctor.cli import doctor_station, station_exit_code
from real_world_rl.setup import probes
from real_world_rl.setup.command import main, next_steps_text
from real_world_rl.setup.render import (
    check_rendered_station,
    render_station_yaml,
    write_station_yaml,
    yaml_value,
)
from real_world_rl.setup.wizard import (
    GELLO_OPTION_ORDER,
    NEW_GELLO_GRIPPER_BINARIZE_THRESHOLD,
    PANDA_JOINT_LIMITS,
    SUGGESTED_SAFETY_BOX,
    Answers,
    ArmDraft,
    StationDraft,
    WizardError,
    run_wizard,
)

DOCS = Path(__file__).resolve().parents[2] / "examples" / "stations"
SERIALS = probes.MOCK_CAMERA_SERIALS
PANDA_WRAP_CENTERS = [0.0, 0.0, 0.0, -1.5708, 0.0, 1.8675, 0.0]
DEFAULT_JOINT_LIMITS = [list(pair) for pair in PANDA_JOINT_LIMITS]
DEFAULT_MAX_DELTA = {"xyz_m": 0.05, "joint_rad": 0.04}
# Contains the home pose of FLAGGED_ONE_ARM.
NARROW_JOINT_LIMITS = "-2,2,-1.5,1.5,-2,2,-3,-0.1,-2,2,0,3.5,-2,2"
FLAGGED_ONE_ARM = [
    "--mock", "--non-interactive", "--arms", "1", "--arm-names", "arm", "--robot-ip", "172.16.0.2",
    "--arm-backend", "franky", "--gripper", "none", "--home-joints", "0,-0.785,0,-2.356,0,1.571,0.785",
    "--cameras", "1=wrist", "--teleop", "gello", "--gello-port", "1", "--gello-baudrate", "57600",
    "--gello-joint-ids", "1,2,3,4,5,6,7", "--gello-gripper-id", "8", "--intervention", "always",
    "--event-device", "none", "--daemon-port", "50060",
]


class Operator:
    """Scripted ``ask``: the first unused (key, answer) whose key is in the prompt answers it;
    any other prompt gets "" (take the default)."""

    def __init__(self, answers=()) -> None:
        self.answers = list(dict(answers).items()) if isinstance(answers, dict) else list(answers)
        self.prompts: list[str] = []

    def __call__(self, prompt: str) -> str:
        self.prompts.append(prompt)
        if len(self.prompts) > 500:
            raise AssertionError(f"the wizard keeps asking: {self.prompts[-3:]}")
        for index, (key, answer) in enumerate(self.answers):
            if key in prompt:
                del self.answers[index]
                return answer
        return ""


def never_asked(prompt: str) -> str:
    raise AssertionError(f"--non-interactive prompted: {prompt}")


@pytest.fixture(autouse=True)
def isolated(tmp_path, monkeypatch):
    monkeypatch.setattr(claims, "_DEFAULT_LOCK_DIR", tmp_path / "claims")
    monkeypatch.delenv(STATION_CONFIG_ENV, raising=False)
    monkeypatch.setenv("XDG_CONFIG_HOME", str(tmp_path / "xdg"))
    monkeypatch.chdir(tmp_path)


def run(argv, ask=None) -> tuple[int, str]:
    lines: list[str] = []
    code = main([str(arg) for arg in argv], ask=ask or Operator(), echo=lines.append)
    return code, "\n".join(lines)


# -- interactive ----------------------------------------------------------------------------------


def test_one_arm_with_defaults_calibrates_writes_and_passes_the_doctor(tmp_path):
    out = tmp_path / "station.yaml"
    operator = Operator()
    code, output = run(["--mock", "--output", out], operator)
    assert code == 0, output
    text = out.read_text()
    assert "robots:" not in text and "daemon:" not in text and "TODO calibrate" not in text
    robot = load_station(out).select()
    config = robot.config
    assert config.name == "arm" and config.robot_ip == "172.16.0.2" and config.arm_backend == "franky"
    assert [(camera.name, camera.serial) for camera in config.cameras] == list(zip(["wrist", "front", "front_2"], SERIALS))
    options = config.teleop.options
    assert config.teleop.backend == "gello" and options["port"] == probes.MOCK_GELLO_PORTS[0]
    assert options["joint_ids"] == [1, 2, 3, 4, 5, 6, 7] and options["gripper_id"] == 8
    np.testing.assert_allclose(options["joint_offsets_rad"], probes.SimulatedGelloBus.HORN_OFFSETS_RAD, atol=1e-4)
    assert options["wrap_centers_rad"] == PANDA_WRAP_CENTERS
    assert list(options) == list(GELLO_OPTION_ORDER)
    assert any("calibrate the GELLO" in prompt for prompt in operator.prompts)
    assert any("calibration pose" in prompt for prompt in operator.prompts)
    assert "summary:" in output and "0 failed" in output
    assert f"serve the robot: rwrl daemon --station {out}" in output
    assert "--teleop-device gello --teleop-options-yaml" in output and "--obs.rgb wrist front front_2" in output
    assert "real_robot_bimanual" not in output
    assert any("safety box of 'arm'" in prompt for prompt in operator.prompts)
    assert "safety box: the TCP position is kept inside it" in output
    assert config.safety_box == list(SUGGESTED_SAFETY_BOX)
    assert config.joint_limits == DEFAULT_JOINT_LIMITS and config.max_delta_per_step == DEFAULT_MAX_DELTA


def test_two_arms_get_their_own_ip_cameras_gello_and_port(tmp_path):
    out = tmp_path / "station.yaml"
    code, output = run(["--mock", "--output", out], Operator({"number of arms": "2"}))
    assert code == 0, output
    station = load_station(out)
    assert station.multi_robot and list(station.robots) == ["left", "right"]
    left, right = station.robots["left"], station.robots["right"]
    assert (left.config.robot_ip, right.config.robot_ip) == probes.MOCK_FRANKA_IPS
    assert (left.bind, right.bind) == ("0.0.0.0:50051", "0.0.0.0:50052")
    assert [camera.name for camera in left.config.cameras] == ["wrist_left"]
    assert [camera.name for camera in right.config.cameras] == ["wrist_right", "front"]
    ports = [robot.config.teleop.options["port"] for robot in (left, right)]
    assert ports == list(probes.MOCK_GELLO_PORTS)
    assert "== robot 'left' ==" in output and "overall: 2 robots, all passed" in output
    assert "--obs.rgb wrist_left wrist_right front" in output
    assert "--env-backend real_robot_bimanual" in output
    assert (
        "--real_robot_bimanual.left.remote <robot-pc>:50051 --real_robot_bimanual.right.remote <robot-pc>:50052"
        in output
    )
    assert "--teleop-device bimanual --teleop-arm-device leader_arm" in output
    assert "coming in a later change" not in output and "--teleop-options-yaml" not in output
    for robot in (left, right):
        assert robot.config.safety_box == list(SUGGESTED_SAFETY_BOX)
        assert robot.config.joint_limits == DEFAULT_JOINT_LIMITS
        assert robot.config.max_delta_per_step == DEFAULT_MAX_DELTA


def test_the_default_local_scope_is_found_without_config(tmp_path):
    code, output = run(["--mock"], Operator({"teleop device": "none"}))
    assert code == 0, output
    assert (tmp_path / "local" / "station.yaml").is_file()
    assert "serve the robot: rwrl daemon  (listens on 0.0.0.0:50051)" in output
    assert "--teleop-device" not in output


def test_an_invalid_answer_is_asked_again(tmp_path):
    echoed: list[str] = []
    operator = Operator([("robot IP", "10.1.2.3"), ("robot IP", "7"), ("teleop device", "none")])
    draft = run_wizard(probes.mock_station_probe(), Answers(arms=1, mock=True), operator, echoed.append)
    assert draft.arms[0].config.robot_ip == "172.16.0.2"
    assert sum("robot IP" in prompt for prompt in operator.prompts) == 3
    assert any("did not answer the probe" in line for line in echoed)
    assert any("pick a number in 1..2" in line for line in echoed)


def test_a_failed_calibration_can_be_skipped_and_leaves_a_todo():
    def broken_bus(port, baudrate, joint_ids, gripper_id):
        class Bus:
            def open(self):
                raise DeviceNotFound(f"cannot open Dynamixel port {port}", hint="check the cable")

            def close(self):
                pass

        return Bus()

    echoed: list[str] = []
    operator = Operator({"retry the calibration": "n"})
    draft = run_wizard(
        probes.mock_station_probe(), Answers(arms=1, mock=True), operator, echoed.append,
        calibration_bus_factory=broken_bus,
    )
    arm = draft.arms[0]
    assert not arm.gello_calibrated and any("calibration failed" in line for line in echoed)
    assert arm.config.teleop.options["joint_offsets_rad"] == [0.0] * 7
    assert "TODO calibrate" in render_station_yaml(draft)


# -- non-interactive and flags --------------------------------------------------------------------


def test_non_interactive_without_flags_fails_and_writes_nothing(tmp_path, capsys):
    out = tmp_path / "station.yaml"
    code, output = run(["--mock", "--non-interactive", "--output", out], never_asked)
    error = capsys.readouterr().err
    assert "error:" not in output
    assert code == 2 and "--non-interactive needs --arms" in error and not out.exists()


def test_non_interactive_flags_write_an_uncalibrated_gello_with_a_todo(tmp_path):
    out = tmp_path / "station.yaml"
    code, output = run([*FLAGGED_ONE_ARM, "--output", out], never_asked)
    assert code == 0, output
    text = out.read_text()
    assert "# TODO calibrate" in text and "daemon:" in text
    robot = load_station(out).select()
    assert robot.bind == "0.0.0.0:50060" and robot.config.gripper is None
    options = robot.config.teleop.options
    assert options["joint_offsets_rad"] == [0.0] * 7 and options["joint_signs"] == [1] * 7
    assert (options["gripper_open_rad"], options["gripper_closed_rad"]) == (0.0, 1.0)
    assert options["wrap_centers_rad"] == PANDA_WRAP_CENTERS
    assert [camera.serial for camera in robot.config.cameras] == [SERIALS[0]]
    gello_backend.GelloTeleop(robot.config.teleop)  # the placeholders still build a teleop


def test_a_new_gello_gripper_channel_is_binarized_and_none_without_a_gripper(tmp_path):
    out = tmp_path / "station.yaml"
    assert run([*FLAGGED_ONE_ARM, "--output", out], never_asked)[0] == 0
    teleop = load_station(out).select().config.teleop
    assert teleop.options["gripper_binarize_threshold"] == NEW_GELLO_GRIPPER_BINARIZE_THRESHOLD == 0.5
    assert gello_backend.GelloTeleop(teleop)._gripper_binarize_threshold == 0.5  # the backend honours it

    without = tmp_path / "no_gripper.yaml"
    argv = [arg if arg != "8" else "none" for arg in FLAGGED_ONE_ARM]
    assert run([*argv, "--output", without], never_asked)[0] == 0
    assert load_station(without).select().config.teleop.options["gripper_binarize_threshold"] is None


def test_unprobed_values_need_allow_unprobed(tmp_path, capsys):
    argv = [*FLAGGED_ONE_ARM, "--robot-ip", "10.1.2.3", "--output", tmp_path / "station.yaml"]
    code, output = run(argv, never_asked)
    error = capsys.readouterr().err
    assert "error:" not in output
    assert code == 2 and "did not answer the probe" in error
    code, output = run([*argv, "--allow-unprobed"], never_asked)
    assert code == 0, output
    assert load_station(tmp_path / "station.yaml").select().config.robot_ip == "10.1.2.3"


def test_non_interactive_calibration_is_rejected():
    with pytest.raises(WizardError, match="calibration needs an operator"):
        run_wizard(probes.mock_station_probe(), Answers(non_interactive=True, calibrate=True), never_asked, print)


def test_only_needs_from():
    with pytest.raises(WizardError, match="pass --station"):
        run_wizard(probes.mock_station_probe(), Answers(only="cameras"), never_asked, print)


def test_per_arm_flags_need_one_value_per_arm_or_one_for_all(tmp_path, capsys):
    argv = [*FLAGGED_ONE_ARM, "--arms", "2", "--arm-names", "left", "right", "--output", tmp_path / "s.yaml"]
    code, output = run([*argv, "--robot-ip", "172.16.0.2", "172.16.1.2", "10.0.0.1"], never_asked)
    error = capsys.readouterr().err
    assert "error:" not in output
    assert code == 2 and "--robot-ip needs one value per arm (2)" in error


# -- safety limits -----------------------------------------------------------------------------------


def test_non_interactive_writes_the_default_safety_limits(tmp_path):
    out = tmp_path / "station.yaml"
    code, output = run([*FLAGGED_ONE_ARM, "--output", out], never_asked)
    assert code == 0, output
    text = out.read_text()
    for key in ("safety_box", "joint_limits", "max_delta_per_step"):
        assert f"{key}: null" not in text
    config = load_station(out).select().config
    assert config.safety_box == list(SUGGESTED_SAFETY_BOX)
    assert config.joint_limits == DEFAULT_JOINT_LIMITS
    assert config.max_delta_per_step == DEFAULT_MAX_DELTA
    safety = ArmSafety.from_config(config)
    assert safety is not None and (safety.max_delta_xyz_m, safety.max_delta_joint_rad) == (0.05, 0.04)


def test_safety_flags_are_written(tmp_path):
    out = tmp_path / "station.yaml"
    argv = [
        *FLAGGED_ONE_ARM, f"--joint-limits={NARROW_JOINT_LIMITS}", "--safety-box", "[0.2, -0.4, 0.1, 0.6, 0.4, 0.5]",
        "--max-delta-xyz-m", "0.02", "--max-delta-joint-rad", "0.01", "--output", out,
    ]
    code, output = run(argv, never_asked)
    assert code == 0, output
    config = load_station(out).select().config
    assert config.joint_limits == [[-2, 2], [-1.5, 1.5], [-2, 2], [-3, -0.1], [-2, 2], [0, 3.5], [-2, 2]]
    assert config.safety_box == [0.2, -0.4, 0.1, 0.6, 0.4, 0.5]
    assert config.max_delta_per_step == {"xyz_m": 0.02, "joint_rad": 0.01}


def test_per_arm_safety_boxes(tmp_path):
    out = tmp_path / "station.yaml"
    argv = [
        "--mock", "--output", out, "--safety-box", "0.3,-0.3,0.05,0.7,0.3,0.6", "0.35,-0.25,0.1,0.65,0.25,0.5",
    ]
    code, output = run(argv, Operator({"number of arms": "2"}))
    assert code == 0, output
    station = load_station(out)
    assert station.robots["left"].config.safety_box == [0.3, -0.3, 0.05, 0.7, 0.3, 0.6]
    assert station.robots["right"].config.safety_box == [0.35, -0.25, 0.1, 0.65, 0.25, 0.5]


def test_from_keeps_hand_edited_limits_and_fills_missing_ones(tmp_path):
    out = tmp_path / "station.yaml"
    assert run([*FLAGGED_ONE_ARM, "--output", out], never_asked)[0] == 0
    text = out.read_text()
    edits = {
        "safety_box: [0.3, -0.3, 0.05, 0.7, 0.3, 0.6]": "safety_box: [0.2, -0.4, 0.1, 0.6, 0.4, 0.5]",
        "[-3.0718, -0.0698]": "[-3.0, -0.1]",
        "max_delta_per_step: {xyz_m: 0.05, joint_rad: 0.04}": "max_delta_per_step: {xyz_m: 0.02}",
    }
    for old, new in edits.items():
        assert old in text
        text = text.replace(old, new)
    out.write_text(text)
    again = tmp_path / "again.yaml"
    code, output = run(["--mock", "--non-interactive", "--station", out, "--output", again], never_asked)
    assert code == 0, output
    config = load_station(again).select().config
    assert config.safety_box == [0.2, -0.4, 0.1, 0.6, 0.4, 0.5]
    assert config.joint_limits[3] == [-3.0, -0.1] and config.joint_limits[0] == [-2.8973, 2.8973]
    assert config.max_delta_per_step == {"xyz_m": 0.02, "joint_rad": 0.04}

    for key in ("safety_box", "joint_limits", "max_delta_per_step"):
        text = "\n".join(f"{key}: null" if line.startswith(f"{key}:") else line for line in text.splitlines())
    out.write_text(text + "\n")
    assert load_station(out).select().config.safety_box is None
    code, output = run(["--mock", "--non-interactive", "--station", out, "--output", again], never_asked)
    assert code == 0, output
    config = load_station(again).select().config
    assert config.safety_box == list(SUGGESTED_SAFETY_BOX)
    assert config.joint_limits == DEFAULT_JOINT_LIMITS and config.max_delta_per_step == DEFAULT_MAX_DELTA


@pytest.mark.parametrize(
    "box, message",
    [
        ("0.7,-0.3,0.05,0.3,0.3,0.6", "xmin 0.7 must be below xmax 0.3"),
        ("0.3,-0.3,0.6,0.7,0.3,0.6", "zmin 0.6 must be below zmax 0.6"),
        ("0.3,-0.3,0.05,0.7,0.3", "give 6 numbers in m (xmin ymin zmin xmax ymax zmax), got 5"),
        ("0.3,-0.3,0.05,0.7,0.3,high", "give 6 numbers in m"),
    ],
)
def test_an_invalid_safety_box_flag_is_rejected(tmp_path, box, message, capsys):
    out = tmp_path / "station.yaml"
    code, output = run([*FLAGGED_ONE_ARM, "--safety-box", box, "--output", out], never_asked)
    error = capsys.readouterr().err
    assert "error:" not in output
    assert code == 2 and "--safety-box" in error and message in error and not out.exists()


def test_an_invalid_safety_box_answer_is_asked_again():
    echoed: list[str] = []
    operator = Operator([("safety box", "0.3, 0.3"), ("safety box", "0.7,-0.3,0.05,0.3,0.3,0.6"), ("teleop device", "none")])
    draft = run_wizard(probes.mock_station_probe(), Answers(arms=1, mock=True), operator, echoed.append)
    assert draft.arms[0].config.safety_box == list(SUGGESTED_SAFETY_BOX)
    assert sum("safety box of 'arm'" in prompt for prompt in operator.prompts) == 3
    assert any("got 2" in line for line in echoed) and any("xmin 0.7 must be below xmax 0.3" in line for line in echoed)


@pytest.mark.parametrize(
    "flags, message",
    [
        (["--max-delta-xyz-m", "0"], "--max-delta-xyz-m '0': give a positive number of m"),
        (["--max-delta-joint-rad", "fast"], "--max-delta-joint-rad 'fast': 'fast' is not a number"),
        (["--joint-limits", "1,2,3"], "give 14 numbers in rad"),
        (["--joint-limits=2,-2,-1.5,1.5,-2,2,-3,-0.1,-2,2,0,3.5,-2,2"], "joint 1: lo 2.0 must be below hi -2.0"),
    ],
)
def test_invalid_limit_flags_are_rejected(tmp_path, flags, message, capsys):
    code, output = run([*FLAGGED_ONE_ARM, *flags, "--output", tmp_path / "s.yaml"], never_asked)
    error = capsys.readouterr().err
    assert "error:" not in output
    assert code == 2 and message in error


def test_home_joints_outside_the_joint_limits_give_a_readable_error(tmp_path, capsys):
    limits = "-2,2,-1.5,1.5,-2,2,-2,-0.1,-2,2,0,3.5,-2,2"
    code, output = run([*FLAGGED_ONE_ARM, f"--joint-limits={limits}", "--output", tmp_path / "s.yaml"], never_asked)
    assert code == 2
    assert "the home pose must lie inside the joint limits: joint 4 = -2.356 is outside [-2.0, -0.1]" in capsys.readouterr().err


def test_bimanual_next_steps_map_renamed_arms_and_skip_teleop_without_one():
    arms = [
        ArmDraft("a", FrankaConfig(name="a", teleop=None), bind="10.0.0.5:50051"),
        ArmDraft("b", FrankaConfig(name="b", teleop=None), bind="0.0.0.0:50052"),
    ]
    text = next_steps_text(StationDraft(arms), Path("station.yaml"), found_without_config=True)
    assert "--env-backend real_robot_bimanual" in text
    assert "--real_robot_bimanual.left.remote 10.0.0.5:50051 --real_robot_bimanual.right.remote <robot-pc>:50052" in text
    assert "# left = 'a', right = 'b'" in text
    assert "--teleop-device" not in text


# -- regenerate ------------------------------------------------------------------------------------


@pytest.mark.parametrize("arms", ["1", "2"])
def test_from_round_trip_is_byte_identical_and_backs_up(tmp_path, arms):
    out = tmp_path / "station.yaml"
    code, output = run(["--mock", "--output", out], Operator({"number of arms": arms}))
    assert code == 0, output
    original = out.read_text()
    code, output = run(["--mock", "--non-interactive", "--station", out], never_asked)
    assert code == 0, output
    assert out.read_text() == original
    backups = list(tmp_path.glob("station.yaml.bak.*"))
    assert len(backups) == 1 and backups[0].read_text() == original
    copy = tmp_path / "copy.yaml"
    assert run(["--mock", "--non-interactive", "--station", out, "--output", copy], never_asked)[0] == 0
    assert copy.read_text() == original


def test_from_round_trip_keeps_an_uncalibrated_gello(tmp_path):
    out = tmp_path / "station.yaml"
    assert run([*FLAGGED_ONE_ARM, "--output", out], never_asked)[0] == 0
    original = out.read_text()
    assert run(["--mock", "--non-interactive", "--station", out, "--output", tmp_path / "again.yaml"], never_asked)[0] == 0
    assert (tmp_path / "again.yaml").read_text() == original


@pytest.mark.parametrize("only", [[], ["--only", "teleop"]], ids=["all", "only-teleop"])
@pytest.mark.parametrize("key_written", [True, False], ids=["null", "absent"])
def test_from_keeps_an_existing_gellos_continuous_gripper(tmp_path, only, key_written):
    # A station written before the binarize default, with the key at null or without it: it
    # keeps the backend default (continuous width) on regeneration instead of turning into 0.5.
    out = tmp_path / "station.yaml"
    assert run([*FLAGGED_ONE_ARM, "--output", out], never_asked)[0] == 0
    edited = []
    for line in out.read_text().splitlines(keepends=True):
        if "gripper_binarize_threshold:" in line:
            if key_written:
                edited.append(line.replace(": 0.5", ": null", 1))
            continue
        edited.append(line)
    out.write_text("".join(edited))
    assert load_station(out).select().config.teleop.options.get("gripper_binarize_threshold") is None

    again = tmp_path / "again.yaml"
    code, output = run(["--mock", "--non-interactive", "--station", out, *only, "--output", again], never_asked)
    assert code == 0, output
    assert load_station(again).select().config.teleop.options["gripper_binarize_threshold"] is None


def test_only_cameras_reruns_one_section_and_keeps_the_rest(tmp_path):
    out = tmp_path / "station.yaml"
    assert run(["--mock", "--output", out])[0] == 0
    before = load_station(out).select()
    again = tmp_path / "again.yaml"
    argv = ["--mock", "--non-interactive", "--station", out, "--only", "cameras", "--cameras", "2=side", "--output", again]
    code, output = run(argv, never_asked)
    assert code == 0, output
    after = load_station(again).select()
    assert [(camera.name, camera.serial) for camera in after.config.cameras] == [("side", SERIALS[1])]
    assert after.config.teleop == before.config.teleop and after.config.robot_ip == before.config.robot_ip
    assert after.bind == before.bind


def test_only_teleop_can_recalibrate_interactively(tmp_path):
    out = tmp_path / "station.yaml"
    assert run([*FLAGGED_ONE_ARM, "--output", out], never_asked)[0] == 0
    again = tmp_path / "again.yaml"
    code, output = run(["--mock", "--station", out, "--only", "teleop", "--calibrate", "--output", again])
    assert code == 0, output
    assert "TODO calibrate" not in again.read_text()
    options = load_station(again).select().config.teleop.options
    np.testing.assert_allclose(options["joint_offsets_rad"], probes.SimulatedGelloBus.HORN_OFFSETS_RAD, atol=1e-4)


def test_from_rejects_a_non_franka_station(tmp_path, capsys):
    piper = tmp_path / "piper.yaml"
    piper.write_text(yaml.safe_dump({"type": "piper", "arm_backend": "fake"}))
    code, output = run(["--mock", "--non-interactive", "--station", piper, "--output", tmp_path / "s.yaml"], never_asked)
    error = capsys.readouterr().err
    assert "error:" not in output
    assert code == 2 and "edit this one by hand" in error


# -- render and write --------------------------------------------------------------------------------


@pytest.mark.parametrize(
    "value",
    ["0.0.0.0:50051", "123422070001", "", "null", "yes", "a b", "/dev/serial/by-id/usb-FTDI_x-if00-port0",
     "KEY_SPACE", "@home", 1e-05, 1e20, 0.2, -0.785, 0, True, None, [1, 2.5, "x"], {"k": None, "t": "KEY_A"}],
)
def test_yaml_value_reads_back(value):
    assert yaml.safe_load(f"v: {yaml_value(value)}")["v"] == value
    assert yaml.safe_load(f"v: [{yaml_value(value, flow=True)}]")["v"] == [value]


def test_gello_option_order_matches_the_backend():
    assert set(GELLO_OPTION_ORDER) == gello_backend._OPTION_KEYS


def test_bimanual_example_renders_back_to_the_same_station():
    draft = StationDraft.from_station(load_station(DOCS / "station.bimanual.example.yaml"))
    assert [arm.gello_calibrated for arm in draft.arms] == [True, True]
    station = check_rendered_station(render_station_yaml(draft), draft)
    assert list(station.robots) == ["left", "right"]


def test_rendered_station_passes_the_doctor_with_mock(tmp_path):
    draft = StationDraft.from_station(load_station(DOCS / "station.bimanual.example.yaml"))
    path = write_station_yaml(render_station_yaml(draft), path_override=tmp_path / "station.yaml")
    assert station_exit_code(doctor_station(path, mock=True)) == 0


def test_write_station_yaml_locations_and_backups(tmp_path):
    now = datetime(2026, 11, 3, 12, 0, 0, tzinfo=timezone.utc)
    local = write_station_yaml("a: 1\n", "local", cwd=tmp_path, now=now)
    assert local == tmp_path / "local" / "station.yaml" and local.read_text() == "a: 1\n"
    echoed: list[str] = []
    write_station_yaml("a: 2\n", "local", cwd=tmp_path, now=now, echo=echoed.append)
    write_station_yaml("a: 3\n", "local", cwd=tmp_path, now=now)
    assert local.read_text() == "a: 3\n"
    assert (tmp_path / "local" / "station.yaml.bak.20261103T120000Z").read_text() == "a: 1\n"
    assert (tmp_path / "local" / "station.yaml.bak.20261103T120000Z.1").read_text() == "a: 2\n"
    assert "backed up" in echoed[0]
    user = write_station_yaml("b: 1\n", "user", env={"XDG_CONFIG_HOME": str(tmp_path / "xdg")})
    assert user == tmp_path / "xdg" / "rwrl" / "station.yaml"
    assert not list((tmp_path / "local").glob(".station.yaml.*.tmp"))


def test_render_omits_upper_layer_profiles_and_keeps_safety_comment_adjacent():
    from real_world_rl.setup.render import TEMPLATE_PATH

    draft = StationDraft.from_station(load_station(DOCS / "station.bimanual.example.yaml"))
    assert "teleop_profiles" not in render_station_yaml(draft)
    lines = [line for line in TEMPLATE_PATH.read_text().splitlines() if line.strip()]
    index = next(i for i, line in enumerate(lines) if line.startswith("safety_box:"))
    assert lines[index - 2] == "# Safety limits, enforced by the arm (and so by the daemon): servo targets are clipped,"
    assert lines[index - 1] == "# move_to_* targets outside them are refused. null = no limit (not recommended)."


# -- camera stream flags -----------------------------------------------------------------------------


def stream(camera):
    return camera.width, camera.height, camera.fps, camera.enable_depth


def test_camera_flags_default_to_the_camera_config_defaults(tmp_path):
    out = tmp_path / "station.yaml"
    assert run([*FLAGGED_ONE_ARM, "--output", out], never_asked)[0] == 0
    assert load_station(out).select().config.cameras == [CameraConfig(name="wrist", serial=SERIALS[0])]


def test_camera_flags_set_the_stream_of_new_cameras(tmp_path):
    out = tmp_path / "station.yaml"
    flags = ["--camera-resolution", "1280x720", "--camera-fps", "15", "--camera-depth", "yes"]
    code, output = run([*FLAGGED_ONE_ARM, *flags, "--output", out], never_asked)
    assert code == 0, output
    assert stream(load_station(out).select().config.cameras[0]) == (1280, 720, 15, True)


def test_camera_flags_take_one_value_per_arm(tmp_path):
    out = tmp_path / "station.yaml"
    flags = ["--camera-resolution", "1280x720", "640x480", "--camera-fps", "60", "--camera-depth", "no", "yes"]
    code, output = run(["--mock", "--output", out, *flags], Operator({"number of arms": "2"}))
    assert code == 0, output
    robots = load_station(out).robots
    left, right = robots["left"].config, robots["right"].config
    assert {stream(camera) for camera in left.cameras} == {(1280, 720, 60, False)}
    assert {stream(camera) for camera in right.cameras} == {(640, 480, 60, True)}


def test_from_keeps_a_camera_stream_unless_the_flag_is_given(tmp_path):
    first, second, third = (tmp_path / name for name in ("first.yaml", "second.yaml", "third.yaml"))
    flags = ["--camera-resolution", "1280x720", "--camera-fps", "15", "--camera-depth", "yes"]
    assert run([*FLAGGED_ONE_ARM, *flags, "--output", first], never_asked)[0] == 0
    again = ["--mock", "--non-interactive", "--only", "cameras"]
    code, output = run([*again, "--station", first, "--output", second], never_asked)
    assert code == 0, output
    assert stream(load_station(second).select().config.cameras[0]) == (1280, 720, 15, True)
    code, output = run([*again, "--station", second, "--camera-fps", "60", "--output", third], never_asked)
    assert code == 0, output
    assert stream(load_station(third).select().config.cameras[0]) == (1280, 720, 60, True)


@pytest.mark.parametrize(
    "flags, message",
    [
        (["--camera-resolution", "1280"], "--camera-resolution '1280': '1280' is not WIDTHxHEIGHT"),
        (["--camera-resolution", "1280x"], "is not WIDTHxHEIGHT"),
        (["--camera-resolution", "0x480"], "is not WIDTHxHEIGHT"),
        (["--camera-fps", "0"], "--camera-fps '0': fps must be positive, got 0"),
        (["--camera-fps", "fast"], "--camera-fps 'fast': 'fast' is not an integer"),
        (["--camera-depth", "maybe"], "--camera-depth 'maybe': answer yes or no"),
        (["--camera-fps", "30", "60"], "--camera-fps needs one value per arm (1) or one for all, got 2"),
    ],
)
def test_invalid_camera_flags_are_rejected(tmp_path, flags, message, capsys):
    out = tmp_path / "station.yaml"
    code, output = run([*FLAGGED_ONE_ARM, *flags, "--output", out], never_asked)
    error = capsys.readouterr().err
    assert "error:" not in output
    assert code == 2 and message in error and not out.exists()
