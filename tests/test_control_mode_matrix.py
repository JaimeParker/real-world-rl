"""Switching controllers is a station value only.

arm ``cartesian_controller`` in {impedance, internal, serl} x SpaceMouse ``--mode`` in {tracker, serl}:
for every combination the same station (only that value differs) loads, the env and the teleop
source build over fake franky / serl_controller modules, entering Cartesian servo starts the
controller's own motion, a teleop step reaches it as a servo target, and switching to joint servo,
IDLE and back works. ``rwrl teleop run --backend spacemouse --mode ...`` resolves every combination.
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent / "devices"))
from test_franky_backend import make_fake_franky, make_fake_serl_controller  # noqa: E402
from test_joint_trajectory import make_fake_joint_controller
from _teleop_fakes import make_hid  # noqa: E402
from test_spacemouse_teleop_source import _Part  # noqa: E402

import real_world_rl.devices.claims as claims  # noqa: E402
from real_world_rl.devices import ControlMode  # noqa: E402
from real_world_rl.devices.robots.franka import FrankaRobot  # noqa: E402
from real_world_rl.discovery.autoconfig import load_station  # noqa: E402
from real_world_rl.envs.real_robot import RealRobotEnv, RealRobotEnvConfig  # noqa: E402
from real_world_rl.teleop import make_teleop_source  # noqa: E402

CONTROLLERS = ("impedance", "internal", "serl")
MODES = ("tracker", "serl")
HOME = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
# The fake franky's measured TCP pose (0.3, -0.1, 0.5) lies inside.
SAFETY_BOX = [0.1, -0.5, 0.1, 0.8, 0.5, 0.9]


def station_text(controller: str) -> str:
    return yaml.safe_dump({
        "type": "franka", "name": "arm", "arm_backend": "franky", "robot_ip": "10.0.0.2",
        "realtime": "ignore", "cartesian_controller": controller, "gripper": {"backend": "fake"}, "cameras": [],
        "home_joints": HOME, "safety_box": SAFETY_BOX, "teleop": {"backend": "spacemouse"},
    }, sort_keys=False)


@pytest.fixture
def fakes(monkeypatch, tmp_path):
    log: list[str] = []
    franky = make_fake_franky(log)
    serl_controller = make_fake_serl_controller(log)
    monkeypatch.setitem(sys.modules, "franky", franky)
    monkeypatch.setitem(sys.modules, "serl_controller", serl_controller)
    monkeypatch.setitem(sys.modules, "joint_controller", make_fake_joint_controller())
    monkeypatch.setitem(sys.modules, "hid", make_hid())  # the station's SpaceMouse part connects with the robot
    monkeypatch.setattr(claims, "_DEFAULT_LOCK_DIR", tmp_path / "claims")

    def sleep(seconds):  # simulated time for SERL's joint reset: the blend runs and converges
        motion = serl_controller.SerlJointPositionMotion.instances[-1:] or [None]
        motion = motion[0]
        if motion is not None and not motion.finish_requested:
            motion.elapsed_s += seconds
            for robot in robots:
                robot.state.q = motion.target.copy()

    robots: list = []
    original_init = franky.Robot.__init__

    def init(self, *args, **kwargs):
        original_init(self, *args, **kwargs)
        robots.append(self)

    monkeypatch.setattr(franky.Robot, "__init__", init)
    monkeypatch.setattr("real_world_rl.devices.backends.arms.franky._sleep", sleep)
    return franky, serl_controller, log


def cartesian_motion_type(controller: str, franky, serl_controller):
    return {"impedance": franky.CartesianImpedanceTracker, "internal": franky.CartesianVelocityMotion,
            "serl": serl_controller.SerlCartesianImpedanceMotion}[controller]


def load_config(tmp_path: Path, controller: str):
    path = tmp_path / "station.yaml"
    path.write_text(station_text(controller))
    return load_station(path).select().config


def make_env(config, robot, mode: str) -> RealRobotEnv:
    serl = {"cartesian_semantics": "serl", "safety_box": SAFETY_BOX,
            "safety_box_rpy_low": [-3.14] * 3, "safety_box_rpy_high": [3.14] * 3} if mode == "serl" else {}
    # The tracker needs env scales at least its lead caps (one env step reaches every command).
    return RealRobotEnv(RealRobotEnvConfig(robot=config.to_dict(), reset_settle_s=0.0, xyz_scale_m=0.05,
                                           rot_scale_rad=0.3, **serl), robot=robot)


def make_source(env, mode: str):
    part = _Part()
    extra = {"last_gripper_action_getter": lambda: 1.0} if mode == "tracker" else {}
    source = make_teleop_source("spacemouse", env, mode=mode, teleop=part, **extra)
    source.start()
    source.reset()
    if mode == "tracker":
        part.press(0)  # button 0 takes over
    part.twist = np.array([0.5, 0.0, 0.0, 0.0, 0.0, 0.0])
    return part, source


def teleop_step(env, source) -> np.ndarray:
    """Teleop env steps until the operator's motion reaches the action (the tracker's take-over
    step holds the follower's pose first)."""
    for _ in range(3):
        reading = source.poll()
        assert reading.intervened
        env.step(reading.action)
        if reading.action[0] > 0:
            break
    return reading.action


def delivered_cartesian_targets(controller: str, arm, franky) -> list[np.ndarray]:
    """The Cartesian servo targets the running controller received, as [x, y, z] positions."""
    handle = arm._handle
    if controller == "impedance":
        return [np.asarray(target.translation) for target in handle.tracker.targets]
    if controller == "serl":
        return [np.asarray(position) for position, _ in handle.serl_motion.targets]
    # internal: each target is a Cartesian velocity motion; report the commanded linear twist.
    return [motion.target.twist.linear for motion, _ in handle.robot.moves
            if isinstance(motion, franky.CartesianVelocityMotion)]


def assert_cartesian_controller_running(controller: str, arm, franky, serl_controller) -> None:
    handle = arm._handle
    assert arm.control_mode is ControlMode.CARTESIAN_SERVO
    if controller == "impedance":
        assert isinstance(handle.tracker, franky.CartesianImpedanceTracker) and handle.serl_motion is None
    elif controller == "serl":
        assert isinstance(handle.serl_motion, serl_controller.SerlCartesianImpedanceMotion) and handle.tracker is None
    else:  # internal: no motion until the first target; then a velocity motion, no tracker
        assert handle.tracker is None and handle.serl_motion is None


@pytest.mark.parametrize("mode", MODES)
@pytest.mark.parametrize("controller", CONTROLLERS)
@pytest.mark.parametrize("joint_controller", ("impedance", "trajectory_impedance"))
def test_controller_x_spacemouse_mode(fakes, tmp_path, controller, mode, joint_controller):
    franky, serl_controller, _ = fakes
    config = load_config(tmp_path, controller)
    config.joint_controller = joint_controller
    assert config.cartesian_controller == controller and config.arm_backend == "franky"
    robot = FrankaRobot(config)
    env = make_env(config, robot, mode)
    try:
        env.reset()
        arm = robot.arm
        # reset: the joint reset is the controller's (SERL's for serl, franky's Ruckig otherwise)
        reset_motion = (serl_controller.SerlJointPositionMotion if controller == "serl" else franky.JointMotion)
        assert any(isinstance(motion, reset_motion) for motion, _ in arm._handle.robot.moves)
        assert_cartesian_controller_running(controller, arm, franky, serl_controller)
        part, source = make_source(env, mode)
        expected_source = "SpaceMouseSerlTeleopSource" if mode == "serl" else "SpaceMouseTeleopSource"
        assert type(source).__name__ == expected_source
        action = teleop_step(env, source)
        assert action[0] > 0  # +x on the puck
        targets = delivered_cartesian_targets(controller, arm, franky)
        assert targets and targets[-1][0] > (0.0 if controller == "internal" else 0.3)  # toward +x
        assert arm.last_cartesian_target[0] > 0.3

        # joint servo, IDLE and back to Cartesian servo, with no other change
        arm.set_control_mode(ControlMode.JOINT_SERVO)
        assert arm._handle.serl_motion is None
        if joint_controller == "impedance":
            assert isinstance(arm._handle.tracker, franky.JointImpedanceTracker)
        else:
            assert arm._handle.joint_trajectory_motion is not None
        q = np.asarray(arm.read_state()["joint_pos"])
        arm.servo_joints(q)
        assert (arm._handle.tracker if joint_controller == "impedance"
                else arm._handle.joint_trajectory_motion).targets
        arm.set_control_mode(ControlMode.IDLE)
        assert arm._handle.tracker is None and arm._handle.serl_motion is None
        assert not arm._handle.robot.is_in_control
        arm.set_control_mode(ControlMode.CARTESIAN_SERVO)
        assert_cartesian_controller_running(controller, arm, franky, serl_controller)
        source.reset()
        if mode == "tracker":
            part.press(0)
        before = len(delivered_cartesian_targets(controller, arm, franky))
        teleop_step(env, source)
        assert len(delivered_cartesian_targets(controller, arm, franky)) > before
        assert arm.health().ok
    finally:
        env.close()


def test_stations_differ_only_in_the_controller_value():
    lines = {controller: station_text(controller).splitlines() for controller in CONTROLLERS}
    for controller in CONTROLLERS[1:]:
        changed = [(a, b) for a, b in zip(lines["impedance"], lines[controller]) if a != b]
        assert changed == [("cartesian_controller: impedance", f"cartesian_controller: {controller}")]


def test_serl_declares_the_serl_controller_requirement(fakes, tmp_path, monkeypatch):
    from real_world_rl.devices import SdkMissing
    monkeypatch.setitem(sys.modules, "serl_controller", None)
    robot = FrankaRobot(load_config(tmp_path, "serl"))
    env = make_env(robot.config, robot, "tracker")
    try:
        with pytest.raises(SdkMissing, match="serl_controller"):
            env.reset()  # the joint reset is SERL's
    finally:
        env.close()


@pytest.mark.parametrize("controller, check", [("impedance", None), ("internal", "mode:internal_realtime"),
                                               ("serl", "mode:serl_realtime")])
def test_realtime_is_a_doctor_warning_only(fakes, tmp_path, monkeypatch, controller, check):
    import real_world_rl.devices.backends.arms.franky as franky_arm_module
    monkeypatch.setattr(franky_arm_module, "REALTIME_KERNEL_FLAG_PATH", str(tmp_path / "no-rt"))
    robot = FrankaRobot(load_config(tmp_path, controller))  # realtime: ignore
    robot.connect()
    try:
        results = {result.name: result for result in robot.arm.preflight()}
        assert all(result.ok for result in results.values())
        realtime = {name for name in results if name.endswith("_realtime")}
        if check is None:
            assert not realtime
        else:
            assert realtime == {check} and results[check].warning and "realtime: enforce" in results[check].hint
        robot.arm.set_control_mode(ControlMode.CARTESIAN_SERVO)  # still runs: a WARN, not a refusal
    finally:
        robot.disconnect()


# -- rwrl teleop run --backend spacemouse --mode {tracker, serl} x cartesian_controller -------

from test_cli_teleop_run import session  # noqa: E402,F401  (the CLI fixture: no child process, no devices)


@pytest.mark.parametrize("mode", MODES)
@pytest.mark.parametrize("controller", CONTROLLERS)
def test_cli_teleop_run_spacemouse_mode_x_controller(session, tmp_path, capsys, controller, mode):
    import json
    from real_world_rl.cli import main
    document = yaml.safe_load(session.path.read_text())
    document["cartesian_controller"] = controller
    session.path.write_text(yaml.safe_dump(document))
    preset = yaml.safe_load(session.preset.read_text())
    if mode == "serl":  # SERL's env semantics need its pose limits, as a serl preset sets them
        preset["real_robot"].update(safety_box=SAFETY_BOX, safety_box_rpy_low=[-3.14] * 3,
                                    safety_box_rpy_high=[3.14] * 3)
    session.preset.write_text(yaml.safe_dump(preset))
    assert main([*session.prefix, "teleop", "run", "--backend", "spacemouse", "--mode", mode,
                 "--record", str(tmp_path / "demos")]) == 0, capsys.readouterr().out
    payload = json.loads(capsys.readouterr().out)
    resolved = yaml.safe_load((Path(payload["report_directory"]) / "resolved-config.yaml").read_bytes())
    assert resolved["spacemouse_mode"] == mode and resolved["teleop_device"] == "spacemouse"
    assert resolved["real_robot"]["robot"]["cartesian_controller"] == controller
    assert resolved["real_robot"].get("cartesian_semantics", "current") == ("serl" if mode == "serl" else "current")
    assert len(session.calls) == 1


@pytest.mark.parametrize("mode", MODES)
def test_serl_reflex_mid_episode_auto_recovers_and_env_reset_starts_a_new_episode(fakes, tmp_path, mode):
    franky, serl_controller, _ = fakes
    config = load_config(tmp_path, "serl")
    robot = FrankaRobot(config)
    env = make_env(config, robot, mode)
    try:
        env.reset()
        arm = robot.arm
        part, source = make_source(env, mode)
        teleop_step(env, source)
        first = arm._handle.serl_motion
        robot_handle = arm._handle.robot
        robot_handle.is_in_control = False  # a reflex ends SERL's controller
        robot_handle.pending_motion_error = franky.ControlException("libfranka: cartesian_reflex")
        teleop_step(env, source)  # the next teleop step recovers and lands on a fresh controller
        assert arm._handle.serl_motion is not first and arm._handle.serl_motion.targets
        assert arm.serl_auto_recovery["episode_count"] == 1
        env.reset()
        assert arm.serl_auto_recovery["episode_count"] == 0 and arm.serl_auto_recovery["total_count"] == 1
    finally:
        env.close()
