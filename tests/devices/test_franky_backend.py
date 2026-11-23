"""Hardware-free tests for the franky Franka arm/gripper backends against a fake ``franky`` module."""
from __future__ import annotations

import dataclasses
import enum
import sys
import types

import numpy as np
import pytest

import real_world_rl.devices.backends.arms.franky as franky_arm_module
from real_world_rl.devices import (
    Arm,
    ControlMode,
    DeviceFault,
    DeviceNotFound,
    Gripper,
    GripperMode,
    SdkMissing,
    VersionMismatch,
)
from real_world_rl.devices.backends.arms.franky import FrankyArm
from real_world_rl.devices.backends.grippers.franky import FrankyGripper
from real_world_rl.devices.robots.franka import FrankaConfig

QUAT_Z90_XYZW = np.array([0.0, 0.0, np.sqrt(0.5), np.sqrt(0.5)])
JACOBIAN = np.arange(42, dtype=np.float64).reshape(6, 7) / 100.0


def make_fake_franky(log: list[str]) -> types.ModuleType:
    franky = types.ModuleType("franky")

    class FrankyException(Exception):
        pass

    for exc_name in (
        "CommandException",
        "ControlException",
        "IncompatibleVersionException",
        "NetworkException",
        "RealtimeException",
    ):
        setattr(franky, exc_name, type(exc_name, (FrankyException,), {}))
    franky.Exception = FrankyException
    franky.GripperException = type("GripperException", (RuntimeError,), {})
    franky.RealtimeConfig = enum.Enum("RealtimeConfig", "Enforce Ignore")
    franky.RobotMode = enum.Enum(
        "RobotMode", "Other Idle Move Guiding Reflex UserStopped AutomaticErrorRecovery"
    )
    franky.Frame = enum.Enum("Frame", "Joint1 EndEffector")

    class Affine:
        def __init__(self, translation=(0.0, 0.0, 0.0), quaternion=(0.0, 0.0, 0.0, 1.0)):
            self.translation = np.asarray(translation, dtype=np.float64)
            self.quaternion = np.asarray(quaternion, dtype=np.float64)

    class CartesianMotion:
        def __init__(self, target):
            self.target = target

    class JointMotion:
        def __init__(self, target):
            self.target = np.asarray(target)

    class Twist:
        def __init__(self, linear, angular):
            self.linear, self.angular = np.asarray(linear), np.asarray(angular)

    class RobotVelocity:
        def __init__(self, twist):
            self.twist = twist

    class Duration:
        def __init__(self, ms):
            self.ms = ms

    class CartesianVelocityMotion:
        def __init__(self, target, duration, relative_dynamics_factor=1.0):
            self.target, self.duration = target, duration
            self.relative_dynamics_factor = relative_dynamics_factor

    class RobotState:
        def __init__(self):
            from real_world_rl.devices.robots.franka_kinematics import FLANGE_T_HAND
            self.F_T_EE = FLANGE_T_HAND.copy()
            self.O_T_EE = Affine([0.3, -0.1, 0.5], QUAT_Z90_XYZW)
            self.q = np.linspace(0.0, 0.6, 7)
            self.dq = np.full(7, 0.1)
            self.O_F_ext_hat_K = np.array([1.0, 2.0, 3.0, 0.4, 0.5, 0.6])
            self.robot_mode = franky.RobotMode.Idle
            self.current_errors = "Errors()"

    class Model:
        def zero_jacobian(self, frame, state):
            assert frame is franky.Frame.EndEffector
            return JACOBIAN

    class Robot:
        connect_error: Exception | None = None

        def __init__(self, fci_hostname, **kwargs):
            if Robot.connect_error is not None:
                raise Robot.connect_error
            log.append("Robot")
            self.fci_hostname = fci_hostname
            self.kwargs = kwargs
            self.state = RobotState()
            self.model = Model()
            self.is_in_control = False
            self.has_errors = False
            self.pending_motion_error: Exception | None = None
            self.moves: list[tuple[object, bool]] = []
            self.move_options: list[dict] = []
            self.loads: list[tuple] = []
            self.set_load_error: Exception | None = None

        def move(self, motion, asynchronous=False, limit_rate=None, cutoff_frequency=None):
            log.append(f"move:{type(motion).__name__}")
            self.moves.append((motion, asynchronous))
            self.move_options.append({"limit_rate": limit_rate, "cutoff_frequency": cutoff_frequency})
            self.is_in_control = isinstance(motion, _Tracker) or asynchronous

        def set_load(self, load_mass, F_x_Cload, load_inertia):
            log.append("set_load")
            if self.set_load_error is not None:
                raise self.set_load_error
            self.loads.append((load_mass, list(F_x_Cload), list(load_inertia)))

        def join_motion(self, timeout=None):
            log.append("join_motion")
            self.is_in_control = False
            error, self.pending_motion_error = self.pending_motion_error, None
            if error is not None:
                raise error
            return True

        def stop(self):
            log.append("robot.stop")
            if self.is_in_control:
                self.pending_motion_error = franky.ControlException("libfranka: Move command preempted!")
            self.is_in_control = False

        def recover_from_errors(self):
            log.append("recover_from_errors")
            self.state.robot_mode = franky.RobotMode.Idle
            return True

    class _Tracker:
        def __init__(self, robot):
            log.append(type(self).__name__)
            self.robot = robot
            self.targets: list[object] = []
            robot.move(self, asynchronous=True)

        def set_target(self, target):
            log.append(f"{type(self).__name__}.set_target")
            self.targets.append(target)

        def stop(self):
            log.append(f"{type(self).__name__}.stop")
            self.robot.join_motion()

    class Gripper:
        connect_error: Exception | None = None
        grasp_behavior = "ok"

        def __init__(self, fci_hostname):
            if Gripper.connect_error is not None:
                raise Gripper.connect_error
            self.fci_hostname = fci_hostname
            self.width = 0.04
            self.is_grasped = False
            self.calls: list[tuple] = []

        def open(self, speed):
            self.calls.append(("open", speed))
            return True

        def move(self, width, speed):
            self.calls.append(("move", width, speed))
            return True

        def grasp(self, width, speed, force, epsilon_inner=0.005, epsilon_outer=0.005):
            self.calls.append(("grasp", width, speed, force, epsilon_inner, epsilon_outer))
            if Gripper.grasp_behavior == "gripper_exception":
                raise franky.GripperException("nothing grasped")
            if Gripper.grasp_behavior == "command_exception":
                raise franky.CommandException("grasp command rejected")
            return Gripper.grasp_behavior == "ok"

    franky.Affine = Affine
    franky.CartesianMotion = CartesianMotion
    franky.JointMotion = JointMotion
    franky.Twist, franky.RobotVelocity, franky.Duration = Twist, RobotVelocity, Duration
    franky.CartesianVelocityMotion = CartesianVelocityMotion
    franky.TorqueStopMotion = type("TorqueStopMotion", (), {})
    franky.Robot = Robot
    franky.Gripper = Gripper
    franky.CartesianImpedanceTracker = type("CartesianImpedanceTracker", (_Tracker,), {})
    franky.JointImpedanceTracker = type("JointImpedanceTracker", (_Tracker,), {})
    return franky


@pytest.fixture
def log():
    return []


@pytest.fixture
def franky(monkeypatch, log):
    fake = make_fake_franky(log)
    monkeypatch.setitem(sys.modules, "franky", fake)
    return fake


@pytest.fixture
def arm(franky):
    arm = FrankyArm(robot_ip="10.0.0.2", realtime="ignore")
    arm.connect()
    yield arm
    arm.disconnect()


@pytest.fixture
def gripper(franky):
    gripper = FrankyGripper(robot_ip="10.0.0.2", max_width_m=0.08, speed=0.1, force=30.0)
    gripper.connect()
    yield gripper
    gripper.disconnect()


def test_registry_resolves_franky():
    assert Arm.backend("franky") is FrankyArm
    assert Gripper.backend("franky") is FrankyGripper


def test_from_config_wiring():
    config = FrankaConfig(limits={"joints": [[-4., 4.]] * 7}, robot_ip="192.168.1.7", relative_dynamics_factor=0.1, realtime="enforce")
    arm = FrankyArm.from_config(config)
    assert (arm.name, arm.robot_ip, arm.relative_dynamics_factor, arm.realtime) == (
        "arm",
        "192.168.1.7",
        0.1,
        "enforce",
    )
    assert arm.home_joints == config.home_joints
    assert arm.num_joints == 7
    gripper = FrankyGripper.from_config(config)
    assert (gripper.robot_ip, gripper.max_width_m, gripper.mode) == (
        "192.168.1.7",
        0.08,
        GripperMode.BINARY,
    )


def test_missing_sdk_hints_distribution_name(monkeypatch):
    monkeypatch.setitem(sys.modules, "franky", None)
    with pytest.raises(SdkMissing) as info:
        FrankyArm(robot_ip="10.0.0.2").connect()
    assert "franky-control" in info.value.hint


def test_connect_passes_settings_and_reads_first_feedback(arm, franky):
    robot = arm._handle.robot
    assert robot.fci_hostname == "10.0.0.2"
    assert robot.kwargs == {
        "relative_dynamics_factor": 0.2,
        "realtime_config": franky.RealtimeConfig.Ignore,
    }
    health = arm.health()
    assert health.first_feedback_received and health.ok
    assert health.robot_mode == "Idle"


def test_enforce_realtime_config(franky):
    arm = FrankyArm(robot_ip="10.0.0.2", realtime="enforce")
    arm.connect()
    assert arm._handle.robot.kwargs["realtime_config"] is franky.RealtimeConfig.Enforce


def test_incompatible_version_maps_to_version_mismatch(franky):
    franky.Robot.connect_error = franky.IncompatibleVersionException("server version: 5, library version: 9")
    with pytest.raises(VersionMismatch) as info:
        FrankyArm(robot_ip="10.0.0.2").connect()
    assert "server version: 5" in info.value.message
    assert "by-robot-server-version" in info.value.hint


@pytest.mark.parametrize("error_name", ["NetworkException", "RuntimeError"])
def test_unreachable_robot_maps_to_device_not_found(franky, error_name):
    error_cls = getattr(franky, error_name, RuntimeError)
    franky.Robot.connect_error = error_cls("UDP receive: Timeout")
    arm = FrankyArm(robot_ip="10.0.0.2")
    with pytest.raises(DeviceNotFound) as info:
        arm.connect()
    assert "10.0.0.2" in info.value.message
    assert "FCI" in info.value.hint
    assert not arm.is_connected


def test_realtime_exception_gets_realtime_hint(franky):
    franky.Robot.connect_error = franky.RealtimeException("kernel does not have realtime capabilities")
    with pytest.raises(DeviceFault) as info:
        FrankyArm(robot_ip="10.0.0.2", realtime="enforce").connect()
    assert "realtime: ignore" in info.value.hint


def test_read_state_shapes_and_values(arm):
    state = arm.read_state()
    assert {k: v.shape for k, v in state.items()} == {
        "tcp_pose": (7,),
        "tcp_vel": (6,),
        "joint_pos": (7,),
        "joint_vel": (7,),
        "tcp_force": (3,),
        "tcp_torque": (3,),
    }
    np.testing.assert_allclose(state["tcp_pose"][:3], [0.3, -0.1, 0.5])
    np.testing.assert_allclose(state["tcp_pose"][3:], QUAT_Z90_XYZW)  # xyzw, w last
    np.testing.assert_allclose(state["tcp_vel"], JACOBIAN @ np.full(7, 0.1))
    np.testing.assert_allclose(state["tcp_force"], [1.0, 2.0, 3.0])
    np.testing.assert_allclose(state["tcp_torque"], [0.4, 0.5, 0.6])
    observation = arm.get_observation()
    assert observation["joint_pos"].dtype == np.float32


def test_cartesian_servo_starts_tracker_and_sets_target(arm, franky, log):
    log.clear()
    arm.set_control_mode(ControlMode.CARTESIAN_SERVO)
    assert log == [
        "join_motion",
        "recover_from_errors",
        "CartesianImpedanceTracker",
        "move:CartesianImpedanceTracker",
    ]
    pose = np.array([0.4, 0.0, 0.3, *QUAT_Z90_XYZW])
    arm.send_action({"tcp_pose": pose})
    target = arm._handle.tracker.targets[-1]
    assert isinstance(target, franky.Affine)
    np.testing.assert_allclose(target.translation, pose[:3])
    np.testing.assert_allclose(target.quaternion, pose[3:])


def test_switch_to_joint_servo_stops_old_tracker_and_recovers(arm, franky, log):
    arm.set_control_mode(ControlMode.CARTESIAN_SERVO)
    old_tracker = arm._handle.tracker
    log.clear()
    arm.set_control_mode(ControlMode.JOINT_SERVO)
    stop_index = log.index("CartesianImpedanceTracker.stop")
    assert log[stop_index:][-4:] == [
        "join_motion",
        "recover_from_errors",
        "JointImpedanceTracker",
        "move:JointImpedanceTracker",
    ]
    assert isinstance(arm._handle.tracker, franky.JointImpedanceTracker)
    assert arm._handle.tracker is not old_tracker
    q = np.full(7, 0.2)
    arm.send_action({"joint_pos": q})
    np.testing.assert_allclose(arm._handle.tracker.targets[-1], q)


def test_move_to_joints_in_servo_mode_is_a_fault(arm):
    arm.set_control_mode(ControlMode.JOINT_SERVO)
    with pytest.raises(DeviceFault) as info:
        arm.move_to_joints(np.zeros(7))
    assert "set_control_mode" in info.value.hint


@pytest.mark.parametrize("blocking", [True, False])
def test_move_to_joints_in_idle(arm, franky, blocking):
    q = np.linspace(0.0, 0.3, 7)
    arm.move_to_joints(q, blocking=blocking)
    motion, asynchronous = arm._handle.robot.moves[-1]
    assert isinstance(motion, franky.JointMotion)
    np.testing.assert_allclose(motion.target, q)
    assert asynchronous is (not blocking)


def test_move_to_pose_in_idle(arm, franky):
    pose = np.array([0.4, 0.1, 0.2, *QUAT_Z90_XYZW])
    arm.move_to_pose(pose, blocking=False)
    motion, asynchronous = arm._handle.robot.moves[-1]
    assert isinstance(motion, franky.CartesianMotion)
    np.testing.assert_allclose(motion.target.quaternion, QUAT_Z90_XYZW)
    assert asynchronous is True


def test_stop_in_idle_is_a_noop(arm, log):
    log.clear()
    arm.stop()
    assert log == []


def test_stop_in_idle_halts_async_position_motion(arm, log):
    arm.move_to_joints(np.zeros(7), blocking=False)
    log.clear()
    arm.stop()
    assert log == ["robot.stop", "join_motion"]
    assert arm.health().ok  # the preemption caused by the deliberate stop is not a fault


def test_servo_after_stop_restarts_tracker(arm, log):
    arm.set_control_mode(ControlMode.CARTESIAN_SERVO)
    arm.stop()
    assert arm._handle.tracker is None
    arm.send_action({"tcp_pose": np.array([0.4, 0.0, 0.3, 0.0, 0.0, 0.0, 1.0])})
    assert len(arm._handle.tracker.targets) == 1


def test_tracker_restart_after_plain_stop_never_resends_old_target(arm, log):
    old = np.array([0.4, 0.0, 0.3, 0.0, 0.0, 0.0, 1.0])
    new = np.array([0.35, 0.05, 0.25, 0.0, 0.0, 0.0, 1.0])
    arm.set_control_mode(ControlMode.CARTESIAN_SERVO)
    arm.servo_cartesian(old)
    np.testing.assert_array_equal(arm.last_cartesian_target, old)
    arm.stop()
    assert arm.last_cartesian_target is None
    assert arm.control_mode is ControlMode.CARTESIAN_SERVO
    restarted = arm._live_tracker(ControlMode.CARTESIAN_SERVO)  # what the next servo call does
    assert restarted.targets == []  # the restart itself sends nothing, least of all the old target
    arm.servo_cartesian(new)
    assert arm._handle.tracker is restarted
    assert len(restarted.targets) == 1
    np.testing.assert_allclose(restarted.targets[0].translation, new[:3])
    np.testing.assert_array_equal(arm.last_cartesian_target, new)


def test_stop_with_trace_clears_last_cartesian_target(arm):
    arm.set_control_mode(ControlMode.CARTESIAN_SERVO)
    arm.servo_cartesian(np.array([0.4, 0.0, 0.3, 0.0, 0.0, 0.0, 1.0]))
    assert arm.last_cartesian_target is not None
    arm.stop_with_trace(post_stop_s=0)
    assert arm.control_mode is ControlMode.IDLE
    assert arm.last_cartesian_target is None


def test_failed_stop_with_trace_still_clears_last_cartesian_target(arm):
    arm.set_control_mode(ControlMode.CARTESIAN_SERVO)
    arm.servo_cartesian(np.array([0.4, 0.0, 0.3, 0.0, 0.0, 0.0, 1.0]))
    def fail():
        raise RuntimeError('trace stop failure')
    arm._handle.tracker.stop = fail
    with pytest.raises(DeviceFault, match='trace stop failure'):
        arm.stop_with_trace(post_stop_s=0)
    assert arm.last_cartesian_target is None


def test_dead_tracker_raises_recoverable_fault(arm):
    arm.set_control_mode(ControlMode.JOINT_SERVO)
    arm._handle.robot.is_in_control = False  # e.g. a reflex ended the torque motion
    with pytest.raises(DeviceFault) as info:
        arm.send_action({"joint_pos": np.zeros(7)})
    assert info.value.recoverable
    assert not arm.health().ok
    with pytest.raises(DeviceFault, match="unrecovered"):
        arm.send_action({"joint_pos": np.zeros(7)})
    arm.recover()
    arm.send_action({"joint_pos": np.zeros(7)})
    assert arm.health().ok


def test_disconnect_stops_tracker_and_never_powers_off(franky, log):
    arm = FrankyArm(robot_ip="10.0.0.2")
    arm.connect()
    arm.set_control_mode(ControlMode.CARTESIAN_SERVO)
    robot = arm._handle.robot
    log.clear()
    arm.disconnect()
    assert log[0] == "CartesianImpedanceTracker.stop"
    assert not any(word in entry.lower() for entry in log for word in ("power", "shutdown", "brake"))
    assert not robot.is_in_control
    assert not arm.is_connected


def test_health_records_robot_errors_once(arm):
    arm._handle.robot.has_errors = True
    arm.health()
    health = arm.health()
    assert not health.ok
    assert len(health.errors) == 1


@pytest.mark.parametrize(
    ("realtime", "flag", "ok"),
    [("enforce", "0", False), ("ignore", "0", True), ("enforce", "1", True), ("enforce", None, False)],
)
def test_preflight_realtime_kernel(franky, monkeypatch, tmp_path, realtime, flag, ok):
    flag_path = tmp_path / "realtime"
    if flag is not None:
        flag_path.write_text(f"{flag}\n")
    monkeypatch.setattr(franky_arm_module, "REALTIME_KERNEL_FLAG_PATH", str(flag_path))
    arm = FrankyArm(robot_ip="10.0.0.2", realtime=realtime)
    arm.connect()
    results = {result.name: result for result in arm.preflight()}
    check = results["mode:realtime_kernel"]
    assert check.layer == 5 and check.ok is ok
    if not ok:
        assert "PREEMPT_RT" in check.hint and "realtime: ignore" in check.hint
    if realtime == "ignore":
        assert "RealtimeConfig.Ignore" in check.message
    assert results["mode:libfranka_version"].ok


def test_preflight_robot_mode_flags_reflex_and_points_at_rwrl_recover(arm, franky):
    arm._handle.robot.state.robot_mode = franky.RobotMode.Reflex
    arm.read_state()
    check = {result.name: result for result in arm.preflight()}["mode:robot_mode"]
    assert not check.ok
    assert "rwrl recover" in check.hint and "--fix" not in check.hint
    arm.recover()  # what rwrl recover runs; the fake recover_from_errors returns the robot to Idle
    assert {result.name: result for result in arm.preflight()}["mode:robot_mode"].ok


def test_gripper_connect_failure_maps_to_device_not_found(franky):
    franky.Gripper.connect_error = franky.NetworkException("Connection refused")
    with pytest.raises(DeviceNotFound) as info:
        FrankyGripper(robot_ip="10.0.0.2").connect()
    assert "FCI" in info.value.hint


def test_gripper_open_close_move_width(gripper):
    hand = gripper._handle
    gripper.open()
    gripper.close()
    gripper.move(0.5)
    assert hand.calls == [
        ("open", 0.1),
        ("grasp", 0.0, 0.1, 30.0, 1.0, 1.0),
        ("move", 0.04, 0.1),
    ]
    assert gripper.width == pytest.approx(0.5)
    hand.width = 0.09
    assert gripper.width == 1.0
    hand.is_grasped = True
    assert gripper.is_grasped is True


@pytest.mark.parametrize("behavior", ["fail", "gripper_exception"])
def test_gripper_close_on_nothing_does_not_raise(gripper, franky, behavior):
    franky.Gripper.grasp_behavior = behavior
    gripper.close()
    assert gripper._handle.calls[-1][0] == "grasp"


def test_gripper_rejected_grasp_command_is_a_fault(gripper, franky):
    franky.Gripper.grasp_behavior = "command_exception"
    with pytest.raises(DeviceFault, match="grasp command rejected"):
        gripper.close()


def test_gripper_binary_send_action(gripper):
    gripper.send_action({"gripper": np.array([1.0])})
    gripper.send_action({"gripper": np.array([0.0])})
    assert [call[0] for call in gripper._handle.calls] == ["open", "grasp"]


def test_gripper_continuous_send_action(franky):
    gripper = FrankyGripper(robot_ip="10.0.0.2", mode=GripperMode.CONTINUOUS, speed=0.1)
    gripper.connect()
    gripper.send_action({"gripper": np.array([0.25])})
    assert gripper._handle.calls == [("move", pytest.approx(0.02), 0.1)]


def test_reflex_during_move_maps_to_recoverable_fault(arm, franky):
    def reflex(motion, asynchronous=False):
        raise franky.ControlException("libfranka: Move command aborted: motion aborted by reflex!")

    arm._handle.robot.move = reflex
    with pytest.raises(DeviceFault, match="reflex") as info:
        arm.move_to_joints(np.zeros(7))
    assert info.value.recoverable is True
    assert "recover()" in info.value.hint
    assert not arm.health().ok


def test_reflex_starting_tracker_maps_to_recoverable_fault(arm, franky):
    def reflex(motion, asynchronous=False):
        raise franky.ControlException("cartesian_reflex")

    arm._handle.robot.move = reflex
    with pytest.raises(DeviceFault, match="cartesian_reflex") as info:
        arm.set_control_mode(ControlMode.CARTESIAN_SERVO)
    assert info.value.recoverable is True
    assert any("cartesian_reflex" in error for error in arm.health().errors)


def test_reflex_of_async_move_survives_mode_switch_until_recover(arm, franky):
    arm.move_to_joints(np.zeros(7), blocking=False)
    robot = arm._handle.robot
    robot.is_in_control = False  # the motion already ended, with a reflex
    robot.pending_motion_error = franky.ControlException("joint_reflex")
    arm.set_control_mode(ControlMode.JOINT_SERVO)
    health = arm.health()
    assert not health.ok
    assert any("joint_reflex" in error for error in health.errors)
    arm.recover()
    assert arm.health().ok


def test_deliberate_stop_before_mode_switch_is_not_an_error(arm):
    arm.move_to_joints(np.zeros(7), blocking=False)
    arm.set_control_mode(ControlMode.JOINT_SERVO)
    assert arm.health().ok


def test_recover_failure_keeps_errors(arm):
    arm._note_error("reflex")
    arm._handle.robot.recover_from_errors = lambda: False
    with pytest.raises(DeviceFault, match="did not recover"):
        arm.recover()
    assert arm.health().errors == ["reflex"]


def test_gripper_from_config_rejects_borrowed_session(arm):
    with pytest.raises(ValueError, match="shares_arm_session: false"):
        FrankyGripper.from_config(FrankaConfig(limits={"joints": [[-4., 4.]] * 7}, robot_ip="10.0.0.2"), owner=arm)


def test_tracker_stop_failure_is_recorded_without_raising(arm):
    arm.set_control_mode(ControlMode.CARTESIAN_SERVO)
    def fail():
        raise RuntimeError("stop timed out")
    arm._handle.tracker.stop = fail
    arm.stop()
    assert any("stop timed out" in error for error in arm.health().errors)
    assert arm._handle.tracker is None
    # A later mode transition still reaches the existing settle/recover path.
    arm.set_control_mode(ControlMode.JOINT_SERVO)
    assert arm.control_mode is ControlMode.JOINT_SERVO


def test_position_stop_join_failure_is_recorded(arm, monkeypatch):
    arm.move_to_joints(np.zeros(7), blocking=False)
    def fail():
        raise RuntimeError("join lost connection")
    monkeypatch.setattr(arm._handle.robot, "join_motion", fail)
    arm.stop()
    assert any("join lost connection" in error for error in arm.health().errors)


def test_position_stop_timeout_is_recorded(arm, monkeypatch):
    arm.move_to_joints(np.zeros(7), blocking=False)
    monkeypatch.setattr(arm._handle.robot, "join_motion", lambda: False)
    arm.stop()
    assert any("did not complete" in error for error in arm.health().errors)


def test_stop_sample_uses_sdk_time_and_unfiltered_feedback(arm):
    from types import SimpleNamespace
    state = arm._handle.robot.state
    state.time = SimpleNamespace(to_sec=lambda: 12.34)
    state.current_errors = []
    snapshot = arm.stop_sample()
    assert snapshot['device_time_s'] == 12.34
    assert snapshot['joint_vel'] == state.dq.tolist()
    assert snapshot['errors'] == []
    assert snapshot['last_cartesian_target'] is None


def test_stop_sample_reads_named_error_flags_not_object_truthiness(arm):
    from types import SimpleNamespace
    state = arm._handle.robot.state
    state.current_errors = SimpleNamespace(joint_reflex=False, cartesian_reflex=False)
    assert bool(state.current_errors)  # SDK object truthiness is not error status.
    assert arm.stop_sample()['errors'] == []
    state.current_errors.cartesian_reflex = True
    assert arm.stop_sample()['errors'] == ['cartesian_reflex']


def test_normal_stop_trace_uses_same_robot_and_does_not_claim_sparse_pass(arm):
    robot = arm._handle.robot
    arm.set_control_mode(ControlMode.CARTESIAN_SERVO)
    arm.begin_stop_trace()
    result = arm.stop_with_trace(post_stop_s=0)
    assert arm._handle.robot is robot
    assert result['assessment']['status'] == 'BLOCKED'
    assert result['trace']['stop_end_s'] >= result['trace']['stop_start_s']
    assert result['trace']['control_mode'] == 'cartesian_servo'
    assert result['trace']['tracker_active'] is True
    assert arm.last_stop_trace == result
    result['trace']['samples'].clear()
    assert arm.last_stop_trace is not result


def test_traced_stop_failure_raises_and_retains_evidence(arm):
    arm.set_control_mode(ControlMode.CARTESIAN_SERVO)
    arm.begin_stop_trace()
    def fail():
        raise RuntimeError('trace stop failure')
    arm._handle.tracker.stop = fail
    with pytest.raises(DeviceFault, match='trace stop failure'):
        arm.stop_with_trace(post_stop_s=0)
    assert 'trace stop failure' in arm.last_stop_trace['trace']['stop_error']
    assert arm.last_stop_trace['assessment']['status'] == 'FAIL'


def test_emergency_stop_does_not_use_normal_trace_wait(arm, monkeypatch):
    arm.set_control_mode(ControlMode.CARTESIAN_SERVO)
    def forbidden(*args, **kwargs):
        raise AssertionError('emergency stop must not call normal tracing workflow')
    monkeypatch.setattr(arm, 'stop_with_trace', forbidden)
    arm.stop()


def test_post_stop_observation_does_not_hold_stop_lock(arm, monkeypatch):
    import threading
    entered, release, emergency_done = threading.Event(), threading.Event(), threading.Event()
    failures = []
    arm.set_control_mode(ControlMode.CARTESIAN_SERVO)
    arm.begin_stop_trace()
    def wait(_):
        entered.set()
        release.wait(2.)
    monkeypatch.setattr('real_world_rl.devices.backends.arms.franky.time.sleep', wait)
    def normal():
        try:
            arm.stop_with_trace()
        except Exception as error:
            failures.append(error)
    def emergency():
        try:
            arm.stop()
        finally:
            emergency_done.set()
    thread = threading.Thread(target=normal)
    thread.start()
    emergency_thread = None
    try:
        assert entered.wait(1.)
        emergency_thread = threading.Thread(target=emergency)
        emergency_thread.start()
        assert emergency_done.wait(.5)
    finally:
        release.set()
        thread.join(2.)
        if emergency_thread is not None:
            emergency_thread.join(2.)
    assert not failures


def test_traced_stop_is_idle_before_observation_and_rejects_new_motion(arm, monkeypatch):
    arm.set_control_mode(ControlMode.CARTESIAN_SERVO)
    arm.begin_stop_trace()
    observed = []
    def observe(seconds):
        observed.append(seconds)
        assert arm.control_mode is ControlMode.IDLE
        # Exercise the actual daemon predicate with an expired action deadline.
        import threading
        from real_world_rl.server.service import RobotService
        service = object.__new__(RobotService)
        service._lock = threading.RLock()
        service.robot = types.SimpleNamespace(arm=arm)
        service._last_action_time = service._servo_entered_time = 0.
        service.watchdog_s = .01
        service._watchdog_trips = 0
        service._watchdog_check()
        assert service._watchdog_trips == 0
        for operation in (
            lambda: arm.set_control_mode(ControlMode.CARTESIAN_SERVO),
            lambda: arm.move_to_joints(np.zeros(7)),
            lambda: arm.servo_cartesian(np.array([0, 0, 0, 0, 0, 0, 1.])),
        ):
            with pytest.raises(DeviceFault, match='observation in progress'):
                operation()
        arm.stop()  # Emergency/plain stop remains available during observation.
    monkeypatch.setattr(franky_arm_module.time, 'sleep', observe)
    arm.stop_with_trace(post_stop_s=.1)
    assert observed == [.1]
    assert arm.control_mode is ControlMode.IDLE
    arm.set_control_mode(ControlMode.CARTESIAN_SERVO)


def test_idle_arm_traced_stop_is_blocked(arm):
    assert arm.control_mode is ControlMode.IDLE
    result = arm.stop_with_trace(post_stop_s=0)
    assert result['trace']['control_mode'] == 'idle'
    assert result['trace']['tracker_active'] is False
    assert result['assessment']['status'] == 'BLOCKED'
    assert 'no active motion at stop' in result['assessment']['missing_evidence']


def test_trace_preparation_failure_cannot_prevent_stop(arm, monkeypatch):
    arm.set_control_mode(ControlMode.CARTESIAN_SERVO)
    tracker = arm._handle.tracker
    stopped = []
    original = tracker.stop
    def stop():
        stopped.append(True)
        return original()
    tracker.stop = stop
    def unavailable():
        assert stopped == [True]
        raise OSError('metadata unavailable')
    monkeypatch.setattr(arm, '_new_stop_recorder', unavailable)
    evidence = arm.stop_with_trace(post_stop_s=0)
    assert arm.control_mode is ControlMode.IDLE
    assert evidence['assessment']['status'] == 'BLOCKED'
    assert 'metadata unavailable' in evidence['trace']['sampling_errors'][0]


def test_trace_metadata_cached_per_connection_outside_stop_lock(arm, monkeypatch):
    from real_world_rl.devices import provenance
    calls = []
    original = provenance.sdk_artifacts
    def fingerprint(*args, **kwargs):
        assert not arm._stop_lock._is_owned()
        calls.append(True)
        return original(*args, **kwargs)
    monkeypatch.setattr(provenance, 'sdk_artifacts', fingerprint)
    arm.begin_stop_trace()
    previous = arm._stop_recorder
    finish = previous.finish
    def finish_without_lock(*args, **kwargs):
        assert not arm._stop_lock._is_owned()
        return finish(*args, **kwargs)
    monkeypatch.setattr(previous, 'finish', finish_without_lock)
    arm.begin_stop_trace()
    assert calls == [True]
    arm.disconnect()
    arm.connect()
    arm.begin_stop_trace()
    assert calls == [True, True]


def test_slow_trace_preparation_does_not_block_plain_stop(arm, monkeypatch):
    import threading
    entered, release, stopped = threading.Event(), threading.Event(), threading.Event()
    errors = []
    arm.set_control_mode(ControlMode.CARTESIAN_SERVO)
    original = arm._new_stop_recorder
    def slow():
        entered.set()
        assert release.wait(2.)
        return original()
    monkeypatch.setattr(arm, '_new_stop_recorder', slow)
    def begin():
        try:
            arm.begin_stop_trace()
        except BaseException as error:
            errors.append(error)
    thread = threading.Thread(target=begin)
    thread.start()
    stopper = threading.Thread(target=lambda: (arm.stop(), stopped.set()))
    try:
        assert entered.wait(1.)
        stopper.start()
        assert stopped.wait(1.)
    finally:
        release.set()
        thread.join(2.)
        if stopper.ident is not None:
            stopper.join(2.)
    assert not errors


def test_recorder_mark_failure_cannot_prevent_stop(arm, monkeypatch):
    arm.set_control_mode(ControlMode.CARTESIAN_SERVO)
    arm.begin_stop_trace()
    def fail(**kwargs):
        raise OSError('recorder marking failed')
    monkeypatch.setattr(arm._stop_recorder, 'mark_stop_start', fail)
    result = arm.stop_with_trace(post_stop_s=0)
    assert arm.control_mode is ControlMode.IDLE
    assert arm._handle.tracker is None
    assert result['assessment']['status'] == 'BLOCKED'
    assert any('recorder marking failed' in message for message in result['trace']['sampling_errors'])


# --- cartesian_controller: internal ---------------------------------------------------------


@pytest.fixture
def internal_arm(franky):
    arm = FrankyArm(robot_ip="10.0.0.2", realtime="ignore", cartesian_controller="internal")
    arm.connect()
    yield arm
    arm.disconnect()


def test_internal_controller_sends_velocity_motions_not_a_tracker(internal_arm, franky, log):
    log.clear()
    internal_arm.set_control_mode(ControlMode.CARTESIAN_SERVO)
    assert "CartesianImpedanceTracker" not in log
    assert internal_arm._handle.tracker is None
    measured = np.array([0.3, -0.1, 0.5])
    internal_arm.send_action({"tcp_pose": np.array([*(measured + [0.01, 0.0, 0.0]), *QUAT_Z90_XYZW])})
    motion, asynchronous = internal_arm._handle.robot.moves[-1]
    assert asynchronous and isinstance(motion, franky.CartesianVelocityMotion)
    twist = motion.target.twist
    np.testing.assert_allclose(twist.linear, [0.01 / franky_arm_module.INTERNAL_HORIZON_S, 0, 0], atol=1e-12)
    np.testing.assert_allclose(twist.angular, 0.0, atol=1e-9)
    assert motion.duration.ms == franky_arm_module.INTERNAL_HOLD_MS
    assert motion.relative_dynamics_factor == internal_arm.relative_dynamics_factor
    internal_arm.send_action({"tcp_pose": np.array([*measured, *QUAT_Z90_XYZW])})
    assert len(internal_arm._handle.robot.moves) == 2  # every target preempts the last motion


def test_internal_twist_is_clipped_and_rotates_about_the_base_frame():
    identity = np.array([0.0, 0.0, 0.0, 1.0])
    yaw = np.array([0.0, 0.0, np.sin(0.05), np.cos(0.05)])  # +0.1 rad about base z
    linear, angular = franky_arm_module.internal_servo_twist(
        np.array([1.0, 0.0, 0.0, *yaw]), np.zeros(3), identity)
    np.testing.assert_allclose(linear, [franky_arm_module.INTERNAL_MAX_SPEED_M_S, 0, 0])
    np.testing.assert_allclose(angular, [0, 0, 0.1 / franky_arm_module.INTERNAL_HORIZON_S], atol=1e-9)
    flip = np.array([1.0, 0.0, 0.0, 0.0])  # pi about x
    _, angular = franky_arm_module.internal_servo_twist(np.array([0, 0, 0, *flip]), np.zeros(3), identity)
    np.testing.assert_allclose(np.linalg.norm(angular), franky_arm_module.INTERNAL_MAX_SPEED_RAD_S)


def test_internal_motion_killed_by_a_reflex_is_a_fault(internal_arm, franky):
    internal_arm.set_control_mode(ControlMode.CARTESIAN_SERVO)
    pose = np.array([0.3, -0.1, 0.5, *QUAT_Z90_XYZW])
    internal_arm.send_action({"tcp_pose": pose})
    robot = internal_arm._handle.robot
    robot.is_in_control = False
    robot.pending_motion_error = franky.ControlException("cartesian_motion_generator_joint_velocity_discontinuity")
    with pytest.raises(DeviceFault, match="discontinuity"):
        internal_arm.send_action({"tcp_pose": pose})
    with pytest.raises(DeviceFault, match="unrecovered"):
        internal_arm.send_action({"tcp_pose": pose})


def test_internal_motion_that_came_to_rest_restarts(internal_arm, franky):
    internal_arm.set_control_mode(ControlMode.CARTESIAN_SERVO)
    pose = np.array([0.3, -0.1, 0.5, *QUAT_Z90_XYZW])
    internal_arm.send_action({"tcp_pose": pose})
    internal_arm._handle.robot.is_in_control = False  # its hold time ran out
    internal_arm.send_action({"tcp_pose": pose})
    assert len(internal_arm._handle.robot.moves) == 2


def test_internal_controller_mode_exit_stops_the_motion(internal_arm, franky, log):
    internal_arm.set_control_mode(ControlMode.CARTESIAN_SERVO)
    internal_arm.send_action({"tcp_pose": np.array([0.3, -0.1, 0.5, *QUAT_Z90_XYZW])})
    log.clear()
    internal_arm.set_control_mode(ControlMode.IDLE)
    assert log[:2] == ["robot.stop", "join_motion"]
    assert not internal_arm._handle.streaming


def test_cartesian_controller_is_validated_and_read_from_the_station():
    with pytest.raises(ValueError, match="cartesian_controller"):
        FrankyArm(robot_ip="10.0.0.2", cartesian_controller="motion")
    with pytest.raises(ValueError, match="cartesian_controller"):
        FrankaConfig(limits={"joints": [[-4., 4.]] * 7}, cartesian_controller="motion")
    arm = FrankyArm.from_config(FrankaConfig(limits={"joints": [[-4., 4.]] * 7}, cartesian_controller="internal"))
    assert arm.cartesian_controller == "internal"


# --- cartesian_controller: serl ------------------------------------------------------------

from real_world_rl.devices.control import SERL_PARAM_DEFAULTS, SERL_PARAM_RANGES  # noqa: E402

HIL_SERL_DEFAULT_RESET = [0.0, 0.0, 0.0, -1.9, -0.0, 2.0, 0.0]


def make_fake_serl_controller(log: list[str]) -> types.ModuleType:
    """The parts of the serl_controller extension the backend uses, with its clamping semantics."""
    sc = types.ModuleType("serl_controller")
    sc.__version__ = "0.1.0"
    sc.SERL_COMMIT = "1f140ef"
    sc.WITH_FRANKY = True
    sc.JOINT_BLEND_DURATION_S = 10.0
    sc.FRANKA_ROS_LIMIT_RATE = True
    sc.FRANKA_ROS_CUTOFF_FREQUENCY = 100.0
    sc.BUILD_INFO = {"serl_controller_version": "0.1.0", "build_id": "fake-build", "with_franky": True}

    def clamp(params):
        return {k: min(max(float(v), SERL_PARAM_RANGES[k][0]), SERL_PARAM_RANGES[k][1]) for k, v in params.items()}

    class SerlCartesianImpedanceMotion:
        instances: list = []

        def __init__(self, params):
            log.append("SerlCartesianImpedanceMotion")
            self.params = {**SERL_PARAM_DEFAULTS, **clamp(params)}
            self.targets: list[tuple] = []
            self.updates: list[dict] = []
            SerlCartesianImpedanceMotion.instances.append(self)

        def set_target(self, position, quat_xyzw):
            log.append("SerlCartesianImpedanceMotion.set_target")
            self.targets.append((list(position), list(quat_xyzw)))
            return len(self.targets)

        def snapshot(self):
            return {"tick": 1}

        def update_params(self, updates):
            log.append("SerlCartesianImpedanceMotion.update_params")
            self.updates.append(dict(updates))
            self.params.update(clamp(updates))
            return dict(self.params)

    class SerlJointPositionMotion:
        instances: list = []

        def __init__(self, target, robot_model="panda"):
            log.append("SerlJointPositionMotion")
            self.robot_model = robot_model
            self.target = np.asarray(target, dtype=np.float64)
            self.elapsed_s = 0.0
            self.finish_requested = False
            SerlJointPositionMotion.instances.append(self)

        def finish(self):
            log.append("SerlJointPositionMotion.finish")
            self.finish_requested = True

        def snapshot(self):
            return {"elapsed_s": self.elapsed_s, "finished": self.finish_requested, "soft_limited_ticks": 3}

    sc.SerlCartesianImpedanceMotion = SerlCartesianImpedanceMotion
    sc.SerlJointPositionMotion = SerlJointPositionMotion
    return sc


@pytest.fixture
def serl_controller(monkeypatch, log, franky):
    fake = make_fake_serl_controller(log)
    monkeypatch.setitem(sys.modules, "serl_controller", fake)
    return fake


@pytest.fixture
def clock(monkeypatch):
    """Simulated time for the SERL joint reset: every sleep advances the running blend motion and,
    while ``converge`` is set, moves the measured joints along SERL's blend."""
    state = {"robot": None, "converge": True, "slept": 0.0}

    def fake_sleep(seconds):
        state["slept"] += seconds
        robot = state["robot"]
        if robot is None or not robot.moves:
            return
        motion = robot.moves[-1][0]
        if type(motion).__name__ != "SerlJointPositionMotion" or not robot.is_in_control:
            return
        start = state.setdefault(id(motion), np.asarray(robot.state.q, dtype=np.float64).copy())
        motion.elapsed_s += seconds
        if state["converge"]:
            t = min(motion.elapsed_s, 10.0)
            robot.state.q = (t / 10.0) * motion.target + ((10 - t) / 10.0) * start

    monkeypatch.setattr(franky_arm_module, "_sleep", fake_sleep)
    return state


@pytest.fixture
def serl_arm(franky, serl_controller, clock):
    arm = FrankyArm(robot_ip="10.0.0.2", realtime="enforce", cartesian_controller="serl",
                    serl={"translational_stiffness": 1500.0})
    arm.connect()
    clock["robot"] = arm._handle.robot
    yield arm
    arm.disconnect()


def test_serl_servo_starts_serl_motion_with_station_params_and_franka_ros_filters(serl_arm, serl_controller, log):
    serl_arm.set_control_mode(ControlMode.CARTESIAN_SERVO)
    robot = serl_arm._handle.robot
    motion, asynchronous = robot.moves[-1]
    assert isinstance(motion, serl_controller.SerlCartesianImpedanceMotion) and asynchronous
    assert robot.move_options[-1] == {"limit_rate": True, "cutoff_frequency": 100.0}
    assert motion.params == {**SERL_PARAM_DEFAULTS, "translational_stiffness": 1500.0}
    assert serl_arm._handle.tracker is None
    pose = np.array([0.31, -0.1, 0.5, *QUAT_Z90_XYZW])
    serl_arm.send_action({"tcp_pose": pose})
    assert motion.targets == [(pose[:3].tolist(), pose[3:].tolist())]  # xyz + quat (x, y, z, w)
    assert np.allclose(serl_arm.last_cartesian_target, pose)


def test_serl_mode_exit_stops_with_torque_stop_motion_and_joint_servo_stays_franky(serl_arm, franky, log):
    serl_arm.set_control_mode(ControlMode.CARTESIAN_SERVO)
    log.clear()
    serl_arm.set_control_mode(ControlMode.JOINT_SERVO)
    assert log[:4] == ["move:TorqueStopMotion", "join_motion", "join_motion", "recover_from_errors"]
    assert log[-2:] == ["JointImpedanceTracker", "move:JointImpedanceTracker"]
    assert serl_arm._handle.serl_motion is None
    serl_arm.set_control_mode(ControlMode.IDLE)
    assert serl_arm._handle.tracker is None and not serl_arm._handle.robot.is_in_control


def test_serl_reflex_is_a_recoverable_fault_refused_until_recover(serl_arm, franky, serl_controller):
    serl_arm._serl_auto_recover = {"enabled": False, "max_per_episode": 5}  # serl_auto_recover: {enabled: false}
    serl_arm.set_control_mode(ControlMode.CARTESIAN_SERVO)
    robot = serl_arm._handle.robot
    pose = np.array([0.3, -0.1, 0.5, *QUAT_Z90_XYZW])
    robot.is_in_control = False
    robot.pending_motion_error = franky.ControlException("libfranka: Move command aborted: cartesian_reflex")
    with pytest.raises(DeviceFault, match="cartesian_reflex") as info:
        serl_arm.send_action({"tcp_pose": pose})
    assert info.value.recoverable and not serl_arm.health().ok
    with pytest.raises(DeviceFault, match="unrecovered errors"):
        serl_arm.send_action({"tcp_pose": pose})
    serl_arm.recover()
    serl_arm.send_action({"tcp_pose": pose})  # a fresh SERL controller, like relaunching impedance.launch
    assert len(serl_controller.SerlCartesianImpedanceMotion.instances) == 2
    assert serl_controller.SerlCartesianImpedanceMotion.instances[-1].targets[-1][0] == pose[:3].tolist()


def _serl_reflex(robot, franky, text="libfranka: Move command aborted: cartesian_reflex"):
    robot.is_in_control = False
    robot.pending_motion_error = franky.ControlException(text)
    robot.state.robot_mode = franky.RobotMode.Reflex


def test_serl_reflex_auto_recovers_restarts_and_applies_the_target(serl_arm, franky, serl_controller, log, caplog):
    serl_arm.set_control_mode(ControlMode.CARTESIAN_SERVO)
    robot = serl_arm._handle.robot
    first = serl_arm._handle.serl_motion
    _serl_reflex(robot, franky)
    serl_arm.health()  # a health poll in between notes the robot's errors; auto-recovery clears them too
    log.clear()
    pose = np.array([0.31, -0.1, 0.5, *QUAT_Z90_XYZW])
    with caplog.at_level("WARNING"):
        serl_arm.send_action({"tcp_pose": pose})
    second = serl_arm._handle.serl_motion
    assert second is not first and isinstance(second, serl_controller.SerlCartesianImpedanceMotion)
    assert log == ["join_motion", "recover_from_errors", "SerlCartesianImpedanceMotion",
                   "move:SerlCartesianImpedanceMotion", "SerlCartesianImpedanceMotion.set_target"]
    assert robot.move_options[-1] == {"limit_rate": True, "cutoff_frequency": 100.0}
    assert second.targets == [(pose[:3].tolist(), pose[3:].tolist())] and first.targets == []
    assert "cartesian_reflex" in caplog.text and "auto-recovery 1/5" in caplog.text
    assert serl_arm.health().ok and serl_arm.control_mode is ControlMode.CARTESIAN_SERVO
    status = serl_arm.serl_auto_recovery
    assert (status["enabled"], status["max_per_episode"], status["episode_count"], status["total_count"]) == (True, 5, 1, 1)
    assert "cartesian_reflex" in status["last"]["error"] and len(status["last"]["tcp_pose"]) == 7
    assert serl_arm.control_metadata["serl"]["auto_recover"] == {"enabled": True, "max_per_episode": 5}


def test_serl_auto_recover_cap_faults_and_episode_resets(franky, serl_controller, clock):
    arm = FrankyArm(robot_ip="10.0.0.2", cartesian_controller="serl",
                    serl_auto_recover={"enabled": True, "max_per_episode": 2})
    arm.connect()
    arm.set_control_mode(ControlMode.CARTESIAN_SERVO)
    robot = arm._handle.robot
    pose = np.array([0.3, -0.1, 0.5, *QUAT_Z90_XYZW])
    for _ in range(2):
        _serl_reflex(robot, franky)
        arm.send_action({"tcp_pose": pose})
    assert arm.serl_auto_recovery["episode_count"] == 2
    _serl_reflex(robot, franky, "libfranka: communication_constraints_violation")
    with pytest.raises(DeviceFault, match="max_per_episode 2 reached") as info:
        arm.send_action({"tcp_pose": pose})
    assert info.value.recoverable and "communication_constraints_violation" in info.value.message
    with pytest.raises(DeviceFault, match="unrecovered errors"):
        arm.send_action({"tcp_pose": pose})
    arm.recover()  # an explicit recover does not start a new episode ...
    arm.send_action({"tcp_pose": pose})
    _serl_reflex(robot, franky)
    with pytest.raises(DeviceFault, match="max_per_episode"):
        arm.send_action({"tcp_pose": pose})
    arm.recover()
    arm.reset_serl_auto_recovery()  # ... the env reset does
    arm.send_action({"tcp_pose": pose})
    _serl_reflex(robot, franky)
    arm.send_action({"tcp_pose": pose})
    assert arm.serl_auto_recovery["episode_count"] == 1 and arm.serl_auto_recovery["total_count"] == 3
    arm.set_control_mode(ControlMode.IDLE)
    arm.set_control_mode(ControlMode.CARTESIAN_SERVO)  # and so does entering Cartesian servo
    assert arm.serl_auto_recovery["episode_count"] == 0
    arm.disconnect()


def test_serl_auto_recover_disabled_and_unrecoverable_stay_faults(franky, serl_controller, clock):
    arm = FrankyArm.from_config(FrankaConfig(limits={"joints": [[-4., 4.]] * 7}, cartesian_controller="serl", serl_auto_recover={"enabled": False}))
    assert arm.serl_auto_recovery["enabled"] is False and arm.serl_auto_recovery["max_per_episode"] == 5
    arm.connect()
    arm.set_control_mode(ControlMode.CARTESIAN_SERVO)
    pose = np.array([0.3, -0.1, 0.5, *QUAT_Z90_XYZW])
    _serl_reflex(arm._handle.robot, franky)
    with pytest.raises(DeviceFault, match="cartesian_reflex"):
        arm.send_action({"tcp_pose": pose})
    assert arm.serl_auto_recovery["total_count"] == 0
    arm.disconnect()
    arm = FrankyArm(robot_ip="10.0.0.2", cartesian_controller="serl")
    arm.connect()
    arm.set_control_mode(ControlMode.CARTESIAN_SERVO)
    robot = arm._handle.robot
    _serl_reflex(robot, franky)
    robot.recover_from_errors = lambda: False  # e.g. the external activation device is pressed
    with pytest.raises(DeviceFault, match="could not clear") as info:
        arm.send_action({"tcp_pose": pose})
    assert info.value.recoverable and arm.serl_auto_recovery["total_count"] == 0
    arm.disconnect()
    arm = FrankyArm(robot_ip="10.0.0.2", cartesian_controller="serl")
    arm.connect()
    arm.set_control_mode(ControlMode.CARTESIAN_SERVO)
    robot = arm._handle.robot
    robot.is_in_control = False  # ended by a stop from elsewhere, not by a reflex: not auto-recovered
    robot.pending_motion_error = franky.ControlException("libfranka: Move command preempted!")
    with pytest.raises(DeviceFault, match="preempted"):
        arm.send_action({"tcp_pose": pose})
    assert arm.serl_auto_recovery["total_count"] == 0
    arm.disconnect()


def test_serl_auto_recover_config_validation():
    from real_world_rl.devices.control import serl_auto_recover_config
    assert serl_auto_recover_config(None) == {"enabled": True, "max_per_episode": 5}
    assert FrankaConfig(limits={"joints": [[-4., 4.]] * 7}, serl_auto_recover={"max_per_episode": 0}).serl_auto_recover == {"enabled": True, "max_per_episode": 0}
    for bad, match in (({"cap": 1}, "unknown"), ({"enabled": "yes"}, "true or false"),
                       ({"max_per_episode": -1}, ">= 0"), ({"max_per_episode": 1.5}, ">= 0"), ([1], "mapping")):
        with pytest.raises(ValueError, match=match):
            FrankaConfig(limits={"joints": [[-4., 4.]] * 7}, serl_auto_recover=bad)


@pytest.mark.parametrize("controller", ["impedance", "internal"])
def test_auto_recover_is_serl_only(franky, controller):
    arm = FrankyArm(robot_ip="10.0.0.2", cartesian_controller=controller)
    assert arm.serl_auto_recovery is None


def test_serl_plain_stop_restarts_the_controller_on_the_next_target(serl_arm, serl_controller):
    serl_arm.set_control_mode(ControlMode.CARTESIAN_SERVO)
    serl_arm.stop()
    assert serl_arm._handle.serl_motion is None and serl_arm.last_cartesian_target is None
    serl_arm.send_action({"tcp_pose": np.array([0.3, -0.1, 0.5, *QUAT_Z90_XYZW])})
    assert len(serl_controller.SerlCartesianImpedanceMotion.instances) == 2


def test_serl_live_param_update_merges_clamps_and_persists(serl_arm, serl_controller):
    serl_arm.set_control_mode(ControlMode.CARTESIAN_SERVO)
    motion = serl_arm._handle.serl_motion
    effective = serl_arm.update_serl_params({"translational_clip_x": 0.0075, "rotational_clip_x": 0.5})
    assert motion.updates == [{"translational_clip_x": 0.0075, "rotational_clip_x": 0.1}]
    assert effective["translational_clip_x"] == 0.0075 and effective["rotational_clip_x"] == 0.1
    assert effective["translational_stiffness"] == 1500.0  # partial: the rest is kept
    assert serl_arm.serl_params == effective
    metadata = serl_arm.control_metadata["serl"]
    assert metadata["params"] == effective and metadata["station_params"]["rotational_clip_x"] == 0.05
    assert metadata["clamped"]["rotational_clip_x"] == {"requested": 0.5, "applied": 0.1}
    assert metadata["limit_rate"] is True and metadata["cutoff_frequency"] == 100.0
    assert serl_arm.control_metadata["cartesian_controller"] == "serl"
    with pytest.raises(ValueError, match="unknown SERL compliance"):
        serl_arm.update_serl_params({"stiffness": 1.0})
    with pytest.raises(ValueError, match="finite"):
        serl_arm.update_serl_params({"rotational_Ki": float("nan")})
    assert serl_arm.serl_params == effective
    serl_arm.set_control_mode(ControlMode.IDLE)
    serl_arm.update_serl_params({"rotational_stiffness": 250.0})  # stored for the next start
    serl_arm.set_control_mode(ControlMode.CARTESIAN_SERVO)
    restarted = serl_arm._handle.serl_motion
    assert restarted is not motion and restarted.params == {**effective, "rotational_stiffness": 250.0}


def test_update_serl_params_needs_serl_mode(arm):
    assert arm.serl_params is None
    with pytest.raises(DeviceFault, match="SERL"):
        arm.update_serl_params({"translational_stiffness": 1000.0})


def test_serl_joint_reset_blends_polls_finishes_and_recovers(serl_arm, serl_controller, log, clock):
    log.clear()
    serl_arm.move_to_joints(np.array(HIL_SERL_DEFAULT_RESET))
    robot = serl_arm._handle.robot
    motion, asynchronous = robot.moves[-1]
    assert isinstance(motion, serl_controller.SerlJointPositionMotion) and asynchronous
    assert robot.move_options[-1] == {"limit_rate": True, "cutoff_frequency": 100.0}
    np.testing.assert_array_equal(motion.target, HIL_SERL_DEFAULT_RESET)
    assert motion.robot_model == "panda"  # franka_hw's soft limits with franka_control's panda URDF (default)
    assert motion.finish_requested and motion.elapsed_s > 10.0  # ended in the hold phase
    assert log[-3:] == ["SerlJointPositionMotion.finish", "join_motion", "recover_from_errors"]
    report = serl_arm.last_joint_reset
    assert report["reached"] and report["max_abs_error_rad"] < 1e-2
    assert serl_arm._handle.joint_reset is None and serl_arm.control_mode is ControlMode.IDLE
    # the caller restarts the impedance controller, as franka_server does after jointreset
    serl_arm.set_control_mode(ControlMode.CARTESIAN_SERVO)
    assert type(robot.moves[-1][0]).__name__ == "SerlCartesianImpedanceMotion"


def test_serl_joint_reset_uses_the_station_robot_model(franky, serl_controller, clock):
    arm = FrankyArm.from_config(FrankaConfig(limits={"joints": [[-4., 4.]] * 7}, cartesian_controller="serl", robot_model="fr3"))
    arm.connect()
    clock["robot"] = arm._handle.robot
    arm.move_to_joints(np.array(HIL_SERL_DEFAULT_RESET))
    assert arm._handle.robot.moves[-1][0].robot_model == "fr3"
    assert arm.last_joint_reset["robot_model"] == "fr3" and arm.control_metadata["serl"]["robot_model"] == "fr3"
    with pytest.raises(ValueError, match="robot_model"):
        FrankaConfig(limits={"joints": [[-4., 4.]] * 7}, robot_model="fer")
    with pytest.raises(ValueError, match="robot_model"):
        FrankyArm(robot_ip="10.0.0.2", robot_model="FR3")
    arm.disconnect()


@pytest.mark.parametrize("model, server, controller, warning, text", [
    ("panda", 9, "impedance", True, "Franka Research 3"),
    ("fr3", 5, "serl", True, "Franka Emika Robot"),
    ("fr3", 9, "serl", False, "server version 9"),
    ("panda", 5, "impedance", False, "server version 5"),
    ("panda", None, "serl", True, "could not be cross-checked"),
    ("panda", None, "impedance", False, "unknown"),
])
def test_preflight_cross_checks_robot_model_with_server_version(franky, serl_controller, model, server, controller,
                                                                warning, text):
    arm = FrankyArm(robot_ip="10.0.0.2", robot_model=model, cartesian_controller=controller)
    arm.connect()
    if server is not None:
        arm._handle.robot.server_version = server
    check = {result.name: result for result in arm.preflight()}["mode:robot_model"]
    assert check.ok and check.warning is warning and text in check.message
    if warning:
        assert "Desk" in check.hint and "robot_model: panda | fr3" in check.hint
    arm.disconnect()


@pytest.mark.parametrize("model, urdf, warning, text", [
    ("fr3", '<robot name="fr3"><joint name="fr3_joint1"/></robot>', False, "robot URDF: fr3"),
    ("panda", '<robot name="fr3"><joint name="fr3_joint1"/></robot>', True, "URDF (Robot.model_urdf) is a fr3"),
    ("panda", '<robot><joint name="panda_joint1"/></robot>', False, "robot URDF: panda"),
    ("fr3", "<robot/>", True, "could not be cross-checked"),
])
def test_preflight_robot_model_from_urdf_when_franky_has_no_server_version(franky, serl_controller, model, urdf,
                                                                           warning, text):
    # franky 2.0 binds Robot.model_urdf but no Robot.server_version (only Gripper.server_version).
    arm = FrankyArm(robot_ip="10.0.0.2", robot_model=model, cartesian_controller="serl")
    arm.connect()
    assert not hasattr(arm._handle.robot, "server_version")
    arm._handle.robot.model_urdf = urdf
    check = {result.name: result for result in arm.preflight()}["mode:robot_model"]
    assert check.ok and check.warning is warning and text in check.message
    arm.disconnect()


def test_robot_model_from_urdf_needs_exactly_one_model():
    assert franky_arm_module.robot_model_from_urdf("x fr3_joint7 y") == "fr3"
    assert franky_arm_module.robot_model_from_urdf("panda_joint1") == "panda"
    assert franky_arm_module.robot_model_from_urdf("panda_joint1 fr3_joint1") is None
    assert franky_arm_module.robot_model_from_urdf(None) is None


def test_serl_joint_reset_timeout_continues_like_franka_server(serl_arm, clock, caplog):
    clock["converge"] = False  # the arm never gets there (blocked)
    serl_arm.move_to_joints(np.array(HIL_SERL_DEFAULT_RESET))
    report = serl_arm.last_joint_reset
    assert not report["reached"] and report["polls"] == franky_arm_module.SERL_JOINT_RESET_MAX_POLLS + 1
    assert "joint reset TIMEOUT" in caplog.text
    assert serl_arm._handle.robot.moves[-1][0].finish_requested


def test_serl_joint_reset_reflex_is_a_fault(serl_arm, franky, clock, monkeypatch):
    robot = serl_arm._handle.robot

    def reflex(seconds):
        robot.is_in_control = False
        robot.pending_motion_error = franky.ControlException("libfranka: joint_position_limits_violation")

    monkeypatch.setattr(franky_arm_module, "_sleep", reflex)
    with pytest.raises(DeviceFault, match="joint_position_limits_violation") as info:
        serl_arm.move_to_joints(np.array(HIL_SERL_DEFAULT_RESET))
    assert info.value.recoverable and serl_arm._handle.joint_reset is None


def test_serl_joint_reset_non_blocking_holds_until_stop(serl_arm, log):
    serl_arm.move_to_joints(np.array(HIL_SERL_DEFAULT_RESET), blocking=False)
    motion = serl_arm._handle.joint_reset
    assert motion is not None and not motion.finish_requested
    serl_arm.stop()
    assert serl_arm._handle.joint_reset is None and "robot.stop" in log


def test_serl_move_to_pose_stays_ruckig(serl_arm, franky):
    serl_arm.move_to_pose(np.array([0.3, -0.1, 0.5, *QUAT_Z90_XYZW]))
    assert isinstance(serl_arm._handle.robot.moves[-1][0], franky.CartesianMotion)


def test_serl_needs_the_serl_controller_package(franky, monkeypatch):
    monkeypatch.setitem(sys.modules, "serl_controller", None)
    arm = FrankyArm(robot_ip="10.0.0.2", cartesian_controller="serl")
    arm.connect()
    with pytest.raises(SdkMissing) as info:
        arm.set_control_mode(ControlMode.CARTESIAN_SERVO)
    assert "franky_control" in info.value.hint and "serl_controller" in info.value.hint
    assert "same build" in info.value.hint.lower()
    assert "error" in arm.control_metadata["serl"]


def test_serl_controller_that_does_not_load_is_a_version_mismatch(franky, monkeypatch):
    import importlib
    real_import = importlib.import_module

    def failing(name, *args, **kwargs):
        if name == "serl_controller":
            raise ImportError("libfranka-ed02dacf.so.0.17.0: cannot open shared object file")
        return real_import(name, *args, **kwargs)

    monkeypatch.setattr(franky_arm_module.importlib, "import_module", failing)
    with pytest.raises(VersionMismatch, match="libfranka-ed02dacf"):
        franky_arm_module.require_serl_controller("arm")


def test_serl_config_from_station(franky, serl_controller):
    config = FrankaConfig(limits={"joints": [[-4., 4.]] * 7}, cartesian_controller="serl", serl={"rotational_clip_x": 0.5, "translational_Ki": 1})
    assert config.serl == {"rotational_clip_x": 0.1, "translational_Ki": 1.0}  # clamped like dynamic_reconfigure
    arm = FrankyArm.from_config(config)
    assert arm.serl_params == {**SERL_PARAM_DEFAULTS, "rotational_clip_x": 0.1, "translational_Ki": 1.0}
    with pytest.raises(ValueError, match="unknown SERL"):
        FrankaConfig(limits={"joints": [[-4., 4.]] * 7}, cartesian_controller="serl", serl={"stiffness": 1.0})
    with pytest.raises(ValueError, match="arm_backend franky"):
        FrankaConfig(limits={"joints": [[-4., 4.]] * 7}, cartesian_controller="serl", arm_backend="serl_http")


def test_station_load_is_set_after_connect_in_every_mode(franky, log):
    load = {"mass_kg": 0.5, "center_of_mass_m": [0.0, 0.01, 0.05],
            "inertia": [[0.01, 0.001, 0.0], [0.001, 0.02, 0.0], [0.0, 0.0, 0.03]]}
    config = FrankaConfig(limits={"joints": [[-4., 4.]] * 7}, load=load)
    assert config.load["inertia"] == [0.01, 0.001, 0.0, 0.001, 0.02, 0.0, 0.0, 0.0, 0.03]
    for controller in ("impedance", "internal", "serl"):
        arm = FrankyArm.from_config(dataclasses.replace(config, cartesian_controller=controller))
        arm.connect()
        assert arm._handle.robot.loads == [(0.5, [0.0, 0.01, 0.05], config.load["inertia"])]
        assert arm.control_metadata["load"] == config.load
        arm.disconnect()


def test_station_load_validation_and_failure(franky):
    with pytest.raises(ValueError, match="symmetric"):
        FrankaConfig(limits={"joints": [[-4., 4.]] * 7}, load={"mass_kg": 1.0, "center_of_mass_m": [0, 0, 0], "inertia": [1, 2, 0, 0, 1, 0, 0, 0, 1]})
    with pytest.raises(ValueError, match="missing"):
        FrankaConfig(limits={"joints": [[-4., 4.]] * 7}, load={"mass_kg": 1.0})
    with pytest.raises(ValueError, match="mass_kg"):
        FrankaConfig(limits={"joints": [[-4., 4.]] * 7}, load={"mass_kg": -1.0, "center_of_mass_m": [0, 0, 0], "inertia": [0] * 9})
    arm = FrankyArm(robot_ip="10.0.0.2", load={"mass_kg": 1.0, "center_of_mass_m": [0, 0, 0], "inertia": [0] * 9})
    franky.Robot.set_load = lambda self, *args: (_ for _ in ()).throw(franky.CommandException("rejected"))
    with pytest.raises(DeviceFault, match="set_load"):
        arm.connect()


def test_serl_preflight_warns_without_enforced_realtime(franky, serl_controller, monkeypatch, tmp_path):
    monkeypatch.setattr(franky_arm_module, "REALTIME_KERNEL_FLAG_PATH", str(tmp_path / "none"))
    arm = FrankyArm(robot_ip="10.0.0.2", realtime="ignore", cartesian_controller="serl")
    arm.connect()
    check = {result.name: result for result in arm.preflight()}["mode:serl_realtime"]
    assert check.ok and check.warning and "realtime: enforce" in check.hint
    enforced = FrankyArm(robot_ip="10.0.0.2", realtime="enforce", cartesian_controller="serl")
    enforced.connect()
    check = {result.name: result for result in enforced.preflight()}["mode:serl_realtime"]
    assert check.ok and not check.warning
    plain = FrankyArm(robot_ip="10.0.0.2")
    plain.connect()
    assert "mode:serl_realtime" not in {result.name for result in plain.preflight()}


# --- serl_controller / franky build pairing (doctor) ---

def _fake_site(tmp_path, *, franky_version="2.0.0+libfranka.0.17.0", libfranky=b"libfranky-bytes",
               built_sha=None, libfranka="libfranka-ed02dacf.so.0.17.0", with_libfranka=True):
    import hashlib
    import json
    site = tmp_path / "site"
    (site / "serl_controller").mkdir(parents=True)
    (site / "franky").mkdir()
    (site / "franky_control.libs").mkdir()
    (site / "franky" / "libfranky.so").write_bytes(libfranky)
    if with_libfranka:
        (site / "franky_control.libs" / libfranka).write_bytes(b"x")
    info = {"serl_controller_version": "0.1.0", "with_franky": True, "franky_dist_version": "2.0.0+libfranka.0.17.0",
            "libfranky_sha256": built_sha or hashlib.sha256(b"libfranky-bytes").hexdigest(),
            "libfranka_file": libfranka, "build_id": "rwrl-franky-build-test"}
    (site / "serl_controller" / "build_info.json").write_text(json.dumps(info))
    return site, franky_version


@pytest.fixture
def fake_site(monkeypatch, tmp_path):
    def install(**kwargs):
        site, version = _fake_site(tmp_path, **kwargs)
        monkeypatch.setattr(franky_arm_module, "_package_dir",
                            lambda name: site / name if (site / name).is_dir() else None)
        monkeypatch.setattr(franky_arm_module.importlib.metadata, "version",
                            lambda dist: version if dist == "franky-control" else "0")
        return site
    return install


def test_pairing_ok_for_the_same_build(fake_site):
    fake_site()
    ok, message, hint, details = franky_arm_module.serl_build_pairing()
    assert ok and "same build" in message and "rwrl-franky-build-test" in message


@pytest.mark.parametrize("kwargs, needle", [
    ({"franky_version": "2.0.0"}, "built against franky_control 2.0.0+libfranka.0.17.0, installed is 2.0.0"),
    ({"libfranky": b"official-wheel"}, "not the wheel serl_controller was built against"),
    ({"with_libfranka": False}, "libfranka-ed02dacf.so.0.17.0"),
])
def test_pairing_fails_for_another_franky_wheel(fake_site, kwargs, needle):
    fake_site(**kwargs)
    ok, message, hint, _ = franky_arm_module.serl_build_pairing()
    assert not ok and needle in message
    assert "pip uninstall -y franky-control serl-controller" in hint and "same build" in hint.lower()


def _source_layout(site, tmp_path, *, libfranka_bytes=b"system-libfranka", recorded=None):
    """Turn the fake site into franky built from source (install-on-ubuntu.sh): no
    franky_control.libs, serl_controller records the system libfranka's path and digest."""
    import hashlib
    import json
    import shutil
    shutil.rmtree(site / "franky_control.libs")
    system = tmp_path / "usr-local-lib" / "libfranka.so.0.17.0"
    system.parent.mkdir()
    system.write_bytes(libfranka_bytes)
    info_path = site / "serl_controller" / "build_info.json"
    info = json.loads(info_path.read_text())
    info.update(libfranka_file="", franky_layout="source", libfranka_path=str(system),
                libfranka_sha256=recorded or hashlib.sha256(b"system-libfranka").hexdigest())
    info_path.write_text(json.dumps(info))
    return system


def test_pairing_ok_for_a_source_built_franky(fake_site, tmp_path):
    site = fake_site()
    system = _source_layout(site, tmp_path)
    ok, message, hint, details = franky_arm_module.serl_build_pairing()
    assert ok and "same build" in message and f"source build, {system}" in message, message


@pytest.mark.parametrize("change, needle", [
    ("remove", "is gone"),
    ("rebuild", "changed since serl_controller was built"),
])
def test_pairing_fails_when_the_system_libfranka_changed(fake_site, tmp_path, change, needle):
    site = fake_site()
    system = _source_layout(site, tmp_path)
    if change == "remove":
        system.unlink()
    else:
        system.write_bytes(b"another libfranka")
    ok, message, hint, _ = franky_arm_module.serl_build_pairing()
    assert not ok and needle in message and "install-on-ubuntu.sh" in hint


def test_pairing_fails_for_an_official_wheel_over_a_source_build(fake_site, tmp_path):
    site = fake_site(libfranky=b"official-wheel")
    _source_layout(site, tmp_path)
    ok, message, _, _ = franky_arm_module.serl_build_pairing()
    assert not ok and "not the wheel serl_controller was built against" in message


def test_pairing_fails_without_serl_controller(monkeypatch):
    monkeypatch.setattr(franky_arm_module, "_package_dir", lambda name: None)
    ok, message, _, _ = franky_arm_module.serl_build_pairing()
    assert not ok and "not installed" in message


def test_doctor_sdk_layer_checks_the_pairing_only_in_serl_mode(fake_site):
    from real_world_rl.doctor.checks import _sdk_layer
    fake_site(franky_version="2.0.0")
    checks, _ = _sdk_layer(FrankaConfig(limits={"joints": [[-4., 4.]] * 7}, cartesian_controller="serl"))
    results = {result.name: result for result in (check() for check in checks)}
    serl = results["sdk:serl_controller"]
    assert serl.layer == 1 and not serl.ok and "installed is 2.0.0" in serl.message and serl.hint
    checks, _ = _sdk_layer(FrankaConfig(limits={"joints": [[-4., 4.]] * 7}))
    assert "sdk:serl_controller" not in {check().name for check in checks}


# --- live SERL parameters over the daemon ---

def test_serl_params_over_grpc(franky, serl_controller, clock, tmp_path, monkeypatch):
    pytest.importorskip("grpc")
    import real_world_rl.devices.claims as claims
    from real_world_rl.devices import Robot
    from real_world_rl.devices.robots.franka import FrankaRobot
    from real_world_rl.server.service import RobotService
    from real_world_rl.server.transport import GrpcTransport

    monkeypatch.setattr(claims, "_DEFAULT_LOCK_DIR", tmp_path / "claims")
    robot = FrankaRobot(FrankaConfig(limits={"joints": [[-4., 4.]] * 7}, cartesian_controller="serl", gripper=None, claim=False,
                                     serl={"translational_stiffness": 1800.0}))
    service = RobotService(robot, watchdog_s=5.0)
    transport = GrpcTransport()
    transport.start(service, "127.0.0.1:0")
    remote = Robot.from_dict({"type": "remote", "address": f"127.0.0.1:{transport.port}", "timeout_s": 2.0})
    try:
        remote.connect()
        arm = remote.arm
        assert arm.serl_params["translational_stiffness"] == 1800.0
        assert arm.control_metadata["serl"]["station_params"]["translational_stiffness"] == 1800.0
        arm.set_control_mode(ControlMode.CARTESIAN_SERVO)
        effective = arm.update_serl_params({"rotational_stiffness": 250, "rotational_clip_z": 0.5})
        assert effective["rotational_stiffness"] == 250.0 and effective["rotational_clip_z"] == 0.1
        motion = robot.arm._handle.serl_motion
        assert motion.updates == [{"rotational_stiffness": 250.0, "rotational_clip_z": 0.1}]
        assert robot.arm.control_metadata["serl"]["clamped"]["rotational_clip_z"]["requested"] == 0.5
        assert arm.serl_params == effective
        with pytest.raises(ValueError):
            arm.update_serl_params({"nope": 1.0})  # refused client-side, nothing sent
        assert len(motion.updates) == 1
        arm.set_control_mode(ControlMode.IDLE)
        clock["robot"] = robot.arm._handle.robot
        arm.move_to_joints(np.array(HIL_SERL_DEFAULT_RESET))  # SERL joint reset through the daemon
        assert robot.arm.last_joint_reset["reached"]
        report = arm.last_joint_reset  # over gRPC
        assert report == robot.arm.last_joint_reset and report["soft_limited_ticks"] == 3
        assert report["robot_model"] == "panda"
        # serl_auto_recover over the daemon: a reflex is cleared on the next target, counted, exposed
        assert arm.serl_auto_recovery == robot.arm.serl_auto_recovery and arm.serl_auto_recovery["total_count"] == 0
        arm.set_control_mode(ControlMode.CARTESIAN_SERVO)
        _serl_reflex(robot.arm._handle.robot, franky)
        pose = np.array([0.3, -0.1, 0.5, *QUAT_Z90_XYZW])
        arm.send_action({"tcp_pose": pose})
        assert robot.arm._handle.serl_motion.targets[-1][0] == pose[:3].tolist()
        status = arm.serl_auto_recovery
        assert status["episode_count"] == 1 and "cartesian_reflex" in status["last"]["error"]
        arm.reset_serl_auto_recovery()
        assert robot.arm.serl_auto_recovery["episode_count"] == 0 and arm.serl_auto_recovery["total_count"] == 1
    finally:
        remote.disconnect()
        transport.stop(grace_s=0.1)
        service.close()


def test_only_the_serl_joint_reset_gets_a_long_grpc_deadline(monkeypatch):
    pytest.importorskip("grpc")
    from real_world_rl.server.remote import SERL_JOINT_RESET_DEADLINE_S, RemoteArm
    from real_world_rl.server.transport.grpc_transport import GrpcClient

    deadlines = []

    class Owner:
        is_connected = True

    client = GrpcClient("127.0.0.1:1", timeout_s=5.0)
    monkeypatch.setattr(client, "_invoke", lambda method, kwargs, timeout_s=None: (
        deadlines.append((method, timeout_s)), {"result": None})[1])

    def remote(controller):
        arm = RemoteArm("arm", num_joints=7, supported_state_fields=("joint_pos",), client=client, owner=Owner(),
                        control_metadata={"cartesian_controller": controller, "capabilities": [], "effective": {}})
        monkeypatch.setattr(arm, "_require_connected", lambda: None)
        return arm

    q = np.zeros(7)
    remote("serl")._move_to_joints(q, blocking=True)
    remote("serl")._move_to_joints(q, blocking=False)
    remote("impedance")._move_to_joints(q, blocking=True)
    remote("serl")._move_to_pose(np.array([0.3, 0, 0.5, 0, 0, 0, 1.0]), blocking=True)
    assert deadlines == [("arm.move_to_joints", SERL_JOINT_RESET_DEADLINE_S), ("arm.move_to_joints", 5.0),
                         ("arm.move_to_joints", 5.0), ("arm.move_to_pose", 5.0)]


def test_serl_mode_warns_about_ignored_cartesian_control_keys(caplog):
    from real_world_rl.doctor.checks import _sdk_layer
    from real_world_rl.devices.robots.franka import serl_ignored_control_keys
    control = {"translational_stiffness": 2000.0, "rotational_error_clip": [0.05] * 3, "max_delta_tau": 0.5,
               "joint_stiffness": [100.0] * 7}
    with caplog.at_level("WARNING"):
        config = FrankaConfig(limits={"joints": [[-4., 4.]] * 7}, cartesian_controller="serl", control=control)
    assert "ignores control: ['translational_stiffness', 'rotational_error_clip']" in caplog.text
    assert serl_ignored_control_keys(config) == ["translational_stiffness", "rotational_error_clip"]
    assert serl_ignored_control_keys(FrankaConfig(limits={"joints": [[-4., 4.]] * 7}, control=control)) == []
    checks, _ = _sdk_layer(config)
    result = {r.name: r for r in (check() for check in checks)}["config:serl_control"]
    assert result.ok and result.warning and "translational_stiffness" in result.message
    clean, _ = _sdk_layer(FrankaConfig(limits={"joints": [[-4., 4.]] * 7}, cartesian_controller="serl", control={"max_delta_tau": 0.5}))
    result = {r.name: r for r in (check() for check in clean)}["config:serl_control"]
    assert result.ok and not result.warning


@pytest.mark.parametrize('offset,valid', [(0., True), (0.00009, True), (.001, False)])
def test_connect_checks_configured_tcp_transform(franky, offset, valid):
    from real_world_rl.devices.robots.franka_kinematics import FLANGE_T_HAND
    tool = FLANGE_T_HAND.copy()
    tool[2, 3] += offset
    arm = FrankyArm(robot_ip='x', flange_T_ee=tool)
    if valid:
        arm.connect()
        arm.disconnect()
    else:
        with pytest.raises(DeviceFault, match='F_T_EE') as error:
            arm.connect()
        assert 'flange_T_ee' in error.value.hint and 'rebase' in error.value.hint
        assert not arm.is_connected


def test_tcp_transform_missing_or_invalid_fails_closed():
    arm = FrankyArm(robot_ip='x')
    for state in (types.SimpleNamespace(), types.SimpleNamespace(F_T_EE=np.full((4, 4), np.nan))):
        with pytest.raises(DeviceFault, match='F_T_EE'):
            arm._check_tcp_transform(types.SimpleNamespace(state=state))


def test_tcp_transform_accepts_column_major_sdk_array():
    from real_world_rl.devices.robots.franka_kinematics import FLANGE_T_HAND
    arm = FrankyArm(robot_ip='x')
    arm._check_tcp_transform(types.SimpleNamespace(state=types.SimpleNamespace(F_T_EE=FLANGE_T_HAND.flatten(order='F'))))
    q = np.zeros(7)
    np.testing.assert_allclose(arm.tcp_position_from_joints(q), [.088, 0, .8226], atol=1e-12)
