"""Native joint trajectory integration using only the published Python API."""
import copy
import sys
import types

import numpy as np
import pytest

from test_franky_backend import (
    franky, log, clock, serl_controller, _serl_reflex,
)
from real_world_rl.devices import ControlMode, DeviceFault, SdkMissing
from real_world_rl.devices.backends.arms.franky import FrankyArm, JOINT_CONTROLLERS
from real_world_rl.devices.robots.franka import FrankaConfig, DATASHEET_JOINT_LIMITS


def make_fake_joint_controller():
    module = types.ModuleType("joint_controller")

    def default_params(robot_model):
        limits = DATASHEET_JOINT_LIMITS[robot_model]
        return dict(stiffness=[600.] * 7, damping=[30.] * 7, velocity_filter_alpha=0.1,
                    max_velocity=[0.6] * 4 + [0.75] * 3, max_acceleration=[2.] * 7,
                    min_segment_duration_s=0.1, lower_limit=[p[0] for p in limits],
                    upper_limit=[p[1] for p in limits], limit_margin=0.05,
                    command_timeout_s=0.5, max_delta_tau=1., compensate_coriolis=True)

    class TrajectoryImpedanceMotion:
        def __init__(self, params):
            self.values = copy.deepcopy(params)
            self.targets = []
            self.updates = []
            self.reject = False

        def params(self):
            return copy.deepcopy(self.values)

        def set_target(self, q):
            if self.reject or not np.isfinite(q).all():
                return False
            self.targets.append(np.array(q))
            return True

        def update_params(self, updates):
            self.updates.append(copy.deepcopy(updates))
            self.values.update(copy.deepcopy(updates))
            return self.params()

        def telemetry(self):
            return {"ticks": 1}

        def request_stop(self):
            raise AssertionError("request_stop holds position; use franky torque stop to end motion")

    module.default_params = default_params
    module.TrajectoryImpedanceMotion = TrajectoryImpedanceMotion
    module.build_info = lambda: {"franky_dist_version": "1.2.3"}
    return module


@pytest.fixture
def joint_controller(monkeypatch):
    module = make_fake_joint_controller()
    monkeypatch.setitem(sys.modules, "joint_controller", module)
    return module


@pytest.fixture
def arm(franky, joint_controller, clock):
    arm = FrankyArm(robot_ip="10.0.0.2", joint_controller="trajectory_impedance")
    arm.connect()
    yield arm
    arm.disconnect()


def test_validation_and_defaults():
    assert FrankaConfig(limits={"joints": [[-4., 4.]] * 7}).joint_controller == "impedance"
    assert FrankyArm(robot_ip="x").joint_trajectory_params is None
    for cls, kw in ((FrankaConfig, {"limits": {"joints": [[-4., 4.]] * 7}}), (FrankyArm, {"robot_ip": "x"})):
        with pytest.raises(ValueError, match="trajectory_impedance"):
            cls(joint_controller="invalid", **kw)
        with pytest.raises(ValueError, match="unknown joint_trajectory"):
            cls(joint_trajectory={"typo": 1}, **kw)


@pytest.mark.parametrize("ending", ["idle", "stop", "disconnect"])
def test_lifecycle(arm, franky, joint_controller, ending):
    arm.set_control_mode(ControlMode.JOINT_SERVO)
    robot = arm._handle.robot
    motion = arm._handle.joint_trajectory_motion
    assert isinstance(motion, joint_controller.TrajectoryImpedanceMotion)
    assert robot.moves[-1] == (motion, True)
    assert not motion.targets  # native starting() seeds the measured joints
    q = arm.read_state()["joint_pos"]
    arm.servo_joints(q)
    np.testing.assert_array_equal(motion.targets[-1], q)
    if ending == "idle":
        arm.set_control_mode(ControlMode.IDLE)
    elif ending == "stop":
        arm.stop()
        arm.servo_joints(q)
        assert arm._handle.joint_trajectory_motion is not motion
        arm.stop()
    else:
        arm.disconnect()
    assert isinstance(robot.moves[-1][0], franky.TorqueStopMotion)
    assert not robot.is_in_control


def test_rejected_target(arm):
    arm.set_control_mode(ControlMode.JOINT_SERVO)
    arm._handle.joint_trajectory_motion.reject = True
    with pytest.raises(DeviceFault, match=r"joint target rejected \(non-finite\)"):
        arm.servo_joints(np.zeros(7))


def test_reflex_and_cap(arm, franky, caplog):
    arm._serl_auto_recover["max_per_episode"] = 1
    arm.set_control_mode(ControlMode.JOINT_SERVO)
    old = arm._handle.joint_trajectory_motion
    _serl_reflex(arm._handle.robot, franky)
    arm.servo_joints(np.zeros(7))
    assert arm._handle.joint_trajectory_motion is not old
    assert len(arm._handle.joint_trajectory_motion.targets) == 1
    assert arm.serl_auto_recovery["episode_count"] == 1
    assert "auto-recovery 1/1" in caplog.text
    _serl_reflex(arm._handle.robot, franky)
    with pytest.raises(DeviceFault, match="max_per_episode"):
        arm.servo_joints(np.zeros(7))


@pytest.mark.parametrize("model", ["panda", "fr3"])
def test_params(arm, joint_controller, model):
    arm.robot_model = model
    expected = joint_controller.default_params(model)
    assert arm.joint_trajectory_params == expected
    arm.update_joint_trajectory_params({"max_delta_tau": 2.})
    arm.set_control_mode(ControlMode.JOINT_SERVO)
    motion = arm._handle.joint_trajectory_motion
    assert motion.params()["max_delta_tau"] == 2.
    effective = arm.update_joint_trajectory_params({"stiffness": [100.] * 7})
    assert motion.updates == [{"stiffness": [100.] * 7}]
    assert effective == arm.joint_trajectory_params
    effective["stiffness"][0] = 0
    assert arm.joint_trajectory_params["stiffness"][0] == 100.
    with pytest.raises(ValueError, match="unknown"):
        arm.update_joint_trajectory_params({"nope": 1})
    configured = FrankyArm.from_config(FrankaConfig(limits={"joints": DATASHEET_JOINT_LIMITS[model]}, joint_controller="trajectory_impedance",
                             robot_model=model, joint_trajectory={"max_delta_tau": 3.}))
    assert configured.joint_trajectory_params == {**expected, "max_delta_tau": 3.}


@pytest.mark.parametrize("cartesian", ["impedance", "internal", "serl"])
@pytest.mark.parametrize("joint", JOINT_CONTROLLERS)
def test_mode_matrix(franky, joint_controller, serl_controller, clock, cartesian, joint):
    arm = FrankyArm(robot_ip="x", cartesian_controller=cartesian, joint_controller=joint)
    arm.connect()
    try:
        arm.set_control_mode(ControlMode.CARTESIAN_SERVO)
        assert arm._handle.joint_trajectory_motion is None
        assert (arm._handle.serl_motion is not None) == (cartesian == "serl")
        assert (arm._handle.tracker is not None) == (cartesian == "impedance")
        arm.set_control_mode(ControlMode.JOINT_SERVO)
        assert (arm._handle.joint_trajectory_motion is not None) == (joint == "trajectory_impedance")
        assert (arm._handle.tracker is not None) == (joint == "impedance")
        arm.servo_joints(np.zeros(7))
        arm.set_control_mode(ControlMode.CARTESIAN_SERVO)
        assert arm._handle.joint_trajectory_motion is None
    finally:
        arm.disconnect()


def test_missing_package(franky, monkeypatch):
    monkeypatch.setitem(sys.modules, "joint_controller", None)
    arm = FrankyArm(robot_ip="x", joint_controller="trajectory_impedance")
    with pytest.raises(SdkMissing, match="joint_controller") as error:
        _ = arm.joint_trajectory_params
    assert "native/joint_controller" in error.value.hint


@pytest.mark.parametrize("version,ok", [("1.2.3", True), ("2.0", False)])
def test_doctor_pairing(joint_controller, monkeypatch, version, ok):
    from real_world_rl.doctor.deps import joint_controller_pairing
    monkeypatch.setattr("importlib.metadata.version", lambda name: version)
    assert joint_controller_pairing()[0] is ok


def test_doctor_missing(monkeypatch):
    from real_world_rl.doctor.deps import joint_controller_pairing
    monkeypatch.setitem(sys.modules, "joint_controller", None)
    assert joint_controller_pairing()[0] is False


def test_realtime(arm, monkeypatch):
    monkeypatch.setattr("real_world_rl.devices.backends.arms.franky._read_realtime_flag", lambda: "0")
    results = {r.name: r for r in arm.preflight()}
    assert results["mode:joint_trajectory_realtime"].warning
    arm.realtime = "enforce"
    results = {r.name: r for r in arm.preflight()}
    assert not results["mode:realtime_kernel"].ok


def test_params_grpc(franky, joint_controller, clock):
    pytest.importorskip("grpc")
    from real_world_rl.devices import Robot
    from real_world_rl.devices.robots.franka import FrankaRobot
    from real_world_rl.server.service import RobotService
    from real_world_rl.server.transport import GrpcTransport
    robot = FrankaRobot(FrankaConfig(limits={"joints": [[-4., 4.]] * 7}, joint_controller="trajectory_impedance", gripper=None, claim=False))
    service = RobotService(robot, watchdog_s=5.)
    transport = GrpcTransport()
    transport.start(service, "127.0.0.1:0")
    remote = Robot.from_dict({"type": "remote", "address": f"127.0.0.1:{transport.port}"})
    try:
        remote.connect()
        remote.arm.set_control_mode(ControlMode.JOINT_SERVO)
        result = remote.arm.update_joint_trajectory_params({"stiffness": [123.] * 7})
        assert remote.arm.joint_trajectory_params == result
        assert robot.arm._handle.joint_trajectory_motion.params() == result
    finally:
        remote.disconnect()
        transport.stop(grace_s=0.1)
        service.close()


def test_rpc_dispatch_without_socket(arm, monkeypatch):
    from real_world_rl.devices.robots.franka import FrankaRobot
    from real_world_rl.server.service import RobotService
    from real_world_rl.server.remote import RemoteArm
    from real_world_rl.server.transport.base import is_read_only
    robot = FrankaRobot(FrankaConfig(limits={"joints": [[-4., 4.]] * 7}, arm_backend="fake", gripper=None, claim=False))
    robot.children["arm"] = arm
    service = RobotService(robot, watchdog_s=5.)
    remote = object.__new__(RemoteArm)
    monkeypatch.setattr(remote, "_call", lambda method, **kwargs: service.dispatch("arm." + method, kwargs))
    try:
        arm.set_control_mode(ControlMode.JOINT_SERVO)
        effective = remote.update_joint_trajectory_params({"max_velocity": [0.3] * 7, "compensate_coriolis": False})
        assert remote.joint_trajectory_params == effective
        assert arm._handle.joint_trajectory_motion.params() == effective
        assert is_read_only("arm.joint_trajectory_params")
        assert not is_read_only("arm.update_joint_trajectory_params")
    finally:
        service.close()


@pytest.mark.parametrize("enabled,recovers", [(False, True), (True, False)])
def test_reflex_disabled_or_recovery_failure(arm, franky, monkeypatch, enabled, recovers):
    arm._serl_auto_recover["enabled"] = enabled
    arm.set_control_mode(ControlMode.JOINT_SERVO)
    _serl_reflex(arm._handle.robot, franky)
    monkeypatch.setattr(arm._handle.robot, "recover_from_errors", lambda: recovers)
    with pytest.raises(DeviceFault):
        arm.servo_joints(np.zeros(7))
    assert arm.serl_auto_recovery["total_count"] == 0


def test_doctor_checks_enabled_only_for_trajectory(franky, joint_controller, monkeypatch):
    from real_world_rl.doctor.checks import _sdk_layer
    monkeypatch.setattr("importlib.metadata.version", lambda name: "1.2.3")
    for controller in JOINT_CONTROLLERS:
        checks, _ = _sdk_layer(FrankaConfig(limits={"joints": [[-4., 4.]] * 7}, joint_controller=controller, gripper=None))
        results = {r.name: r for r in (check() for check in checks)}
        assert ("sdk:joint_controller" in results) == (controller == "trajectory_impedance")
        if controller == "trajectory_impedance":
            assert results["sdk:joint_controller"].ok
    joint_controller.build_info = lambda: {}
    checks, _ = _sdk_layer(FrankaConfig(limits={"joints": [[-4., 4.]] * 7}, joint_controller="trajectory_impedance", gripper=None))
    assert not next(r for r in (check() for check in checks) if r.name == "sdk:joint_controller").ok


def test_homing_unchanged(arm, franky):
    arm.move_to_joints(np.zeros(7))
    assert isinstance(arm._handle.robot.moves[-1][0], franky.JointMotion)


def test_native_rejects_nonfinite_direct_command(arm):
    arm.set_control_mode(ControlMode.JOINT_SERVO)
    with pytest.raises(DeviceFault, match="joint target rejected"):
        arm._servo_joints(np.full(7, np.nan))


def test_station_joint_trajectory_bounds_and_live_updates(franky, joint_controller):
    from real_world_rl.devices.limits import ArmLimits
    from real_world_rl.devices.joint_trajectory import station_joint_trajectory
    bounds = [[-1., 1.]] * 7
    assert station_joint_trajectory({}, bounds) == {'lower_limit': [-1.]*7, 'upper_limit': [1.]*7}
    arm = FrankyArm(robot_ip='x', joint_controller='trajectory_impedance', safety=ArmLimits(joints=bounds))
    assert arm.joint_trajectory_params['lower_limit'] == [-1.]*7
    assert arm.joint_trajectory_params['upper_limit'] == [1.]*7
    for updates in ({'lower_limit': [-2.]*7}, {'upper_limit': [2.]*7},
                    {'lower_limit': [np.nan]*7}, {'upper_limit': [0.]*6},
                    {'lower_limit': [.5]*7, 'upper_limit': [-.5]*7}):
        with pytest.raises(ValueError):
            station_joint_trajectory(updates, bounds)
        with pytest.raises(ValueError):
            arm.update_joint_trajectory_params(updates)
    arm.connect()
    try:
        arm.set_control_mode(ControlMode.JOINT_SERVO)
        motion = arm._handle.joint_trajectory_motion
        with pytest.raises(ValueError, match='tighten'):
            arm.update_joint_trajectory_params({'upper_limit': [2.]*7})
        assert motion.updates == []
        assert arm.update_joint_trajectory_params({'upper_limit': [.8]*7})['upper_limit'] == [.8]*7
    finally:
        arm.disconnect()
