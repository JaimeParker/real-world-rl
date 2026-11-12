"""Control configuration is typed, IDLE-only, RPC-visible and SDK-checked."""
import subprocess
import sys

import numpy as np
import pytest

from real_world_rl.devices.control import ArmControlConfig
from real_world_rl.devices.arm import ControlMode
from real_world_rl.devices.errors import DeviceFault, VersionMismatch
from real_world_rl.devices.robots.franka import FrankaConfig, FrankaRobot
from real_world_rl.server.remote import RemoteRobot
from real_world_rl.server.service import RobotService
from real_world_rl.server.transport.local import LocalClient


def test_control_validation_and_roundtrip():
    config = ArmControlConfig.from_dict({"translational_stiffness": 500, "cartesian_damping": [10] * 6, "max_delta_tau": .5})
    assert ArmControlConfig.from_dict(config.to_dict()) == config
    assert config.for_mode("cartesian_servo")["damping"] == [10] * 6
    assert config.for_mode("joint_servo") == {"max_delta_tau": .5}
    for invalid in ({"unknown": 1}, {"max_delta_tau": 0}, {"cartesian_damping": [1]}, {"joint_stiffness": [float("nan")]}, {"rotational_stiffness": True}):
        with pytest.raises(ValueError): ArmControlConfig.from_dict(invalid)


def test_station_control_and_remote_idle_configuration():
    robot = FrankaRobot(FrankaConfig(arm_backend="fake", gripper=None, claim=False,
                                    control={"translational_stiffness": 300}))
    service = RobotService(robot, claim=False)
    remote = RemoteRobot.from_client(LocalClient(service))
    try:
        remote.connect()
        assert remote.arm.control_config == {"translational_stiffness": 300}
        effective = remote.arm.configure_control({"max_delta_tau": .4})
        assert effective == {"translational_stiffness": 300, "max_delta_tau": .4}
        assert remote.arm.control_metadata["effective"] == effective
        assert "max_delta_tau" in remote.arm.control_metadata["capabilities"]
        remote.arm.set_control_mode(ControlMode.CARTESIAN_SERVO)
        with pytest.raises(DeviceFault, match="IDLE"):
            remote.arm.configure_control({"max_delta_tau": .2})
    finally:
        remote.disconnect()
        service.close()


def test_unsupported_backend_refuses_nonempty_but_empty_is_compatible():
    from real_world_rl.devices.backends.arms.fake import FakeArm
    class UnsupportedArm(FakeArm):
        CONTROL_CAPABILITIES = ()
    arm = UnsupportedArm()
    assert arm.configure_control({}) == {}
    with pytest.raises(ValueError, match="does not support"):
        arm.configure_control({"max_delta_tau": 1})


def test_franky_kwargs_reach_actual_tracker_and_old_sdk_is_rejected(monkeypatch):
    from test_franky_backend import make_fake_franky
    from real_world_rl.devices.backends.arms.franky import FrankyArm
    log = []
    franky = make_fake_franky(log)
    captured = []
    original = franky.CartesianImpedanceTracker
    class ConfiguredTracker(original):
        def __init__(self, robot, *, translational_stiffness=None, damping=None,
                     translational_error_clip=None, max_delta_tau=1):
            captured.append(dict(stiffness=translational_stiffness, damping=damping,
                                 error_clip=translational_error_clip, max_delta_tau=max_delta_tau))
            super().__init__(robot)
    franky.CartesianImpedanceTracker = ConfiguredTracker
    # Common tau is also supported by the joint tracker in the installed API.
    joint = franky.JointImpedanceTracker
    class JointTracker(joint):
        def __init__(self, robot, *, max_delta_tau=1): super().__init__(robot)
    franky.JointImpedanceTracker = JointTracker
    monkeypatch.setitem(sys.modules, "franky", franky)
    arm = FrankyArm(robot_ip="test")
    arm.connect()
    try:
        arm.configure_control({"translational_stiffness": 500, "cartesian_damping": [10] * 6,
                               "translational_error_clip": [.02] * 3, "max_delta_tau": .5})
        arm.set_control_mode(ControlMode.CARTESIAN_SERVO)
        assert captured[0]["stiffness"] == 500
        np.testing.assert_array_equal(captured[0]["damping"], [10] * 6)
        assert captured[0]["max_delta_tau"] == .5
        arm.set_control_mode(ControlMode.IDLE)
        with pytest.raises(VersionMismatch, match="does not support"):
            arm.configure_control({"gains_time_constant": .1})
    finally:
        arm.disconnect()


def test_control_import_is_torch_free():
    result = subprocess.run([sys.executable, "-c", "import sys; import real_world_rl.devices.control; import real_world_rl.server.service; assert 'torch' not in sys.modules; assert 'rl_garden' not in sys.modules"], capture_output=True, text=True)
    assert result.returncode == 0, result.stderr


def test_station_renderer_preserves_control_defaults():
    from real_world_rl.setup.render import render_station_yaml
    from real_world_rl.setup.wizard import ArmDraft, StationDraft
    import yaml
    profile = {"translational_stiffness": 400, "max_delta_tau": .5}
    config = FrankaConfig(control=profile)
    rendered = yaml.safe_load(render_station_yaml(StationDraft([ArmDraft("robot", config)])))
    assert rendered["control"] == profile
    assert FrankaConfig.from_dict(rendered).control == profile


def test_experiment_can_only_tighten_station_torque_and_error_limits():
    from real_world_rl.devices.errors import SafetyViolation
    robot = FrankaRobot(FrankaConfig(arm_backend="fake", gripper=None, claim=False,
                      control={"max_delta_tau": .5, "translational_error_clip": [.02] * 3}))
    robot.connect()
    try:
        old = robot.arm.configure_control({"max_delta_tau": .3})
        with pytest.raises(SafetyViolation):
            robot.arm.configure_control({"max_delta_tau": .7})
        assert robot.arm.control_config == old
        with pytest.raises(SafetyViolation):
            robot.arm.configure_control({"translational_error_clip": [.02, .03, .01]})
        assert robot.arm.control_config == old
    finally:
        robot.disconnect()


def test_wizard_optional_control_input_roundtrips_and_from_preserves():
    import yaml
    from real_world_rl.setup.command import build_parser, answers_from_args
    from real_world_rl.setup.probes import mock_station_probe
    from real_world_rl.setup.wizard import ArmDraft, StationDraft, run_wizard, WizardError
    from real_world_rl.setup.render import render_station_yaml
    previous = StationDraft([ArmDraft("arm", FrankaConfig(name="arm", cameras=[], teleop=None))])
    def never(prompt): pytest.fail("optional control input added a question: " + prompt)
    parsed = build_parser().parse_args(["--non-interactive", "--only", "daemon", "--control", "{max_delta_tau: 0.4}"])
    draft = run_wizard(mock_station_probe(), answers_from_args(parsed, previous), never, lambda _: None)
    assert draft.arms[0].config.control == {"max_delta_tau": .4}
    assert yaml.safe_load(render_station_yaml(draft))["control"] == {"max_delta_tau": .4}
    parsed = build_parser().parse_args(["--non-interactive", "--only", "daemon"])
    restored = run_wizard(mock_station_probe(), answers_from_args(parsed, draft), never, lambda _: None)
    assert restored.arms[0].config.control == {"max_delta_tau": .4}
    parsed = build_parser().parse_args(["--non-interactive", "--only", "daemon", "--control", "{max_delta_tau: -1}"])
    with pytest.raises(WizardError, match="control"):
        run_wizard(mock_station_probe(), answers_from_args(parsed, draft), never, lambda _: None)
