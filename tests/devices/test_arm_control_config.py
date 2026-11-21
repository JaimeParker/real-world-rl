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


# --- SERL compliance parameters (cartesian_controller: serl) ---

def test_serl_parameter_table_is_serl_compliance_param_cfg():
    from real_world_rl.devices.control import SERL_COMPLIANCE_PARAMS, SERL_PARAM_DEFAULTS
    assert len(SERL_COMPLIANCE_PARAMS) == 20
    assert SERL_COMPLIANCE_PARAMS[0] == ("translational_stiffness", 2000.0, 0.0, 6000.0)
    assert SERL_PARAM_DEFAULTS["translational_damping"] == 89.0 and SERL_PARAM_DEFAULTS["rotational_damping"] == 7.0
    assert SERL_PARAM_DEFAULTS["nullspace_stiffness"] == 0.2 and SERL_PARAM_DEFAULTS["joint1_nullspace_stiffness"] == 100.0
    assert SERL_PARAM_DEFAULTS["translational_clip_neg_z"] == 0.01 and SERL_PARAM_DEFAULTS["rotational_clip_y"] == 0.05
    assert SERL_PARAM_DEFAULTS["translational_Ki"] == 0.0 and SERL_PARAM_DEFAULTS["rotational_Ki"] == 0.0


def test_serl_table_matches_the_vendored_cfg():
    """The Python table and the native package's vendored compliance_param.cfg agree."""
    import re
    from pathlib import Path
    from real_world_rl.devices.control import SERL_COMPLIANCE_PARAMS
    cfg = (Path(__file__).resolve().parents[2] / "native/serl_controller/tests/cpp/golden/serl_original/cfg/compliance_param.cfg")
    if not cfg.is_file():
        pytest.skip("native/ sources not present")
    rows = re.findall(r'gen\.add\("(\w+)", double_t, 0, "[^"]*", ([\d.]+), ([\d.]+), ([\d.]+)\)', cfg.read_text())
    assert [(n, float(d), float(lo), float(hi)) for n, d, lo, hi in rows] == list(SERL_COMPLIANCE_PARAMS)


def test_serl_profiles_are_partial_updates_on_the_current_params(caplog):
    from real_world_rl.devices.control import (SERL_PARAM_DEFAULTS, apply_serl_profile, resolve_serl_params,
                                               serl_param_updates, serl_profile_update, validate_serl_profiles)
    station = resolve_serl_params({"translational_stiffness": 1800.0})
    profiles = {"compliance": {"translational_clip_x": 0.0075},
                "precision": {"translational_clip_x": 0.1, "rotational_clip_x": 0.5, "rotational_stiffness": 250}}
    with caplog.at_level("WARNING"):
        validate_serl_profiles(profiles)
    assert "switching order matters" in caplog.text and "rotational_clip_x" in caplog.text
    assert serl_profile_update(profiles, "compliance") == {"translational_clip_x": 0.0075}
    precision = apply_serl_profile(station, profiles, "precision")
    assert precision["rotational_clip_x"] == 0.1 and precision["rotational_stiffness"] == 250.0
    assert precision["translational_stiffness"] == 1800.0
    # HIL-SERL semantics: compliance after precision keeps precision's rotational values
    compliance = apply_serl_profile(precision, profiles, "compliance")
    assert compliance["translational_clip_x"] == 0.0075 and compliance["rotational_stiffness"] == 250.0
    assert apply_serl_profile(station, profiles, "compliance")["rotational_stiffness"] == SERL_PARAM_DEFAULTS["rotational_stiffness"]
    with pytest.raises(KeyError, match="unknown serl profile"):
        serl_profile_update(profiles, "reset")
    caplog.clear()
    with caplog.at_level("WARNING"):
        validate_serl_profiles({"a": {"translational_clip_x": 0.01}, "b": {"translational_clip_x": 0.02}})
    assert "switching order" not in caplog.text
    values, clamped = serl_param_updates({"translational_stiffness": -5})
    assert values == {"translational_stiffness": 0.0} and clamped["translational_stiffness"]["requested"] == -5.0
    for invalid in ({"x": {}}, {"": {"rotational_Ki": 1}}, {"a": {"bad": 1}}, {"a": {"rotational_Ki": True}}, ["a"]):
        with pytest.raises(ValueError):
            validate_serl_profiles(invalid)


def test_task_preset_serl_profiles_are_validated():
    pytest.importorskip("gymnasium")
    from real_world_rl.envs.real_robot.config import RealRobotEnvConfig
    config = RealRobotEnvConfig(serl_profiles={"compliance": {"translational_clip_x": 0.0075},
                                               "precision": {"rotational_clip_x": 0.5}})
    assert config.serl_profiles["precision"] == {"rotational_clip_x": 0.1}
    with pytest.raises(ValueError, match="unknown SERL"):
        RealRobotEnvConfig(serl_profiles={"compliance": {"stiffness": 1.0}})
    assert RealRobotEnvConfig().serl_profiles is None


def test_station_renderer_writes_serl_and_load():
    import yaml
    from real_world_rl.setup.render import check_rendered_station, render_station_yaml
    from real_world_rl.setup.wizard import ArmDraft, StationDraft
    config = FrankaConfig(cartesian_controller="serl", realtime="enforce",
                          serl={"translational_stiffness": 2000.0, "rotational_clip_neg_z": 0.02},
                          load={"mass_kg": 0.3, "center_of_mass_m": [0, 0, 0.04], "inertia": [0.001, 0, 0, 0, 0.001, 0, 0, 0, 0.001]})
    draft = StationDraft([ArmDraft("robot", config)])
    text = render_station_yaml(draft)
    rendered = yaml.safe_load(text)
    assert rendered["cartesian_controller"] == "serl" and rendered["serl"] == config.serl
    assert rendered["load"] == config.load
    check_rendered_station(text, draft)
    default = yaml.safe_load(render_station_yaml(StationDraft([ArmDraft("robot", FrankaConfig())])))
    assert default["serl"] is None and default["load"] is None
