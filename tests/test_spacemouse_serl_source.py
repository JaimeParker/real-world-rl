"""Native SERL intervention and shared entrypoint wiring; no hardware needed."""
from __future__ import annotations

import dataclasses
import json
import sys
from types import SimpleNamespace

import numpy as np
import pytest
import torch
from _teleop_fakes import ContractEnv, make_hid
from test_spacemouse_teleop_source import _Part

from real_world_rl.devices.errors import SdkMissing

from real_world_rl.action_contract import LEGACY_ACTION_CONTRACT, demo_contract_of
from real_world_rl.envs.wrappers.intervention import TeleopInterventionWrapper
from real_world_rl.teleop import make_teleop_source
from real_world_rl.teleop.composite import CompositeTeleopSource
from real_world_rl.teleop.hold import shared_cartesian_tracker
from real_world_rl.teleop.keyboard import OperatorKeySource
from real_world_rl.teleop.spacemouse import SpaceMouseTeleopSource
from real_world_rl.teleop.spacemouse_serl import SpaceMouseSerlTeleopSource, TRACKER_OPTIONS
from real_world_rl.training import env_building
from real_world_rl.training.entrypoints.hil_serl import HilSerlArgs


def _source(*, gripper=True, **kwargs):
    part = _Part()
    contract = {**LEGACY_ACTION_CONTRACT, "gripper_in_action": gripper}
    return part, SpaceMouseSerlTeleopSource(part, env_contract=contract, **kwargs)


def _args(backend="real_robot", **kwargs):
    args = HilSerlArgs(env_backend=backend, teleop_device="spacemouse", spacemouse_mode="serl", **kwargs)
    if backend == "real_robot":
        args.real_robot = dataclasses.replace(
            args.real_robot, robot={"type": "unused"}, safety_box=[-1] * 3 + [1] * 3,
            safety_box_rpy_low=[0, -1, -1], safety_box_rpy_high=[3, 1, 1],
        )
    else:
        args.franka_real.env_kwargs_json = json.dumps({
            "safety_box_low": [-1] * 3, "safety_box_high": [1] * 3,
            "safety_box_rpy_low": [0, -1, -1], "safety_box_rpy_high": [3, 1, 1],
        })
    return args


def test_serl_norm_uses_all_six_axes_strictly_above_001():
    part, source = _source()
    # float64 boundary isolates strict comparison from float32 quantization.
    read = part.read
    part.read = lambda timeout_s: dataclasses.replace(read(timeout_s), action=part.twist.copy())
    for axis in range(6):
        for value, expected in ((0.0, False), (0.001, False), (0.001001, True)):
            part.twist = np.zeros(6)
            part.twist[axis] = value
            assert source.poll().intervened is expected
    part.twist = np.full(6, 0.0005)
    assert source.poll().intervened  # individually below threshold, jointly above


def test_serl_rotates_translation_and_rotation_without_touching_buttons():
    from scipy.spatial.transform import Rotation
    rotation = Rotation.from_euler("x", 42, degrees=True).as_matrix()
    part, source = _source(input_rotation=rotation.tolist())
    part.twist = np.array([0, 0, .5, 0, 0, .2])
    reading = source.poll()
    np.testing.assert_allclose(reading.action[:3], rotation @ part.twist[:3], atol=1e-7)
    np.testing.assert_allclose(reading.action[3:6], rotation @ part.twist[3:], atol=1e-7)
    assert reading.intervened and reading.action[-1] == 0
    part.twist[:] = 0
    assert not source.poll().intervened


def test_serl_uses_raw_axes_without_per_axis_deadzone():
    part, source = _source()
    part.twist = np.array([0.002, -0.003, 0.004, -0.005, 0.006, -0.007])
    reading = source.poll()
    assert reading.intervened
    np.testing.assert_array_equal(reading.action[:6], part.twist.astype(np.float32))
    assert reading.action.dtype == np.float32


def test_serl_left_close_right_open_and_left_priority():
    part, source = _source()
    for levels, low, high in (([True, False], -1, -0.9), ([False, True], 0.9, 1), ([True, True], -1, -0.9)):
        part.levels = levels
        reading = source.poll()
        assert reading.intervened
        assert low <= reading.action[-1] <= high
        assert reading.buttons == dict(zip(("left", "right"), levels))


def test_serl_buttons_do_not_intervene_without_gripper():
    part, source = _source(gripper=False)
    part.levels = [True, True]
    reading = source.poll()
    assert not reading.intervened
    assert reading.action.shape == (6,)
    assert reading.buttons == {"left": True, "right": True}


def test_serl_no_button_emits_zero_gripper():
    part, source = _source()
    part.twist[0] = 0.2
    assert source.poll().action[-1] == 0.0  # ignores sample.gripper=1
    part.twist[:] = 0
    assert source.poll().action[-1] == 0.0


def test_serl_samples_native_gripper_intervals_each_step():
    seed = 15
    rng, expected = np.random.default_rng(seed), np.random.default_rng(seed)
    part, source = _source(rng=rng)
    for levels, low, high in (([True, False], -1, -0.9), ([True, False], -1, -0.9), ([True, True], -1, -0.9), ([False, True], 0.9, 1)):
        part.levels = levels
        assert source.poll().action[-1] == np.float32(expected.uniform(low, high, size=(1,))[0])
    # No draw on idle or on a gripper-less env, even with held buttons.
    part.levels = [False, False]
    source.poll()
    no_gripper, arm_only = _source(gripper=False, rng=rng)
    no_gripper.levels = [True, True]
    arm_only.poll()
    assert rng.random() == expected.random()


def test_serl_releases_on_next_idle_step():
    part, source = _source()
    part.twist[4] = 0.2
    assert source.poll().intervened
    part.twist[:] = 0
    assert not source.poll().intervened
    part.levels = [True, False]
    assert source.poll().intervened
    part.levels = [False, False]
    assert not source.poll().intervened


def test_serl_ignores_latched_press_after_button_release():
    part, source = _source()
    part.press(0, 1)
    assert not source.poll().intervened


def test_serl_held_button_after_reset_still_intervenes():
    part, source = _source()
    part.levels = [True, False]
    part.press(0)
    source.reset()
    assert part.reset_calls == 1 and not part.pressed
    assert source.poll().intervened


def test_serl_replaces_whole_policy_action_and_reports_buttons():
    env = ContractEnv()
    part, source = _source()
    wrapped = TeleopInterventionWrapper(env, teleop=source)
    try:
        part.twist[:] = [0.2, 0, 0, 0, 0, -0.3]
        policy = torch.ones(1, 7)
        info = wrapped.step(policy)[4]
        expected = torch.tensor([[0.2, 0, 0, 0, 0, -0.3, 0]])
        torch.testing.assert_close(info["intervene_action"], expected)
        torch.testing.assert_close(env.step_actions[-1], expected)
        assert info["policy_action"] is policy
        assert info["intervention_onset"] is True
        assert info["left"] is False and info["right"] is False
        part.twist[:] = 0
        info = wrapped.step(policy)[4]
        assert not info["intervened"] and "intervene_action" not in info
        assert info["left"] is False and info["right"] is False
        assert env.step_actions[-1] is policy
    finally:
        wrapped.close()


def test_serl_composite_uses_motion_intervention_and_keyboard_labels(caplog):
    args = _args(teleop_event_device="stdin")
    kwargs = env_building.teleop_wrapper_kwargs(args)
    assert kwargs["intervention_from"] == "motion"
    part, motion = _source()
    keys = OperatorKeySource()
    composite = CompositeTeleopSource(motion, keys, intervention_from="motion")
    keys._keys.press("t")
    reading = composite.poll()
    assert not reading.intervened and "ignores" in caplog.text
    part.levels = [False, True]
    keys._keys.press("h")
    keys._keys.press("s")
    reading = composite.poll()
    assert reading.intervened and reading.events["success"]
    assert reading.buttons == {"left": False, "right": True}
    keys._keys.press("q")
    part.levels = [False, False]
    reading = composite.poll()
    assert not reading.intervened and reading.events["operator_quit"]


@pytest.mark.parametrize("key", sorted(TRACKER_OPTIONS))
def test_serl_rejects_tracker_kwargs_and_gripper_getter(key):
    with pytest.raises(TypeError, match="serl mode does not accept"):
        SpaceMouseSerlTeleopSource(**{key: None})


@pytest.mark.parametrize("key", sorted(TRACKER_OPTIONS | {"mode"}))
def test_serl_rejects_tracker_and_mode_keys_in_options_yaml(key, tmp_path):
    options = tmp_path / "teleop.yaml"
    options.write_text(f"{key}: null\n")
    with pytest.raises(ValueError, match="serl mode does not accept"):
        env_building.motion_teleop_kwargs(_args(teleop_options_yaml=str(options)))


@pytest.mark.parametrize("backend", ["real_robot", "franka_real"])
def test_serl_learner_contract_without_opening_device(backend, monkeypatch):
    from rl_garden.envs import backend_registry
    from real_world_rl.envs.franka_real.env import FrankaRealEnv
    from real_world_rl.envs.real_robot.env import RealRobotEnv, RealRobotVectorEnv

    def forbidden_start(self):
        raise AssertionError("learner must not open a SpaceMouse")

    monkeypatch.setattr(SpaceMouseSerlTeleopSource, "start", forbidden_start)

    def make_env(name, request):
        if name == "real_robot":
            env = RealRobotVectorEnv(RealRobotEnv(request.backend_config, robot=SimpleNamespace(is_connected=False)))
        else:
            from real_world_rl.envs.backends.franka_real import FrankaRealBackend
            env = FrankaRealEnv(FrankaRealBackend._make_cfg(request), bridge_client=object())
        return env, None

    monkeypatch.setattr(backend_registry, "make_training_envs", make_env)
    env = env_building.build_hil_env(_args(backend), role="learner", enable_teleop=False)
    try:
        contract = demo_contract_of(env)
        assert contract["experiment"]["cartesian_semantics"] == "serl"
        assert contract["experiment"]["sources"]["teleop_control"] == {
            "version": "spacemouse_serl_v1", "mode": "serl",
            "motion_threshold": 0.001, "gripper": "serl_buttons",
        }
    finally:
        env.close()


@pytest.mark.parametrize("backend", ["real_robot", "franka_real"])
def test_serl_mode_sets_env_cartesian_semantics(backend):
    args = _args(backend)
    request = env_building.hil_env_request(args, "test")
    if backend == "real_robot":
        assert request.backend_config.cartesian_semantics == "serl"
    else:
        assert json.loads(request.backend_config.env_kwargs_json)["cartesian_semantics"] == "serl"


def test_serl_mode_with_non_spacemouse_device_is_rejected():
    args = _args()
    args.teleop_device = "fake"
    with pytest.raises(ValueError, match="requires teleop_device"):
        env_building.hil_env_request(args, "test")


def test_default_spacemouse_remains_tracker():
    from _teleop_fakes import CartesianPoseEnv

    args = HilSerlArgs(teleop_device="spacemouse")
    assert args.spacemouse_mode == "tracker"
    assert env_building.teleop_control_spec(args)["version"] == "cartesian_tracker_v1"
    source = make_teleop_source("spacemouse", CartesianPoseEnv(), teleop=_Part(), last_gripper_action_getter=lambda: 1)
    assert type(source) is SpaceMouseTeleopSource


def test_serl_no_tracker_and_no_hold_target():
    part, source = _source()
    wrapped = TeleopInterventionWrapper(ContractEnv(), teleop=source)
    try:
        assert shared_cartesian_tracker(wrapped) is None
        assert not hasattr(source, "cartesian_tracker")
        assert not hasattr(source, "_tracker")
        source.start()  # a caller-owned part needs no connect/disconnect methods
        part.levels = [True, False]
        assert wrapped.step(torch.ones(1, 7))[4]["intervened"]
    finally:
        wrapped.close()


def test_serl_owned_hid_lifecycle_and_factory_dispatch(monkeypatch):
    hid = make_hid()
    monkeypatch.setitem(sys.modules, "hid", hid)
    source = make_teleop_source("spacemouse", ContractEnv(), mode="serl", device_index=2)
    assert isinstance(source, SpaceMouseSerlTeleopSource) and not hid.devices
    source.reset()
    source.start()
    try:
        assert hid.devices[0].kwargs == {"device_index": 2}
        assert source.poll().buttons == {"left": False, "right": False}
    finally:
        source.close()
    assert hid.devices[0].closed
    source.close()


def test_serl_missing_sdk_fails_at_start(monkeypatch):
    monkeypatch.setitem(sys.modules, "hid", None)
    source = SpaceMouseSerlTeleopSource()
    source.reset()
    with pytest.raises(SdkMissing):
        source.start()


def test_serl_device_table_reaches_the_owned_device_part(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    monkeypatch.setenv("XDG_CONFIG_HOME", str(tmp_path / "xdg"))
    table = tmp_path / "spacemouse-devices.yaml"
    table.write_text("schema_version: 1\ndevices:\n  BT: {hid_id: [0x256F, 0xC63A], layout: SpaceMouse Wireless}\n")
    source = SpaceMouseSerlTeleopSource(device_table=str(table))
    assert source._part.config.options["device_table"] == str(table)
    assert "device_table" not in SpaceMouseSerlTeleopSource()._part.config.options
    options = tmp_path / "teleop.yaml"
    options.write_text(f"device_table: {table}\n")
    kwargs = env_building.motion_teleop_kwargs(_args(teleop_options_yaml=str(options)))
    assert kwargs["mode"] == "serl" and kwargs["device_table"] == str(table)


@pytest.mark.parametrize("contract", [
    {**LEGACY_ACTION_CONTRACT, "action_mode": "joint_delta"},
    {**LEGACY_ACTION_CONTRACT, "arm_action_dim": 7},
    {**LEGACY_ACTION_CONTRACT, "num_arms": 2},
])
def test_serl_rejects_other_action_contracts(contract):
    with pytest.raises(ValueError, match="one cartesian_delta arm with 6 axes"):
        SpaceMouseSerlTeleopSource(env_contract=contract)


def test_serl_contract_records_nondefault_axis_inversion(tmp_path):
    path = tmp_path / "teleop.yaml"
    path.write_text("invert_axes: [0, 5]\n")
    assert env_building.teleop_control_spec(_args(teleop_options_yaml=str(path)))["invert_axes"] == [0, 5]


@pytest.mark.parametrize("origin", ["CLI", "preset"])
def test_serl_rejects_explicit_current_from_training_provenance(origin):
    from rl_garden.common.effective_config import FieldSource
    from rl_garden.training.inspection import config_session, standalone_preflight

    args = _args()
    preflight = standalone_preflight(args, training_phase="real_world", algorithm="hil_serl", sources={
        "real_robot.cartesian_semantics": FieldSource(kind=origin, detail="explicit"),
    })
    with config_session(preflight, dry_run=False):
        with pytest.raises(ValueError, match="conflicts.*current"):
            env_building.hil_env_request(args, "conflict")


@pytest.mark.parametrize("event_device", [None, "stdin"])
def test_serl_wrapper_factory_does_not_inject_gripper_holding_getter(event_device, monkeypatch):
    import real_world_rl.teleop.composite as composite_module

    hid = make_hid()
    monkeypatch.setitem(sys.modules, "hid", hid)
    keys = OperatorKeySource()
    factory = composite_module.make_teleop_source
    monkeypatch.setattr(composite_module, "make_teleop_source", lambda name, env, **kwargs:
                        keys if name == "stdin" else factory(name, env, **kwargs))
    kwargs = env_building.teleop_wrapper_kwargs(_args(teleop_event_device=event_device))
    env = TeleopInterventionWrapper(ContractEnv(), **kwargs)
    try:
        source = env.source.motion if event_device else env.source
        assert isinstance(source, SpaceMouseSerlTeleopSource)
        assert not env.step(torch.ones(1, 7))[4]["intervened"]
    finally:
        env.close()
    assert hid.devices[0].closed


def test_serl_record_train_deploy_mode_roundtrip(tmp_path, monkeypatch):
    from rl_garden.common.resolved_config import persist_resolved_config
    from rl_garden.training.online.entrypoints.rlpd_hybrid import RLPDHybridArgs
    from real_world_rl.demos.record import parse_record_args
    from real_world_rl.deploy import DeployArgs, build_deploy_env, restore_training_args
    from real_world_rl.action_contract import save_experiment_metadata
    from real_world_rl.envs.real_robot import RealRobotEnvConfig
    from real_world_rl.training.registry import RealWorldAlgorithmRegistry

    preset = tmp_path / "serl.yaml"
    preset.write_text(
        "env_backend: real_robot\nteleop_device: spacemouse\n"
        "reward_from_teleop_events: false\n"
        "real_robot:\n  robot: {type: franka, arm_backend: fake, gripper: {backend: fake}}\n"
        "  safety_box: [-1, -1, -1, 1, 1, 1]\n"
        "  safety_box_rpy_low: [0, -1, -1]\n  safety_box_rpy_high: [3, 1, 1]\n"
    )
    recorded = parse_record_args([
        "--config", str(preset), "--spacemouse-mode", "serl",
        "--output-path", str(tmp_path / "demo.pkl"),
    ])
    assert dataclasses.asdict(recorded)["real_robot"]["cartesian_semantics"] == "serl"

    registry = RealWorldAlgorithmRegistry()
    registry.register("hil_serl", HilSerlArgs, lambda _: None)
    monkeypatch.setattr(registry, "discover", lambda: None)
    command = registry.parse_command([
        "hil_serl", "--config", str(preset), "--spacemouse-mode", "serl",
    ])
    trained = command.args
    env_building.ensure_real_world_backend(trained.env_backend)
    assert registry._preflight_config(command).inputs["real_robot"]["cartesian_semantics"] == "serl"
    # Serialize before any env is built: this is the learner's original save order.
    persist_resolved_config(
        trained, training_phase="real_world", algorithm="hil_serl",
        run_name="train", log_dir=str(tmp_path),
    )
    config_path = tmp_path / "train" / "config.json"
    serialized = json.loads(config_path.read_text())
    assert serialized["args"]["spacemouse_mode"] == "serl"
    assert serialized["args"]["real_robot"]["cartesian_semantics"] == "serl"
    training_env = env_building.build_hil_env(trained, role="learner", enable_teleop=False)
    try:
        save_experiment_metadata(training_env, tmp_path / "train" / "experiment.json")
    finally:
        training_env.close()
    assert env_building.teleop_control_spec(recorded) == env_building.teleop_control_spec(trained)

    hid = make_hid()
    monkeypatch.setitem(sys.modules, "hid", hid)
    monkeypatch.setattr("real_world_rl.deploy._training_args_cls", lambda _: RLPDHybridArgs)
    monkeypatch.setattr("rl_garden.common.checkpoint.load_checkpoint_file", lambda *args, **kwargs: {
        "metadata": {"hyperparameters": {"spacemouse_mode": "tracker", "real_robot": {"cartesian_semantics": "current"}}},
    })
    deploy = DeployArgs(
        checkpoint=str(tmp_path / "train" / "checkpoints" / "unused.pt"),
        config_path=str(config_path), teleop_device="spacemouse",
        real_robot=RealRobotEnvConfig(robot=trained.real_robot.robot),
    )
    restored = restore_training_args(deploy)
    assert restored.spacemouse_mode == "serl"
    assert restored.real_robot.cartesian_semantics == "serl"
    assert deploy.spacemouse_mode is None
    env = build_deploy_env(deploy, restored)
    try:
        from real_world_rl.demos.record import intervention_wrapper_of

        assert isinstance(intervention_wrapper_of(env).source, SpaceMouseSerlTeleopSource)
        assert demo_contract_of(env)["experiment"]["sources"]["teleop_control"] == env_building.teleop_control_spec(recorded)
    finally:
        env.close()
    assert hid.devices and all(device.closed for device in hid.devices)

    # A deliberate SERL override on a saved current env is rejected without opening HID.
    serialized["args"]["spacemouse_mode"] = "tracker"
    serialized["args"]["real_robot"]["cartesian_semantics"] = "current"
    config_path.write_text(json.dumps(serialized))
    override = dataclasses.replace(deploy, spacemouse_mode="serl")
    restored = restore_training_args(override)
    opened = len(hid.devices)
    with pytest.raises(ValueError, match="override conflicts.*current"):
        build_deploy_env(override, restored)
    assert len(hid.devices) == opened


@pytest.mark.parametrize("backend", ["real_robot", "franka_real"])
def test_training_registry_finalizes_serl_before_preflight_serialization(backend):
    from real_world_rl.training.registry import RealWorldAlgorithmRegistry

    original = _args(backend)
    sources = {}
    finalized, derived = RealWorldAlgorithmRegistry()._normalize_runtime(original, sources)
    serialized = dataclasses.asdict(finalized)
    path = "real_robot.cartesian_semantics" if backend == "real_robot" else "franka_real.env_kwargs_json"
    assert serialized["spacemouse_mode"] == "serl"
    assert sources[path].kind == "runtime-derived"
    assert path in derived
    if backend == "real_robot":
        assert serialized["real_robot"]["cartesian_semantics"] == "serl"
        assert original.real_robot.cartesian_semantics == "current"
    else:
        assert json.loads(serialized["franka_real"]["env_kwargs_json"])["cartesian_semantics"] == "serl"


@pytest.mark.parametrize("origin", ["CLI", "preset"])
def test_training_finalization_rejects_explicit_current_before_serialization(origin):
    from rl_garden.common.effective_config import FieldSource
    from real_world_rl.training.registry import RealWorldAlgorithmRegistry

    sources = {"real_robot.cartesian_semantics": FieldSource(kind=origin, detail="explicit")}
    with pytest.raises(ValueError, match="conflicts.*current"):
        RealWorldAlgorithmRegistry()._normalize_runtime(_args(), sources)


@pytest.mark.parametrize("method", ["hil_serl", "residual_hil_serl", "silri"])
def test_direct_learner_resolves_serl_before_saving_config(method, tmp_path, monkeypatch):
    import importlib
    from rl_garden.common.resolved_config import resolved_run_config

    class ConfigCaptured(Exception):
        pass

    def capture(args, **kwargs):
        serialized = resolved_run_config(args, training_phase="real_world", algorithm=method)
        assert serialized["args"]["spacemouse_mode"] == "serl"
        assert serialized["args"]["real_robot"]["cartesian_semantics"] == "serl"
        raise ConfigCaptured

    monkeypatch.setattr("rl_garden.common.resolved_config.persist_resolved_config", capture)
    monkeypatch.setattr("rl_garden.common.seed_everything", lambda _: None)
    module = importlib.import_module(f"real_world_rl.training.entrypoints.{method}")
    with pytest.raises(ConfigCaptured):
        module._run_learner(_args(log_dir=str(tmp_path)))


def test_serl_rejects_explicit_current_in_franka_json():
    args = _args("franka_real")
    options = json.loads(args.franka_real.env_kwargs_json)
    options["cartesian_semantics"] = "current"
    args.franka_real.env_kwargs_json = json.dumps(options)
    with pytest.raises(ValueError, match="conflicts.*current"):
        env_building.hil_env_request(args, "conflict")


def test_legacy_reward_collector_rejects_serl_before_startup():
    from real_world_rl.reward.collect_data import main

    with pytest.raises(ValueError, match="collect_data --env-backend real_robot"):
        main(argv=["--output-dir", "unused", "--teleop-device", "spacemouse", "--spacemouse-mode", "serl"])


def test_serl_rejects_partial_action_selection():
    part, source = _source()
    with pytest.raises(ValueError, match="all arm and gripper"):
        TeleopInterventionWrapper(ContractEnv(), teleop=source, selected_parts=["arm.arm"])


@pytest.mark.parametrize("missing", ["safety_box", "safety_box_rpy_low", "safety_box_rpy_high"])
def test_serl_requires_complete_xyz_and_rpy_safety_boxes(missing):
    args = _args()
    setattr(args.real_robot, missing, None)
    with pytest.raises(ValueError, match="serl requires"):
        env_building.hil_env_request(args, "missing-limits")


def test_serl_diagnostic_filter_does_not_change_raw_takeover_rule():
    part, source = _source(gripper=False, diagnostic_axes='translation', diagnostic_gain=.2)
    part.twist = np.array([0., 0., 0., .5, 0., 0.])
    reading = source.poll()
    assert reading.intervened
    np.testing.assert_array_equal(reading.action, np.zeros(6))


def test_serl_input_evidence_uses_consumed_sample_and_survives_composite():
    part, source = _source(gripper=False, diagnostic_axes='translation', diagnostic_gain=.2)
    read = part.read
    decoded = [1., 2., 3., 4., 5., 6.]
    part.read = lambda timeout_s: dataclasses.replace(read(timeout_s), seq=17,
        diagnostics={'decoded_axes': decoded, 'hid_report_count': 31})
    part.twist = np.array([.5, .4, .3, .2, .1, 0.])
    composite = CompositeTeleopSource(source, OperatorKeySource(has_gripper=False), intervention_from='motion')
    reading = composite.poll()
    assert reading.diagnostics['sample_seq'] == 17
    assert reading.diagnostics['hid_report_count'] == 31
    np.testing.assert_allclose(reading.diagnostics['base_input_after_filter'], [.1, .08, .06, 0, 0, 0])
    assert reading.diagnostics['decoded_axes'] == decoded
    decoded[0] = 99.
    assert reading.diagnostics['decoded_axes'][0] == 1.
