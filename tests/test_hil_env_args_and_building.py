"""HilEnvArgs extraction (HilSerlArgs must resolve to the same fields and defaults as before)
and training/env_building.build_hil_env."""
from __future__ import annotations

import dataclasses
from pathlib import Path

import gymnasium as gym
import pytest
from gymnasium import spaces
from gymnasium.vector.utils import batch_space

from rl_garden.encoders.config import EncoderConfig
from rl_garden.training.online.entrypoints.rlpd_hybrid import RLPDHybridArgs
from rl_garden_real_world.envs.bimanual.config import BimanualRealRobotEnvConfig
from rl_garden_real_world.envs.real_robot.config import RealRobotEnvConfig
from rl_garden_real_world.training import env_building
from rl_garden_real_world.training.args import (
    FrankaRealConfig,
    HilEnvArgs,
    RealWorldFrankaArgs,
)
from rl_garden_real_world.training.entrypoints.hil_serl import HilSerlArgs

# Every HilSerlArgs field that is not inherited from RLPDHybridArgs, with the default it had
# when it was declared on HilSerlArgs / RealWorldFrankaArgs directly.
HIL_SERL_OWN_DEFAULTS = {
    "role": "actor",
    "sync_host": "127.0.0.1",
    "sync_port": 6000,
    "control_hz": 10.0,
    "deterministic_actor": False,
    "train_freq": 1,
    "publish_freq": 100,
    "env_backend": "franka_real",
    "real_robot": RealRobotEnvConfig(),
    "real_robot_bimanual": BimanualRealRobotEnvConfig(),
    "franka_real": FrankaRealConfig(),
    "eval_n_trajs": 10,
    "classifier_checkpoint": None,
    "classifier_threshold": 0.5,
    "classifier_image_keys": (),
    "use_grasp_penalty": False,
    "grasp_penalty_value": -0.05,
    "record_video_dir": None,
    "memory_efficient_buffer": False,
    "teleop_device": "pico",
    "teleop_arm_device": "leader_arm",
    "robot_teleop_action_mode": None,
    "robot_teleop_has_gripper": True,
    "teleop_event_device": None,
    "teleop_onset": "ramp",
    "teleop_arm_tolerance_rad": 0.1,
    "teleop_ramp_step_rad": 0.05,
    "teleop_options_yaml": None,
    "convert_obs_rotation": False,
    "demo_buffer_size": 100_000,
    "demo_data_ratio": 0.5,
    "buffer_period": 1000,
    "demo_dataset_paths": (),
    "fwbw": False,
    "sync_port_backward": None,
    "fwbw_direction": "forward",
    "backup_entropy": False,
    "encoder": EncoderConfig(
        image_augmentation="random_shift", pooling_method="spatial_learned_embeddings"
    ),
}


def _defaults(cls) -> dict:
    instance = cls()
    return {f.name: getattr(instance, f.name) for f in dataclasses.fields(cls)}


def test_hil_serl_args_fields_and_defaults_are_unchanged():
    resolved = _defaults(HilSerlArgs)
    inherited = _defaults(RLPDHybridArgs)

    assert set(resolved) == set(inherited) | set(HIL_SERL_OWN_DEFAULTS)
    for name, value in resolved.items():
        expected = HIL_SERL_OWN_DEFAULTS[name] if name in HIL_SERL_OWN_DEFAULTS else inherited[name]
        assert value == expected, name


def test_hil_serl_args_inherits_the_shared_env_args():
    assert issubclass(HilSerlArgs, HilEnvArgs)
    assert issubclass(HilSerlArgs, RealWorldFrankaArgs)
    assert issubclass(HilSerlArgs, RLPDHybridArgs)


def test_hil_env_args_holds_exactly_the_env_teleop_classifier_and_demo_fields():
    assert {f.name for f in dataclasses.fields(HilEnvArgs)} == {
        "eval_n_trajs",
        "classifier_checkpoint",
        "classifier_threshold",
        "classifier_image_keys",
        "use_grasp_penalty",
        "grasp_penalty_value",
        "record_video_dir",
        "memory_efficient_buffer",
        "teleop_device",
        "teleop_arm_device",
        "robot_teleop_action_mode",
        "robot_teleop_has_gripper",
        "teleop_event_device",
        "teleop_onset",
        "teleop_arm_tolerance_rad",
        "teleop_ramp_step_rad",
        "teleop_options_yaml",
        "convert_obs_rotation",
        "demo_buffer_size",
        "demo_data_ratio",
        "buffer_period",
        "demo_dataset_paths",
        "fwbw",
        "sync_port_backward",
        "fwbw_direction",
    }


def test_hil_serl_role_still_accepts_eval_and_teleop_device_accepts_fake():
    args = HilSerlArgs(role="eval", teleop_device="fake")
    assert args.role == "eval" and args.teleop_device == "fake"


# -- build_hil_env -------------------------------------------------------------------------


class _FakeEnv(gym.Env):
    num_envs = 1

    def __init__(self):
        self.single_observation_space = spaces.Dict({"state": spaces.Box(-1, 1, (4,), dtype="float32")})
        self.single_action_space = spaces.Box(-1, 1, (7,), dtype="float32")
        self.observation_space = batch_space(self.single_observation_space, 1)
        self.action_space = batch_space(self.single_action_space, 1)


class _RecordingTeleopWrapper(gym.Wrapper):
    instances: list = []

    def __init__(self, env, device="pico", **kwargs):
        super().__init__(env)
        self.device = device
        self.kwargs = kwargs
        type(self).instances.append(self)

    def __getattr__(self, name):
        return getattr(self.env, name)


@pytest.fixture
def fake_backend(monkeypatch):
    base = _FakeEnv()
    monkeypatch.setattr(
        "rl_garden.envs.backend_registry.make_training_envs", lambda backend, req: (base, None)
    )
    monkeypatch.setattr(
        "rl_garden_real_world.envs.wrappers.teleop_intervention.TeleopInterventionWrapper",
        _RecordingTeleopWrapper,
    )
    _RecordingTeleopWrapper.instances = []
    return base


def test_build_hil_env_wraps_teleop_by_default(fake_backend, monkeypatch):
    monkeypatch.setattr(env_building, "hil_env_request", lambda args, run_name: None)
    env = env_building.build_hil_env(HilSerlArgs(teleop_device="spacemouse"), role="actor")

    assert isinstance(env, _RecordingTeleopWrapper)
    assert env.env is fake_backend
    assert env.device == "spacemouse"


def test_build_hil_env_without_teleop_skips_the_intervention_wrapper(fake_backend, monkeypatch):
    monkeypatch.setattr(env_building, "hil_env_request", lambda args, run_name: None)
    env = env_building.build_hil_env(HilSerlArgs(), role="learner", enable_teleop=False)

    assert env is fake_backend
    assert _RecordingTeleopWrapper.instances == []


def test_build_hil_env_without_teleop_keeps_the_outer_wrappers(fake_backend, monkeypatch):
    from rl_garden.envs.wrappers.fwbw_reset_free import FWBWResetFreeWrapper
    from rl_garden_real_world.envs.wrappers.grasp_penalty import GraspPenaltyWrapper

    monkeypatch.setattr(env_building, "hil_env_request", lambda args, run_name: None)
    env = env_building.build_hil_env(
        HilSerlArgs(use_grasp_penalty=True, fwbw=True), role="learner", enable_teleop=False
    )

    assert isinstance(env, FWBWResetFreeWrapper)
    assert isinstance(env.env, GraspPenaltyWrapper)
    assert env.env.env is fake_backend
    assert _RecordingTeleopWrapper.instances == []


def test_build_hil_env_names_the_request_after_the_role_unless_given(fake_backend, monkeypatch):
    names = []

    def _record(args, run_name):
        names.append(run_name)

    monkeypatch.setattr(env_building, "hil_env_request", _record)
    args = HilSerlArgs()
    env_building.build_hil_env(args, role="actor")
    env_building.build_hil_env(args, role="eval")
    env_building.build_hil_env(args, role="learner", enable_teleop=False, run_name="my_run")

    assert names == ["real_world_actor", "real_world_eval", "my_run"]


def test_hil_serl_keeps_its_private_aliases():
    from rl_garden_real_world.training.entrypoints import hil_serl

    assert hil_serl._hil_serl_env_request is env_building.hil_env_request
    assert hil_serl._robot_teleop_kwargs is env_building.robot_teleop_kwargs
    assert hil_serl._tcp_pose_quat_slice is env_building.tcp_pose_quat_slice


def test_compose_hil_env_with_teleop_matches_the_hil_serl_alias(fake_backend):
    from rl_garden_real_world.training.entrypoints.hil_serl import _build_env

    env = _build_env(HilSerlArgs(), env_request=None)
    assert isinstance(env, _RecordingTeleopWrapper) and env.env is fake_backend


# -- teleop wrapper arguments --------------------------------------------------------------

_ONSET = {"onset": "arm", "arm_tolerance_rad": 0.2, "ramp_step_rad": 0.03}


def _onset_args(**overrides) -> HilSerlArgs:
    return HilSerlArgs(
        teleop_onset="arm", teleop_arm_tolerance_rad=0.2, teleop_ramp_step_rad=0.03, **overrides
    )


@pytest.mark.parametrize("device", ["pico", "spacemouse", "fake", "keyboard"])
def test_devices_without_arguments_get_the_device_alone(device):
    assert env_building.teleop_wrapper_kwargs(HilSerlArgs(teleop_device=device)) == {"device": device}


def test_robot_device_keeps_its_part_arguments_and_no_onset():
    args = HilSerlArgs(env_backend="real_robot", teleop_device="robot", robot_teleop_action_mode="joint_abs")
    assert env_building.teleop_wrapper_kwargs(args) == {
        "device": "robot",
        **env_building.robot_teleop_kwargs(args),
    }


def test_leader_arm_gets_the_robot_part_arguments_plus_onset():
    args = _onset_args(
        env_backend="real_robot", teleop_device="leader_arm", robot_teleop_action_mode="joint_delta"
    )
    kwargs = env_building.teleop_wrapper_kwargs(args)
    assert kwargs == {"device": "leader_arm", **env_building.robot_teleop_kwargs(args), **_ONSET}
    assert kwargs["teleop_contract"] == ("joint_delta", args.real_robot.num_joints, True)


def test_leader_arm_needs_the_real_robot_backend():
    with pytest.raises(ValueError, match="leader_arm needs --env-backend real_robot"):
        env_building.teleop_wrapper_kwargs(HilSerlArgs(teleop_device="leader_arm"))


def test_gello_gets_the_onset_arguments():
    assert env_building.teleop_wrapper_kwargs(_onset_args(teleop_device="gello")) == {
        "device": "gello",
        **_ONSET,
    }


def test_event_device_builds_a_composite_with_the_motion_devices_arguments():
    args = _onset_args(
        env_backend="real_robot",
        teleop_device="leader_arm",
        robot_teleop_action_mode="joint_delta",
        teleop_event_device="keyboard",
    )
    assert env_building.teleop_wrapper_kwargs(args) == {
        "device": "composite",
        "motion": "leader_arm",
        "events": "keyboard",
        "intervention_from": "events",
        "motion_kwargs": {**env_building.robot_teleop_kwargs(args), **_ONSET},
    }


def test_build_hil_env_wraps_a_composite_when_an_event_device_is_set(fake_backend, monkeypatch):
    monkeypatch.setattr(env_building, "hil_env_request", lambda args, run_name: None)
    args = HilSerlArgs(teleop_device="spacemouse", teleop_event_device="keyboard")
    env = env_building.build_hil_env(args, role="actor")

    assert isinstance(env, _RecordingTeleopWrapper) and env.env is fake_backend
    assert env.device == "composite"
    assert env.kwargs == {
        "motion": "spacemouse",
        "events": "keyboard",
        "intervention_from": "events",
        "motion_kwargs": {},
    }


def test_build_hil_env_passes_no_teleop_arguments_for_the_default_device(fake_backend, monkeypatch):
    monkeypatch.setattr(env_building, "hil_env_request", lambda args, run_name: None)
    env = env_building.build_hil_env(HilSerlArgs(), role="actor")
    assert env.device == "pico" and env.kwargs == {}


# -- --teleop-options-yaml -----------------------------------------------------------------


def test_options_yaml_reaches_the_spacemouse_source(tmp_path, fake_backend, monkeypatch):
    options = tmp_path / "spacemouse.yaml"
    options.write_text("device_index: 2\nhold_s: 0.3\n", encoding="utf-8")
    monkeypatch.setattr(env_building, "hil_env_request", lambda args, run_name: None)
    args = HilSerlArgs(teleop_device="spacemouse", teleop_options_yaml=str(options))
    env = env_building.build_hil_env(args, role="actor")
    assert env.device == "spacemouse" and env.kwargs == {"device_index": 2, "hold_s": 0.3}


def test_options_yaml_overrides_the_derived_kwargs_and_feeds_the_composite_motion(tmp_path):
    options = tmp_path / "gello.yaml"
    options.write_text("port: /dev/ttyUSB0\nramp_step_rad: 0.02\n", encoding="utf-8")
    args = _onset_args(teleop_device="gello", teleop_options_yaml=str(options))
    assert env_building.teleop_wrapper_kwargs(args) == {
        "device": "gello",
        **_ONSET,
        "port": "/dev/ttyUSB0",
        "ramp_step_rad": 0.02,
    }
    composite = env_building.teleop_wrapper_kwargs(
        dataclasses.replace(args, teleop_event_device="keyboard")
    )
    assert composite["motion_kwargs"] == {**_ONSET, "port": "/dev/ttyUSB0", "ramp_step_rad": 0.02}


def test_options_yaml_must_hold_a_mapping(tmp_path):
    empty, listed = tmp_path / "empty.yaml", tmp_path / "list.yaml"
    empty.write_text("", encoding="utf-8")
    listed.write_text("- 1\n- 2\n", encoding="utf-8")
    assert env_building.teleop_options_from_yaml(str(empty)) == {}
    assert env_building.teleop_options_from_yaml(None) == {}
    with pytest.raises(ValueError, match="mapping"):
        env_building.teleop_options_from_yaml(str(listed))


def test_options_yaml_device_index_opens_that_spacemouse(tmp_path, monkeypatch):
    import sys

    from _teleop_fakes import FakeEnv, make_pyspacemouse

    from rl_garden_real_world.envs.wrappers.intervention import TeleopInterventionWrapper

    pyspacemouse = make_pyspacemouse()
    monkeypatch.setitem(sys.modules, "pyspacemouse", pyspacemouse)
    options = tmp_path / "spacemouse.yaml"
    options.write_text("device_index: 2\n", encoding="utf-8")
    args = HilSerlArgs(teleop_device="spacemouse", teleop_options_yaml=str(options))
    wrapped = TeleopInterventionWrapper(FakeEnv(), **env_building.teleop_wrapper_kwargs(args))
    try:
        assert pyspacemouse.devices[0].kwargs == {"device_index": 2}
    finally:
        wrapped.close()


# -- --teleop-device bimanual ---------------------------------------------------------------

BIMANUAL = "real_robot_bimanual"
STATION_BIMANUAL_YAML = Path(__file__).resolve().parents[1] / "robot_infra" / "docs" / "station.bimanual.example.yaml"


def _bimanual_args(**overrides) -> HilSerlArgs:
    return _onset_args(env_backend=BIMANUAL, teleop_device="bimanual", **overrides)


def test_bimanual_leader_arms_get_the_onset_per_arm():
    assert env_building.teleop_wrapper_kwargs(_bimanual_args()) == {
        "device": "bimanual",
        "device_per_arm": "leader_arm",
        "per_arm_kwargs": {},
        "common_kwargs": _ONSET,
    }


def test_bimanual_declares_each_arms_robot_teleop_part():
    args = _bimanual_args(robot_teleop_action_mode="joint_delta")
    kwargs = env_building.teleop_wrapper_kwargs(args)
    assert kwargs["common_kwargs"] == {"teleop_contract": ("joint_delta", 7, True), **_ONSET}

    robot = env_building.teleop_wrapper_kwargs(dataclasses.replace(args, teleop_arm_device="robot"))
    assert robot["device_per_arm"] == "robot"
    assert robot["common_kwargs"] == {"teleop_contract": ("joint_delta", 7, True)}  # no onset

    gello = env_building.teleop_wrapper_kwargs(dataclasses.replace(args, teleop_arm_device="gello"))
    assert gello["device_per_arm"] == "gello" and gello["common_kwargs"] == _ONSET  # no declaration


def test_bimanual_with_an_event_device_is_the_motion_half_of_a_composite():
    kwargs = env_building.teleop_wrapper_kwargs(_bimanual_args(teleop_event_device="keyboard"))
    assert kwargs == {
        "device": "composite",
        "motion": "bimanual",
        "events": "keyboard",
        "intervention_from": "events",
        "motion_kwargs": {"device_per_arm": "leader_arm", "per_arm_kwargs": {}, "common_kwargs": _ONSET},
    }


def test_bimanual_needs_the_bimanual_backend():
    with pytest.raises(ValueError, match="bimanual needs --env-backend real_robot_bimanual"):
        env_building.teleop_wrapper_kwargs(HilSerlArgs(env_backend="real_robot", teleop_device="bimanual"))


def test_bimanual_options_yaml_splits_arm_sections_from_common_keys(tmp_path):
    options = tmp_path / "bimanual.yaml"
    options.write_text(
        "left: {port: /dev/left}\nright: {port: /dev/right}\nhold_s: 0.2\nintervention_from: all\n",
        encoding="utf-8",
    )
    args = _bimanual_args(teleop_arm_device="gello", teleop_options_yaml=str(options))
    assert env_building.teleop_wrapper_kwargs(args) == {
        "device": "bimanual",
        "device_per_arm": "gello",
        "per_arm_kwargs": {"left": {"port": "/dev/left"}, "right": {"port": "/dev/right"}},
        "intervention_from": "all",
        "common_kwargs": {**_ONSET, "hold_s": 0.2},
    }

    options.write_text("left: /dev/left\n", encoding="utf-8")
    with pytest.raises(ValueError, match="'left' section must be a mapping"):
        env_building.teleop_wrapper_kwargs(args)


def test_bimanual_takes_each_arms_gello_options_from_a_station_yaml():
    args = _bimanual_args(teleop_arm_device="gello", teleop_options_yaml=str(STATION_BIMANUAL_YAML))
    kwargs = env_building.teleop_wrapper_kwargs(args)
    per_arm = kwargs["per_arm_kwargs"]
    assert set(per_arm) == {"left", "right"} and kwargs["common_kwargs"] == _ONSET
    assert "LEFT" in per_arm["left"]["port"] and "RIGHT" in per_arm["right"]["port"]
    assert per_arm["left"]["joint_ids"] == [1, 2, 3, 4, 5, 6, 7] and per_arm["left"]["hold_s"] == 0.0

    renamed = dataclasses.replace(
        args, real_robot_bimanual=BimanualRealRobotEnvConfig(arm_names=("a", "b"))
    )
    with pytest.raises(ValueError, match=r"do not include the env's arms \['a', 'b'\]"):
        env_building.teleop_wrapper_kwargs(renamed)
    # The daemons read their own GELLOs: a station yaml configures only local ones.
    with pytest.raises(ValueError, match="station yaml"):
        env_building.teleop_wrapper_kwargs(dataclasses.replace(args, teleop_arm_device="leader_arm"))


def _single_robot_station(tmp_path) -> Path:
    path = tmp_path / "robot.yaml"
    path.write_text(
        "type: franka\nname: arm\narm_backend: franky\n"
        "teleop:\n  name: gello\n  backend: gello\n  hold_s: 0.1\n"
        "  options: {port: /dev/gello, joint_ids: [1, 2, 3, 4, 5, 6, 7]}\n",
        encoding="utf-8",
    )
    return path


def test_teleop_options_from_yaml_reads_flat_single_and_multi_robot_forms(tmp_path):
    flat = tmp_path / "flat.yaml"
    flat.write_text("port: /dev/ttyUSB0\n", encoding="utf-8")
    assert env_building.teleop_options_from_yaml(str(flat)) == {"port": "/dev/ttyUSB0"}
    assert env_building.teleop_options_from_yaml(str(_single_robot_station(tmp_path))) == {
        "port": "/dev/gello",
        "joint_ids": [1, 2, 3, 4, 5, 6, 7],
        "hold_s": 0.1,
    }
    multi = env_building.teleop_options_from_yaml(str(STATION_BIMANUAL_YAML))
    assert set(multi) == {"left", "right"} and "LEFT" in multi["left"]["port"]


def test_a_station_robot_without_a_gello_teleop_is_rejected(tmp_path):
    station = tmp_path / "station.yaml"
    station.write_text("robots:\n  left: {type: franka}\n", encoding="utf-8")
    with pytest.raises(ValueError, match="'left' has no teleop"):
        env_building.teleop_options_from_yaml(str(station))
    station.write_text("teleop: {backend: spacemouse, options: {}}\n", encoding="utf-8")
    with pytest.raises(ValueError, match="'spacemouse' teleop"):
        env_building.teleop_options_from_yaml(str(station))


def test_single_arm_gello_reads_a_single_robot_station_yaml(tmp_path):
    args = _onset_args(teleop_device="gello", teleop_options_yaml=str(_single_robot_station(tmp_path)))
    assert env_building.teleop_wrapper_kwargs(args) == {
        "device": "gello",
        **_ONSET,
        "port": "/dev/gello",
        "joint_ids": [1, 2, 3, 4, 5, 6, 7],
        "hold_s": 0.1,
    }


def test_single_arm_devices_refuse_station_yamls_they_cannot_use(tmp_path):
    with pytest.raises(ValueError, match="drives one arm"):
        env_building.teleop_wrapper_kwargs(
            _onset_args(teleop_device="gello", teleop_options_yaml=str(STATION_BIMANUAL_YAML))
        )
    with pytest.raises(ValueError, match="not --teleop-device spacemouse"):
        env_building.teleop_wrapper_kwargs(
            HilSerlArgs(teleop_device="spacemouse", teleop_options_yaml=str(_single_robot_station(tmp_path)))
        )
    with pytest.raises(ValueError, match="describes one robot"):
        env_building.teleop_wrapper_kwargs(
            _bimanual_args(teleop_arm_device="gello", teleop_options_yaml=str(_single_robot_station(tmp_path)))
        )


@pytest.fixture
def bimanual_backend_env(tmp_path, monkeypatch):
    """A two-arm vector env over fake Frankas, each with a scripted fake leader part."""
    import robot_infra.devices.claims as claims
    from _bimanual_fakes import bimanual_config, fake_franka
    from robot_infra.devices.teleop import TeleopConfig

    from rl_garden_real_world.envs.bimanual import BimanualRealRobotEnv, BimanualRealRobotVectorEnv

    monkeypatch.setattr(claims, "_DEFAULT_LOCK_DIR", tmp_path / "claims")
    leader = {"action_space": "joint_abs", "num_joints": 7, "script": [{"action": [0.0] * 7}]}
    robots = {
        arm: fake_franka(arm, teleop=TeleopConfig(name="leader", backend="fake", options=leader))
        for arm in ("left", "right")
    }
    config = bimanual_config(shared={"action_mode": "joint_delta"})
    return BimanualRealRobotVectorEnv(BimanualRealRobotEnv(config, robots=robots))


def test_bimanual_args_build_the_bimanual_source_over_the_two_arm_env(bimanual_backend_env):
    from rl_garden_real_world.envs.wrappers.intervention import TeleopInterventionWrapper
    from rl_garden_real_world.teleop.bimanual import BimanualTeleopSource
    from rl_garden_real_world.teleop.leader_arm import LeaderArmPartSource

    args = _bimanual_args(real_robot_bimanual=bimanual_backend_env.env.config)
    wrapped = TeleopInterventionWrapper(bimanual_backend_env, **env_building.teleop_wrapper_kwargs(args))
    try:
        source = wrapped.source
        assert isinstance(source, BimanualTeleopSource) and source.arm_names == ["left", "right"]
        for arm, child in source.sources.items():
            assert isinstance(child, LeaderArmPartSource)
            assert child._onset == "arm" and child._ramp_step_rad == 0.03
            assert child._resolve() is bimanual_backend_env.env.robot[arm].teleop
    finally:
        wrapped.close()
