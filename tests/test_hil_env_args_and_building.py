"""HilEnvArgs extraction (HilSerlArgs must resolve to the same fields and defaults as before)
and training/env_building.build_hil_env."""
from __future__ import annotations

import dataclasses

import gymnasium as gym
import pytest
from gymnasium import spaces
from gymnasium.vector.utils import batch_space

from rl_garden.encoders.config import EncoderConfig
from rl_garden.training.online.entrypoints.rlpd_hybrid import RLPDHybridArgs
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
    "robot_teleop_action_mode": None,
    "robot_teleop_has_gripper": True,
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
        "robot_teleop_action_mode",
        "robot_teleop_has_gripper",
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
