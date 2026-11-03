"""real_robot_bimanual backend: registration, config resolution, the torch vector view, and the
intervention wrapper over it."""
from __future__ import annotations

import importlib
from pathlib import Path

import numpy as np
import pytest
import torch
from _bimanual_fakes import ARM_STATE_DIM, ARMS, bimanual_config, camera_name

import robot_infra.devices.claims as claims
from rl_garden.envs.backend_registry import EnvRequest, _validate_env_observation_contract
from rl_garden.observations.config import ObservationConfig
from rl_garden.observations.schema import ObservationContractError

from real_world_rl.envs.backends.real_robot_bimanual import RealRobotBimanualBackend
from real_world_rl.envs.real_robot import RealRobotEnvConfig
from real_world_rl.envs.wrappers.intervention import InterventionWrapper
from real_world_rl.teleop import TeleopReading
from real_world_rl.teleop.fake import ScriptedTeleopSource

CAMERAS = (camera_name("left"), camera_name("right"))
ACTION_DIM = 2 * 6 + 2


@pytest.fixture(autouse=True)
def _claim_dir(tmp_path, monkeypatch):
    monkeypatch.setattr(claims, "_DEFAULT_LOCK_DIR", tmp_path / "claims")


def _req(observation: ObservationConfig | None = None, **config_kwargs) -> EnvRequest:
    return EnvRequest(
        env_id="unused",
        num_envs=8,
        control_mode="",
        render_mode="rgb_array",
        seed=1,
        num_eval_envs=4,
        observation=observation or ObservationConfig(state=True, rgb=CAMERAS),
        backend_config=bimanual_config(**config_kwargs),
    )


@pytest.fixture
def built_envs():
    envs = []
    yield envs
    for env in envs:
        env.close()


# -- registration ---------------------------------------------------------------------


def test_backend_class_contract():
    assert RealRobotBimanualBackend.__dict__["api_version"] == 2
    assert RealRobotBimanualBackend.config_field == "real_robot_bimanual"
    for name in ("resolve_config", "make_train_env", "make_eval_env"):
        assert name in RealRobotBimanualBackend.__dict__


def test_entry_point_declared_in_pyproject_resolves_to_backend():
    tomllib = pytest.importorskip("tomllib")
    pyproject = tomllib.loads((Path(__file__).resolve().parents[1] / "pyproject.toml").read_text())
    target = pyproject["project"]["entry-points"]["rlgarden.env_backends"]["real_robot_bimanual"]
    module = importlib.import_module(target)
    assert module.RealRobotBimanualBackend is RealRobotBimanualBackend


def test_registered_under_real_robot_bimanual_next_to_real_robot():
    from rl_garden.envs.backend_registry import _REGISTRY
    from real_world_rl.envs.backends.real_robot import RealRobotBackend

    assert _REGISTRY["real_robot_bimanual"] is RealRobotBimanualBackend
    assert _REGISTRY["real_robot"] is RealRobotBackend
    # Separate shared-env caches: a request never gets the other backend's env.
    assert RealRobotBimanualBackend._envs_by_request is not RealRobotBackend._envs_by_request


# -- config resolution ---------------------------------------------------------------


def test_resolve_config_applies_requested_image_size_to_both_arms():
    req = _req(ObservationConfig(state=True, rgb=CAMERAS, image_size=(16, 16)))
    config = RealRobotBimanualBackend.resolve_config(req, is_eval=False)
    assert config.left.image_size == (16, 16) and config.right.image_size == (16, 16)
    assert config.cameras == list(CAMERAS)
    assert RealRobotBimanualBackend.resolve_config(req, is_eval=True) == config
    assert req.backend_config.left.image_size is None


@pytest.mark.parametrize(
    "observation, config_kwargs, match",
    [
        (ObservationConfig(state=True, depth=CAMERAS), {}, "depth"),
        (ObservationConfig(state=True, extra_state=("obj",)), {}, "extra_state"),
        (ObservationConfig(state=True, rgb=CAMERAS, frame_stack=2), {}, "frame_stack"),
        # The arms' cameras in the wrong order, a missing one, an unknown one.
        (ObservationConfig(state=True, rgb=CAMERAS[::-1]), {}, "arm order"),
        (ObservationConfig(state=True, rgb=CAMERAS[:1]), {}, "arm order"),
        (ObservationConfig(state=True, rgb=CAMERAS + ("front",)), {}, "arm order"),
        # Requested cameras with no per-arm camera lists: the backend cannot assign them.
        (
            ObservationConfig(state=True, rgb=CAMERAS),
            {"shared": {"cameras": []}},
            "real_robot_bimanual.<arm>.cameras",
        ),
        (
            ObservationConfig(state=True, rgb=CAMERAS, image_size=(16, 16)),
            {"shared": {"image_size": (8, 8)}},
            "conflict",
        ),
    ],
)
def test_resolve_config_rejects_unsupported_observation(observation, config_kwargs, match):
    with pytest.raises(ObservationContractError, match=match):
        RealRobotBimanualBackend.resolve_config(_req(observation, **config_kwargs), is_eval=False)


def test_resolve_config_requires_bimanual_config():
    req = _req()
    req.backend_config = RealRobotEnvConfig(remote="host:1")
    with pytest.raises(ValueError, match="BimanualRealRobotEnvConfig"):
        RealRobotBimanualBackend.resolve_config(req, is_eval=False)


# -- vector env ------------------------------------------------------------------------


def test_make_env_is_shared_and_returns_flattened_torch(built_envs):
    req = _req()
    env = RealRobotBimanualBackend.make_train_env(req)
    built_envs.append(env)
    assert RealRobotBimanualBackend.make_eval_env(req) is env
    assert env.num_envs == 1
    _validate_env_observation_contract(env, req)
    assert list(env.single_observation_space.spaces) == ["rgb_left_wrist", "rgb_right_wrist", "state"]
    assert env.single_observation_space["state"].shape == (2 * ARM_STATE_DIM,)
    assert env.action_space.shape == (1, ACTION_DIM)
    assert env.action_contract["num_arms"] == 2
    assert env.action_mode == "cartesian_delta" and env.gripper_in_action and env.arm_action_dim == 6
    assert set(env.robot) == set(ARMS)

    obs, _ = env.reset(seed=0)
    assert obs["state"].shape == (1, 2 * ARM_STATE_DIM) and obs["state"].dtype == torch.float32
    for cam in CAMERAS:
        assert obs[f"rgb_{cam}"].shape == (1, 24, 32, 3) and obs[f"rgb_{cam}"].dtype == torch.uint8
    # gripper_pos sits after tcp_pose, tcp_vel, joint_pos, joint_vel in each arm's block.
    gripper_offset = 7 + 6 + 7 + 7
    assert obs["state"][0, gripper_offset].item() == 1.0
    assert obs["state"][0, ARM_STATE_DIM + gripper_offset].item() == 1.0

    obs, reward, terminated, truncated, info = env.step(torch.zeros(1, ACTION_DIM))
    assert reward.shape == (1,) and reward.dtype == torch.float32
    assert terminated.dtype == torch.bool and truncated.dtype == torch.bool
    assert info["elapsed_steps"] == 1 and "left_tcp_pose" in info

    env.close()
    assert not any(robot.is_connected for robot in env.robot.values())


def test_state_is_arms_in_order_each_in_config_field_order(built_envs):
    req = _req(ObservationConfig(state=True), shared={"state_fields": ("gripper_pos", "joint_pos"), "cameras": []})
    env = RealRobotBimanualBackend.make_train_env(req)
    built_envs.append(env)
    _validate_env_observation_contract(env, req)
    assert list(env.single_observation_space.spaces) == ["state"]
    obs, _ = env.reset()
    np.testing.assert_allclose(obs["state"][0].numpy(), ([1.0] + [0.0] * 7) * 2)
    env.step(torch.zeros(1, ACTION_DIM))


def test_rgb_only_request(built_envs):
    req = _req(ObservationConfig(state=False, rgb=CAMERAS, image_size=(16, 16)))
    env = RealRobotBimanualBackend.make_train_env(req)
    built_envs.append(env)
    _validate_env_observation_contract(env, req)
    obs, _ = env.reset()
    assert set(obs) == {"rgb_left_wrist", "rgb_right_wrist"}
    assert obs["rgb_right_wrist"].shape == (1, 16, 16, 3)


def test_reward_scale_bias_wrapper_keeps_contract(built_envs):
    req = _req(ObservationConfig(state=True), shared={"cameras": []})
    req.reward_bias = 1.0
    env = RealRobotBimanualBackend.make_train_env(req)
    built_envs.append(env)
    _validate_env_observation_contract(env, req)
    env.reset()
    _, reward, *_ = env.step(torch.zeros(1, ACTION_DIM))
    assert reward.tolist() == [1.0]


# -- intervention wrapper ------------------------------------------------------------


class _TwoArmScriptedSource(ScriptedTeleopSource):
    num_arms = 2


def _two_arm_reading(intervened: bool, grippers=(1.0, 1.0)) -> TeleopReading:
    action = np.zeros(ACTION_DIM, dtype=np.float32)
    action[0] = 1.0  # left +x
    action[-2:] = grippers
    return TeleopReading(action=action, intervened=intervened)


def test_intervention_wrapper_accepts_a_two_arm_source_and_holds_both_grippers(built_envs):
    req = _req(ObservationConfig(state=True), shared={"cameras": []})
    env = RealRobotBimanualBackend.make_train_env(req)
    built_envs.append(env)
    source = _TwoArmScriptedSource([_two_arm_reading(False), _two_arm_reading(True, (-1.0, 0.5))])
    wrapped = InterventionWrapper(env, source)
    assert source.start_calls == 1

    wrapped.reset()
    torch.testing.assert_close(wrapped._last_gripper_action, torch.tensor([1.0, 1.0]))
    policy_action = torch.zeros(1, ACTION_DIM)
    policy_action[0, -2:] = torch.tensor([0.25, -0.25])
    wrapped.step(policy_action)
    torch.testing.assert_close(wrapped._last_gripper_action, torch.tensor([0.25, -0.25]))

    _, _, _, _, info = wrapped.step(policy_action)
    assert info["intervened"] is True and info["intervene_action"].shape == (1, ACTION_DIM)
    torch.testing.assert_close(wrapped._last_gripper_action, torch.tensor([-1.0, 0.5]))


def test_intervention_wrapper_rejects_a_single_arm_source(built_envs):
    req = _req(ObservationConfig(state=True), shared={"cameras": []})
    env = RealRobotBimanualBackend.make_train_env(req)
    built_envs.append(env)
    source = ScriptedTeleopSource([TeleopReading.idle(7)])
    with pytest.raises(ValueError, match="drives 1 arm.*num_arms=2"):
        InterventionWrapper(env, source)
    assert source.start_calls == 0
