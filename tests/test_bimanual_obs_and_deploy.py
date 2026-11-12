"""Two-arm plumbing outside the env: per-arm quaternion slices, training args and teleop checks,
and deploy (contract resolution, restore, health over both arms)."""
from __future__ import annotations

import dataclasses
import types

import gymnasium as gym
import numpy as np
import pytest
import torch
import tyro
from _bimanual_fakes import ARM_STATE_DIM, bimanual_config
from gymnasium import spaces
from gymnasium.vector.utils import batch_space

import real_world_rl.devices.claims as claims
from rl_garden.observations.config import ObservationConfig

import real_world_rl.envs.backends.real_robot_bimanual  # noqa: F401  (registers the backend)
from real_world_rl.deploy import (
    DeployArgs,
    build_deploy_env,
    resolve_deploy_bimanual_config,
    restore_training_args,
    run_deploy,
)
from real_world_rl.envs.bimanual import BimanualRealRobotEnvConfig
from real_world_rl.envs.real_robot import RealRobotEnvConfig
from real_world_rl.envs.wrappers.obs_chain import (
    apply_obs_chain,
    tcp_pose_quat_slice,
    tcp_pose_quat_slices,
)
from real_world_rl.envs.wrappers.rotvec_obs import RotvecObsWrapper
from real_world_rl.training import env_building
from real_world_rl.training.entrypoints.hil_serl import HilSerlArgs

BIMANUAL = "real_robot_bimanual"


@pytest.fixture
def _claim_dir(tmp_path, monkeypatch):
    monkeypatch.setattr(claims, "_DEFAULT_LOCK_DIR", tmp_path / "claims")


def _remote_bimanual(**arm_overrides) -> BimanualRealRobotEnvConfig:
    return BimanualRealRobotEnvConfig(
        left=RealRobotEnvConfig(remote="left-pc:50051", **arm_overrides),
        right=RealRobotEnvConfig(remote="right-pc:50051", **arm_overrides),
    )


# -- quaternion slices ---------------------------------------------------------------


def test_quat_slices_shift_by_one_arm_block_per_arm():
    assert tcp_pose_quat_slices(_remote_bimanual()) == [(3, 7), (ARM_STATE_DIM + 3, ARM_STATE_DIM + 7)]
    config = _remote_bimanual(num_joints=6, state_fields=("joint_pos", "tcp_pose", "gripper_pos"))
    assert tcp_pose_quat_slices(config) == [(9, 13), (23, 27)]  # 6 + 7 + 1 = 14 per arm
    assert tcp_pose_quat_slices(_remote_bimanual(state_fields=("joint_pos",))) == []


def test_quat_slices_of_a_single_arm_config_wrap_the_single_slice():
    config = RealRobotEnvConfig(remote="h:1", num_joints=6, state_fields=("joint_pos", "tcp_pose"))
    assert tcp_pose_quat_slices(config) == [tcp_pose_quat_slice(config)] == [(9, 13)]
    assert tcp_pose_quat_slices(dataclasses.replace(config, state_fields=("joint_pos",))) == []


class _StateEnv(gym.Env):
    num_envs = 1

    def __init__(self, state_dim):
        self.single_observation_space = spaces.Dict(
            {"state": spaces.Box(-np.inf, np.inf, (state_dim,), dtype="float32")}
        )
        self.observation_space = batch_space(self.single_observation_space, 1)


def test_obs_chain_converts_every_arms_quaternion():
    config = _remote_bimanual(num_joints=6, state_fields=("joint_pos", "tcp_pose", "gripper_pos"))
    env = apply_obs_chain(
        _StateEnv(28), frame_stack=1, image_keys=(), convert_obs_rotation=True,
        quat_slice=tcp_pose_quat_slices(config),
    )
    assert isinstance(env, RotvecObsWrapper)
    assert env.single_observation_space["state"].shape == (26,)
    state = torch.zeros(1, 28)
    state[:, [12, 26]] = 1.0  # identity quaternions (w last) of both arms
    state[:, 27] = 7.0  # right gripper_pos
    converted = env.observation({"state": state})["state"]
    assert converted.shape == (1, 26)
    assert torch.allclose(converted[:, 9:12], torch.zeros(1, 3))
    assert torch.allclose(converted[:, 22:25], torch.zeros(1, 3))
    assert converted[0, 25].item() == 7.0


def test_obs_chain_skips_an_empty_slice_list(caplog):
    with caplog.at_level("WARNING"):
        env = apply_obs_chain(_StateEnv(6), frame_stack=1, image_keys=(), convert_obs_rotation=True, quat_slice=[])
    assert not isinstance(env, RotvecObsWrapper)
    assert "convert_obs_rotation skipped" in caplog.text


# -- training args and env building ---------------------------------------------------


def test_training_args_parse_the_bimanual_backend_and_resolve_its_config():
    from real_world_rl.training.entrypoints.serl import SerlArgs

    args = tyro.cli(
        SerlArgs,
        args=[
            "--env_backend", BIMANUAL,
            "--real_robot_bimanual.left.remote", "10.0.0.2:50051",
            "--real_robot_bimanual.right.remote", "10.0.0.3:50051",
            "--real_robot_bimanual.max_episode_steps", "40",
        ],
    )
    assert args.env_backend == BIMANUAL
    assert args.real_robot_bimanual.left.remote == "10.0.0.2:50051"
    assert args.real_robot_bimanual.right.remote == "10.0.0.3:50051"
    assert args.real_robot_bimanual.max_episode_steps == 40
    assert args.resolve_backend_config() is args.real_robot_bimanual


def test_env_building_quat_slice_per_arm_for_bimanual_and_unchanged_otherwise():
    args = HilSerlArgs(env_backend=BIMANUAL)
    assert env_building.tcp_pose_quat_slice(args) == [(3, 7), (ARM_STATE_DIM + 3, ARM_STATE_DIM + 7)]
    no_pose = dataclasses.replace(args, real_robot_bimanual=_remote_bimanual(state_fields=("joint_pos",)))
    assert env_building.tcp_pose_quat_slice(no_pose) is None
    assert env_building.tcp_pose_quat_slice(HilSerlArgs(env_backend="real_robot")) == (3, 7)
    assert env_building.tcp_pose_quat_slice(HilSerlArgs()) == (3, 7)


@pytest.mark.parametrize("device", ["robot", "leader_arm", "gello", "leader_cartesian", "spacemouse", "pico"])
def test_single_arm_teleop_devices_are_rejected_on_bimanual(device):
    with pytest.raises(ValueError, match="needs a bimanual teleop source: use --teleop-device bimanual"):
        env_building.teleop_wrapper_kwargs(HilSerlArgs(env_backend=BIMANUAL, teleop_device=device))
    with pytest.raises(ValueError, match="needs a bimanual teleop source: use --teleop-device bimanual"):
        env_building.teleop_wrapper_kwargs(
            HilSerlArgs(env_backend=BIMANUAL, teleop_device=device, teleop_event_device="keyboard")
        )


def test_device_agnostic_teleop_arguments_pass_through_on_bimanual():
    # The intervention wrapper then checks the source's arm count against the env's contract.
    args = HilSerlArgs(env_backend=BIMANUAL, teleop_device="fake")
    assert env_building.teleop_wrapper_kwargs(args) == {"device": "fake"}


# -- deploy ------------------------------------------------------------------------------


def _training(real_robot_bimanual=None, **extra):
    return types.SimpleNamespace(real_robot_bimanual=real_robot_bimanual, **extra)


def test_deploy_cli_contract_conflicts_are_listed_per_arm():
    trained = dataclasses.replace(_remote_bimanual(), max_episode_steps=7)
    cli = dataclasses.replace(bimanual_config(shared={"xyz_scale_m": 0.05}), max_episode_steps=5)
    args = DeployArgs(checkpoint="unused.pt", env_backend=BIMANUAL, real_robot_bimanual=cli)
    with pytest.raises(ValueError) as error:
        resolve_deploy_bimanual_config(args, _training(trained))
    message = str(error.value)
    assert "left.xyz_scale_m" in message and "right.xyz_scale_m" in message
    assert "max_episode_steps: training=7, cli=5" in message


def test_deploy_default_cli_adopts_each_arms_training_contract_and_keeps_connections():
    left_limits, right_limits = [[-1.0, 1.0]] * 7, [[-0.5, 0.5]] * 7
    trained = BimanualRealRobotEnvConfig(
        left=RealRobotEnvConfig(
            remote="left-pc:1", action_mode="joint_delta", control_hz=5.0, joint_limits=left_limits,
            state_fields=("joint_pos", "tcp_pose"),
        ),
        right=RealRobotEnvConfig(
            remote="right-pc:1", action_mode="joint_delta", control_hz=5.0, joint_limits=right_limits,
            state_fields=("joint_pos", "tcp_pose"),
        ),
        arm_names=("a", "b"),
        max_episode_steps=9,
    )
    cli = bimanual_config()
    args = DeployArgs(checkpoint="unused.pt", env_backend=BIMANUAL, real_robot_bimanual=cli)
    resolved = resolve_deploy_bimanual_config(args, _training(trained))
    assert resolved.arm_names == ("a", "b") and resolved.max_episode_steps == 9
    assert resolved.left.joint_limits == left_limits and resolved.right.joint_limits == right_limits
    for arm in ("left", "right"):
        arm_config = getattr(resolved, arm)
        assert arm_config.action_mode == "joint_delta" and arm_config.control_hz == 5.0
        assert arm_config.state_fields == ("joint_pos", "tcp_pose")
        assert arm_config.robot == getattr(cli, arm).robot and arm_config.remote is None
        assert arm_config.cameras == getattr(cli, arm).cameras


def test_deploy_without_training_bimanual_config_falls_back_to_cli_with_warning(caplog):
    cli = bimanual_config()
    args = DeployArgs(checkpoint="unused.pt", env_backend=BIMANUAL, real_robot_bimanual=cli)
    with caplog.at_level("WARNING"):
        assert resolve_deploy_bimanual_config(args, _training(None)) is cli
    assert "no real_robot_bimanual section" in caplog.text


def test_deploy_args_parse_the_bimanual_backend():
    args = tyro.cli(
        DeployArgs,
        args=[
            "--checkpoint", "ckpt.pt",
            "--env-backend", BIMANUAL,
            "--real_robot_bimanual.left.remote", "10.0.0.2:50051",
            "--real_robot_bimanual.right.remote", "10.0.0.3:50051",
        ],
    )
    assert args.env_backend == BIMANUAL
    assert args.real_robot_bimanual.left.remote == "10.0.0.2:50051"
    assert DeployArgs(checkpoint="x").env_backend == "real_robot"


def _persist_bimanual_run(tmp_path, trained: BimanualRealRobotEnvConfig):
    from rl_garden.common.resolved_config import persist_resolved_config

    persist_resolved_config(
        HilSerlArgs(env_backend=BIMANUAL, real_robot_bimanual=trained, convert_obs_rotation=True),
        training_phase="real_world", algorithm="hil_serl", run_name="run", log_dir=str(tmp_path),
    )
    checkpoint = tmp_path / "run" / "checkpoints" / "checkpoint_0.pt"
    checkpoint.parent.mkdir()
    torch.save({"metadata": {}}, checkpoint)
    return checkpoint


def test_restore_training_args_reads_the_bimanual_config(tmp_path):
    trained = BimanualRealRobotEnvConfig(
        left=RealRobotEnvConfig(remote="left-pc:1", token="t", action_mode="joint_delta", image_size=(16, 16)),
        right=RealRobotEnvConfig(remote="right-pc:1", action_mode="joint_delta", image_size=(16, 16)),
        arm_names=("a", "b"),
        max_episode_steps=12,
    )
    checkpoint = _persist_bimanual_run(tmp_path, trained)

    restored = restore_training_args(
        DeployArgs(checkpoint=str(checkpoint), algo="rlpd_hybrid", env_backend=BIMANUAL)
    )
    assert restored.real_robot_bimanual == trained
    assert restored.convert_obs_rotation is True


def test_restore_training_args_rejects_a_backend_other_than_the_training_runs(tmp_path):
    checkpoint = _persist_bimanual_run(tmp_path, _remote_bimanual())
    with pytest.raises(ValueError, match="--env-backend real_robot_bimanual"):
        restore_training_args(DeployArgs(checkpoint=str(checkpoint), algo="rlpd_hybrid"))


class _ZeroPolicy:
    def __init__(self, action_dim: int) -> None:
        self.action_dim = action_dim

    def eval(self):
        pass

    def predict(self, obs, deterministic=False):
        return torch.zeros(1, self.action_dim)


def test_deploy_env_converts_both_quaternions_and_checks_both_arms_health(_claim_dir):
    trained = bimanual_config(shared={"cameras": []})
    args = DeployArgs(
        checkpoint="unused.pt", env_backend=BIMANUAL, real_robot_bimanual=trained,
        episodes=1, max_steps_per_episode=2,
    )
    training_args = _training(
        trained, obs=ObservationConfig(state=True), convert_obs_rotation=True, classifier_image_keys=()
    )
    env = build_deploy_env(args, training_args)
    from real_world_rl.envs.wrappers.experiment import ExperimentContractWrapper
    assert isinstance(env, ExperimentContractWrapper)
    assert isinstance(env.env, RotvecObsWrapper)
    assert env.single_observation_space["state"].shape == (2 * ARM_STATE_DIM - 2,)
    assert env.single_action_space.shape == (14,)

    (result,) = run_deploy(args, env=env, policy=_ZeroPolicy(14))
    assert result.steps == 2 and result.truncated and result.health_ok is True
    assert not any(robot.is_connected for robot in env.robot.values())
