"""The silri entrypoint: Args defaults, the builder, role wiring, rejected options and the
registrations (real-world CLI and rl-garden's online registry for deploy/eval)."""
from __future__ import annotations

import gymnasium as gym
import numpy as np
import pytest
import rl_garden_real_world.envs.backends.franka_real  # noqa: F401  (registers the backend)
from _silri_fakes import SpacesEnv
from gymnasium import spaces
from gymnasium.vector.utils import batch_space
from rl_garden_real_world.algorithms.silri import SiLRI
from rl_garden_real_world.training.entrypoints.silri import (
    SilriArgs,
    build_silri,
    run_silri,
)

_SMALL = dict(
    buffer_device="cpu",
    feature_dim=8,
    hidden_dim=8,
    behavior_hidden_dims=(8,),
    lambda_hidden_dims=(8,),
    discrete_hidden_dim=8,
)


class _GymEnv(gym.Env):
    """A 7-D (arm 6 + gripper) one-env torch env, as the HIL wrapper stack expects."""

    num_envs = 1

    def __init__(self):
        self.single_observation_space = spaces.Dict(
            {"state": spaces.Box(-1.0, 1.0, (4,), dtype=np.float32)}
        )
        self.single_action_space = spaces.Box(-1.0, 1.0, (7,), dtype=np.float32)
        self.observation_space = batch_space(self.single_observation_space, 1)
        self.action_space = batch_space(self.single_action_space, 1)


class _ArmOnlyContractEnv(SpacesEnv):
    action_contract = {"action_mode": "cartesian_delta", "arm_action_dim": 3, "gripper_in_action": False}


def _args(**overrides) -> SilriArgs:
    return SilriArgs(**{**_SMALL, **overrides})


def test_silri_args_defaults_follow_the_reference_recipe():
    args = SilriArgs()
    assert args.role == "actor"
    assert args.env_backend == "franka_real"
    assert (args.buffer_size, args.demo_buffer_size, args.batch_size) == (30_000, 10_000, 256)
    assert (args.gamma, args.tau, args.utd) == (0.97, 0.005, 2)
    assert (args.actor_lr, args.critic_lr, args.behavior_lr, args.lambda_lr) == (3e-4, 3e-4, 3e-4, 3e-6)
    assert args.grad_clip_norm == 40.0
    assert (args.actor_noise_std, args.constraint_margin) == (0.05, 0.2)
    assert (args.bc_pretrain_steps, args.behavior_refresh_steps, args.behavior_refresh_every) == (500, 50, 50)
    assert (args.std_min, args.std_max) == (1e-5, 2.0)
    assert args.onset_to_demo_buffer is False
    assert (args.learning_starts, args.demo_learning_starts) == (10, 10)
    assert args.demo_data_ratio == 0.5
    assert args.discrete_gripper is True
    assert args.encoder.image_augmentation == "random_shift"


def test_build_silri_forwards_the_args():
    args = _args(gamma=0.9, constraint_margin=0.4, utd=3, onset_to_demo_buffer=True)
    agent = build_silri(args, SpacesEnv(), None, logger=None, checkpoint_dir=None)

    assert type(agent) is SiLRI
    assert (agent.gamma, agent.constraint_margin, agent.utd) == (0.9, 0.4, 3)
    assert agent.onset_to_demo_buffer is True
    assert agent.discrete_gripper is True and agent.policy.continuous_action_dim == 2
    assert agent.offline_replay_buffer is None  # the caller initialises the demo buffer


def test_build_silri_extra_kwargs_and_algorithm_cls_win():
    class _Custom(SiLRI):
        pass

    agent = build_silri(
        _args(), SpacesEnv(), None, logger=None, checkpoint_dir=None,
        algorithm_cls=_Custom, constraint_margin=0.7,
    )
    assert type(agent) is _Custom and agent.constraint_margin == 0.7


class _TwoArmContractEnv(SpacesEnv):
    action_contract = {
        "action_mode": "cartesian_delta",
        "arm_action_dim": 2,
        "gripper_in_action": True,
        "num_arms": 2,
        "arm_names": ["left", "right"],
    }

    def __init__(self):
        super().__init__(action_dim=6)


def test_build_silri_takes_the_gripper_count_from_the_env_contract():
    single = build_silri(_args(), SpacesEnv(), None, logger=None, checkpoint_dir=None)
    assert single.num_grippers == 1 and single.policy.continuous_action_dim == 2
    two_arms = build_silri(_args(), _TwoArmContractEnv(), None, logger=None, checkpoint_dir=None)
    assert two_arms.num_grippers == 2 and two_arms.policy.continuous_action_dim == 4
    assert two_arms.policy.discrete_head.net[-1].out_features == 9


def test_build_silri_rejects_a_gripper_head_without_a_gripper_dim():
    with pytest.raises(ValueError, match="gripper_in_action"):
        build_silri(_args(), _ArmOnlyContractEnv(), None, logger=None, checkpoint_dir=None)
    agent = build_silri(
        _args(discrete_gripper=False), _ArmOnlyContractEnv(), None, logger=None, checkpoint_dir=None
    )
    assert agent.policy.discrete_head is None


@pytest.mark.parametrize(
    "overrides, error", [({"fwbw": True}, NotImplementedError), ({"use_grasp_penalty": True}, ValueError)]
)
def test_run_silri_rejects_unsupported_options(overrides, error):
    with pytest.raises(error):
        run_silri(_args(role="learner", **overrides))


def test_run_learner_builds_the_agent_with_a_demo_buffer_and_starts_the_loop(monkeypatch, tmp_path):
    captured = {}
    monkeypatch.setattr(
        "rl_garden.envs.backend_registry.make_training_envs", lambda backend, req: (_GymEnv(), None)
    )

    class _FakeLearnerLoop:
        def __init__(self, agent, host, port, **kwargs):
            captured.update(agent=agent, host=host, port=port, kwargs=kwargs)

        def run(self):
            captured["ran"] = True

    monkeypatch.setattr("rl_garden_real_world.loops.silri.SilriLearnerLoop", _FakeLearnerLoop)

    args = _args(
        role="learner",
        teleop_device="pico",  # never opened: the learner builds the env without teleop
        sync_host="0.0.0.0",
        sync_port=6500,
        log_dir=str(tmp_path),
        log_type="none",
        exp_name="silri_test",
        demo_buffer_size=16,
        demo_data_ratio=0.25,
        buffer_period=50,
        demo_dataset_paths=("/tmp/demos/*.pkl",),
        train_freq=2,
    )
    run_silri(args)

    agent = captured["agent"]
    assert captured["ran"] is True
    assert isinstance(agent, SiLRI)
    assert agent.offline_replay_buffer.buffer_size == 16 and agent.offline_data_ratio == 0.25
    assert (captured["host"], captured["port"]) == ("0.0.0.0", 6500)
    kwargs = captured["kwargs"]
    assert kwargs["buffer_period"] == 50 and kwargs["train_freq"] == 2
    assert kwargs["demo_dataset_paths"] == ("/tmp/demos/*.pkl",)
    assert kwargs["checkpoint_dir"] == str(tmp_path / "silri_test" / "checkpoints")
    assert (tmp_path / "silri_test" / "config.json").exists()


def test_run_actor_starts_the_silri_actor_loop(monkeypatch):
    captured = {}
    monkeypatch.setattr(
        "rl_garden.envs.backend_registry.make_training_envs", lambda backend, req: (_GymEnv(), None)
    )

    class _FakeActorLoop:
        def __init__(self, env, policy, sync_client, **kwargs):
            captured.update(env=env, policy=policy, sync_client=sync_client, kwargs=kwargs)

        def run(self):
            captured["ran"] = True

    monkeypatch.setattr("rl_garden_real_world.loops.silri.SilriActorLoop", _FakeActorLoop)

    args = _args(role="actor", teleop_device="fake", sync_host="10.0.0.1", sync_port=7000)
    run_silri(args)

    from rl_garden_real_world.envs.wrappers.intervention import InterventionWrapper
    from rl_garden_real_world.policies.silri_policy import SiLRIPolicy

    assert captured["ran"] is True
    assert isinstance(captured["env"], InterventionWrapper)
    assert isinstance(captured["policy"], SiLRIPolicy)
    assert captured["sync_client"]._base_url == "http://10.0.0.1:7000"
    assert captured["kwargs"]["control_hz"] == args.control_hz


def test_run_eval_requires_a_checkpoint():
    with pytest.raises(ValueError, match="load_checkpoint"):
        run_silri(_args(role="eval"))


def test_silri_is_registered_on_the_real_world_cli():
    from rl_garden_real_world.training.registry import registry

    registry.discover()
    entry = registry.entries()["silri"]
    assert entry.args_cls is SilriArgs and entry.run_fn is run_silri


def test_silri_row_registers_on_the_online_registry_for_deploy_and_eval():
    from rl_garden_real_world.deploy import _resolve_build_agent
    from rl_garden_real_world.training import rlgarden_algorithms

    from rl_garden.training.online.registry import registry as online_registry

    args_cls, run_fn, algorithm_cls, build_agent = rlgarden_algorithms._ALGORITHMS["silri"]
    assert (args_cls, run_fn, build_agent) == (SilriArgs, run_silri, build_silri)
    assert algorithm_cls() is SiLRI

    entry = online_registry.entries()["silri"]
    assert entry.build_agent is build_silri and entry.algorithm_cls() is SiLRI
    assert _resolve_build_agent("silri") is build_silri
