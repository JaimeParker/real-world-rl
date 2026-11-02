"""residual_hil_serl: Args defaults, agent building for both variants, option checks,
registration on the real-world and rl-garden online registries, the actor / learner /
eval roles, and deploy restoring a residual checkpoint with its base provider."""
from __future__ import annotations

import json

import gymnasium as gym
import pytest
import torch
from gymnasium import spaces
from gymnasium.vector.utils import batch_space
from rl_garden_real_world.algorithms.residual import ResidualRLPD, ResidualRLPDHybrid
from rl_garden_real_world.policies.base_provider import (
    ResidualPolicyWithBase,
    ZeroBaseProvider,
)
from rl_garden_real_world.training.entrypoints.hil_serl import HilSerlArgs
from rl_garden_real_world.training.entrypoints.residual_hil_serl import (
    ResidualHilSerlArgs,
    ResidualHilSerlHybridArgs,
    build_residual_hil_serl,
    run_residual_hil_serl,
    validate_residual_args,
)

_ACTION_DIM = 7  # cartesian twist + gripper: what the fake teleop declares by default


class _FakeEnv(gym.Env):
    """Spaces of a 7-D cartesian HIL env; every episode is one successful step."""

    num_envs = 1

    def __init__(self):
        self.single_observation_space = spaces.Dict({"state": spaces.Box(-1, 1, (4,), dtype="float32")})
        self.single_action_space = spaces.Box(-1, 1, (_ACTION_DIM,), dtype="float32")
        self.observation_space = batch_space(self.single_observation_space, 1)
        self.action_space = batch_space(self.single_action_space, 1)
        self.actions = []

    def reset(self, *, seed=None, options=None):
        return {"state": torch.zeros(1, 4)}, {}

    def step(self, action):
        self.actions.append(action)
        return (
            {"state": torch.zeros(1, 4)},
            torch.ones(1),
            torch.ones(1, dtype=torch.bool),
            torch.zeros(1, dtype=torch.bool),
            {},
        )

    def close(self):
        pass


def _args(**overrides) -> ResidualHilSerlArgs:
    kwargs = dict(
        teleop_device="fake",
        hidden_dim=8,
        discrete_hidden_dim=8,
        buffer_size=16,
        buffer_device="cpu",
        base_provider="zero",
    )
    kwargs.update(overrides)
    return ResidualHilSerlArgs(**kwargs)


@pytest.fixture
def fake_backend(monkeypatch):
    # The env request resolves the franka_real backend's config: register it even when
    # this file runs on its own (the entry point is not loaded without an install).
    import rl_garden_real_world.envs.backends.franka_real  # noqa: F401

    envs = []

    def _make(backend, request):
        envs.append(_FakeEnv())
        return envs[-1], None

    monkeypatch.setattr("rl_garden.envs.backend_registry.make_training_envs", _make)
    return envs


def test_args_defaults():
    args = ResidualHilSerlArgs()
    assert isinstance(args, HilSerlArgs)
    assert args.residual_variant == "continuous"
    assert (args.residual_action_scale, args.residual_gripper_scale) == (0.1, 0.0)
    assert args.residual_zero_init is True and args.residual_log_std_init == -3.0
    assert args.residual_bc_weight == 0.0 and args.residual_warmup_unit == 0.0
    assert args.residual_base_resync_on_intervention is True
    assert args.critic_only_updates == 0
    assert args.allow_base_less_demos is False
    assert args.base_provider == ""
    assert ResidualHilSerlHybridArgs().residual_variant == "hybrid"


@pytest.mark.parametrize("variant, cls", [("continuous", ResidualRLPD), ("hybrid", ResidualRLPDHybrid)])
def test_builds_both_variants_with_the_residual_options(variant, cls):
    args = _args(residual_variant=variant, residual_bc_weight=0.25, critic_only_updates=4, backup_entropy=False)
    agent = build_residual_hil_serl(args, _FakeEnv(), None, None, None)
    assert type(agent) is cls
    assert agent.base_provider_spec == "zero"
    assert agent.residual_bc_weight == 0.25 and agent.critic_only_updates == 4
    assert agent.backup_entropy is False
    expected_dim = _ACTION_DIM if variant == "continuous" else _ACTION_DIM - 1
    assert agent.policy.residual_dim == expected_dim


@pytest.mark.parametrize(
    "overrides, error, match",
    [
        ({"base_provider": " "}, ValueError, "--base-provider"),
        ({"fwbw": True}, NotImplementedError, "fwbw"),
        ({"memory_efficient_buffer": True}, ValueError, "memory-efficient"),
        ({"nstep": 3}, ValueError, "nstep"),
        ({"use_grasp_penalty": True}, ValueError, "grasp-penalty"),
    ],
)
def test_unsupported_options_are_rejected(overrides, error, match):
    with pytest.raises(error, match=match):
        validate_residual_args(_args(**overrides))


def test_hybrid_accepts_the_grasp_penalty():
    validate_residual_args(_args(residual_variant="hybrid", use_grasp_penalty=True))


def test_registered_on_the_real_world_registry():
    from rl_garden_real_world.training.registry import registry

    registry.discover()
    entry = registry.entries()["residual_hil_serl"]
    assert entry.args_cls is ResidualHilSerlArgs
    assert entry.run_fn is run_residual_hil_serl
    assert "residual_hil_serl_hybrid" not in registry.entries()


def test_registered_on_the_online_registry_with_one_name_per_class():
    import rl_garden_real_world.training.rlgarden_algorithms  # noqa: F401  (registers the table)

    from rl_garden.training.online.registry import registry

    entries = registry.entries()
    for name, args_cls, cls in (
        ("residual_hil_serl", ResidualHilSerlArgs, ResidualRLPD),
        ("residual_hil_serl_hybrid", ResidualHilSerlHybridArgs, ResidualRLPDHybrid),
    ):
        assert entries[name].args_cls is args_cls
        assert entries[name].algorithm_cls() is cls
        assert entries[name].build_agent is build_residual_hil_serl


def test_actor_builds_the_provider_and_the_residual_loop(monkeypatch, fake_backend):
    captured = {}

    class _FakeActorLoop:
        def __init__(self, env, policy, sync_client, base_provider, **kwargs):
            captured.update(policy=policy, sync_client=sync_client, provider=base_provider, kwargs=kwargs)

        def run(self):
            captured["ran"] = True

    monkeypatch.setattr("rl_garden_real_world.residual.ResidualActorLoop", _FakeActorLoop)
    run_residual_hil_serl(
        _args(
            role="actor",
            sync_host="10.0.0.1",
            sync_port=7000,
            residual_warmup_unit=0.2,
            residual_base_resync_on_intervention=False,
        )
    )

    assert captured["ran"] is True
    assert isinstance(captured["provider"], ZeroBaseProvider)
    assert captured["provider"].action_dim == _ACTION_DIM
    assert captured["sync_client"]._base_url == "http://10.0.0.1:7000"
    assert captured["kwargs"]["warmup_unit_residual"] == 0.2
    assert captured["kwargs"]["base_resync_on_intervention"] is False
    assert captured["policy"].residual_dim == _ACTION_DIM


def test_learner_builds_the_full_agent_without_a_provider(monkeypatch, fake_backend, tmp_path):
    captured = {}

    class _FakeLearnerLoop:
        def __init__(self, agent, host, port, **kwargs):
            captured.update(agent=agent, host=host, port=port, kwargs=kwargs)

        def run(self):
            captured["ran"] = True

    def _no_provider(*args, **kwargs):
        raise AssertionError("the learner must not build the base provider")

    monkeypatch.setattr("rl_garden_real_world.residual.ResidualLearnerLoop", _FakeLearnerLoop)
    monkeypatch.setattr("rl_garden_real_world.policies.base_provider.make_base_provider", _no_provider)
    run_residual_hil_serl(
        _args(
            role="learner",
            residual_variant="hybrid",
            sync_port=6500,
            log_dir=str(tmp_path),
            log_type="none",
            demo_buffer_size=32,
            buffer_period=50,
        )
    )

    assert captured["ran"] is True
    assert type(captured["agent"]) is ResidualRLPDHybrid
    assert captured["agent"].offline_replay_buffer is not None
    assert captured["port"] == 6500
    assert captured["kwargs"]["buffer_period"] == 50
    assert captured["kwargs"]["allow_base_less_demos"] is False
    config = json.loads(next(tmp_path.glob("*/config.json")).read_text(encoding="utf-8"))
    assert "residual_hil_serl" in json.dumps(config)


def test_eval_composes_the_policy_with_the_provider(monkeypatch, fake_backend, capsys):
    import rl_garden_real_world.training.entrypoints.residual_hil_serl as entrypoint

    built = {}
    original_build = entrypoint.build_residual_hil_serl

    def _build_without_loading(args, env, eval_env, logger, checkpoint_dir):
        built["load_checkpoint"] = args.load_checkpoint
        return original_build(
            entrypoint.dataclasses.replace(args, load_checkpoint=None), env, eval_env, logger, checkpoint_dir
        )

    monkeypatch.setattr(entrypoint, "build_residual_hil_serl", _build_without_loading)
    run_residual_hil_serl(_args(role="eval", load_checkpoint="/tmp/ckpt.pt", eval_n_trajs=3))

    assert built["load_checkpoint"] == "/tmp/ckpt.pt"
    env = fake_backend[-1]
    assert len(env.actions) == 3
    # Zero base and a zero-initialised residual: every executed action is 0.
    for action in env.actions:
        assert torch.equal(action, torch.zeros(1, _ACTION_DIM))
    assert "[eval] success_rate=1.000" in capsys.readouterr().out


def test_eval_requires_a_checkpoint(fake_backend):
    with pytest.raises(ValueError, match="load_checkpoint"):
        run_residual_hil_serl(_args(role="eval", load_checkpoint=None))


# -- deploy ----------------------------------------------------------------------------


class _DeployEnv(_FakeEnv):
    """Never terminates; deploy caps the episode."""

    def step(self, action):
        obs, reward, _, truncated, info = super().step(action)
        return obs, reward, torch.zeros(1, dtype=torch.bool), truncated, info


@pytest.mark.parametrize("algo, variant", [("residual_hil_serl", "continuous"), ("residual_hil_serl_hybrid", "hybrid")])
def test_deploy_restores_a_residual_checkpoint_with_its_recorded_provider(tmp_path, algo, variant):
    import rl_garden_real_world.training.rlgarden_algorithms  # noqa: F401  (registers the table)
    from rl_garden_real_world.deploy import (
        DeployArgs,
        restore_policy,
        restore_training_args,
        run_deploy,
    )

    run_dir = tmp_path / "run"
    checkpoint = run_dir / "checkpoints" / "checkpoint_0.pt"
    env = _DeployEnv()
    saved = build_residual_hil_serl(_args(residual_variant=variant), env, None, None, None)
    saved.save(checkpoint)
    (run_dir / "config.json").write_text(
        json.dumps({"inputs": {"hidden_dim": 8, "discrete_hidden_dim": 8, "residual_variant": variant}}),
        encoding="utf-8",
    )

    args = DeployArgs(checkpoint=str(checkpoint), algo=algo, episodes=2, max_steps_per_episode=2)
    training_args = restore_training_args(args)
    assert training_args.base_provider == "zero"  # from the checkpoint metadata
    policy = restore_policy(args, training_args, env)
    assert isinstance(policy, ResidualPolicyWithBase)
    assert isinstance(policy.provider, ZeroBaseProvider)
    expected = saved.policy.state_dict()
    for key, value in policy.state_dict().items():
        torch.testing.assert_close(value.cpu(), expected[key].cpu())

    results = run_deploy(args, env=env, policy=policy)
    assert [r.steps for r in results] == [2, 2]
    if variant == "continuous":
        assert all(torch.equal(a, torch.zeros(1, _ACTION_DIM)) for a in env.actions)


def test_deploy_base_provider_flag_overrides_the_recorded_spec(tmp_path):
    from rl_garden_real_world.deploy import DeployArgs, _compose_with_base_provider

    agent = build_residual_hil_serl(_args(base_provider="act:/missing.ckpt"), _FakeEnv(), None, None, None)
    args = DeployArgs(checkpoint="unused.pt", algo="residual_hil_serl", base_provider="zero")
    policy = _compose_with_base_provider(args, agent, agent.policy, _FakeEnv())
    assert isinstance(policy.provider, ZeroBaseProvider)
