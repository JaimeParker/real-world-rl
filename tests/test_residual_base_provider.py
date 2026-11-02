"""Base action providers: spec parsing, the factory, ``CheckpointBaseProvider`` on tiny
``real_world_bc`` and ``rlpd`` checkpoints, and ``ResidualPolicyWithBase``."""
from __future__ import annotations

import pytest
import torch
from _residual_fakes import (
    ACTION_DIM,
    STATE_DIM,
    DictEnv,
    RecordingProvider,
    residual_agent,
)
from rl_garden_real_world.algorithms.residual import ResidualRLPD
from rl_garden_real_world.policies.base_provider import (
    BaseActionProvider,
    CheckpointBaseProvider,
    ResidualPolicyWithBase,
    ZeroBaseProvider,
    _SpacesOnlyEnv,
    make_base_provider,
    parse_base_provider_spec,
)


def _spaces():
    env = DictEnv()
    return env.single_observation_space, env.single_action_space


def _make(spec):
    observation_space, action_space = _spaces()
    return make_base_provider(spec, device="cpu", observation_space=observation_space, action_space=action_space)


@pytest.mark.parametrize(
    "spec, expected",
    [
        ("zero", {"kind": "zero", "path": None, "options": {}}),
        (
            "checkpoint:/runs/bc/checkpoints/final.pt?algo=real_world_bc",
            {"kind": "checkpoint", "path": "/runs/bc/checkpoints/final.pt", "options": {"algo": "real_world_bc"}},
        ),
        (
            "act:/data/act.ckpt?temporal_agg=0&temporal_agg_k=0.05",
            {"kind": "act", "path": "/data/act.ckpt", "options": {"temporal_agg": "0", "temporal_agg_k": "0.05"}},
        ),
    ],
)
def test_parse_base_provider_spec(spec, expected):
    assert parse_base_provider_spec(spec) == expected


@pytest.mark.parametrize("spec, match", [("", "empty"), ("bogus:/x", "unknown base provider kind"), ("act:/x?flag", "malformed")])
def test_parse_base_provider_spec_rejects_bad_specs(spec, match):
    with pytest.raises(ValueError, match=match):
        parse_base_provider_spec(spec)


def test_zero_provider_returns_zeros_per_observation_row():
    provider = _make("zero")
    assert isinstance(provider, ZeroBaseProvider)
    assert isinstance(provider, BaseActionProvider)
    action = provider.select_action({"state": torch.randn(3, STATE_DIM)})
    assert torch.equal(action, torch.zeros(3, ACTION_DIM))
    assert provider.spec() == {"kind": "zero", "path": None, "options": {}}


@pytest.mark.parametrize(
    "spec, match",
    [
        ("zero:/some/path", "takes no path"),
        ("zero?x=1", "unknown option"),
        ("checkpoint:/x.pt", r"\?algo="),
        ("checkpoint", "needs a path"),
        ("act", "needs a path"),
        ("checkpoint:/x.pt?algo=rlpd&extra=1", "unknown option"),
    ],
)
def test_factory_rejects_incomplete_specs(spec, match):
    with pytest.raises(ValueError, match=match):
        _make(spec)


def test_checkpoint_provider_on_a_real_world_bc_checkpoint(tmp_path):
    from rl_garden_real_world.training.entrypoints.bc import RealWorldBCArgs

    from rl_garden.training.offline.entrypoints.bc import build_bc

    observation_space, action_space = _spaces()
    bc_args = RealWorldBCArgs(buffer_size=8, buffer_device="cpu", device="cpu")
    saved = build_bc(bc_args, _SpacesOnlyEnv(observation_space, action_space), None, eval_env=None)
    with torch.no_grad():
        for param in saved.policy.parameters():
            param.add_(0.1)
    path = tmp_path / "run" / "checkpoints" / "final.pt"
    saved.save(path)

    provider = _make(f"checkpoint:{path}?algo=real_world_bc")
    assert isinstance(provider, CheckpointBaseProvider)
    obs = {"state": torch.randn(2, STATE_DIM)}
    with torch.no_grad():
        expected = saved.policy.predict(obs, deterministic=True).clamp(-1.0, 1.0)
    torch.testing.assert_close(provider.select_action(obs), expected.cpu())
    assert provider.spec() == {"kind": "checkpoint", "path": str(path), "options": {"algo": "real_world_bc"}}


def test_checkpoint_provider_on_an_online_registry_checkpoint(tmp_path):
    from rl_garden.training.online.entrypoints.rlpd import RLPDArgs, build_rlpd

    observation_space, action_space = _spaces()
    rlpd_args = RLPDArgs(num_envs=1, num_eval_envs=1, buffer_size=8, buffer_device="cpu")
    saved = build_rlpd(rlpd_args, _SpacesOnlyEnv(observation_space, action_space), None, None, None)
    path = tmp_path / "run" / "checkpoints" / "final.pt"
    saved.save(path)

    provider = _make(f"checkpoint:{path}?algo=rlpd")
    obs = {"state": torch.randn(2, STATE_DIM)}
    saved.policy.to("cpu")
    with torch.no_grad():
        expected = saved.policy.predict(obs, deterministic=True).clamp(-1.0, 1.0)
    torch.testing.assert_close(provider.select_action(obs), expected)


def test_residual_policy_with_base_feeds_the_provider_action_and_resets_it():
    policy = residual_agent(ResidualRLPD).policy
    provider = RecordingProvider()
    composed = ResidualPolicyWithBase(policy, provider).eval()

    action = composed.predict({"state": torch.full((1, STATE_DIM), 400.0)}, deterministic=True)
    # Zero-initialised residual: the composed action is the base action.
    torch.testing.assert_close(action, torch.full((1, ACTION_DIM), RecordingProvider.value_for(400.0)))
    assert provider.seen == [400.0]

    composed.reset_episode()
    assert provider.reset_calls == 1
    assert composed.state_dict().keys() == policy.state_dict().keys()
