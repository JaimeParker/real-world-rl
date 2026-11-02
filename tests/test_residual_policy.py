"""ResidualRLPDPolicy / ResidualRLPDHybridPolicy: zero-initialised composition, scaling
and clamping, the hybrid gripper, log-prob shapes and the log-std initialisation."""
from __future__ import annotations

import pytest
import torch
import torch.nn as nn
from _residual_fakes import ACTION_DIM, STATE_DIM, residual_agent
from rl_garden_real_world.algorithms.residual import ResidualRLPD, ResidualRLPDHybrid
from rl_garden_real_world.policies.residual_policy import (
    ResidualRLPDHybridPolicy,
    ResidualRLPDPolicy,
)


def _obs(batch: int = 5) -> dict:
    return {"state": torch.randn(batch, STATE_DIM)}


def _base(batch: int = 5) -> torch.Tensor:
    return torch.rand(batch, ACTION_DIM) * 1.8 - 0.9


def test_policy_classes_per_variant():
    assert isinstance(residual_agent(ResidualRLPD).policy, ResidualRLPDPolicy)
    assert isinstance(residual_agent(ResidualRLPDHybrid).policy, ResidualRLPDHybridPolicy)


def test_zero_init_continuous_policy_acts_exactly_like_the_base():
    policy = residual_agent(ResidualRLPD).policy
    obs, base = _obs(), _base()
    with torch.no_grad():
        unit = policy.unit_residual(obs, base, deterministic=True)
        action = policy.predict(obs, deterministic=True, base_actions=base)
    assert torch.equal(unit, torch.zeros(5, ACTION_DIM))
    assert torch.equal(action, base)


def test_zero_init_hybrid_policy_keeps_the_base_arm_and_takes_the_dqn_gripper():
    policy = residual_agent(ResidualRLPDHybrid).policy
    obs, base = _obs(), _base()
    with torch.no_grad():
        action = policy.predict(obs, deterministic=True, base_actions=base)
    assert action.shape == (5, ACTION_DIM)
    assert torch.equal(action[:, :-1], base[:, :-1])
    assert set(action[:, -1].tolist()) <= {-1.0, 0.0, 1.0}


class _FixedLogits(nn.Module):
    def __init__(self, logits) -> None:
        super().__init__()
        self.register_buffer("logits", torch.tensor(logits))

    def forward(self, features):
        return self.logits.expand(features.shape[0], -1)


def test_hybrid_gripper_comes_from_the_dqn_head_not_the_base():
    policy = residual_agent(ResidualRLPDHybrid).policy
    policy.discrete_critic = _FixedLogits([0.0, 0.0, 5.0])  # argmax 2 -> gripper +1
    base = _base()
    base[:, -1] = -1.0
    with torch.no_grad():
        action = policy.predict(_obs(), deterministic=True, base_actions=base)
    assert torch.equal(action[:, -1], torch.ones(5))


def test_compose_scales_the_unit_residual_and_clamps_to_the_action_box():
    policy = residual_agent(ResidualRLPD, residual_action_scale=0.1, residual_gripper_scale=0.0).policy
    torch.testing.assert_close(policy.residual_scale, torch.tensor([0.1, 0.1, 0.0]))
    base = torch.tensor([[0.95, 0.0, 0.5]])
    unit = torch.tensor([[1.0, -0.5, 1.0]])
    torch.testing.assert_close(policy.compose(base, unit), torch.tensor([[1.0, -0.05, 0.5]]))


def test_hybrid_compose_covers_the_arm_dims_only():
    policy = residual_agent(ResidualRLPDHybrid).policy
    assert policy.residual_dim == ACTION_DIM - 1
    base = torch.tensor([[0.0, -0.98, -1.0]])
    unit = torch.tensor([[1.0, -1.0]])
    torch.testing.assert_close(policy.compose(base, unit), torch.tensor([[0.1, -1.0]]))


@pytest.mark.parametrize("cls, width", [(ResidualRLPD, ACTION_DIM), (ResidualRLPDHybrid, ACTION_DIM - 1)])
def test_actor_action_log_prob_returns_the_composed_action_and_the_unit_log_prob(cls, width):
    policy = residual_agent(cls).policy
    obs, base = _obs(6), _base(6)
    action, log_prob, features = policy.actor_action_log_prob(obs, base_actions=base)
    assert action.shape == (6, width)
    assert log_prob.shape == (6, 1)
    assert features.shape[0] == 6

    composed, unit, unit_log_prob, _ = policy.residual_action_log_prob(obs, base_actions=base)
    assert unit.shape == (6, width)
    assert unit.abs().max() <= 1.0
    torch.testing.assert_close(composed, policy.compose(base, unit))
    assert unit_log_prob.shape == (6, 1)


def test_actor_diagnostics_take_the_base_actions():
    policy = residual_agent(ResidualRLPD).policy
    diagnostics = policy.actor_diagnostics(_obs(), _base())
    assert set(diagnostics) == {"policy_std", "action_saturation", "entropy_gaussian", "tanh_correction"}


def test_methods_without_base_actions_raise():
    policy = residual_agent(ResidualRLPD).policy
    with pytest.raises(ValueError, match="base_actions"):
        policy.predict(_obs(), deterministic=True)
    with pytest.raises(ValueError, match="base_actions"):
        policy.actor_action_log_prob(_obs())


@pytest.mark.parametrize("log_std_mode", ["clamp", "tanh"])
def test_log_std_starts_at_residual_log_std_init(log_std_mode):
    policy = residual_agent(
        ResidualRLPD, actor_log_std_mode=log_std_mode, residual_log_std_init=-2.5
    ).policy
    features = policy.extract_features(_obs(), stop_gradient=True)
    _, log_std = policy.actor(policy._actor_input(features, _base()))
    torch.testing.assert_close(log_std, torch.full_like(log_std, -2.5), atol=1e-4, rtol=0)


def test_log_std_init_outside_the_actor_range_is_rejected():
    with pytest.raises(ValueError, match="residual_log_std_init"):
        residual_agent(ResidualRLPD, residual_log_std_init=-9.0)


def test_without_zero_init_the_residual_is_not_zero():
    policy = residual_agent(ResidualRLPD, residual_zero_init=False).policy
    with torch.no_grad():
        unit = policy.unit_residual(_obs(), _base(), deterministic=True)
    assert unit.abs().sum() > 0


def test_actor_input_is_the_actor_features_plus_the_residual_dims():
    for cls in (ResidualRLPD, ResidualRLPDHybrid):
        policy = residual_agent(cls).policy
        first_linear = next(m for m in policy.actor.trunk.modules() if isinstance(m, nn.Linear))
        assert first_linear.in_features == policy._actor_fd + policy.residual_dim


def test_residual_scale_is_part_of_the_state_dict():
    policy = residual_agent(ResidualRLPD).policy
    torch.testing.assert_close(policy.state_dict()["residual_scale"], torch.tensor([0.1, 0.1, 0.0]))
