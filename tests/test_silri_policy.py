"""SiLRIPolicy: initialisation, predict (deterministic, noise, gripper mapping) and the
optimizer parameter groups."""
from __future__ import annotations

import itertools
import math

import pytest
import torch
from _silri_fakes import (
    ACTION_DIM,
    CONTINUOUS_DIM,
    STATE_DIM,
    TWO_ARM_ACTION_DIM,
    random_obs,
    silri_agent,
)

from rl_garden.policies.rlpd_hybrid_policy import _DISCRETE_ACTION_VALUES

_GROUPS = (
    "actor_parameters",
    "critic_and_encoder_parameters",
    "behavior_parameters",
    "lambda_parameters",
    "discrete_parameters",
)
_OPTIMIZER_OF_GROUP = {
    "actor_parameters": "actor_optimizer",
    "critic_and_encoder_parameters": "q_optimizer",
    "behavior_parameters": "behavior_optimizer",
    "lambda_parameters": "lambda_optimizer",
    "discrete_parameters": "discrete_optimizer",
}


def test_lambda_is_softplus_of_zero_at_initialisation():
    agent = silri_agent()
    obs = random_obs(agent.env.single_observation_space, batch=5)
    torch.testing.assert_close(agent.policy.lambda_value(obs), torch.full((5,), math.log(2.0)))


def test_shapes_follow_the_continuous_action_dims():
    agent = silri_agent()
    policy = agent.policy
    obs = random_obs(agent.env.single_observation_space, batch=5)

    assert policy.continuous_action_dim == CONTINUOUS_DIM
    assert policy.actor_mean(obs).shape == (5, CONTINUOUS_DIM)
    mean, std = policy.behavior(obs)
    assert mean.shape == std.shape == (5, CONTINUOUS_DIM)
    assert bool(((std >= agent.std_min) & (std <= agent.std_max)).all())
    assert policy.discrete_logits(obs).shape == (5, len(_DISCRETE_ACTION_VALUES))
    assert policy.target_action(obs, 0.1).shape == (5, CONTINUOUS_DIM)
    assert policy.predict(obs).shape == (5, ACTION_DIM)


def test_deterministic_predict_is_the_clamped_actor_mean_without_noise():
    agent = silri_agent()
    policy = agent.policy
    obs = random_obs(agent.env.single_observation_space, batch=6)
    with torch.no_grad():
        first = policy.predict(obs, deterministic=True)
        second = policy.predict(obs, deterministic=True)
        mean = policy.actor_mean(obs).clamp(-1 + 1e-6, 1 - 1e-6)
    torch.testing.assert_close(first, second)
    torch.testing.assert_close(first[:, :CONTINUOUS_DIM], mean)


def test_sampling_noise_std_is_actor_noise_std():
    torch.manual_seed(0)
    agent = silri_agent(actor_noise_std=0.05)
    # Zero observations give a zero actor mean (zero biases), far from the clamp.
    obs = {"state": torch.zeros(20_000, STATE_DIM)}
    with torch.no_grad():
        noisy = agent.policy.predict(obs, deterministic=False)[:, :CONTINUOUS_DIM]
        mean = agent.policy.predict(obs, deterministic=True)[:, :CONTINUOUS_DIM]
    noise = noisy - mean
    assert abs(float(noise.std()) - 0.05) < 2e-3
    assert abs(float(noise.mean())) < 2e-3


@pytest.mark.parametrize("index", range(len(_DISCRETE_ACTION_VALUES)))
def test_gripper_comes_from_the_discrete_head_argmax(index):
    agent = silri_agent()
    output_layer = agent.policy.discrete_head.net[-1]
    with torch.no_grad():
        output_layer.weight.zero_()
        output_layer.bias.zero_()
        output_layer.bias[index] = 5.0
        action = agent.policy.predict(random_obs(agent.env.single_observation_space, batch=4))
    assert action.shape == (4, ACTION_DIM)
    torch.testing.assert_close(action[:, -1], torch.full((4,), _DISCRETE_ACTION_VALUES[index]))


def test_without_discrete_gripper_every_dim_is_continuous():
    agent = silri_agent(discrete_gripper=False)
    policy = agent.policy
    assert policy.discrete_head is None
    assert policy.continuous_action_dim == ACTION_DIM
    assert list(policy.discrete_parameters()) == []
    assert agent.discrete_optimizer is None
    obs = random_obs(agent.env.single_observation_space, batch=3)
    assert policy.predict(obs).shape == (3, ACTION_DIM)


@pytest.mark.parametrize("rgb", [False, True])
def test_parameter_groups_are_disjoint_complete_and_owned_by_one_optimizer(rgb):
    agent = silri_agent(rgb=rgb)
    policy = agent.policy
    groups = {name: {id(p) for p in getattr(policy, name)()} for name in _GROUPS}

    for first, second in itertools.combinations(_GROUPS, 2):
        assert groups[first].isdisjoint(groups[second]), (first, second)
    trainable = {id(p) for p in policy.parameters() if p.requires_grad}
    assert set().union(*groups.values()) == trainable

    targets = list(policy.actor_target.parameters()) + list(policy.critic_target.parameters())
    assert targets and all(not p.requires_grad for p in targets)
    assert {id(p) for p in targets}.isdisjoint(trainable)

    for group, optimizer_name in _OPTIMIZER_OF_GROUP.items():
        optimizer = getattr(agent, optimizer_name)
        owned = {id(p) for param_group in optimizer.param_groups for p in param_group["params"]}
        assert owned == groups[group], optimizer_name


def test_behavior_and_multiplier_own_their_extractors():
    agent = silri_agent(rgb=True)
    policy = agent.policy
    extractors = {
        id(policy.actor_extractor),
        id(policy.critic_extractor),
        id(policy.behavior_model.extractor),
        id(policy.lambda_net.extractor),
    }
    assert len(extractors) == 4
    assert agent.encoder_sharing == "separate"


def test_actor_target_starts_equal_to_the_actor_and_does_not_copy_the_extractor():
    policy = silri_agent(rgb=True).policy
    for online, target in zip(policy.actor.parameters(), policy.actor_target.parameters()):
        torch.testing.assert_close(online, target)
    target_ids = {id(p) for p in policy.actor_target.parameters()}
    assert target_ids.isdisjoint({id(p) for p in policy.actor_extractor.parameters()})


def test_one_gripper_policy_keeps_the_three_way_head():
    policy = silri_agent().policy
    assert policy.num_grippers == 1
    assert policy.discrete_head.net[-1].out_features == len(_DISCRETE_ACTION_VALUES)


def test_two_grippers_widen_the_head_and_append_both_grippers_last():
    agent = silri_agent(action_dim=TWO_ARM_ACTION_DIM, num_grippers=2)
    policy = agent.policy
    obs = random_obs(agent.env.single_observation_space, batch=4)
    assert policy.num_grippers == 2 and policy.continuous_action_dim == 4
    assert policy.discrete_logits(obs).shape == (4, 9)
    output_layer = policy.discrete_head.net[-1]
    with torch.no_grad():
        output_layer.weight.zero_()
        output_layer.bias.zero_()
        output_layer.bias[5] = 5.0  # joint index 5: first gripper hold (0), second open (+1)
        action = policy.predict(obs, deterministic=True)
    assert action.shape == (4, TWO_ARM_ACTION_DIM)
    torch.testing.assert_close(action[:, -2:], torch.tensor([[0.0, 1.0]] * 4))


def test_discrete_gripper_needs_a_continuous_dim_before_the_grippers():
    with pytest.raises(ValueError, match="continuous action dim"):
        silri_agent(action_dim=2, num_grippers=2)
