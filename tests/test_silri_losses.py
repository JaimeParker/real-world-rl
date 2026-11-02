"""SiLRI's losses on hand-built batches: TD target, actor objective with the (1 + lambda)
normalisation, the multiplier's ascent direction, and the masks of the behaviour NLL and
the gripper cross-entropy."""
from __future__ import annotations

import math

import torch
import torch.nn.functional as F
from _silri_fakes import CONTINUOUS_DIM, TWO_ARM_ACTION_DIM, hand_batch, silri_agent
from torch.distributions import Normal


def test_td_target_uses_the_noisy_target_action_and_masks_done(monkeypatch):
    agent = silri_agent(gamma=0.9)
    policy = agent.policy
    batch = hand_batch(
        [False, True, False],
        rewards=torch.tensor([1.0, 2.0, -1.0]),
        dones=torch.tensor([0.0, 1.0, 0.0]),
    )
    next_action = torch.tensor([[0.3, -0.2], [0.1, 0.1], [-0.5, 0.4]])
    noise_stds = []

    def fake_target_action(next_obs, noise_std):
        noise_stds.append(noise_std)
        return next_action

    monkeypatch.setattr(policy, "target_action", fake_target_action)
    target = agent._td_target(batch)

    with torch.no_grad():
        features = policy.extract_critic_features(batch.next_obs)
        next_q = policy.q_values_all(features, next_action, target=True).min(dim=0).values
    expected = batch.rewards.reshape(-1, 1) + (1 - batch.dones.reshape(-1, 1)) * 0.9 * next_q
    assert noise_stds == [agent.actor_noise_std]
    torch.testing.assert_close(target, expected)
    torch.testing.assert_close(target[1], batch.rewards[1:2])


def test_target_action_reads_the_lagged_actor_and_clamps():
    agent = silri_agent()
    policy = agent.policy
    obs = hand_batch([False] * 4).next_obs
    with torch.no_grad():
        noiseless = policy.target_action(obs, 0.0)
        target_mean = policy.actor_target.deterministic_action(policy.extract_actor_features(obs))
        for param in policy.actor.parameters():
            param.add_(1.0)
        after_actor_change = policy.target_action(obs, 0.0)
        noisy = policy.target_action(obs, 10.0)
    torch.testing.assert_close(noiseless, target_mean)
    torch.testing.assert_close(after_actor_change, noiseless)
    assert float(noisy.abs().max()) <= 1.0
    assert not torch.allclose(noisy, noiseless)


def test_critic_loss_scores_only_the_continuous_action_dims():
    agent = silri_agent()
    batch = hand_batch([False, True])
    flipped_gripper = hand_batch([False, True], actions=batch.actions.clone())
    flipped_gripper.obs, flipped_gripper.next_obs = batch.obs, batch.next_obs
    flipped_gripper.rewards = batch.rewards
    flipped_gripper.actions[:, -1] = -flipped_gripper.actions[:, -1] + 0.5
    torch.manual_seed(1)
    loss = agent._critic_loss_from_batch(batch)
    torch.manual_seed(1)
    loss_with_other_gripper = agent._critic_loss_from_batch(flipped_gripper)
    torch.testing.assert_close(loss, loss_with_other_gripper)


def _set_constant_multiplier(agent, value: float) -> None:
    output = agent.policy.lambda_net.output
    with torch.no_grad():
        output.weight.zero_()
        output.bias.fill_(math.log(math.expm1(value)))  # softplus^-1(value)


def test_actor_loss_is_normalised_by_one_plus_lambda():
    agent = silri_agent()
    policy = agent.policy
    _set_constant_multiplier(agent, 3.0)
    batch = hand_batch([True, False, True, False])

    terms = agent._actor_terms(batch)

    with torch.no_grad():
        actor_mean = policy.actor_mean(batch.obs)
        behavior_mean, behavior_std = policy.behavior(batch.obs)
        distance = torch.linalg.vector_norm(actor_mean - behavior_mean, dim=-1)
        features = policy.extract_critic_features(batch.obs)
        min_q = policy.q_values_all(features, actor_mean).min(dim=0).values.squeeze(-1)
    torch.testing.assert_close(terms.multiplier.detach(), torch.full((4,), 3.0))
    torch.testing.assert_close(terms.distance, distance)
    torch.testing.assert_close(terms.sigma_sum, behavior_std.sum(dim=-1))
    torch.testing.assert_close(terms.actor_loss, ((-min_q + 3.0 * distance) / 4.0).mean())


def test_actor_loss_holds_the_behaviour_model_and_multiplier_fixed():
    agent = silri_agent()
    policy = agent.policy
    terms = agent._actor_terms(hand_batch([True, False, True]))
    terms.actor_loss.backward()
    assert all(p.grad is None for p in policy.behavior_parameters())
    assert all(p.grad is None for p in policy.lambda_parameters())
    assert any(p.grad is not None for p in policy.actor_parameters())


def test_lambda_loss_ascends_where_the_distance_exceeds_the_tolerance():
    agent = silri_agent(constraint_margin=0.2)
    multiplier = torch.ones(3, requires_grad=True)
    distance = torch.tensor([1.0, 1.0, 0.0])
    sigma_sum = torch.tensor([0.1, 0.5, 0.1])

    agent._lambda_loss(multiplier, distance, sigma_sum).backward()

    excess = torch.tensor([0.7, 0.3, -0.3])
    torch.testing.assert_close(multiplier.grad, -excess / 3)
    # Gradient descent raises lambda on rows 0 and 1, lowers it on row 2.
    assert bool((multiplier.grad[:2] < 0).all()) and float(multiplier.grad[2]) > 0


def test_one_multiplier_update_moves_lambda_with_the_constraint():
    # margin -10: every row violates the tolerance; margin +10: none does.
    for margin, rises in ((-10.0, True), (10.0, False)):
        agent = silri_agent(constraint_margin=margin, lambda_lr=1e-2)
        batch = hand_batch([True, False, True, False])
        before = agent.policy.lambda_value(batch.obs).detach()
        agent._actor_and_lambda_step(batch)
        after = agent.policy.lambda_value(batch.obs).detach()
        output_bias = float(agent.policy.lambda_net.output.bias)
        assert (float(after.mean()) > float(before.mean())) is rises
        assert (output_bias > 0.0) is rises


def test_behavior_nll_counts_only_intervened_rows():
    agent = silri_agent()
    policy = agent.policy
    batch = hand_batch([True, False, True, False])

    nll, sigma_sum = agent._behavior_nll(batch)
    other = hand_batch([True, False, True, False], actions=batch.actions.clone())
    other.obs = batch.obs
    other.actions[[1, 3], :CONTINUOUS_DIM] = torch.tensor([[0.9, -0.9], [-0.9, 0.9]])
    nll_other_policy_rows, _ = agent._behavior_nll(other)

    with torch.no_grad():
        mean, std = policy.behavior(batch.obs)
        log_prob = Normal(mean, std).log_prob(batch.actions[:, :CONTINUOUS_DIM]).sum(-1)
    torch.testing.assert_close(nll, -(log_prob[0] + log_prob[2]) / 2)
    torch.testing.assert_close(nll_other_policy_rows, nll)
    torch.testing.assert_close(sigma_sum, std.sum(-1).mean())


def test_behavior_nll_is_zero_without_intervened_rows():
    agent = silri_agent()
    nll, _ = agent._behavior_nll(hand_batch([False, False]))
    torch.testing.assert_close(nll, torch.tensor(0.0))


def test_gripper_cross_entropy_counts_only_intervened_rows():
    agent = silri_agent()
    batch = hand_batch([True, False, True, False])
    batch.actions[:, -1] = torch.tensor([1.0, -1.0, 0.0, 1.0])
    logits = torch.randn(4, 3)

    loss = agent._gripper_cross_entropy(logits, batch)
    batch.actions[[1, 3], -1] = torch.tensor([0.0, -1.0])
    loss_other_policy_rows = agent._gripper_cross_entropy(logits, batch)

    # Gripper values -1/0/1 map to indices 0/1/2.
    expected = F.cross_entropy(logits[[0, 2]], torch.tensor([2, 1]))
    torch.testing.assert_close(loss, expected)
    torch.testing.assert_close(loss_other_policy_rows, expected)


def test_actor_bc_nll_uses_the_fixed_noise_std():
    agent = silri_agent(actor_noise_std=0.1)
    policy = agent.policy
    batch = hand_batch([True, True, False])
    features = policy.extract_actor_features(batch.obs)
    nll = agent._actor_bc_nll(features, batch)
    with torch.no_grad():
        mean = policy.actor_mean(batch.obs)
        squared = (batch.actions[:2, :CONTINUOUS_DIM] - mean[:2]).pow(2).sum(-1)
        constant = CONTINUOUS_DIM * (math.log(0.1) + 0.5 * math.log(2 * math.pi))
        expected = (squared / (2 * 0.1**2) + constant).mean()
    torch.testing.assert_close(nll.detach(), expected)


def test_two_gripper_cross_entropy_uses_the_joint_label_of_the_last_two_dims():
    agent = silri_agent(action_dim=TWO_ARM_ACTION_DIM, num_grippers=2)
    actions = torch.zeros(3, TWO_ARM_ACTION_DIM)
    actions[:, -2:] = torch.tensor([[1.0, 0.0], [-1.0, 1.0], [0.0, -1.0]])
    batch = hand_batch([True, True, True], actions=actions)
    logits = torch.randn(3, 9)

    loss = agent._gripper_cross_entropy(logits, batch)

    # Joint label = 3 * index(first gripper) + index(second gripper).
    torch.testing.assert_close(loss, F.cross_entropy(logits, torch.tensor([7, 2, 3])))
