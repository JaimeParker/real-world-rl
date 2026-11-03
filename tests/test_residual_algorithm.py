"""ResidualRLPD / ResidualRLPDHybrid: replay with base actions, training with the demo
buffer under high UTD, the BC term, the critic-only warm-up and checkpoints."""
from __future__ import annotations

import pytest
import torch
from _residual_fakes import ACTION_DIM, TWO_ARM_ACTION_DIM, DictEnv, add_rows, residual_agent
from real_world_rl.algorithms.residual import ResidualRLPD, ResidualRLPDHybrid
from real_world_rl.buffers import InterventionReplayBuffer, InterventionSample

from rl_garden.common.checkpoint import load_checkpoint_file

_VARIANTS = [ResidualRLPD, ResidualRLPDHybrid]


def _filled_agent(cls, *, grasp_penalty=False, **overrides):
    if grasp_penalty:
        overrides["use_grasp_penalty"] = True
    agent = residual_agent(cls, **overrides)
    agent.init_demo_buffer(buffer_size=32, demo_data_ratio=0.5)
    penalty = -0.05 if grasp_penalty else None
    add_rows(agent.replay_buffer.add, 16, base=0.2, grasp_penalty=penalty)
    add_rows(agent.add_demo_transition, 8, intervened=True, base=0.2, human_offset=0.05, grasp_penalty=penalty)
    return agent


@pytest.mark.parametrize("cls", _VARIANTS)
def test_online_and_demo_buffers_store_base_actions(cls):
    agent = _filled_agent(cls)
    for buffer in (agent.replay_buffer, agent.offline_replay_buffer):
        assert isinstance(buffer, InterventionReplayBuffer)
        assert buffer.store_base_actions
    assert agent.offline_replay_buffer.num_envs == 1
    torch.testing.assert_close(agent.offline_replay_buffer.base_actions[0, 0], torch.full((ACTION_DIM,), 0.2))
    assert bool(agent.offline_replay_buffer.intervened[:8].all())


@pytest.mark.parametrize("cls", _VARIANTS)
def test_one_train_call_with_demo_mixing_and_utd_4(cls):
    agent = _filled_agent(cls, residual_bc_weight=1.0)
    info = agent.train(4, compute_info=True)
    assert info["utd_ratio"] == 4.0
    assert {"actor/residual_bc", "actor/intervened_fraction"} <= set(info)
    assert all(torch.isfinite(torch.tensor(value)) for value in info.values())
    assert 0.0 < info["actor/intervened_fraction"] < 1.0


def test_hybrid_with_grasp_penalty_trains_and_lists_each_slice_key_once():
    agent = _filled_agent(ResidualRLPDHybrid, grasp_penalty=True)
    keys = agent._extra_batch_slice_keys
    assert len(keys) == len(set(keys))
    assert {"intervened", "policy_actions", "base_actions", "next_base_actions", "grasp_penalty"} <= set(keys)
    assert agent.replay_buffer.store_grasp_penalty and agent.offline_replay_buffer.store_grasp_penalty
    agent.train(4)


@pytest.mark.parametrize("cls", _VARIANTS)
def test_extras_survive_high_utd_slicing_of_a_mixed_batch(cls):
    agent = _filled_agent(cls)
    batch = agent._sample_train_batch(8)
    assert isinstance(batch, InterventionSample)
    minibatch = agent._slice_batch(batch, 2, 2)
    assert isinstance(minibatch, InterventionSample)
    for name in ("intervened", "policy_actions", "base_actions", "next_base_actions"):
        torch.testing.assert_close(getattr(minibatch, name), getattr(batch, name)[2:4])


def test_scale_vectors_per_variant():
    continuous = residual_agent(ResidualRLPD, residual_action_scale=0.2, residual_gripper_scale=0.5)
    assert continuous.residual_scale_vector() == [0.2, 0.2, 0.5]
    hybrid = residual_agent(ResidualRLPDHybrid, residual_action_scale=0.2)
    assert hybrid.residual_scale_vector() == [0.2, 0.2]


@pytest.mark.parametrize(
    "cls, overrides, match",
    [
        (ResidualRLPDHybrid, {"residual_gripper_scale": 0.1}, "DQN head"),
        (ResidualRLPD, {"residual_action_scale": 0.0}, "residual_action_scale"),
        (ResidualRLPD, {"nstep": 2}, "nstep"),
        (ResidualRLPDHybrid, {"memory_efficient_buffer": True}, "memory_efficient_buffer"),
    ],
)
def test_unsupported_configurations_are_rejected(cls, overrides, match):
    with pytest.raises(ValueError, match=match):
        residual_agent(cls, **overrides)


def test_bc_term_pulls_the_unit_residual_towards_the_human_residual():
    def trained_distance(bc_weight: float, data) -> tuple[float, float]:
        agent = residual_agent(ResidualRLPD, residual_bc_weight=bc_weight, policy_lr=1e-2)

        def distance() -> float:
            with torch.no_grad():
                unit = agent.policy.unit_residual(data.obs, data.base_actions, deterministic=True)
            # Arm dims: human = base + 0.05 with scale 0.1 -> implied unit residual 0.5.
            return float((unit[:, :-1] - 0.5).pow(2).sum(-1).mean())

        before = distance()
        for _ in range(25):
            agent.policy.prepare_batch_all(data.obs)
            agent._actor_step(data)
        return before, distance()

    source = residual_agent(ResidualRLPD)
    add_rows(source.replay_buffer.add, 16, intervened=True, base=0.1, human_offset=0.05)
    data = source.replay_buffer.sample(8)

    before, with_bc = trained_distance(10.0, data)
    _, without_bc = trained_distance(0.0, data)
    assert before == pytest.approx(0.5)
    assert with_bc < before
    assert with_bc < without_bc


def test_bc_loss_ignores_rows_without_intervention_and_dims_without_scale():
    agent = residual_agent(ResidualRLPD)
    add_rows(agent.replay_buffer.add, 4, intervened=False, base=0.0, human_offset=0.05)
    data = agent.replay_buffer.sample(4)
    loss, fraction = agent._residual_bc_loss(torch.zeros(4, ACTION_DIM), data)
    assert float(loss) == 0.0 and float(fraction) == 0.0

    data.intervened[:] = True
    data.actions[:, -1] = 1.0  # gripper dim has scale 0: no target there
    loss, fraction = agent._residual_bc_loss(torch.zeros(4, ACTION_DIM), data)
    assert float(fraction) == 1.0
    assert float(loss) == pytest.approx(2 * 0.5**2)


def _snapshot(module):
    return [p.detach().clone() for p in module.parameters()]


def _changed(before, module) -> bool:
    return any(not torch.equal(a, b) for a, b in zip(before, module.parameters()))


@pytest.mark.parametrize("cls", _VARIANTS)
def test_critic_only_updates_gate_the_actor_for_the_first_train_calls(cls):
    agent = _filled_agent(cls, critic_only_updates=2)
    actor_before = _snapshot(agent.policy.actor)
    critic_before = _snapshot(agent.policy.critic)

    agent.train(4)
    agent.train(4)
    assert not _changed(actor_before, agent.policy.actor)
    assert _changed(critic_before, agent.policy.critic)
    assert agent._training_update_mask().update_actor

    agent.train(4)
    assert _changed(actor_before, agent.policy.actor)
    assert agent._completed_train_calls == 3


def test_high_utd_remainder_counts_as_one_train_call():
    agent = _filled_agent(ResidualRLPD, critic_only_updates=1)
    actor_before = _snapshot(agent.policy.actor)
    agent.train(6)  # one high-UTD group of 4 plus a recursive train(2) for the remainder
    assert agent._completed_train_calls == 1
    assert not _changed(actor_before, agent.policy.actor)


@pytest.mark.parametrize("cls", _VARIANTS)
def test_checkpoint_round_trip_keeps_weights_counter_and_metadata(cls, tmp_path):
    overrides = dict(base_provider_spec="checkpoint:/runs/bc/final.pt?algo=real_world_bc", residual_bc_weight=0.5,
                     critic_only_updates=3)
    agent = _filled_agent(cls, **overrides)
    agent.train(4)
    path = agent.save(tmp_path / "checkpoint.pt")

    hyperparameters = load_checkpoint_file(path, map_location="cpu")["metadata"]["hyperparameters"]
    assert hyperparameters["base_provider"] == "checkpoint:/runs/bc/final.pt?algo=real_world_bc"
    assert hyperparameters["residual_scale"] == agent.residual_scale_vector()
    assert hyperparameters["residual_bc_weight"] == 0.5
    assert hyperparameters["critic_only_updates"] == 3

    restored = residual_agent(cls, **overrides)
    restored.load(path)
    expected = agent.policy.state_dict()
    for key, value in restored.policy.state_dict().items():
        torch.testing.assert_close(value, expected[key])
    assert restored._completed_train_calls == 1

    other = ResidualRLPDHybrid if cls is ResidualRLPD else ResidualRLPD
    with pytest.raises(ValueError, match="algorithm mismatch"):
        residual_agent(other).load(path, strict=True)


def test_checkpoint_evaluation_is_refused_with_a_reason():
    for cls in _VARIANTS:
        assert "base action provider" in cls.checkpoint_eval_unsupported_reason


def test_auto_target_entropy_is_minus_the_residual_dim():
    hybrid = residual_agent(ResidualRLPDHybrid, env=DictEnv(action_dim=7))
    assert hybrid.target_entropy == -6.0
    continuous = residual_agent(ResidualRLPD, env=DictEnv(action_dim=7))
    assert continuous.target_entropy == -7.0


def test_explicit_target_entropy_is_kept():
    agent = residual_agent(ResidualRLPDHybrid, env=DictEnv(action_dim=7), target_entropy=-1.5)
    assert agent.target_entropy == -1.5


# -- two arms: the last two action dims are the grippers --------------------------------


def _two_arm_agent(cls, **overrides):
    return residual_agent(cls, env=DictEnv(action_dim=TWO_ARM_ACTION_DIM), num_grippers=2, **overrides)


def test_single_gripper_agents_record_one_gripper():
    for cls in _VARIANTS:
        agent = residual_agent(cls)
        assert agent.num_grippers == 1
        assert agent._checkpoint_metadata()["num_grippers"] == 1


def test_two_gripper_scale_vectors_per_variant():
    continuous = _two_arm_agent(ResidualRLPD, residual_action_scale=0.2, residual_gripper_scale=0.5)
    assert continuous.residual_scale_vector() == [0.2, 0.2, 0.2, 0.2, 0.5, 0.5]
    assert continuous.policy.residual_dim == TWO_ARM_ACTION_DIM
    hybrid = _two_arm_agent(ResidualRLPDHybrid, residual_action_scale=0.2)
    assert hybrid.residual_scale_vector() == [0.2, 0.2, 0.2, 0.2]
    assert hybrid.policy.residual_dim == 4
    assert hybrid.policy.discrete_critic.net[-1].out_features == 9


@pytest.mark.parametrize("cls", _VARIANTS)
def test_two_gripper_predict_keeps_the_layout(cls):
    agent = _two_arm_agent(cls)
    base = torch.full((1, TWO_ARM_ACTION_DIM), 0.2)
    with torch.no_grad():
        action = agent.policy.predict({"state": torch.zeros(1, 4)}, deterministic=True, base_actions=base)
    assert action.shape == (1, TWO_ARM_ACTION_DIM)
    # Zero-initialised residual: the arm dims are the base's.
    torch.testing.assert_close(action[:, :4], base[:, :4])
    if cls is ResidualRLPDHybrid:
        assert set(action[0, -2:].tolist()) <= {-1.0, 0.0, 1.0}
    else:
        torch.testing.assert_close(action[:, -2:], base[:, -2:])


@pytest.mark.parametrize("cls", _VARIANTS)
def test_two_gripper_agents_train_and_record_num_grippers(cls):
    agent = _two_arm_agent(cls, residual_bc_weight=1.0)
    agent.init_demo_buffer(buffer_size=32, demo_data_ratio=0.5)
    rows = dict(action_dim=TWO_ARM_ACTION_DIM, num_grippers=2, base=0.2, gripper=-1.0)
    add_rows(agent.replay_buffer.add, 16, **rows)
    add_rows(agent.add_demo_transition, 8, intervened=True, human_offset=0.05, **rows)
    info = agent.train(4, compute_info=True)
    assert all(torch.isfinite(torch.tensor(value)) for value in info.values())
    assert agent._checkpoint_metadata()["num_grippers"] == 2
