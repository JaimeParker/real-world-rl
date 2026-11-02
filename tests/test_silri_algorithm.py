"""SiLRI: update schedule, target updates, pretraining and behaviour refreshes, end-to-end
``train`` on state-only and image observations, buffers and checkpoints."""
from __future__ import annotations

import math

import pytest
import torch
from _silri_fakes import TWO_ARM_ACTION_DIM, fill, silri_agent
from rl_garden_real_world.buffers.intervention_replay_buffer import (
    InterventionReplayBuffer,
    InterventionSample,
)

_TRAIN_INFO_KEYS = {
    "critic/loss",
    "actor/loss",
    "actor/q",
    "actor/distance",
    "lambda/loss",
    "lambda/mean",
    "behavior/sigma_sum",
    "discrete/ce",
    "batch/intervened_fraction",
}
_OPTIMIZERS = (
    "q_optimizer",
    "actor_optimizer",
    "behavior_optimizer",
    "lambda_optimizer",
    "discrete_optimizer",
)


def _ready_agent(rgb: bool = False, **overrides):
    agent = silri_agent(rgb=rgb, **overrides)
    agent.init_demo_buffer(buffer_size=32, demo_data_ratio=0.5)
    fill(agent, 8)
    fill(agent, 8, demo=True)
    return agent


def _record_updates(agent, monkeypatch) -> list[str]:
    events: list[str] = []
    labels = {
        "q_optimizer": "critic",
        "actor_optimizer": "actor",
        "discrete_optimizer": "gripper",
        "lambda_optimizer": "lambda",
        "behavior_optimizer": "behavior",
    }
    for name, label in labels.items():
        getattr(agent, name).register_step_post_hook(
            lambda optimizer, args, kwargs, label=label: events.append(label)
        )
    target_update = agent._target_update

    def recording_target_update():
        events.append("target")
        target_update()

    monkeypatch.setattr(agent, "_target_update", recording_target_update)
    return events


def test_one_gradient_step_runs_utd_critic_updates_and_one_actor_update(monkeypatch):
    agent = _ready_agent(utd=2)
    events = _record_updates(agent, monkeypatch)
    sampled = []
    sample = agent._sample_train_batch
    monkeypatch.setattr(agent, "_sample_train_batch", lambda size: sampled.append(size) or sample(size))

    agent.train(1)

    assert events == ["critic", "target", "critic", "actor", "gripper", "lambda", "target"]
    assert sampled == [agent.batch_size, agent.batch_size]
    assert agent.global_update == 1


def test_gradient_steps_repeat_the_schedule_and_advance_global_update(monkeypatch):
    agent = _ready_agent(utd=3)
    events = _record_updates(agent, monkeypatch)

    agent.train(2)

    assert events.count("critic") == 6
    assert events.count("target") == 6
    assert events.count("actor") == events.count("lambda") == events.count("gripper") == 2
    assert "behavior" not in events
    assert agent.global_update == 2


def test_target_update_moves_both_targets():
    agent = silri_agent(tau=1.0)
    policy = agent.policy
    with torch.no_grad():
        for param in list(policy.actor.parameters()) + list(policy.critic.parameters()):
            param.add_(0.5)
    agent._target_update()
    for online, target in zip(policy.actor.parameters(), policy.actor_target.parameters()):
        torch.testing.assert_close(online, target)
    for online, target in zip(policy.critic.parameters(), policy.critic_target.parameters()):
        torch.testing.assert_close(online, target)


def test_training_batches_mix_online_and_demo_rows():
    agent = silri_agent(batch_size=8)
    agent.init_demo_buffer(buffer_size=32, demo_data_ratio=0.5)
    fill(agent, 8, intervened=False)
    fill(agent, 8, demo=True)
    batch = agent._sample_train_batch(8)
    assert isinstance(batch, InterventionSample)
    assert int(batch.intervened.sum()) == 4
    assert batch.intervention_onset is not None


@pytest.mark.parametrize("rgb", [False, True])
def test_train_end_to_end_reports_every_metric(rgb):
    agent = _ready_agent(rgb=rgb)
    info = agent.train(1, compute_info=True)
    assert set(info) == _TRAIN_INFO_KEYS
    assert all(math.isfinite(value) for value in info.values())
    assert info["batch/intervened_fraction"] == pytest.approx(0.5)
    assert agent.global_update == 1


def test_train_without_gripper_head_has_no_gripper_metric():
    agent = _ready_agent(discrete_gripper=False)
    info = agent.train(1, compute_info=True)
    assert set(info) == _TRAIN_INFO_KEYS - {"discrete/ce"}
    assert agent.train(1) == {}


def test_both_buffers_store_the_intervention_columns():
    agent = silri_agent()
    agent.init_demo_buffer(buffer_size=16)
    for buffer, num_envs in ((agent.replay_buffer, agent.num_envs), (agent.offline_replay_buffer, 1)):
        assert isinstance(buffer, InterventionReplayBuffer)
        assert buffer.store_onset and not buffer.store_base_actions
        assert buffer.num_envs == num_envs
        assert buffer.slice_keys() == agent._extra_batch_slice_keys


def test_learn_is_not_supported():
    with pytest.raises(NotImplementedError, match="SilriLearnerLoop"):
        silri_agent().learn(10)


@pytest.mark.parametrize("bad", [{"utd": 0}, {"utd": 1.5}, {"actor_noise_std": 0.0}])
def test_invalid_hyperparameters_are_rejected(bad):
    with pytest.raises(ValueError):
        silri_agent(**bad)


# -- pretraining and behaviour refreshes ---------------------------------------------------


@pytest.mark.parametrize("init_demo_buffer", [False, True])
def test_pretrain_and_refresh_skip_with_a_warning_without_demo_data(init_demo_buffer):
    agent = silri_agent()
    if init_demo_buffer:
        agent.init_demo_buffer(buffer_size=16)
    with pytest.warns(RuntimeWarning, match="demo buffer is empty"):
        assert agent.pretrain() == {}
    with pytest.warns(RuntimeWarning, match="demo buffer is empty"):
        assert agent.refresh_behavior() == {}
    assert agent.pretrained is False


def test_pretrain_fits_behaviour_and_actor_then_syncs_the_actor_target(monkeypatch):
    agent = _ready_agent(bc_pretrain_steps=3)
    events = _record_updates(agent, monkeypatch)
    behavior_before = [p.detach().clone() for p in agent.policy.behavior_parameters()]
    agent.record_demo_additions(2)

    info = agent.pretrain()

    assert events == ["behavior", "actor", "gripper"] * 3
    assert set(info) == {"behavior/nll", "behavior/sigma_sum", "actor/bc_nll", "discrete/ce"}
    assert agent.pretrained is True
    assert agent.demo_additions_since_refresh == 0
    policy = agent.policy
    for online, target in zip(policy.actor.parameters(), policy.actor_target.parameters()):
        torch.testing.assert_close(online, target)
    assert any(
        not torch.equal(before, after)
        for before, after in zip(behavior_before, policy.behavior_parameters())
    )
    assert agent.global_update == 0


def test_refresh_trains_only_the_behaviour_model_and_resets_the_counter(monkeypatch):
    agent = _ready_agent(behavior_refresh_steps=4, behavior_refresh_every=3)
    events = _record_updates(agent, monkeypatch)
    actor_before = [p.detach().clone() for p in agent.policy.actor_parameters()]

    agent.record_demo_additions(2)
    assert not agent.behavior_refresh_due
    agent.record_demo_additions(1)
    assert agent.behavior_refresh_due

    info = agent.refresh_behavior()

    assert events == ["behavior"] * 4
    assert set(info) == {"behavior/nll", "behavior/sigma_sum"}
    assert agent.demo_additions_since_refresh == 0 and not agent.behavior_refresh_due
    for before, after in zip(actor_before, agent.policy.actor_parameters()):
        torch.testing.assert_close(before, after)


def test_refresh_every_zero_disables_refreshes():
    agent = silri_agent(behavior_refresh_every=0)
    agent.record_demo_additions(1_000)
    assert not agent.behavior_refresh_due


# -- checkpoints ---------------------------------------------------------------------------


def _assert_nested_close(first, second) -> None:
    if isinstance(first, dict):
        assert set(first) == set(second)
        for key in first:
            _assert_nested_close(first[key], second[key])
    elif isinstance(first, (list, tuple)):
        assert len(first) == len(second)
        for a, b in zip(first, second):
            _assert_nested_close(a, b)
    elif isinstance(first, torch.Tensor):
        torch.testing.assert_close(first, second)
    else:
        assert first == second


def test_checkpoint_round_trip_restores_optimizers_and_counters(tmp_path):
    agent = _ready_agent()
    agent.pretrain(steps=1)
    agent.train(1)
    agent.record_demo_additions(2)
    path = agent.save(tmp_path / "checkpoint.pt")

    restored = silri_agent()
    restored.load(path, load_replay_buffer=False)

    for name in _OPTIMIZERS:
        state = getattr(restored, name).state_dict()
        assert state["state"], name
        _assert_nested_close(state, getattr(agent, name).state_dict())
    _assert_nested_close(restored.policy.state_dict(), agent.policy.state_dict())
    assert restored.pretrained is True
    assert restored.demo_additions_since_refresh == 2
    assert restored.global_update == agent.global_update == 1


def test_checkpoint_metadata_names_silri_hyperparameters_only(tmp_path):
    agent = silri_agent(constraint_margin=0.3, behavior_hidden_dims=(16, 8))
    meta = agent._checkpoint_metadata()
    assert meta["constraint_margin"] == 0.3
    assert meta["behavior_hidden_dims"] == [16, 8]
    assert meta["discrete_gripper"] is True
    assert meta["lambda_lr"] == agent.lambda_lr and meta["utd"] == 2
    for unused in ("stddev_schedule", "nstep", "policy_lr", "q_lr", "training_freq"):
        assert unused not in meta
    assert "SiLRI" in type(agent)._compatible_checkpoint_algorithms


def test_two_gripper_agent_trains_and_records_num_grippers():
    agent = _ready_agent(action_dim=TWO_ARM_ACTION_DIM, num_grippers=2)
    info = agent.train(1, compute_info=True)
    assert set(info) == _TRAIN_INFO_KEYS
    assert all(math.isfinite(value) for value in info.values())
    assert agent._checkpoint_metadata()["num_grippers"] == 2
    assert silri_agent()._checkpoint_metadata()["num_grippers"] == 1
