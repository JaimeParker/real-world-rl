"""Regression checks for data review lifecycle and optimizer state."""
import pytest
import torch
from _silri_fakes import hand_batch, silri_agent
from test_reward_collect_real_robot import CameraEnv, _wrapped, _reading, _args
from real_world_rl.reward.collect_real_robot import LabelledFrames, collect_classifier_frames
from real_world_rl.demos.record import hold_action_fn_for


def test_empty_pretraining_keeps_existing_adam_parameters_and_steps(monkeypatch):
    agent = silri_agent()
    batch = hand_batch([True, True])
    monkeypatch.setattr(agent, "_demo_buffer_ready", lambda phase: True)
    monkeypatch.setattr(agent, "_demo_batch", lambda: batch)
    agent.pretrain(1)
    params = [p.detach().clone() for p in agent.policy.parameters()]
    optimizers = (agent.actor_optimizer, agent.discrete_optimizer, agent.behavior_optimizer)
    steps = [[state["step"].clone() for state in optimizer.state.values()]
             for optimizer in optimizers]
    batch.human_action_mask = torch.zeros_like(batch.actions, dtype=torch.bool)
    agent.pretrain(1)
    for param, previous in zip(agent.policy.parameters(), params):
        torch.testing.assert_close(param, previous, rtol=0, atol=0)
    for optimizer, previous_steps in zip(optimizers, steps):
        for state, previous in zip(optimizer.state.values(), previous_steps):
            torch.testing.assert_close(state["step"], previous, rtol=0, atol=0)


def test_collector_shared_outcomes_do_not_count_label_only_as_intervention(tmp_path, capsys):
    env = _wrapped(CameraEnv(), [_reading("success", intervened=False, episode_end=True)])
    try:
        frames = collect_classifier_frames(env, _args(tmp_path, successes_needed=1),
                                           hold_action_fn=hold_action_fn_for(env))
    finally:
        env.close()
    summary = frames.metrics.summary()
    assert summary["completed_episodes"] == 1
    assert summary["task_success_rate"] == summary["autonomous_success_rate"] == 1
    assert summary["step_intervention_rate"] == 0
    assert "autonomous_success_rate" in capsys.readouterr().out


def test_collector_interrupt_reports_partial_attempt(tmp_path):
    env = _wrapped(CameraEnv(interrupt_step=2), [_reading("success"), _reading()])
    frames = LabelledFrames()
    try:
        with pytest.raises(KeyboardInterrupt):
            collect_classifier_frames(env, _args(tmp_path, successes_needed=5),
                                      hold_action_fn=hold_action_fn_for(env), frames=frames)
    finally:
        env.close()
    assert frames.metrics.summary()["interrupted_episodes"] == 1
    assert frames.metrics.summary()["completed_episodes"] == 0


@pytest.mark.parametrize("explicit", [None, "intervened", "human_action_mask", "partial_mask"])
def test_masked_bc_legacy_resume_preserves_supervision_loss_gradients_and_updates(tmp_path, monkeypatch, explicit):
    from rl_garden.algorithms.bc import BC
    from rl_garden.common.checkpoint import replay_buffer_path_for_checkpoint
    from real_world_rl.algorithms.bc import MaskedBC
    from _silri_fakes import SpacesEnv
    kwargs = dict(device="cpu", buffer_device="cpu", buffer_size=8,
                  batch_size=2, std_log=False, log_freq=0)
    legacy = BC(SpacesEnv(), **kwargs)
    obs = {"state": torch.ones(1, 4)}
    legacy.replay_buffer.add(obs, obs, torch.full((1, 3), 0.4), torch.ones(1), torch.zeros(1))
    path = tmp_path / "legacy.pt"
    legacy.save(path, include_replay_buffer=True)
    replay_path = replay_buffer_path_for_checkpoint(path)
    if explicit is not None:
        state = torch.load(replay_path, weights_only=False)
        key = "human_action_mask" if explicit == "partial_mask" else explicit
        state[key] = (torch.zeros_like(state["actions"], dtype=torch.bool) if key == "human_action_mask"
                      else torch.zeros(state["actions"].shape[:-1], dtype=torch.bool))
        if explicit == "partial_mask":
            state[key][:state["pos"], ..., 0] = True
        torch.save(state, replay_path)
    restored = MaskedBC(SpacesEnv(), **kwargs)
    with pytest.warns(UserWarning, match="Legacy BC replay retains offline supervision intent"):
        restored.load(path)
    assert len(restored.replay_buffer) == 1
    assert restored.replay_buffer.legacy_provenance_unknown
    data = restored.replay_buffer.sample(2)
    monkeypatch.setattr(restored, "_sample_train_batch", lambda: data)
    old_loss, _ = legacy._compute_losses(data)
    new_loss, _ = restored._compute_losses(data)
    if explicit is None:
        assert data.human_action_mask.all()
        torch.testing.assert_close(new_loss, old_loss)
        old_grads = torch.autograd.grad(old_loss, tuple(legacy.policy.parameters()), allow_unused=True)
        new_grads = torch.autograd.grad(new_loss, tuple(restored.policy.parameters()), allow_unused=True)
        for old, new in zip(old_grads, new_grads):
            if old is not None:
                torch.testing.assert_close(old, new)
    elif explicit == "partial_mask":
        assert data.human_action_mask[..., 0].all()
        assert not data.human_action_mask[..., 1:].any()
    else:
        assert not data.human_action_mask.any()
        assert new_loss.item() == 0
    before = [p.detach().clone() for p in restored.policy.parameters()]
    restored.train(1)
    changed = any(not torch.equal(p, old) for p, old in zip(restored.policy.parameters(), before))
    assert changed is (explicit in (None, "partial_mask"))
    assert bool(restored.actor_optimizer.state) is (explicit in (None, "partial_mask"))


def test_direct_unknown_native_replay_does_not_inherit_bc_supervision(tmp_path):
    from rl_garden.algorithms.bc import BC
    from rl_garden.common.checkpoint import replay_buffer_path_for_checkpoint
    from real_world_rl.algorithms.bc import MaskedBC
    from _silri_fakes import SpacesEnv
    kwargs = dict(device="cpu", buffer_device="cpu", buffer_size=8,
                  batch_size=2, std_log=False, log_freq=0)
    original = BC(SpacesEnv(), **kwargs)
    obs = {"state": torch.zeros(1, 4)}
    original.replay_buffer.add(obs, obs, torch.zeros(1, 3), torch.zeros(1), torch.zeros(1))
    path = tmp_path / "unknown.pt"
    original.save(path, include_replay_buffer=True)
    restored = MaskedBC(SpacesEnv(), **kwargs)
    with pytest.warns(UserWarning, match="Legacy replay lacks intervention provenance"):
        restored.load_replay_buffer(replay_buffer_path_for_checkpoint(path))
    assert not restored.replay_buffer.human_action_mask.any()
