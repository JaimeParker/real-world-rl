"""SilriLearnerLoop: two-buffer routing (with and without onset routing), the kept extras,
the training gate (and LearnerLoop's default gate), pretraining once, behaviour-refresh
cadence, the gradient steps per iteration and demo preload."""
from __future__ import annotations

import pickle
import threading
from types import SimpleNamespace

import torch
from _silri_fakes import ACTION_DIM, STATE_DIM, silri_agent
from real_world_rl.loops.hil_serl import HilSerlLearnerLoop
from real_world_rl.loops.learner import LearnerLoop
from real_world_rl.loops.silri import SilriLearnerLoop


def _agent(**overrides):
    agent = silri_agent(**overrides)
    agent.init_demo_buffer(buffer_size=32, demo_data_ratio=0.5)
    return agent


def _loop(agent, tmp_path, **kwargs) -> SilriLearnerLoop:
    return SilriLearnerLoop(
        agent, "127.0.0.1", 0, checkpoint_dir=str(tmp_path), buffer_period=1000, **kwargs
    )


def _transition(intervened=False, onset=False, **extra) -> dict:
    action = torch.rand(1, ACTION_DIM) * 2 - 1
    action[:, -1] = 1.0
    return {
        "obs": {"state": torch.rand(1, STATE_DIM)},
        "next_obs": {"state": torch.rand(1, STATE_DIM)},
        "action": action,
        "reward": torch.ones(1),
        "done": torch.zeros(1),
        "intervened": intervened,
        "policy_action": torch.full((1, ACTION_DIM), 0.25),
        "intervention_onset": onset,
        **extra,
    }


def _counting(monkeypatch, agent, name: str) -> list:
    calls: list = []
    original = getattr(agent, name)

    def wrapper(*args, **kwargs):
        calls.append((args, kwargs))
        return original(*args, **kwargs)

    monkeypatch.setattr(agent, name, wrapper)
    return calls


def test_silri_learner_loop_is_a_hil_serl_learner_loop():
    assert issubclass(SilriLearnerLoop, HilSerlLearnerLoop)


def test_routing_copies_intervened_rows_to_the_demo_buffer(tmp_path):
    agent = _agent()
    loop = _loop(agent, tmp_path)

    loop._on_transition(_transition(intervened=False))
    loop._on_transition(_transition(intervened=False, onset=True))
    loop._on_transition(_transition(intervened=True))

    assert len(agent.replay_buffer) == 3
    assert len(agent.offline_replay_buffer) == 1
    assert agent.demo_additions_since_refresh == 1
    assert agent.replay_buffer.intervened[:3, 0].tolist() == [False, False, True]
    assert agent.replay_buffer.intervention_onset[:3, 0].tolist() == [False, True, False]
    torch.testing.assert_close(
        agent.replay_buffer.policy_actions[0], torch.full((1, ACTION_DIM), 0.25)
    )
    assert loop.received_transitions == 3


def test_onset_rows_reach_the_demo_buffer_only_when_asked(tmp_path):
    agent = _agent(onset_to_demo_buffer=True)
    loop = _loop(agent, tmp_path)

    loop._on_transition(_transition(intervened=False, onset=True))
    loop._on_transition(_transition(intervened=False))

    demo = agent.offline_replay_buffer
    assert len(demo) == 1
    # Still a policy row: it never trains the behaviour model or the gripper head.
    assert demo.intervened[0, 0].item() is False
    assert demo.intervention_onset[0, 0].item() is True
    assert agent.demo_additions_since_refresh == 1


def test_only_silri_extras_reach_the_buffers_and_snapshots(tmp_path):
    agent = _agent()
    loop = _loop(agent, tmp_path)

    loop._on_transition(
        _transition(intervened=True, grasp_penalty=torch.tensor([-0.05]), actor_id="robot")
    )

    expected = {
        "obs",
        "next_obs",
        "action",
        "reward",
        "done",
        "intervened",
        "intervention_onset",
        "policy_action",
    }
    assert set(loop._pending_online[0]) == expected
    assert set(loop._pending_demo[0]) == expected
    assert loop.received_by_actor == {"robot": 1}


def test_snapshots_reload_with_their_extras(tmp_path):
    agent = _agent()
    loop = SilriLearnerLoop(agent, "127.0.0.1", 0, checkpoint_dir=str(tmp_path), buffer_period=2)
    loop._on_transition(_transition(intervened=False, onset=True))
    loop._on_transition(_transition(intervened=True))

    reloaded_agent = _agent()
    reloaded = SilriLearnerLoop(
        reloaded_agent, "127.0.0.1", 0, checkpoint_dir=str(tmp_path), buffer_period=2
    )

    assert reloaded.received_transitions == 2
    assert reloaded_agent.replay_buffer.intervention_onset[:2, 0].tolist() == [True, False]
    assert len(reloaded_agent.offline_replay_buffer) == 1
    # Reloaded rows are not new demo additions.
    assert reloaded_agent.demo_additions_since_refresh == 0


def test_demo_pkls_without_silri_fields_load_as_intervened_rows(tmp_path):
    transitions = [_transition() for _ in range(3)]
    for transition in transitions:
        for key in ("intervened", "policy_action", "intervention_onset"):
            transition.pop(key)
    demo_path = tmp_path / "demos.pkl"
    with open(demo_path, "wb") as f:
        pickle.dump(transitions, f)

    agent = _agent()
    loop = _loop(agent, tmp_path / "ckpt", demo_dataset_paths=[str(demo_path)])

    demo = agent.offline_replay_buffer
    assert len(demo) == 3
    assert demo.intervened[:3, 0].tolist() == [True, True, True]
    assert demo.intervention_onset[:3, 0].tolist() == [False, False, False]
    torch.testing.assert_close(demo.policy_actions[:3], demo.actions[:3])
    assert loop.received_transitions == 0
    assert agent.demo_additions_since_refresh == 0


def test_training_waits_for_both_buffers(tmp_path):
    agent = _agent(learning_starts=2, demo_learning_starts=2)
    loop = _loop(agent, tmp_path)

    loop._on_transition(_transition(intervened=True))
    assert not loop._ready_to_train()  # 1 online row
    loop._on_transition(_transition(intervened=False))
    assert not loop._ready_to_train()  # 2 online rows, 1 demo row
    loop._on_transition(_transition(intervened=True))
    assert loop._ready_to_train()


def test_learner_loop_default_gate_is_learning_starts_received_transitions():
    agent = SimpleNamespace(learning_starts=2)
    loop = LearnerLoop.__new__(LearnerLoop)
    loop.agent = agent
    loop._lock = threading.Lock()
    loop._received = 1
    assert not loop._ready_to_train()
    loop._received = 2
    assert loop._ready_to_train()


def _ready_loop(tmp_path, **overrides):
    agent = _agent(**overrides)
    loop = _loop(agent, tmp_path)
    for intervened in (True, False, True, False):
        loop._on_transition(_transition(intervened=intervened))
    assert loop._ready_to_train()
    return agent, loop


def test_pretrain_runs_once_before_the_first_train_step(tmp_path, monkeypatch):
    agent, loop = _ready_loop(tmp_path)
    pretrain_calls = _counting(monkeypatch, agent, "pretrain")
    train_calls = _counting(monkeypatch, agent, "train")
    order: list[str] = []
    monkeypatch.setattr(agent, "_target_update", lambda: order.append("train"))
    original_sync = agent.policy.sync_actor_target
    monkeypatch.setattr(
        agent.policy, "sync_actor_target", lambda: (order.append("pretrain"), original_sync())
    )

    loop._train_step()
    loop._train_step()

    assert len(pretrain_calls) == 1 and agent.pretrained
    assert len(train_calls) == 2
    assert order[0] == "pretrain" and order.count("pretrain") == 1


def test_a_restored_pretrained_agent_is_not_pretrained_again(tmp_path, monkeypatch):
    agent, loop = _ready_loop(tmp_path)
    agent.pretrained = True
    pretrain_calls = _counting(monkeypatch, agent, "pretrain")

    loop._train_step()

    assert pretrain_calls == []


def test_pretrain_is_skipped_when_the_demo_buffer_is_empty(tmp_path, monkeypatch):
    agent = _agent(demo_learning_starts=0)
    loop = _loop(agent, tmp_path)
    loop._on_transition(_transition(intervened=False))
    loop._on_transition(_transition(intervened=False))
    pretrain_calls = _counting(monkeypatch, agent, "pretrain")
    monkeypatch.setattr(agent, "train", lambda steps, compute_info=False: {})

    loop._train_step()

    assert pretrain_calls == [] and not agent.pretrained


def test_behaviour_refresh_follows_demo_additions(tmp_path, monkeypatch):
    agent, loop = _ready_loop(tmp_path, behavior_refresh_every=3)
    loop._train_step()  # pretrains; that resets the addition counter
    refresh_calls = _counting(monkeypatch, agent, "refresh_behavior")

    loop._on_transition(_transition(intervened=True))
    loop._on_transition(_transition(intervened=True))
    loop._on_transition(_transition(intervened=False))
    loop._train_step()
    assert refresh_calls == []

    loop._on_transition(_transition(intervened=True))
    loop._train_step()
    assert len(refresh_calls) == 1
    assert agent.demo_additions_since_refresh == 0


def test_each_iteration_runs_train_freq_gradient_steps(tmp_path, monkeypatch):
    agent = _agent(utd=2)
    loop = SilriLearnerLoop(
        agent, "127.0.0.1", 0, checkpoint_dir=str(tmp_path), buffer_period=1000, train_freq=3
    )
    for intervened in (True, False, True, False):
        loop._on_transition(_transition(intervened=intervened))
    agent.pretrained = True
    train_calls = _counting(monkeypatch, agent, "train")

    loop._train_step()

    assert train_calls[0][0] == (3,)
    assert agent.global_update == 3


class _RecordingLogger:
    def __init__(self):
        self.scalars: list[tuple[str, float, int]] = []

    def add_scalar(self, tag, value, step):
        self.scalars.append((tag, value, step))

    def at(self, prefix: str) -> dict[str, tuple[float, int]]:
        return {tag: (value, step) for tag, value, step in self.scalars if tag.startswith(prefix)}


def test_behaviour_phases_are_logged_when_they_run(tmp_path):
    agent, loop = _ready_loop(tmp_path, behavior_refresh_every=2)
    logger = _RecordingLogger()
    agent.logger = logger  # log_freq=0: only the behaviour phases log

    loop._train_iteration(loop.received_transitions)

    logged = logger.at("behavior/")
    assert set(logged) == {
        "behavior/pretrained",
        "behavior/pretrain/behavior_nll",
        "behavior/pretrain/behavior_sigma_sum",
        "behavior/pretrain/actor_bc_nll",
        "behavior/pretrain/discrete_ce",
    }
    assert logged["behavior/pretrained"] == (1.0, 4)
    assert loop._learner_scalars()["intervened_received"] == 2

    logger.scalars.clear()
    loop._train_iteration(loop.received_transitions)
    assert logger.scalars == []  # pretraining is considered once, no refresh due

    loop._on_transition(_transition(intervened=True))
    loop._on_transition(_transition(intervened=True))
    loop._train_iteration(loop.received_transitions)

    logged = logger.at("behavior/")
    assert set(logged) == {
        "behavior/refresh/behavior_nll",
        "behavior/refresh/behavior_sigma_sum",
        "behavior/refresh_count",
    }
    assert logged["behavior/refresh_count"] == (1.0, 6)


def test_a_restored_pretrained_agent_logs_the_flag_without_pretrain_losses(tmp_path):
    agent, loop = _ready_loop(tmp_path)
    agent.pretrained = True
    agent.logger = _RecordingLogger()

    loop._train_iteration(loop.received_transitions)

    assert agent.logger.at("behavior/") == {"behavior/pretrained": (1.0, 4)}
