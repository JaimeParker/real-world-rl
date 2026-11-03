"""HilSerlLearnerLoop._buffer_kwargs: the default split, overrides that keep
extra fields, and that extras survive a snapshot + reload."""
from __future__ import annotations

import glob
import os
import pickle
from types import SimpleNamespace

import torch

from real_world_rl.loops.hil_serl import HilSerlLearnerLoop


class _RecordingAgent:
    """The surface HilSerlLearnerLoop touches, recording every buffer write."""

    buffer_device = torch.device("cpu")
    learning_starts = 0
    utd = 1

    def __init__(self):
        self.online = []
        self.demo = []
        self.replay_buffer = SimpleNamespace(
            add=lambda *args, **kwargs: self.online.append((args, kwargs))
        )

    def add_demo_transition(self, *args, **kwargs):
        self.demo.append((args, kwargs))


class _KeepIntervenedLoop(HilSerlLearnerLoop):
    """What a residual/SiLRI loop does: keep ``intervened`` and the extras."""

    def _buffer_kwargs(self, transition):
        core, extra = super()._buffer_kwargs(transition)
        extra["intervened"] = transition["intervened"]
        extra["policy_action"] = transition["policy_action"]
        return core, extra


def _transition(intervened=False, **extra) -> dict:
    return {
        "obs": torch.zeros(1, 4),
        "next_obs": torch.ones(1, 4),
        "action": torch.zeros(1, 3),
        "reward": torch.ones(1),
        "done": torch.zeros(1, dtype=torch.bool),
        "intervened": intervened,
        **extra,
    }


def _loop(cls, tmp_path, agent=None, buffer_period=1000):
    return cls(
        agent or _RecordingAgent(),
        "127.0.0.1",
        0,
        checkpoint_dir=str(tmp_path),
        buffer_period=buffer_period,
    )


def test_default_buffer_kwargs_splits_core_from_extra_and_drops_routing_keys(tmp_path):
    loop = _loop(HilSerlLearnerLoop, tmp_path)
    penalty = torch.tensor([-0.05])
    received = _transition(intervened=True, actor_id="a", grasp_penalty=penalty)
    snapshot_of_input = dict(received)

    core, extra = loop._buffer_kwargs(received)

    assert list(core) == ["obs", "next_obs", "action", "reward", "done"]
    assert list(extra) == ["grasp_penalty"] and extra["grasp_penalty"] is penalty
    assert received == snapshot_of_input  # the hook does not mutate its input


def test_default_loop_forwards_only_core_and_extra_and_snapshots_the_same(tmp_path):
    agent = _RecordingAgent()
    loop = _loop(HilSerlLearnerLoop, tmp_path, agent)

    loop._on_transition(_transition(intervened=True, actor_id="a", grasp_penalty=torch.tensor([-0.05])))

    (args, kwargs), = agent.online
    assert len(args) == 5 and list(kwargs) == ["grasp_penalty"]
    assert len(agent.demo) == 1 and list(agent.demo[0][1]) == ["grasp_penalty"]
    assert set(loop._pending_online[0]) == {"obs", "next_obs", "action", "reward", "done", "grasp_penalty"}
    assert loop.received_by_actor == {"a": 1}


def test_override_keeps_extra_fields_and_routing_still_reads_intervened(tmp_path):
    agent = _RecordingAgent()
    loop = _loop(_KeepIntervenedLoop, tmp_path, agent)
    policy_action = torch.full((1, 3), 0.25)

    loop._on_transition(_transition(intervened=False, policy_action=policy_action))
    loop._on_transition(_transition(intervened=True, policy_action=policy_action))

    assert len(agent.online) == 2 and len(agent.demo) == 1
    for _, kwargs in (*agent.online, *agent.demo):
        assert set(kwargs) == {"intervened", "policy_action"}
        torch.testing.assert_close(kwargs["policy_action"], policy_action)
    assert [kwargs["intervened"] for _, kwargs in agent.online] == [False, True]
    assert agent.demo[0][1]["intervened"] is True


def test_extras_survive_snapshot_and_reload(tmp_path):
    agent = _RecordingAgent()
    loop = _loop(_KeepIntervenedLoop, tmp_path, agent, buffer_period=2)
    first, second = torch.full((1, 3), 0.25), torch.full((1, 3), 0.75)

    loop._on_transition(_transition(intervened=False, policy_action=first, actor_id="robot"))
    loop._on_transition(_transition(intervened=True, policy_action=second, actor_id="robot"))

    # Snapshot files hold the extras (and not the observability-only actor_id).
    with open(glob.glob(os.path.join(str(tmp_path), "buffer", "*.pkl"))[0], "rb") as f:
        online = pickle.load(f)
    with open(glob.glob(os.path.join(str(tmp_path), "demo_buffer", "*.pkl"))[0], "rb") as f:
        demo = pickle.load(f)
    assert [t["intervened"] for t in online] == [False, True]
    assert all("actor_id" not in t for t in online + demo)
    torch.testing.assert_close(online[1]["policy_action"], second)
    assert len(demo) == 1 and demo[0]["intervened"] is True

    reloaded_agent = _RecordingAgent()
    reloaded = _loop(_KeepIntervenedLoop, tmp_path, reloaded_agent, buffer_period=2)

    assert reloaded.received_transitions == 2
    assert [kwargs["intervened"] for _, kwargs in reloaded_agent.online] == [False, True]
    torch.testing.assert_close(reloaded_agent.online[0][1]["policy_action"], first)
    torch.testing.assert_close(reloaded_agent.online[1][1]["policy_action"], second)
    assert len(reloaded_agent.demo) == 1
    torch.testing.assert_close(reloaded_agent.demo[0][1]["policy_action"], second)
