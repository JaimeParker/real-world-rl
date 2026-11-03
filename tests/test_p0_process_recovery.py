"""Crash and publication fault injection for the local P0 durability boundary."""
from __future__ import annotations

import json
import os
import subprocess
import sys
import threading
from pathlib import Path

import numpy as np
import pytest
import torch
from gymnasium import spaces
from rl_garden.buffers.replay_buffer import ReplayBuffer
from rl_garden.common.checkpoint import replay_buffer_state_dict
from real_world_rl.buffers.intervention_replay_buffer import InterventionReplayBuffer
from real_world_rl.loops.checkpoint import save_bundle, restore_bundle
from real_world_rl.loops.durable import DurableStore, encode, decode
from real_world_rl.loops.learner import LearnerLoop
from real_world_rl.loops.replay_state import restore_buffer


def buffer(cls=ReplayBuffer):
    return cls(spaces.Dict({"state": spaces.Box(-1, 1, (2,), dtype=np.float32)}),
               spaces.Box(-1, 1, (2,), dtype=np.float32), num_envs=1, buffer_size=8,
               storage_device="cpu", sample_device="cpu")


def transition(index):
    return dict(obs={"state": torch.full((1, 2), float(index))},
                next_obs={"state": torch.full((1, 2), float(index + 1))},
                action=torch.full((1, 2), float(index)), reward=torch.tensor([float(index)]),
                done=torch.tensor([False]), transition_id=f"actor/episode/{index}", actor_id="actor")


class TinyAgent:
    def __init__(self):
        self.global_update = 0
        self.replay_buffer = buffer()
        self.offline_replay_buffer = None
        self.buffer_device = "cpu"

    def save(self, path, include_replay_buffer=False):
        torch.save({"global_update": self.global_update}, path)

    def load(self, path, load_replay_buffer=False):
        self.global_update = torch.load(path, weights_only=True)["global_update"]


def loop(agent, store):
    # No HTTP server or robot is needed to exercise admission, cursor and replay.
    result = LearnerLoop.__new__(LearnerLoop)
    result.agent = agent
    result._journal = store
    result._journal_cursor = 0
    result._lock = threading.Lock()
    result._received = 0
    result._received_by_actor = {}
    import time
    from real_world_rl.metrics import EpisodeMetrics
    result.ingest_batch_size = 256
    result.admission_capacity = 10000
    result._observation_space = result._action_space = None
    result.experiment_hash = "unknown"
    result.experiment_contract = result.effective_experiment = None
    result._admitted = 0
    result._admission_started = time.monotonic()
    result._last_ingest_timestamp = None
    result.episode_metrics = EpisodeMetrics()
    result._episode_starts = {}
    result._admission_metrics_lock = threading.Lock()
    result._admission_retries = 0
    result._updates_at_start = result._received_at_start = 0
    return result


@pytest.mark.parametrize("kind", ["outbox", "journal"])
def test_committed_sqlite_survives_real_process_exit(tmp_path, kind):
    path = tmp_path / f"{kind}.sqlite3"
    script = '''
import os, sys
from real_world_rl.loops.durable import DurableStore, encode
from real_world_rl.loops.sync import ActorSyncClient
path, kind = sys.argv[1:]
if kind == "outbox":
    client = ActorSyncClient("http://127.0.0.1:1", preflight=False, outbox_path=path, actor_id="actor")
    client.push_transition({"value": 7})
    client.hold_transition({"value": 8})
else:
    store = DurableStore(path)
    store.append("actor/message/1", encode({"value": 7}))
os._exit(73)
'''
    child = subprocess.run([sys.executable, "-c", script, str(path), kind],
                           cwd=Path(__file__).resolve().parents[1], env=os.environ.copy(),
                           capture_output=True, text=True, timeout=30)
    assert child.returncode == 73, child.stderr
    store = DurableStore(path)
    try:
        rows = store.read()
        assert len(rows) == 1
        assert (decode(rows[0][2])["data"] if kind == "outbox" else decode(rows[0][2]))["value"] == 7
        if kind == "outbox":
            assert decode(store.pending())["value"] == 8
        identity = store.identity
    finally:
        store.close()
    reopened = DurableStore(path)
    try:
        assert reopened.identity == identity
    finally:
        reopened.close()


def test_lost_ack_resend_is_admitted_once_after_restart(tmp_path):
    path = tmp_path / "journal.sqlite3"
    payload = encode(transition(1))
    journal = DurableStore(path)
    first_sequence = journal.append("actor/episode/1", payload)
    journal.close()  # Receiver committed, but sender never saw the ACK.
    journal = DurableStore(path)
    try:
        assert journal.append("actor/episode/1", payload) == first_sequence
        learner = loop(TinyAgent(), journal)
        assert learner._drain_journal() == 1
        assert learner._drain_journal() == 0
        assert learner._received == 1
        assert len(learner.agent.replay_buffer) == 1
    finally:
        journal.close()


def test_failed_bundle_write_keeps_old_manifest_generation(tmp_path, monkeypatch):
    import real_world_rl.loops.checkpoint as checkpoint
    agent = TinyAgent()
    manifest = save_bundle(agent, tmp_path, {"journal_cursor": 1})
    committed = manifest.read_bytes()
    generation = tmp_path / json.loads(committed)["generation"]
    real_save = checkpoint.torch.save

    def fail_replay(value, path, *args, **kwargs):
        if Path(path).name == "replay.pt":
            Path(path).write_bytes(b"partial generation")
            raise OSError("injected disk failure")
        return real_save(value, path, *args, **kwargs)

    agent.global_update = 2
    monkeypatch.setattr(checkpoint.torch, "save", fail_replay)
    with pytest.raises(OSError, match="injected disk failure"):
        save_bundle(agent, tmp_path, {"journal_cursor": 2})
    assert manifest.read_bytes() == committed
    assert (generation / "agent.pt").is_file()
    assert (generation / "replay.pt").is_file()
    target = TinyAgent()
    assert restore_bundle(target, manifest)["journal_cursor"] == 1
    assert target.global_update == 0


def test_bundle_cursor_replays_only_tail_once(tmp_path):
    journal = DurableStore(tmp_path / "journal.sqlite3")
    try:
        first = loop(TinyAgent(), journal)
        first._admit_transition(transition(1))
        first._drain_journal()
        manifest = save_bundle(first.agent, tmp_path / "checkpoints",
                               {"journal_cursor": first._journal_cursor, "received": first._received})
        first._admit_transition(transition(2))
        # Simulate process death after admission, before the new row enters replay.
        recovered = loop(TinyAgent(), journal)
        state = restore_bundle(recovered.agent, manifest)
        recovered._journal_cursor = state["journal_cursor"]
        recovered._received = state["received"]
        assert recovered._drain_journal() == 1
        assert recovered._drain_journal() == 0
        assert recovered._received == 2
        assert len(recovered.agent.replay_buffer) == 2
        torch.testing.assert_close(recovered.agent.replay_buffer.rewards[:2, 0], torch.tensor([1., 2.]))
    finally:
        journal.close()


def test_legacy_hil_core_replay_keeps_unknown_provenance():
    original = buffer(InterventionReplayBuffer)
    row = transition(1)
    original.add(row["obs"], row["next_obs"], row["action"], row["reward"], row["done"], intervened=True)
    restored = buffer(InterventionReplayBuffer)
    with pytest.warns(UserWarning, match="provenance"):
        restore_buffer(restored, replay_buffer_state_dict(original))
    assert len(restored) == 1
    assert restored.legacy_provenance_unknown
    assert not restored.intervened.any()
    torch.testing.assert_close(restored.actions, original.actions)


def test_replay_prevalidation_rejects_partial_schema_before_mutation():
    from real_world_rl.loops.replay_state import serialize_buffer, validate_buffer
    original = buffer()
    target = buffer()
    state = serialize_buffer(original)
    state["obs"]["state"] = torch.zeros(1, 1, 2)
    with pytest.raises(ValueError, match="shape/dtype"):
        validate_buffer(target, state)
    assert target.pos == 0
    assert not target.full
    assert target.obs["state"].shape == (8, 1, 2)
