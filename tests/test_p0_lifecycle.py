"""P0 durable admission, startup, and consistent two-buffer recovery."""
import threading
from pathlib import Path

import pytest
import torch

from real_world_rl.loops.actor import ActorLoop
from real_world_rl.loops.durable import DurableStore, encode, decode
from real_world_rl.loops.sync import ActorSyncClient, LearnerSyncServer, PolicySnapshot
from real_world_rl.loops.silri import SilriLearnerLoop
from _silri_fakes import silri_agent
from test_silri_learner_loop import _transition


def make_loop(directory, **kwargs):
    agent = silri_agent()
    agent.init_demo_buffer(32, .5)
    agent.checkpoint_dir = str(directory)
    return SilriLearnerLoop(agent, "127.0.0.1", 0, str(directory), **kwargs)


def test_store_commits_ids_and_pending_across_reopen(tmp_path):
    path = tmp_path / "outbox.sqlite3"
    store = DurableStore(path)
    payload = encode({"x": torch.ones(1)})
    sequence = store.append("stable", payload)
    identity = store.identity
    store.set_pending(payload)
    store.close()
    store = DurableStore(path)
    assert store.identity == identity
    assert store.append("stable", payload) == sequence
    assert len(store) == 1
    assert decode(store.pending())["x"].item() == 1
    with pytest.raises(ValueError, match="different payload"):
        store.append("stable", encode({"x": torch.zeros(1)}))
    store.close()


def test_http_ack_is_journal_only_and_duplicate_is_once(tmp_path):
    loop = make_loop(tmp_path)
    loop._server.start()
    client = ActorSyncClient("http://%s:%s" % loop._server.server_address,
                             outbox_path=str(tmp_path / "outbox.sqlite3"))
    transition = _transition(intervened=True, transition_id="test/robot/session/1")
    try:
        # Admission never takes the replay lock, even while training owns it.
        with loop._lock:
            client.start()
            client.push_transition(transition)
            assert client.stop(timeout=2)["pending"] == 0
        assert len(loop.agent.replay_buffer) == 0
        assert len(loop._journal) == 1
        loop._admit_transition(transition)
        assert len(loop._journal) == 1
        loop._drain_journal()
        assert len(loop.agent.replay_buffer) == 1
        assert len(loop.agent.offline_replay_buffer) == 1
    finally:
        loop._server.stop()
        loop._journal.close()


def test_bundle_restores_two_buffers_and_only_journal_tail(tmp_path):
    first = make_loop(tmp_path)
    first.agent.pretrained = True
    first.agent.demo_additions_since_refresh = 4
    first._policy_ready = True
    first._admit_transition(_transition(intervened=True, transition_id="one"))
    first._drain_journal()
    first._save_bundle()
    saved_additions = first.agent.demo_additions_since_refresh
    first._admit_transition(_transition(intervened=True, transition_id="two"))
    first._journal.close()
    first._server.stop()
    second = make_loop(tmp_path)
    try:
        assert second.restore_checkpoint_bundle()
        assert second.received_transitions == 1
        assert len(second.agent.replay_buffer) == len(second.agent.offline_replay_buffer) == 1
        assert second.agent.pretrained
        assert second._policy_ready
        assert second.agent.demo_additions_since_refresh == saved_additions
        assert second._drain_journal() == 1
        assert second.received_transitions == 2
        assert len(second.agent.replay_buffer) == len(second.agent.offline_replay_buffer) == 2
        assert second.agent.demo_additions_since_refresh == saved_additions + 1
        assert second._drain_journal() == 0
    finally:
        second._server.stop()
        second._journal.close()


def test_restore_failure_does_not_replace_manifest_or_ingest(tmp_path):
    loop = make_loop(tmp_path)
    loop._save_bundle()
    manifest = tmp_path / "bundle_manifest.json"
    original = manifest.read_bytes()
    loop._journal.close()
    loop._server.stop()
    # A different database cannot satisfy the checkpoint's journal identity.
    broken = make_loop(tmp_path, journal_path=str(tmp_path / "other.sqlite3"))
    broken._admit_transition(_transition(transition_id="tail"))
    with pytest.raises(ValueError, match="journal identity"):
        broken.run(total_transitions=0)
    assert manifest.read_bytes() == original
    assert broken.received_transitions == 0
    store = DurableStore(tmp_path / "other.sqlite3")
    assert len(store) == 1
    store.close()


def test_actor_initial_wait_and_robot_close_precede_upload_drain():
    order = []
    class Env:
        num_envs = 1
        def reset(self, seed=None):
            order.append("reset")
            assert "load" in order
            return torch.zeros(1, 2), {}
        def close(self):
            order.append("close")
    class Policy:
        def eval(self): pass
        def load_state_dict(self, state): order.append("load")
    class Client:
        def start(self): order.append("start")
        def wait_for_policy(self): order.append("initial"); return snapshot
        def latest_policy_snapshot(self): return snapshot
        def stop(self): order.append("drain")
    snapshot = PolicySnapshot("session", 1, {}, policy_ready=False, phase="warmup")
    loop = ActorLoop(Env(), Policy(), Client())
    loop.run(total_steps=0)
    assert order == ["start", "initial", "load", "reset", "close", "drain"]
    assert loop.params_version == 1 and not loop.policy_ready


def test_initial_timeout_closes_robot_and_still_stops_client():
    order = []
    class Env:
        num_envs = 1
        def reset(self, seed=None): pytest.fail("reset before initial policy")
        def close(self): order.append("close")
    class Policy:
        def eval(self): pass
    class Client:
        def start(self): pass
        def wait_for_policy(self): raise TimeoutError("initial")
        def stop(self): order.append("drain")
    with pytest.raises(TimeoutError):
        ActorLoop(Env(), Policy(), Client()).run()
    assert order == ["close", "drain"]


def test_learner_publishes_initial_during_demo_warmup(tmp_path):
    loop = make_loop(tmp_path)
    thread = threading.Thread(target=loop.run, daemon=True)
    thread.start()
    client = ActorSyncClient("http://%s:%s" % loop._server.server_address,
                             preflight=False, outbox_path=str(tmp_path / "out.sqlite3"),
                             startup_timeout=2)
    try:
        client.start()
        snapshot = client.wait_for_policy()
        assert snapshot.global_update == 0
        assert not snapshot.policy_ready
        assert snapshot.phase == "warmup"
        assert loop.agent.global_update == 0
    finally:
        client.stop(timeout=2)
        loop.stop()
        thread.join(5)
    assert not thread.is_alive()


def test_held_promotion_rolls_back_insert_and_clear_together(tmp_path):
    import sqlite3
    path = tmp_path / "held.sqlite3"
    store = DurableStore(path)
    held = {"transition_id": "stable-held", "intervention_onset": True}
    store.set_pending(encode(held))
    payload = encode({"kind": "transition", "data": held})
    # Inject a crash-equivalent transaction abort after INSERT, before clear.
    store._connection.execute("CREATE TEMP TRIGGER fail_clear BEFORE DELETE ON metadata "
                              "BEGIN SELECT RAISE(ABORT, 'injected'); END")
    with pytest.raises(sqlite3.IntegrityError, match="injected"):
        store.promote_pending("stable-held", payload)
    assert len(store) == 0
    assert decode(store.pending()) == held
    store.close()
    store = DurableStore(path)
    store.promote_pending("stable-held", payload)
    store.close()
    store = DurableStore(path)
    assert store.pending() is None
    assert len(store) == 1
    assert store.append("stable-held", payload) == store.read()[0][0]
    assert len(store) == 1
    store.close()


def test_fwbw_second_client_start_failure_closes_robot_and_both_clients():
    from real_world_rl.loops.actor import FWBWActorLoop
    order = []
    class Env:
        num_envs = 1
        def reset(self, **kwargs): pytest.fail("reset after startup failure")
        def close(self): order.append("close")
    class Client:
        def __init__(self, name): self.name = name
        def start(self):
            order.append(self.name + "-start")
            if self.name == "backward": raise OSError("startup")
        def stop(self): order.append(self.name + "-stop")
    class Policy:
        def eval(self): pass
    with pytest.raises(OSError, match="startup"):
        FWBWActorLoop(Env(), {k: Policy() for k in ("forward", "backward")},
                      {k: Client(k) for k in ("forward", "backward")}).run()
    assert order[-3:] == ["close", "forward-stop", "backward-stop"]


def test_disk_full_stops_before_next_robot_step(tmp_path, monkeypatch):
    import numpy as np
    from gymnasium import spaces
    order = []
    class Env:
        num_envs = 1
        def __init__(self): self.steps = 0
        def reset(self, seed=None): return torch.zeros(1, 2), {}
        def step(self, action):
            self.steps += 1
            return torch.zeros(1, 2), torch.zeros(1), torch.zeros(1), torch.zeros(1), {}
        def close(self): order.append("close")
    class Policy:
        def eval(self): pass
        def predict(self, obs, deterministic=False): return torch.zeros(1, 1)
        def load_state_dict(self, params): pass
    client = ActorSyncClient("http://127.0.0.1:1", preflight=False,
                             outbox_path=str(tmp_path / "full.sqlite3"))
    snapshot = PolicySnapshot("session", 1, {})
    monkeypatch.setattr(client, "start", lambda: None)
    monkeypatch.setattr(client, "wait_for_policy", lambda: snapshot)
    monkeypatch.setattr(client, "latest_policy_snapshot", lambda: snapshot)
    original_append = client._outbox.append_many
    def full(*args, **kwargs):
        if env.steps:
            raise OSError("disk full")
        return original_append(*args, **kwargs)
    monkeypatch.setattr(client._outbox, "append_many", full)
    env = Env()
    with pytest.raises(OSError, match="disk full"):
        ActorLoop(env, Policy(), client, control_hz=1000000).run(total_steps=5)
    assert env.steps == 1
    assert order == ["close"]
    assert client.stop() == {"pending": 1}  # committed episode_start remains uploadable


def test_silri_advance_commits_previous_and_next_pending_together(tmp_path):
    path = tmp_path / "advance.sqlite3"
    client = ActorSyncClient("http://127.0.0.1:1", outbox_path=str(path), actor_id="robot")
    previous = {"intervened": False, "intervention_onset": False}
    current = {"intervened": True, "intervention_onset": False}
    client.advance_held_transition(None, previous)
    previous["intervention_onset"] = True
    client.advance_held_transition(previous, current)
    assert decode(client._outbox.pending())["transition_id"] == current["transition_id"]
    row = decode(client._outbox.read()[0][2])["data"]
    assert row["transition_id"] == previous["transition_id"]
    assert row["intervention_onset"]
    client.stop(flush=False)
    store = DurableStore(path)
    assert len(store) == 1
    assert decode(store.pending())["transition_id"] == current["transition_id"]
    store.close()

@pytest.mark.parametrize("fwbw", [False, True])
def test_control_resources_close_before_blocking_tail_persistence(fwbw):
    from real_world_rl.loops.actor import FWBWActorLoop
    entered, release = threading.Event(), threading.Event()
    order, errors = [], []
    original = RuntimeError("rollout failure")
    class Env:
        num_envs = 1
        def reset(self, seed=None): return torch.zeros(1, 2), {}
        def step(self, action): raise original
        def close(self): order.append("env-close")
    class Policy:
        def eval(self): pass
        def load_state_dict(self, state): pass
        def predict(self, obs, deterministic=False): return torch.zeros(1, 1)
    class Client:
        def start(self): pass
        def wait_for_policy(self): return snapshot
        def latest_policy_snapshot(self): return snapshot
        def stop(self): order.append("drain")
    class Provider:
        def close(self): order.append("provider-close")
    snapshot = PolicySnapshot("session", 1, {})
    def blocked_tail(*args, **kwargs):
        order.append("persist")
        entered.set()
        assert release.wait(5)
        raise OSError("tail write failed")
    if fwbw:
        loop = FWBWActorLoop(Env(), {key: Policy() for key in ("forward", "backward")},
                            {key: Client() for key in ("forward", "backward")})
        # FWBW owns one archive per direction, each with independent episode data.
        from unittest.mock import patch
        patcher = patch.object(ActorLoop, "_end_trajectory", blocked_tail)
        patcher.start()
    else:
        loop = ActorLoop(Env(), Policy(), Client())
        loop.base_provider = Provider()
        loop._on_run_end = blocked_tail
    def run():
        try: loop.run(total_steps=1)
        except BaseException as exc: errors.append(exc)
    thread = threading.Thread(target=run, daemon=True)
    thread.start()
    try:
        assert entered.wait(3)
        assert order[0] == "env-close"
        if not fwbw: assert order[1] == "provider-close"
        assert "drain" not in order
    finally:
        release.set()
        thread.join(5)
        if fwbw: patcher.stop()
    assert not thread.is_alive()
    assert errors == [original]
    assert order.count("drain") == (2 if fwbw else 1)


def test_partial_demo_write_failure_keeps_bundle_and_replays_once(tmp_path, monkeypatch):
    loop = make_loop(tmp_path)
    loop._save_bundle()
    manifest = tmp_path / "bundle_manifest.json"
    committed = manifest.read_bytes()
    original = OSError("demo write failed after online add")
    def fail_demo(*args, **kwargs): raise original
    monkeypatch.setattr(loop.agent, "add_demo_transition", fail_demo)
    start = loop._server.start
    def start_with_tail():
        start()
        loop._admit_transition(_transition(intervened=True, transition_id="durable-tail"))
    monkeypatch.setattr(loop._server, "start", start_with_tail)
    with pytest.raises(OSError) as error:
        loop.run(total_transitions=1)
    assert error.value is original
    assert loop._ingestion_failed
    assert loop._journal_cursor == 0
    assert len(loop.agent.replay_buffer) == 1
    assert len(loop.agent.offline_replay_buffer) == 0
    assert manifest.read_bytes() == committed
    recovered = make_loop(tmp_path)
    try:
        assert recovered.restore_checkpoint_bundle()
        assert recovered._drain_journal() == 1
        assert recovered._drain_journal() == 0
        assert recovered.received_transitions == 1
        assert len(recovered.agent.replay_buffer) == 1
        assert len(recovered.agent.offline_replay_buffer) == 1
    finally:
        recovered._server.stop()
        recovered._journal.close()

@pytest.mark.parametrize("phase", ["train", "pretrain", "refresh_behavior"])
def test_failed_model_phase_does_not_checkpoint_partial_update(tmp_path, monkeypatch, phase):
    loop = make_loop(tmp_path)
    if phase == "refresh_behavior":
        loop.agent.pretrained = True
        loop.agent.behavior_refresh_every = 1
    loop._save_bundle()
    manifest = tmp_path / "bundle_manifest.json"
    committed = manifest.read_bytes()
    failure = RuntimeError("interrupted optimizer phase")
    original = getattr(loop.agent, phase)
    def partial_phase(*args, **kwargs):
        original(*args, **kwargs)
        raise failure
    monkeypatch.setattr(loop.agent, phase, partial_phase)
    loop._admit_records([_transition(intervened=True, transition_id=f"tail-{n}") for n in range(2)])
    with pytest.raises(RuntimeError) as error:
        loop.run()
    assert error.value is failure
    assert loop._training_failed
    assert manifest.read_bytes() == committed
    recovered = make_loop(tmp_path)
    try:
        assert recovered.restore_checkpoint_bundle()
        assert recovered.agent.global_update == 0
        assert recovered.agent.pretrained == (phase == "refresh_behavior")
        assert recovered._drain_journal() == 2
        assert recovered._drain_journal() == 0
        assert recovered.received_transitions == 2
        assert len(recovered.agent.replay_buffer) == 2
        assert len(recovered.agent.offline_replay_buffer) == 2
    finally:
        recovered._server.stop()
        recovered._journal.close()

@pytest.mark.parametrize("path", ["snapshot", "direct"])
@pytest.mark.parametrize("explicit_termination", [False, True])
def test_legacy_replay_paths_use_projected_termination_and_expert_values(tmp_path, path, explicit_termination):
    import pickle
    from real_world_rl.loops.learner import LearnerLoop
    transition = _transition(intervened=True, onset=True,
        syncing=True, done=torch.ones(1), terminated=torch.zeros(1, dtype=torch.bool),
        truncated=torch.ones(1, dtype=torch.bool),
        human_action_mask=torch.ones(1, 3, dtype=torch.bool))
    if not explicit_termination:
        transition.pop("terminated")
        transition.pop("truncated")
    if path == "snapshot":
        directory = tmp_path / "buffer"
        directory.mkdir()
        with (directory / "transitions_1.pkl").open("wb") as stream:
            pickle.dump([transition], stream)
    loop = make_loop(tmp_path)
    try:
        if path == "direct":
            LearnerLoop._on_transition(loop, transition)
        buffer = loop.agent.replay_buffer
        assert len(buffer) == 1
        assert bool(buffer.dones[0].any()) == (not explicit_termination)
        assert not bool(buffer.intervened[0].any())
        assert not bool(buffer.human_action_mask[0].any())
        assert not bool(buffer.intervention_onset[0].any())
    finally:
        loop._server.stop()
        loop._journal.close()


def test_sync_partial_worker_start_failure_releases_started_worker_and_store(tmp_path, monkeypatch):
    client = ActorSyncClient("http://127.0.0.1:1", preflight=False,
                             outbox_path=str(tmp_path / "startup.sqlite3"))
    failure = RuntimeError("cannot start poll worker")
    def failed_start(): raise failure
    monkeypatch.setattr(client._poll_thread, "start", failed_start)
    with pytest.raises(RuntimeError) as error:
        try:
            client.start()
        finally:
            client.stop(flush=False, timeout=2)
    assert error.value is failure
    assert not client._push_thread.is_alive()
    assert client._outbox._closed
    assert client.stop() == {"pending": 0}

@pytest.mark.parametrize("fwbw", [False, True])
def test_synced_action_bounds_mismatch_fails_before_reset(fwbw):
    from real_world_rl.loops.actor import FWBWActorLoop
    order = []
    class Env:
        num_envs = 1
        def reset(self, seed=None): pytest.fail("action bounds changed before reset")
        def close(self): order.append("close")
    class Policy(torch.nn.Module):
        def __init__(self):
            super().__init__()
            self.register_buffer("action_scale", torch.ones(1))
            self.register_buffer("action_bias", torch.zeros(1))
    class Client:
        def start(self): pass
        def wait_for_policy(self): return snapshot
        def latest_policy_snapshot(self): return snapshot
        def stop(self): order.append("drain")
    snapshot = PolicySnapshot("session", 1,
        {"action_scale": torch.full((1,), 2.0), "action_bias": torch.zeros(1)})
    loop = (FWBWActorLoop(Env(), {key: Policy() for key in ("forward", "backward")},
                         {key: Client() for key in ("forward", "backward")})
            if fwbw else ActorLoop(Env(), Policy(), Client()))
    with pytest.raises(ValueError):
        loop.run(total_steps=0)
    assert order[0] == "close"
    assert order.count("drain") == (2 if fwbw else 1)
