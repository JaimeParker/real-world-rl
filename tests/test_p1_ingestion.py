"""Real blocking training still permits bounded, validated durable HTTP ACKs."""
import threading
import urllib.error
import urllib.request

import pytest
import torch

from real_world_rl.loops.durable import DurableStore, encode, decode
from real_world_rl.loops.sync import ActorSyncClient
from real_world_rl.loops.ingestion import AdmissionBusy
from test_p0_lifecycle import make_loop
from test_silri_learner_loop import _transition


def post(loop, records):
    request = urllib.request.Request("http://%s:%s/records" % loop._server.server_address,
                                     data=encode(records), method="POST")
    with urllib.request.urlopen(request, timeout=2) as response:
        return response.status


@pytest.mark.parametrize("phase", ["train", "pretrain", "refresh_behavior"])
def test_real_training_phase_can_block_while_http_ack_commits(tmp_path, monkeypatch, phase):
    loop = make_loop(tmp_path)
    loop.agent.save_final_checkpoint = False
    entered, release = threading.Event(), threading.Event()
    original = getattr(loop.agent, phase)
    def blocked(*args, **kwargs):
        entered.set()
        assert release.wait(5), "test did not release training phase"
        return original(*args, **kwargs)
    monkeypatch.setattr(loop.agent, phase, blocked)
    if phase == "refresh_behavior":
        loop.agent.pretrained = True
        loop.agent.behavior_refresh_every = 1
    owner_threads = []
    add = loop.agent.replay_buffer.add
    def tracked_add(*args, **kwargs):
        owner_threads.append(threading.get_ident())
        return add(*args, **kwargs)
    monkeypatch.setattr(loop.agent.replay_buffer, "add", tracked_add)
    loop._admit_records([_transition(intervened=True, transition_id=f"seed-{n}") for n in range(2)])
    errors = []
    def run():
        try: loop.run(total_transitions=3)
        except BaseException as exc: errors.append(exc)
    thread = threading.Thread(target=run, daemon=True)
    thread.start()
    try:
        assert entered.wait(5)
        tail = _transition(intervened=True, transition_id="tail")
        assert post(loop, [tail]) == 204
        assert post(loop, [tail]) == 204
        assert len(loop._journal) == 3
        assert len(loop.agent.replay_buffer) == 2
        # No buffer writer runs in a request thread, including during pretraining.
        assert set(owner_threads) == {thread.ident}
    finally:
        release.set()
        loop.stop()
        thread.join(8)
    assert not thread.is_alive() and not errors
    assert loop.received_transitions == 3
    assert len(loop.agent.offline_replay_buffer) == 3
    assert set(owner_threads) == {thread.ident}


def test_batch_validation_is_atomic_and_backpressure_retries_are_idempotent(tmp_path):
    loop = make_loop(tmp_path, admission_capacity=2)
    loop._server.start()
    try:
        first = _transition(transition_id="one")
        invalid = _transition(transition_id="bad")
        invalid["action"] = torch.zeros(1, 99)
        with pytest.raises(urllib.error.HTTPError) as error:
            post(loop, [first, invalid])
        assert error.value.code == 400
        assert len(loop._journal) == 0
        second = _transition(transition_id="two")
        assert post(loop, [first, second]) == 204
        assert post(loop, [first, second]) == 204
        third = _transition(transition_id="three")
        with pytest.raises(urllib.error.HTTPError) as error:
            post(loop, [third])
        assert error.value.code == 503
        assert len(loop._journal) == 2
        loop._drain_journal()
        assert post(loop, [third]) == 204
        loop._drain_journal()
        assert loop.received_transitions == 3
    finally:
        loop._server.stop()
        loop._journal.close()


def test_outbox_capacity_stops_collection_and_hash_is_bound_to_database(tmp_path):
    path = tmp_path / "outbox.sqlite3"
    client = ActorSyncClient("http://127.0.0.1:1", outbox_path=str(path),
                             outbox_max_records=2, experiment_hash="experiment-A")
    client.push_transition(_transition(transition_id="a"))
    client.push_transition(_transition(transition_id="b"))
    with pytest.raises((AdmissionBusy, OSError)):
        client.push_transition(_transition(transition_id="c"))
    assert client.transport_metrics()["outbox_depth"] == 2
    client.stop(flush=False)
    with pytest.raises(ValueError, match="different experiment"):
        ActorSyncClient("http://127.0.0.1:1", outbox_path=str(path), experiment_hash="experiment-B")


def test_actor_restart_emits_durable_interrupted_episode(tmp_path, monkeypatch):
    from real_world_rl.trajectory import begin_episode
    path = tmp_path / "actor.sqlite3"
    first = ActorSyncClient("http://127.0.0.1:1", outbox_path=str(path), preflight=False)
    first.push_record(begin_episode("run/old-session/episode", run_id="run", policy_version=5,
                                    policy_phase="online", policy_session_id="old-policy", global_update=50))
    first.stop(flush=False)
    restarted = ActorSyncClient("http://127.0.0.1:1", outbox_path=str(path), preflight=False)
    # Exercise startup repair without launching network workers.
    monkeypatch.setattr(restarted._push_thread, "start", lambda: None)
    monkeypatch.setattr(restarted._poll_thread, "start", lambda: None)
    restarted.start()
    records = [decode(payload)["data"] for _, _, payload in restarted._outbox.read()]
    end = [record for record in records if record.get("record_type") == "episode_end"]
    assert len(end) == 1 and end[0]["outcome"]["interrupted"]
    assert (end[0]["policy_version"], end[0]["policy_phase"], end[0]["global_update"]) == (5, "online", 50)
    assert restarted._outbox.active_episode() is None
    restarted._started = False
    restarted.stop(flush=False)


def test_episode_metrics_rebuild_from_journal_without_training_projection(tmp_path):
    from real_world_rl.trajectory import begin_episode, end_episode
    from real_world_rl.metrics import EpisodeOutcome
    loop = make_loop(tmp_path)
    records = [begin_episode("e"), end_episode("e", step_count=1,
               outcome=EpisodeOutcome(True, True, 1, 1))]
    loop._admit_records(records)
    loop._drain_journal()
    summary = loop.episode_metrics.summary()
    assert summary["autonomous_success_rate"] == 0
    assert summary["task_success_rate"] == 1
    assert len(loop.agent.replay_buffer) == 0
    loop._save_bundle()
    loop._server.stop(); loop._journal.close()
    restored = make_loop(tmp_path)
    try:
        restored.restore_checkpoint_bundle()
        assert restored.episode_metrics.summary() == summary
        assert restored._drain_journal() == 0
    finally:
        restored._server.stop(); restored._journal.close()


def test_upload_batch_shrinks_to_encoded_byte_limit(tmp_path):
    from real_world_rl.loops.sync import LearnerSyncServer
    batches = []
    server = LearnerSyncServer("127.0.0.1", 0, lambda record: None,
                               on_records=lambda records: batches.append(records),
                               max_payload_bytes=7000)
    client = ActorSyncClient("http://%s:%s" % server.server_address,
                             outbox_path=str(tmp_path / "bytes.sqlite3"),
                             batch_size=32, max_batch_bytes=7000)
    for n in range(8):
        client.push_transition(_transition(transition_id=f"image-{n}"))
    server.start()
    try:
        client.start()
        assert client.stop(timeout=3)["pending"] == 0
        assert sum(map(len, batches)) == 8
        assert len(batches) > 1
        assert all(len(encode(batch)) <= 7000 for batch in batches)
    finally:
        client.stop(flush=False)
        server.stop()


def test_concurrent_duplicate_admission_counts_unique_rows_exactly(tmp_path):
    from concurrent.futures import ThreadPoolExecutor
    loop = make_loop(tmp_path)
    common = _transition(transition_id="shared")
    try:
        with ThreadPoolExecutor(max_workers=8) as pool:
            futures = [pool.submit(loop._admit_records, [common, _transition(transition_id=f"unique-{n}")]) for n in range(16)]
            for future in futures: future.result()
        assert len(loop._journal) == loop._admitted == 17
        assert loop._admission_retries == 15
        assert len(loop.agent.replay_buffer) == 0
    finally:
        loop._server.stop(); loop._journal.close()


def test_admission_semaphore_rejects_before_reading_second_body(tmp_path):
    import http.client
    from real_world_rl.loops.sync import LearnerSyncServer
    entered, release = threading.Event(), threading.Event()
    def blocked(records):
        entered.set()
        assert release.wait(3)
    server = LearnerSyncServer("127.0.0.1", 0, lambda record: None,
                               on_records=blocked, max_concurrent_admissions=1)
    server.start()
    errors = []
    def first():
        try:
            request = urllib.request.Request("http://%s:%s/records" % server.server_address,
                                              data=encode([_transition(transition_id="one")]), method="POST")
            with urllib.request.urlopen(request, timeout=3) as response: assert response.status == 204
        except BaseException as exc: errors.append(exc)
    thread = threading.Thread(target=first)
    thread.start()
    connection = http.client.HTTPConnection(*server.server_address, timeout=1)
    try:
        assert entered.wait(2)
        connection.putrequest("POST", "/records")
        connection.putheader("Content-Length", "1000000")
        connection.endheaders()  # Deliberately send no body.
        assert connection.getresponse().status == 503
    finally:
        connection.close(); release.set(); thread.join(4); server.stop()
    assert not errors


def test_held_and_active_episode_progress_roll_back_together(tmp_path):
    import sqlite3
    store = DurableStore(tmp_path / "active.sqlite3")
    previous, current = encode({"transition_id": "old"}), encode({"transition_id": "new"})
    old_active = {"episode_id": "e", "step_count": 1}
    store.set_pending(previous, active_episode=old_active)
    store._connection.execute("CREATE TEMP TRIGGER fail_active BEFORE INSERT ON metadata "
                              "WHEN NEW.key='active_episode' BEGIN SELECT RAISE(ABORT, 'active failed'); END")
    try:
        with pytest.raises(sqlite3.IntegrityError, match="active failed"):
            store.promote_pending("old", encode({"kind": "transition"}), next_pending=current,
                                  active_episode={"episode_id": "e", "step_count": 2})
        assert len(store) == 0
        assert store.pending() == previous
        assert store.active_episode() == old_active
    finally:
        store.close()


def test_store_runs_wal_normal_and_cached_usage_tracks_table(tmp_path):
    import sqlite3
    path = tmp_path / "usage.sqlite3"
    store = DurableStore(path)

    def table_usage():
        count, size = store._connection.execute(
            "SELECT COUNT(*),COALESCE(SUM(LENGTH(payload)),0) FROM messages").fetchone()
        return {"depth": count, "bytes": size}

    try:
        assert store._connection.execute("PRAGMA journal_mode").fetchone()[0] == "wal"
        assert store._connection.execute("PRAGMA synchronous").fetchone()[0] == 1  # NORMAL
        a, b, c = encode({"x": torch.ones(3)}), encode({"x": torch.ones(5)}), encode({"y": 1})
        store.append_many([("a", a), ("b", b)], max_records=2)
        store.append_many([("a", a)], max_records=2)  # a duplicate retry consumes no capacity
        assert store.metrics() == table_usage() == {"depth": 2, "bytes": len(a) + len(b)}
        with pytest.raises(AdmissionBusy):
            store.append_many([("c", c)], max_records=2)
        with pytest.raises(AdmissionBusy):
            store.append_many([("c", c)], max_bytes=len(a) + len(b) + len(c) - 1)
        store.append("c", c)
        store.append("c", c)
        held = encode({"kind": "transition", "data": {"transition_id": "held"}})
        store.promote_pending("held", held)
        store.promote_pending("held", held)
        store.remove("a")
        store.remove("a")
        store.remove("missing")
        expected = {"depth": 3, "bytes": len(b) + len(c) + len(held)}
        assert store.metrics() == table_usage() == expected
        # A rolled-back promotion leaves the cached usage unchanged.
        store.set_pending(encode({"transition_id": "rolled-back"}))
        store._connection.execute("CREATE TEMP TRIGGER fail_clear BEFORE DELETE ON metadata "
                                  "BEGIN SELECT RAISE(ABORT, 'injected'); END")
        with pytest.raises(sqlite3.IntegrityError, match="injected"):
            store.promote_pending("rolled-back", encode({"kind": "transition", "data": {}}))
        assert store.metrics() == table_usage() == expected
        store.close()
        store = DurableStore(path)
        assert store.metrics() == expected
        rows = store.read()
        assert store.metrics(after=rows[0][0]) == {
            "depth": 2, "bytes": sum(len(payload) for _, _, payload in rows[1:])}
    finally:
        store.close()


def test_silri_seed_demo_preserves_partial_human_masks(tmp_path):
    import pickle
    seed = _transition(intervened=True)
    seed["human_action_mask"] = torch.tensor([[True, False, False]])
    seed["override_mask"] = torch.tensor([[True, True, False]])
    path = tmp_path / "seed.pkl"
    with path.open("wb") as stream: pickle.dump([seed], stream)
    loop = make_loop(tmp_path / "checkpoints", demo_dataset_paths=[str(path)])
    try:
        torch.testing.assert_close(loop.agent.offline_replay_buffer.human_action_mask[0], seed["human_action_mask"])
        torch.testing.assert_close(loop.agent.offline_replay_buffer.override_mask[0], seed["override_mask"])
    finally:
        loop._server.stop(); loop._journal.close()


def test_bundle_initial_snapshot_retains_effective_sdk_and_actor_rejects_before_home(tmp_path, monkeypatch):
    import real_world_rl.devices.claims as claims
    from test_tasks_and_preprocess import single
    from real_world_rl.envs.real_robot import RealRobotVectorEnv
    from real_world_rl.envs.wrappers.experiment import apply_experiment_contract
    from real_world_rl.action_contract import effective_experiment_of
    from real_world_rl.loops.sync import PolicySnapshot
    from real_world_rl.loops.actor import ActorLoop
    monkeypatch.setattr(claims, "_DEFAULT_LOCK_DIR", tmp_path / "claims")
    source_env = apply_experiment_contract(RealRobotVectorEnv(single()))
    source_env.reset()
    effective = effective_experiment_of(source_env)
    source_env.close()
    first = make_loop(tmp_path / "checkpoint")
    first.effective_experiment = effective
    first._save_bundle()
    first._journal.close(); first._server.stop()
    restored = make_loop(tmp_path / "checkpoint")
    try:
        restored.restore_checkpoint_bundle()
        restored._publish_policy()
        version, payload = restored._server._latest_params()
        snapshot = PolicySnapshot(version=version, **decode(payload))
        assert snapshot.effective_experiment == effective
        target = apply_experiment_contract(RealRobotVectorEnv(single()))
        raw = target.env.env
        arm_class = type(raw.robot.arm)
        getter = arm_class.control_metadata.fget
        monkeypatch.setattr(arm_class, "control_metadata", property(lambda arm: {**getter(arm), "sdk_version": "changed-sdk"}))
        moves = []
        monkeypatch.setattr(raw.robot.arm, "move_to_joints", lambda *a, **k: moves.append(True))
        class Client:
            def start(self): pass
            def wait_for_policy(self): return snapshot
            def latest_policy_snapshot(self): return snapshot
            def stop(self): pass
        class Policy:
            def eval(self): pass
            def load_state_dict(self, state): pass
        with pytest.raises(ValueError, match="control/SDK"):
            ActorLoop(target, Policy(), Client()).run(total_steps=0)
        assert moves == []
    finally:
        restored._server.stop(); restored._journal.close()


def test_fwbw_trace_segments_and_partial_human_rows_reach_both_buffers(tmp_path, monkeypatch):
    from real_world_rl.loops.actor import FWBWActorLoop
    from real_world_rl.loops.sync import PolicySnapshot
    class Env:
        num_envs = 1
        def __init__(self): self.steps = 0; self.resets = []
        def reset(self, seed=None):
            self.resets.append(seed)
            return {"state": torch.zeros(1, 4)}, {"fwbw_direction": "forward"}
        def step(self, action):
            self.steps += 1
            direction = "backward" if (self.steps // 2) % 2 else "forward"
            info = {"fwbw_direction": direction}
            if self.steps % 2 == 0: info["task_success"] = True
            if self.steps == 1:
                info.update(intervene_action=torch.full((1, 3), .9),
                            human_action_mask=torch.tensor([[True, False, False]]),
                            override_mask=torch.tensor([[True, False, False]]))
            return {"state": torch.ones(1, 4)}, torch.ones(1), torch.tensor([False]), torch.tensor([False]), info
        def close(self): pass
    class Policy:
        def __init__(self, value): self.value = value
        def eval(self): pass
        def load_state_dict(self, state): pass
        def predict(self, obs, deterministic=False): return torch.full((1, 3), self.value)
    clients = {}
    for direction in ("forward", "backward"):
        client = ActorSyncClient("http://127.0.0.1:1", preflight=False,
                                 outbox_path=str(tmp_path / direction / "outbox.sqlite3"))
        snapshot = PolicySnapshot(direction, 1, {})
        monkeypatch.setattr(client, "start", lambda: None)
        monkeypatch.setattr(client, "wait_for_policy", lambda snapshot=snapshot: snapshot)
        monkeypatch.setattr(client, "latest_policy_snapshot", lambda snapshot=snapshot: snapshot)
        clients[direction] = client
    env = Env()
    FWBWActorLoop(env, {"forward": Policy(.1), "backward": Policy(.2)}, clients,
                  control_hz=1000000).run(total_steps=5)
    assert env.resets == [1]
    traces = {}
    for direction in clients:
        store = DurableStore(tmp_path / direction / "outbox.sqlite3")
        traces[direction] = [decode(payload)["data"] for _, _, payload in store.read()]
        store.close()
    forward = traces["forward"]
    assert [record["record_type"] for record in forward] == ["episode_start", "transition", "transition", "episode_end", "episode_start", "transition", "episode_end"]
    assert forward[2]["segment_end"] and not forward[2]["terminated"]
    from real_world_rl.trajectory import training_projection
    projected = training_projection(forward[2])
    assert bool(torch.as_tensor(projected["episode_end"]).all())
    assert not bool(projected["done"].any())
    assert forward[3]["outcome"]["end_reason"] == "direction_switch"
    assert forward[-1]["outcome"]["interrupted"]
    assert len({record["sequence"] for record in forward}) == len(forward)
    learner = make_loop(tmp_path / "learner")
    try:
        learner._admit_records(forward)
        learner._drain_journal()
        assert len(learner.agent.replay_buffer) == 3
        assert len(learner.agent.offline_replay_buffer) == 1
        torch.testing.assert_close(learner.agent.offline_replay_buffer.human_action_mask[0], torch.tensor([[True, False, False]]))
        torch.testing.assert_close(learner.agent.offline_replay_buffer.actions[0], torch.full((1, 3), .9))
        torch.testing.assert_close(learner.agent.offline_replay_buffer.policy_actions[0], torch.full((1, 3), .1))
    finally:
        learner._server.stop(); learner._journal.close()
