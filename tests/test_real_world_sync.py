"""Tests for the actor<->learner HTTP sync layer."""
from __future__ import annotations

import io
import threading
import time
import urllib.error
import urllib.request

import pytest
import torch

from real_world_rl.loops.sync import ActorSyncClient, LearnerSyncServer


def _start_server(on_transition):
    server = LearnerSyncServer("127.0.0.1", 0, on_transition=on_transition)
    server.start()
    host, port = server.server_address
    return server, f"http://{host}:{port}"


def test_push_transition_reaches_learner_callback():
    received = []
    event = threading.Event()

    def on_transition(transition):
        received.append(transition)
        event.set()

    server, url = _start_server(on_transition)
    client = ActorSyncClient(url, poll_interval=10.0)
    try:
        client.push_transition(
            {
                "obs": torch.zeros(1, 4),
                "next_obs": torch.ones(1, 4),
                "action": torch.zeros(1, 2),
                "reward": torch.tensor([1.0]),
                "done": torch.tensor([False]),
            }
        )
        client.start()
        assert event.wait(timeout=5.0), "transition never reached the learner"
    finally:
        client.stop()
        server.stop()

    assert len(received) == 1
    torch.testing.assert_close(received[0]["next_obs"], torch.ones(1, 4))


def test_policy_params_round_trip_and_version_gating():
    server, url = _start_server(on_transition=lambda t: None)
    client = ActorSyncClient(url, poll_interval=10.0)
    try:
        # No params published yet.
        client._poll_once()
        assert client.latest_policy_params() is None

        server.publish_params({"w": torch.tensor([1.0, 2.0])})
        client._poll_once()
        params = client.latest_policy_params()
        assert params is not None
        torch.testing.assert_close(params["w"], torch.tensor([1.0, 2.0]))
        version_after_first_poll = client._cached_version

        # Polling again with no new publish must not change the version.
        client._poll_once()
        assert client._cached_version == version_after_first_poll

        server.publish_params({"w": torch.tensor([3.0, 4.0])})
        client._poll_once()
        torch.testing.assert_close(
            client.latest_policy_params()["w"], torch.tensor([3.0, 4.0])
        )
        assert client._cached_version == version_after_first_poll + 1
    finally:
        server.stop()


def test_push_transition_survives_learner_being_down_temporarily():
    # Client created against a port nobody is listening on; push must not
    # raise (the robot control loop must never crash on a network error).
    client = ActorSyncClient(
        "http://127.0.0.1:1", poll_interval=10.0, timeout=0.5, preflight=False
    )
    client.start()
    try:
        client.push_transition({"obs": torch.zeros(1, 2)})
        client._poll_once()  # must not raise either
        assert client.latest_policy_params() is None
    finally:
        client.stop()


def test_push_transition_preserves_all_rows_beyond_legacy_queue_limit():
    from real_world_rl.loops.durable import decode
    client = ActorSyncClient("http://127.0.0.1:1", max_queue_size=4)
    for seq in range(10):
        client.push_transition({"seq": seq})
    rows = client._outbox.read()
    assert len(rows) == 10
    assert client.dropped_transitions == 0
    assert [decode(row[2])["data"]["seq"] for row in rows] == list(range(10))
    client.stop(flush=False)


def test_stop_does_not_block_when_queue_full():
    # Regression guard for the Item-1 fix: stop()'s sentinel used to be a
    # plain blocking queue.put(None), which could hang forever against a
    # full queue. The learner responds (with a small per-call delay so the
    # queue genuinely backs up under max_queue_size=2), so nothing here
    # depends on network-timeout behavior -- just that stop() returns fast.
    def on_transition(transition):
        time.sleep(0.05)

    server, url = _start_server(on_transition)
    client = ActorSyncClient(url, poll_interval=10.0, max_queue_size=2)
    try:
        client.start()
        for seq in range(8):
            client.push_transition({"seq": seq})

        start = time.perf_counter()
        client.stop()
        elapsed = time.perf_counter() - start
        assert elapsed < 3.0, f"stop() blocked for {elapsed:.2f}s with a full queue"
    finally:
        server.stop()


def _free_closed_port() -> int:
    import socket

    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        return sock.getsockname()[1]


def test_preflight_failure_raises_with_hint():
    port = _free_closed_port()
    client = ActorSyncClient(f"http://127.0.0.1:{port}", poll_interval=10.0)
    with pytest.raises(RuntimeError, match=rf"learner unreachable at 127.0.0.1:{port}") as info:
        client.start()
    assert "--sync_host/--sync_port" in str(info.value)


def test_preflight_passes_against_live_server():
    server, url = _start_server(on_transition=lambda t: None)
    client = ActorSyncClient(url, poll_interval=10.0)
    try:
        client.start()
    finally:
        client.stop()
        server.stop()


def test_consecutive_failures_raise_after_limit_and_success_resets(monkeypatch):
    client = ActorSyncClient(
        "http://127.0.0.1:1", poll_interval=10.0, max_consecutive_failures=3
    )

    def refuse(*args, **kwargs):
        raise urllib.error.URLError("refused")

    monkeypatch.setattr(urllib.request, "urlopen", refuse)
    client._poll_once()
    client._poll_once()
    with pytest.raises(RuntimeError, match="after 3 attempts"):
        client._poll_once()
    # the fatal error is surfaced on the actor's thread too.
    with pytest.raises(RuntimeError, match="learner unreachable at 127.0.0.1:1"):
        client.push_transition({"obs": torch.zeros(1)})
    with pytest.raises(RuntimeError):
        client.latest_policy_params()


def test_success_resets_failure_counter(monkeypatch):
    server, url = _start_server(on_transition=lambda t: None)
    client = ActorSyncClient(url, poll_interval=10.0, max_consecutive_failures=3)
    real_urlopen = urllib.request.urlopen
    try:
        monkeypatch.setattr(
            urllib.request, "urlopen", lambda *a, **k: (_ for _ in ()).throw(ConnectionResetError())
        )
        client._poll_once()
        client._poll_once()
        assert client._consecutive_failures == 2
        monkeypatch.setattr(urllib.request, "urlopen", real_urlopen)
        client._poll_once()
        assert client._consecutive_failures == 0
        client.latest_policy_params()  # no fatal error
    finally:
        server.stop()


def test_learner_outage_past_old_failure_limit_delivers_every_record(tmp_path, monkeypatch):
    import real_world_rl.loops.sync as sync

    # A short back-off cap lets the upload worker itself fail >30 times quickly.
    monkeypatch.setattr(sync, "_MAX_RETRY_DELAY_S", 0.02)
    port = _free_closed_port()
    client = ActorSyncClient(
        f"http://127.0.0.1:{port}", poll_interval=0.05, timeout=0.5, preflight=False,
        outbox_path=str(tmp_path / "outbox.sqlite3"),
    )
    received = []
    server = None
    client.start()
    try:
        for seq in range(5):
            client.push_transition({"seq": seq})
        deadline = time.time() + 10.0
        while client._retries <= 30 and time.time() < deadline:
            time.sleep(0.02)
        assert client._retries > 30, "upload worker never exceeded the old fatal limit"
        assert client._fatal_error is None
        client.push_transition({"seq": 5})
        assert client.latest_policy_params() is None

        server = LearnerSyncServer(
            "127.0.0.1", port, on_transition=lambda t: received.append(t["seq"])
        )
        server.start()
        deadline = time.time() + 10.0
        while len(client._outbox) and time.time() < deadline:
            time.sleep(0.02)
        assert len(client._outbox) == 0, "outbox was not drained after the learner came back"
        assert set(received) == set(range(6))
        client.push_transition({"seq": 6})
        assert client.stop(timeout=5.0) == {"pending": 0}
        assert set(received) == set(range(7))
    finally:
        client.stop(flush=False)
        if server is not None:
            server.stop()


def test_upload_retry_backs_off_exponentially_up_to_cap(tmp_path):
    client = ActorSyncClient(
        f"http://127.0.0.1:{_free_closed_port()}", poll_interval=10.0, timeout=0.5,
        preflight=False, outbox_path=str(tmp_path / "outbox.sqlite3"),
    )
    real_wait = client._stop_event.wait
    delays = []

    def record_backoff(timeout=None):
        if timeout == 10.0:  # the poll worker's interval
            return real_wait(timeout)
        delays.append(timeout)
        return client._stop_event.is_set()

    client._stop_event.wait = record_backoff
    client.push_transition({"seq": 0})
    client.start()
    try:
        deadline = time.time() + 5.0
        while len(delays) < 8 and time.time() < deadline:
            time.sleep(0.01)
        assert delays[:8] == pytest.approx([0.1, 0.2, 0.4, 0.8, 1.6, 2.0, 2.0, 2.0])
        assert client._fatal_error is None
    finally:
        client.stop(flush=False)


def test_stats_round_trip_keeps_latest_per_actor():
    server, url = _start_server(on_transition=lambda t: None)
    client_a = ActorSyncClient(url, poll_interval=10.0, actor_id="a")
    client_b = ActorSyncClient(url, poll_interval=10.0, actor_id="b")
    try:
        client_a.start()
        client_b.start()
        client_a.post_stats({"episode_return": 1.0, "step": 10})
        client_a.post_stats({"episode_return": 2.0, "step": 20})
        client_b.post_stats({"episode_return": 5.0})
        deadline = time.time() + 5.0
        while time.time() < deadline:
            stats = server.latest_stats()
            if stats.get("a", {}).get("step") == 20 and "b" in stats:
                break
            time.sleep(0.02)
        assert stats["a"] == {"episode_return": 2.0, "step": 20, "actor_id": "a"}
        assert stats["b"]["episode_return"] == 5.0
    finally:
        client_a.stop()
        client_b.stop()
        server.stop()


def test_drain_stats_returns_every_posted_message_once_in_order():
    server, url = _start_server(on_transition=lambda t: None)
    client = ActorSyncClient(url, poll_interval=10.0, actor_id="a")
    try:
        client.start()
        client.post_stats({"return": 1.0})
        client.post_stats({"return": 2.0})
        drained: list[dict] = []
        deadline = time.time() + 5.0
        while len(drained) < 2 and time.time() < deadline:
            drained += server.drain_stats()
            time.sleep(0.02)
        assert drained == [{"return": 1.0, "actor_id": "a"}, {"return": 2.0, "actor_id": "a"}]
        assert server.drain_stats() == []
        # latest_stats is independent of draining.
        assert server.latest_stats() == {"a": {"return": 2.0, "actor_id": "a"}}
    finally:
        client.stop()
        server.stop()


def test_transition_with_plain_containers_round_trips_under_weights_only():
    payload = {
        "obs": {"state": torch.zeros(1, 3), "rgb": torch.zeros(1, 2, 2, 3, dtype=torch.uint8)},
        "action": torch.zeros(1, 2),
        "done": torch.tensor([False]),
        "intervened": True,
        "grasp_penalty": 0.5,
        "actor_id": "a",
    }
    buffer = io.BytesIO()
    torch.save(payload, buffer)
    buffer.seek(0)
    loaded = torch.load(buffer, weights_only=True)
    assert loaded["intervened"] is True and loaded["actor_id"] == "a"
    torch.testing.assert_close(loaded["obs"]["state"], payload["obs"]["state"])


def test_server_loads_transitions_with_weights_only(monkeypatch):
    seen = {}
    real_load = torch.load

    def spy(*args, **kwargs):
        seen["weights_only"] = kwargs.get("weights_only")
        return real_load(*args, **kwargs)

    monkeypatch.setattr(torch, "load", spy)
    received = threading.Event()
    server, url = _start_server(lambda t: received.set())
    client = ActorSyncClient(url, poll_interval=10.0)
    try:
        client.push_transition({"obs": torch.zeros(1), "intervened": False})
        client.start()
        assert received.wait(timeout=5.0)
    finally:
        client.stop()
        server.stop()
    assert seen["weights_only"] is True
