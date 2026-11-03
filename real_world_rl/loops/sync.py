"""Actor<->learner HTTP sync layer for real-robot training.

Self-written, stdlib-only (``http.server`` / ``urllib.request``) -- SERL uses
an external ``agentlace`` package for this, but it is a pip dependency not
vendored anywhere in this repo, so it isn't reused here. Transitions flow
actor -> learner; policy parameters flow learner -> actor. Neither side may
block the other's real-time loop: ``LearnerSyncServer`` accepts a transition
POST and hands it off to a callback immediately, and ``ActorSyncClient``
queues transitions and polls for params on a background thread so the
actor's fixed-frequency control loop never waits on network I/O.
"""
from __future__ import annotations

import io
import json
import pickle
import logging
import socket
import time
import uuid
import tempfile
from pathlib import Path
from dataclasses import dataclass

from real_world_rl.loops.durable import DurableStore, encode, decode
from real_world_rl.loops.ingestion import AdmissionBusy
import threading
import urllib.error
import urllib.parse
import urllib.request
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any, Callable, Optional

import torch

logger = logging.getLogger(__name__)

# Stats messages kept for ``LearnerSyncServer.drain_stats`` when nobody drains them
# (the oldest are dropped beyond this); actors post one message per episode.
_MAX_QUEUED_STATS = 10_000

# Transitions are deserialised with ``torch.load(weights_only=True)``, so a
# payload may only contain tensors and plain containers/primitives (dict,
# list, tuple, str, int, float, bool, None) -- never arbitrary pickled objects.
# Stats travel as JSON on /stats.
_NETWORK_ERRORS = (urllib.error.URLError, ConnectionError, TimeoutError)


class LearnerSyncServer:
    """Runs on the learner process.

    Receives transitions via POST /transition (each call synchronously
    invokes ``on_transition`` on the server's request-handling thread --
    callers that mutate shared state from it are responsible for their own
    locking). Serves the latest published policy params via
    GET /policy_params, versioned so a client only downloads a payload when
    the version differs from the one it already has cached.
    """

    def __init__(
        self,
        host: str,
        port: int,
        on_transition: Callable[[dict[str, Any]], None],
        on_records: Optional[Callable[[list[dict[str, Any]]], None]] = None,
        max_payload_bytes: int = 128 * 1024 * 1024,
        max_concurrent_admissions: int = 2,
    ) -> None:
        self._on_transition = on_transition
        self._on_records = on_records
        if max_payload_bytes < 1 or max_concurrent_admissions < 1:
            raise ValueError("admission body and concurrency limits must be positive")
        self.max_payload_bytes = max_payload_bytes
        self._admission_slots = threading.BoundedSemaphore(max_concurrent_admissions)
        self._params_lock = threading.Lock()
        self._params_bytes: Optional[bytes] = None
        self._params_version = 0
        self.session_id = uuid.uuid4().hex
        self._params_metadata = {}
        self._stats_lock = threading.Lock()
        self._latest_stats: dict[str, dict[str, Any]] = {}
        self._queued_stats: deque[dict[str, Any]] = deque(maxlen=_MAX_QUEUED_STATS)

        handler = _make_handler(self)
        self._httpd = ThreadingHTTPServer((host, port), handler)
        self._thread = threading.Thread(target=self._httpd.serve_forever, daemon=True)

    @property
    def server_address(self) -> tuple[str, int]:
        return self._httpd.server_address

    def start(self) -> None:
        self._thread.start()
        self._started = True

    def stop(self) -> None:
        if getattr(self, "_started", False):
            self._httpd.shutdown()
        self._httpd.server_close()
        if getattr(self, "_started", False):
            self._thread.join(timeout=5.0)

    def publish_params(self, state_dict: dict[str, Any], *, global_update: int = 0, policy_ready: bool = True, phase: str = "online", experiment_hash: str = "unknown", experiment_contract=None, effective_experiment=None) -> None:
        buffer = io.BytesIO()
        torch.save({"state_dict": state_dict, "session_id": self.session_id, "global_update": global_update, "policy_ready": policy_ready, "phase": phase, "experiment_hash": experiment_hash, "experiment_contract": experiment_contract, "effective_experiment": effective_experiment}, buffer)
        with self._params_lock:
            self._params_bytes = buffer.getvalue()
            self._params_version += 1

    def latest_stats(self) -> dict[str, dict[str, Any]]:
        """Most recent stats dict posted by each actor, keyed by ``actor_id``
        (``"unknown"`` when the actor did not stamp one)."""
        with self._stats_lock:
            return {actor_id: dict(stats) for actor_id, stats in self._latest_stats.items()}

    def drain_stats(self) -> list[dict[str, Any]]:
        """Every stats dict posted since the previous call, oldest first, and clears
        them. Independent of :meth:`latest_stats`. Without a drain only the newest
        ``_MAX_QUEUED_STATS`` messages are kept."""
        with self._stats_lock:
            drained = list(self._queued_stats)
            self._queued_stats.clear()
        return drained

    def _record_stats(self, stats: dict[str, Any]) -> None:
        actor_id = str(stats.get("actor_id", "unknown"))
        with self._stats_lock:
            self._latest_stats[actor_id] = stats
            self._queued_stats.append(dict(stats))
        logger.info("actor stats [%s]: %s", actor_id, stats)

    def _latest_params(self) -> tuple[int, Optional[bytes]]:
        with self._params_lock:
            return self._params_version, self._params_bytes


def _make_handler(server: LearnerSyncServer):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, format: str, *args: Any) -> None:
            del format, args  # silence stdlib's default stderr access log

        def do_POST(self) -> None:
            if not server._admission_slots.acquire(blocking=False):
                self.send_response(503)
                self.send_header("Connection", "close")
                self.end_headers()
                self.close_connection = True
                return
            try:
                self.connection.settimeout(5.0)
                self._admit_POST()
            except (TimeoutError, ConnectionError):
                self.close_connection = True
            finally:
                server._admission_slots.release()

        def _admit_POST(self) -> None:
            if self.path not in ("/transition", "/records", "/stats"):
                self.send_response(404)
                self.end_headers()
                return
            try:
                length = int(self.headers.get("Content-Length", 0))
            except ValueError:
                self.send_error(400, "invalid Content-Length")
                return
            if length <= 0 or length > server.max_payload_bytes:
                self.send_error(413, "payload exceeds admission limit")
                return
            body = self.rfile.read(length)
            if self.path == "/stats":
                server._record_stats(json.loads(body))
            else:
                try:
                    transition = torch.load(io.BytesIO(body), map_location="cpu", weights_only=True)
                    records = transition if self.path == "/records" else [transition]
                    if not isinstance(records, list) or not 1 <= len(records) <= 256:
                        raise ValueError("records batch must contain 1..256 records")
                    if server._on_records is not None:
                        server._on_records(records)
                    else:
                        for record in records:
                            server._on_transition(record)
                except (ValueError, EOFError, pickle.UnpicklingError):
                    logger.exception("invalid trajectory admission")
                    self.send_response(400)
                    self.end_headers()
                    return
                except (OSError, RuntimeError):
                    logger.exception("transition admission temporarily unavailable")
                    self.send_response(503)
                    self.end_headers()
                    return
            self.send_response(204)
            self.end_headers()

        def do_GET(self) -> None:
            if not self.path.startswith("/policy_params"):
                self.send_response(404)
                self.end_headers()
                return
            client_version = -1
            if "?" in self.path:
                query = self.path.split("?", 1)[1]
                params = dict(p.split("=", 1) for p in query.split("&") if "=" in p)
                if "version" in params:
                    client_version = int(params["version"])
            version, payload = server._latest_params()
            client_session = params.get("session") if "?" in self.path else None
            if payload is None or (version == client_version and (client_session is None or client_session == server.session_id)):
                self.send_response(204)
                self.send_header("X-Params-Version", str(version))
                self.end_headers()
                return
            self.send_response(200)
            self.send_header("Content-Type", "application/octet-stream")
            self.send_header("X-Params-Version", str(version))
            self.send_header("Content-Length", str(len(payload)))
            self.end_headers()
            self.wfile.write(payload)

    return Handler


_ACTIVE_POLICY_KEYS = ("policy_version", "policy_session_id", "global_update", "policy_phase")


def _episode_progress(active, record):
    human = record.get("human_action_mask")
    controlled = bool(torch.as_tensor(human).any()) if human is not None else bool(record.get("intervened", False))
    return {**active, "step_count": record["step_index"] + 1,
            "sequence": record.get("sequence", 0),
            "intervention_steps": active.get("intervention_steps", 0) + int(controlled),
            **{key: record[key] for key in _ACTIVE_POLICY_KEYS if key in record}}


class ActorSyncClient:
    """Runs on the actor process.

    ``latest_policy_params`` reads a local cache; ``push_transition`` commits
    the local outbox before returning. A background worker owns HTTP uploads, and policy params are polled on a fixed interval
    by a separate background thread and cached locally -- the actor's control
    loop only ever reads the cache, it never makes a network call itself.
    This is required for the actor to hold its fixed control frequency: a
    slow learner (e.g. mid gradient-step) must never stall robot control.

    Messages are committed to a SQLite WAL outbox before returning to the
    control thread. The upload worker retries without dropping rows and removes
    them only after HTTP acknowledgement. Disk write failure stops collection.
    ``max_queue_size`` remains an accepted compatibility argument.

    ``LearnerSyncServer`` has no per-client state -- multiple
    ``ActorSyncClient`` instances (separate processes, e.g. one per robot)
    can already push to and pull from one learner with no protocol changes.
    ``actor_id``, when set, is stamped onto every pushed transition purely
    for observability on the learner side (see ``LearnerLoop.received_by_actor``);
    it carries no routing behavior.

    Failure handling: network errors on push/poll are tolerated (the control
    loop must not stall), but counted across both workers. The first failure
    and every 10th afterwards log a warning with the target host:port; after
    ``max_consecutive_failures`` in a row the client gives up, and the next
    ``push_transition`` / ``latest_policy_params`` call on the actor's thread
    raises ``RuntimeError`` (any success resets the count). ``start()`` also
    runs a one-time TCP preflight so a wrong host/port fails immediately.
    """

    def __init__(
        self,
        learner_url: str,
        poll_interval: float = 1.0,
        timeout: float = 5.0,
        max_queue_size: int = 600,
        actor_id: Optional[str] = None,
        max_consecutive_failures: int = 30,
        preflight: bool = True,
        outbox_path: Optional[str] = None,
        run_id: Optional[str] = None,
        startup_timeout: float = 30.0,
        drain_timeout: float = 30.0,
        batch_size: int = 32,
        max_batch_bytes: int = 64 * 1024 * 1024,
        outbox_max_records: int = 100_000,
        outbox_max_bytes: int = 1024 * 1024 * 1024,
        experiment_hash: Optional[str] = None,
        experiment_contract: Optional[dict] = None,
    ) -> None:
        self._base_url = learner_url.rstrip("/")
        self.session_id = uuid.uuid4().hex
        self._sequence = 0
        self._outbox = DurableStore(outbox_path or Path(tempfile.mkdtemp(prefix="rl-actor-")) / "outbox.sqlite3")
        self.run_id = run_id or self._outbox.identity
        if not 1 <= batch_size <= 256 or outbox_max_records < 1 or outbox_max_bytes < 1 or max_batch_bytes < 1:
            raise ValueError("batch size and outbox capacity must be positive (batch <=256)")
        self.batch_size = batch_size
        self.max_batch_bytes = max_batch_bytes
        self.outbox_max_records = outbox_max_records
        self.outbox_max_bytes = outbox_max_bytes
        self.experiment_hash = experiment_hash or "unknown"
        self.experiment_contract = experiment_contract
        try:
            self._outbox.bind_experiment(self.experiment_hash, experiment_contract)
        except BaseException:
            self._outbox.close()
            raise
        self._retries = 0
        self._sent_records = 0
        self._transport_started = time.monotonic()
        self.startup_timeout = startup_timeout
        self.drain_timeout = drain_timeout
        self._shutdown_result = None
        self._accepting = True
        self._wake_push = threading.Event()
        self._initial_params = threading.Event()
        self._snapshot = None
        self._started = False
        self._shutdown_deadline = None
        parsed = urllib.parse.urlparse(self._base_url)
        self._host = parsed.hostname or ""
        self._port = parsed.port or 80
        self._max_consecutive_failures = max_consecutive_failures
        self._preflight = preflight
        self._failure_lock = threading.Lock()
        self._consecutive_failures = 0
        self._fatal_error: Optional[RuntimeError] = None
        self._timeout = timeout
        self._poll_interval = poll_interval
        self._actor_id = actor_id

        del max_queue_size
        self._dropped_lock = threading.Lock()
        self._dropped = 0
        self._params_lock = threading.Lock()
        self._cached_version = -1
        self._cached_params: Optional[dict[str, Any]] = None

        self._stop_event = threading.Event()
        self._push_thread = threading.Thread(target=self._push_worker, daemon=True)
        self._poll_thread = threading.Thread(target=self._poll_worker, daemon=True)

    def _unreachable_error(self, attempts: int) -> RuntimeError:
        return RuntimeError(
            f"learner unreachable at {self._host}:{self._port} after {attempts} attempts; "
            "check --sync_host/--sync_port and that the learner process is running"
        )

    def _preflight_check(self) -> None:
        try:
            socket.create_connection((self._host, self._port), timeout=2.0).close()
        except OSError as exc:
            raise self._unreachable_error(1) from exc

    def _record_success(self) -> None:
        with self._failure_lock:
            self._consecutive_failures = 0

    def _record_failure(self, exc: BaseException) -> None:
        """Counts a network failure; stores and raises the fatal
        ``RuntimeError`` once ``max_consecutive_failures`` is reached."""
        with self._failure_lock:
            self._consecutive_failures += 1
            count = self._consecutive_failures
            if count == 1 or count % 10 == 0:
                logger.warning(
                    "learner at %s:%s unreachable (%d consecutive failures): %s",
                    self._host, self._port, count, exc,
                )
            if count >= self._max_consecutive_failures:
                self._fatal_error = self._unreachable_error(count)
                raise self._fatal_error

    def _raise_if_fatal(self) -> None:
        if self._fatal_error is not None:
            raise self._fatal_error

    def post_stats(self, stats: dict[str, Any]) -> None:
        """Non-blocking: queues a JSON-serialisable stats dict for the
        learner's ``latest_stats()`` and ``drain_stats()`` (subject to the same
        durable outbox)."""
        self._raise_if_fatal()
        if self._actor_id is not None:
            stats = {**stats, "actor_id": self._actor_id}
        self._put_dropping_oldest(_StatsMessage(stats))

    def bind_experiment(self, experiment_hash: str, experiment_contract=None) -> None:
        self._outbox.bind_experiment(experiment_hash, experiment_contract)
        self.experiment_hash = experiment_hash or "unknown"
        self.experiment_contract = experiment_contract

    def transport_metrics(self) -> dict[str, float]:
        metrics = self._outbox.metrics()
        elapsed = max(.001, time.monotonic() - self._transport_started)
        return {"outbox_depth": metrics["depth"], "outbox_bytes": metrics["bytes"],
                "upload_retries": self._retries, "uploaded_records": self._sent_records,
                "upload_records_per_s": self._sent_records / elapsed}

    def push_record(self, record: dict[str, Any]) -> None:
        self.push_transition(record)

    def start(self) -> None:
        pending = self._outbox.pending()
        if pending is not None:
            self.push_transition(decode(pending))
        active = self._outbox.active_episode()
        if active is not None:
            from real_world_rl.trajectory import end_episode
            from real_world_rl.metrics import EpisodeOutcome
            count = active.pop("step_count", 0)
            intervention_steps = active.pop("intervention_steps", 0)
            episode_id = active.pop("episode_id")
            active["sequence"] = active.get("sequence", 0) + 1
            active["timestamp_s"] = time.time()
            self.push_record(end_episode(episode_id, step_count=count,
                            outcome=EpisodeOutcome(None, intervention_steps > 0, count, intervention_steps, completed=False,
                                                   interrupted=True, end_reason="process_restart"),
                            interrupted=True, end_reason="process_restart", **active))
        if self._preflight:
            self._preflight_check()
        self._started = True
        self._push_thread.start()
        self._poll_thread.start()

    def stop(self, flush: bool = True, timeout: Optional[float] = None) -> dict[str, int]:
        if self._shutdown_result is not None:
            return dict(self._shutdown_result)
        timeout = self.drain_timeout if timeout is None else timeout
        self._accepting = False
        self._shutdown_deadline = time.monotonic() + timeout
        self._stop_event.set()
        self._flush_on_stop = flush
        self._wake_push.set()
        if self._started:
            for thread in (self._poll_thread, self._push_thread):
                # Thread.start can fail after the other worker has started.
                if thread.ident is not None:
                    thread.join(timeout=max(0, self._shutdown_deadline - time.monotonic()))
        remaining = len(self._outbox)
        if remaining:
            logger.warning("%d unacknowledged messages retained in %s", remaining, self._outbox.path)
        self._shutdown_result = {"pending": remaining}
        def release_store():
            if self._push_thread.ident is not None:
                self._push_thread.join()
            self._outbox.close()
        if self._started and self._push_thread.is_alive():
            threading.Thread(target=release_store, daemon=True).start()
        else:
            self._outbox.close()
        return dict(self._shutdown_result)

    def wait_for_policy(self, timeout: Optional[float] = None) -> "PolicySnapshot":
        timeout = self.startup_timeout if timeout is None else timeout
        deadline = time.monotonic() + timeout
        while not self._initial_params.wait(min(0.05, max(0, deadline - time.monotonic()))):
            self._raise_if_fatal()
            if time.monotonic() >= deadline:
                raise TimeoutError("initial learner policy was not available before startup deadline")
        return self.latest_policy_snapshot()

    def latest_policy_snapshot(self) -> Optional["PolicySnapshot"]:
        self._raise_if_fatal()
        with self._params_lock:
            return self._snapshot

    @property
    def dropped_transitions(self) -> int:
        with self._dropped_lock:
            return self._dropped

    def _put_dropping_oldest(self, item: Any) -> None:
        # Historical private name retained; reliable delivery never drops rows.
        if not self._accepting:
            raise RuntimeError("actor sync client is closing")
        message_id = self._new_message_id() if isinstance(item, _StatsMessage) else (item.get("record_id") or item.get("transition_id") or self._new_message_id())
        if isinstance(item, _StatsMessage):
            envelope = {"kind": "stats", "data": item.stats}
        else:
            item = {"experiment_hash": self.experiment_hash, **item, "transition_id": message_id}
            envelope = {"kind": "transition", "data": item}
        pending = self._outbox.pending()
        if pending is not None and decode(pending).get("transition_id") == message_id:
            self._outbox.promote_pending(message_id, encode(envelope))
        else:
            options = {}
            if not isinstance(item, _StatsMessage):
                kind = item.get("record_type")
                if kind == "episode_start":
                    options["active_episode"] = {key: item[key] for key in ("episode_id", "run_id", "actor_id", "session_id", "sequence", "experiment_hash", *_ACTIVE_POLICY_KEYS) if key in item}
                elif kind == "episode_end":
                    options["active_episode"] = None
                elif kind == "transition" and self._outbox.active_episode() is not None:
                    options["active_episode"] = _episode_progress(self._outbox.active_episode(), item)
            self._outbox.append_many([(message_id, encode(envelope))], max_records=self.outbox_max_records,
                                     max_bytes=self.outbox_max_bytes, **options)
        self._wake_push.set()

    def _new_message_id(self) -> str:
        self._sequence += 1
        return f"{self.run_id}/{self._actor_id or 'unknown'}/{self.session_id}/{self._sequence}"

    def _check_outbox_capacity(self, extra: bytes) -> None:
        usage = self._outbox.metrics()
        if usage["depth"] + 1 > self.outbox_max_records or usage["bytes"] + len(extra) > self.outbox_max_bytes:
            raise OSError("actor outbox capacity exhausted; collection stopped")

    def hold_transition(self, transition: dict[str, Any]) -> None:
        if "transition_id" not in transition:
            transition["transition_id"] = self._new_message_id()
        self._check_outbox_capacity(encode(transition))
        self._outbox.set_pending(encode(transition))

    def advance_held_transition(self, previous: Optional[dict[str, Any]], current: dict[str, Any]) -> None:
        """Commit the completed predecessor and delayed successor together."""
        if "transition_id" not in current:
            current["transition_id"] = self._new_message_id()
        self._check_outbox_capacity(encode(current))
        active = self._outbox.active_episode()
        active_options = {}
        if active is not None and "step_index" in current:
            active_options["active_episode"] = _episode_progress(active, current)
        if previous is None:
            self._outbox.set_pending(encode(current), **active_options)
            return
        previous = dict(previous)
        if self._actor_id is not None:
            previous["actor_id"] = self._actor_id
        self._outbox.promote_pending(previous["transition_id"],
                                     encode({"kind": "transition", "data": previous}),
                                     next_pending=encode(current), **active_options)
        self._wake_push.set()

    def clear_held_transition(self) -> None:
        self._outbox.set_pending(None)

    def push_transition(self, transition: dict[str, Any]) -> None:
        if self._actor_id is not None:
            transition = {**transition, "actor_id": self._actor_id}
        self._put_dropping_oldest(transition)
        self._raise_if_fatal()

    def latest_policy_params(self) -> Optional[dict[str, Any]]:
        """Returns the most recently cached params, or ``None`` before the
        first successful poll. Never blocks or makes a network call."""
        self._raise_if_fatal()
        with self._params_lock:
            return self._cached_params

    def _push_worker(self) -> None:
        while True:
            closing = self._stop_event.is_set()
            if closing and (not getattr(self, "_flush_on_stop", True) or time.monotonic() >= self._shutdown_deadline):
                return
            rows = self._outbox.read(limit=self.batch_size, max_bytes=self.max_batch_bytes)
            if not rows:
                if closing:
                    return
                self._wake_push.wait(0.1)
                self._wake_push.clear()
                continue
            first = decode(rows[0][2])
            if first["kind"] == "stats":
                rows = rows[:1]
                path, data = "/stats", json.dumps(first["data"]).encode()
            else:
                selected, records = [], []
                for row in rows:
                    item = decode(row[2])
                    if item["kind"] == "stats":
                        break
                    selected.append(row)
                    records.append(item["data"])
                rows = selected
                path, data = "/records", encode(records)
                while len(data) > self.max_batch_bytes and len(rows) > 1:
                    rows = rows[:max(1, len(rows) // 2)]
                    records = records[:len(rows)]
                    data = encode(records)
                if len(data) > self.max_batch_bytes:
                    self._fatal_error = RuntimeError(f"single durable record requires {len(data)} upload bytes, exceeding max_batch_bytes={self.max_batch_bytes}; record retained in {self._outbox.path}")
                    return
            try:
                timeout = self._timeout
                if closing:
                    timeout = min(timeout, max(0.01, self._shutdown_deadline - time.monotonic()))
                req = urllib.request.Request(f"{self._base_url}{path}", data=data, method="POST")
                with urllib.request.urlopen(req, timeout=timeout) as resp:
                    resp.read()
                for _, message_id, _ in rows:
                    self._outbox.remove(message_id)
                self._sent_records += len(rows)
                self._record_success()
            except _NETWORK_ERRORS as exc:
                self._retries += 1
                if isinstance(exc, urllib.error.HTTPError) and 400 <= exc.code < 500 and exc.code not in (408, 429):
                    self._fatal_error = RuntimeError(f"learner rejected durable records: HTTP {exc.code}; unsent records retained in {self._outbox.path}")
                    return
                try:
                    self._record_failure(exc)
                except RuntimeError:
                    return
                self._wake_push.wait(0.1)
                self._wake_push.clear()

    def _poll_worker(self) -> None:
        while not self._stop_event.is_set():
            try:
                self._poll_once()
            except RuntimeError:
                return
            self._stop_event.wait(self._poll_interval)

    def _poll_once(self) -> None:
        try:
            req = urllib.request.Request(
                f"{self._base_url}/policy_params?version={self._cached_version}&session={getattr(self._snapshot, 'session_id', '')}"
            )
            with urllib.request.urlopen(req, timeout=self._timeout) as resp:
                version = int(resp.headers.get("X-Params-Version", self._cached_version))
                if resp.status == 204:
                    self._cached_version = version
                    self._record_success()
                    return
                body = resp.read()
                params = torch.load(io.BytesIO(body), map_location="cpu", weights_only=True)
            with self._params_lock:
                self._cached_version = version
                if "state_dict" in params and "session_id" in params:
                    self._snapshot = PolicySnapshot(version=version, **params)
                    self._cached_params = params["state_dict"]
                else:
                    self._cached_params = params
                    self._snapshot = PolicySnapshot("legacy", version, params)
                self._initial_params.set()
            self._record_success()
        except _NETWORK_ERRORS as exc:
            self._record_failure(exc)  # keeps the previously cached params until fatal


class _StatsMessage:
    __slots__ = ("stats",)

    def __init__(self, stats: dict[str, Any]) -> None:
        self.stats = stats


@dataclass(frozen=True)
class PolicySnapshot:
    session_id: str
    version: int
    state_dict: dict[str, Any]
    global_update: int = 0
    policy_ready: bool = True
    phase: str = "online"
    experiment_hash: str = "unknown"
    experiment_contract: Optional[dict] = None
    effective_experiment: Optional[dict] = None
