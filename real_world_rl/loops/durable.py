"""Local SQLite durability for actor delivery and learner ingestion.

Use a local filesystem: WAL's locking protocol is not suitable for network shares.
Stores run in WAL mode with synchronous=NORMAL, so a commit does not fsync (the actor
commits one row per robot step). A successful append survives a crash of this process;
an OS crash or power loss can roll back the most recent commits.

Each file must have a single writing ``DurableStore``: the instance caches its message
count and payload bytes for capacity checks instead of scanning the table per append.
"""
from __future__ import annotations

import io
import json
from dataclasses import dataclass
import sqlite3
import threading
import uuid
from pathlib import Path
from typing import Any

import torch

_UNSET = object()


def encode(value: Any) -> bytes:
    stream = io.BytesIO()
    torch.save(value, stream)
    return stream.getvalue()


def decode(payload: bytes) -> Any:
    return torch.load(io.BytesIO(payload), map_location="cpu", weights_only=True)


@dataclass(frozen=True)
class AppendResult:
    sequences: list[int]
    added_count: int


class DurableStore:
    """Ordered, idempotent messages; one connection serialized by a short lock."""

    def __init__(self, path: str | Path) -> None:
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        self.path = path
        self._lock = threading.Lock()
        self._closed = False
        self._connection = sqlite3.connect(path, check_same_thread=False, timeout=10)
        self._connection.execute("PRAGMA journal_mode=WAL")
        self._connection.execute("PRAGMA synchronous=NORMAL")
        self._connection.execute(
            "CREATE TABLE IF NOT EXISTS messages (seq INTEGER PRIMARY KEY AUTOINCREMENT, "
            "message_id TEXT NOT NULL UNIQUE, payload BLOB NOT NULL)"
        )
        self._connection.execute("CREATE TABLE IF NOT EXISTS metadata (key TEXT PRIMARY KEY, value TEXT NOT NULL)")
        self._connection.execute("INSERT OR IGNORE INTO metadata VALUES ('identity', ?)", (uuid.uuid4().hex,))
        self.identity = self._connection.execute("SELECT value FROM metadata WHERE key='identity'").fetchone()[0]
        self._connection.commit()
        count, size = self._connection.execute("SELECT COUNT(*),COALESCE(SUM(LENGTH(payload)),0) FROM messages").fetchone()
        self._message_count, self._message_bytes = int(count), int(size)

    def _usage(self, after: int) -> tuple[int, int]:
        """Count and payload bytes of messages with ``seq > after``; caller holds ``_lock``."""
        if after <= 0:
            return self._message_count, self._message_bytes
        count, size = self._connection.execute("SELECT COUNT(*),COALESCE(SUM(LENGTH(payload)),0) FROM messages WHERE seq>?", (after,)).fetchone()
        return int(count), int(size)

    def active_episode(self):
        with self._lock:
            row = self._connection.execute("SELECT value FROM metadata WHERE key='active_episode'").fetchone()
            return None if row is None else json.loads(row[0])

    def update_active_episode(self, value):
        with self._lock, self._connection:
            self._connection.execute("INSERT OR REPLACE INTO metadata VALUES ('active_episode',?)", (json.dumps(value),))

    def bind_experiment(self, experiment_hash: str, contract=None) -> None:
        if not experiment_hash or experiment_hash == "unknown":
            return
        with self._lock, self._connection:
            existing = self._connection.execute("SELECT value FROM metadata WHERE key='experiment_hash'").fetchone()
            if existing is not None and existing[0] != experiment_hash:
                raise ValueError("durable store belongs to a different experiment")
            if existing is None:
                if self._connection.execute("SELECT COUNT(*) FROM messages").fetchone()[0]:
                    raise ValueError("existing durable records have unknown experiment; use a new store or restore their original experiment")
                self._connection.execute("INSERT INTO metadata VALUES ('experiment_hash',?)", (experiment_hash,))
            if contract is not None:
                self._connection.execute("INSERT OR REPLACE INTO metadata VALUES ('experiment_contract',?)", (json.dumps(contract, sort_keys=True),))

    def metrics(self, after: int = 0) -> dict[str, int]:
        with self._lock:
            count, size = self._usage(after)
            return {"depth": count, "bytes": size}

    def append_many(self, messages, *, after: int = 0, max_records: int | None = None, max_bytes: int | None = None, active_episode=_UNSET) -> AppendResult:
        """Bounded atomic admission. Duplicate retries consume no extra capacity."""
        from real_world_rl.loops.ingestion import AdmissionBusy
        with self._lock:
            with self._connection:
                unique = {}
                for message_id, payload in messages:
                    if message_id in unique and unique[message_id] != payload:
                        raise ValueError("batch repeats an ID with different payload")
                    unique[message_id] = payload
                added_count = added_bytes = 0
                for message_id, payload in unique.items():
                    row = self._connection.execute("SELECT payload FROM messages WHERE message_id=?", (message_id,)).fetchone()
                    if row is None:
                        added_count += 1
                        added_bytes += len(payload)
                    elif row[0] != payload:
                        raise ValueError(f"message ID reused with different payload: {message_id}")
                count, size = self._usage(after)
                if (max_records is not None and count + added_count > max_records) or (max_bytes is not None and size + added_bytes > max_bytes):
                    raise AdmissionBusy("durable admission capacity exhausted")
                self._connection.executemany("INSERT OR IGNORE INTO messages(message_id,payload) VALUES (?,?)", unique.items())
                if active_episode is not _UNSET and added_count:
                    if active_episode is None:
                        self._connection.execute("DELETE FROM metadata WHERE key='active_episode'")
                    else:
                        self._connection.execute("INSERT OR REPLACE INTO metadata VALUES ('active_episode',?)", (json.dumps(active_episode),))
                result = AppendResult([int(self._connection.execute("SELECT seq FROM messages WHERE message_id=?", (message_id,)).fetchone()[0]) for message_id, _ in messages], added_count)
            # Count only after the transaction committed; a rollback adds nothing.
            self._message_count += added_count
            self._message_bytes += added_bytes
            return result

    def append(self, message_id: str, payload: bytes) -> int:
        with self._lock:
            with self._connection:
                inserted = self._connection.execute(
                    "INSERT OR IGNORE INTO messages(message_id,payload) VALUES (?,?)",
                    (message_id, payload),
                ).rowcount == 1
                row = self._connection.execute(
                    "SELECT seq,payload FROM messages WHERE message_id=?", (message_id,)
                ).fetchone()
                if row[1] != payload:
                    raise ValueError(f"message ID reused with different payload: {message_id}")
            if inserted:
                self._message_count += 1
                self._message_bytes += len(payload)
            return int(row[0])

    def promote_pending(self, message_id: str, payload: bytes, next_pending: bytes | None = None, active_episode=_UNSET) -> int:
        """Atomically turn a delayed transition into an uploadable message."""
        with self._lock:
            with self._connection:
                inserted = self._connection.execute(
                    "INSERT OR IGNORE INTO messages(message_id,payload) VALUES (?,?)", (message_id, payload)
                ).rowcount == 1
                row = self._connection.execute("SELECT seq,payload FROM messages WHERE message_id=?", (message_id,)).fetchone()
                if row[1] != payload:
                    raise ValueError(f"message ID reused with different payload: {message_id}")
                if next_pending is None:
                    self._connection.execute("DELETE FROM metadata WHERE key='held_transition'")
                else:
                    self._connection.execute("INSERT OR REPLACE INTO metadata VALUES ('held_transition', ?)", (next_pending,))
                if active_episode is not _UNSET:
                    self._connection.execute("INSERT OR REPLACE INTO metadata VALUES ('active_episode',?)", (json.dumps(active_episode),))
            if inserted:
                self._message_count += 1
                self._message_bytes += len(payload)
            return int(row[0])

    def read(self, after: int = 0, limit: int = 256, max_bytes: int | None = None) -> list[tuple[int, str, bytes]]:
        with self._lock:
            if max_bytes is not None:
                metadata = self._connection.execute("SELECT seq,LENGTH(payload) FROM messages WHERE seq>? ORDER BY seq LIMIT ?", (after, limit)).fetchall()
                count, size = 0, 0
                for _, length in metadata:
                    if count and size + length > max_bytes:
                        break
                    count += 1
                    size += length
                limit = count
            return self._connection.execute(
                "SELECT seq,message_id,payload FROM messages WHERE seq>? ORDER BY seq LIMIT ?",
                (after, limit),
            ).fetchall()

    def remove(self, message_id: str) -> None:
        with self._lock:
            with self._connection:
                row = self._connection.execute("SELECT LENGTH(payload) FROM messages WHERE message_id=?", (message_id,)).fetchone()
                if row is not None:
                    self._connection.execute("DELETE FROM messages WHERE message_id=?", (message_id,))
            if row is not None:
                self._message_count -= 1
                self._message_bytes -= int(row[0])

    def __len__(self) -> int:
        with self._lock:
            return int(self._connection.execute("SELECT COUNT(*) FROM messages").fetchone()[0])

    def set_pending(self, payload: bytes | None, active_episode=_UNSET) -> None:
        with self._lock, self._connection:
            if payload is None:
                self._connection.execute("DELETE FROM metadata WHERE key='held_transition'")
            else:
                self._connection.execute("INSERT OR REPLACE INTO metadata VALUES ('held_transition', ?)", (payload,))
            if active_episode is not _UNSET:
                self._connection.execute("INSERT OR REPLACE INTO metadata VALUES ('active_episode',?)", (json.dumps(active_episode),))

    def pending(self) -> bytes | None:
        with self._lock:
            row = self._connection.execute("SELECT value FROM metadata WHERE key='held_transition'").fetchone()
            return None if row is None else row[0]

    def close(self) -> None:
        with self._lock:
            if not self._closed:
                self._connection.close()
                self._closed = True
