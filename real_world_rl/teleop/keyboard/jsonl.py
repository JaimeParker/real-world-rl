"""Bounded, nonblocking operator events for agents; no robot commands in the schema."""
from __future__ import annotations

import json
import os
import select
import stat
import sys

from real_world_rl.teleop.core.operator_keys import OperatorKeySource
from real_world_rl.teleop.core.registry import register_teleop_source

_KEYS = {"takeover": "t", "handback": "h", "success": "s", "failure": "f",
         "rerecord": "r", "end": "esc", "next": "n", "quit": "q"}
_MAX_BYTES = 65536


@register_teleop_source("jsonl")
class JsonlEventSource(OperatorKeySource):
    """Read one ``{"event": "..."}`` per poll, from append-only file or stdin.

    Regular file EOF means wait for an append. Pipe EOF is a lost operator and
    raises, unless the last delivered event was quit. Reset clears takeover,
    never discards unread events. A producer should wait for session phase output
    before sending the next episode's events.
    """

    def __init__(self, *, path="-", fd=None, **kwargs):
        super().__init__(**kwargs)
        self.path = path
        self._fd = fd
        self._owns_fd = False
        self._regular = False
        self._buffer = b""
        self._quit = False

    def start(self):
        if self._fd is None:
            if self.path == "-":
                self._fd = sys.stdin.fileno()
            else:
                self._fd = os.open(self.path, os.O_RDONLY | os.O_NONBLOCK)
                self._owns_fd = True
        self._regular = stat.S_ISREG(os.fstat(self._fd).st_mode)
        if os.isatty(self._fd):
            self.close()
            raise ValueError("JSONL events require a file or pipe, not a terminal")

    def poll(self):
        from real_world_rl.devices.errors import DeviceFault
        if self._fd is None:
            raise RuntimeError("JSONL event source has not started")
        if b"\n" not in self._buffer and select.select([self._fd], [], [], 0)[0]:
            try:
                chunk = os.read(self._fd, 4096)
            except BlockingIOError:
                chunk = None
            if chunk == b"" and not self._regular and not self._quit:
                raise DeviceFault("JSONL event stream closed before quit")
            if chunk:
                self._buffer += chunk
            if len(self._buffer) > _MAX_BYTES:
                raise DeviceFault("JSONL event line exceeds 65536 bytes")
        if b"\n" in self._buffer:
            line, self._buffer = self._buffer.split(b"\n", 1)
            try:
                event = json.loads(line)
                if not isinstance(event, dict) or set(event) != {"event"}:
                    raise ValueError("expected exactly one event field")
                name = event["event"]
                if not isinstance(name, str) or name not in _KEYS:
                    raise ValueError("unknown event")
            except (ValueError, UnicodeError) as exc:
                raise DeviceFault(f"invalid JSONL operator event: {exc}") from exc
            self._keys.press(_KEYS[name])
            self._quit = name == "quit"
        return super().poll()

    def close(self):
        if self._owns_fd and self._fd is not None:
            os.close(self._fd)
        self._fd = None
        self._owns_fd = False
