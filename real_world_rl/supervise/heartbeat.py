"""Child side of the supervisor heartbeat: one byte per loop iteration down an inherited pipe.

``rwrl supervise -- <command>`` hands ``<command>`` the write end of a pipe and
announces its file descriptor in ``RWRL_HEARTBEAT_FD``. ``Heartbeat.from_env()`` picks it
up; without the variable every call is a no-op, so call sites need no ``if supervised`` branch.
"""
from __future__ import annotations

import contextlib
import os
import stat
from collections.abc import Iterator, Mapping
from typing import Callable, Optional

HEARTBEAT_FD_ENV = "RWRL_HEARTBEAT_FD"
# One byte per message. Any byte other than LONG_OPERATION_BYTE counts as a beat.
BEAT_BYTE = b"."
LONG_OPERATION_BYTE = b"L"


class SupervisorLost(RuntimeError):
    """The supervisor closed its end of the heartbeat pipe (it died or was killed)."""


class Heartbeat:
    """Tells the supervisor this process is alive. ``beat()`` once per control-loop iteration;
    ``begin_long_operation()`` right before a call that legitimately blocks for longer than the
    supervisor's ``--heartbeat-timeout-s`` (an env reset), followed by ``beat()`` when it returns."""

    def __init__(self, fd: Optional[int]) -> None:
        self._fd = fd

    @classmethod
    def from_env(cls, environ: Optional[Mapping[str, str]] = None) -> "Heartbeat":
        value = (os.environ if environ is None else environ).get(HEARTBEAT_FD_ENV)
        if not value:
            return cls(None)
        try:
            fd = int(value)
            is_pipe = stat.S_ISFIFO(os.fstat(fd).st_mode)
        except (ValueError, OSError) as error:
            raise ValueError(
                f"{HEARTBEAT_FD_ENV}={value!r} is not an open file descriptor of this process ({error}); "
                "it is set by `rwrl supervise` for its direct child only"
            ) from None
        if not is_pipe:
            raise ValueError(f"{HEARTBEAT_FD_ENV}={value!r} is not a pipe")
        os.set_blocking(fd, False)
        return cls(fd)

    @property
    def enabled(self) -> bool:
        return self._fd is not None

    def beat(self) -> None:
        self._send(BEAT_BYTE)

    def begin_long_operation(self) -> None:
        self._send(LONG_OPERATION_BYTE)

    def _send(self, message: bytes) -> None:
        if self._fd is None:
            return
        try:
            os.write(self._fd, message)
        except BlockingIOError:
            pass  # the pipe is full of unread beats: the supervisor already knows we are alive
        except BrokenPipeError:
            raise SupervisorLost(
                "the rwrl supervisor is gone (heartbeat pipe closed); stopping instead of "
                "running unsupervised"
            ) from None


@contextlib.contextmanager
def capture_heartbeats() -> Iterator[Callable[[], bytes]]:
    """Test helper: route ``Heartbeat.from_env()`` to a fresh pipe for the duration of the block.

    Yields ``received()``, which returns every message written since the previous call (``b"."``
    per beat, ``b"L"`` per long operation)::

        with capture_heartbeats() as received:
            run_recorder(args)
            assert received().count(BEAT_BYTE) == number_of_steps + number_of_resets
    """
    read_fd, write_fd = os.pipe()
    os.set_blocking(read_fd, False)
    previous = os.environ.get(HEARTBEAT_FD_ENV)
    os.environ[HEARTBEAT_FD_ENV] = str(write_fd)

    def received() -> bytes:
        chunks = []
        while True:
            try:
                chunk = os.read(read_fd, 65536)
            except BlockingIOError:
                break
            if not chunk:
                break
            chunks.append(chunk)
        return b"".join(chunks)

    try:
        yield received
    finally:
        if previous is None:
            os.environ.pop(HEARTBEAT_FD_ENV, None)
        else:
            os.environ[HEARTBEAT_FD_ENV] = previous
        os.close(write_fd)
        os.close(read_fd)
