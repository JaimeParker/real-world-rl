"""The terminal as an event device (``device="stdin"``): the operator keys of
``real_world_rl.teleop.keyboard`` (``t``/``h``/``s``/``f``/``r``/``Esc``/``q``) typed into the
terminal that runs the process. Needs no desktop session (works over SSH), unlike ``keyboard``.
"""
from __future__ import annotations

import os
import select
import sys
import threading
from typing import Any, List, Optional

from real_world_rl.teleop.keyboard import EPISODE_END_KEY, OperatorKeySource
from real_world_rl.teleop.registry import register_teleop_source

__all__ = ["StdinTeleopSource", "keys_from_bytes"]

_ESCAPE = "\x1b"
# An escape sequence (arrow and function keys) starts with ESC followed by one of these.
_SEQUENCE_INTRODUCERS = "[O"
_READ_SIZE = 64


def keys_from_bytes(data: bytes) -> List[str]:
    """The key names in a chunk read from the terminal: lower-cased characters, ``"esc"`` for a
    lone Escape. Escape sequences (arrow and function keys) are skipped, not read as Esc."""
    text = data.decode(errors="ignore")
    keys: List[str] = []
    index = 0
    while index < len(text):
        char = text[index]
        if char == _ESCAPE and index + 1 < len(text) and text[index + 1] in _SEQUENCE_INTRODUCERS:
            end = index + 2
            while end < len(text) and not ("@" <= text[end] <= "~"):
                end += 1  # parameter and intermediate bytes up to the final byte
            index = end + 1
            continue
        keys.append(EPISODE_END_KEY if char == _ESCAPE else char.lower())
        index += 1
    return keys


@register_teleop_source("stdin")
class StdinTeleopSource(OperatorKeySource):
    """The operator keys from a file descriptor (``sys.stdin`` by default), read by a thread.

    ``start()`` puts a terminal into cbreak mode (one key at a time, no echo; Ctrl-C still
    interrupts) and starts the reader; ``close()`` stops it and restores the terminal settings.
    A descriptor that is not a terminal (a pipe) is read as is. Every byte is a press, so a held
    key repeats. End of input or a read error makes the next ``poll()`` raise ``DeviceFault``
    instead of reading as an idle operator.
    """

    def __init__(self, *, fd: Optional[int] = None, poll_interval_s: float = 0.02, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        if poll_interval_s <= 0:
            raise ValueError(f"poll_interval_s must be positive, got {poll_interval_s}")
        self._fd = fd
        self._poll_interval_s = float(poll_interval_s)
        self._stop = threading.Event()
        self._reader: Optional[threading.Thread] = None
        self._saved_terminal: Any = None
        self._failure: Optional[str] = None

    def start(self) -> None:
        if self._reader is not None:
            return
        fd = sys.stdin.fileno() if self._fd is None else self._fd
        self._fd = fd
        if os.isatty(fd):
            import termios
            import tty

            self._saved_terminal = termios.tcgetattr(fd)
            tty.setcbreak(fd)
        self._stop.clear()
        reader = threading.Thread(target=self._read_keys, name="stdin-teleop", daemon=True)
        self._reader = reader
        try:
            reader.start()
        except BaseException:
            self._reader = None
            self._restore_terminal()
            raise

    def close(self) -> None:
        reader, self._reader = self._reader, None
        self._stop.set()
        if reader is not None:
            reader.join(timeout=1.0)
        self._restore_terminal()

    def _check_alive(self) -> None:
        if self._failure is not None:
            from robot_infra.devices.errors import DeviceFault

            raise DeviceFault(
                f"the terminal key reader stopped: {self._failure}",
                hint="run the process in an interactive terminal (stdin attached), or use another event device",
                device="stdin",
            )

    def _read_keys(self) -> None:
        try:
            while not self._stop.is_set():
                ready, _, _ = select.select([self._fd], [], [], self._poll_interval_s)
                if not ready:
                    continue
                data = os.read(self._fd, _READ_SIZE)
                if not data:
                    self._failure = "end of input"
                    return
                for name in keys_from_bytes(data):
                    self._keys.press(name)
        except Exception as error:  # surfaced by the next poll
            self._failure = repr(error)

    def _restore_terminal(self) -> None:
        saved, self._saved_terminal = self._saved_terminal, None
        if saved is not None:
            import termios

            termios.tcsetattr(self._fd, termios.TCSADRAIN, saved)
