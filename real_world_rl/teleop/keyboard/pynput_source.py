"""Keyboard of this machine as an event device (``device="keyboard"``), read with ``pynput``.

The keys are the operator keys of ``real_world_rl.teleop.core.operator_keys``, shared with the
terminal source (``device="stdin"``).
"""
from __future__ import annotations

import threading
from typing import Any, Optional, Set

from real_world_rl.teleop.core.operator_keys import EPISODE_END_KEY, OperatorKeySource
from real_world_rl.teleop.core.registry import register_teleop_source


@register_teleop_source("keyboard")
class KeyboardTeleopSource(OperatorKeySource):
    """The operator keys (``real_world_rl.teleop.core.operator_keys``) from ``pynput``. Keys act
    on press; a held key (auto-repeat) acts once until it is released.

    ``pynput`` (extra ``reward-collection``) is imported when ``start()`` starts the listener, so
    building the source works without it; on Linux it needs an X session (``device="stdin"``
    does not). ``close()`` stops the listener.
    """

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._held: Set[str] = set()
        self._held_lock = threading.Lock()
        self._keyboard: Any = None
        self._listener: Any = None

    def start(self) -> None:
        if self._listener is not None:
            return
        try:
            from pynput import keyboard
        except ImportError:
            from real_world_rl.devices.errors import SdkMissing

            raise SdkMissing(
                "the keyboard teleop source needs pynput",
                hint="pip install pynput (extra reward-collection); on Linux it needs an X session, "
                "or use --teleop-event-device stdin from a terminal",
                device="keyboard",
            ) from None
        self._keyboard = keyboard
        listener = keyboard.Listener(on_press=self._on_press, on_release=self._on_release)
        listener.start()
        self._listener = listener

    def close(self) -> None:
        listener, self._listener = self._listener, None
        if listener is not None:
            listener.stop()

    def _check_alive(self) -> None:
        if self._listener is not None and not self._listener.is_alive():
            from real_world_rl.devices.errors import DeviceFault

            raise DeviceFault(
                "the keyboard listener stopped",
                hint="check the X session (DISPLAY) and restart the run",
                device="keyboard",
            )

    # -- listener callbacks (pynput's thread) --------------------------------------

    def _key_name(self, key: Any) -> Optional[str]:
        if key == self._keyboard.Key.esc:
            return EPISODE_END_KEY
        char = getattr(key, "char", None)
        return char.lower() if isinstance(char, str) else None

    def _on_press(self, key: Any) -> None:
        name = self._key_name(key)
        if name is None:
            return
        with self._held_lock:
            if name in self._held:
                return  # auto-repeat of a held key
            self._held.add(name)
        self._keys.press(name)

    def _on_release(self, key: Any) -> None:
        name = self._key_name(key)
        if name is not None:
            with self._held_lock:
                self._held.discard(name)
