"""Keyboard of this machine as an event device (``device="keyboard"``), read with ``pynput``."""
from __future__ import annotations

import threading
import time
from typing import Any, Dict, Optional, Set

import numpy as np

from rl_garden_real_world.action_contract import (
    LEGACY_ACTION_CONTRACT,
    get_env_action_contract,
)
from rl_garden_real_world.teleop.base import TeleopReading, TeleopSource
from rl_garden_real_world.teleop.registry import register_teleop_source

TOGGLE_KEY = "space"
EPISODE_END_KEY = "esc"
EVENT_KEYS: Dict[str, str] = {"s": "success", "f": "failure", "r": "rerecord"}


@register_teleop_source("keyboard")
class KeyboardTeleopSource(TeleopSource):
    """Space toggles ``intervened``, Esc raises ``episode_end``, ``s``/``f``/``r`` raise the
    ``success``/``failure``/``rerecord`` events. Keys act on press; a held key (auto-repeat) acts
    once until it is released. Presses latch until the next ``poll()``.

    The action is always zeros of the declared width, so on its own the keyboard only suits
    event-only tests; it is meant as the event half of ``CompositeTeleopSource``. ``from_env``
    declares the env's action space so the source validates against any env.

    ``pynput`` (extra ``reward-collection``) is imported when ``start()`` starts the listener, so
    building the source works without it; on Linux it needs an X session. ``reset()`` clears the
    latched presses and the toggle; ``close()`` stops the listener.
    """

    def __init__(
        self,
        *,
        action_mode: str = "cartesian_delta",
        arm_action_dim: int = 6,
        has_gripper: bool = True,
    ) -> None:
        self._action_mode = action_mode
        self._arm_action_dim = int(arm_action_dim)
        self._has_gripper = bool(has_gripper)
        self._lock = threading.Lock()
        self._toggled = False
        self._episode_end = False
        self._events: Dict[str, bool] = {}
        self._held: Set[str] = set()
        self._keyboard: Any = None
        self._listener: Any = None

    @classmethod
    def from_env(cls, env: Any, **kwargs: Any) -> "KeyboardTeleopSource":
        contract = get_env_action_contract(env) or LEGACY_ACTION_CONTRACT
        kwargs.setdefault("action_mode", contract["action_mode"])
        kwargs.setdefault("arm_action_dim", int(contract["arm_action_dim"]))
        kwargs.setdefault("has_gripper", bool(contract["gripper_in_action"]))
        return cls(**kwargs)

    @property
    def action_mode(self) -> str:
        return self._action_mode

    @property
    def arm_action_dim(self) -> int:
        return self._arm_action_dim

    @property
    def has_gripper(self) -> bool:
        return self._has_gripper

    def start(self) -> None:
        if self._listener is not None:
            return
        try:
            from pynput import keyboard
        except ImportError:
            from robot_infra.devices.errors import SdkMissing

            raise SdkMissing(
                "the keyboard teleop source needs pynput",
                hint="pip install pynput (extra reward-collection); on Linux it needs an X session",
                device="keyboard",
            ) from None
        self._keyboard = keyboard
        listener = keyboard.Listener(on_press=self._on_press, on_release=self._on_release)
        listener.start()
        self._listener = listener

    def poll(self) -> TeleopReading:
        """The toggle state and the presses latched since the previous poll. A listener that
        died raises ``DeviceFault`` instead of reading as an idle operator."""
        if self._listener is not None and not self._listener.is_alive():
            from robot_infra.devices.errors import DeviceFault

            raise DeviceFault(
                "the keyboard listener stopped",
                hint="check the X session (DISPLAY) and restart the run",
                device="keyboard",
            )
        with self._lock:
            intervened, episode_end, events = self._toggled, self._episode_end, self._events
            self._episode_end, self._events = False, {}
        return TeleopReading(
            action=np.zeros(self._arm_action_dim + int(self._has_gripper), dtype=np.float32),
            intervened=intervened,
            episode_end=episode_end,
            events=events,
            timestamp=time.monotonic(),
        )

    def reset(self) -> None:
        with self._lock:
            self._toggled = False
            self._episode_end = False
            self._events = {}

    def close(self) -> None:
        listener, self._listener = self._listener, None
        if listener is not None:
            listener.stop()

    # -- listener callbacks (pynput's thread) --------------------------------------

    def _key_name(self, key: Any) -> Optional[str]:
        if key == self._keyboard.Key.space:
            return TOGGLE_KEY
        if key == self._keyboard.Key.esc:
            return EPISODE_END_KEY
        char = getattr(key, "char", None)
        return char.lower() if isinstance(char, str) else None

    def _on_press(self, key: Any) -> None:
        name = self._key_name(key)
        if name is None:
            return
        with self._lock:
            if name in self._held:
                return  # auto-repeat of a held key
            self._held.add(name)
            if name == TOGGLE_KEY:
                self._toggled = not self._toggled
            elif name == EPISODE_END_KEY:
                self._episode_end = True
            elif name in EVENT_KEYS:
                self._events[EVENT_KEYS[name]] = True

    def _on_release(self, key: Any) -> None:
        name = self._key_name(key)
        if name is not None:
            with self._lock:
                self._held.discard(name)
