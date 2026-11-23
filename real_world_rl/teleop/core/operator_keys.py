"""The operator keys, shared by the event devices ``keyboard`` (pynput,
``real_world_rl.teleop.keyboard.pynput_source``), ``stdin`` and ``jsonl``:

- ``t`` takes over (``intervened`` becomes true), ``h`` hands back (false);
- ``s``/``f``/``r`` raise the ``success``/``failure``/``rerecord`` events;
- ``Esc`` ends the episode;
- ``n`` requests the next episode only in session/hold mode;
- ``q`` ends the episode and raises ``operator_quit``: the recorder and the actor loops stop
  after finishing that episode, as after Ctrl-C.

``episode_end_behavior=confirm`` (``real_world_rl.teleop.core.session.ConfirmSession``) also reads
``space`` (start the episode), ``n`` (reset for the next one), ``o``/``c`` (open/close the gripper)
and ``enter`` (confirm the gripper check) between episodes. It enables ``confirm_keys`` on the
source, which then keeps the presses in order for ``poll_keys()``; inside an episode they do
nothing (``n`` still raises ``next_episode`` in session/hold mode).
"""
from __future__ import annotations

import threading
import time
from typing import Any, Dict, List

import numpy as np

from real_world_rl.contracts.action import (
    LEGACY_ACTION_CONTRACT,
    get_env_action_contract,
)
from real_world_rl.teleop.core.contract import TeleopReading, TeleopSource

TAKE_OVER_KEY = "t"
HAND_BACK_KEY = "h"
EPISODE_END_KEY = "esc"
QUIT_KEY = "q"
START_KEY = "space"
NEXT_KEY = "n"
GRIPPER_OPEN_KEY = "o"
GRIPPER_CLOSE_KEY = "c"
CONFIRM_KEY = "enter"
# Event names of the confirm keys, for event devices that report presses as events (a GELLO
# event_device): event name -> key name.
CONFIRM_EVENT_KEYS: Dict[str, str] = {
    "start_episode": START_KEY,
    "next_episode": NEXT_KEY,
    "gripper_open": GRIPPER_OPEN_KEY,
    "gripper_close": GRIPPER_CLOSE_KEY,
    "confirm": CONFIRM_KEY,
    "operator_quit": QUIT_KEY,
}
EVENT_KEYS: Dict[str, str] = {"s": "success", "f": "failure", "r": "rerecord"}
OPERATOR_QUIT_EVENT = "operator_quit"
# Bound on the presses kept between two polls (a stuck key cannot grow memory).
_MAX_PRESSES = 64


def operator_quit_requested(info: Any) -> bool:
    """Whether a step's ``info`` carries the operator's quit key (``info["teleop_events"]``)."""
    events = (info or {}).get("teleop_events") or {}
    return bool(events.get(OPERATOR_QUIT_EVENT, False))


class OperatorKeyLatch:
    """Thread-safe state of the operator keys: the take-over flag, and the episode-end and event
    presses latched until the next ``take()``."""

    def __init__(self, *, session_events=False, confirm_keys=False) -> None:
        self.session_events = session_events
        self.confirm_keys = confirm_keys
        self._lock = threading.Lock()
        self._intervened = False
        self._episode_end = False
        self._events: Dict[str, bool] = {}
        self._presses: List[str] = []

    def press(self, name: str) -> None:
        """Applies one key press (the module's key names; other names are ignored, but with
        ``confirm_keys`` every press is also kept, in order, for ``take_presses``)."""
        with self._lock:
            if self.confirm_keys and len(self._presses) < _MAX_PRESSES:
                self._presses.append(name)
            if name == TAKE_OVER_KEY:
                self._intervened = True
            elif name == HAND_BACK_KEY:
                self._intervened = False
            elif name == EPISODE_END_KEY:
                self._episode_end = True
            elif name == QUIT_KEY:
                self._episode_end = True
                self._events[OPERATOR_QUIT_EVENT] = True
            elif name == "n" and self.session_events:
                self._events["next_episode"] = True
            elif name in EVENT_KEYS:
                self._events[EVENT_KEYS[name]] = True

    def take(self) -> tuple[bool, bool, Dict[str, bool]]:
        """``(intervened, episode_end, events)``; drops the latched presses."""
        with self._lock:
            reading = (self._intervened, self._episode_end, self._events)
            self._episode_end, self._events = False, {}
        return reading

    def take_presses(self) -> List[str]:
        """The key names pressed since the previous call, in order (only with ``confirm_keys``)."""
        with self._lock:
            presses, self._presses = self._presses, []
        return presses

    def reset(self) -> None:
        with self._lock:
            self._intervened = False
            self._episode_end = False
            self._events = {}
            self._presses = []


class OperatorKeySource(TeleopSource):
    """An event device emitting zeros of the declared width, with ``intervened`` and the events
    from the operator keys. ``from_env`` declares the env's action space so the source validates
    against any env; on its own it only suits event-only tests, it is meant as the event half of
    ``CompositeTeleopSource``. ``reset()`` hands back and drops the latched presses."""

    def __init__(
        self,
        *,
        action_mode: str = "cartesian_delta",
        arm_action_dim: int = 6,
        has_gripper: bool = True,
        session_events: bool = False,
    ) -> None:
        self._action_mode = action_mode
        self._arm_action_dim = int(arm_action_dim)
        self._has_gripper = bool(has_gripper)
        self._keys = OperatorKeyLatch(session_events=session_events)
        self._recent_presses: List[str] = []

    @classmethod
    def from_env(cls, env: Any, **kwargs: Any) -> "OperatorKeySource":
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

    def enable_session_events(self):
        self._keys.session_events = True

    def enable_confirm_keys(self) -> None:
        """Keep every press in order for ``poll_keys`` (``episode_end_behavior=confirm``)."""
        self._keys.confirm_keys = True

    def provides_operator_keys(self) -> bool:
        return True

    def poll_keys(self) -> List[str]:
        """``poll()``, then the key names pressed since the previous poll, in order. Presses
        that an ordinary ``poll()`` (an env step) already took are not repeated."""
        self.poll()
        presses, self._recent_presses = self._recent_presses, []
        return presses

    def poll(self) -> TeleopReading:
        """The take-over state and the presses latched since the previous poll."""
        self._check_alive()
        self._recent_presses = self._keys.take_presses()
        intervened, episode_end, events = self._keys.take()
        return TeleopReading(
            action=np.zeros(self._arm_action_dim + int(self._has_gripper), dtype=np.float32),
            intervened=intervened,
            episode_end=episode_end,
            events=events,
            timestamp=time.monotonic(),
        )

    def reset(self) -> None:
        self._keys.reset()
        self._recent_presses = []

    def _check_alive(self) -> None:
        """Raises when the key reader died, so a dead device never reads as an idle operator."""
