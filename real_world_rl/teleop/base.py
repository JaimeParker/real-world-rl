"""The teleop source contract the intervention wrapper consumes.

A ``TeleopSource`` turns one operator device into ``TeleopReading``s in the env's normalized
``[-1, 1]`` action space. The wrapper owns the env side (reset order, action replacement, info
keys); a source owns only the device: opening it (``start``), reading it (``poll``), dropping
latched state after an env reset (``reset``) and releasing it (``close``).
"""
from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Any, Callable, Mapping

import numpy as np

_LEGACY_ACTION_MODE = "cartesian_delta"
_LEGACY_ARM_ACTION_DIM = 6


def held_gripper_action(last_gripper_action_getter: Callable[[], Any], arm_index: int = 0) -> float:
    """The gripper action last sent to arm ``arm_index``. The getter returns one entry per arm
    (``InterventionWrapper`` keeps a ``(num_arms,)`` tensor); a bare number counts as one arm."""
    held = np.asarray(last_gripper_action_getter(), dtype=np.float64).reshape(-1)
    return float(held[arm_index])


@dataclass(frozen=True)
class TeleopReading:
    """One poll of a teleop source.

    ``action`` is float32 ``(A,)`` in the env's normalized ``[-1, 1]`` action space with the
    gripper last when the source ``has_gripper``; it only matters while ``intervened``.
    ``events`` holds the latched button edges that fired since the previous poll
    (``success``/``failure``/``rerecord``); it is empty when none did.
    """

    action: np.ndarray
    intervened: bool
    episode_end: bool = False
    events: Mapping[str, bool] = field(default_factory=dict)
    timestamp: float = 0.0
    human_action_mask: Any = None
    override_mask: Any = None

    def __post_init__(self) -> None:
        action = np.asarray(self.action, dtype=np.float32)
        if action.ndim != 1:
            raise ValueError(f"TeleopReading.action must be 1-D, got shape {action.shape}")
        object.__setattr__(self, "action", action)
        for name in ("human_action_mask", "override_mask"):
            value = getattr(self, name)
            if value is not None:
                value = np.asarray(value, dtype=bool)
                if value.shape != action.shape:
                    raise ValueError(f"{name} shape must match action")
                object.__setattr__(self, name, value)

    @classmethod
    def idle(cls, action_dim: int) -> "TeleopReading":
        """A reading from an operator who is not touching the device."""
        return cls(action=np.zeros(action_dim, dtype=np.float32), intervened=False)


class TeleopSource(ABC):
    """One operator device. Subclasses declare the action space they emit and implement ``poll``.

    ``action_mode`` (``cartesian_delta`` | ``joint_delta`` | ``joint_abs``), ``arm_action_dim``
    and ``has_gripper`` describe the emitted actions; the wrapper checks them against the env's
    action contract. Register a subclass with ``register_teleop_source`` to make it available as
    ``device="<name>"``; ``from_env`` is the factory the registry calls.
    """

    @property
    @abstractmethod
    def action_mode(self) -> str: ...

    @property
    @abstractmethod
    def arm_action_dim(self) -> int: ...

    @property
    @abstractmethod
    def has_gripper(self) -> bool: ...

    @classmethod
    def from_env(cls, env: Any, **kwargs: Any) -> "TeleopSource":
        """Builds the source for ``env`` (its action contract is read through the wrapper chain)."""
        return cls(**kwargs)

    def start(self) -> None:
        """Opens the device. The intervention wrapper calls it once after validating the source."""

    @abstractmethod
    def poll(self) -> TeleopReading:
        """Reads the device once; read failures propagate (a dropped intervention must not pass as idle)."""

    def reset(self) -> None:
        """Called after the env reset: drops latched events and sticky device state."""

    def close(self) -> None:
        """Releases the device. Must be safe to call more than once."""


def reading_from_legacy_sample(sample: Any, *, action_dim: int) -> TeleopReading:
    """``TeleopReading`` from a legacy sample (``TeleOpSample``-shaped).

    Needs ``.intervened`` and ``.episode_end``; ``.action`` is only read when present (a sample
    that is not intervening may omit it), ``.events`` and ``.timestamp`` are optional.
    """
    action = getattr(sample, "action", None)
    events = getattr(sample, "events", None) or {}
    return TeleopReading(
        action=np.zeros(action_dim, dtype=np.float32) if action is None else action,
        intervened=bool(sample.intervened),
        episode_end=bool(sample.episode_end),
        events={str(key): True for key, fired in events.items() if fired},
        timestamp=float(getattr(sample, "timestamp", 0.0)),
    )


class LegacyTeleopAdapter(TeleopSource):
    """Wraps a duck-typed source: ``poll()`` returning something with ``.intervened``/``.action``/
    ``.episode_end``, optional ``reset()``/``close()``, and optional ``action_mode``/
    ``arm_action_dim``/``has_gripper`` attributes (default cartesian_delta / 6 / True)."""

    def __init__(self, legacy: Any) -> None:
        self.legacy = legacy

    @property
    def action_mode(self) -> str:
        return str(getattr(self.legacy, "action_mode", _LEGACY_ACTION_MODE))

    @property
    def arm_action_dim(self) -> int:
        return int(getattr(self.legacy, "arm_action_dim", _LEGACY_ARM_ACTION_DIM))

    @property
    def has_gripper(self) -> bool:
        return bool(getattr(self.legacy, "has_gripper", True))

    def poll(self) -> TeleopReading:
        return reading_from_legacy_sample(
            self.legacy.poll(), action_dim=self.arm_action_dim + int(self.has_gripper)
        )

    def reset(self) -> None:
        reset = getattr(self.legacy, "reset", None)
        if callable(reset):
            reset()

    def close(self) -> None:
        close = getattr(self.legacy, "close", None)
        if callable(close):
            close()


def as_teleop_source(source: Any) -> TeleopSource:
    """``source`` itself when it is a ``TeleopSource``, else a ``LegacyTeleopAdapter`` over it."""
    return source if isinstance(source, TeleopSource) else LegacyTeleopAdapter(source)
