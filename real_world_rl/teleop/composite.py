"""One motion device plus one event device acting as a single teleop source (``device="composite"``).

The motion half drives the arm (a leader arm, a SpaceMouse, ...); the event half carries the
operator's keys (``keyboard`` or the terminal, ``stdin``): the intervention state (``t`` takes
over and ``h`` hands back), episode end, ``success``/``failure``/``rerecord`` and
``operator_quit``. A motion half may keep its own take-over and ignore that state (the
SpaceMouse's button 0).
"""
from __future__ import annotations

from abc import abstractmethod
from typing import Any, Mapping, Optional

from real_world_rl.teleop.base import TeleopReading, TeleopSource
from real_world_rl.teleop.registry import (
    make_teleop_source,
    register_teleop_source,
)

__all__ = ["INTERVENTION_FROM", "CompositeTeleopSource", "ToggleableTeleopSource"]

INTERVENTION_FROM = ("motion", "events")


class ToggleableTeleopSource(TeleopSource):
    """A motion source whose takeover depends on the operator's intervention state (an onset
    policy that rate-limits or gates the first intervened steps, or a command tracker that
    anchors at the take-over and holds at the hand-back).

    ``CompositeTeleopSource(intervention_from="events")`` hands the event device's state (its
    ``intervened``: the keyboard's latched take-over/hand-back keys) to ``poll_with_toggle``
    instead of overriding ``intervened`` with it, so the onset policy also applies when the
    toggle lives on another device.
    """

    @abstractmethod
    def poll_with_toggle(self, toggle: bool) -> TeleopReading:
        """``poll()`` with ``toggle`` in place of the device's own intervention signal."""


@register_teleop_source("composite")
class CompositeTeleopSource(TeleopSource):
    """Declares the motion source's action space. ``poll()`` polls the event half, then the motion
    half: ``action`` comes from motion, ``intervened`` from the half named by
    ``intervention_from`` (a ``ToggleableTeleopSource`` motion half receives the event toggle and
    decides), ``episode_end`` and ``events`` are ORed over both. ``start``/``reset``/``close``
    reach both halves; ``close`` closes the second even if the first raises.
    """

    def __init__(
        self,
        motion: TeleopSource,
        events: TeleopSource,
        *,
        intervention_from: str = "events",
    ) -> None:
        if intervention_from not in INTERVENTION_FROM:
            raise ValueError(
                f"intervention_from must be one of {list(INTERVENTION_FROM)}, got {intervention_from!r}"
            )
        self.motion = motion
        self.events = events
        self.intervention_from = intervention_from

    @classmethod
    def from_env(
        cls,
        env: Any,
        *,
        motion: str,
        events: str,
        motion_kwargs: Optional[Mapping[str, Any]] = None,
        events_kwargs: Optional[Mapping[str, Any]] = None,
        intervention_from: str = "events",
    ) -> "CompositeTeleopSource":
        """Builds both halves by registry name with ``make_teleop_source(name, env, **kwargs)``."""
        motion_source = make_teleop_source(motion, env, **dict(motion_kwargs or {}))
        events_source = make_teleop_source(events, env, **dict(events_kwargs or {}))
        return cls(motion_source, events_source, intervention_from=intervention_from)

    @property
    def action_mode(self) -> str:
        return self.motion.action_mode

    @property
    def arm_action_dim(self) -> int:
        return self.motion.arm_action_dim

    @property
    def has_gripper(self) -> bool:
        return self.motion.has_gripper

    @property
    def num_arms(self) -> int:
        """The motion half's arm count (the event half only adds buttons)."""
        return int(getattr(self.motion, "num_arms", 1))

    @property
    def arm_names(self) -> Optional[list[str]]:
        names = getattr(self.motion, "arm_names", None)
        return None if names is None else [str(name) for name in names]

    def start(self) -> None:
        self.motion.start()
        try:
            self.events.start()
        except BaseException:
            self.motion.close()
            raise

    def poll(self) -> TeleopReading:
        event_reading = self.events.poll()
        if self.intervention_from == "motion":
            ignore_toggle = getattr(self.motion, "ignore_event_toggle", None)
            if callable(ignore_toggle):
                ignore_toggle(event_reading.intervened)
            motion_reading = self.motion.poll()
            intervened = motion_reading.intervened
        elif isinstance(self.motion, ToggleableTeleopSource):
            motion_reading = self.motion.poll_with_toggle(event_reading.intervened)
            intervened = motion_reading.intervened
        else:
            motion_reading = self.motion.poll()
            intervened = event_reading.intervened
        return TeleopReading(
            action=motion_reading.action,
            intervened=bool(intervened),
            episode_end=bool(motion_reading.episode_end or event_reading.episode_end),
            events={**dict(event_reading.events), **dict(motion_reading.events)},
            timestamp=motion_reading.timestamp,
            buttons=motion_reading.buttons,
            diagnostics=getattr(motion_reading, "diagnostics", {}),
            human_action_mask=None if motion_reading.human_action_mask is None else motion_reading.human_action_mask & bool(intervened),
            override_mask=None if motion_reading.override_mask is None else motion_reading.override_mask & bool(intervened),
        )

    def reset(self) -> None:
        try:
            self.motion.reset()
        finally:
            self.events.reset()

    def close(self) -> None:
        try:
            self.motion.close()
        finally:
            self.events.close()
