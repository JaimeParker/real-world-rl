"""Explicit task outcomes shared by actors, evaluation and collection.

Completed episodes, including failures, timeouts and unknown labels, form the
success-rate denominator. Interrupted and discarded attempts are reported apart.
"""
from __future__ import annotations

from dataclasses import dataclass, asdict
from typing import Any, Mapping


def _any(value: Any) -> bool:
    if value is None:
        return False
    if hasattr(value, "any"):
        return bool(value.any())
    if isinstance(value, (list, tuple)):
        return any(_any(item) for item in value)
    return bool(value)


@dataclass(frozen=True)
class EpisodeOutcome:
    task_success: bool | None
    had_intervention: bool
    steps: int
    intervention_steps: int
    completed: bool = True
    interrupted: bool = False
    discarded: bool = False
    outcome_source: str = "unknown"
    end_reason: str = "completed"
    tcp_proxy_success: bool | None = None

    @property
    def autonomous_success(self) -> bool:
        return self.completed and self.task_success is True and not self.had_intervention

    def to_dict(self) -> dict:
        return asdict(self)


class EpisodeOutcomeAccumulator:
    def __init__(self) -> None:
        self.task_success: bool | None = None
        self.steps = 0
        self.intervention_steps = 0
        self._manual_failure = False
        self.outcome_source = "unknown"
        self.tcp_proxy_success = None

    def observe(self, info: Mapping[str, Any] | None) -> None:
        self.steps += 1
        info = info or {}
        if "tcp_proxy_success" in info: self.tcp_proxy_success = _any(info["tcp_proxy_success"])
        # An explicit mask is authoritative: label-only buttons never count as control.
        controlled = _any(info["human_action_mask"]) if "human_action_mask" in info else _any(
            info.get("intervened", info.get("is_intervention", False))
        )
        self.intervention_steps += int(controlled)
        events = {} if info.get("task_outcome_finalized") else (info.get("teleop_events") or {})
        if _any(events.get("failure")) or _any(info.get("manual_failure")):
            self._manual_failure = True
        if info.get("task_outcome_finalized"):
            value = info.get("task_success")
            self.task_success = None if value is None else _any(value)
            self.outcome_source = str(info.get("task_success_source", "unknown"))
        else:
            for key in ("task_success", "success", "succeed"):
                if key in info and info[key] is not None:
                    self.task_success = _any(info[key])
                    self.outcome_source = str(info.get("task_success_source", "environment"))
                    break
        if _any(events.get("success")):
            self.task_success = True
            self.outcome_source = "manual"
        if self._manual_failure:
            self.task_success = False
            self.outcome_source = "manual"

    def finish(self, *, interrupted: bool = False, discarded: bool = False,
               terminated: Any = False, truncated: Any = False,
               end_reason: str | None = None) -> EpisodeOutcome:
        reason = end_reason or ("discarded" if discarded else "interrupted" if interrupted
                                else "timeout" if _any(truncated) and not _any(terminated)
                                else "terminated" if _any(terminated) else "completed")
        if self._manual_failure and not (interrupted or discarded):
            reason = "manual_failure"
        return EpisodeOutcome(self.task_success, self.intervention_steps > 0, self.steps,
                              self.intervention_steps, not (interrupted or discarded),
                              interrupted, discarded, self.outcome_source, reason, self.tcp_proxy_success)



class EpisodeMetrics:
    def __init__(self) -> None:
        self.outcomes: list[EpisodeOutcome] = []

    def add(self, outcome: EpisodeOutcome) -> None:
        self.outcomes.append(outcome)

    def summary(self) -> dict[str, float | int]:
        completed = [outcome for outcome in self.outcomes if outcome.completed]
        n = len(completed)
        steps = sum(outcome.steps for outcome in completed)
        autonomous = sum(outcome.autonomous_success for outcome in completed)
        rate = autonomous / n if n else 0.0
        return {
            "success_rate": rate,
            "autonomous_success_rate": rate,
            "tcp_proxy_success_rate": sum(outcome.tcp_proxy_success is True for outcome in completed) / n if n else 0.0,
            "task_success_rate": sum(outcome.task_success is True for outcome in completed) / n if n else 0.0,
            "episode_intervention_rate": sum(outcome.had_intervention for outcome in completed) / n if n else 0.0,
            "step_intervention_rate": sum(outcome.intervention_steps for outcome in completed) / steps if steps else 0.0,
            "completed_episodes": n,
            "unknown_episodes": sum(outcome.task_success is None for outcome in completed),
            "interrupted_episodes": sum(outcome.interrupted for outcome in self.outcomes),
            "discarded_episodes": sum(outcome.discarded for outcome in self.outcomes),
        }
