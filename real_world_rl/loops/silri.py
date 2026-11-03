"""SiLRI (HIL-RL) actor/learner loops. Sync goes through the HIL-SERL sync classes."""
from __future__ import annotations

from typing import Any, Optional

import torch

from real_world_rl.loops.hil_serl import HilSerlActorLoop, HilSerlLearnerLoop
from real_world_rl.loops.learner import _TRANSITION_TENSOR_KEYS
from real_world_rl.training.demo_loading import load_demo_pkls

__all__ = ["SilriActorLoop", "SilriLearnerLoop"]


def _behavior_phase_metrics(phase: str, info: Optional[dict[str, float]]) -> dict[str, float]:
    """``pretrain()`` / ``refresh_behavior()`` info as ``<phase>/<key>`` with ``/`` in
    the key flattened to ``_`` (``behavior/nll`` -> ``refresh/behavior_nll``)."""
    return {f"{phase}/{key.replace('/', '_')}": value for key, value in (info or {}).items()}


class SilriActorLoop(HilSerlActorLoop):
    """SiLRI's ActorLoop: HIL-SERL's intervention tagging plus the policy's own proposal and
    an ``intervention_onset`` tag on the transition right before a human takes over.

    Pushes are delayed by one step: a transition is held back until the next one is known,
    and when the next one is intervened while the held one is not, the held one is pushed
    with ``intervention_onset=True``. The held transition is flushed when the episode ends,
    so a tag never crosses an episode boundary (a takeover at the first step of an episode
    tags nothing). A transition still held when :meth:`run` stops is pushed then, untagged
    (its successor is unknown).

    Pushed fields: the ``ActorLoop`` default (``obs``, ``next_obs``, the executed
    ``action``, ``reward``, ``done = terminated``) plus ``intervened``, ``policy_action``
    (``info["policy_action"]``, the policy's proposal) and ``intervention_onset``.
    """

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        self._held_transition: Optional[dict] = None

    def _build_transition(
        self,
        obs,
        next_obs,
        policy_action: torch.Tensor,
        reward: torch.Tensor,
        terminated: torch.Tensor,
        truncated: torch.Tensor,
        info: dict,
        ctx: dict,
    ) -> dict:
        transition = super()._build_transition(
            obs, next_obs, policy_action, reward, terminated, truncated, info, ctx
        )
        transition["policy_action"] = info.get("policy_action", policy_action)
        transition["intervention_onset"] = False
        return transition

    def _push(self, transition: dict) -> None:
        held = self._held_transition
        if held is not None:
            if transition["intervened"] and not held["intervened"]:
                held["intervention_onset"] = True
            super()._push(held)
        self._held_transition = transition

    def _flush_held_transition(self) -> None:
        if self._held_transition is not None:
            super()._push(self._held_transition)
            self._held_transition = None

    def _on_episode_end(self, info: dict) -> None:
        self._flush_held_transition()
        super()._on_episode_end(info)

    def _on_run_end(self) -> None:
        self._flush_held_transition()
        super()._on_run_end()


class SilriLearnerLoop(HilSerlLearnerLoop):
    """SiLRI's LearnerLoop: HIL-SERL's two-buffer routing and crash-recovery snapshots, plus
    SiLRI's schedule.

    - Every transition goes to the online buffer; intervened ones are also copied to the
      demo buffer, and so is the transition before an intervention onset when the agent's
      ``onset_to_demo_buffer`` is set. Each copy counts towards the agent's behaviour
      refresh (``agent.record_demo_additions``).
    - Buffer rows keep ``intervened``, ``policy_action`` and ``intervention_onset`` (the
      ``InterventionReplayBuffer`` columns) and drop every other extra field.
    - Training waits for ``learning_starts`` received transitions and ``demo_learning_starts``
      demo rows (both buffers, as the reference).
    - Before the first ``train`` of this loop, when the agent is not ``pretrained`` (a
      checkpoint restores the flag) and the demo buffer holds data, ``agent.pretrain()`` runs
      once. Before every ``train``, ``agent.refresh_behavior()`` runs when
      ``agent.behavior_refresh_due``. Both run in the training thread under the buffer lock,
      so transitions arriving meanwhile wait (the reference also blocks ingestion there).
    - One loop iteration calls ``agent.train(train_freq)``: SiLRI's ``utd`` is the number of
      critic updates inside each gradient step, not a multiplier of gradient steps.
    - Demo pkls without SiLRI's fields (``--demo_dataset_paths``) load as intervened,
      non-onset rows whose policy action is the recorded action.
    - With an agent logger, behaviour phases are logged when they run, at the iteration's
      metrics step: the first train step logs ``behavior/pretrained`` (0/1, also 1 for a
      restored pretrained agent) and, when pretraining ran, its losses as
      ``behavior/pretrain/<key>``; every refresh logs ``behavior/refresh/<key>`` and
      ``behavior/refresh_count``.
    """

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        self._pretrain_considered = False
        self._behavior_refresh_count = 0

    def _load_demo_datasets(self, paths) -> None:
        load_demo_pkls(
            paths,
            self._add_demo_with_defaults,
            self.agent.buffer_device,
            expected_contract=self._action_contract,
        )

    def _add_demo_with_defaults(self, obs, next_obs, action, reward, done, **extra) -> None:
        self.agent.add_demo_transition(
            obs,
            next_obs,
            action,
            reward,
            done,
            intervened=extra.get("intervened", True),
            policy_action=extra.get("policy_action"),
            intervention_onset=extra.get("intervention_onset", False),
        )

    def _buffer_kwargs(self, transition: dict[str, Any]) -> tuple[dict[str, Any], dict[str, Any]]:
        core = {key: transition[key] for key in _TRANSITION_TENSOR_KEYS}
        extra: dict[str, Any] = {
            "intervened": bool(transition.get("intervened", False)),
            "intervention_onset": bool(transition.get("intervention_onset", False)),
        }
        if transition.get("policy_action") is not None:
            extra["policy_action"] = transition["policy_action"]
        return core, extra

    def _routes_to_demo(self, transition: dict[str, Any]) -> bool:
        if bool(transition.get("intervened", False)):
            return True
        return self.agent.onset_to_demo_buffer and bool(transition.get("intervention_onset", False))

    def _on_transition(self, transition: dict[str, Any]) -> None:
        to_demo = self._routes_to_demo(transition)
        actor_id = transition.get("actor_id")
        core, extra = self._buffer_kwargs(transition)
        stored = {**core, **extra}
        device = self.agent.buffer_device
        with self._lock:
            self._add_transition(self.agent.replay_buffer.add, stored, device)
            self._pending_online.append(stored)
            if to_demo:
                self._add_transition(self.agent.add_demo_transition, stored, device)
                self._pending_demo.append(stored)
                self.agent.record_demo_additions(1)
            if stored["intervened"]:
                self._intervened_received += 1
            self._received += 1
            key = actor_id if actor_id is not None else "unknown"
            self._received_by_actor[key] = self._received_by_actor.get(key, 0) + 1
            self._step_since_snapshot += 1
            if self._step_since_snapshot >= self._buffer_period:
                self._snapshot()

    def _ready_to_train(self) -> bool:
        return super()._ready_to_train() and (
            self.agent.demo_transitions >= self.agent.demo_learning_starts
        )

    def _maybe_pretrain(self) -> dict[str, float]:
        """Pretrains once (see the class docstring). Returns the behaviour metrics of
        the first call (the ``pretrained`` flag, plus the losses when it pretrained)
        and an empty dict afterwards."""
        if self._pretrain_considered:
            return {}
        self._pretrain_considered = True
        metrics: dict[str, float] = {}
        if not self.agent.pretrained and self.agent.demo_transitions > 0:
            metrics = _behavior_phase_metrics("pretrain", self.agent.pretrain())
        metrics["pretrained"] = float(self.agent.pretrained)
        return metrics

    def _train_step(self, compute_info: bool = False) -> dict[str, float]:
        with self._lock:
            behavior_metrics = self._maybe_pretrain()
            if self.agent.behavior_refresh_due:
                behavior_metrics.update(
                    _behavior_phase_metrics("refresh", self.agent.refresh_behavior())
                )
                self._behavior_refresh_count += 1
                behavior_metrics["refresh_count"] = self._behavior_refresh_count
            info = self.agent.train(max(1, self.train_freq), compute_info=compute_info)
        # Outside the buffer lock: logging must not hold back transition ingestion.
        self._log_scalars("behavior", behavior_metrics, self._metrics_step)
        self._maybe_save_periodic_checkpoint()
        return info
