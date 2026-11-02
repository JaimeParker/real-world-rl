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
"""
from __future__ import annotations

from typing import Any

from rl_garden_real_world.hil_serl.learner_loop import HilSerlLearnerLoop
from rl_garden_real_world.learner_loop import _TRANSITION_TENSOR_KEYS
from rl_garden_real_world.training.demo_loading import load_demo_pkls


class SilriLearnerLoop(HilSerlLearnerLoop):
    def __init__(self, *args: Any, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        self._pretrain_considered = False

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

    def _maybe_pretrain(self) -> None:
        if self._pretrain_considered:
            return
        self._pretrain_considered = True
        if not self.agent.pretrained and self.agent.demo_transitions > 0:
            self.agent.pretrain()

    def _train_step(self, compute_info: bool = False) -> dict[str, float]:
        with self._lock:
            self._maybe_pretrain()
            if self.agent.behavior_refresh_due:
                self.agent.refresh_behavior()
            info = self.agent.train(max(1, self.train_freq), compute_info=compute_info)
        self._maybe_save_periodic_checkpoint()
        return info
