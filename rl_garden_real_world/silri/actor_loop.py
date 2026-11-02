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
from __future__ import annotations

from typing import Any, Optional

import torch

from rl_garden_real_world.hil_serl.actor_loop import HilSerlActorLoop


class SilriActorLoop(HilSerlActorLoop):
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
