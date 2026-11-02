"""Residual RL's learner loop: ``HilSerlLearnerLoop`` that keeps every extra field.

Transitions from ``ResidualActorLoop`` carry ``intervened``, ``policy_action``,
``base_actions``, ``next_base_actions`` (and ``grasp_penalty`` with the grasp
penalty wrapper); all of them reach ``InterventionReplayBuffer.add`` and the
crash-recovery snapshots. Only ``actor_id`` is dropped.

Pre-collected demo pkls (``demo_dataset_paths``) are human actions: a transition
without ``intervened`` / ``policy_action`` gets ``intervened=True`` and
``policy_action=action``. The base actions cannot be recovered on the learner, so a
demo pkl without ``base_actions`` / ``next_base_actions`` is an error, unless
``allow_base_less_demos`` is set: its rows then get zero base actions (with a
warning), and the critic target and the BC term learn them as residuals from a
zero base.
"""
from __future__ import annotations

import glob
import warnings
from typing import Any, Callable, Sequence

import torch

from rl_garden.algorithms.off_policy import OffPolicyAlgorithm
from rl_garden_real_world.hil_serl.learner_loop import HilSerlLearnerLoop
from rl_garden_real_world.learner_loop import _TRANSITION_TENSOR_KEYS
from rl_garden_real_world.training.demo_loading import load_demo_pkls

_BASE_FIELDS = ("base_actions", "next_base_actions")


class ResidualLearnerLoop(HilSerlLearnerLoop):
    def __init__(
        self,
        agent: OffPolicyAlgorithm,
        host: str,
        port: int,
        checkpoint_dir: str,
        *,
        allow_base_less_demos: bool = False,
        **hil_serl_kwargs: Any,
    ) -> None:
        # Set before super().__init__(): it loads the demo datasets.
        self.allow_base_less_demos = allow_base_less_demos
        self._warned_base_less_demos = False
        super().__init__(agent, host, port, checkpoint_dir, **hil_serl_kwargs)

    def _buffer_kwargs(self, transition: dict[str, Any]) -> tuple[dict[str, Any], dict[str, Any]]:
        core = {key: transition[key] for key in _TRANSITION_TENSOR_KEYS}
        extra = {
            key: value
            for key, value in transition.items()
            if key not in _TRANSITION_TENSOR_KEYS and key != "actor_id"
        }
        return core, extra

    def _load_demo_datasets(self, paths: Sequence[str]) -> None:
        for pattern in paths:
            for pkl_path in sorted(glob.glob(pattern)):
                load_demo_pkls(
                    [glob.escape(pkl_path)],
                    self._demo_adder(self.agent.add_demo_transition, pkl_path),
                    self.agent.buffer_device,
                    expected_contract=self._action_contract,
                )

    def _demo_adder(self, adder: Callable[..., None], pkl_path: str) -> Callable[..., None]:
        def add(obs, next_obs, action, reward, done, **extra: Any) -> None:
            missing = [key for key in _BASE_FIELDS if key not in extra]
            if missing:
                if not self.allow_base_less_demos:
                    raise ValueError(
                        f"demo pkl {pkl_path} has transitions without {missing}: the learner cannot "
                        "recover the base policy's actions. Record demos with the residual actor, or "
                        "pass --allow-base-less-demos to learn them as residuals from a zero base."
                    )
                if not self._warned_base_less_demos:
                    warnings.warn(
                        f"demo transitions without {missing} (first in {pkl_path}): loading them "
                        "with zero base actions (--allow-base-less-demos)",
                        stacklevel=2,
                    )
                    self._warned_base_less_demos = True
                for key in missing:
                    extra[key] = torch.zeros_like(action)
            extra.setdefault("intervened", True)
            extra.setdefault("policy_action", action)
            adder(obs, next_obs, action, reward, done, **extra)

        return add
