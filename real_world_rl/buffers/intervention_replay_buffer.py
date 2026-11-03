"""Replay buffer that keeps human-intervention metadata next to each transition.

``InterventionReplayBuffer`` is a :class:`~rl_garden.buffers.replay_buffer.ReplayBuffer`
with extra ``(T, N, ...)`` columns for HIL-style learners: whether the human
overrode the policy at that step, the action the policy itself proposed, and,
optionally, the base policy's actions (residual RL) and the intervention onset
edge. ``add`` takes them as keyword arguments, and :meth:`sample` returns an
:class:`InterventionSample`.

Algorithms that sample from it extend ``SACCore._extra_batch_slice_keys`` with
:meth:`InterventionReplayBuffer.slice_keys`, so high-UTD minibatch slicing keeps
the extra fields.

Single-step transitions only (``nstep == 1``) and in-memory storage only
(no ``mmap_dir``): the extra columns are not covered by the n-step window or
the mmap manifest.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Optional

import torch
from gymnasium import spaces

from rl_garden.buffers.replay_buffer import ReplayBuffer
from rl_garden.common.types import ReplayBufferSample, TensorDict


@dataclass
class InterventionSample(ReplayBufferSample):
    """A mini-batch with intervention metadata (all on the sample device)."""

    intervened: torch.Tensor  # (B,) bool
    policy_actions: torch.Tensor  # (B, A) the action the policy proposed
    base_actions: Optional[torch.Tensor] = None  # (B, A), when store_base_actions
    next_base_actions: Optional[torch.Tensor] = None  # (B, A), when store_base_actions
    intervention_onset: Optional[torch.Tensor] = None  # (B,) bool, when store_onset
    grasp_penalty: Optional[torch.Tensor] = None  # (B,), when store_grasp_penalty


class InterventionReplayBuffer(ReplayBuffer):
    """``ReplayBuffer`` plus ``intervened`` / ``policy_actions`` columns.

    ``store_base_actions`` adds ``base_actions`` and ``next_base_actions``
    (the base policy's action at ``obs`` and ``next_obs``); ``store_onset``
    adds ``intervention_onset``. ``store_grasp_penalty`` is the base class's
    flag and fills ``InterventionSample.grasp_penalty``.

    ``add`` rejects an extra that is passed but not stored, and a stored extra
    that is missing, so a loop and buffer configured differently fail on the
    first transition.
    """

    def __init__(
        self,
        observation_space: spaces.Dict,
        action_space: spaces.Box,
        num_envs: int,
        buffer_size: int,
        storage_device: torch.device | str = "cuda",
        sample_device: torch.device | str = "cuda",
        mmap_dir: Optional[Any] = None,
        *,
        store_base_actions: bool = False,
        store_onset: bool = False,
        **replay_buffer_kwargs: Any,
    ) -> None:
        if mmap_dir is not None:
            raise ValueError(
                "InterventionReplayBuffer keeps its extra columns in memory only; "
                "mmap_dir is not supported."
            )
        super().__init__(
            observation_space,
            action_space,
            num_envs,
            buffer_size,
            storage_device=storage_device,
            sample_device=sample_device,
            **replay_buffer_kwargs,
        )
        self.store_base_actions = store_base_actions
        self.store_onset = store_onset

        shape = (self.per_env_buffer_size, num_envs)
        action_shape = shape + tuple(action_space.shape)
        self.intervened = torch.zeros(shape, dtype=torch.bool, device=self.storage_device)
        self.policy_actions = torch.zeros(
            action_shape, dtype=torch.float32, device=self.storage_device
        )
        if store_base_actions:
            self.base_actions = torch.zeros(
                action_shape, dtype=torch.float32, device=self.storage_device
            )
            self.next_base_actions = torch.zeros(
                action_shape, dtype=torch.float32, device=self.storage_device
            )
        if store_onset:
            self.intervention_onset = torch.zeros(
                shape, dtype=torch.bool, device=self.storage_device
            )

    def slice_keys(self) -> tuple[str, ...]:
        """The extra ``InterventionSample`` fields present on this buffer, for
        ``SACCore._extra_batch_slice_keys``."""
        keys = ["intervened", "policy_actions"]
        if self.store_base_actions:
            keys += ["base_actions", "next_base_actions"]
        if self.store_onset:
            keys.append("intervention_onset")
        if self.store_grasp_penalty:
            keys.append("grasp_penalty")
        return tuple(keys)

    def add(
        self,
        obs: TensorDict,
        next_obs: TensorDict,
        action: torch.Tensor,
        reward: torch.Tensor,
        done: torch.Tensor,
        *,
        intervened: torch.Tensor | bool,
        policy_action: Optional[torch.Tensor] = None,
        base_actions: Optional[torch.Tensor] = None,
        next_base_actions: Optional[torch.Tensor] = None,
        intervention_onset: Optional[torch.Tensor | bool] = None,
        grasp_penalty: Optional[torch.Tensor] = None,
    ) -> None:
        """``policy_action=None`` stores ``action`` as the policy's proposal.
        ``intervened`` and ``intervention_onset`` may be tensors broadcastable
        to ``(num_envs,)`` or Python bools."""
        self._check_extras(base_actions, next_base_actions, intervention_onset, grasp_penalty)

        pos = self.pos
        self.intervened[pos] = self._flag_to_storage(intervened)
        self.policy_actions[pos] = self._float_to_storage(
            action if policy_action is None else policy_action
        )
        if self.store_base_actions:
            self.base_actions[pos] = self._float_to_storage(base_actions)
            self.next_base_actions[pos] = self._float_to_storage(next_base_actions)
        if self.store_onset:
            self.intervention_onset[pos] = self._flag_to_storage(intervention_onset)

        super().add(obs, next_obs, action, reward, done, grasp_penalty=grasp_penalty)

    def _check_extras(
        self, base_actions, next_base_actions, intervention_onset, grasp_penalty
    ) -> None:
        if self.store_base_actions:
            if base_actions is None or next_base_actions is None:
                raise ValueError(
                    "buffer was constructed with store_base_actions=True; add() needs both "
                    "base_actions and next_base_actions."
                )
        elif base_actions is not None or next_base_actions is not None:
            raise ValueError(
                "base_actions/next_base_actions were passed but this buffer was constructed "
                "with store_base_actions=False."
            )
        if self.store_onset:
            if intervention_onset is None:
                raise ValueError(
                    "buffer was constructed with store_onset=True; add() needs intervention_onset."
                )
        elif intervention_onset is not None:
            raise ValueError(
                "intervention_onset was passed but this buffer was constructed with "
                "store_onset=False."
            )
        if grasp_penalty is not None and not self.store_grasp_penalty:
            raise ValueError(
                "grasp_penalty was passed but this buffer was constructed with "
                "store_grasp_penalty=False."
            )

    def _flag_to_storage(self, value: torch.Tensor | bool) -> torch.Tensor:
        return torch.as_tensor(value).to(device=self.storage_device, dtype=torch.bool)

    def _float_to_storage(self, value: torch.Tensor) -> torch.Tensor:
        return value.to(device=self.storage_device, dtype=torch.float32)

    def _index_batch(
        self, batch_inds: torch.Tensor, env_inds: torch.Tensor
    ) -> InterventionSample:
        base = super()._index_batch(batch_inds, env_inds)

        def column(storage: Optional[torch.Tensor]) -> Optional[torch.Tensor]:
            return None if storage is None else storage[batch_inds, env_inds].to(self.sample_device)

        return InterventionSample(
            obs=base.obs,
            next_obs=base.next_obs,
            actions=base.actions,
            rewards=base.rewards,
            dones=base.dones,
            intervened=column(self.intervened),
            policy_actions=column(self.policy_actions),
            base_actions=column(getattr(self, "base_actions", None)),
            next_base_actions=column(getattr(self, "next_base_actions", None)),
            intervention_onset=column(getattr(self, "intervention_onset", None)),
            grasp_penalty=getattr(base, "grasp_penalty", None),
        )
