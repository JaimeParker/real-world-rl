"""Residual RL's actor loop: base action from a provider, residual from the policy.

Per step: the base action of ``obs`` (the one computed for the previous step's
``next_obs`` when the episode continues, else a fresh provider query), the unit
residual, and the composed env action. Until the first parameter payload
arrives the policy is untrained, so the unit residual is
``warmup_unit_residual * U(-1, 1)`` (``0`` by default: pure base actions) and
the gripper is the base's. Every pushed transition carries the base actions of
``obs`` and of the pre-reset ``next_obs``, the policy's own proposal and the
intervention flag; under intervention ``action`` is the human's.
"""
from __future__ import annotations

from typing import Any, Optional

import torch

from rl_garden.policies.base import BasePolicy
from rl_garden_real_world.hil_serl.actor_loop import HilSerlActorLoop
from rl_garden_real_world.policies.base_provider import BaseActionProvider
from rl_garden_real_world.sync import ActorSyncClient


class ResidualActorLoop(HilSerlActorLoop):
    def __init__(
        self,
        env: Any,
        policy: BasePolicy,
        sync_client: ActorSyncClient,
        base_provider: BaseActionProvider,
        *,
        warmup_unit_residual: float = 0.0,
        control_hz: float = 10.0,
        device: str | torch.device = "cpu",
        deterministic: bool = False,
        seed: int = 1,
    ) -> None:
        super().__init__(
            env,
            policy,
            sync_client,
            control_hz=control_hz,
            device=device,
            deterministic=deterministic,
            seed=seed,
        )
        if not 0.0 <= warmup_unit_residual <= 1.0:
            raise ValueError(f"warmup_unit_residual must be in [0, 1], got {warmup_unit_residual}")
        self.base_provider = base_provider
        self.warmup_unit_residual = float(warmup_unit_residual)
        # Base action of the next step's obs, computed while building this step's transition.
        self._next_base_actions: Optional[torch.Tensor] = None

    def _base_action(self, obs) -> torch.Tensor:
        return self.base_provider.select_action(obs).to(self.device)

    def _act(self, obs) -> tuple[torch.Tensor, dict]:
        base_actions = self._next_base_actions
        if base_actions is None:
            base_actions = self._base_action(obs)
        self._next_base_actions = None
        with torch.no_grad():
            if self.params_version == 0:
                unit = self.warmup_unit_residual * (
                    2.0 * torch.rand(base_actions.shape[0], self.policy.residual_dim, device=self.device) - 1.0
                )
                # Base gripper while warming up: the hybrid DQN head is untrained too.
                action = torch.cat(
                    [self.policy.compose(base_actions, unit), base_actions[..., self.policy.residual_dim :]],
                    dim=-1,
                )
            else:
                action, unit = self.policy.predict_with_unit(
                    self._obs_to_policy_device(obs),
                    deterministic=self.deterministic,
                    base_actions=base_actions,
                )
        return action, {"base_actions": base_actions, "policy_action": action, "unit": unit}

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
        # next_obs is the pre-reset observation, so this is its base action even when
        # the episode ends here; it is reused as the next step's base otherwise.
        self._next_base_actions = self._base_action(next_obs)
        transition["policy_action"] = ctx["policy_action"]
        transition["base_actions"] = ctx["base_actions"]
        transition["next_base_actions"] = self._next_base_actions
        return transition

    def _on_episode_end(self, info: dict) -> None:
        self.base_provider.reset()
        self._next_base_actions = None
