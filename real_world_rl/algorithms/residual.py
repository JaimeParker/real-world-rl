"""Residual SAC with human intervention: ``ResidualRLPD`` and ``ResidualRLPDHybrid``.

The policy learns a bounded correction on top of a frozen base policy's action
(``policies/residual_policy.py``). The learner never builds the base policy:
the actor stores the base action of every ``obs`` and ``next_obs`` in replay
(``InterventionReplayBuffer(store_base_actions=True)``), and the actor and TD
target condition on those stored base actions.

Under intervention the human overrides the *total* action. Replay stores the
human's action as ``actions`` (the critic learns its value as is, even outside
the residual box), plus the policy's own proposal, the base actions and the
``intervened`` flag. The optional BC term (``residual_bc_weight``) pulls the
unit residual towards the residual the human action implies,
``clip((a_human - base) / scale, -1, 1)``, on intervened rows.

``ResidualRLPD`` puts the residual on every env action dim (gripper scale
``residual_gripper_scale``, ``0`` by default) and has no DQN head;
``ResidualRLPDHybrid`` puts it on the arm dims and keeps ``RLPDHybrid``'s DQN
gripper head. Both take the env's last ``num_grippers`` action dims as the
grippers (one per arm, the ``action_contract`` layout), as every HIL env has
them (the intervention wrapper requires it). Real-world loops only:
``learn()``'s rollout and rl-garden's checkpoint evaluation call
``policy.predict`` without a base action, which the residual policy refuses.
"""
from __future__ import annotations

import dataclasses
from typing import Any, ClassVar, Optional

import torch

from rl_garden.algorithms.rlpd import RLPD
from rl_garden.algorithms.rlpd_hybrid import RLPDHybrid
from rl_garden.buffers.demo_intervention import DemoInterventionMixin
from rl_garden.common.training_phase import TrainingUpdateMask
from real_world_rl.buffers.intervention_replay_buffer import (
    InterventionReplayBuffer,
)
from real_world_rl.policies.residual_policy import (
    ResidualRLPDHybridPolicy,
    ResidualRLPDPolicy,
)


class ResidualMixin:
    """Residual actor, base-action replay, the BC term and the critic-only warm-up.

    ``critic_only_updates``: the first that many ``train()`` calls (as the learner
    loop makes them, one per training step) update the critics only; the actor and the
    temperature start afterwards. Implemented through the update mask
    (``_training_update_mask`` with ``update_actor=False``), so ``train`` and
    ``train_high_utd`` skip their actor steps; the count is saved with checkpoints.
    """

    checkpoint_eval_unsupported_reason: ClassVar[Optional[str]] = (
        "the residual policy acts on top of a base action provider that rl-garden-eval "
        "does not build; evaluate with residual_hil_serl --role eval or "
        "real_world_rl.deploy"
    )
    #: Whether the residual covers the env's gripper dim (scaled by
    #: ``residual_gripper_scale``) or leaves it to another head.
    _residual_covers_gripper: ClassVar[bool] = True

    def __init__(
        self,
        env: Any,
        eval_env: Optional[Any] = None,
        *,
        residual_action_scale: float = 0.1,
        residual_gripper_scale: float = 0.0,
        residual_zero_init: bool = True,
        residual_log_std_init: float = -3.0,
        residual_bc_weight: float = 0.0,
        base_provider_spec: str = "zero",
        critic_only_updates: int = 0,
        **kwargs: Any,
    ) -> None:
        if residual_action_scale <= 0:
            raise ValueError(f"residual_action_scale must be > 0, got {residual_action_scale}")
        if residual_gripper_scale < 0:
            raise ValueError(f"residual_gripper_scale must be >= 0, got {residual_gripper_scale}")
        if not self._residual_covers_gripper and residual_gripper_scale != 0:
            raise ValueError(
                f"{type(self).__name__} takes the gripper from its DQN head; "
                f"residual_gripper_scale must be 0, got {residual_gripper_scale}"
            )
        if residual_bc_weight < 0:
            raise ValueError(f"residual_bc_weight must be >= 0, got {residual_bc_weight}")
        if critic_only_updates < 0:
            raise ValueError(f"critic_only_updates must be >= 0, got {critic_only_updates}")
        # Set before super().__init__(): it builds the policy and the replay buffer.
        self.residual_action_scale = float(residual_action_scale)
        self.residual_gripper_scale = float(residual_gripper_scale)
        self.residual_zero_init = bool(residual_zero_init)
        self.residual_log_std_init = float(residual_log_std_init)
        self.residual_bc_weight = float(residual_bc_weight)
        self.base_provider_spec = base_provider_spec
        self.critic_only_updates = int(critic_only_updates)
        self._completed_train_calls = 0
        self._critic_only_train_call = False
        self._inside_train_call = False
        self._residual_actor_info: dict[str, torch.Tensor] = {}
        super().__init__(env, eval_env, **kwargs)
        slice_keys = list(self._extra_batch_slice_keys)
        slice_keys += [key for key in self.replay_buffer.slice_keys() if key not in slice_keys]
        self._extra_batch_slice_keys = tuple(slice_keys)

    # --- construction ---

    def residual_scale_vector(self) -> list[float]:
        """Per-dim residual scale: ``residual_action_scale`` on the arm dims, then
        ``residual_gripper_scale`` on each of the ``num_grippers`` gripper dims when the
        residual covers the grippers."""
        arm_dim = int(self.env.single_action_space.shape[0]) - self.num_grippers
        scales = [self.residual_action_scale] * arm_dim
        if self._residual_covers_gripper:
            scales.extend([self.residual_gripper_scale] * self.num_grippers)
        return scales

    def _policy_kwargs(self, observation_space) -> dict[str, Any]:
        return {
            **super()._policy_kwargs(observation_space),
            "residual_scale": self.residual_scale_vector(),
            "residual_zero_init": self.residual_zero_init,
            "residual_log_std_init": self.residual_log_std_init,
        }

    def _setup_model(self) -> None:
        super()._setup_model()
        # SAC resolves "auto" inline as -A_env; the entropy term here is the unit
        # residual's, whose dimension is the residual actor's (-A_cont).
        if self.target_entropy_arg == "auto":
            self.target_entropy = -float(self.policy.residual_dim)

    def _intervention_buffer_kwargs(self) -> dict[str, Any]:
        """Extra ``InterventionReplayBuffer`` keywords of the concrete algorithm."""
        return {}

    def _intervention_buffer(self, num_envs: int, buffer_size: int) -> InterventionReplayBuffer:
        if self.nstep != 1:
            raise ValueError(f"{type(self).__name__} needs nstep == 1, got {self.nstep}")
        if self.mmap_dir is not None:
            raise ValueError(f"{type(self).__name__} keeps replay in memory; mmap_dir is not supported")
        return InterventionReplayBuffer(
            observation_space=self.env.single_observation_space,
            action_space=self.env.single_action_space,
            num_envs=num_envs,
            buffer_size=buffer_size,
            storage_device=self.buffer_device,
            sample_device=self.device,
            store_base_actions=True,
            **self._intervention_buffer_kwargs(),
        )

    def _build_replay_buffer(self) -> InterventionReplayBuffer:
        return self._intervention_buffer(self.num_envs, self.buffer_size)

    def _build_prior_data_buffer(self, buffer_size: int) -> InterventionReplayBuffer:
        return self._intervention_buffer(1, buffer_size)

    # --- losses ---

    def _target_action_log_prob(self, data) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        return self.policy.actor_action_log_prob(
            data.next_obs, stop_gradient=True, base_actions=data.next_base_actions
        )

    def _residual_bc_loss(self, unit: torch.Tensor, data) -> tuple[torch.Tensor, torch.Tensor]:
        """``(masked mean BC loss, intervened fraction)``: the squared distance between
        the unit residual and ``clip((a_human - base) / scale, -1, 1)`` summed over the
        dims with a non-zero scale, averaged over intervened rows."""
        scale = self.policy.residual_scale
        residual_dim = scale.shape[0]
        active = scale > 0
        safe_scale = torch.where(active, scale, torch.ones_like(scale))
        human = data.actions[..., :residual_dim]
        base = data.base_actions[..., :residual_dim]
        target_unit = ((human - base) / safe_scale).clamp(-1.0, 1.0)
        per_row = ((unit - target_unit).pow(2) * active).sum(dim=-1)
        mask = data.intervened.reshape(-1).to(per_row.dtype)
        loss = (per_row * mask).sum() / mask.sum().clamp_min(1.0)
        return loss, mask.mean()

    def _actor_loss_from_batch(self, data) -> tuple[torch.Tensor, torch.Tensor]:
        alpha = self._current_alpha().detach()
        action, unit, log_prob, actor_features = self.policy.residual_action_log_prob(
            data.obs, base_actions=data.base_actions
        )
        critic_features = self.policy.critic_features_for(data.obs, actor_features, stop_gradient=True)
        min_q = self.policy.min_q_value(critic_features, action, subsample_size=None, target=False)
        loss = (alpha * log_prob - min_q).mean()
        bc_loss, intervened_fraction = self._residual_bc_loss(unit, data)
        if self.residual_bc_weight > 0:
            loss = loss + self.residual_bc_weight * bc_loss
        self._residual_actor_info = {
            "actor/residual_bc": bc_loss.detach(),
            "actor/intervened_fraction": intervened_fraction.detach(),
        }
        return loss, log_prob.detach()

    def _post_actor_update(self, data) -> dict[str, torch.Tensor]:
        return {**super()._post_actor_update(data), **self._residual_actor_info}

    def _compute_actor_diagnostics(self, data) -> dict[str, torch.Tensor]:
        return self.policy.actor_diagnostics(data.obs, data.base_actions)

    # --- critic-only warm-up ---

    def train(self, gradient_steps: int, compute_info: bool = False) -> dict[str, float]:
        # SACCore.train calls itself for a high-UTD remainder: only the outermost call counts.
        if self._inside_train_call:
            return super().train(gradient_steps, compute_info)
        self._inside_train_call = True
        self._critic_only_train_call = self._completed_train_calls < self.critic_only_updates
        try:
            return super().train(gradient_steps, compute_info)
        finally:
            self._inside_train_call = False
            self._critic_only_train_call = False
            self._completed_train_calls += 1

    def _training_update_mask(self) -> TrainingUpdateMask:
        mask = super()._training_update_mask()
        if self._critic_only_train_call:
            return dataclasses.replace(mask, update_actor=False)
        return mask

    # --- checkpoints ---

    def _training_state_dict(self) -> dict[str, Any]:
        return {
            **super()._training_state_dict(),
            "residual_completed_train_calls": self._completed_train_calls,
        }

    def _load_training_state_dict(self, state: dict[str, Any]) -> None:
        super()._load_training_state_dict(state)
        self._completed_train_calls = int(state.get("residual_completed_train_calls", 0))

    def _checkpoint_metadata(self) -> dict[str, Any]:
        return {
            **super()._checkpoint_metadata(),
            "residual_action_scale": self.residual_action_scale,
            "residual_gripper_scale": self.residual_gripper_scale,
            "residual_zero_init": self.residual_zero_init,
            "residual_log_std_init": self.residual_log_std_init,
            "residual_bc_weight": self.residual_bc_weight,
            "residual_scale": self.residual_scale_vector(),
            "num_grippers": self.num_grippers,
            "critic_only_updates": self.critic_only_updates,
            "base_provider": self.base_provider_spec,
        }


class ResidualRLPD(ResidualMixin, DemoInterventionMixin, RLPD):
    """Residual RLPD over every env action dim, with the growing intervention demo
    buffer of ``DemoInterventionMixin``. ``num_grippers`` (the env's trailing gripper
    dims) sets which dims get ``residual_gripper_scale``."""

    _compatible_checkpoint_algorithms = ("ResidualRLPD",)
    policy_class = ResidualRLPDPolicy

    def __init__(self, env: Any, eval_env: Optional[Any] = None, *, num_grippers: int = 1, **kwargs: Any) -> None:
        # Set before super().__init__(): it builds the policy from residual_scale_vector().
        self.num_grippers = int(num_grippers)
        super().__init__(env, eval_env, **kwargs)


class ResidualRLPDHybrid(ResidualMixin, RLPDHybrid):
    """Residual RLPD over the arm dims; the grippers are ``RLPDHybrid``'s DQN head
    (``num_grippers`` of them, an ``RLPDHybrid`` keyword), trained on the recorded
    gripper actions (the human's on intervened rows)."""

    _compatible_checkpoint_algorithms = ("ResidualRLPDHybrid",)
    policy_class = ResidualRLPDHybridPolicy
    _residual_covers_gripper = False

    def _intervention_buffer_kwargs(self) -> dict[str, Any]:
        if self.memory_efficient_buffer:
            raise ValueError(
                "ResidualRLPDHybrid stores base actions per row; memory_efficient_buffer "
                "is not supported"
            )
        return {"store_grasp_penalty": self.use_grasp_penalty}
