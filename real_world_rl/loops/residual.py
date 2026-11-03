"""Residual RL's actor and learner loops.

``ResidualActorLoop`` composes a base action from a provider with the policy's residual;
``ResidualLearnerLoop`` keeps every extra transition field for ``InterventionReplayBuffer``.
Sync goes through the HIL-SERL sync classes.
"""
from __future__ import annotations

import glob
import warnings
from typing import Any, Callable, Optional, Sequence

import torch

from rl_garden.algorithms.off_policy import OffPolicyAlgorithm
from rl_garden.policies.base import BasePolicy
from real_world_rl.loops.hil_serl import HilSerlActorLoop, HilSerlLearnerLoop
from real_world_rl.loops.learner import _TRANSITION_TENSOR_KEYS
from real_world_rl.loops.sync import ActorSyncClient
from real_world_rl.policies.base_provider import BaseActionProvider
from real_world_rl.training.demo_loading import load_demo_pkls

__all__ = ["ResidualActorLoop", "ResidualLearnerLoop"]

_BASE_FIELDS = ("base_actions", "next_base_actions")


class ResidualActorLoop(HilSerlActorLoop):
    """Residual RL's actor loop: base action from a provider, residual from the policy.

    Per step: the base action of ``obs`` (the one computed for the previous step's
    ``next_obs`` when the episode continues, else a fresh provider query), the unit
    residual, and the composed env action. Until the first parameter payload
    arrives the policy is untrained, so the unit residual is
    ``warmup_unit_residual * U(-1, 1)`` (``0`` by default: pure base actions) and
    the gripper is the base's. Every pushed transition carries the base actions of
    ``obs`` and of the pre-reset ``next_obs``, the policy's own proposal and the
    intervention flag; under intervention ``action`` is the human's.

    The provider is reset at every episode end and, with ``base_resync_on_intervention``
    (default on), also after every intervened step, before the base of that step's
    ``next_obs`` is computed. While the human acts the robot leaves the trajectory a
    stateful provider (ACT) planned for: without the resync it keeps replaying a stale
    open-loop chunk (``temporal_agg=0``) or keeps averaging chunks predicted before and
    during the takeover into its temporal ensemble (``temporal_agg=1``), so the stored
    ``base_actions`` / ``next_base_actions`` and the base the policy acts on after the
    release lag the robot by up to one chunk. With the resync the base of the obs after
    an intervened step is the first action of a chunk freshly planned at that obs (no
    stale chunk, no ensemble history), so the first step after the human releases acts on
    a base planned at its own observation. The step on which the human takes over keeps
    the base computed before the takeover: nothing can know it in advance. Stateless
    providers (``zero``, ``checkpoint``) have a no-op ``reset``, so for them the option
    changes nothing.
    """

    def __init__(
        self,
        env: Any,
        policy: BasePolicy,
        sync_client: ActorSyncClient,
        base_provider: BaseActionProvider,
        *,
        warmup_unit_residual: float = 0.0,
        base_resync_on_intervention: bool = True,
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
        if not isinstance(base_resync_on_intervention, bool):
            raise TypeError(
                f"base_resync_on_intervention must be a bool, got {base_resync_on_intervention!r}"
            )
        self.base_provider = base_provider
        self.warmup_unit_residual = float(warmup_unit_residual)
        self.base_resync_on_intervention = base_resync_on_intervention
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
            if not self.policy_ready:
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
        if self.base_resync_on_intervention and transition["intervened"]:
            # The human moved the robot off the provider's plan: replan from next_obs.
            self.base_provider.reset()
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
        super()._on_episode_end(info)


class ResidualLearnerLoop(HilSerlLearnerLoop):
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
            if key in ("intervened", "policy_action", "base_actions", "next_base_actions", "intervention_onset", "grasp_penalty", "human_action_mask", "override_mask", "episode_end")
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
            # This adapter accepts **extra, but its actual replay target has a
            # strict add contract. Canonical demo rows also project episode_end,
            # which ordinary residual replay does not store.
            from real_world_rl.training.demo_loading import training_extras
            projected = training_extras(extra, self.agent.offline_replay_buffer.add)
            adder(obs, next_obs, action, reward, done, **projected)

        return add
