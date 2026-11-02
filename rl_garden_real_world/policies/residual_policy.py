"""Residual actor on top of RLPD's policies.

The actor sees the actor features concatenated with the base action's continuous
part and outputs a *unit residual* ``u`` in ``[-1, 1]``. The executed continuous
action is ``clamp(base + residual_scale * u, -1, 1)``; the critic scores that
composed action, exactly as it would score any env action.

``ResidualRLPDPolicy`` applies the residual to every env action dim (the
algorithm sets the gripper dim's scale, ``0`` by default). ``ResidualRLPDHybridPolicy``
applies it to the arm dims only; the gripper comes from the DQN head exactly as
in ``RLPDHybridPolicy.predict`` and the base's gripper entry is ignored.

Every method that takes ``base_actions`` expects the full env-width base action
``(B, A_env)`` (what a ``BaseActionProvider`` returns and what replay stores) and
slices the continuous part itself.
"""
from __future__ import annotations

import inspect
import math
from typing import Any, Optional, Sequence

import torch
import torch.nn as nn

from rl_garden.common.types import Obs
from rl_garden.networks.actor_critic import SquashedGaussianActor, get_actor_critic_arch
from rl_garden.policies.rlpd_hybrid_policy import RLPDHybridPolicy
from rl_garden.policies.rlpd_policy import RLPDPolicy

# Actor construction options RLPDPolicy (and RLPDHybridPolicy through it) accepts, with
# their defaults: the residual actor is rebuilt with exactly what the parent's was.
_RLPD_POLICY_DEFAULTS = {
    name: parameter.default
    for name, parameter in inspect.signature(RLPDPolicy.__init__).parameters.items()
    if parameter.default is not inspect.Parameter.empty
}


def _option(kwargs: dict[str, Any], name: str) -> Any:
    return kwargs.get(name, _RLPD_POLICY_DEFAULTS[name])


class ResidualActorMixin:
    """Replaces the parent's actor with a residual actor conditioned on the base action.

    ``residual_scale`` has one entry per continuous action dim of the policy (the
    policy's ``action_space``). With ``residual_zero_init`` the mean head starts at
    zero, so the deterministic residual is exactly ``0`` and the composed action is
    the base action. The log-std head always starts state-independent at
    ``residual_log_std_init`` (in log-std units, converted for ``log_std_mode``).
    """

    def __init__(
        self,
        *,
        residual_scale: Sequence[float],
        residual_zero_init: bool = True,
        residual_log_std_init: float = -3.0,
        **policy_kwargs: Any,
    ) -> None:
        super().__init__(**policy_kwargs)
        residual_dim = int(self.action_space.shape[0])
        scale = torch.as_tensor(list(residual_scale), dtype=torch.float32).reshape(-1)
        if scale.numel() != residual_dim:
            raise ValueError(
                f"residual_scale has {scale.numel()} entries, the residual acts on "
                f"{residual_dim} action dims"
            )
        if bool((scale < 0).any()):
            raise ValueError(f"residual_scale entries must be >= 0, got {scale.tolist()}")
        # Persistent: published to the actor and saved with the weights.
        self.register_buffer("residual_scale", scale)

        parent_actor = self.actor
        actor_arch, _ = get_actor_critic_arch(_option(policy_kwargs, "net_arch"))
        self.actor = SquashedGaussianActor(
            self._actor_fd + residual_dim,
            self.action_space,
            hidden_dims=actor_arch,
            use_layer_norm=_option(policy_kwargs, "actor_use_layer_norm"),
            dropout_rate=_option(policy_kwargs, "actor_dropout_rate"),
            kernel_init=_option(policy_kwargs, "kernel_init"),
            backbone_type=_option(policy_kwargs, "backbone_type"),
            use_pnorm=_option(policy_kwargs, "use_pnorm"),
            std_parameterization=parent_actor.std_parameterization,
            log_std_mode=parent_actor.log_std_mode,
            log_std_min=parent_actor.log_std_min,
            log_std_max=parent_actor.log_std_max,
        )
        if residual_zero_init:
            nn.init.zeros_(self.actor.fc_mean.weight)
            nn.init.zeros_(self.actor.fc_mean.bias)
        self._init_residual_log_std(residual_log_std_init)

    def _init_residual_log_std(self, log_std_init: float) -> None:
        actor = self.actor
        if not actor.log_std_min <= log_std_init <= actor.log_std_max:
            raise ValueError(
                f"residual_log_std_init={log_std_init} is outside the actor's log-std range "
                f"[{actor.log_std_min}, {actor.log_std_max}]"
            )
        raw = log_std_init
        if actor.log_std_mode == "tanh":
            # forward(): log_std = min + (max - min) * (tanh(raw) + 1) / 2
            normalized = 2.0 * (log_std_init - actor.log_std_min) / (actor.log_std_max - actor.log_std_min) - 1.0
            normalized = min(max(normalized, -1.0 + 1e-6), 1.0 - 1e-6)
            raw = math.atanh(normalized)
        with torch.no_grad():
            if actor.fc_logstd is not None:
                actor.fc_logstd.weight.zero_()
                actor.fc_logstd.bias.fill_(raw)
            else:
                actor.log_stds.fill_(raw)

    @property
    def residual_dim(self) -> int:
        return int(self.residual_scale.shape[0])

    # --- composition ---

    def _continuous_base(self, base_actions: Optional[torch.Tensor]) -> torch.Tensor:
        if base_actions is None:
            raise ValueError(
                f"{type(self).__name__} acts on top of a base action: pass base_actions "
                "(a BaseActionProvider's action for the same observations)."
            )
        return base_actions[..., : self.residual_dim]

    def compose(self, base_actions: torch.Tensor, unit: torch.Tensor) -> torch.Tensor:
        """``clamp(base + residual_scale * unit, -1, 1)`` over the residual dims."""
        return (self._continuous_base(base_actions) + self.residual_scale * unit).clamp(-1.0, 1.0)

    def _actor_input(self, features: torch.Tensor, base_actions: Optional[torch.Tensor]) -> torch.Tensor:
        base = self._continuous_base(base_actions).to(features)
        return torch.cat([self._transform_features_for_actor(features), base], dim=-1)

    def _env_action(self, obs: Obs, features: torch.Tensor, continuous_action: torch.Tensor) -> torch.Tensor:
        """The env action for a composed continuous action; the hybrid variant appends
        the DQN head's gripper."""
        del obs, features
        return continuous_action

    # --- inference ---

    def _unit_and_features(
        self, obs: Obs, base_actions: Optional[torch.Tensor], deterministic: bool
    ) -> tuple[torch.Tensor, torch.Tensor]:
        features = self.extract_actor_features(obs)
        actor_input = self._actor_input(features, base_actions)
        if deterministic:
            return self.actor.deterministic_action(actor_input), features
        unit, _ = self.actor.action_log_prob(actor_input)
        return unit, features

    def unit_residual(
        self, obs: Obs, base_actions: torch.Tensor, deterministic: bool = False
    ) -> torch.Tensor:
        """``(B, residual_dim)`` unit residual in ``[-1, 1]``."""
        return self._unit_and_features(obs, base_actions, deterministic)[0]

    def predict_with_unit(
        self,
        obs: Obs,
        deterministic: bool = False,
        *,
        base_actions: Optional[torch.Tensor] = None,
    ) -> tuple[torch.Tensor, torch.Tensor]:
        """``(env_action, unit_residual)``; ``predict`` is its first element."""
        unit, features = self._unit_and_features(obs, base_actions, deterministic)
        continuous = self.compose(base_actions, unit)
        return self._env_action(obs, features, continuous), unit

    def predict(
        self,
        obs: Obs,
        deterministic: bool = False,
        *,
        base_actions: Optional[torch.Tensor] = None,
    ) -> torch.Tensor:
        return self.predict_with_unit(obs, deterministic, base_actions=base_actions)[0]

    # --- training ---

    def residual_action_log_prob(
        self,
        obs: Obs,
        *,
        base_actions: Optional[torch.Tensor],
        stop_gradient: Optional[bool] = None,
    ) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor]:
        """``(composed continuous action, unit residual, log_prob, actor features)``.

        ``log_prob`` is the unit residual's (tanh-squashed Gaussian) log-density: the
        ``log(residual_scale)`` Jacobian of the composition is not included, so the
        entropy target applies to the unit residual. ``stop_gradient=None`` applies the
        policy's ``encoder_sharing`` rule, as ``SACPolicy.actor_action_log_prob`` does.
        """
        features = self._actor_role_features(obs, stop_gradient)
        unit, log_prob = self.actor.action_log_prob(self._actor_input(features, base_actions))
        return self.compose(base_actions, unit), unit, log_prob, features

    def actor_action_log_prob(
        self,
        obs: Obs,
        stop_gradient: Optional[bool] = None,
        detach_encoder: Optional[bool] = None,
        *,
        base_actions: Optional[torch.Tensor] = None,
    ) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        """``(composed continuous action, log_prob of the unit residual, actor features)``,
        the ``SACPolicy.actor_action_log_prob`` triple for the residual actor."""
        if detach_encoder is not None:
            stop_gradient = detach_encoder
        action, _, log_prob, features = self.residual_action_log_prob(
            obs, base_actions=base_actions, stop_gradient=stop_gradient
        )
        return action, log_prob, features

    def actor_diagnostics(
        self, obs: Obs, base_actions: Optional[torch.Tensor] = None
    ) -> dict[str, torch.Tensor]:
        """``SACPolicy.actor_diagnostics`` for the residual actor (statistics of the unit
        residual's distribution)."""
        with torch.no_grad():
            features = self.extract_features(obs, stop_gradient=True)
            mean, log_std = self.actor(self._actor_input(features, base_actions))
            std = log_std.exp()
            normal = torch.distributions.Normal(mean, std)
            x_t = normal.rsample()
            y_t = torch.tanh(x_t)
            gaussian_term = normal.log_prob(x_t).sum(-1)
            tanh_correction = torch.log(self.actor.action_scale * (1 - y_t.pow(2)) + 1e-6).sum(-1)
            return {
                "policy_std": std.mean(),
                "action_saturation": torch.tanh(mean).abs().mean(),
                "entropy_gaussian": -gaussian_term.mean(),
                "tanh_correction": tanh_correction.mean(),
            }


class ResidualRLPDPolicy(ResidualActorMixin, RLPDPolicy):
    """``RLPDPolicy`` with a residual actor over every env action dim."""


class ResidualRLPDHybridPolicy(ResidualActorMixin, RLPDHybridPolicy):
    """``RLPDHybridPolicy`` with a residual actor over the arm dims; the gripper comes
    from the DQN head (the base's gripper entry is ignored)."""

    def _env_action(self, obs: Obs, features: torch.Tensor, continuous_action: torch.Tensor) -> torch.Tensor:
        discrete_features = self.critic_features_for(obs, features, stop_gradient=True)
        discrete_index = self.discrete_critic(discrete_features).argmax(dim=-1)
        return torch.cat([continuous_action, self.discrete_action_from_index(discrete_index)], dim=-1)
