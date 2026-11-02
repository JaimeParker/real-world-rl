"""SiLRI policy: a deterministic tanh actor with fixed Gaussian sampling noise, a twin
critic, a learned human behaviour model and a state-wise Lagrange multiplier.

``SiLRIPolicy`` extends rl-garden's :class:`~rl_garden.policies.ddpg_policy.DDPGPolicy`
(actor, twin critic and critic target over the actor/critic extractors) with:

- ``actor_target``: a lagged copy of the actor head. It reads features through the
  actor's own extractor (only the head is lagged);
- ``behavior_model`` (beta): its own extractor, an MLP, a ``tanh`` mean head and a
  ``log_std`` head with ``std = clamp(exp(log_std), std_min, std_max)``, a diagonal
  Gaussian over the continuous action dims fitted to human actions;
- ``lambda_net``: its own extractor, an MLP with a zero-initialised output layer and a
  ``softplus``, so ``lambda(s) = softplus(0) = log 2`` at initialisation;
- ``discrete_head`` (only with ``discrete_gripper``): gripper logits over
  ``_DISCRETE_ACTION_VALUES`` (open/hold/close) on the actor's features.

With ``discrete_gripper`` the last env action dim is the gripper: the actor, critic and
behaviour model only see the continuous dims before it.
"""
from __future__ import annotations

import copy
from typing import Optional, Sequence

import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F
from gymnasium import spaces

from rl_garden.common.types import Obs
from rl_garden.common.utils import polyak_update
from rl_garden.encoders.base import BaseFeaturesExtractor
from rl_garden.networks.discrete_critic import DiscreteCritic
from rl_garden.networks.mlp import create_mlp
from rl_garden.policies.base import EncoderSharing
from rl_garden.policies.ddpg_policy import DDPGPolicy
from rl_garden.policies.rlpd_hybrid_policy import _DISCRETE_ACTION_VALUES

# Sampled actions stay strictly inside the tanh range (as the reference's select_action).
_ACTION_EPS = 1e-6
# Uniform init range of the behaviour model's output heads (the reference's init_final).
_HEAD_INIT_RANGE = 0.05


def _hidden_trunk(input_dim: int, hidden_dims: Sequence[int]) -> tuple[nn.Sequential, int]:
    """Linear -> LayerNorm -> SiLU blocks (the reference MLP block) and their output width."""
    hidden_dims = tuple(int(dim) for dim in hidden_dims)
    trunk = create_mlp(input_dim, -1, hidden_dims, activation_fn=nn.SiLU, use_layer_norm=True)
    return trunk, (hidden_dims[-1] if hidden_dims else input_dim)


class GaussianBehaviorModel(nn.Module):
    """Diagonal Gaussian ``N(mu_b(s), diag(sigma_b(s)^2))`` over the continuous action dims."""

    def __init__(
        self,
        extractor: BaseFeaturesExtractor,
        action_dim: int,
        hidden_dims: Sequence[int],
        *,
        std_min: float,
        std_max: float,
    ) -> None:
        super().__init__()
        if not 0.0 < std_min <= std_max:
            raise ValueError(f"need 0 < std_min <= std_max, got std_min={std_min}, std_max={std_max}.")
        self.extractor = extractor
        self.trunk, trunk_dim = _hidden_trunk(extractor.features_dim, hidden_dims)
        self.mean = nn.Linear(trunk_dim, action_dim)
        self.log_std = nn.Linear(trunk_dim, action_dim)
        for layer in (self.mean, self.log_std):
            nn.init.uniform_(layer.weight, -_HEAD_INIT_RANGE, _HEAD_INIT_RANGE)
            nn.init.uniform_(layer.bias, -_HEAD_INIT_RANGE, _HEAD_INIT_RANGE)
        self.std_min = float(std_min)
        self.std_max = float(std_max)

    def forward(self, obs: Obs) -> tuple[torch.Tensor, torch.Tensor]:
        hidden = self.trunk(self.extractor.extract(obs))
        mean = torch.tanh(self.mean(hidden))
        std = torch.exp(self.log_std(hidden)).clamp(self.std_min, self.std_max)
        return mean, std


class LagrangeMultiplierNet(nn.Module):
    """State-wise multiplier ``lambda(s) = softplus(f(s)) >= 0``, ``f`` zero at init."""

    def __init__(self, extractor: BaseFeaturesExtractor, hidden_dims: Sequence[int]) -> None:
        super().__init__()
        self.extractor = extractor
        self.trunk, trunk_dim = _hidden_trunk(extractor.features_dim, hidden_dims)
        self.output = nn.Linear(trunk_dim, 1)
        nn.init.zeros_(self.output.weight)
        nn.init.zeros_(self.output.bias)

    def forward(self, obs: Obs) -> torch.Tensor:
        hidden = self.trunk(self.extractor.extract(obs))
        return F.softplus(self.output(hidden)).squeeze(-1)


class SiLRIPolicy(DDPGPolicy):
    """See the module docstring. ``predict`` returns the tanh mean, plus
    ``N(0, actor_noise_std^2)`` noise unless ``deterministic``, clamped to
    ``[-1 + 1e-6, 1 - 1e-6]``; with ``discrete_gripper`` the gripper value of the
    ``discrete_head`` argmax is appended last.

    Parameter groups (one optimizer each): ``actor_parameters`` and
    ``critic_and_encoder_parameters`` (``DDPGPolicy``'s), ``behavior_parameters``,
    ``lambda_parameters`` and ``discrete_parameters``. ``actor_target`` and
    ``critic_target`` belong to none (Polyak-updated, no gradients).
    """

    def __init__(
        self,
        observation_space: spaces.Space,
        action_space: spaces.Box,
        *,
        actor_extractor: BaseFeaturesExtractor,
        behavior_extractor: BaseFeaturesExtractor,
        lambda_extractor: BaseFeaturesExtractor,
        critic_extractor: Optional[BaseFeaturesExtractor] = None,
        encoder_sharing: EncoderSharing = "separate",
        feature_dim: int = 256,
        hidden_dim: int = 256,
        actor_noise_std: float = 0.05,
        std_min: float = 1e-5,
        std_max: float = 2.0,
        behavior_hidden_dims: Sequence[int] = (256, 256),
        lambda_hidden_dims: Sequence[int] = (256, 256),
        discrete_gripper: bool = False,
        discrete_hidden_dim: int = 256,
    ) -> None:
        if not isinstance(action_space, spaces.Box):
            raise TypeError("SiLRIPolicy requires a Box action space.")
        if discrete_gripper:
            if int(np.prod(action_space.shape)) < 2:
                raise ValueError(
                    "discrete_gripper needs at least one continuous action dim before the gripper dim."
                )
            continuous_action_space = spaces.Box(
                low=action_space.low[:-1], high=action_space.high[:-1], dtype=action_space.dtype
            )
        else:
            continuous_action_space = action_space
        if actor_noise_std < 0.0:
            raise ValueError(f"actor_noise_std must be >= 0, got {actor_noise_std}.")
        super().__init__(
            observation_space,
            continuous_action_space,
            actor_extractor=actor_extractor,
            critic_extractor=critic_extractor,
            encoder_sharing=encoder_sharing,
            feature_dim=feature_dim,
            hidden_dim=hidden_dim,
        )
        self.env_action_space = action_space
        self.continuous_action_dim = int(np.prod(continuous_action_space.shape))
        self.actor_noise_std = float(actor_noise_std)
        self.discrete_gripper = bool(discrete_gripper)

        # Lagged copy of the actor head only; it reads the actor extractor's features.
        self.actor_target = copy.deepcopy(self.actor)
        for param in self.actor_target.parameters():
            param.requires_grad_(False)

        self.behavior_model = GaussianBehaviorModel(
            behavior_extractor,
            self.continuous_action_dim,
            behavior_hidden_dims,
            std_min=std_min,
            std_max=std_max,
        )
        self.lambda_net = LagrangeMultiplierNet(lambda_extractor, lambda_hidden_dims)

        self.discrete_head: Optional[DiscreteCritic] = None
        if self.discrete_gripper:
            self.discrete_head = DiscreteCritic(
                self.actor_features_dim,
                (int(discrete_hidden_dim),),
                n_actions=len(_DISCRETE_ACTION_VALUES),
                use_layer_norm=True,
            )
            self.register_buffer("_discrete_action_values", torch.tensor(_DISCRETE_ACTION_VALUES))

    # ------------------------------------------------------------------
    # Inference
    # ------------------------------------------------------------------

    def predict(self, obs: Obs, deterministic: bool = False) -> torch.Tensor:
        features = self.extract_actor_features(obs)
        action = self.actor.deterministic_action(features)
        if not deterministic:
            action = action + self.actor_noise_std * torch.randn_like(action)
        action = action.clamp(-1.0 + _ACTION_EPS, 1.0 - _ACTION_EPS)
        if self.discrete_head is None:
            return action
        gripper_index = self.discrete_head(features).argmax(dim=-1)
        return torch.cat([action, self.discrete_action_from_index(gripper_index)], dim=-1)

    def discrete_action_from_index(self, index: torch.Tensor) -> torch.Tensor:
        """The gripper value ``(B, 1)`` of each discrete index."""
        return self._discrete_action_values[index].unsqueeze(-1)

    # ------------------------------------------------------------------
    # Training helpers
    # ------------------------------------------------------------------

    def continuous_actions(self, actions: torch.Tensor) -> torch.Tensor:
        """The continuous part of env actions (all dims but the gripper under ``discrete_gripper``)."""
        return actions[..., : self.continuous_action_dim]

    def actor_mean(self, obs: Obs) -> torch.Tensor:
        """``mu_pi(s)``: the actor's tanh mean, ``(B, continuous_action_dim)``."""
        return self.actor.deterministic_action(self.extract_actor_features(obs))

    def target_action(self, next_obs: Obs, noise_std: float) -> torch.Tensor:
        """``clamp(mu_target(s') + N(0, noise_std^2), -1, 1)`` from ``actor_target``."""
        mean = self.actor_target.deterministic_action(self.extract_actor_features(next_obs))
        return (mean + noise_std * torch.randn_like(mean)).clamp(-1.0, 1.0)

    def behavior(self, obs: Obs) -> tuple[torch.Tensor, torch.Tensor]:
        """``(mu_b, sigma_b)`` of the behaviour model, each ``(B, continuous_action_dim)``."""
        return self.behavior_model(obs)

    def lambda_value(self, obs: Obs) -> torch.Tensor:
        """``lambda(s)``, ``(B,)``."""
        return self.lambda_net(obs)

    def discrete_logits(self, obs: Obs) -> torch.Tensor:
        """Gripper logits ``(B, 3)`` over ``_DISCRETE_ACTION_VALUES``."""
        return self.discrete_logits_from_features(self.extract_actor_features(obs))

    def discrete_logits_from_features(self, actor_features: torch.Tensor) -> torch.Tensor:
        if self.discrete_head is None:
            raise RuntimeError("this policy was built without a discrete gripper head.")
        return self.discrete_head(actor_features)

    def polyak_actor_target(self, tau: float) -> None:
        polyak_update(self.actor.parameters(), self.actor_target.parameters(), tau)

    def sync_actor_target(self) -> None:
        """``actor_target <- actor`` (after behaviour-cloning pretraining)."""
        self.actor_target.load_state_dict(self.actor.state_dict())

    def prepare_batch_all(self, obs: Obs, next_obs: Optional[Obs] = None) -> None:
        """Also prepares the behaviour and multiplier extractors (on ``obs`` only:
        neither reads ``next_obs``)."""
        super().prepare_batch_all(obs, next_obs)
        self.behavior_model.extractor.prepare_batch(obs)
        self.lambda_net.extractor.prepare_batch(obs)

    # ------------------------------------------------------------------
    # Parameter groups for optimizers
    # ------------------------------------------------------------------

    def behavior_parameters(self):
        yield from self.behavior_model.parameters()

    def lambda_parameters(self):
        yield from self.lambda_net.parameters()

    def discrete_parameters(self):
        if self.discrete_head is not None:
            yield from self.discrete_head.parameters()
