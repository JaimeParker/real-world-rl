"""SiLRI: real-world RL from suboptimal human interventions (HIL-RL, arXiv 2512.24288).

An off-policy deterministic-mean actor-critic (no entropy term) whose actor is pulled
towards a learned model of the human's actions, with a state-wise Lagrange multiplier
that only enforces the pull where the human is consistent:

``beta(s) = N(mu_b(s), diag(sigma_b(s)^2))`` is fitted to intervened actions;
``lambda(s) = softplus(f(s))`` rises where ``||mu_pi(s) - mu_b(s)||_2`` exceeds the
tolerance ``sum_j sigma_b,j(s) + constraint_margin`` and decays elsewhere.

Per gradient step (``train``), ``utd`` critic updates on fresh mixed batches (online +
demo buffer, ``_sample_train_batch``), each followed by a Polyak update of both targets,
except that on the last batch the actor (+ gripper head) and the multiplier are updated
between the critic update and the target update::

    a'    = clamp(mu_target(s') + N(0, actor_noise_std^2), -1, 1)
    y     = r + (1 - done) * gamma * min_k Q_target_k(s', a')
    L_Q   = sum_k MSE(Q_k(s, a_cont), y)
    d     = ||mu_pi(s) - mu_b(s)||_2                       (beta, lambda held fixed)
    L_pi  = mean[(-min_k Q_k(s, mu_pi(s)) + lambda(s) * d) / (1 + lambda(s))]
    L_lam = -mean[lambda(s) * (d - sum_j sigma_b,j(s) - constraint_margin)]
    L_g   = CE(gripper logits, gripper label) over intervened rows (discrete_gripper)

The behaviour model is only trained on the demo buffer: ``pretrain`` (behaviour NLL plus
an actor NLL under the fixed ``actor_noise_std``, then ``actor_target <- actor``) and
``refresh_behavior`` (behaviour NLL), both on intervened rows only. The real-world
learner loop (``rl_garden_real_world.silri.SilriLearnerLoop``) schedules them.

Built on rl-garden's ``DDPG`` (constructor plumbing, encoders, checkpointing) and
``DemoInterventionMixin`` (the demo buffer and the mixed sampling). Both buffers are
``InterventionReplayBuffer``s that store ``intervened``, ``policy_action`` and
``intervention_onset``. There is no simulator loop: ``learn`` raises, so ``DDPG``'s
exploration-noise schedule (``_rollout_action``) is never used.
"""
from __future__ import annotations

import warnings
from typing import Any, NamedTuple, Optional, Sequence

import torch
import torch.nn.functional as F
from torch.distributions import Normal

from rl_garden.algorithms.ddpg import DDPG
from rl_garden.algorithms.observation_encoders import EncoderSharing
from rl_garden.algorithms.rlpd_hybrid import _discrete_labels_from_actions
from rl_garden.buffers.demo_intervention import DemoInterventionMixin
from rl_garden.common.logger import Logger
from rl_garden.common.optim import make_optimizer
from rl_garden.common.utils import polyak_update
from rl_garden.encoders.base import BaseFeaturesExtractor
from rl_garden.encoders.config import EncoderConfig
from rl_garden.encoders.factory import build_observation_encoder
from rl_garden.observations import (
    ObservationSchema,
    ObsGroups,
    normalize_observation_space,
    resolve_obs_groups,
)
from rl_garden_real_world.buffers.intervention_replay_buffer import (
    InterventionReplayBuffer,
)
from rl_garden_real_world.policies.silri_policy import SiLRIPolicy


def _masked_mean(values: torch.Tensor, mask: torch.Tensor) -> torch.Tensor:
    """Mean of ``values`` over the rows where ``mask`` is set; 0 when none is."""
    weights = mask.to(values.dtype)
    return (values * weights).sum() / weights.sum().clamp(min=1.0)


def _mean_info(logged: dict[str, list[torch.Tensor]]) -> dict[str, float]:
    return {key: float(torch.stack(values).mean().item()) for key, values in logged.items()}


class _ActorTerms(NamedTuple):
    actor_loss: torch.Tensor  # scalar, L_pi
    gripper_loss: Optional[torch.Tensor]  # scalar CE, None without a gripper head
    min_q: torch.Tensor  # (B,) detached
    distance: torch.Tensor  # (B,) detached ||mu_pi - mu_b||_2
    sigma_sum: torch.Tensor  # (B,) detached sum_j sigma_b,j
    multiplier: torch.Tensor  # (B,) lambda(s), differentiable w.r.t. the multiplier net


class SiLRI(DemoInterventionMixin, DDPG):
    """See the module docstring.

    ``utd`` is the number of critic updates per gradient step (one actor and one
    multiplier update each), so ``train(n)`` runs ``n * utd`` critic updates and
    ``global_update`` advances by ``n``. ``discrete_gripper`` treats the last
    ``num_grippers`` action dims as open/hold/close grippers predicted jointly by a
    classification head over ``3 ** num_grippers`` classes; the critic and the behaviour
    model then see the other dims only. ``learning_starts`` and
    ``demo_learning_starts`` are the online and demo buffer sizes the learner loop
    waits for. ``onset_to_demo_buffer`` is read by the learner loop (route the
    transition before an intervention onset to the demo buffer too; such rows keep
    ``intervened=False`` and so never train the behaviour model or the gripper head).
    """

    policy_class = SiLRIPolicy
    # The actor has its own encoder, trained by the actor loss (as in the reference).
    encoder_sharing: EncoderSharing = "separate"
    _compatible_checkpoint_algorithms = ("SiLRI",)
    _extra_batch_slice_keys = ("intervened", "policy_actions", "intervention_onset")
    # DDPG/OffPolicyAlgorithm hyperparameters SiLRI fixes and never reads.
    _UNUSED_PARENT_HYPERPARAMETERS = (
        "policy_lr",
        "q_lr",
        "nstep",
        "stddev_schedule",
        "stddev_clip",
        "num_expl_steps",
        "weight_decay",
        "use_adamw",
        "lr_schedule",
        "lr_warmup_steps",
        "lr_decay_steps",
        "lr_min_ratio",
        "replay_lazy_next_obs",
        "replay_pin_sampled_batch",
        "training_freq",
        "bootstrap_at_done",
    )

    def __init__(
        self,
        env: Any,
        eval_env: Optional[Any] = None,
        *,
        buffer_size: int = 30_000,
        buffer_device: str = "cuda",
        learning_starts: int = 10,
        demo_learning_starts: int = 10,
        batch_size: int = 256,
        gamma: float = 0.97,
        tau: float = 0.005,
        utd: int = 2,
        actor_lr: float = 3e-4,
        critic_lr: float = 3e-4,
        behavior_lr: float = 3e-4,
        lambda_lr: float = 3e-6,
        grad_clip_norm: Optional[float] = 40.0,
        actor_noise_std: float = 0.05,
        constraint_margin: float = 0.2,
        bc_pretrain_steps: int = 500,
        behavior_refresh_steps: int = 50,
        behavior_refresh_every: int = 50,
        std_min: float = 1e-5,
        std_max: float = 2.0,
        onset_to_demo_buffer: bool = False,
        feature_dim: int = 256,
        hidden_dim: int = 256,
        behavior_hidden_dims: Sequence[int] = (256, 256),
        lambda_hidden_dims: Sequence[int] = (256, 256),
        discrete_gripper: bool = False,
        discrete_hidden_dim: int = 256,
        num_grippers: int = 1,
        encoder_config: Optional[EncoderConfig] = None,
        obs_groups: Optional[ObsGroups] = None,
        critic_encoder_config: Optional[EncoderConfig] = None,
        encoder_sharing: Optional[EncoderSharing] = None,
        image_augmentation_seed: Optional[int] = None,
        policy_kwargs: Optional[dict[str, Any]] = None,
        seed: int = 1,
        device: str | torch.device = "auto",
        logger: Optional[Logger] = None,
        std_log: bool = True,
        log_freq: int = 1_000,
        num_eval_steps: int = 50,
        checkpoint_dir: Optional[str] = None,
        checkpoint_freq: int = 0,
        save_replay_buffer: bool = False,
        save_final_checkpoint: bool = True,
    ) -> None:
        if int(utd) != utd or utd < 1:
            raise ValueError(f"utd must be a positive integer (critic updates per step), got {utd}.")
        if actor_noise_std <= 0.0:
            raise ValueError(f"actor_noise_std must be positive, got {actor_noise_std}.")
        for name, value in (
            ("bc_pretrain_steps", bc_pretrain_steps),
            ("behavior_refresh_steps", behavior_refresh_steps),
            ("behavior_refresh_every", behavior_refresh_every),
            ("demo_learning_starts", demo_learning_starts),
        ):
            if value < 0:
                raise ValueError(f"{name} must be >= 0, got {value}.")
        # Read by _setup_model, which DDPG.__init__ calls last.
        self._init_prior_data_params()
        self.actor_lr = actor_lr
        self.critic_lr = critic_lr
        self.behavior_lr = behavior_lr
        self.lambda_lr = lambda_lr
        self.actor_noise_std = float(actor_noise_std)
        self.constraint_margin = float(constraint_margin)
        self.bc_pretrain_steps = int(bc_pretrain_steps)
        self.behavior_refresh_steps = int(behavior_refresh_steps)
        self.behavior_refresh_every = int(behavior_refresh_every)
        self.std_min = float(std_min)
        self.std_max = float(std_max)
        self.onset_to_demo_buffer = bool(onset_to_demo_buffer)
        self.demo_learning_starts = int(demo_learning_starts)
        self.behavior_hidden_dims = tuple(int(dim) for dim in behavior_hidden_dims)
        self.lambda_hidden_dims = tuple(int(dim) for dim in lambda_hidden_dims)
        self.discrete_gripper = bool(discrete_gripper)
        self.discrete_hidden_dim = int(discrete_hidden_dim)
        self.num_grippers = int(num_grippers)
        # Checkpointed progress counters.
        self.pretrained = False
        self.demo_additions_since_refresh = 0
        super().__init__(
            env,
            eval_env,
            buffer_size=buffer_size,
            buffer_device=buffer_device,
            learning_starts=learning_starts,
            batch_size=batch_size,
            gamma=gamma,
            tau=tau,
            training_freq=1,
            utd=int(utd),
            policy_lr=actor_lr,
            q_lr=critic_lr,
            feature_dim=feature_dim,
            hidden_dim=hidden_dim,
            nstep=1,
            grad_clip_norm=grad_clip_norm,
            # DDPG substitutes its DrQ-v2 encoder for None; SiLRI keeps rl-garden's default.
            encoder_config=encoder_config if encoder_config is not None else EncoderConfig(),
            obs_groups=obs_groups,
            critic_encoder_config=critic_encoder_config,
            encoder_sharing=encoder_sharing,
            image_augmentation_seed=image_augmentation_seed,
            policy_kwargs=policy_kwargs,
            seed=seed,
            device=device,
            logger=logger,
            std_log=std_log,
            log_freq=log_freq,
            num_eval_steps=num_eval_steps,
            checkpoint_dir=checkpoint_dir,
            checkpoint_freq=checkpoint_freq,
            save_replay_buffer=save_replay_buffer,
            save_final_checkpoint=save_final_checkpoint,
        )

    def learn(self, total_timesteps: int) -> "SiLRI":
        raise NotImplementedError(
            "SiLRI trains from transitions pushed by a real-world actor "
            "(rl_garden_real_world.silri.SilriLearnerLoop); it has no simulator rollout loop."
        )

    # ------------------------------------------------------------------
    # Construction
    # ------------------------------------------------------------------

    def _setup_model(self) -> None:
        # Not DDPG._setup_model: that one builds a DDPGPolicy and requires image observations.
        self.policy = self.policy_class(**self._policy_kwargs(self.env.single_observation_space)).to(
            self.device
        )
        policy = self.policy
        self._critic_params = list(policy.critic_and_encoder_parameters())
        self._actor_params = list(policy.actor_parameters())
        self._behavior_params = list(policy.behavior_parameters())
        self._lambda_params = list(policy.lambda_parameters())
        discrete_params = list(policy.discrete_parameters())

        self.q_optimizer = make_optimizer(self._critic_params, lr=self.critic_lr)
        self.actor_optimizer = make_optimizer(self._actor_params, lr=self.actor_lr)
        self.behavior_optimizer = make_optimizer(self._behavior_params, lr=self.behavior_lr)
        self.lambda_optimizer = make_optimizer(self._lambda_params, lr=self.lambda_lr)
        # The reference trains the gripper head with the actor's optimizer and learning rate.
        self.discrete_optimizer = (
            make_optimizer(discrete_params, lr=self.actor_lr) if discrete_params else None
        )
        self._lr_schedulers = []
        self.replay_buffer = self._build_replay_buffer()

    def _policy_kwargs(self, observation_space) -> dict[str, Any]:
        extractor_kwargs = self._policy_extractor_kwargs(
            observation_space, augmentation_seed=self._image_augmentation_seed
        )
        return {
            "observation_space": observation_space,
            "action_space": self.env.single_action_space,
            **extractor_kwargs,
            "behavior_extractor": self._build_auxiliary_extractor(observation_space),
            "lambda_extractor": self._build_auxiliary_extractor(observation_space),
            "feature_dim": self.feature_dim,
            "hidden_dim": self.hidden_dim,
            "actor_noise_std": self.actor_noise_std,
            "std_min": self.std_min,
            "std_max": self.std_max,
            "behavior_hidden_dims": self.behavior_hidden_dims,
            "lambda_hidden_dims": self.lambda_hidden_dims,
            "discrete_gripper": self.discrete_gripper,
            "discrete_hidden_dim": self.discrete_hidden_dim,
            "num_grippers": self.num_grippers,
        }

    def _build_auxiliary_extractor(self, observation_space) -> BaseFeaturesExtractor:
        """A fresh extractor over the actor's observation keys, built like the actor's
        (the behaviour model and the multiplier each own one)."""
        schema = ObservationSchema.from_space(normalize_observation_space(observation_space))
        actor_schema = resolve_obs_groups(schema, self.obs_groups)["actor"]
        return build_observation_encoder(
            observation_space,
            self.encoder_config,
            schema=actor_schema,
            augmentation_seed=self._image_augmentation_seed,
        )

    def _build_replay_buffer(self) -> InterventionReplayBuffer:
        return self._intervention_buffer(num_envs=self.num_envs, buffer_size=self.buffer_size)

    def _build_prior_data_buffer(self, buffer_size: int) -> InterventionReplayBuffer:
        return self._intervention_buffer(num_envs=1, buffer_size=buffer_size)

    def _intervention_buffer(self, *, num_envs: int, buffer_size: int) -> InterventionReplayBuffer:
        return InterventionReplayBuffer(
            self.env.single_observation_space,
            self.env.single_action_space,
            num_envs=num_envs,
            buffer_size=int(buffer_size),
            storage_device=self.buffer_device,
            sample_device=self.device,
            store_onset=True,
        )

    # ------------------------------------------------------------------
    # Demo-buffer bookkeeping (driven by the learner loop)
    # ------------------------------------------------------------------

    @property
    def demo_transitions(self) -> int:
        """Rows in the demo buffer (0 before ``init_demo_buffer``)."""
        return 0 if self.offline_replay_buffer is None else len(self.offline_replay_buffer)

    def record_demo_additions(self, count: int = 1) -> None:
        """Counts transitions added to the demo buffer since the last behaviour refresh."""
        self.demo_additions_since_refresh += int(count)

    @property
    def behavior_refresh_due(self) -> bool:
        """``behavior_refresh_every`` demo additions since the last refresh (0 disables)."""
        return (
            self.behavior_refresh_every > 0
            and self.demo_additions_since_refresh >= self.behavior_refresh_every
        )

    # ------------------------------------------------------------------
    # Losses
    # ------------------------------------------------------------------

    def _td_target(self, data) -> torch.Tensor:
        """``y = r + (1 - done) * gamma * min_k Q_target_k(s', a')``, ``(B, 1)``."""
        with torch.no_grad():
            next_action = self.policy.target_action(data.next_obs, self.actor_noise_std)
            next_features = self.policy.extract_critic_features(data.next_obs)
            next_q = self.policy.q_values_all(next_features, next_action, target=True)
            not_done = 1.0 - data.dones.reshape(-1, 1).float()
            return data.rewards.reshape(-1, 1) + not_done * self.gamma * next_q.min(dim=0).values

    def _critic_loss_from_batch(self, data) -> torch.Tensor:
        target_q = self._td_target(data)
        features = self.policy.extract_critic_features(data.obs)
        q_all = self.policy.q_values_all(
            features, self.policy.continuous_actions(data.actions), target=False
        )
        return self._critic_loss(q_all, target_q)

    def _actor_terms(self, data) -> _ActorTerms:
        obs = data.obs
        actor_features = self.policy.extract_actor_features(obs)
        actor_mean = self.policy.actor.deterministic_action(actor_features)
        with torch.no_grad():
            behavior_mean, behavior_std = self.policy.behavior(obs)
        multiplier = self.policy.lambda_value(obs)
        held_multiplier = multiplier.detach()
        distance = torch.linalg.vector_norm(actor_mean - behavior_mean, dim=-1)
        critic_features = self.policy.extract_critic_features(obs).detach()
        min_q = (
            self.policy.q_values_all(critic_features, actor_mean, target=False)
            .min(dim=0)
            .values.squeeze(-1)
        )
        actor_loss = ((-min_q + held_multiplier * distance) / (1.0 + held_multiplier)).mean()
        gripper_loss = None
        if self.policy.discrete_head is not None:
            gripper_loss = self._gripper_cross_entropy(
                self.policy.discrete_logits_from_features(actor_features), data
            )
        return _ActorTerms(
            actor_loss=actor_loss,
            gripper_loss=gripper_loss,
            min_q=min_q.detach(),
            distance=distance.detach(),
            sigma_sum=behavior_std.sum(dim=-1),
            multiplier=multiplier,
        )

    def _lambda_loss(
        self, multiplier: torch.Tensor, distance: torch.Tensor, sigma_sum: torch.Tensor
    ) -> torch.Tensor:
        """``-mean[lambda(s) * (d - sum_j sigma_b,j - margin)]``: descending it raises
        ``lambda`` where the actor is farther from the behaviour mean than tolerated."""
        excess_distance = distance.detach() - sigma_sum.detach() - self.constraint_margin
        return -(multiplier * excess_distance).mean()

    def _gripper_cross_entropy(self, logits: torch.Tensor, data) -> torch.Tensor:
        labels = _discrete_labels_from_actions(data.actions, self.num_grippers)
        return _masked_mean(F.cross_entropy(logits, labels, reduction="none"), data.intervened)

    def _behavior_nll(self, data) -> tuple[torch.Tensor, torch.Tensor]:
        """``-log N(a_cont; mu_b, diag(sigma_b^2))`` averaged over intervened rows, and
        the detached mean ``sum_j sigma_b,j``."""
        mean, std = self.policy.behavior(data.obs)
        log_prob = Normal(mean, std).log_prob(self.policy.continuous_actions(data.actions)).sum(-1)
        return _masked_mean(-log_prob, data.intervened), std.detach().sum(dim=-1).mean()

    def _actor_bc_nll(self, actor_features: torch.Tensor, data) -> torch.Tensor:
        """``-log N(a_cont; mu_pi, actor_noise_std^2 I)`` averaged over intervened rows."""
        mean = self.policy.actor.deterministic_action(actor_features)
        log_prob = (
            Normal(mean, self.actor_noise_std)
            .log_prob(self.policy.continuous_actions(data.actions))
            .sum(-1)
        )
        return _masked_mean(-log_prob, data.intervened)

    # ------------------------------------------------------------------
    # Updates
    # ------------------------------------------------------------------

    def _apply_gradients(
        self,
        loss: torch.Tensor,
        optimizers: Sequence[torch.optim.Optimizer],
        clipped_params: list[torch.nn.Parameter],
    ) -> None:
        for optimizer in optimizers:
            optimizer.zero_grad(set_to_none=True)
        loss.backward()
        if self.grad_clip_norm is not None:
            torch.nn.utils.clip_grad_norm_(clipped_params, self.grad_clip_norm)
        for optimizer in optimizers:
            optimizer.step()

    def _actor_optimizers(self) -> tuple[torch.optim.Optimizer, ...]:
        """The actor's optimizer and, with a gripper head, its own (one backward for both,
        as the reference's single actor optimizer; only actor gradients are clipped)."""
        if self.discrete_optimizer is None:
            return (self.actor_optimizer,)
        return (self.actor_optimizer, self.discrete_optimizer)

    def _critic_step(self, data) -> torch.Tensor:
        loss = self._critic_loss_from_batch(data)
        self._apply_gradients(loss, (self.q_optimizer,), self._critic_params)
        return loss.detach()

    def _actor_and_lambda_step(self, data) -> dict[str, torch.Tensor]:
        terms = self._actor_terms(data)
        actor_objective = (
            terms.actor_loss if terms.gripper_loss is None else terms.actor_loss + terms.gripper_loss
        )
        self._apply_gradients(actor_objective, self._actor_optimizers(), self._actor_params)
        lambda_loss = self._lambda_loss(terms.multiplier, terms.distance, terms.sigma_sum)
        self._apply_gradients(lambda_loss, (self.lambda_optimizer,), self._lambda_params)
        info = {
            "actor/loss": terms.actor_loss.detach(),
            "actor/q": terms.min_q.mean(),
            "actor/distance": terms.distance.mean(),
            "lambda/loss": lambda_loss.detach(),
            "lambda/mean": terms.multiplier.detach().mean(),
            "behavior/sigma_sum": terms.sigma_sum.mean(),
            "batch/intervened_fraction": data.intervened.float().mean(),
        }
        if terms.gripper_loss is not None:
            info["discrete/ce"] = terms.gripper_loss.detach()
        return info

    def _target_update(self) -> None:
        polyak_update(
            self.policy.critic.parameters(), self.policy.critic_target.parameters(), self.tau
        )
        self.policy.polyak_actor_target(self.tau)

    def _training_batch(self):
        data = self._sample_train_batch(self.batch_size)
        self.policy.prepare_batch_all(data.obs, data.next_obs)
        return data

    def train(self, gradient_steps: int, compute_info: bool = False) -> dict[str, float]:
        logged: dict[str, list[torch.Tensor]] = {}
        for _ in range(gradient_steps):
            self._global_update += 1
            for _ in range(int(self.utd) - 1):
                critic_loss = self._critic_step(self._training_batch())
                self._target_update()
                logged.setdefault("critic/loss", []).append(critic_loss)
            data = self._training_batch()
            critic_loss = self._critic_step(data)
            actor_info = self._actor_and_lambda_step(data)
            self._target_update()
            logged.setdefault("critic/loss", []).append(critic_loss)
            for key, value in actor_info.items():
                logged.setdefault(key, []).append(value)
        return _mean_info(logged) if compute_info else {}

    # ------------------------------------------------------------------
    # Demo-buffer phases
    # ------------------------------------------------------------------

    def _demo_buffer_ready(self, phase: str) -> bool:
        if self.demo_transitions == 0:
            warnings.warn(
                f"SiLRI.{phase} skipped: the demo buffer is empty.", RuntimeWarning, stacklevel=3
            )
            return False
        return True

    def _demo_batch(self):
        data = self.offline_replay_buffer.sample(self.batch_size)
        self.policy.prepare_batch_all(data.obs)
        return data

    def _behavior_step(self, data) -> tuple[torch.Tensor, torch.Tensor]:
        nll, sigma_sum = self._behavior_nll(data)
        self._apply_gradients(nll, (self.behavior_optimizer,), self._behavior_params)
        return nll.detach(), sigma_sum

    def pretrain(self, steps: Optional[int] = None) -> dict[str, float]:
        """Behaviour cloning on the demo buffer: per step one behaviour-model update and
        one actor update (actor NLL under ``actor_noise_std`` plus the gripper CE), then
        ``actor_target <- actor``. ``steps`` defaults to ``bc_pretrain_steps``. Skipped
        with a warning when the demo buffer is empty; sets ``pretrained``."""
        steps = self.bc_pretrain_steps if steps is None else int(steps)
        if not self._demo_buffer_ready("pretrain"):
            return {}
        logged: dict[str, list[torch.Tensor]] = {}
        for _ in range(steps):
            data = self._demo_batch()
            nll, sigma_sum = self._behavior_step(data)
            actor_features = self.policy.extract_actor_features(data.obs)
            bc_nll = self._actor_bc_nll(actor_features, data)
            bc_objective = bc_nll
            if self.policy.discrete_head is not None:
                gripper_loss = self._gripper_cross_entropy(
                    self.policy.discrete_logits_from_features(actor_features), data
                )
                bc_objective = bc_objective + gripper_loss
                logged.setdefault("discrete/ce", []).append(gripper_loss.detach())
            self._apply_gradients(bc_objective, self._actor_optimizers(), self._actor_params)
            logged.setdefault("behavior/nll", []).append(nll)
            logged.setdefault("behavior/sigma_sum", []).append(sigma_sum)
            logged.setdefault("actor/bc_nll", []).append(bc_nll.detach())
        self.policy.sync_actor_target()
        self.pretrained = True
        # Pretraining just fitted the behaviour model to every demo row.
        self.demo_additions_since_refresh = 0
        return _mean_info(logged)

    def refresh_behavior(self, steps: Optional[int] = None) -> dict[str, float]:
        """``steps`` (default ``behavior_refresh_steps``) behaviour-model updates on the
        demo buffer; resets the demo-addition counter. Skipped with a warning when the
        demo buffer is empty."""
        steps = self.behavior_refresh_steps if steps is None else int(steps)
        if not self._demo_buffer_ready("refresh_behavior"):
            return {}
        logged: dict[str, list[torch.Tensor]] = {}
        for _ in range(steps):
            nll, sigma_sum = self._behavior_step(self._demo_batch())
            logged.setdefault("behavior/nll", []).append(nll)
            logged.setdefault("behavior/sigma_sum", []).append(sigma_sum)
        self.demo_additions_since_refresh = 0
        return _mean_info(logged)

    # ------------------------------------------------------------------
    # Checkpoint
    # ------------------------------------------------------------------

    def _optimizer_names(self) -> tuple[str, ...]:
        return (
            *super()._optimizer_names(),
            "behavior_optimizer",
            "lambda_optimizer",
            "discrete_optimizer",
        )

    def _extra_checkpoint_state(self) -> dict[str, Any]:
        return {
            **super()._extra_checkpoint_state(),
            "pretrained": self.pretrained,
            "demo_additions_since_refresh": self.demo_additions_since_refresh,
        }

    def _load_extra_checkpoint_state(self, state: dict[str, Any]) -> None:
        super()._load_extra_checkpoint_state(state)
        self.pretrained = bool(state.get("pretrained", False))
        self.demo_additions_since_refresh = int(state.get("demo_additions_since_refresh", 0))

    def _checkpoint_metadata(self) -> dict[str, Any]:
        meta = super()._checkpoint_metadata()
        for key in self._UNUSED_PARENT_HYPERPARAMETERS:
            meta.pop(key, None)
        meta.update(
            {
                "actor_lr": self.actor_lr,
                "critic_lr": self.critic_lr,
                "behavior_lr": self.behavior_lr,
                "lambda_lr": self.lambda_lr,
                "actor_noise_std": self.actor_noise_std,
                "constraint_margin": self.constraint_margin,
                "bc_pretrain_steps": self.bc_pretrain_steps,
                "behavior_refresh_steps": self.behavior_refresh_steps,
                "behavior_refresh_every": self.behavior_refresh_every,
                "std_min": self.std_min,
                "std_max": self.std_max,
                "onset_to_demo_buffer": self.onset_to_demo_buffer,
                "demo_learning_starts": self.demo_learning_starts,
                "behavior_hidden_dims": list(self.behavior_hidden_dims),
                "lambda_hidden_dims": list(self.lambda_hidden_dims),
                "discrete_gripper": self.discrete_gripper,
                "discrete_hidden_dim": self.discrete_hidden_dim,
                "num_grippers": self.num_grippers,
            }
        )
        return meta
