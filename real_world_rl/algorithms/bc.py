"""Local masked BC, compatible with existing BC weights and optimizers."""
import torch
from rl_garden.algorithms.bc import BC
from real_world_rl.buffers.intervention_replay_buffer import InterventionReplayBuffer
from real_world_rl.algorithms.masked_imitation import human_mask, masked_sum_mean


class DemoReplayBuffer(InterventionReplayBuffer):
    """Offline dataset intent supplies legacy expert defaults, unlike online replay."""
    def add(self, obs, next_obs, action, reward, done, *, intervened=True,
            human_action_mask=None, override_mask=None, policy_action=None):
        super().add(obs, next_obs, action, reward, done, intervened=intervened,
                    human_action_mask=human_action_mask, override_mask=override_mask,
                    policy_action=policy_action)


class MaskedBC(BC):
    _compatible_checkpoint_algorithms = ("BC",)

    def _build_replay_buffer(self):
        return DemoReplayBuffer(self.env.single_observation_space, self.env.single_action_space,
            num_envs=self.num_envs, buffer_size=self.buffer_size,
            storage_device=self.buffer_device, sample_device=self.device)

    def _compute_losses(self, data):
        features = self.policy.extract_features(data.obs, stop_gradient=False)
        actor = self.policy.actor
        mean, log_std = actor(features)
        normal = torch.distributions.Normal(mean, log_std.exp())
        if self.tanh_squash:
            y = ((data.actions - actor.action_bias) / actor.action_scale).clamp(-1 + 1e-6, 1 - 1e-6)
            x = .5 * (torch.log1p(y) - torch.log1p(-y))
            nll = -normal.log_prob(x) + torch.log(actor.action_scale * (1 - y.square()) + 1e-6)
        else:
            nll = -normal.log_prob(data.actions)
        mask = human_mask(data)
        loss = masked_sum_mean(nll, mask)
        with torch.no_grad():
            error = (actor.deterministic_action(features) - data.actions).square()
            mse = (error * mask).sum() / mask.sum().clamp_min(1)
        return loss, dict(loss=float(loss.detach()), actor_loss=float(loss.detach()), behavior_log_prob=-float(loss.detach()), behavior_mse=float(mse))

    def train(self, gradient_steps: int, compute_info: bool = False):
        if gradient_steps <= 0:
            raise ValueError(f"gradient_steps must be positive, got {gradient_steps}.")
        metrics_sum = {}
        self.policy.train()
        for _ in range(gradient_steps):
            self._global_update += 1
            data = self._sample_train_batch()
            self.actor_optimizer.zero_grad(set_to_none=True)
            if not human_mask(data).any():
                metrics = dict(loss=0.0, actor_loss=0.0,
                               behavior_log_prob=0.0, behavior_mse=0.0)
            else:
                loss, metrics = self._compute_losses(data)
                loss.backward()
                self._clip_grad_norm()
                self.actor_optimizer.step()
                self._step_schedulers()
            for key, value in metrics.items():
                metrics_sum[key] = metrics_sum.get(key, 0.0) + value
        return {key: value / gradient_steps for key, value in metrics_sum.items()}

    def save(self, path, include_replay_buffer=False):
        from rl_garden.common.checkpoint import replay_buffer_path_for_checkpoint
        saved = super().save(path, include_replay_buffer=include_replay_buffer)
        if include_replay_buffer:
            self._save_extended_replay(replay_buffer_path_for_checkpoint(saved))
        return saved

    def _save_extended_replay(self, path):
        from pathlib import Path
        from real_world_rl.loops.replay_state import serialize_buffer
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        torch.save(serialize_buffer(self.replay_buffer), path)
        return path

    def load_replay_buffer(self, path, strict=True, *, _legacy_bc_supervision=False):
        from real_world_rl.loops.replay_state import restore_buffer
        state = torch.load(path, map_location="cpu", weights_only=False)
        if "real_world_replay_version" not in state and state.get("buffer_class") == "ReplayBuffer":
            state = {**state, "buffer_class": type(self.replay_buffer).__name__}
            if _legacy_bc_supervision:
                # The saved BC algorithm establishes offline supervision intent.
                # Preserve explicit labels; source and episode outcomes stay unknown.
                import warnings
                actions = state["actions"]
                if "intervened" not in state:
                    if "human_action_mask" in state:
                        state["intervened"] = state["human_action_mask"].any(-1)
                    else:
                        flags = torch.zeros(actions.shape[:-1], dtype=torch.bool)
                        flags[:] = state["full"]
                        if not state["full"]:
                            flags[:state["pos"]] = True
                        state["intervened"] = flags
                state.setdefault("human_action_mask", state["intervened"][..., None].expand_as(actions).clone())
                state.setdefault("override_mask", state["human_action_mask"].clone())
                state.setdefault("policy_actions", actions.clone())
                state["legacy_provenance_unknown"] = True
                warnings.warn("Legacy BC replay retains offline supervision intent; action source, "
                              "policy proposals and episode outcomes remain unknown.", stacklevel=2)
        restore_buffer(self.replay_buffer, state)

    def load(self, path, strict=True, load_replay_buffer=True, load_optimizers=True):
        from pathlib import Path
        from rl_garden.common.checkpoint import load_checkpoint_file, replay_buffer_path_for_checkpoint
        super().load(path, strict=strict, load_replay_buffer=False, load_optimizers=load_optimizers)
        if load_replay_buffer:
            checkpoint_path = Path(path)
            checkpoint = load_checkpoint_file(path, map_location="cpu")
            replay_ref = checkpoint.get("metadata", {}).get("replay_buffer_path")
            replay_path = (checkpoint_path.parent / replay_ref if replay_ref is not None
                           else replay_buffer_path_for_checkpoint(checkpoint_path))
            if replay_path.exists():
                self.load_replay_buffer(replay_path, strict=strict,
                                        _legacy_bc_supervision=checkpoint.get("metadata", {}).get("algorithm_class") == "BC")
            elif replay_ref is not None:
                raise FileNotFoundError(f"Checkpoint references replay buffer {replay_path}, but it does not exist.")
        return self
