"""ActorLoop: fixed-control-frequency rollout loop driving a real-robot env.

Deliberately independent of ``OffPolicyAlgorithm.learn()`` -- that loop
inlines rollout and gradient updates together for a single process; a real
robot's ``env.step()`` must run at a fixed wall-clock cadence and can never be
paused for a gradient update, so the actor only ever does inference (under
``torch.no_grad()``) against a policy module, never training.

HITL intervention / reward-classifier / safety-box logic all live in the env
wrapper stack (``rl_garden/envs/wrappers/`` and
``rl_garden_real_world/envs/wrappers/``) and are exercised transparently
through ``env.step()`` -- ``ActorLoop`` itself doesn't need to know about any
of them, it only reads ``info["intervene_action"]`` when present so the
replay data records the action that was actually executed, matching SERL's
convention for off-policy consistency under human intervention.

The per-step body of :meth:`ActorLoop.run` is split into overridable hooks
(``_act``, ``_build_transition`` (which calls ``_extra_transition_fields``), ``_push``,
``_on_episode_end``, ``_on_params_refreshed``, ``_on_run_end``) so subclasses can change what
is proposed, what is recorded and where it goes without copying the loop's pacing/reset
structure; the defaults are the plain SERL actor. Consumers: ``HilSerlActorLoop``
(``_extra_transition_fields``), ``ResidualActorLoop`` (``_act``, ``_build_transition``,
``_on_episode_end``) and ``SilriActorLoop`` (``_build_transition``, ``_push``,
``_on_episode_end``, ``_on_run_end``). ``FWBWActorLoop`` does not use the hooks.
"""
from __future__ import annotations

import time
from typing import Any, Optional

import torch

from rl_garden.policies.base import BasePolicy
from rl_garden_real_world.sync import ActorSyncClient


class ActorLoop:
    def __init__(
        self,
        env: Any,
        policy: BasePolicy,
        sync_client: ActorSyncClient,
        control_hz: float = 10.0,
        device: str | torch.device = "cpu",
        deterministic: bool = False,
        seed: int = 1,
    ) -> None:
        if getattr(env, "num_envs", 1) != 1:
            raise ValueError(
                f"ActorLoop drives exactly one real robot; got env.num_envs="
                f"{getattr(env, 'num_envs', None)!r}."
            )
        self.env = env
        self.policy = policy
        self.sync_client = sync_client
        self.control_period = 1.0 / control_hz
        self.device = torch.device(device)
        self.deterministic = deterministic
        self.seed = seed
        # Number of distinct parameter payloads loaded from the learner so far.
        self.params_version = 0
        self._last_loaded_params: Optional[dict] = None

    def _obs_to_policy_device(self, obs):
        if isinstance(obs, dict):
            return {k: v.to(self.device) for k, v in obs.items()}
        return obs.to(self.device)

    def _maybe_refresh_policy(self) -> None:
        params = self.sync_client.latest_policy_params()
        if params is not None:
            self.policy.load_state_dict(params)
            # The sync client caches one object per downloaded payload, so
            # identity tells a new payload from the same one served again.
            if params is not self._last_loaded_params:
                self._last_loaded_params = params
                self.params_version += 1
                self._on_params_refreshed(self.params_version)

    def _predict(self, obs) -> torch.Tensor:
        with torch.no_grad():
            return self.policy.predict(
                self._obs_to_policy_device(obs), deterministic=self.deterministic
            )

    def _act(self, obs) -> tuple[torch.Tensor, dict]:
        """The action sent to ``env.step`` for ``obs`` and a context dict that
        reaches :meth:`_build_transition` of the same step."""
        return self._predict(obs), {}

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
        """The transition dict pushed to the learner for one env step.
        ``policy_action`` and ``ctx`` are what :meth:`_act` returned; the
        stored action is the executed one (``info["intervene_action"]`` under
        human intervention). ``truncated`` is unused here and available to
        overrides."""
        del truncated, ctx
        return {
            "obs": obs,
            "next_obs": next_obs,
            "action": info.get("intervene_action", policy_action),
            "reward": reward,
            "done": terminated,
            **self._extra_transition_fields(info),
        }

    def _push(self, transition: dict) -> None:
        self.sync_client.push_transition(transition)

    def _on_episode_end(self, info: dict) -> None:
        """Called with the last step's ``info`` when the episode ended
        (terminated or truncated), before ``env.reset``. No-op by default."""

    def _on_params_refreshed(self, version: int) -> None:
        """Called after a new parameter payload was loaded into the policy;
        ``version`` counts payloads loaded by this loop, starting at 1.
        No-op by default."""

    def _on_run_end(self) -> None:
        """Called once when :meth:`run` stops (normally or by an exception),
        before the sync client is stopped, so a subclass can still push
        transitions it holds back. No-op by default."""

    def _extra_transition_fields(self, info: dict) -> dict:
        """Hook for subclasses that need to tag pushed transitions with
        extra, non-tensor metadata derived from ``info`` (e.g. HIL-SERL's
        human-intervention flag). No-op by default."""
        return {}

    def run(self, total_steps: Optional[int] = None) -> None:
        self.sync_client.start()
        self.policy.eval()
        try:
            obs, _ = self.env.reset(seed=self.seed)
            step = 0
            while total_steps is None or step < total_steps:
                loop_start = time.perf_counter()
                self._maybe_refresh_policy()

                policy_action, ctx = self._act(obs)
                env_action = policy_action.to(self.env_device(obs))
                next_obs, reward, terminated, truncated, info = self.env.step(env_action)

                self._push(
                    self._build_transition(
                        obs, next_obs, policy_action, reward, terminated, truncated, info, ctx
                    )
                )

                if bool(terminated) or bool(truncated):
                    self._on_episode_end(info)
                    obs, _ = self.env.reset(seed=self.seed)
                else:
                    obs = next_obs
                step += 1

                elapsed = time.perf_counter() - loop_start
                sleep_for = self.control_period - elapsed
                if sleep_for > 0:
                    time.sleep(sleep_for)
        finally:
            try:
                self._on_run_end()
            finally:
                self.sync_client.stop()

    @staticmethod
    def env_device(obs) -> torch.device:
        sample = next(iter(obs.values())) if isinstance(obs, dict) else obs
        return sample.device


class FWBWActorLoop:
    """Reset-free forward/backward variant of :class:`ActorLoop`.

    Drives one env wrapped with
    ``rl_garden.envs.wrappers.fwbw_reset_free.FWBWResetFreeWrapper`` (which
    must sit outermost, above ``RewardClassifierWrapper``), holding two full
    (policy, learner-sync-client) pairs and switching which pair does
    inference / receives each transition based on ``info["fwbw_direction"]``
    -- a single ``ActorLoop`` can't do this since it only ever knows about
    one policy and one learner.
    """

    def __init__(
        self,
        env: Any,
        policies: dict[str, BasePolicy],
        sync_clients: dict[str, ActorSyncClient],
        control_hz: float = 10.0,
        device: str | torch.device = "cpu",
        deterministic: bool = False,
        seed: int = 1,
    ) -> None:
        if getattr(env, "num_envs", 1) != 1:
            raise ValueError(
                f"FWBWActorLoop drives exactly one real robot; got env.num_envs="
                f"{getattr(env, 'num_envs', None)!r}."
            )
        if set(policies) != {"forward", "backward"} or set(sync_clients) != {"forward", "backward"}:
            raise ValueError(
                "policies and sync_clients must each have exactly 'forward' and "
                "'backward' keys."
            )
        self.env = env
        self.policies = policies
        self.sync_clients = sync_clients
        self.control_period = 1.0 / control_hz
        self.device = torch.device(device)
        self.deterministic = deterministic
        self.seed = seed

    def _obs_to_policy_device(self, obs):
        if isinstance(obs, dict):
            return {k: v.to(self.device) for k, v in obs.items()}
        return obs.to(self.device)

    def _extra_transition_fields(self, info: dict) -> dict:
        """See :meth:`ActorLoop._extra_transition_fields`. No-op by default."""
        return {}

    def run(self, total_steps: Optional[int] = None) -> None:
        for client in self.sync_clients.values():
            client.start()
        for policy in self.policies.values():
            policy.eval()
        try:
            obs, info = self.env.reset(seed=self.seed)
            direction = info.get("fwbw_direction", "forward")
            step = 0
            while total_steps is None or step < total_steps:
                loop_start = time.perf_counter()

                policy = self.policies[direction]
                sync_client = self.sync_clients[direction]
                params = sync_client.latest_policy_params()
                if params is not None:
                    policy.load_state_dict(params)

                with torch.no_grad():
                    policy_action = policy.predict(
                        self._obs_to_policy_device(obs), deterministic=self.deterministic
                    )
                env_action = policy_action.to(ActorLoop.env_device(obs))
                next_obs, reward, terminated, truncated, info = self.env.step(env_action)

                executed_action = info.get("intervene_action", policy_action)
                sync_client.push_transition(
                    {
                        "obs": obs,
                        "next_obs": next_obs,
                        "action": executed_action,
                        "reward": reward,
                        "done": terminated,
                        **self._extra_transition_fields(info),
                    }
                )

                direction = info.get("fwbw_direction", direction)
                if bool(terminated) or bool(truncated):
                    obs, info = self.env.reset(seed=self.seed)
                    direction = info.get("fwbw_direction", direction)
                else:
                    obs = next_obs
                step += 1

                elapsed = time.perf_counter() - loop_start
                sleep_for = self.control_period - elapsed
                if sleep_for > 0:
                    time.sleep(sleep_for)
        finally:
            for client in self.sync_clients.values():
                client.stop()
