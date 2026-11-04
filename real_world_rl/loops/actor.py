"""ActorLoop: fixed-control-frequency rollout loop driving a real-robot env.

Deliberately independent of ``OffPolicyAlgorithm.learn()`` -- that loop
inlines rollout and gradient updates together for a single process; a real
robot's ``env.step()`` must run at a fixed wall-clock cadence and can never be
paused for a gradient update, so the actor only ever does inference (under
``torch.no_grad()``) against a policy module, never training.

HITL intervention / reward-classifier / safety-box logic all live in the env
wrapper stack (``rl_garden/envs/wrappers/`` and
``real_world_rl/envs/wrappers/``) and are exercised transparently
through ``env.step()`` -- ``ActorLoop`` itself doesn't need to know about any
of them, it only reads ``info["intervene_action"]`` when present so the
replay data records the action that was actually executed, matching SERL's
convention for off-policy consistency under human intervention. An
``operator_quit`` teleop event (the operator's ``q`` key) ends the episode and
stops the loop, which then cleans up as after Ctrl-C.

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
import logging
import sys
from typing import Any, Optional

import torch

from robot_infra.supervise import Heartbeat

from rl_garden.policies.base import BasePolicy
from real_world_rl.loops.sync import ActorSyncClient
from real_world_rl.checkpoint_contract import load_policy_state_with_action_contract
from real_world_rl.teleop.keyboard import operator_quit_requested


logger = logging.getLogger(__name__)


def _cleanup(actions) -> None:
    """Attempt every release; preserve the active rollout exception."""
    active_exception = sys.exc_info()[0] is not None
    first_error = None
    for action in actions:
        try:
            action()
        except BaseException as exc:
            logger.exception("rollout cleanup failed")
            if first_error is None:
                first_error = exc
    if first_error is not None and not active_exception:
        raise first_error


def _close(resource) -> None:
    close = getattr(resource, "close", None)
    if close is not None:
        close()


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
        self._last_snapshot_key = None
        self.policy_ready = False
        self.policy_phase = "warmup"
        self._trajectory_episode = None
        self._trajectory_step = 0
        self._trajectory_outcome = None
        self._snapshot_metadata = {}

    def _obs_to_policy_device(self, obs):
        if isinstance(obs, dict):
            return {k: v.to(self.device) for k, v in obs.items()}
        return obs.to(self.device)

    def _maybe_refresh_policy(self) -> None:
        snapshot_reader = getattr(self.sync_client, "latest_policy_snapshot", None)
        if snapshot_reader is not None:
            snapshot = snapshot_reader()
            if snapshot is None:
                return
            key = (snapshot.session_id, snapshot.version)
            if key == self._last_snapshot_key:
                return
            self._validate_snapshot_experiment(snapshot)
            load_policy_state_with_action_contract(self.policy, snapshot.state_dict)
            self._snapshot_metadata = {"policy_version": snapshot.version, "policy_session_id": snapshot.session_id,
                                       "global_update": snapshot.global_update, "policy_phase": snapshot.phase}
            self._last_snapshot_key = key
            self.policy_ready = snapshot.policy_ready
            self.policy_phase = snapshot.phase
        else:
            params = self.sync_client.latest_policy_params()
            if params is None or params is self._last_loaded_params:
                return
            load_policy_state_with_action_contract(self.policy, params)
            self._last_loaded_params = params
            self.policy_ready = True
        self.params_version += 1
        self._on_params_refreshed(self.params_version)

    def _validate_snapshot_experiment(self, snapshot) -> None:
        saved_hash = getattr(snapshot, "experiment_hash", "unknown")
        actual_hash = getattr(self.sync_client, "experiment_hash", "unknown")
        if saved_hash not in (None, "unknown") and actual_hash not in (None, "unknown") and saved_hash != actual_hash:
            raise ValueError("learner policy experiment hash differs from actor")
        saved = getattr(snapshot, "effective_experiment", None) or getattr(snapshot, "experiment_contract", None)
        if saved is not None:
            from real_world_rl.action_contract import validate_experiment
            validate_experiment(self.env, saved)

    def _bind_experiment(self) -> None:
        if not hasattr(self.sync_client, "push_record"):
            return
        from real_world_rl.action_contract import experiment_contract_of, experiment_hash
        self.sync_client.bind_experiment(experiment_hash(self.env), experiment_contract_of(self.env))

    def _trajectory_metadata(self) -> dict:
        return {"run_id": self.sync_client.run_id, "actor_id": self.sync_client._actor_id or "unknown",
                "session_id": self.sync_client.session_id, "sequence": self.sync_client._sequence,
                "timestamp_s": time.time(), "experiment_hash": self.sync_client.experiment_hash,
                **self._snapshot_metadata}

    def _begin_trajectory(self) -> None:
        if not hasattr(self.sync_client, "push_record"):
            return
        from real_world_rl.trajectory import begin_episode
        from real_world_rl.metrics import EpisodeOutcomeAccumulator
        from real_world_rl.action_contract import effective_experiment_of, save_experiment_metadata
        save_experiment_metadata(self.env, self.sync_client._outbox.path.with_name("experiment.json"))
        self._trajectory_episode = self.sync_client._new_message_id()
        self._trajectory_step = 0
        self._trajectory_outcome = EpisodeOutcomeAccumulator()
        self.sync_client.push_record(begin_episode(self._trajectory_episode,
                                    effective_experiment=effective_experiment_of(self.env),
                                    **self._trajectory_metadata()))

    def _archive_transition(self, transition, terminated, truncated, info, policy_action=None, segment_end=False):
        if self._trajectory_episode is None:
            return transition
        from real_world_rl.trajectory import transition_record
        self._trajectory_outcome.observe(info)
        action = transition["action"]
        transition.setdefault("policy_action", action if policy_action is None else policy_action)
        human = info.get("human_action_mask")
        if human is None:
            human = torch.full_like(action, "intervene_action" in info, dtype=torch.bool)
        human = torch.as_tensor(human, device=action.device, dtype=torch.bool).reshape_as(action)
        override = info.get("override_mask", human)
        transition.update(human_action_mask=human,
                          override_mask=torch.as_tensor(override, device=action.device, dtype=torch.bool).reshape_as(action),
                          intervened=bool(human.any()), syncing=bool(info.get("syncing", False)))
        identity = self.sync_client._new_message_id()
        record = transition_record(transition, episode_id=self._trajectory_episode,
                                   step_index=self._trajectory_step, record_id=identity,
                                   terminated=bool(terminated), truncated=bool(truncated), segment_end=bool(segment_end),
                                   executed_action=action, reward_components=info.get("reward_components", {}),
                                   reward_source=info.get("reward_source", "unknown"),
                                   outcome_source=info.get("task_success_source", "unknown"),
                                   **self._trajectory_metadata())
        self._trajectory_step += 1
        return record

    def _end_trajectory(self, *, terminated=False, truncated=False, interrupted=False, end_reason=None) -> None:
        if self._trajectory_episode is None:
            return
        from real_world_rl.trajectory import end_episode
        outcome = self._trajectory_outcome.finish(terminated=terminated, truncated=truncated, interrupted=interrupted, end_reason=end_reason)
        self.sync_client._new_message_id()  # end events also advance the actor sequence
        episode_id = self._trajectory_episode
        self._trajectory_episode = None
        self.sync_client.push_record(end_episode(episode_id, step_count=self._trajectory_step,
                                    outcome=outcome, interrupted=interrupted,
                                    transport_metrics=self.sync_client.transport_metrics(), **self._trajectory_metadata()))
        self._trajectory_episode = None

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
        heartbeat = Heartbeat.from_env()
        try:
            self._bind_experiment()
            self.sync_client.start()
            self.policy.eval()
            wait = getattr(self.sync_client, "wait_for_policy", None)
            if wait is not None:
                wait()
            self._maybe_refresh_policy()
            heartbeat.begin_long_operation()
            obs, _ = self.env.reset(seed=self.seed)
            heartbeat.beat()
            self._begin_trajectory()
            step = 0
            while total_steps is None or step < total_steps:
                loop_start = time.perf_counter()
                self._maybe_refresh_policy()

                policy_action, ctx = self._act(obs)
                env_action = policy_action.to(self.env_device(obs))
                next_obs, reward, terminated, truncated, info = self.env.step(env_action)
                heartbeat.beat()

                self._push(
                    self._archive_transition(self._build_transition(
                        obs, next_obs, policy_action, reward, terminated, truncated, info, ctx
                    ), terminated, truncated, info, policy_action)
                )

                if bool(terminated) or bool(truncated) or operator_quit_requested(info):
                    self._on_episode_end(info)
                    self._end_trajectory(terminated=bool(terminated), truncated=bool(truncated))
                    if operator_quit_requested(info):
                        logger.info("operator quit: stopping the actor after this episode")
                        break
                    if total_steps is None or step + 1 < total_steps:
                        heartbeat.begin_long_operation()
                        obs, _ = self.env.reset(seed=None)
                        heartbeat.beat()
                        self._begin_trajectory()
                else:
                    obs = next_obs
                step += 1

                elapsed = time.perf_counter() - loop_start
                sleep_for = self.control_period - elapsed
                if sleep_for > 0:
                    time.sleep(sleep_for)
        finally:
            heartbeat.begin_long_operation()  # draining uploads and closing the env may take a while
            _cleanup([
                lambda: _close(self.env),
                lambda: _close(getattr(self, "base_provider", None)),
                self._on_run_end,
                lambda: self._end_trajectory(interrupted=True),
                self.sync_client.stop,
            ])

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
        snapshots = {}
        heartbeat = Heartbeat.from_env()
        archives = {direction: ActorLoop(self.env, self.policies[direction], client,
                                         device=self.device, deterministic=self.deterministic, seed=self.seed)
                    for direction, client in self.sync_clients.items()}
        try:
            for direction, client in self.sync_clients.items():
                archives[direction]._bind_experiment()
                client.start()
                wait = getattr(client, "wait_for_policy", None)
                if wait is not None:
                    snapshot = wait()
                    archives[direction]._validate_snapshot_experiment(snapshot)
                    load_policy_state_with_action_contract(self.policies[direction], snapshot.state_dict)
                    archives[direction]._snapshot_metadata = {"policy_version": snapshot.version,
                        "policy_session_id": snapshot.session_id, "global_update": snapshot.global_update, "policy_phase": snapshot.phase}
                    snapshots[direction] = (snapshot.session_id, snapshot.version)
            for policy in self.policies.values():
                policy.eval()
            heartbeat.begin_long_operation()
            obs, info = self.env.reset(seed=self.seed)
            heartbeat.beat()
            direction = info.get("fwbw_direction", "forward")
            archives[direction]._begin_trajectory()
            step = 0
            while total_steps is None or step < total_steps:
                loop_start = time.perf_counter()

                policy = self.policies[direction]
                sync_client = self.sync_clients[direction]
                reader = getattr(sync_client, "latest_policy_snapshot", None)
                if reader is not None:
                    snapshot = reader()
                    if snapshot is not None and snapshots.get(direction) != (snapshot.session_id, snapshot.version):
                        archives[direction]._validate_snapshot_experiment(snapshot)
                        load_policy_state_with_action_contract(policy, snapshot.state_dict)
                        archives[direction]._snapshot_metadata = {"policy_version": snapshot.version,
                            "policy_session_id": snapshot.session_id, "global_update": snapshot.global_update, "policy_phase": snapshot.phase}
                        snapshots[direction] = (snapshot.session_id, snapshot.version)
                else:
                    params = sync_client.latest_policy_params()
                    if params is not None and snapshots.get(direction) is not params:
                        load_policy_state_with_action_contract(policy, params)
                        snapshots[direction] = params

                with torch.no_grad():
                    policy_action = policy.predict(
                        self._obs_to_policy_device(obs), deterministic=self.deterministic
                    )
                env_action = policy_action.to(ActorLoop.env_device(obs))
                next_obs, reward, terminated, truncated, info = self.env.step(env_action)
                heartbeat.beat()

                next_direction = info.get("fwbw_direction", direction)
                direction_changed = next_direction != direction
                executed_action = info.get("intervene_action", policy_action)
                sync_client.push_transition(archives[direction]._archive_transition(
                    {
                        "obs": obs,
                        "next_obs": next_obs,
                        "action": executed_action,
                        "reward": reward,
                        "done": terminated,
                        **self._extra_transition_fields(info),
                    }, terminated, truncated, info, policy_action, segment_end=direction_changed)
                )

                ended = bool(terminated) or bool(truncated) or operator_quit_requested(info)
                if ended or direction_changed:
                    archives[direction]._end_trajectory(terminated=bool(terminated), truncated=bool(truncated),
                        end_reason="direction_switch" if direction_changed and not ended else None)
                    if operator_quit_requested(info):
                        logger.info("operator quit: stopping the actor after this episode")
                        break
                    direction = next_direction
                    if total_steps is None or step + 1 < total_steps:
                        if ended:
                            heartbeat.begin_long_operation()
                            obs, info = self.env.reset(seed=None)
                            heartbeat.beat()
                            direction = info.get("fwbw_direction", direction)
                        else:
                            obs = next_obs
                        archives[direction]._begin_trajectory()
                else:
                    obs = next_obs
                step += 1

                elapsed = time.perf_counter() - loop_start
                sleep_for = self.control_period - elapsed
                if sleep_for > 0:
                    time.sleep(sleep_for)
        finally:
            heartbeat.begin_long_operation()
            _cleanup([lambda: _close(self.env),
                      *[lambda archive=archive: archive._end_trajectory(interrupted=True) for archive in archives.values()],
                      *[client.stop for client in self.sync_clients.values()]])
