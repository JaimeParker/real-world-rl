"""HIL-SERL's actor, learner and sync classes.

``HilSerlActorLoop`` tags pushed transitions with the intervention flag and posts
per-episode statistics to the learner, ``HilSerlLearnerLoop`` routes transitions into the
online and demo buffers and snapshots both for crash recovery, and the sync classes are
the base ones under HIL-SERL names.
"""
from __future__ import annotations

import glob
import os
import pickle
import time
from dataclasses import dataclass, field
from typing import Any, Mapping, Optional, Sequence

import torch

from rl_garden.algorithms.off_policy import OffPolicyAlgorithm
from rl_garden_real_world.action_contract import (
    read_contract_sidecar,
    validate_transitions_against_contract,
    write_contract_sidecar,
)
from rl_garden_real_world.loops.actor import ActorLoop
from rl_garden_real_world.loops.learner import _TRANSITION_TENSOR_KEYS, LearnerLoop
from rl_garden_real_world.loops.sync import ActorSyncClient, LearnerSyncServer
from rl_garden_real_world.training.demo_loading import (
    as_dict_observation,
    load_demo_pkls,
)

__all__ = [
    "HilSerlActorLoop",
    "HilSerlLearnerLoop",
    "HilSerlActorSyncClient",
    "HilSerlLearnerSyncServer",
]


def _optional_float(value: Any) -> Optional[float]:
    """``float(value)`` for a number or a one-element array/tensor, else ``None``."""
    if value is None:
        return None
    try:
        return float(value)
    except (TypeError, ValueError, RuntimeError):
        return None


def _add_mean_and_max(stats: dict[str, float], name: str, values: list[float]) -> None:
    if values:
        stats[f"{name}_mean"] = sum(values) / len(values)
        stats[f"{name}_max"] = max(values)


@dataclass
class _EpisodeStatistics:
    """Accumulates one actor episode for :meth:`HilSerlActorLoop._on_episode_end`."""

    # Cumulative device safety-clip count the episode's increase is measured from:
    # the last reading of the previous episode, else this episode's first reading.
    safety_clips_baseline: Optional[int] = None
    reward_sum: float = 0.0
    length: int = 0
    intervened_steps: int = 0
    intervention_count: int = 0
    previously_intervened: bool = False
    terminated: bool = False
    human_episode_end: bool = False
    safety_clips: Optional[int] = None
    rtt_ms: list[float] = field(default_factory=list)
    step_time_ms: list[float] = field(default_factory=list)
    param_refresh_ms: list[float] = field(default_factory=list)

    def record_step(self, reward: Any, terminated: Any, intervened: bool, info: dict) -> None:
        self.reward_sum += float(torch.as_tensor(reward, dtype=torch.float64).sum())
        self.length += 1
        if intervened:
            self.intervened_steps += 1
            if not self.previously_intervened:
                self.intervention_count += 1
        self.previously_intervened = intervened
        self.terminated = bool(terminated)
        self.human_episode_end = bool(info.get("human_episode_end", False))
        rtt_ms = _optional_float(info.get("rtt_ms"))
        if rtt_ms is not None:
            self.rtt_ms.append(rtt_ms)
        safety_clips = _optional_float(info.get("safety_clips"))
        if safety_clips is not None:
            self.safety_clips = int(safety_clips)
            if self.safety_clips_baseline is None:
                self.safety_clips_baseline = self.safety_clips

    def summary(self) -> dict[str, float]:
        """The stats message posted to the learner (all values JSON numbers)."""
        stats: dict[str, float] = {
            "return": self.reward_sum,
            "length": self.length,
            "intervened_steps": self.intervened_steps,
            "intervention_count": self.intervention_count,
            "intervention_rate": self.intervened_steps / self.length if self.length else 0.0,
            "success": float(self.terminated),
            "human_episode_end": float(self.human_episode_end),
        }
        if self.safety_clips is not None:
            stats["safety_clips"] = self.safety_clips - self.safety_clips_baseline
            stats["safety_clips_total"] = self.safety_clips
        _add_mean_and_max(stats, "rtt_ms", self.rtt_ms)
        _add_mean_and_max(stats, "step_time_ms", self.step_time_ms)
        _add_mean_and_max(stats, "param_refresh_ms", self.param_refresh_ms)
        return stats


class HilSerlActorLoop(ActorLoop):
    """HIL-SERL's ActorLoop. HITL intervention and the hybrid continuous+discrete
    action are handled by the env wrapper stack and the policy respectively, so
    the transition overrides are tagging pushed transitions with whether a human
    was intervening -- ``HilSerlLearnerLoop`` uses this to route the transition
    into the online replay buffer or the growing demo buffer -- and forwarding
    ``grasp_penalty`` when ``GraspPenaltyWrapper`` is in the env stack (it flows
    straight through to ``DictReplayBuffer.add(grasp_penalty=...)`` once
    received, no further routing needed).

    Also accumulates per-episode statistics and posts them with
    ``sync_client.post_stats`` at every episode end (skipped for a client without
    ``post_stats``); the learner logs them as ``actor/<key>``. Keys: ``episode``,
    ``return``, ``length``, ``intervened_steps``, ``intervention_count`` (takeover
    onsets), ``intervention_rate``, ``success`` (the episode terminated),
    ``human_episode_end`` (the operator ended it), ``safety_clips`` (increase of the
    device clip counter) and ``safety_clips_total`` when the env reports it,
    ``rtt_ms_mean/max`` when the env reports ``rtt_ms``, ``step_time_ms_mean/max`` (from
    the policy refresh to ``env.step`` returning), ``param_refresh_ms_mean/max`` (the
    per-step policy ``load_state_dict``) and the client's cumulative
    ``dropped_transitions``. Subclasses that override ``_build_transition`` or
    ``_on_episode_end`` call ``super()`` so the statistics stay complete.
    """

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        self._episode_statistics = _EpisodeStatistics()
        self._episodes_ended = 0
        self._step_started_at: Optional[float] = None

    def _extra_transition_fields(self, info: dict) -> dict:
        fields = {"intervened": "intervene_action" in info}
        if "grasp_penalty" in info:
            fields["grasp_penalty"] = info["grasp_penalty"]
        return fields

    def _maybe_refresh_policy(self) -> None:
        # First call of every control step: starts the step timer.
        self._step_started_at = time.perf_counter()
        super()._maybe_refresh_policy()
        self._episode_statistics.param_refresh_ms.append(
            1000.0 * (time.perf_counter() - self._step_started_at)
        )

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
        statistics = self._episode_statistics
        if self._step_started_at is not None:
            statistics.step_time_ms.append(1000.0 * (time.perf_counter() - self._step_started_at))
            self._step_started_at = None
        statistics.record_step(reward, terminated, transition["intervened"], info)
        return transition

    def _on_episode_end(self, info: dict) -> None:
        self._post_episode_statistics()
        super()._on_episode_end(info)

    def _post_episode_statistics(self) -> None:
        finished = self._episode_statistics
        last_safety_clips = (
            finished.safety_clips if finished.safety_clips is not None else finished.safety_clips_baseline
        )
        self._episode_statistics = _EpisodeStatistics(safety_clips_baseline=last_safety_clips)
        self._episodes_ended += 1
        post_stats = getattr(self.sync_client, "post_stats", None)
        if post_stats is None:
            return
        stats = {"episode": self._episodes_ended, **finished.summary()}
        dropped = getattr(self.sync_client, "dropped_transitions", None)
        if dropped is not None:
            stats["dropped_transitions"] = int(dropped)
        post_stats(stats)


class HilSerlLearnerLoop(LearnerLoop):
    """HIL-SERL's LearnerLoop -- this migration targets HIL-SERL's ``train_rlpd.py``
    capability set (online RLPD + demo mixing + HITL + reward classifier), which
    uses RLPD's existing static ``offline_dataset`` loading, not a
    continuously growing on-disk dataset. So ``_refresh_offline_data()`` stays
    the base class's no-op default here too; HG-DAgger's growing-dataset problem
    is out of scope for this round (see ``robot_infra/docs/robot-infra-roadmap.md`` in this repository).

    Overrides ``_on_transition`` to route transitions the way HIL-SERL's own
    actor loop does (``3rd_party/hil-serl/examples/train_rlpd.py:178-193``):
    *every* transition goes into the online replay buffer, and human-intervened
    transitions are *additionally* copied into the growing demo buffer
    (``DemoInterventionMixin``, see ``rl_garden/buffers/demo_intervention.py``)
    -- the demo buffer is a duplicated subset of the online buffer, not a
    disjoint partition of it. Also periodically pickle-snapshots both buffers to
    disk for crash recovery -- ``LearnerLoop.run()`` never triggers the existing
    ``save_replay_buffer``/``include_replay_buffer`` checkpoint mechanism
    (``rl_garden/algorithms/off_policy.py``), so this pkl snapshot is the only
    buffer persistence path for real-world hil_serl training. Mirrors HIL-SERL's
    own mechanism (``train_rlpd.py:140-233``): transitions accumulated since the
    last snapshot are pickled to ``<checkpoint_dir>/buffer/transitions_{n}.pkl``
    (every transition) and ``<checkpoint_dir>/demo_buffer/transitions_{n}.pkl``
    (intervened only) every ``buffer_period`` received transitions, and both
    directories are glob-reloaded on startup.

    Also loads ``demo_dataset_paths`` (pre-collected demo transitions, HIL-SERL's
    ``--demo_path`` equivalent -- ``train_rlpd.py:458-466``) into the demo buffer
    on every start, unconditionally, before the crash-recovery snapshot reload --
    matching HIL-SERL's own always-reload-``demo_path`` behavior exactly. Uses
    the same pkl transition-dict format as the buffer snapshots above (not
    HIL-SERL's own numpy dict shape), so a previous run's
    ``demo_buffer/*.pkl`` snapshot files can be reused directly as seed data.

    ``_buffer_kwargs(transition) -> (core, extra)`` is the hook that decides which fields of a
    received transition reach ``replay_buffer.add`` / ``add_demo_transition`` (and so the
    snapshots). The default drops ``intervened`` (routing only) and ``actor_id`` and forwards
    the rest, which is what ``DictReplayBuffer`` accepts (e.g. ``grasp_penalty``).
    ``ResidualLearnerLoop`` and ``SilriLearnerLoop`` override it to keep ``intervened``,
    ``policy_action`` and their own columns for ``InterventionReplayBuffer``.

    Logged learner scalars add ``learner/demo_buffer_size`` (demo buffer rows) and
    ``learner/intervened_received`` (intervened transitions received over the network
    by this process; snapshot reloads are not counted) to the base ones.
    """

    def __init__(
        self,
        agent: OffPolicyAlgorithm,
        host: str,
        port: int,
        checkpoint_dir: str,
        buffer_period: int = 1000,
        demo_dataset_paths: Sequence[str] = (),
        train_freq: int = 1,
        publish_freq: int = 100,
        idle_poll_interval: float = 0.1,
        action_contract: Optional[Mapping[str, Any]] = None,
    ) -> None:
        super().__init__(agent, host, port, train_freq, publish_freq, idle_poll_interval)
        # ``action_contract.DEMO_CONTRACT_KEYS`` of the env; None (contract-less backends)
        # skips snapshot/demo action checks and writes no sidecars.
        self._action_contract = action_contract
        self._checkpoint_dir = checkpoint_dir
        self._buffer_period = buffer_period
        self._pending_online: list[dict[str, Any]] = []
        self._pending_demo: list[dict[str, Any]] = []
        self._step_since_snapshot = 0
        self._intervened_received = 0
        self._load_demo_datasets(demo_dataset_paths)
        self._reload_snapshots()

    def _learner_scalars(self) -> dict[str, float]:
        scalars = super()._learner_scalars()
        demo_buffer = self.agent.offline_replay_buffer
        with self._lock:
            scalars["demo_buffer_size"] = 0 if demo_buffer is None else len(demo_buffer)
            scalars["intervened_received"] = self._intervened_received
        return scalars

    def _load_demo_datasets(self, paths: Sequence[str]) -> None:
        # Not counted toward `_received`/`received_transitions` -- that
        # gates LearnerLoop.run()'s learning_starts wait, and HIL-SERL's own
        # analogous gate (`len(replay_buffer) < training_starts`) only ever
        # counts the online buffer, never `--demo_path` preload.
        load_demo_pkls(
            paths,
            self.agent.add_demo_transition,
            self.agent.buffer_device,
            expected_contract=self._action_contract,
        )

    def _reload_snapshots(self) -> None:
        device = self.agent.buffer_device
        # "buffer" holds every transition ever received (mirrors HIL-SERL's
        # data_store); "demo_buffer" is the intervened subset, duplicated
        # into it, not a disjoint remainder -- so only count a transition as
        # received once, off the "buffer" directory, or reloading both would
        # double-count every intervened transition.
        sources = (
            (os.path.join(self._checkpoint_dir, "buffer"), self.agent.replay_buffer.add, True),
            (os.path.join(self._checkpoint_dir, "demo_buffer"), self.agent.add_demo_transition, False),
        )
        for directory, adder, count_received in sources:
            for pkl_path in sorted(glob.glob(os.path.join(directory, "*.pkl"))):
                with open(pkl_path, "rb") as f:
                    transitions = pickle.load(f)
                validate_transitions_against_contract(
                    transitions,
                    self._action_contract,
                    source=pkl_path,
                    saved=read_contract_sidecar(pkl_path),
                )
                for transition in transitions:
                    self._add_transition(adder, transition, device)
                    if count_received:
                        self._received += 1

    @staticmethod
    def _add_transition(adder, transition: dict[str, Any], device) -> None:
        tensors = {
            k: (v.to(device) if isinstance(v, torch.Tensor) else v)
            for k, v in transition.items()
            if k in _TRANSITION_TENSOR_KEYS
        }
        extra = {
            k: (v.to(device) if isinstance(v, torch.Tensor) else v)
            for k, v in transition.items()
            if k not in _TRANSITION_TENSOR_KEYS
        }
        adder(
            as_dict_observation(tensors["obs"]),
            as_dict_observation(tensors["next_obs"]),
            tensors["action"],
            tensors["reward"],
            tensors["done"],
            **extra,
        )

    def _buffer_kwargs(self, transition: dict[str, Any]) -> tuple[dict[str, Any], dict[str, Any]]:
        """Splits a received transition into ``(core, extra)``: ``core`` holds the
        five tensor keys (``obs``, ``next_obs``, ``action``, ``reward``, ``done``)
        and ``extra`` the keywords forwarded to ``replay_buffer.add`` and
        ``add_demo_transition``. What this returns is also what snapshots pickle
        and what a restart re-adds, so extras survive a reload.

        The default drops ``intervened`` (routing only) and ``actor_id``
        (observability only) and forwards everything else, e.g. ``grasp_penalty``.
        An override that keeps ``intervened`` needs a buffer ``add`` that takes it.
        The routing decision reads ``transition["intervened"]`` before this call.
        """
        core = {key: transition[key] for key in _TRANSITION_TENSOR_KEYS}
        extra = {
            key: value
            for key, value in transition.items()
            if key not in _TRANSITION_TENSOR_KEYS and key not in ("intervened", "actor_id")
        }
        return core, extra

    def _on_transition(self, transition: dict[str, Any]) -> None:
        intervened = bool(transition.get("intervened", False))
        actor_id = transition.get("actor_id")
        core, extra = self._buffer_kwargs(transition)
        stored = {**core, **extra}
        device = self.agent.buffer_device
        with self._lock:
            self._add_transition(self.agent.replay_buffer.add, stored, device)
            self._pending_online.append(stored)
            if intervened:
                self._add_transition(self.agent.add_demo_transition, stored, device)
                self._pending_demo.append(stored)
                self._intervened_received += 1
            self._received += 1
            key = actor_id if actor_id is not None else "unknown"
            self._received_by_actor[key] = self._received_by_actor.get(key, 0) + 1
            self._step_since_snapshot += 1
            if self._step_since_snapshot >= self._buffer_period:
                self._snapshot()

    def _snapshot(self) -> None:
        buffer_dir = os.path.join(self._checkpoint_dir, "buffer")
        demo_dir = os.path.join(self._checkpoint_dir, "demo_buffer")
        os.makedirs(buffer_dir, exist_ok=True)
        os.makedirs(demo_dir, exist_ok=True)
        step = self._received
        for directory, pending in ((buffer_dir, self._pending_online), (demo_dir, self._pending_demo)):
            pkl_path = os.path.join(directory, f"transitions_{step}.pkl")
            with open(pkl_path, "wb") as f:
                pickle.dump(pending, f)
            if self._action_contract is not None:
                write_contract_sidecar(pkl_path, self._action_contract)
        self._pending_online = []
        self._pending_demo = []
        self._step_since_snapshot = 0


class HilSerlActorSyncClient(ActorSyncClient):
    """HIL-SERL's actor sync client -- currently identical to the base; kept as its own
    subclass for naming symmetry with ``rl_garden_real_world.loops.serl``.
    """


class HilSerlLearnerSyncServer(LearnerSyncServer):
    """HIL-SERL's learner sync server -- currently identical to the base; kept as its own
    subclass for naming symmetry with ``rl_garden_real_world.loops.serl``.
    """
