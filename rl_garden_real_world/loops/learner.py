"""LearnerLoop: drives an existing ``OffPolicyAlgorithm`` from transitions
received over the network instead of ``OffPolicyAlgorithm.learn()``'s
single-process rollout+update loop (which assumes it owns the env directly
and isn't a fit for a real robot stepped by a separate actor process).

Algorithm-agnostic by construction: only touches the public surface any
``OffPolicyAlgorithm`` subclass (``SAC``, ``RLPD``, ``TD3``, ...) already
exposes -- ``replay_buffer.add(...)``, ``train(gradient_steps)``,
``policy.state_dict()``, and the checkpoint-related attributes/methods
(``checkpoint_dir``, ``checkpoint_freq``, ``save_replay_buffer``,
``save_final_checkpoint``, ``global_update``, ``save(...)``) -- so no
base-class change beyond the read-only ``global_update`` property was needed
to build this.

Also periodically saves the full agent state (weights + optimizer), since
``OffPolicyAlgorithm.learn()``'s own periodic-checkpoint machinery
(``_maybe_save_periodic_checkpoint``/``_save_checkpoint``,
``rl_garden/algorithms/off_policy.py``) is only ever called from within
``learn()``'s own rollout loop, which real-world training never runs.
Without this, a learner crash loses all trained weights/optimizer state --
only the replay/demo buffers survive (see ``HilSerlLearnerLoop``'s pkl
snapshots). Cadence mirrors HIL-SERL's own ``learner()``
(``3rd_party/hil-serl/examples/train_rlpd.py:314-355``): keyed on gradient
update count, not received-transition count. Resuming weights on startup
reuses the existing ``--load_checkpoint`` flag
(``build_rlpd``/``build_rlpd_hybrid`` already call ``agent.load(...)`` before
handing the agent to this loop) -- no new resume logic needed here.

Metrics: when the agent has a ``logger``, every metric is logged at step = received
transitions (the analogue of ``learn()``'s global step). On each training iteration
whose step crosses an ``agent.log_freq`` boundary, the train step runs with
``compute_info=True``, its info goes to ``agent._log_update_metrics`` and the
``learner/*`` scalars of :meth:`LearnerLoop._learner_scalars` plus
``learner/train_time_s`` are logged. Every loop iteration drains the actors'
episode stats from the sync server and logs them as ``actor/<key>`` (or
``actor/<actor_id>/<key>`` when the actor stamps an id). Without a logger nothing is
logged and the stats queue is left alone.
"""
from __future__ import annotations

import threading
import time
from numbers import Real
from pathlib import Path
from typing import Any, Mapping, Optional

import torch

from rl_garden.algorithms.off_policy import OffPolicyAlgorithm
from rl_garden_real_world.loops.sync import LearnerSyncServer

_TRANSITION_TENSOR_KEYS = ("obs", "next_obs", "action", "reward", "done")
# Metric cadence in received transitions for an agent without ``log_freq``
# (``OffPolicyAlgorithm``'s own default).
_DEFAULT_LOG_FREQ = 1_000


class LearnerLoop:
    """Owns a full algorithm instance and trains it from actor-supplied data.

    ``agent.replay_buffer.add()`` (invoked from the HTTP server's request
    thread on every received transition) and ``agent.train()`` (invoked from
    :meth:`run`'s loop) both touch the replay buffer, so both are serialized
    under one lock -- mirrors the thread-safety SERL gets from its own
    ``MemoryEfficientReplayBufferDataStore``.
    """

    def __init__(
        self,
        agent: OffPolicyAlgorithm,
        host: str,
        port: int,
        train_freq: int = 1,
        publish_freq: int = 100,
        idle_poll_interval: float = 0.1,
    ) -> None:
        self.agent = agent
        self.train_freq = train_freq
        self.publish_freq = publish_freq
        self.idle_poll_interval = idle_poll_interval

        self._lock = threading.Lock()
        self._received = 0
        self._received_by_actor: dict[str, int] = {}
        self._last_checkpoint_update = 0
        # Received-transition count at the last update-metrics log, and the metrics
        # step of the current training iteration (set by :meth:`_train_iteration`).
        self._last_log_step = 0
        self._metrics_step = 0
        self._server = LearnerSyncServer(host, port, on_transition=self._on_transition)

    @property
    def received_transitions(self) -> int:
        with self._lock:
            return self._received

    @property
    def received_by_actor(self) -> dict[str, int]:
        """Per-``actor_id`` received-transition counts, for observability
        when multiple actors push to this learner. Transitions pushed
        without an ``actor_id`` (``ActorSyncClient(actor_id=None)``, the
        default) are tallied under ``"unknown"``."""
        with self._lock:
            return dict(self._received_by_actor)

    def _on_transition(self, transition: dict[str, Any]) -> None:
        # actor_id is observability-only metadata, not a buffer field --
        # DictReplayBuffer.add()/TensorReplayBuffer.add() have fixed
        # signatures with no **kwargs, so it must never reach .add().
        actor_id = transition.get("actor_id")
        device = self.agent.buffer_device
        tensors = {
            k: v.to(device) if isinstance(v, torch.Tensor) else v
            for k, v in transition.items()
            if k in _TRANSITION_TENSOR_KEYS
        }
        extra = {
            k: v.to(device) if isinstance(v, torch.Tensor) else v
            for k, v in transition.items()
            if k not in _TRANSITION_TENSOR_KEYS and k != "actor_id"
        }
        with self._lock:
            self.agent.replay_buffer.add(
                tensors["obs"],
                tensors["next_obs"],
                tensors["action"],
                tensors["reward"],
                tensors["done"],
                **extra,
            )
            self._received += 1
            key = actor_id if actor_id is not None else "unknown"
            self._received_by_actor[key] = self._received_by_actor.get(key, 0) + 1

    def _refresh_offline_data(self) -> None:
        """Hook for methods that need to periodically re-read a growing
        on-disk dataset (e.g. HIL-SERL's demo/correction data). No-op by
        default -- SERL doesn't need it."""

    def _ready_to_train(self) -> bool:
        """Whether :meth:`run` may start training: ``learning_starts`` received
        transitions by default. Subclasses add their own conditions (SiLRI also
        waits for a minimum demo buffer)."""
        return self.received_transitions >= self.agent.learning_starts

    def _train_step(self, compute_info: bool = False) -> dict[str, float]:
        gradient_steps = max(1, int(self.train_freq * self.agent.utd))
        with self._lock:
            info = self.agent.train(gradient_steps, compute_info=compute_info)
        self._maybe_save_periodic_checkpoint()
        return info

    def _maybe_save_periodic_checkpoint(self) -> None:
        agent = self.agent
        if agent.checkpoint_dir is None or agent.checkpoint_freq <= 0:
            return
        update = agent.global_update
        if update // agent.checkpoint_freq <= self._last_checkpoint_update // agent.checkpoint_freq:
            return
        agent.save(
            Path(agent.checkpoint_dir) / f"checkpoint_{update}.pt",
            include_replay_buffer=agent.save_replay_buffer,
        )
        self._last_checkpoint_update = update

    # --- metrics ---

    @property
    def log_freq(self) -> int:
        """Update-metrics cadence in received transitions: ``agent.log_freq``
        (``0`` disables), else ``_DEFAULT_LOG_FREQ``."""
        log_freq = getattr(self.agent, "log_freq", None)
        return _DEFAULT_LOG_FREQ if log_freq is None else int(log_freq)

    def _metrics_logger(self) -> Any:
        return getattr(self.agent, "logger", None)

    def _log_scalars(self, prefix: str, values: Mapping[str, Any], step: int) -> None:
        """Logs every real-number value as ``<prefix>/<key>`` at ``step``; no-op
        without a logger."""
        logger = self._metrics_logger()
        if logger is None:
            return
        for key, value in values.items():
            if isinstance(value, Real):
                logger.add_scalar(f"{prefix}/{key}", float(value), step)

    def _learner_scalars(self) -> dict[str, float]:
        """The ``learner/*`` scalars logged on update-metrics steps. Subclasses
        extend the dict with their own counters."""
        with self._lock:
            received = self._received
            buffer_size = len(self.agent.replay_buffer)
        return {
            "received": received,
            "update": self.agent.global_update,
            "buffer_size": buffer_size,
        }

    def _log_actor_stats(self, step: int) -> None:
        """Logs the episode stats the actors posted since the last call."""
        if self._metrics_logger() is None:
            return
        for stats in self._server.drain_stats():
            actor_id = stats.get("actor_id")
            prefix = "actor" if actor_id is None else f"actor/{actor_id}"
            self._log_scalars(
                prefix, {key: value for key, value in stats.items() if key != "actor_id"}, step
            )

    def _train_iteration(self, step: int) -> None:
        """One training iteration of :meth:`run` at metrics step ``step`` (received
        transitions). Asks the train step for its info only when ``step`` crosses a
        :attr:`log_freq` boundary and the agent has a logger, then logs it through
        ``agent._log_update_metrics`` together with the ``learner/*`` scalars."""
        self._metrics_step = step
        log_freq = self.log_freq
        should_log = (
            self._metrics_logger() is not None
            and log_freq > 0
            and step // log_freq > self._last_log_step // log_freq
        )
        started = time.perf_counter()
        info = self._train_step(compute_info=should_log)
        if not should_log:
            return
        train_time_s = time.perf_counter() - started
        self.agent._log_update_metrics(info, step)
        self._log_scalars(
            "learner", {**self._learner_scalars(), "train_time_s": train_time_s}, step
        )
        self._last_log_step = step

    def run(self, total_transitions: Optional[int] = None) -> None:
        """Runs until ``total_transitions`` have been received (or forever,
        if ``None``, until :meth:`stop` is called from another thread)."""
        self._server.start()
        self._stop = False
        try:
            update = 0
            while not self._stop:
                if total_transitions is not None and self.received_transitions >= total_transitions:
                    break
                step = self.received_transitions
                self._log_actor_stats(step)
                if not self._ready_to_train():
                    time.sleep(self.idle_poll_interval)
                    continue
                self._refresh_offline_data()
                self._train_iteration(step)
                update += 1
                if update % self.publish_freq == 0:
                    self._server.publish_params(self.agent.policy.state_dict())
        finally:
            self._server.stop()
            agent = self.agent
            if agent.checkpoint_dir is not None and agent.save_final_checkpoint:
                agent.save(
                    Path(agent.checkpoint_dir) / "final.pt",
                    include_replay_buffer=agent.save_replay_buffer,
                )
            self._log_actor_stats(self.received_transitions)

    def stop(self) -> None:
        self._stop = True
