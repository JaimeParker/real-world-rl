"""Single-owner training loop with durable, idempotent network ingestion.

HTTP workers commit CPU payloads to the journal before acknowledging them.
The training thread alone drains that journal, mutates both replay buffers,
runs updates and publishes complete checkpoint generations. Journal records
remain available for replay after the checkpoint's committed cursor.
"""
from __future__ import annotations

import threading
import time
import tempfile

from real_world_rl.loops.durable import DurableStore, encode, decode
from real_world_rl.loops.ingestion import validate_admission
from real_world_rl.loops.checkpoint import save_bundle, restore_bundle, legacy_has_replay
from real_world_rl.loops.actor import _cleanup, _close
from numbers import Real
from pathlib import Path
from typing import Any, Mapping, Optional

import torch

from rl_garden.algorithms.off_policy import OffPolicyAlgorithm
from real_world_rl.loops.sync import LearnerSyncServer

_TRANSITION_TENSOR_KEYS = ("obs", "next_obs", "action", "reward", "done")
# Metric cadence in received transitions for an agent without ``log_freq``
# (``OffPolicyAlgorithm``'s own default).
_DEFAULT_LOG_FREQ = 1_000
_BUFFER_EXTRA_KEYS = frozenset(("intervened", "policy_action", "base_actions", "next_base_actions", "intervention_onset", "grasp_penalty", "discounts", "episode_end", "human_action_mask", "override_mask"))


class LearnerLoop:
    """Owns the algorithm and its replay buffers on the training thread."""

    def __init__(
        self,
        agent: OffPolicyAlgorithm,
        host: str,
        port: int,
        train_freq: int = 1,
        publish_freq: int = 100,
        idle_poll_interval: float = 0.1,
        journal_path: Optional[str] = None,
        resume_bundle_path: Optional[str] = None,
        resume_replay: bool = True,
        admission_capacity: int = 10_000,
        ingest_batch_size: int = 256,
        experiment_hash: Optional[str] = None,
        experiment_contract: Optional[dict] = None,
        effective_experiment: Optional[dict] = None,
    ) -> None:
        self.agent = agent
        self.train_freq = train_freq
        self.publish_freq = publish_freq
        self.idle_poll_interval = idle_poll_interval
        if admission_capacity < 1 or ingest_batch_size < 1:
            raise ValueError("admission capacity and ingest batch size must be positive")
        self.admission_capacity = admission_capacity
        self.ingest_batch_size = ingest_batch_size
        self.experiment_hash = experiment_hash or "unknown"
        self.experiment_contract = experiment_contract
        self.effective_experiment = effective_experiment
        from real_world_rl.metrics import EpisodeMetrics
        self.episode_metrics = EpisodeMetrics()
        self._episode_starts = {}
        self._admitted = 0
        self._admission_retries = 0
        self._admission_started = time.monotonic()
        self._admission_metrics_lock = threading.Lock()
        self._updates_at_start = agent.global_update
        self._received_at_start = 0
        self._last_ingest_timestamp = None
        env = getattr(agent, "env", None)
        self._observation_space = getattr(env, "single_observation_space", None)
        self._action_space = getattr(env, "single_action_space", None)

        self._lock = threading.Lock()
        self._received = 0
        self._received_by_actor: dict[str, int] = {}
        self._last_checkpoint_update = 0
        # Received-transition count at the last update-metrics log, and the metrics
        # step of the current training iteration (set by :meth:`_train_iteration`).
        self._last_log_step = 0
        self._metrics_step = 0
        directory = getattr(agent, "checkpoint_dir", None)
        self._bundle_dir = Path(directory) if directory else None
        self._resume_bundle_path = resume_bundle_path
        self._resume_replay = resume_replay
        self._legacy_replay_selected = resume_replay and legacy_has_replay(resume_bundle_path)
        self._restored = False
        self._journal = DurableStore(journal_path or (Path(directory) / "journal.sqlite3" if directory else Path(tempfile.mkdtemp(prefix="rl-learner-")) / "journal.sqlite3"))
        try:
            self._journal.bind_experiment(self.experiment_hash, self.experiment_contract)
        except BaseException:
            self._journal.close()
            raise
        self._journal_cursor = 0
        self._ingestion_failed = False
        self._training_failed = False
        self._journal_identity = self._journal.identity
        self._policy_ready = agent.global_update > 0
        try:
            self._server = LearnerSyncServer(host, port, on_transition=self._admit_transition, on_records=self._admit_records)
        except BaseException:
            self._journal.close()
            raise

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

    def _to_buffer_device(self, value):
        if isinstance(value, torch.Tensor):
            return value.to(self.agent.buffer_device)
        if isinstance(value, dict):
            return {key: self._to_buffer_device(item) for key, item in value.items()}
        if isinstance(value, tuple):
            return tuple(self._to_buffer_device(item) for item in value)
        if isinstance(value, list):
            return [self._to_buffer_device(item) for item in value]
        return value

    def _on_transition(self, transition: dict[str, Any]) -> None:
        from real_world_rl.training.demo_loading import training_extras
        from real_world_rl.trajectory import training_projection
        transition = {**transition, **training_projection(transition)}
        # actor_id is observability-only metadata, not a buffer field --
        # DictReplayBuffer.add()/TensorReplayBuffer.add() have fixed
        # signatures with no **kwargs, so it must never reach .add().
        actor_id = transition.get("actor_id")
        tensors = {
            k: self._to_buffer_device(v)
            for k, v in transition.items()
            if k in _TRANSITION_TENSOR_KEYS
        }
        extra = {
            k: self._to_buffer_device(v)
            for k, v in training_extras(transition, self.agent.replay_buffer.add).items()
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

    def _validate_effective_experiment(self, incoming) -> None:
        if incoming is None:
            return
        from real_world_rl.action_contract import compared_experiment, semantic_experiment

        def compared(contract):
            if isinstance(contract, Mapping) and "experiment" in contract:
                contract = {**contract, "experiment": compared_experiment(contract["experiment"])}
            return semantic_experiment(contract)

        requested = compared({key: value for key, value in incoming.items()
                              if key not in ("runtime_control", "provenance")})
        expected = compared(self.experiment_contract)
        if expected is not None and requested != expected:
            raise ValueError("actor effective experiment differs from learner requested contract")
        saved_runtime = (self.effective_experiment or {}).get("runtime_control", {})
        if any(saved_runtime.values()) and incoming.get("runtime_control", {}) != saved_runtime:
            raise ValueError("actor effective control/SDK contract differs from learner run")

    def _admit_transition(self, transition: dict[str, Any]) -> None:
        self._admit_records([transition])

    def _admit_records(self, records: list[dict[str, Any]]) -> None:
        # Validate every row before the atomic journal commit and HTTP ACK.
        messages = []
        for record in records:
            record = {"experiment_hash": "unknown", **record}
            validate_admission(record, observation_space=self._observation_space,
                               action_space=self._action_space, experiment_hash=self.experiment_hash)
            self._validate_effective_experiment(record.get("effective_experiment"))
            identity = record.get("record_id", record.get("transition_id"))
            messages.append((identity, encode(record)))
        result = self._journal.append_many(messages, after=self._journal_cursor, max_records=self.admission_capacity)
        with self._admission_metrics_lock:
            self._admitted += result.added_count
            self._admission_retries += len(messages) - result.added_count

    def _consume_record(self, record) -> None:
        kind = record.get("record_type", "transition")
        if kind == "transition":
            from real_world_rl.trajectory import training_projection
            self._on_transition({**record, **training_projection(record)})
            self._last_ingest_timestamp = record.get("timestamp_s")
        elif kind == "episode_start":
            if record.get("effective_experiment") is not None:
                self._validate_effective_experiment(record["effective_experiment"])
                self.effective_experiment = record["effective_experiment"]
            self._episode_starts[record["episode_id"]] = record
        elif kind == "episode_end":
            if record.get("transport_metrics"):
                self._log_scalars("transport/" + str(record.get("actor_id", "unknown")), record["transport_metrics"], self.received_transitions)
            from real_world_rl.metrics import EpisodeOutcome
            outcome = record.get("outcome")
            if outcome is not None:
                self.episode_metrics.add(EpisodeOutcome(**outcome))
            self._episode_starts.pop(record["episode_id"], None)
            self._log_scalars("episodes", self.episode_metrics.summary(), self.received_transitions)

    def _drain_journal(self, limit: Optional[int] = None) -> int:
        rows = self._journal.read(self._journal_cursor, limit or self.ingest_batch_size)
        for sequence, _, payload in rows:
            try:
                self._consume_record(decode(payload))
            except BaseException:
                # A multi-buffer write or snapshot can fail after mutating memory.
                # Recover from the previous committed bundle and this journal tail;
                # replaying or checkpointing these partial writes duplicates rows.
                # This includes KeyboardInterrupt: unlike an interrupted update, a
                # half-applied record cannot be told apart from a complete one.
                self._ingestion_failed = True
                raise
            self._journal_cursor = sequence
        return len(rows)

    def _loop_state(self) -> dict[str, Any]:
        return {
            "experiment_hash": self.experiment_hash,
            "experiment_contract": self.experiment_contract,
            "effective_experiment": self.effective_experiment,
            "seed_datasets": getattr(self, "_seed_datasets", []),
            "episode_outcomes": [outcome.to_dict() for outcome in self.episode_metrics.outcomes],
            "episode_starts": self._episode_starts,
            "journal_cursor": self._journal_cursor,
            "journal_identity": self._journal_identity,
            "received": self._received,
            "received_by_actor": self._received_by_actor,
            "policy_ready": self._policy_ready,
            "intervened_received": getattr(self, "_intervened_received", 0),
            "pretrain_considered": getattr(self, "_pretrain_considered", False),
            "behavior_refresh_count": getattr(self, "_behavior_refresh_count", 0),
        }

    def restore_checkpoint_bundle(self, path=None) -> bool:
        if self._restored:
            return False
        path = path or self._resume_bundle_path
        if path is None and self._bundle_dir is not None:
            candidate = self._bundle_dir / "bundle_manifest.json"
            if candidate.exists():
                path = candidate
        if path is None:
            self._restored = True
            return False
        if self._legacy_replay_selected and len(self._journal):
            raise ValueError("legacy replay checkpoint has no journal cursor; use its matching bundle manifest or an empty journal")
        state = restore_bundle(self.agent, path, load_replay_buffer=self._resume_replay, expected_journal_identity=self._journal_identity, expected_experiment_hash=self.experiment_hash, expected_seed_datasets=getattr(self, "_seed_datasets", []))
        self._restored = True
        if state is None:
            if self._legacy_replay_selected:
                self._received = len(self.agent.replay_buffer)
            self._policy_ready = self.agent.global_update > 0
            return False
        if state["journal_identity"] != self._journal_identity:
            raise ValueError("checkpoint journal identity differs; restore the matching durable journal")
        self._journal_cursor = state["journal_cursor"]
        self._received = state["received"]
        self.effective_experiment = state.get("effective_experiment")
        from real_world_rl.metrics import EpisodeOutcome
        self.episode_metrics.outcomes = [EpisodeOutcome(**outcome) for outcome in state.get("episode_outcomes", [])]
        self._episode_starts = state.get("episode_starts", {})
        self._seed_datasets = state.get("seed_datasets", [])
        self._received_by_actor = state["received_by_actor"]
        self._policy_ready = state["policy_ready"]
        if hasattr(self, "_intervened_received"):
            self._intervened_received = state["intervened_received"]
        if hasattr(self, "_pretrain_considered"):
            self._pretrain_considered = state["pretrain_considered"]
            self._behavior_refresh_count = state["behavior_refresh_count"]
        return True

    def _publish_policy(self) -> None:
        phase = "online" if self._policy_ready else ("pretrained" if getattr(self.agent, "pretrained", False) else "warmup")
        self._server.publish_params(self.agent.policy.state_dict(), global_update=self.agent.global_update, policy_ready=self._policy_ready, phase=phase, experiment_hash=self.experiment_hash,
                                    experiment_contract=self.experiment_contract, effective_experiment=self.effective_experiment)

    def _save_bundle(self) -> None:
        if self._bundle_dir is not None:
            save_bundle(self.agent, self._bundle_dir, self._loop_state())

    def _finish_ingestion(self) -> None:
        while self._drain_journal():
            pass
        snapshot = getattr(self, "_snapshot", None)
        if snapshot is not None and (self._pending_online or self._pending_demo):
            snapshot()

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
        self._policy_ready = True
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
        self._save_bundle()
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
        queue = self._journal.metrics(self._journal_cursor)
        elapsed = max(.001, time.monotonic() - self._admission_started)
        return {
            "journal_backlog": queue["depth"],
            "journal_backlog_bytes": queue["bytes"],
            "admitted_records": self._admitted,
            "receive_records_per_s": self._admitted / elapsed,
            "train_updates_per_s": (self.agent.global_update - self._updates_at_start) / elapsed,
            "ingest_transitions_per_s": (received - self._received_at_start) / elapsed,
            "duplicate_records": self._admission_retries,
            "ingestion_lag_s": 0 if self._last_ingest_timestamp is None else max(0, time.time() - self._last_ingest_timestamp),
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
        if ``None``, until :meth:`stop` is called from another thread).

        A ``KeyboardInterrupt`` or ``SystemExit`` raised by a training iteration
        ends the run like :meth:`stop` (final checkpoint and bundle written) and
        is then re-raised. Any other exception from an iteration skips the final
        checkpoint so the last complete bundle stays the recovery point; so does
        any exception, interrupts included, while a journal record is applied."""
        self._stop = False
        startup_complete = False
        try:
            self.restore_checkpoint_bundle()
            self._updates_at_start = self.agent.global_update
            self._received_at_start = self._received
            # Replay every committed tail record before exposing the initial policy.
            while self._drain_journal():
                pass
            self._publish_policy()
            self._server.start()
            startup_complete = True
            update = 0
            while not self._stop:
                self._drain_journal()
                if total_transitions is not None and self.received_transitions >= total_transitions:
                    break
                step = self.received_transitions
                self._log_actor_stats(step)
                if not self._ready_to_train():
                    time.sleep(self.idle_poll_interval)
                    continue
                was_ready = self._policy_ready
                try:
                    self._refresh_offline_data()
                    self._train_iteration(step)
                except (KeyboardInterrupt, SystemExit):
                    # An operator stop is a normal stop: the final checkpoint and
                    # bundle are written as after stop().
                    raise
                except BaseException:
                    # Optimizers/pretraining need not roll back a partial update.
                    # Keep the last complete generation as the recovery boundary.
                    self._training_failed = True
                    raise
                self._policy_ready = True
                update += 1
                if not was_ready or update % self.publish_freq == 0:
                    self._publish_policy()
        finally:
            def final_checkpoint():
                agent = self.agent
                if agent.checkpoint_dir is not None and agent.save_final_checkpoint:
                    agent.save(Path(agent.checkpoint_dir) / "final.pt", include_replay_buffer=agent.save_replay_buffer)
                    self._save_bundle()
            _cleanup([
                lambda: _close(getattr(self.agent, "env", None)),
                self._server.stop,
                lambda: self._finish_ingestion() if startup_complete and not self._ingestion_failed else None,
                lambda: final_checkpoint() if startup_complete and not self._ingestion_failed and not self._training_failed else None,
                lambda: self._log_actor_stats(self.received_transitions),
                self._journal.close,
            ])

    def stop(self) -> None:
        self._stop = True
