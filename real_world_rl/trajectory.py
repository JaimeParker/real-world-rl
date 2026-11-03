"""Versioned plain-dict archive records; replay is a training projection."""
from __future__ import annotations
from typing import Any, Mapping
import torch

# Archive provenance is not a replay-buffer add() argument. Keep this projection
# explicit so adding trajectory metadata cannot accidentally change training.
TRAINING_EXTRA_KEYS = frozenset({
    "intervened", "human_action_mask", "override_mask", "policy_action", "base_actions", "next_base_actions",
    "intervention_onset", "grasp_penalty", "episode_end", "step_success", "discounts",
})


def begin_episode(episode_id, *, run_id="unknown", actor_id="unknown", experiment_hash="unknown", action_contract=None, **metadata):
    return dict(metadata, schema_version=1, record_type="episode_start", record_id=f"{episode_id}/start", episode_id=episode_id, run_id=run_id, actor_id=actor_id, experiment_hash=experiment_hash, action_contract=action_contract)


def transition_record(transition, *, episode_id, step_index, record_id=None, experiment_hash="unknown", **metadata):
    identity = record_id or transition.get("transition_id") or f"{episode_id}/step/{step_index}"
    record = dict(transition, **metadata)
    record.update(schema_version=1, record_type="transition", record_id=identity, transition_id=identity, episode_id=episode_id, step_index=step_index, experiment_hash=experiment_hash)
    validate_record(record)
    return record


def end_episode(episode_id, *, step_count, outcome=None, interrupted=False, **metadata):
    if hasattr(outcome, "to_dict"):
        outcome = outcome.to_dict()
    return dict(metadata, schema_version=1, record_type="episode_end", record_id=f"{episode_id}/end", episode_id=episode_id, step_count=step_count, outcome=outcome, interrupted=interrupted)


def validate_record(record):
    if record.get("schema_version") != 1 or record.get("record_type") not in ("episode_start", "transition", "episode_end"):
        raise ValueError("Unsupported trajectory record schema")
    for key in ("record_id", "episode_id"):
        if not isinstance(record.get(key), str) or not record[key]:
            raise ValueError(f"trajectory requires {key}")
    for key in ("step_index", "step_count", "sequence"):
        if key in record and (not isinstance(record[key], int) or isinstance(record[key], bool) or record[key] < 0):
            raise ValueError(f"{key} must be a nonnegative integer")
    if "timestamp_s" in record:
        import math
        if not isinstance(record["timestamp_s"], (int, float)) or not math.isfinite(record["timestamp_s"]):
            raise ValueError("timestamp_s must be finite")
    if record["record_type"] == "episode_end":
        if not isinstance(record.get("interrupted", False), bool):
            raise ValueError("interrupted must be bool")
        outcome = record.get("outcome")
        if outcome is not None:
            if not isinstance(outcome, dict):
                raise ValueError("outcome must be a mapping")
            for key in ("task_success", "had_intervention", "completed", "interrupted", "discarded"):
                if key in outcome and outcome[key] is not None and not isinstance(outcome[key], bool):
                    raise ValueError(f"outcome {key} must be bool or unknown")
            for key in ("steps", "intervention_steps"):
                if key in outcome and (not isinstance(outcome[key], int) or isinstance(outcome[key], bool) or outcome[key] < 0):
                    raise ValueError(f"outcome {key} must be a nonnegative integer")
    if record["record_type"] == "transition":
        for key in ("obs", "next_obs", "action", "reward", "done", "step_index"):
            if key not in record:
                raise ValueError(f"transition missing {key}")
        if record.get("transition_id", record["record_id"]) != record["record_id"]:
            raise ValueError("transition_id must equal record_id")
        for name in ("terminated", "truncated"):
            if name in record:
                value = record[name]
                if not isinstance(value, bool) and not (isinstance(value, torch.Tensor) and value.dtype == torch.bool and value.numel() == 1):
                    raise ValueError(f"{name} must be bool or one bool tensor")
        reward = torch.as_tensor(record["reward"])
        if reward.numel() != 1 or not torch.isfinite(reward).all():
            raise ValueError("reward must be a finite scalar")
        components = record.get("reward_components", {})
        if not isinstance(components, dict):
            raise ValueError("reward_components must be a mapping")
        for name, value in components.items():
            number = torch.as_tensor(value)
            if number.numel() != 1 or not torch.isfinite(number).all():
                raise ValueError(f"reward component {name} must be finite scalar")
        if "outcome_source" in record and not isinstance(record["outcome_source"], str):
            raise ValueError("outcome_source must be a string")
        action = torch.as_tensor(record["action"])
        if action.ndim not in (1, 2) or not torch.isfinite(action).all():
            raise ValueError("transition action must be finite with shape (A,) or (1,A)")
        for key in ("human_action_mask", "override_mask"):
            if key in record:
                mask = torch.as_tensor(record[key])
                if mask.numel() != action.numel() or mask.dtype != torch.bool:
                    raise ValueError(f"{key} must be bool and match action dimensions")


def training_projection(record):
    result = {key: value for key, value in record.items() if key in TRAINING_EXTRA_KEYS or key in ("obs", "next_obs", "action", "reward", "done")}
    if "terminated" in record or "truncated" in record or "training_segment_end" in record or "segment_end" in record:
        result["episode_end"] = (torch.as_tensor(record.get("terminated", False), dtype=torch.bool)
                                 | torch.as_tensor(record.get("truncated", False), dtype=torch.bool)
                                 | torch.as_tensor(record.get("training_segment_end", False), dtype=torch.bool)
                                 | torch.as_tensor(record.get("segment_end", False), dtype=torch.bool))
    if "terminated" in record:
        done = record.get("done")
        if isinstance(done, torch.Tensor):
            result["done"] = torch.as_tensor(record["terminated"], device=done.device,
                                             dtype=done.dtype).expand_as(done)
        else:
            result["done"] = record["terminated"]
    if record.get("syncing", False):
        if "human_action_mask" in result:
            result["human_action_mask"] = torch.zeros_like(torch.as_tensor(result["human_action_mask"]), dtype=torch.bool)
        result["intervened"] = False
        if "intervention_onset" in result:
            result["intervention_onset"] = False
    return result

class DurableArchive:
    """Append-only SQLite archive with bounded-memory paginated iteration."""
    def __init__(self, path, *, run_id=None, actor_id="recorder", experiment_hash=None):
        import uuid
        from real_world_rl.loops.durable import DurableStore
        self.store = DurableStore(path)
        if experiment_hash:
            try:
                self.store.bind_experiment(experiment_hash)
            except BaseException:
                self.store.close()
                raise
        self.session_id = uuid.uuid4().hex
        self._sequence = 0
        self.run_id = run_id or self.store.identity
        self.actor_id = actor_id
        self.experiment_hash = experiment_hash or "uncontracted"
        self._last = None
        first = True
        for record in self:
            if first and record.get("run_id") not in (None, "unknown"):
                if run_id is not None and run_id != record["run_id"]:
                    self.store.close()
                    raise ValueError("recorder archive run_id differs from requested identity")
                self.run_id = record["run_id"]
            if first and experiment_hash is None and record.get("experiment_hash"):
                self.experiment_hash = record["experiment_hash"]
            first = False
            self._sequence = max(self._sequence, record.get("sequence", -1) + 1)
            self._last = record
        if self._last is not None and self._last["record_type"] != "episode_end":
            episode_id = self._last["episode_id"]
            count = sum(row["record_type"] == "transition" and row["episode_id"] == episode_id for row in self)
            self.append(end_episode(episode_id, step_count=count, interrupted=True, end_reason="process_restart"))

    def __iter__(self):
        from real_world_rl.loops.durable import decode
        cursor = 0
        while True:
            batch = self.store.read(cursor, limit=16)
            if not batch:
                return
            for _, _, payload in batch:
                yield decode(payload)
            cursor = batch[-1][0]

    def __len__(self):
        return len(self.store)

    def __getitem__(self, index):
        if index != -1 or self._last is None:
            raise IndexError("archive supports only last-record lookup; iterate for records")
        return self._last

    def append(self, record):
        import time
        from real_world_rl.loops.durable import encode
        record = dict(record)
        for key, value in (("run_id", self.run_id), ("actor_id", self.actor_id),
                           ("session_id", self.session_id), ("experiment_hash", self.experiment_hash)):
            if record.get(key) in (None, "unknown"):
                record[key] = value
        record.setdefault("sequence", self._sequence)
        self._sequence += 1
        record.setdefault("timestamp_s", time.time())
        record.setdefault("policy_version", 0)
        record.setdefault("policy_phase", "teleop")
        record.setdefault("stage", "teleop")
        validate_record(record)
        self.store.append(record["record_id"], encode(record))
        self._last = record

    def close(self):
        self.store.close()
