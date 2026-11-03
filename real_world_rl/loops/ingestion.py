"""CPU-only validation before durable admission; no replay mutations."""
from __future__ import annotations

from typing import Any
import json
import torch


class AdmissionBusy(RuntimeError):
    pass


def _tensor(value: Any, name: str) -> torch.Tensor:
    if not isinstance(value, torch.Tensor) or value.device.type != "cpu":
        raise ValueError(f"{name} must be a CPU tensor")
    if value.is_floating_point() and not torch.isfinite(value).all():
        raise ValueError(f"{name} contains non-finite values")
    return value


def _observation(value, name, space=None):
    if isinstance(value, dict):
        if not value:
            raise ValueError(f"{name} must not be empty")
        spaces = getattr(space, "spaces", None)
        if spaces is not None and value.keys() != spaces.keys():
            raise ValueError(f"{name} observation keys differ from training spaces")
        return {key: _observation(item, f"{name}.{key}", None if spaces is None else spaces[key]) for key, item in value.items()}
    tensor = _tensor(value, name)
    if tensor.ndim < 1 or tensor.shape[0] != 1:
        raise ValueError(f"{name} requires one vectorized environment")
    if space is not None:
        if tuple(tensor.shape) != (1, *space.shape):
            raise ValueError(f"{name} shape differs from training space")
        import numpy as np
        expected_dtype = torch.from_numpy(np.empty(0, dtype=space.dtype)).dtype
        if tensor.dtype != expected_dtype:
            raise ValueError(f"{name} dtype differs from training space")
    return tuple(tensor.shape)


def validate_admission(record, *, observation_space=None, action_space=None, experiment_hash=None):
    if not isinstance(record, dict):
        raise ValueError("trajectory record must be a dictionary")
    identity = record.get("record_id", record.get("transition_id"))
    if not isinstance(identity, str) or not identity or len(identity) > 1024:
        raise ValueError("record requires a bounded stable ID")
    if "record_id" in record and "transition_id" in record and record["record_id"] != record["transition_id"]:
        raise ValueError("record_id and transition_id must agree")
    if "record_type" in record:
        from real_world_rl.trajectory import validate_record
        validate_record(record)
    if experiment_hash and experiment_hash != "unknown" and record.get("experiment_hash") != experiment_hash:
        raise ValueError("record experiment hash differs from learner")
    for name in ("run_id", "actor_id", "session_id", "experiment_hash"):
        if name in record and (not isinstance(record[name], str) or len(record[name]) > 1024):
            raise ValueError(f"{name} must be a bounded string")
    for name in ("sequence", "step_index", "step_count", "policy_version", "global_update"):
        if name in record and (isinstance(record[name], bool) or not isinstance(record[name], int) or record[name] < 0):
            raise ValueError(f"{name} must be a nonnegative integer")
    for name in ("effective_experiment", "experiment_contract", "action_contract", "transport_metrics"):
        value = record.get(name)
        if value is not None:
            if not isinstance(value, dict):
                raise ValueError(f"{name} must be a JSON mapping")
            try:
                json.dumps(value, allow_nan=False)
            except (TypeError, ValueError) as exc:
                raise ValueError(f"{name} must contain finite JSON metadata") from exc
    if record.get("record_type", "transition") != "transition":
        if record.get("record_type") == "episode_end":
            outcome = record.get("outcome")
            if outcome is not None:
                from real_world_rl.metrics import EpisodeOutcome
                if not isinstance(outcome, dict):
                    raise ValueError("episode outcome must be a mapping")
                try:
                    EpisodeOutcome(**outcome)
                except (TypeError, ValueError) as exc:
                    raise ValueError("invalid episode outcome") from exc
        return
    for key in ("obs", "next_obs", "action", "reward", "done"):
        if key not in record:
            raise ValueError(f"transition missing {key}")
    if _observation(record["obs"], "obs", observation_space) != _observation(record["next_obs"], "next_obs", observation_space):
        raise ValueError("obs and next_obs shapes differ")
    action = _tensor(record["action"], "action")
    if action.ndim != 2 or action.shape[0] != 1 or not action.shape[1]:
        raise ValueError("action must have shape (1,A)")
    if action_space is not None and tuple(action.shape) != (1, *action_space.shape):
        raise ValueError("action shape differs from training action space")
    for key in ("reward", "done"):
        if _tensor(record[key], key).numel() != 1:
            raise ValueError(f"{key} must contain one value")
    if not bool(((record["done"] == 0) | (record["done"] == 1)).all()):
        raise ValueError("done must contain boolean values")
    for key in ("policy_action", "base_actions", "next_base_actions", "executed_action"):
        if key in record and tuple(_tensor(record[key], key).shape) != tuple(action.shape):
            raise ValueError(f"{key} must match action shape")
    for key in ("human_action_mask", "override_mask"):
        if key in record:
            value = _tensor(record[key], key)
            if value.dtype != torch.bool or tuple(value.shape) != tuple(action.shape):
                raise ValueError(f"{key} must be a boolean tensor matching action")
    for key in ("grasp_penalty",):
        if key in record:
            value = record[key]
            if torch.as_tensor(value).numel() != 1 or not torch.isfinite(torch.as_tensor(value)).all():
                raise ValueError(f"{key} must be one finite value")
