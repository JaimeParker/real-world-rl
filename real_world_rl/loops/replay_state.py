"""Replay checkpoints including real-world extension columns.

Keep rl-garden's validation and FWBW cache invalidation, and add the columns
owned by this package. States contain independent CPU copies, so a running
actor cannot mutate a checkpoint awaiting publication.
"""
from __future__ import annotations

from typing import Any
import warnings

import torch
from rl_garden.common.checkpoint import (
    load_replay_buffer_state_dict,
    replay_buffer_state_dict,
)

_EXTENSION_COLUMNS = (
    "intervened", "policy_actions", "base_actions", "next_base_actions",
    "intervention_onset", "grasp_penalty", "human_action_mask", "override_mask",
)


def _copy(value: Any) -> Any:
    if isinstance(value, torch.Tensor):
        return value.detach().cpu().clone()
    if isinstance(value, dict):
        return {key: _copy(item) for key, item in value.items()}
    if isinstance(value, list):
        return [_copy(item) for item in value]
    if isinstance(value, tuple):
        return tuple(_copy(item) for item in value)
    return value


def serialize_buffer(buffer: Any) -> dict[str, Any]:
    """Serialize core/FWBW state and every configured intervention column."""
    state = replay_buffer_state_dict(buffer)
    state["real_world_replay_version"] = 1
    state["legacy_provenance_unknown"] = bool(getattr(buffer, "legacy_provenance_unknown", False))
    for name in _EXTENSION_COLUMNS:
        column = getattr(buffer, name, None)
        if isinstance(column, torch.Tensor):
            state[name] = column
    return _copy(state)


def validate_buffer(buffer: Any, state: dict[str, Any]) -> None:
    """Check compatibility without mutating model, replay, or cursor state."""
    expected = replay_buffer_state_dict(buffer)
    for name in ("format_version", "buffer_class", "num_envs", "buffer_size", "per_env_buffer_size"):
        if state.get(name) != expected[name]:
            raise ValueError(f"Replay {name} mismatch: {state.get(name)!r}, expected {expected[name]!r}")
    if state.get("real_world_replay_version", 1) != 1:
        raise ValueError("Unsupported real-world replay version")
    pos = state.get("pos")
    if not isinstance(pos, int) or not 0 <= pos < buffer.per_env_buffer_size:
        raise ValueError(f"Invalid replay cursor position: {pos}")
    if not isinstance(state.get("full"), bool):
        raise ValueError("Replay full cursor must be bool")

    def check_tree(target, source, name):
        if isinstance(target, dict):
            if not isinstance(source, dict) or set(target) != set(source):
                raise ValueError(f"Replay observation keys mismatch for {name}")
            for key, item in target.items():
                check_tree(item, source[key], f"{name}.{key}")
        elif isinstance(target, torch.Tensor):
            if not isinstance(source, torch.Tensor) or source.shape != target.shape or source.dtype != target.dtype:
                raise ValueError(f"Replay shape/dtype mismatch for {name}")

    for name in ("obs", "next_obs", "actions", "rewards", "dones"):
        check_tree(expected[name], state.get(name), name)
    for name in ("mc_table", "episode_end", "externally_valid", "external_mc", "step_success"):
        if name in state and name in expected and expected[name] is not None:
            check_tree(expected[name], state[name], name)
    legacy = "real_world_replay_version" not in state
    for name in _EXTENSION_COLUMNS:
        target = getattr(buffer, name, None)
        if not isinstance(target, torch.Tensor):
            if name in state:
                raise ValueError(f"Replay checkpoint has unconfigured column {name!r}")
        elif name not in state:
            if not (legacy and name in ("intervened", "policy_actions", "intervention_onset", "human_action_mask", "override_mask")):
                raise ValueError(f"Replay checkpoint is missing {name!r}; recover legacy transition snapshots")
        else:
            check_tree(target, state[name], name)


def restore_buffer(buffer: Any, state: dict[str, Any]) -> None:
    """Restore a matching buffer; reject missing columns rather than invent labels.

    Legacy native states retain core transitions with unknown human provenance:
    missing expert flags are false, proposals use actions as a placeholder, and
    a warning identifies the loss of provenance. Base actions and configured
    penalties cannot be reconstructed and remain required.
    """
    validate_buffer(buffer, state)
    legacy = "real_world_replay_version" not in state
    state = dict(state)
    if legacy:
        missing_provenance = []
        for name in ("intervened", "policy_actions", "intervention_onset", "human_action_mask", "override_mask"):
            target = getattr(buffer, name, None)
            if isinstance(target, torch.Tensor) and name not in state:
                if name in ("human_action_mask", "override_mask"):
                    flags = state.get("intervened", torch.zeros(target.shape[:-1], dtype=torch.bool))
                    state[name] = flags[..., None].expand_as(target).clone().cpu()
                else:
                    state[name] = state["actions"].clone() if name == "policy_actions" else torch.zeros_like(target, device="cpu")
                missing_provenance.append(name)
        if missing_provenance:
            warnings.warn(
                "Legacy replay lacks intervention provenance (" + ", ".join(missing_provenance)
                + "); restored rows are not expert labels and policy proposals are unknown. "
                "Recover transition snapshots for exact intervention history.",
                stacklevel=2,
            )
    version = state.get("real_world_replay_version", 1)
    if version != 1:
        raise ValueError(f"Unsupported real-world replay version: {version}")
    pos = int(state["pos"])
    if not 0 <= pos < buffer.per_env_buffer_size:
        raise ValueError(f"Invalid replay cursor position: {pos}")
    columns = {}
    for name in _EXTENSION_COLUMNS:
        target = getattr(buffer, name, None)
        if not isinstance(target, torch.Tensor):
            if name in state:
                raise ValueError(f"Replay checkpoint has unconfigured column {name!r}")
            continue
        source = state.get(name)
        if not isinstance(source, torch.Tensor):
            raise ValueError(f"Replay checkpoint is missing {name!r}; recover legacy transition snapshots")
        if source.shape != target.shape or source.dtype != target.dtype:
            raise ValueError(f"Replay column {name!r} shape/dtype mismatch: {source.shape}/{source.dtype}, expected {target.shape}/{target.dtype}")
        columns[name] = source.to(buffer.storage_device).clone()
    load_replay_buffer_state_dict(buffer, _copy(state))
    for name, column in columns.items():
        setattr(buffer, name, column)
    buffer.legacy_provenance_unknown = bool(state.get("legacy_provenance_unknown", False) or (legacy and missing_provenance))
    persist = getattr(buffer, "_persist_cursor", None)
    if persist is not None:
        persist()
