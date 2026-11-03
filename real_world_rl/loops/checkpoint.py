"""Loop-owned, atomically committed checkpoint generations."""
from __future__ import annotations

import json
import os
import uuid
from pathlib import Path
from typing import Any

import torch

from real_world_rl.loops.replay_state import serialize_buffer, restore_buffer, validate_buffer
from real_world_rl.checkpoint_contract import load_with_action_contract


def _sync_file(path: Path) -> None:
    with path.open("rb") as stream:
        os.fsync(stream.fileno())


def save_bundle(agent, directory: str | Path, loop_state: dict[str, Any]) -> Path:
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    generation = directory / f"bundle_{agent.global_update}_{uuid.uuid4().hex}"
    generation.mkdir()
    agent.save(generation / "agent.pt", include_replay_buffer=False)
    state = {
        "version": 1,
        "online": serialize_buffer(agent.replay_buffer),
        "demo": None if getattr(agent, "offline_replay_buffer", None) is None else serialize_buffer(agent.offline_replay_buffer),
        "loop": loop_state,
    }
    torch.save(state, generation / "replay.pt")
    (generation / "experiment.json").write_text(json.dumps({key: loop_state.get(key) for key in
        ("experiment_hash", "experiment_contract", "effective_experiment")}, sort_keys=True, allow_nan=False))
    for name in ("agent.pt", "replay.pt", "experiment.json"):
        _sync_file(generation / name)
    descriptor = os.open(generation, os.O_RDONLY)
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)
    manifest = directory / "bundle_manifest.json"
    temporary = directory / f".manifest_{uuid.uuid4().hex}.tmp"
    temporary.write_text(json.dumps({"version": 1, "generation": generation.name}))
    _sync_file(temporary)
    os.replace(temporary, manifest)
    descriptor = os.open(directory, os.O_RDONLY)
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)
    return manifest


def restore_bundle(agent, path: str | Path, *, load_replay_buffer: bool = True, expected_journal_identity: str | None = None, expected_experiment_hash: str | None = None, expected_seed_datasets=None) -> dict[str, Any] | None:
    path = Path(path)
    if path.is_dir():
        path = path / "bundle_manifest.json"
    if path.name != "bundle_manifest.json":
        # Existing standalone checkpoints retain their documented load semantics.
        load_with_action_contract(agent, path, load_replay_buffer=False)
        if load_replay_buffer and legacy_has_replay(path):
            from rl_garden.common.checkpoint import replay_buffer_path_for_checkpoint
            checkpoint = torch.load(path, map_location="cpu", weights_only=False)
            reference = checkpoint.get("metadata", {}).get("replay_buffer_path")
            replay = path.parent / reference if reference else replay_buffer_path_for_checkpoint(path)
            restore_buffer(agent.replay_buffer, torch.load(replay, map_location="cpu", weights_only=False))
        return None
    manifest = json.loads(path.read_text())
    if manifest.get("version") != 1:
        raise ValueError("unsupported learner checkpoint manifest")
    generation = path.parent / manifest["generation"]
    if generation.parent.resolve() != path.parent.resolve():
        raise ValueError("checkpoint generation must be inside checkpoint directory")
    state = torch.load(generation / "replay.pt", map_location="cpu", weights_only=True)
    if state.get("version") != 1:
        raise ValueError("unsupported learner replay bundle")
    if expected_journal_identity is not None and state["loop"]["journal_identity"] != expected_journal_identity:
        raise ValueError("checkpoint journal identity differs; restore the matching durable journal")
    if expected_experiment_hash and expected_experiment_hash != "unknown" and state["loop"].get("experiment_hash") != expected_experiment_hash:
        raise ValueError("checkpoint experiment hash differs from requested experiment")
    if expected_seed_datasets:
        fingerprint = lambda entries: sorted((entry["sha256"], entry["bytes"]) for entry in entries)
        if fingerprint(expected_seed_datasets) != fingerprint(state["loop"].get("seed_datasets", [])):
            raise ValueError("checkpoint seed dataset contents differ from requested seeds")
    validate_buffer(agent.replay_buffer, state["online"])
    if state["demo"] is not None:
        if getattr(agent, "offline_replay_buffer", None) is None:
            raise ValueError("checkpoint contains demo replay; initialize demo buffer before restore")
        validate_buffer(agent.offline_replay_buffer, state["demo"])
    load_with_action_contract(agent, generation / "agent.pt", load_replay_buffer=False)
    restore_buffer(agent.replay_buffer, state["online"])
    if state["demo"] is not None:
        if getattr(agent, "offline_replay_buffer", None) is None:
            raise ValueError("checkpoint contains demo replay; initialize demo buffer before restore")
        restore_buffer(agent.offline_replay_buffer, state["demo"])
    return state["loop"]


def legacy_has_replay(path) -> bool:
    if path is None:
        return False
    path = Path(path)
    if path.is_dir() or path.name == "bundle_manifest.json":
        return False
    from rl_garden.common.checkpoint import replay_buffer_path_for_checkpoint
    checkpoint = torch.load(path, map_location="cpu", weights_only=False)
    reference = checkpoint.get("metadata", {}).get("replay_buffer_path")
    replay = path.parent / reference if reference else replay_buffer_path_for_checkpoint(path)
    return replay.exists()
