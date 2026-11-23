"""Shared glob+load demo-transition loading, factored out of
``HilSerlLearnerLoop._load_demo_datasets`` so the real-world BC entrypoint
(``training/entrypoints/bc.py``) and the online HIL-SERL demo-buffer preload share one
implementation instead of two copies that could silently drift apart.

A demo source is a demo dataset directory (``real_world_rl.dataset``: a directory holding
``dataset.json``, the recorder's output), a pkl (a pickled list of transition dicts) with its
``.contract.json`` sidecar, or a legacy ``.trajectory.pt`` / ``.trajectory.sqlite3`` archive.
Every source's rows go through the same projection to the adder.
"""
from __future__ import annotations

import glob
import hashlib
import inspect
import os
import pickle
from pathlib import Path
from typing import Any, Callable, Iterable, Mapping, Sequence

import torch

from real_world_rl.contracts.action import (
    read_contract_sidecar,
    validate_transitions_against_contract,
)
from real_world_rl.contracts.trajectory import TRAINING_EXTRA_KEYS

_TRANSITION_TENSOR_KEYS = ("obs", "next_obs", "action", "reward", "done")


def training_extras(transition: Mapping[str, Any], adder: Callable[..., None]) -> dict[str, Any]:
    """Project known training columns to the target buffer's declared interface."""
    from real_world_rl.contracts.trajectory import training_projection
    transition = training_projection(transition)
    parameters = inspect.signature(adder).parameters
    accepts_kwargs = any(p.kind == inspect.Parameter.VAR_KEYWORD for p in parameters.values())
    return {key: value for key, value in transition.items()
            if key in TRAINING_EXTRA_KEYS and (accepts_kwargs or key in parameters)}


def as_dict_observation(obs: Any) -> Any:
    """Wrap a legacy flat-tensor observation as ``{"state": obs}``.

    rl-garden replay buffers are Dict-only (state-only observations live
    under the ``"state"`` key), but pkls written before that change (and
    state-only demo pkls) store ``obs``/``next_obs`` as bare tensors.
    Dict observations in legacy franka_real demos pass through untouched.
    """
    return obs if isinstance(obs, Mapping) else {"state": obs}


def is_dataset_dir(path: str | os.PathLike) -> bool:
    """Whether ``path`` is a demo dataset directory (it holds ``dataset.json``)."""
    from real_world_rl.dataset.format import DATASET_FILE

    return os.path.isdir(path) and os.path.isfile(os.path.join(path, DATASET_FILE))


def _dataset_source(path: str, frame_stack: int) -> tuple[Iterable[Mapping[str, Any]], Any, list]:
    """The training rows of the dataset at ``path`` (streamed, one episode in memory), its
    recorded action contract, and a one-row stand-in carrying the header's action width for the
    contract check."""
    from real_world_rl.dataset import Dataset, iter_transitions

    dataset = Dataset(path)
    header = dataset.header
    # The recording's own choice: whether ramp-onset (syncing) steps are supervision rows.
    keep_syncing = bool((header.get("config") or {}).get("keep_syncing_steps", False))
    rows = iter_transitions(dataset, frame_stack=frame_stack, keep_syncing_steps=keep_syncing)
    width_probe = [{"action": torch.zeros(1, int(header["action_dim"]))}]
    return rows, header.get("action_contract"), width_probe


def demo_source_digest(path: str) -> dict[str, Any]:
    """``{"path", "sha256", "bytes"}`` of a demo source; a dataset directory hashes its header and
    visible episode files in path order (unfinished ``.tmp-`` writes are skipped)."""
    digest = hashlib.sha256()
    if is_dataset_dir(path):
        root = Path(path)
        files = sorted(file for file in root.rglob("*") if file.is_file()
                       and not any(part.startswith(".tmp-") for part in file.relative_to(root).parts))
    else:
        files = [Path(path)]
    size = 0
    for file in files:
        if len(files) > 1:
            digest.update(file.relative_to(path).as_posix().encode() + b"\0")
        with open(file, "rb") as stream:
            for chunk in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(chunk)
        size += file.stat().st_size
    return {"path": os.path.abspath(path), "sha256": digest.hexdigest(), "bytes": size}


def load_demo_pkls(
    paths: Sequence[str],
    adder: Callable[..., None],
    device: torch.device | str,
    expected_contract: Mapping[str, Any] | None = None,
    on_contract: Callable[[Mapping[str, Any] | None], None] | None = None,
    *,
    frame_stack: int = 1,
) -> int:
    """Glob each pattern in ``paths``, load every match (a demo dataset directory, a pkl of
    transition dicts or a legacy trajectory archive), and call ``adder(obs, next_obs, action,
    reward, done, **extra)`` for each transition, moving every tensor value to ``device``
    first. Returns the number of transitions loaded.

    A dataset directory yields ``real_world_rl.dataset.iter_transitions`` rows (its kept
    episodes' training view, the old recorder's pkl rows) with images stacked to
    ``frame_stack``, the training run's ``obs.frame_stack``; pkls and archives hold rows
    already stacked as they were recorded.

    With ``expected_contract`` (``action_contract.DEMO_CONTRACT_KEYS`` of the target env), every
    source is checked before any of its transitions is added: a dataset's header
    ``action_contract`` or a ``<pkl stem>.contract.json`` sidecar must match on
    action_mode/arm_action_dim/gripper_in_action and the limits, scales and state_fields it
    records, and without one only the action width is checked (with a warning). ``None`` skips
    all checks."""
    count = 0
    seen_paths = set()
    for pattern in paths:
        for pkl_path in sorted(glob.glob(pattern)):
            canonical = Path(pkl_path).resolve()
            if canonical in seen_paths:
                continue
            seen_paths.add(canonical)
            checked = None
            if is_dataset_dir(pkl_path):
                transitions, saved_contract, checked = _dataset_source(pkl_path, frame_stack)
            elif os.path.isdir(pkl_path):
                raise ValueError(f"demo path {pkl_path} is a directory without a dataset.json; "
                                 "pass a demo dataset directory or a .pkl")
            elif pkl_path.endswith((".trajectory.pt", ".trajectory.sqlite3")):
                from real_world_rl.contracts.trajectory import validate_record
                archive_store = None
                if pkl_path.endswith(".sqlite3"):
                    from real_world_rl.loops.durable import DurableStore, decode
                    archive_store = DurableStore(pkl_path)
                    def records_iter():
                        cursor = 0
                        while True:
                            batch = archive_store.read(cursor, limit=16)
                            if not batch:
                                return
                            for _, _, payload in batch:
                                yield decode(payload)
                            cursor = batch[-1][0]
                    saved_contract = None
                else:
                    archive = torch.load(pkl_path, map_location="cpu", weights_only=True)
                    records_iter = lambda: iter(archive["records"])
                    saved_contract = archive.get("action_contract")
                try:
                    successful = set()
                    for record in records_iter():
                        validate_record(record)
                        if record["record_type"] == "episode_start":
                            episode_contract = record.get("action_contract")
                            if episode_contract is not None and expected_contract is not None:
                                validate_transitions_against_contract([], expected_contract, source=pkl_path, saved=episode_contract)
                            if saved_contract is None:
                                saved_contract = episode_contract
                        if record["record_type"] == "episode_end":
                            outcome = record.get("outcome") or {}
                            if outcome.get("task_success") is True and outcome.get("completed", True) and not (record.get("interrupted") or outcome.get("discarded") or outcome.get("interrupted")):
                                successful.add(record["episode_id"])
                    transitions = [record for record in records_iter() if record["record_type"] == "transition"
                                   and record["episode_id"] in successful and not record.get("syncing", False)
                                   and bool(torch.as_tensor(record.get("human_action_mask", False)).any())]
                    # A filtered human view is a set of contiguous training segments.
                    # Preserve bootstrap done, but prevent n-step windows crossing gaps.
                    for index, record in enumerate(transitions):
                        following = transitions[index + 1] if index + 1 < len(transitions) else None
                        if following is None or following["episode_id"] != record["episode_id"] or following["step_index"] != record["step_index"] + 1:
                            record["training_segment_end"] = True
                finally:
                    if archive_store is not None:
                        archive_store.close()
            else:
                with open(pkl_path, "rb") as f:
                    transitions = pickle.load(f)
                saved_contract = read_contract_sidecar(pkl_path)
            validate_transitions_against_contract(
                transitions if checked is None else checked,
                expected_contract,
                source=pkl_path,
                saved=saved_contract,
            )
            if on_contract is not None:
                on_contract(saved_contract)
            for transition in transitions:
                transition = dict(transition)
                if "intervened" not in transition and "human_action_mask" not in transition:
                    transition["intervened"] = True
                from real_world_rl.contracts.trajectory import training_projection
                transition = training_projection(transition)
                tensors = {
                    k: (v.to(device) if isinstance(v, torch.Tensor) else v)
                    for k, v in transition.items()
                    if k in _TRANSITION_TENSOR_KEYS
                }
                extra: dict[str, Any] = {
                    k: (v.to(device) if isinstance(v, torch.Tensor) else v)
                    for k, v in training_extras(transition, adder).items()
                }
                adder(
                    as_dict_observation(tensors["obs"]),
                    as_dict_observation(tensors["next_obs"]),
                    tensors["action"],
                    tensors["reward"],
                    tensors["done"],
                    **extra,
                )
                count += 1
    return count


def read_demo_contract(paths: Sequence[str]) -> Mapping[str, Any] | None:
    """Read the first available demo contract (a dataset header's, a pkl sidecar's or an
    archive's) without constructing a robot or learner."""
    for pattern in paths:
        for path in sorted(glob.glob(pattern)):
            if is_dataset_dir(path):
                from real_world_rl.dataset import Dataset

                saved = Dataset(path).header.get("action_contract")
            elif path.endswith(".trajectory.pt"):
                saved = torch.load(path, map_location="cpu", weights_only=True).get("action_contract")
            elif path.endswith(".trajectory.sqlite3"):
                from real_world_rl.loops.durable import DurableStore, decode
                store = DurableStore(path)
                saved = None
                cursor = 0
                try:
                    while saved is None:
                        rows = store.read(cursor, limit=16)
                        if not rows:
                            break
                        for _, _, payload in rows:
                            record = decode(payload)
                            if record.get("record_type") == "episode_start" and record.get("action_contract") is not None:
                                saved = record["action_contract"]
                                break
                        cursor = rows[-1][0]
                finally:
                    store.close()
            else:
                saved = read_contract_sidecar(path)
            if saved is not None:
                return saved
    return None
