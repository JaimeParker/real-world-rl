"""The training view of a dataset: the transition dicts the old recorder pickled.

``iter_transitions`` yields, for kept episodes only, step ``t`` when
``intervened[t] and (keep_syncing_steps or not syncing[t])``, with ``training_segment_end`` on a
row whose next selected row is not ``t + 1``. Images are frame-stacked as rl-garden's
``ImageFrameStackWrapper`` stacks them live: the history is ``frame_stack`` copies of the frame at
a ``stack_reset`` row and shifts by one frame otherwise; ``state`` is never stacked. A step's
``next_obs`` is its ``obs`` history shifted by the next frame, as ``step()`` returned it live, also
when the next row refills the history (the refill applies from that row's own step on).
``export_pkl`` writes those rows as a ``load_demo_pkls`` pkl with its contract sidecar. torch is
imported inside these functions only.
"""
from __future__ import annotations

import pickle
from pathlib import Path
from typing import Any, Iterator

import numpy as np

from real_world_rl.dataset import format as fmt
from real_world_rl.dataset.reader import Dataset, Episode

__all__ = ["export_pkl", "iter_transitions", "next_stack", "selected_steps", "stack_indices"]


def selected_steps(data: dict[str, np.ndarray], *, keep_syncing_steps: bool = False) -> np.ndarray:
    """Indices of the steps the training view keeps."""
    keep = data["intervened"].astype(bool)
    if not keep_syncing_steps:
        keep &= ~data["syncing"].astype(bool)
    return np.flatnonzero(keep)


def stack_indices(stack_reset: np.ndarray, frame_stack: int) -> np.ndarray:
    """``(T+1, frame_stack)`` frame rows making up each observation row's stack, oldest first."""
    rows = np.empty((len(stack_reset), frame_stack), dtype=np.int64)
    history = None
    for row, reset in enumerate(stack_reset):
        history = [row] * frame_stack if reset or history is None else history[1:] + [row]
        rows[row] = history
    return rows


def next_stack(stack: np.ndarray, row: int) -> np.ndarray:
    """The frame rows of ``step()``'s returned stack: ``stack`` shifted by frame ``row``."""
    return np.append(stack[1:], row)


def iter_transitions(
    dataset: Dataset, *, frame_stack: int, keep_syncing_steps: bool = False
) -> Iterator[dict[str, Any]]:
    """The old recorder's pkl rows for every kept episode, in index order (see module doc)."""
    if frame_stack < 1:
        raise ValueError("frame_stack must be at least 1")
    for info in dataset.episodes():
        if info.kept:
            yield from _episode_transitions(dataset, dataset.episode(info.index), frame_stack, keep_syncing_steps)


def export_pkl(
    dataset: Dataset, path: str | Path, *, frame_stack: int, keep_syncing_steps: bool = False
) -> Path:
    """Pickles ``iter_transitions`` as a list to ``path`` (``load_demo_pkls`` format) and writes
    the contract sidecar next to it when the header has an ``action_contract``."""
    from real_world_rl.contracts.action import write_contract_sidecar

    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    rows = list(iter_transitions(dataset, frame_stack=frame_stack, keep_syncing_steps=keep_syncing_steps))
    with open(path, "wb") as stream:
        pickle.dump(rows, stream)
    contract = dataset.header.get("action_contract")
    if contract is not None:
        write_contract_sidecar(path, contract)
    return path


def _episode_transitions(
    dataset: Dataset, episode: Episode, frame_stack: int, keep_syncing_steps: bool
) -> Iterator[dict[str, Any]]:
    import torch

    data = episode.data
    steps = selected_steps(data, keep_syncing_steps=keep_syncing_steps)
    if not len(steps):
        return
    keys = fmt.frame_keys(dataset.header["obs_spec"])
    frames = {key: episode.frames(key) for key in keys}
    stacks = stack_indices(data["stack_reset"], frame_stack) if frame_stack > 1 else None

    def observation(row: int, stack: Any) -> dict[str, Any]:
        obs: dict[str, Any] = {}
        if fmt.STATE_KEY in data:
            obs[fmt.STATE_KEY] = torch.from_numpy(data[fmt.STATE_KEY][row : row + 1].copy())
        for key in keys:
            image = frames[key][row : row + 1] if stack is None else frames[key][stack][None]
            obs[key] = torch.from_numpy(np.ascontiguousarray(image))
        return obs

    def row_of(name: str, step: int) -> Any:
        return torch.from_numpy(data[name][step : step + 1].copy())

    for position, step in enumerate(steps):
        step = int(step)
        terminated = row_of("terminated", step)
        transition: dict[str, Any] = {
            "obs": observation(step, None if stacks is None else stacks[step]),
            "next_obs": observation(step + 1, None if stacks is None else next_stack(stacks[step], step + 1)),
            "action": row_of("action", step),
            "reward": row_of("reward", step),
            "done": terminated.clone(),
            "human_action_mask": row_of("human_action_mask", step),
            "override_mask": row_of("override_mask", step),
            "policy_action": row_of("policy_action", step),
            "intervened": True,
            "terminated": terminated,
            "truncated": row_of("truncated", step),
            "syncing": False,
            "episode_id": episode.meta["episode_id"],
            "step_index": step,
        }
        if "grasp_penalty" in data:
            transition["grasp_penalty"] = row_of("grasp_penalty", step)
        if position + 1 == len(steps) or int(steps[position + 1]) != step + 1:
            transition["training_segment_end"] = True
        yield transition
