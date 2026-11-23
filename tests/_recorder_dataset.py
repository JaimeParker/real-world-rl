"""Recording into a demo dataset from the tests: ``record_into_dataset`` opens the dataset as
``run_recorder`` does, runs ``record_demos`` and returns the training view of what it wrote."""
from __future__ import annotations

from pathlib import Path
from typing import Any, Mapping, Optional

from real_world_rl.dataset import Dataset, iter_transitions
from real_world_rl.demos.record import hold_action_fn_for, obs_spec_of, open_dataset, record_demos

# The fake envs' observation: ``{"state": (1, 4)}`` (or a bare ``(1, 4)`` state tensor).
STATE4_SPEC = {"state": {"shape": [4], "dtype": "float32"}}


def default_obs_spec(env: Any) -> dict:
    try:
        return obs_spec_of(env)
    except ValueError:
        return dict(STATE4_SPEC)


def record_into_dataset(
    env: Any, args: Any, root: Path, *, hold_action_fn: Any = None,
    obs_spec: Optional[Mapping[str, Any]] = None, on_episode: Any = None,
) -> tuple[list[dict], list, Dataset]:
    """Records on ``env`` into the dataset at ``root``; returns the training rows (frame stack 1,
    syncing steps as ``args.keep_syncing_steps``), the recorded episodes and the dataset."""
    writer = open_dataset(env, args, root, obs_spec=obs_spec or default_obs_spec(env))
    episodes = record_demos(
        env, args, hold_action_fn=hold_action_fn or hold_action_fn_for(env), writer=writer, on_episode=on_episode
    )
    dataset = Dataset(root)
    rows = list(iter_transitions(dataset, frame_stack=1, keep_syncing_steps=args.keep_syncing_steps))
    return rows, episodes, dataset
