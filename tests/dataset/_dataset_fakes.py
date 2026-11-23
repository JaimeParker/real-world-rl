"""Synthetic headers and episodes for the dataset tests (no hardware)."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional, Sequence

import numpy as np
import torch

from real_world_rl.dataset import DatasetWriter

ACTION_DIM = 4
# Odd sizes on purpose: the review video pads them to even.
OBS_SPEC = {
    "state": {"shape": [5], "dtype": "float32"},
    "rgb_wrist": {"shape": [9, 15, 3], "dtype": "uint8"},
    "rgb_front": {"shape": [7, 11, 3], "dtype": "uint8"},
    "depth_front": {"shape": [9, 15, 1], "dtype": "float32"},
}
NAMED_STATE_SPEC = {"joint_pos": [7], "gripper_pos": [1]}
CONTRACT = {"action_mode": "joint_abs", "arm_action_dim": 3, "gripper_in_action": True,
            "joint_limits": [[-1.0, 1.0]] * 3}


def open_writer(root, **overrides) -> DatasetWriter:
    fields = dict(control_hz=10.0, obs_spec=OBS_SPEC, action_dim=ACTION_DIM, action_contract=CONTRACT,
                  named_state_spec=NAMED_STATE_SPEC, task_description="peg", config={"seed": 1})
    fields.update(overrides)
    return DatasetWriter.open(root, **fields)


def random_obs(rng: np.random.Generator, spec=OBS_SPEC) -> dict[str, np.ndarray]:
    """One un-batched observation of ``spec``."""
    obs = {}
    for key, entry in spec.items():
        shape, dtype = tuple(entry["shape"]), np.dtype(entry["dtype"])
        if dtype == np.uint8:
            obs[key] = rng.integers(0, 256, size=shape, dtype=np.uint8)
        else:
            obs[key] = rng.standard_normal(shape).astype(dtype)
    return obs


@dataclass
class SyntheticEpisode:
    """What was fed to an ``EpisodeWriter``, un-batched, for comparing against the files."""

    obs: list = field(default_factory=list)
    named: list = field(default_factory=list)
    times: list = field(default_factory=list)
    steps: list = field(default_factory=list)


def write_episode(
    writer: DatasetWriter,
    rng: np.random.Generator,
    intervened: Sequence[bool],
    *,
    syncing: Optional[Sequence[bool]] = None,
    events: Optional[dict[int, dict[str, bool]]] = None,
    status: str = "kept",
    discard_reason: Optional[str] = None,
    success: Optional[bool] = True,
    grasp_penalty: bool = False,
) -> tuple[SyntheticEpisode, object]:
    """Writes an episode of ``len(intervened)`` steps, alternating torch and numpy inputs."""
    record = SyntheticEpisode()
    episode = writer.begin_episode()
    syncing = syncing or [False] * len(intervened)
    events = events or {}

    def observe(row: int) -> None:
        obs = random_obs(rng)
        named = {"joint_pos": rng.standard_normal(7), "gripper_pos": np.float64(rng.random())}
        batched = {key: (torch.from_numpy(value) if row % 2 else value)[None] for key, value in obs.items()}
        record.obs.append(obs)
        record.named.append(named)
        record.times.append(100.0 + 0.1 * row)
        episode.add_observation(batched, named, time_s=record.times[-1])

    observe(0)
    for step, flag in enumerate(intervened):
        values = {
            "action": rng.standard_normal(ACTION_DIM).astype(np.float32),
            "policy_action": rng.standard_normal(ACTION_DIM).astype(np.float32),
            "human_action_mask": np.full(ACTION_DIM, flag),
            "override_mask": np.full(ACTION_DIM, flag),
            "intervened": bool(flag),
            "reward": float(step == len(intervened) - 1),
            "terminated": step == len(intervened) - 1,
            "truncated": False,
            "syncing": bool(syncing[step]),
            "events": dict(events.get(step, {})),
            "grasp_penalty": float(-0.1 * (step % 2)) if grasp_penalty else None,
        }
        record.steps.append(values)
        tensors = {key: torch.as_tensor(values[key])[None] for key in ("action", "policy_action",
                                                                       "human_action_mask", "override_mask")}
        episode.add_step(
            tensors["action"], values["policy_action"][None], tensors["human_action_mask"],
            tensors["override_mask"], torch.tensor(values["intervened"]), torch.tensor([values["reward"]]),
            torch.tensor([values["terminated"]]), values["truncated"], values["syncing"], values["events"],
            grasp_penalty=None if values["grasp_penalty"] is None else torch.tensor([values["grasp_penalty"]]),
        )
        observe(step + 1)
    info = episode.finish(status=status, discard_reason=discard_reason, success=success)
    return record, info
