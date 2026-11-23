"""A small synthetic dataset (written by ``DatasetWriter``) for the dataset CLI and export tests."""
from __future__ import annotations

import numpy as np

from real_world_rl.dataset import DatasetWriter

ACTION_DIM = 3
OBS_SPEC = {
    "state": {"shape": [5], "dtype": "float32"},
    "rgb_wrist": {"shape": [12, 16, 3], "dtype": "uint8"},
    "depth_front": {"shape": [12, 16, 1], "dtype": "float32"},
}
NAMED_STATE_SPEC = {"joint_pos": [7], "gripper_pos": [1]}
CONTRACT = {"action_mode": "joint_abs", "arm_action_dim": 2, "gripper_in_action": True,
            "joint_limits": [[-1.0, 1.0]] * 2}
# (steps, status, discard_reason): index 0 kept, 1 discarded, 2 kept.
EPISODES = ((6, "kept", None), (5, "discarded", "rerecord"), (4, "kept", None))


def build_dataset(root, *, task="insert the peg") -> dict[int, dict]:
    """Writes ``EPISODES`` into ``root``; returns the arrays written, by episode index."""
    writer = DatasetWriter.open(root, control_hz=10.0, obs_spec=OBS_SPEC, action_dim=ACTION_DIM,
                                action_contract=CONTRACT, named_state_spec=NAMED_STATE_SPEC,
                                task_description=task, config={})
    rng = np.random.default_rng(0)
    written = {}
    for steps, status, reason in EPISODES:
        episode = writer.begin_episode()
        record = {"state": [], "joint_pos": [], "gripper_pos": [], "rgb": [], "action": [], "intervened": [],
                  "human_action_mask": []}

        def observe(row: int) -> None:
            obs = {"state": rng.standard_normal(5).astype(np.float32),
                   "rgb_wrist": rng.integers(0, 256, size=(12, 16, 3), dtype=np.uint8),
                   "depth_front": rng.random((12, 16, 1)).astype(np.float32)}
            named = {"joint_pos": rng.standard_normal(7), "gripper_pos": rng.random()}
            record["state"].append(obs["state"])
            record["rgb"].append(obs["rgb_wrist"])
            record["joint_pos"].append(named["joint_pos"].astype(np.float32))
            record["gripper_pos"].append(np.float32(named["gripper_pos"]))
            episode.add_observation({key: value[None] for key, value in obs.items()}, named,
                                    time_s=100.0 + 0.1 * row)

        observe(0)
        for step in range(steps):
            intervened = step >= steps // 2
            action = rng.standard_normal(ACTION_DIM).astype(np.float32)
            mask = np.full(ACTION_DIM, intervened)
            record["action"].append(action)
            record["intervened"].append(intervened)
            record["human_action_mask"].append(mask)
            done = step == steps - 1
            episode.add_step(action[None], action[None], mask[None], mask[None], np.array([intervened]),
                             np.array([float(done)], np.float32), np.array([done]), np.array([False]),
                             np.array([False]), {})
            observe(step + 1)
        info = episode.finish(status=status, discard_reason=reason, success=status == "kept")
        written[info.index] = {key: np.asarray(value) for key, value in record.items()}
    return written
