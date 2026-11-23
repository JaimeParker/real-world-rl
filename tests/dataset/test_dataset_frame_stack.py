"""``iter_transitions`` rebuilds the frame stacks rl-garden's ``ImageFrameStackWrapper`` produced
live, bit for bit, from the single frames and ``stack_reset`` rows the dataset stores."""
from __future__ import annotations

import numpy as np
import pytest
import torch
from gymnasium import spaces
from rl_garden.envs.wrappers.frame_stack import ImageFrameStackWrapper

from real_world_rl.dataset import Dataset, DatasetWriter, iter_transitions

ACTION_DIM = 3
IMAGE_KEYS = ("rgb_wrist", "depth_front")
SPEC = {
    "state": {"shape": [4], "dtype": "float32"},
    "rgb_wrist": {"shape": [5, 7, 3], "dtype": "uint8"},
    "depth_front": {"shape": [5, 7, 1], "dtype": "float32"},
}


class RandomFrameEnv:
    """A batch-of-1 torch env of random frames and states (the generic ``image_keys=`` path)."""

    num_envs = 1

    def __init__(self, seed: int) -> None:
        self.rng = np.random.default_rng(seed)
        self.single_observation_space = spaces.Dict({
            "state": spaces.Box(-np.inf, np.inf, (4,), np.float32),
            "rgb_wrist": spaces.Box(0, 255, (5, 7, 3), np.uint8),
            "depth_front": spaces.Box(0.0, np.inf, (5, 7, 1), np.float32),
        })
        self.unwrapped = self

    def observe(self) -> dict[str, torch.Tensor]:
        return {
            "state": torch.from_numpy(self.rng.standard_normal((1, 4)).astype(np.float32)),
            "rgb_wrist": torch.from_numpy(self.rng.integers(0, 256, (1, 5, 7, 3), dtype=np.uint8)),
            "depth_front": torch.from_numpy(self.rng.random((1, 5, 7, 1)).astype(np.float32)),
        }

    def reset(self, *, seed=None, options=None):
        return self.observe(), {}

    def step(self, action):
        return self.observe(), torch.zeros(1), torch.zeros(1, dtype=torch.bool), torch.zeros(1, dtype=torch.bool), {}


def _record(tmp_path, episodes, *, frame_stack, refill_rows=()):
    """Runs ``episodes`` (one list of intervened flags each) through the real wrapper (or the bare
    env for ``frame_stack=1``), records every observation and returns the live transitions."""
    inner = RandomFrameEnv(seed=frame_stack)
    env = ImageFrameStackWrapper(inner, frame_stack=frame_stack, image_keys=IMAGE_KEYS) if frame_stack > 1 else inner
    writer = DatasetWriter.open(tmp_path / "ds", control_hz=10.0, obs_spec=SPEC, action_dim=ACTION_DIM,
                                recorded_frame_stack=frame_stack)
    live = []
    for intervened in episodes:
        episode = writer.begin_episode()
        obs, _ = env.reset()
        episode.add_observation(obs, time_s=0.0, stack_reset=True)
        for step, flag in enumerate(intervened):
            action = torch.full((1, ACTION_DIM), float(step))
            next_obs, reward, terminated, truncated, _ = env.step(action)
            episode.add_step(action, action, torch.full((1, ACTION_DIM), flag), torch.full((1, ACTION_DIM), flag),
                             flag, reward, terminated, truncated, False, {})
            if flag:
                live.append({"obs": obs, "next_obs": next_obs, "step_index": step})
            refill = step + 1 in refill_rows
            if refill:
                # SessionImageFrameStackWrapper's refill, from the frame the step just returned.
                newest = {key: value[:, -1] if key in IMAGE_KEYS else value for key, value in next_obs.items()}
                obs = env._reset_frames(newest, env_idx=None)
            else:
                obs = next_obs
            episode.add_observation(obs, time_s=0.1 * (step + 1), stack_reset=refill)
        episode.finish(status="kept", success=True)
    return Dataset(tmp_path / "ds"), live


def _assert_equal_obs(rebuilt, live, frame_stack):
    assert set(rebuilt) == set(live)
    for key, value in live.items():
        assert rebuilt[key].dtype == value.dtype and rebuilt[key].shape == value.shape, key
        assert torch.equal(rebuilt[key], value), key
    for key in IMAGE_KEYS:
        assert rebuilt[key].shape[:2] == ((1, frame_stack) if frame_stack > 1 else (1, 5))


@pytest.mark.parametrize("frame_stack", [1, 3])
def test_rebuilt_stacks_equal_the_live_wrapper(tmp_path, frame_stack):
    dataset, live = _record(tmp_path, [[True, True, False, True, True, True], [False, True, True, True]],
                            frame_stack=frame_stack)
    rows = list(iter_transitions(dataset, frame_stack=frame_stack))
    assert [row["step_index"] for row in rows] == [row["step_index"] for row in live] == [0, 1, 3, 4, 5, 1, 2, 3]
    for row, expected in zip(rows, live):
        _assert_equal_obs(row["obs"], expected["obs"], frame_stack)
        _assert_equal_obs(row["next_obs"], expected["next_obs"], frame_stack)


def test_a_mid_episode_refill_restarts_the_history_from_that_row(tmp_path):
    dataset, live = _record(tmp_path, [[True] * 6], frame_stack=3, refill_rows=(3,))
    assert dataset.episode(0).data["stack_reset"].tolist() == [True, False, False, True, False, False, False]
    rows = list(iter_transitions(dataset, frame_stack=3))
    assert len(rows) == len(live) == 6
    for row, expected in zip(rows, live):
        _assert_equal_obs(row["obs"], expected["obs"], 3)
        _assert_equal_obs(row["next_obs"], expected["next_obs"], 3)
    refilled = rows[3]["obs"]["rgb_wrist"][0]
    assert all(torch.equal(refilled[0], frame) for frame in refilled)
    # The step into the refill row saw the shifted history, as step() returned it.
    assert not torch.equal(rows[2]["next_obs"]["rgb_wrist"][0, 0], rows[2]["next_obs"]["rgb_wrist"][0, 2])
