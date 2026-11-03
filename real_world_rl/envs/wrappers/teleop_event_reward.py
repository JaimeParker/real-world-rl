"""Reward and episode end from the operator's ``success``/``failure`` buttons.

``TeleopEventRewardWrapper`` reads ``info["teleop_events"]``, which the intervention wrapper
writes on the steps where the teleop device's buttons fired (the keyboard's ``s``/``f`` with
``--teleop-event-device keyboard``, or a device's own event keys), so it must wrap outside the
intervention wrapper:

- ``success``: the reward becomes at least ``success_reward`` and ``terminated`` is set, so the
  transition is stored as a terminal success and the actor loop resets;
- ``failure``: ``truncated`` is set (the episode ends without a terminal transition); a failure
  in the same step as a success wins, and the success is ignored;
- otherwise the step passes through unchanged.

The wrapper only raises: a reward or ``terminated`` the inner env already set (a reward
classifier, say) is never lowered, so both reward sources can run together. Tensor rewards and
flags keep their device and dtype.
"""
from __future__ import annotations

from typing import Any

import gymnasium as gym
import numpy as np
import torch

SUCCESS_EVENT = "success"
FAILURE_EVENT = "failure"


class TeleopEventRewardWrapper(gym.Wrapper):
    """Success/failure events to reward and episode end; see the module docstring."""

    def __init__(self, env: gym.Env, success_reward: float = 1.0) -> None:
        super().__init__(env)
        self.success_reward = float(success_reward)

    def __getattr__(self, name: str):
        # Transparent to code that reads backend attributes (num_envs, single_observation_space,
        # the action contract) directly, as the other wrappers of the HIL stack are.
        return getattr(self.env, name)

    def step(self, action):
        obs, reward, terminated, truncated, info = self.env.step(action)
        events = info.get("teleop_events") or {}
        if events.get(FAILURE_EVENT, False):
            truncated = _set(truncated)
        elif events.get(SUCCESS_EVENT, False):
            reward = _at_least(reward, self.success_reward)
            terminated = _set(terminated)
        return obs, reward, terminated, truncated, info


def _at_least(reward: Any, floor: float) -> Any:
    if isinstance(reward, torch.Tensor):
        return torch.clamp(reward, min=floor)
    if isinstance(reward, np.ndarray):
        return np.maximum(reward, np.asarray(floor, dtype=reward.dtype))
    return max(float(reward), floor)


def _set(flag: Any) -> Any:
    if isinstance(flag, torch.Tensor):
        return torch.ones_like(flag, dtype=torch.bool)
    if isinstance(flag, np.ndarray):
        return np.ones_like(flag, dtype=bool)
    return True
