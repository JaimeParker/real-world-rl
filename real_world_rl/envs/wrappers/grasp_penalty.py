"""Gripper-flip reward-shaping wrapper (HIL-SERL's ``grasp_penalty``): flags
``info["grasp_penalty"]`` whenever the gripper action flips open<->closed
relative to the previous step, for ``RLPDHybrid``'s discrete critic target to
add on top of the raw environment reward (``use_grasp_penalty=True``).

Ported from ``3rd_party/hil-serl/examples/experiments/usb_pickup_insertion/
wrapper.py``'s task-specific ``GripperPenaltyWrapper`` -- the one HIL-SERL's
own ``train_rlpd.py``/``sac_hybrid_single.py`` actually consume via
``info["grasp_penalty"]`` (an additive side channel), not the base
``franka_env/envs/wrappers.py::GripperPenaltyWrapper``, which instead mutates
``reward`` directly and is never wired into the real training loop.

Must wrap *outside* ``TeleopInterventionWrapper`` in the wrapper stack (i.e.
applied after it) so ``info.get("intervene_action")`` -- the actually-executed
action under human intervention -- is already present when this wrapper's
``step()`` runs.

With several grippers (``gripper_slice``, the last ``num_arms`` action dims of a multi-arm
env, see ``action_contract``) the penalty is ``penalty`` times the number of grippers that
flipped, summed into the one scalar per env.
"""
from __future__ import annotations

from typing import Optional

import gymnasium as gym
import torch


class GraspPenaltyWrapper(gym.Wrapper):
    """``gripper_slice`` selects the gripper dims of the executed action; without it the single
    gripper is ``gripper_action_idx`` (the last dim by default)."""

    def __init__(
        self,
        env: gym.Env,
        penalty: float = -0.05,
        gripper_action_idx: int = -1,
        gripper_slice: Optional[slice] = None,
    ) -> None:
        super().__init__(env)
        self.penalty = penalty
        self.gripper_action_idx = gripper_action_idx
        if gripper_slice is None:
            gripper_slice = slice(gripper_action_idx, gripper_action_idx + 1 or None)
        self.gripper_slice = gripper_slice
        self._last_gripper_open: Optional[torch.Tensor] = None  # per-env, per-gripper bool

    def __getattr__(self, name: str):
        return getattr(self.env, name)

    def reset(self, **kwargs):
        obs, info = self.env.reset(**kwargs)
        self._last_gripper_open = None
        return obs, info

    def step(self, action):
        obs, reward, terminated, truncated, info = self.env.step(action)
        executed_action = info.get("intervene_action", action)
        is_open = executed_action[..., self.gripper_slice] <= 0
        if self._last_gripper_open is None:
            flipped = torch.zeros_like(is_open, dtype=torch.bool)
        else:
            flipped = is_open != self._last_gripper_open
        self._last_gripper_open = is_open
        flip_count = flipped.sum(dim=-1)
        info = dict(info)
        info["grasp_penalty"] = torch.where(
            flip_count > 0,
            flip_count.to(reward.dtype) * self.penalty,
            torch.zeros_like(reward),
        )
        return obs, reward, terminated, truncated, info
