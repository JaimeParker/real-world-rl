"""Converts the quaternion slice(s) of a Franka real-robot env's ``state``
observation into 3D rotation vectors, matching the rotation representation
the action side already uses (``FrankaRealEnv.step()``'s rotvec deltas).
Optional and off by default -- HIL-SERL converts to Euler via
``Quat2EulerWrapper`` instead, but rotvec avoids Euler's gimbal-lock/wraparound
special-casing and stays consistent with this repo's own action convention.
A two-arm env has one quaternion per arm; pass one slice per arm.
"""
from __future__ import annotations

from typing import Sequence, Union

import gymnasium as gym
import numpy as np
import torch
from gymnasium import spaces
from gymnasium.vector.utils import batch_space

# Default quaternion position in the flat "state" layout of Franka-style envs:
# pose (xyz[0:3] + quat_xyzw[3:7]) + vel(6) + gripper_pos(1) + force(3) +
# torque(3). Other robots pass their own ``quat_slice``.
_DEFAULT_QUAT_SLICE = (3, 7)
_QUAT_DIM = 4
_SMALL_ANGLE_EPS = 1e-8

QuatSlice = tuple[int, int]
# One ``(start, end)`` pair, or one pair per quaternion (e.g. per arm).
QuatSlices = Union[QuatSlice, Sequence[QuatSlice]]


def as_quat_slice_list(quat_slice: QuatSlices) -> list[QuatSlice]:
    """``quat_slice`` as a list of ``(start, end)`` pairs (a single pair becomes a one-item list)."""
    if len(quat_slice) == 2 and all(isinstance(bound, (int, np.integer)) for bound in quat_slice):
        return [(int(quat_slice[0]), int(quat_slice[1]))]
    return [(int(start), int(end)) for start, end in quat_slice]


def quat_xyzw_to_rotvec(q: torch.Tensor) -> torch.Tensor:
    """Pure-torch xyzw quaternion -> rotation vector (``(..., 4) -> (..., 3)``).

    Matches ``scipy.spatial.transform.Rotation.as_rotvec``: ``q`` and ``-q``
    give the same rotation (the shortest-angle representative, angle in
    ``[0, pi]``). Input need not be exactly unit norm. Output keeps the input's
    device and dtype.
    """
    xyz, w = q[..., :3], q[..., 3:4]
    # Canonicalise to w >= 0 so the angle stays in [0, pi].
    sign = torch.where(w < 0, -torch.ones_like(w), torch.ones_like(w))
    xyz, w = xyz * sign, w * sign
    sin_half = torch.linalg.norm(xyz, dim=-1, keepdim=True)
    angle = 2.0 * torch.atan2(sin_half, w)
    small = sin_half < _SMALL_ANGLE_EPS
    # angle / sin(angle/2) -> 2 / w as the angle -> 0; avoids dividing by ~0.
    scale = torch.where(small, 2.0 / w.clamp_min(_SMALL_ANGLE_EPS), angle / sin_half.clamp_min(_SMALL_ANGLE_EPS))
    return xyz * scale


class RotvecObsWrapper(gym.ObservationWrapper):
    """Shrinks ``obs["state"]`` by one dim per quaternion by replacing each 4D xyzw
    quaternion at ``quat_slice`` (default ``(3, 7)``; a sequence of pairs for several,
    e.g. one per arm) with its 3D rotation vector."""

    def __init__(
        self, env: gym.Env, quat_slice: QuatSlices = _DEFAULT_QUAT_SLICE
    ) -> None:
        super().__init__(env)
        slices = sorted(as_quat_slice_list(quat_slice))
        if not slices:
            raise ValueError("quat_slice must name at least one quaternion.")
        for start, end in slices:
            if end - start != _QUAT_DIM:
                raise ValueError(f"quat_slice must span 4 dims, got {(start, end)}.")
        for (_, previous_end), (next_start, _) in zip(slices, slices[1:]):
            if next_start < previous_end:
                raise ValueError(f"quat_slice pairs must not overlap, got {slices}.")
        self._quat_slices = slices
        state_space = env.single_observation_space["state"]
        new_state_space = spaces.Box(
            low=-np.inf,
            high=np.inf,
            shape=(state_space.shape[0] - len(slices),),
            dtype=state_space.dtype,
        )
        new_spaces = dict(env.single_observation_space.spaces)
        new_spaces["state"] = new_state_space
        self.single_observation_space = spaces.Dict(new_spaces)
        self.observation_space = batch_space(self.single_observation_space, env.num_envs)

    def __getattr__(self, name: str):
        # See RewardClassifierWrapper -- gymnasium.Wrapper (>=1.0) no longer
        # forwards arbitrary attributes to self.env, but this repo's env
        # backend contract (num_envs, single_observation_space, ...) relies
        # on direct attribute access.
        return getattr(self.env, name)

    def observation(self, obs: dict) -> dict:
        state = obs["state"]
        pieces = []
        position = 0
        for start, end in self._quat_slices:
            pieces.append(state[..., position:start])
            pieces.append(quat_xyzw_to_rotvec(state[..., start:end]))
            position = end
        pieces.append(state[..., position:])
        return {**obs, "state": torch.cat(pieces, dim=-1)}
