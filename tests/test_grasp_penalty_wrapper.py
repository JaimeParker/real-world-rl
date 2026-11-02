"""Tests for GraspPenaltyWrapper."""
from __future__ import annotations

import gymnasium as gym
import pytest
import torch

from rl_garden_real_world.envs.wrappers.grasp_penalty import GraspPenaltyWrapper


class _FakeEnv(gym.Env):
    num_envs = 1

    def reset(self, **kwargs):
        return torch.zeros(1, 4), {}

    def step(self, action):
        del action
        return (
            torch.ones(1, 4),
            torch.tensor([1.0]),
            torch.tensor([False]),
            torch.tensor([False]),
            {},
        )


def _gripper_action(value: float) -> torch.Tensor:
    action = torch.zeros(1, 7)
    action[..., -1] = value
    return action


def test_no_penalty_on_first_step_after_reset():
    wrapped = GraspPenaltyWrapper(_FakeEnv())
    wrapped.reset()
    _, _, _, _, info = wrapped.step(_gripper_action(-1.0))
    assert info["grasp_penalty"].item() == 0.0


def test_penalty_fires_exactly_on_gripper_flip():
    wrapped = GraspPenaltyWrapper(_FakeEnv(), penalty=-0.05)
    wrapped.reset()
    wrapped.step(_gripper_action(-1.0))  # open

    _, _, _, _, info_same = wrapped.step(_gripper_action(-1.0))  # still open
    assert info_same["grasp_penalty"].item() == 0.0

    _, _, _, _, info_flip = wrapped.step(_gripper_action(1.0))  # now closed
    assert info_flip["grasp_penalty"].item() == pytest.approx(-0.05)

    _, _, _, _, info_flip_back = wrapped.step(_gripper_action(-1.0))  # open again
    assert info_flip_back["grasp_penalty"].item() == pytest.approx(-0.05)


def test_honors_intervene_action_over_policy_action():
    class _InterveningEnv(_FakeEnv):
        def step(self, action):
            del action
            return (
                torch.ones(1, 4),
                torch.tensor([1.0]),
                torch.tensor([False]),
                torch.tensor([False]),
                {"intervene_action": _gripper_action(1.0)},
            )

    wrapped = GraspPenaltyWrapper(_InterveningEnv())
    wrapped.reset()
    wrapped.step(_gripper_action(-1.0))  # opens per intervene_action, not the -1.0 policy action

    _, _, _, _, info = wrapped.step(_gripper_action(-1.0))
    # Both steps' executed action (via intervene_action) is 1.0 -> closed,
    # never flips.
    assert info["grasp_penalty"].item() == 0.0


def test_getattr_forwards_to_wrapped_env():
    env = _FakeEnv()
    wrapped = GraspPenaltyWrapper(env)
    assert wrapped.num_envs == 1


# -- several grippers (gripper_slice) ------------------------------------------------------


def _two_gripper_action(left: float, right: float) -> torch.Tensor:
    action = torch.zeros(1, 6)  # two 2-D arm blocks, then the left and right grippers
    action[..., -2:] = torch.tensor([left, right])
    return action


def test_two_grippers_pay_the_penalty_once_per_flipped_gripper():
    wrapped = GraspPenaltyWrapper(_FakeEnv(), penalty=-0.05, gripper_slice=slice(4, 6))
    wrapped.reset()
    _, _, _, _, first = wrapped.step(_two_gripper_action(-1.0, -1.0))
    _, _, _, _, same = wrapped.step(_two_gripper_action(-1.0, -1.0))
    _, _, _, _, one_flip = wrapped.step(_two_gripper_action(1.0, -1.0))
    _, _, _, _, two_flips = wrapped.step(_two_gripper_action(-1.0, 1.0))
    assert first["grasp_penalty"].item() == 0.0 and same["grasp_penalty"].item() == 0.0
    assert one_flip["grasp_penalty"].item() == pytest.approx(-0.05)
    assert two_flips["grasp_penalty"].item() == pytest.approx(-0.10)
    assert two_flips["grasp_penalty"].shape == (1,)


def test_arm_dims_are_ignored_with_a_gripper_slice():
    wrapped = GraspPenaltyWrapper(_FakeEnv(), penalty=-0.05, gripper_slice=slice(4, 6))
    wrapped.reset()
    wrapped.step(_two_gripper_action(-1.0, -1.0))
    arm_moved = _two_gripper_action(-1.0, -1.0)
    arm_moved[..., :4] = 1.0
    _, _, _, _, info = wrapped.step(arm_moved)
    assert info["grasp_penalty"].item() == 0.0


def test_a_one_gripper_slice_matches_the_default_last_dim():
    by_index, by_slice = GraspPenaltyWrapper(_FakeEnv()), GraspPenaltyWrapper(_FakeEnv(), gripper_slice=slice(6, 7))
    for wrapped in (by_index, by_slice):
        wrapped.reset()
    for value in (-1.0, 1.0, 1.0, -1.0):
        expected = by_index.step(_gripper_action(value))[4]["grasp_penalty"]
        torch.testing.assert_close(by_slice.step(_gripper_action(value))[4]["grasp_penalty"], expected)
