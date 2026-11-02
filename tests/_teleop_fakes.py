"""Fakes shared by the teleop tests: a minimal env (one arm, or two arms) and a mocked
``pyspacemouse`` SDK."""
from __future__ import annotations

import time
import types

import gymnasium as gym
import numpy as np
import torch
from gymnasium import spaces


class FakeEnv(gym.Env):
    """Records the actions it is stepped with and how often it is reset/closed."""

    num_envs = 1

    def __init__(self, call_order=None):
        self.reset_calls = 0
        self.close_calls = 0
        self.step_actions = []
        self._call_order = call_order

    def reset(self, **kwargs):
        self.reset_calls += 1
        if self._call_order is not None:
            self._call_order.append("env.reset")
        return torch.zeros(1, 4), {}

    def step(self, action):
        self.step_actions.append(action)
        return torch.ones(1, 4), torch.tensor([0.0]), torch.tensor([False]), torch.tensor([False]), {}

    def close(self):
        self.close_calls += 1
        if self._call_order is not None:
            self._call_order.append("env.close")


class ContractEnv(FakeEnv):
    """A ``FakeEnv`` that exposes an action contract (as ``RealRobotEnv`` does)."""

    def __init__(self, action_mode="cartesian_delta", arm_action_dim=6, gripper_in_action=True):
        super().__init__()
        self.action_contract = {
            "action_mode": action_mode,
            "arm_action_dim": arm_action_dim,
            "gripper_in_action": gripper_in_action,
        }


class TwoArmContractEnv(ContractEnv):
    """A two-arm ``ContractEnv``: actions are ``[left arm, right arm, left gripper, right gripper]``."""

    def __init__(self, action_mode="cartesian_delta", arm_action_dim=6):
        super().__init__(action_mode, arm_action_dim, gripper_in_action=True)
        self.action_contract.update(num_arms=2, arm_names=["left", "right"])
        self.single_action_space = spaces.Box(-1.0, 1.0, (2 * arm_action_dim + 2,), dtype=np.float32)


def make_pyspacemouse() -> types.ModuleType:
    """pyspacemouse 2.x stand-in: set ``fake.twist`` / ``fake.buttons`` to drive the device."""
    fake = types.ModuleType("pyspacemouse")
    fake.twist = (0.0,) * 6
    fake.buttons = (0, 0)
    fake.devices = []

    class Device:
        def __init__(self):
            self.closed = False

        def read(self):
            if self.closed:
                raise OSError("read from a closed SpaceMouse")
            time.sleep(0.001)
            x, y, z, roll, pitch, yaw = fake.twist
            return types.SimpleNamespace(
                x=x, y=y, z=z, roll=roll, pitch=pitch, yaw=yaw, buttons=list(fake.buttons)
            )

        def close(self):
            self.closed = True

    def open_device(**kwargs):
        device = Device()
        device.kwargs = kwargs
        fake.devices.append(device)
        return device

    fake.open = open_device
    return fake
