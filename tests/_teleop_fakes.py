"""Fakes shared by the teleop tests: a minimal env (one arm, or two arms), a cartesian env with a
measured TCP pose, and mocked HID transports."""
from __future__ import annotations

import time
import types
from collections import deque

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


HOME_TCP = np.array([0.5, 0.0, 0.4, 1.0, 0.0, 0.0, 0.0])  # xyz + quat xyzw (TCP facing down)


class CartesianPoseEnv(ContractEnv):
    """A one-arm ``cartesian_delta`` env shaped like ``RealRobotEnv``: ``last_observation`` holds
    the TCP pose (``HOME_TCP`` after a reset), which lands on every commanded pose (base-frame
    ``tcp + action * xyz_scale_m``, rotation ``from_rotvec(action * rot_scale_rad)``
    left-multiplied). ``safety_box`` becomes the experiment's configured box."""

    def __init__(self, xyz_scale_m=0.05, rot_scale_rad=0.3, safety_box=None):
        super().__init__("cartesian_delta", 6)
        self.action_contract.update(xyz_scale_m=xyz_scale_m, rot_scale_rad=rot_scale_rad)
        if safety_box is not None:
            self.action_contract["experiment"] = {"safety_box": list(safety_box)}
        self.single_action_space = spaces.Box(-1.0, 1.0, (7,), dtype=np.float32)
        self.tcp = None

    @property
    def last_observation(self):
        return None if self.tcp is None else {"state": {"tcp_pose": self.tcp.copy()}}

    def reset(self, **kwargs):
        self.tcp = HOME_TCP.copy()
        return super().reset(**kwargs)

    def step(self, action):
        from scipy.spatial.transform import Rotation

        arm = torch.as_tensor(action).reshape(-1)[:6].double().numpy()
        contract = self.action_contract
        rotation = Rotation.from_rotvec(arm[3:] * contract["rot_scale_rad"]) * Rotation.from_quat(self.tcp[3:])
        self.tcp = np.concatenate([self.tcp[:3] + arm[:3] * contract["xyz_scale_m"], rotation.as_quat()])
        return super().step(action)


class TwoArmContractEnv(ContractEnv):
    """A two-arm ``ContractEnv``: actions are ``[left arm, right arm, left gripper, right gripper]``."""

    def __init__(self, action_mode="cartesian_delta", arm_action_dim=6):
        super().__init__(action_mode, arm_action_dim, gripper_in_action=True)
        self.action_contract.update(num_arms=2, arm_names=["left", "right"])
        self.single_action_space = spaces.Box(-1.0, 1.0, (2 * arm_action_dim + 2,), dtype=np.float32)


def make_hid() -> types.ModuleType:
    """trezor hidapi stand-in; twist uses decoded x/y/z/roll/pitch/yaw units."""
    class Hid(types.ModuleType):
        @property
        def buttons(self):
            return self._buttons

        @buttons.setter
        def buttons(self, buttons):
            buttons = tuple(buttons)
            if buttons == self._buttons:
                return
            self._buttons = buttons
            # HID queues each change, including a release followed by another
            # press before the next read. Axis reports never replace these.
            report = [3, sum(int(b) << i for i, b in enumerate(buttons))]
            for device in self.devices:
                if not device.closed:
                    device._button_reports.append(report)

    fake = Hid("hid")
    fake.twist = (0.0,) * 6
    fake._buttons = (0, 0)
    fake.devices = []
    fake.enumerate = lambda: [
        {"vendor_id": 0x256F, "product_id": 0xC62E, "path": str(index).encode()}
        for index in range(4)
    ]

    class Device:
        def __init__(self):
            self.closed = False
            self._button_reports = deque()
            if any(fake.buttons):
                self._button_reports.append([3, sum(int(b) << i for i, b in enumerate(fake.buttons))])
            fake.devices.append(self)

        def open_path(self, path):
            self.kwargs = {"device_index": int(path)}

        def set_nonblocking(self, enabled):
            assert enabled == 1

        def read(self, size):
            if self.closed:
                raise OSError("read from a closed SpaceMouse")
            time.sleep(0.001)
            if self._button_reports:
                return self._button_reports.popleft()
            x, y, z, roll, pitch, yaw = fake.twist
            # Wireless reports carry all axes on channel 1. Undo the table's
            # signs and put pitch before roll, exactly as the hardware does.
            report = [1]
            for value in (x, -y, -z, -pitch, -roll, yaw):
                word = round(value * 350) & 0xFFFF
                report.extend((word & 0xFF, word >> 8))
            return report

        def close(self):
            self.closed = True

    fake.device = Device
    return fake
