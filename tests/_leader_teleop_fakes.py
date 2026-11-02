"""Fakes shared by the leader-arm, GELLO, keyboard and composite teleop tests: a joint-space env
with a settable follower, connected fake leader parts, a ``gello`` backend stand-in and a mocked
``pynput`` SDK."""
from __future__ import annotations

import dataclasses
import enum
import types

import numpy as np
from _teleop_fakes import ContractEnv
from robot_infra.devices.backends.teleops.fake import FakeTeleop
from robot_infra.devices.teleop import TeleopConfig

JOINT_LIMITS = [[-2.0, 2.0]] * 7
JOINT_SCALE_RAD = 0.05


class JointEnv(ContractEnv):
    """A ``ContractEnv`` in a joint action mode whose ``last_observation`` reports ``follower``
    as ``state["joint_pos"]`` (as ``RealRobotEnv`` does). ``robot`` stands in for the env's robot
    (``TeleopPartSource.for_env`` reads ``robot.teleop``)."""

    def __init__(
        self,
        action_mode="joint_abs",
        num_joints=7,
        gripper_in_action=True,
        state_fields=("joint_pos", "gripper_pos"),
        robot=None,
    ):
        super().__init__(action_mode, num_joints, gripper_in_action)
        self.action_contract.update(
            num_joints=num_joints,
            joint_limits=JOINT_LIMITS[:num_joints],
            joint_scale_rad=JOINT_SCALE_RAD,
            state_fields=tuple(state_fields),
        )
        self.follower = np.zeros(num_joints)
        self.robot = robot

    @property
    def last_observation(self):
        return {"state": {"joint_pos": np.asarray(self.follower, dtype=np.float64)}}


def normalized_joints(q) -> np.ndarray:
    """``joint_abs`` action for joint targets ``q`` (onto ``JOINT_LIMITS``)."""
    low, high = np.array(JOINT_LIMITS[: len(q)]).T
    return 2.0 * (np.asarray(q, dtype=np.float64) - low) / (high - low) - 1.0


def executed_joints(action) -> np.ndarray:
    """Joint targets ``RealRobotEnv`` commands for a ``joint_abs`` arm action."""
    action = np.asarray(action, dtype=np.float64)
    low, high = np.array(JOINT_LIMITS[: len(action)]).T
    return low + (action + 1.0) / 2.0 * (high - low)


def connected_joint_part(script, *, num_joints=7, has_gripper=True, action_space="joint_abs", **options):
    """A connected fake leader part playing ``script`` (``FakeTeleop`` steps), one step per read."""
    part = FakeTeleop(
        TeleopConfig(
            name="leader",
            backend="fake",
            options={
                "action_space": action_space,
                "num_joints": num_joints,
                "has_gripper": has_gripper,
                "script": script,
                **options,
            },
        )
    )
    part.connect()
    return part


class FakeGelloTeleop(FakeTeleop):
    """``gello`` backend stand-in: keeps the device config it was built with (``gello_config``) and
    plays the class-level ``script`` as a ``joint_abs`` leader with ``len(joint_ids)`` joints and a
    gripper when ``gripper_id`` is set. Subclass per test so ``script``/``instances`` stay local."""

    script: list = []
    instances: list = []

    def __init__(self, config: TeleopConfig, **kwargs) -> None:
        self.gello_config = config
        type(self).instances.append(self)
        fake_options = {
            "action_space": "joint_abs",
            "num_joints": len(config.options["joint_ids"]),
            "has_gripper": config.options.get("gripper_id") is not None,
            "script": list(type(self).script),
        }
        super().__init__(dataclasses.replace(config, options=fake_options), **kwargs)


def make_pynput() -> types.ModuleType:
    """pynput stand-in: ``fake.keyboard.listeners[-1].press(key)``/``.release(key)`` drive the
    callbacks; ``Key`` has ``space``/``esc``/``shift``, ``KeyCode(char)`` is a character key."""
    fake = types.ModuleType("pynput")
    keyboard = types.ModuleType("pynput.keyboard")

    class Key(enum.Enum):
        space = "space"
        esc = "esc"
        shift = "shift"

    class KeyCode:
        def __init__(self, char):
            self.char = char

    class Listener:
        def __init__(self, on_press=None, on_release=None):
            self.on_press = on_press
            self.on_release = on_release
            self.alive = False
            self.stop_calls = 0
            keyboard.listeners.append(self)

        def start(self):
            self.alive = True

        def stop(self):
            self.alive = False
            self.stop_calls += 1

        def is_alive(self):
            return self.alive

        def press(self, key):
            self.on_press(key)

        def release(self, key):
            self.on_release(key)

        def tap(self, key):
            self.press(key)
            self.release(key)

    keyboard.Key = Key
    keyboard.KeyCode = KeyCode
    keyboard.Listener = Listener
    keyboard.listeners = []
    fake.keyboard = keyboard
    return fake
