"""Read a second arm as a joint-space leader; never send motion commands."""
from __future__ import annotations

import inspect
import time
from collections.abc import Mapping
from typing import Any

import numpy as np

from real_world_rl.devices.arm import Arm
from real_world_rl.devices.gripper import Gripper
from real_world_rl.devices.teleop import RawReading, Teleop, TeleopConfig

_OPTION_KEYS = {"arm", "gripper", "intervention"}


@Teleop.register("arm_leader")
class ArmLeaderTeleop(Teleop):
    """Options: ``arm: {type: <backend>, ...constructor options}`` (required),
    ``gripper: false`` (default; true reads the matching backend's normalized width),
    ``intervention: always`` (default, active every poll) or ``event`` (inactive;
    the source's composite event device supplies the take-over toggle).

    Arm and gripper SDKs are loaded by their backends only when connecting. The host
    doctor's class-level SDK declaration cannot depend on these per-instance options.
    """

    ACTION_SPACE = "joint_abs"
    HAS_GRIPPER = False

    def __init__(self, config: TeleopConfig, **kwargs: Any) -> None:
        options = dict(config.options)
        unknown = options.keys() - _OPTION_KEYS
        if unknown:
            raise ValueError(f"unknown arm_leader options {sorted(unknown)}; known: {sorted(_OPTION_KEYS)}")
        arm_options = options.get("arm")
        if not isinstance(arm_options, Mapping) or not isinstance(arm_options.get("type"), str):
            raise ValueError("arm_leader needs options.arm: {type: <arm backend>, ...}")
        arm_options = dict(arm_options)
        backend = arm_options.pop("type")
        gripper = options.get("gripper", False)
        if not isinstance(gripper, bool):
            raise ValueError("arm_leader gripper must be true or false")
        self._intervention = options.get("intervention", "always")
        if self._intervention not in ("always", "event"):
            raise ValueError("arm_leader intervention must be 'always' or 'event'")
        self.arm = Arm.backend(backend)(**arm_options)
        self.gripper: Gripper | None = None
        if gripper:
            gripper_cls = Gripper.backend(backend)
            parameters = inspect.signature(gripper_cls).parameters
            # Independent gripper sessions reuse the arm's endpoint; integrated grippers
            # borrow the arm session. Only shared constructor options are forwarded.
            gripper_options = {
                key: value for key, value in arm_options.items()
                if key in parameters and key not in {"name", "owner"}
            }
            if not ({"robot_ip", "bridge_url"} & parameters.keys()):
                gripper_options["owner"] = self.arm
            self.gripper = gripper_cls(**gripper_options)
        self.NUM_JOINTS = self.arm.num_joints
        self.HAS_GRIPPER = gripper
        super().__init__(config, **kwargs)

    @classmethod
    def discover(cls) -> set[str]:
        return set()

    def _open_device(self) -> Arm:
        try:
            self.arm.connect()
            if self.gripper is not None:
                self.gripper.connect()
        except BaseException:
            self._close_device(self.arm)
            raise
        return self.arm

    def _close_device(self, handle: Arm) -> None:
        try:
            if self.gripper is not None:
                self.gripper.disconnect()
        finally:
            handle.disconnect()

    def _poll_device(self, handle: Arm) -> RawReading:
        # State reads can return immediately (notably fake/franky); bound the poll rate.
        time.sleep(0.01)
        state = handle.get_observation()  # checks connection, including for the fake arm
        width = None
        if self.gripper is not None:
            width = float(self.gripper.get_observation()["gripper_pos"][0])
            if not np.isfinite(width):
                raise ValueError("leader gripper width must be finite")
            width = float(np.clip(width, 0.0, 1.0))
        return RawReading(
            action=state["joint_pos"], gripper=width, buttons={}, events_now={},
            active=self._intervention == "always",
        )
