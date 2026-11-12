"""Two fake Franka arms (device-layer fake backends) for the two-arm env tests."""
from __future__ import annotations

from typing import Any

import real_world_rl.devices.backends  # noqa: F401  (registers the fake backends)
from real_world_rl.devices import CameraConfig, GripperConfig
from real_world_rl.devices.robots.franka import FrankaConfig, FrankaRobot

from real_world_rl.envs.bimanual import BimanualRealRobotEnvConfig
from real_world_rl.envs.real_robot import RealRobotEnvConfig

ARMS = ("left", "right")
ZERO_HOME = [0.0] * 7
FRAME_HW = (24, 32)
# Per arm: tcp_pose 7 + tcp_vel 6 + joint_pos 7 + joint_vel 7 + gripper_pos 1 + force 3 + torque 3.
ARM_STATE_DIM = 34


def camera_name(arm: str) -> str:
    return f"{arm}_wrist"


def fake_robot_dict(arm: str) -> dict[str, Any]:
    """A local device-layer RobotConfig dict for ``arm``: a fake Franka with a gripper and a camera.

    Each arm gets its own robot and camera names, as two real arms would."""
    return {
        "type": "franka",
        "name": f"{arm}_franka",
        "arm_backend": "fake",
        "gripper": {"backend": "fake"},
        "cameras": [
            {"name": camera_name(arm), "backend": "fake", "width": FRAME_HW[1], "height": FRAME_HW[0], "fps": 60}
        ],
    }


def fake_franka(arm: str, **robot_overrides) -> FrankaRobot:
    return FrankaRobot(
        FrankaConfig(
            name=f"{arm}_franka",
            arm_backend="fake",
            gripper=GripperConfig(backend="fake"),
            cameras=[
                CameraConfig(name=camera_name(arm), backend="fake", width=FRAME_HW[1], height=FRAME_HW[0], fps=60)
            ],
            **robot_overrides,
        )
    )


def arm_config(arm: str, **overrides) -> RealRobotEnvConfig:
    values = dict(robot=fake_robot_dict(arm), cameras=[camera_name(arm)], home_joints=ZERO_HOME)
    values.update(overrides)
    return RealRobotEnvConfig(**values)


def bimanual_config(*, shared: dict | None = None, left: dict | None = None, right: dict | None = None, **top) -> BimanualRealRobotEnvConfig:
    """Both arms with ``shared`` overrides, plus per-arm ``left``/``right`` overrides."""
    shared = shared or {}
    top = {"reset_settle_s": 0.0, **top}
    return BimanualRealRobotEnvConfig(
        left=arm_config("left", **{**shared, **(left or {})}),
        right=arm_config("right", **{**shared, **(right or {})}),
        **top,
    )
