"""Config for the two-arm real-robot env: one ``RealRobotEnvConfig`` per arm.

Pure dataclass (no device-layer/scipy/torch imports), nestable in CLI args like
``RealRobotEnvConfig``: ``--real_robot_bimanual.left.remote HOST:PORT``.
"""
from __future__ import annotations

import dataclasses
from dataclasses import dataclass, field
from typing import Optional

from real_world_rl.envs.real_robot.config import RealRobotEnvConfig
from real_world_rl.tasks.config import TaskConfig

# Per-arm config fields that must be equal on both arms: they define one shared action
# contract (layout and scaling) and one observation layout.
SHARED_ARM_FIELDS = (
    "action_mode",
    "num_joints",
    "control_hz",
    "gripper_in_action",
    "xyz_scale_m",
    "rot_scale_rad",
    "joint_scale_rad",
    "state_fields",
    "image_size",
)
_NUM_ARMS = 2


@dataclass
class BimanualRealRobotEnvConfig:
    """Two arms driven as one env.

    ``left``/``right`` are the arms' own configs, named ``arm_names[0]``/``arm_names[1]`` in
    observation keys and the action contract. Their ``SHARED_ARM_FIELDS`` must agree; their
    connection (``robot``/``remote``/``token``), ``safety_box``, ``joint_limits``,
    ``home_joints`` and ``cameras`` are per arm. Camera names must be unique across arms.
    ``joint_limits`` is set on both arms or on neither (the action contract carries them per
    arm). ``max_episode_steps`` and ``reset_settle_s`` here apply to both arms; the per-arm
    ``max_episode_steps`` must stay unset and the per-arm ``reset_settle_s`` is replaced.
    """

    left: RealRobotEnvConfig = field(default_factory=RealRobotEnvConfig)
    right: RealRobotEnvConfig = field(default_factory=RealRobotEnvConfig)
    arm_names: tuple[str, str] = ("left", "right")
    max_episode_steps: Optional[int] = None
    reset_settle_s: float = 0.5
    task: TaskConfig = field(default_factory=TaskConfig)

    def __post_init__(self) -> None:
        from real_world_rl.tasks.config import finite
        finite([self.reset_settle_s], "reset_settle_s")
        if isinstance(self.task, dict): self.task = TaskConfig(**self.task)
        if isinstance(self.left, dict): self.left = RealRobotEnvConfig(**self.left)
        if isinstance(self.right, dict): self.right = RealRobotEnvConfig(**self.right)
        if self.left.task.name != "none" or self.right.task.name != "none":
            raise ValueError("set task on the bimanual config, leaving per-arm task=none")
        self.arm_names = tuple(self.arm_names)
        if len(self.arm_names) != _NUM_ARMS or len(set(self.arm_names)) != _NUM_ARMS:
            raise ValueError(f"arm_names must be two distinct names, got {list(self.arm_names)}")
        if not all(isinstance(name, str) and name for name in self.arm_names):
            raise ValueError(f"arm_names must be non-empty strings, got {list(self.arm_names)}")
        if self.max_episode_steps is not None and (type(self.max_episode_steps) is not int or self.max_episode_steps <= 0):
            raise ValueError(
                f"max_episode_steps must be positive or None, got {self.max_episode_steps}"
            )
        if self.reset_settle_s < 0:
            raise ValueError(f"reset_settle_s must be >= 0, got {self.reset_settle_s}")
        self.check_arms_consistent()

    @property
    def arm_configs(self) -> dict[str, RealRobotEnvConfig]:
        """Each arm's own config, in ``arm_names`` order."""
        return dict(zip(self.arm_names, (self.left, self.right)))

    @property
    def cameras(self) -> list[str]:
        """Every arm's cameras, in arm order: the env's camera list."""
        return [cam for config in self.arm_configs.values() for cam in config.cameras]

    @property
    def depth_cameras(self):
        return [camera for config in self.arm_configs.values() for camera in config.depth_cameras]

    def arm_env_config(self, arm: str) -> RealRobotEnvConfig:
        """The config the arm's ``RealRobotEnv`` is built from: truncation is the two-arm env's
        job, and both arms settle for the shared ``reset_settle_s``."""
        return dataclasses.replace(
            self.arm_configs[arm], max_episode_steps=None, reset_settle_s=self.reset_settle_s
        )

    def check_arms_consistent(self) -> None:
        """Raise ``ValueError`` unless the arms share ``SHARED_ARM_FIELDS``, have disjoint camera
        names, both or neither set ``joint_limits`` and leave ``max_episode_steps`` unset."""
        left_name, right_name = self.arm_names
        mismatched = [
            f"{name}: {left_name}={getattr(self.left, name)!r}, {right_name}={getattr(self.right, name)!r}"
            for name in SHARED_ARM_FIELDS
            if getattr(self.left, name) != getattr(self.right, name)
        ]
        if mismatched:
            raise ValueError(
                "both arms must share these fields (set them equal on both arms):\n  "
                + "\n  ".join(mismatched)
            )
        shared_cameras = sorted(set(self.left.capture_cameras) & set(self.right.capture_cameras))
        if shared_cameras:
            raise ValueError(
                f"camera names must be unique across arms; both arms name {shared_cameras}"
            )
        if (self.left.joint_limits is None) != (self.right.joint_limits is None):
            raise ValueError(
                "set joint_limits on both arms or on neither (the action contract carries them "
                "per arm)"
            )
        per_arm_caps = [
            name for name, config in self.arm_configs.items() if config.max_episode_steps is not None
        ]
        if per_arm_caps:
            raise ValueError(
                f"max_episode_steps is set on arm(s) {per_arm_caps}; set the two-arm env's "
                "max_episode_steps instead (it truncates both arms together)"
            )

    def require_robot_source(self) -> None:
        for arm, config in self.arm_configs.items():
            try:
                config.require_robot_source()
            except ValueError as error:
                raise ValueError(f"arm {arm!r}: {error}") from None
