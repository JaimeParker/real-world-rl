"""Serializable task definitions; configuration imports need no device or torch SDK."""
from __future__ import annotations
from dataclasses import dataclass, field
import math


def finite(values, name):
    if not all(isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x) for x in values):
        raise ValueError(f"{name} must contain finite numbers")


@dataclass
class ResetConfig:
    home_joints: list[float] | None = None
    lift_m: float = 0.0
    waypoints: list[list[float]] = field(default_factory=list)
    gripper: str = "open"
    xyz_jitter_m: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    settle_s: float | None = None

    def __post_init__(self):
        finite([self.lift_m, *self.xyz_jitter_m], "reset")
        if self.lift_m < 0 or len(self.xyz_jitter_m) != 3 or any(x < 0 or x > 0.1 for x in self.xyz_jitter_m):
            raise ValueError("lift must be nonnegative and xyz_jitter_m must be three bounds in [0, 0.1] metres")
        if any(self.xyz_jitter_m) and not self.waypoints:
            raise ValueError("nonzero xyz_jitter_m requires reset waypoints")
        if self.gripper not in ("open", "close", "keep"):
            raise ValueError("reset gripper must be open, close or keep")
        if self.home_joints is not None:
            finite(self.home_joints, "home_joints")
        for pose in self.waypoints:
            finite(pose, "reset waypoint")
            if len(pose) != 7 or abs(sum(q*q for q in pose[3:]) - 1) > 0.002:
                raise ValueError("reset waypoints must be xyz + unit xyzw quaternion")
        if self.settle_s is not None and (not math.isfinite(self.settle_s) or self.settle_s < 0):
            raise ValueError("settle_s must be finite and nonnegative")


@dataclass
class TaskConfig:
    name: str = "none"
    version: int = 1
    reset: ResetConfig = field(default_factory=ResetConfig)
    arm_resets: dict = field(default_factory=dict)
    targets: dict = field(default_factory=dict)
    success_source: str = "manual"
    classifier_checkpoints: dict = field(default_factory=dict)
    classifier_image_keys: dict = field(default_factory=dict)
    position_tolerance_m: float = 0.03
    success_hold_steps: int = 1
    max_episode_steps: int | None = None
    terminate_on_success: bool = True
    terminate_on_failure: bool = True
    eval_episodes: int | None = None
    eval_seed: int | None = None
    success_reward: float = 1.0
    failure_reward: float = 0.0
    dense_distance_weight: float = 0.0

    def __post_init__(self):
        if isinstance(self.reset, dict): self.reset = ResetConfig(**self.reset)
        self.reset.__post_init__()
        self.arm_resets = {key: ResetConfig(**value) if isinstance(value, dict) else value
                           for key, value in self.arm_resets.items()}
        for reset in self.arm_resets.values(): reset.__post_init__()
        if self.version != 1:
            raise ValueError(f"unsupported task version {self.version}")
        if self.success_source not in ("manual", "classifier", "tcp_proxy"):
            raise ValueError("success_source must be manual, classifier or tcp_proxy")
        finite([self.position_tolerance_m, self.success_reward, self.failure_reward,
                self.dense_distance_weight], "task reward")
        for name in ("max_episode_steps", "eval_episodes"):
            value = getattr(self, name)
            if value is not None and (type(value) is not int or value <= 0):
                raise ValueError(f"{name} must be a positive integer or None")
        if self.eval_seed is not None and (type(self.eval_seed) is not int or self.eval_seed < 0):
            raise ValueError("eval_seed must be a nonnegative integer or None")
        if type(self.success_hold_steps) is not int or type(self.terminate_on_success) is not bool or type(self.terminate_on_failure) is not bool:
            raise ValueError("hold steps must be an integer and termination choices must be booleans")
        if self.position_tolerance_m <= 0 or self.success_hold_steps < 1 or self.dense_distance_weight < 0:
            raise ValueError("task tolerance and hold steps must be positive; dense weight nonnegative")
        for target in self.targets.values():
            finite(target, "task target")
            if len(target) != 3: raise ValueError("task targets must be xyz metres")
