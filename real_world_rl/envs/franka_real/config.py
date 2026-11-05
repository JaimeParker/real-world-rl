"""Full env-factory config for the real-Franka env backend."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Literal, Optional


@dataclass
class FrankaRealEnvConfig:
    bridge_url: str
    device: str = "cpu"
    # (position, rotation) scale applied to the [-1, 1] action before it's
    # added as a delta to the current EE pose -- matches SERL's convention.
    action_scale: tuple[float, float] = (0.02, 0.1)
    gripper_threshold: float = 0.5
    max_episode_steps: int = 100
    reward_scale: float = 1.0
    reward_bias: float = 0.0
    # Cell-specific: physical workspace bounds the EE target position is
    # clipped to before being sent to the bridge.
    safety_box_low: tuple[float, float, float] = (0.0, -0.5, 0.0)
    safety_box_high: tuple[float, float, float] = (1.0, 0.5, 0.5)
    # Optional cell-specific orientation limits (xyz Euler radians) the EE
    # target orientation is clipped to. Both must be set together, or left
    # as None (default) to disable orientation clipping entirely -- matches
    # HIL-SERL's clip_safety_box(), which always clips orientation, but this
    # is opt-in here for backward compatibility with existing deployments
    # that only ever configured the position bounds above.
    safety_box_rpy_low: Optional[tuple[float, float, float]] = None
    safety_box_rpy_high: Optional[tuple[float, float, float]] = None
    camera_keys: tuple[str, ...] = field(default_factory=tuple)
    camera_height: int = 128
    camera_width: int = 128
    cartesian_semantics: Literal["current", "serl"] = "current"
    gripper_action_scale: float = 1.0
    gripper_sleep_s: float = 0.6
    control_hz: float = 10.0
    # The optional six-dimensional action layout applies to SERL only.
    gripper_in_action: bool = True

    def __post_init__(self) -> None:
        if self.cartesian_semantics not in ("current", "serl"):
            raise ValueError("cartesian_semantics must be 'current' or 'serl'")
        if self.cartesian_semantics == "serl":
            from real_world_rl.envs.serl_cartesian import validate_serl_config

            validate_serl_config(
                self.safety_box_low, self.safety_box_high,
                self.safety_box_rpy_low, self.safety_box_rpy_high,
                gripper_action_scale=self.gripper_action_scale,
                gripper_sleep_s=self.gripper_sleep_s, control_hz=self.control_hz,
            )
            from real_world_rl.tasks.config import finite

            finite(self.action_scale, "action_scale")
            if len(self.action_scale) != 2 or min(self.action_scale) < 0:
                raise ValueError("action_scale must contain two nonnegative scales")
