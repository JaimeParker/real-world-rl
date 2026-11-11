"""Config for the generic robot_infra-backed real-robot env.

Pure dataclass (no robot_infra/scipy/torch imports) so it can be nested in CLI
args and inspected without any robot dependency installed.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Literal, Optional

from real_world_rl.tasks.config import TaskConfig
from real_world_rl.envs.real_robot.preprocess import CameraPreprocessConfig

ACTION_MODES = ("cartesian_delta", "joint_delta", "joint_abs")
STATE_FIELDS = (
    "tcp_pose",
    "tcp_vel",
    "joint_pos",
    "joint_vel",
    "gripper_pos",
    "tcp_force",
    "tcp_torque",
)
_SAFETY_BOX_LENGTH = 6


@dataclass
class RealRobotEnvConfig:
    # Exactly one of: a local robot_infra RobotConfig dict (must carry "type"),
    # or the address of a robot_infra daemon. Checked by ``require_robot_source``
    # when the env is built, so the default-constructed config (CLI group) stays valid.
    robot: Optional[dict] = None
    remote: Optional[str] = None
    token: Optional[str] = None
    # Expected daemon geometry; local robots inherit these from their robot dict.
    task_frame: Optional[dict] = None
    workspace: Optional[dict] = None
    num_joints: int = 7
    # Concatenated in this order into the flat backend ``state`` vector.
    state_fields: tuple[str, ...] = STATE_FIELDS
    cameras: list[str] = field(default_factory=list)
    # (H, W) every camera frame is resized to; None keeps the camera's native size.
    image_size: Optional[tuple[int, int]] = None
    depth_cameras: list[str] = field(default_factory=list)
    preprocess: dict = field(default_factory=dict)
    task: TaskConfig = field(default_factory=TaskConfig)
    control: Optional[dict] = None
    action_mode: Literal["cartesian_delta", "joint_delta", "joint_abs"] = "cartesian_delta"
    control_hz: float = 10.0
    xyz_scale_m: float = 0.02
    rot_scale_rad: float = 0.1
    joint_scale_rad: float = 0.05
    # The source of truth for limits is the robot yaml on the robot PC (enforced by the
    # device layer). The three fields below are optional and may only be tighter than it:
    # unset ones are inherited after connect, looser ones fail the first reset.
    # [xmin, ymin, zmin, xmax, ymax, zmax]; clips the cartesian target position.
    safety_box: Optional[list[float]] = None
    # One [low, high] pair per joint; clips joint targets. joint_abs still needs it set here
    # because the action mapping is built at construction, before any connection.
    joint_limits: Optional[list[list[float]]] = None
    # Reset pose; falls back to the robot's own home_joints when unset.
    home_joints: Optional[list[float]] = None
    gripper_in_action: bool = True
    max_episode_steps: Optional[int] = None
    reset_settle_s: float = 0.5
    # In-process step watchdog (robot_infra.supervise.StepWatchdog): after a reset, a gap of more
    # than this many seconds between env steps calls arm.stop() once and the next step raises.
    # None = off. Must exceed one control period. Not part of the experiment contract.
    watchdog_s: Optional[float] = None
    cartesian_semantics: Literal["current", "serl"] = "current"
    safety_box_rpy_low: Optional[list[float]] = None
    safety_box_rpy_high: Optional[list[float]] = None
    gripper_action_scale: float = 1.0
    gripper_sleep_s: float = 0.6

    def __post_init__(self) -> None:
        if self.cartesian_semantics not in ("current", "serl"):
            raise ValueError("cartesian_semantics must be 'current' or 'serl'")
        if self.cartesian_semantics == "serl":
            from real_world_rl.envs.serl_cartesian import validate_serl_config

            if self.action_mode != "cartesian_delta":
                raise ValueError("serl cartesian_semantics requires action_mode='cartesian_delta'")
            validate_serl_config(
                self.safety_box[:3] if self.safety_box is not None else None,
                self.safety_box[3:] if self.safety_box is not None else None,
                self.safety_box_rpy_low, self.safety_box_rpy_high,
                gripper_action_scale=self.gripper_action_scale,
                gripper_sleep_s=self.gripper_sleep_s, control_hz=self.control_hz,
            )
        from real_world_rl.tasks.config import finite
        finite([self.control_hz, self.xyz_scale_m, self.rot_scale_rad,
                self.joint_scale_rad, self.reset_settle_s], "control frequency/scales/reset")
        if min(self.xyz_scale_m, self.rot_scale_rad, self.joint_scale_rad) < 0:
            raise ValueError("action scales must be nonnegative")
        if self.reset_settle_s < 0:
            raise ValueError("reset_settle_s must be nonnegative")
        if isinstance(self.task, dict): self.task = TaskConfig(**self.task)
        self.depth_cameras = list(self.depth_cameras)
        self.preprocess = {key: CameraPreprocessConfig(**value) if isinstance(value, dict) else value
                           for key, value in self.preprocess.items()}
        for preprocessing in self.preprocess.values(): preprocessing.__post_init__()
        if self.control is not None and not isinstance(self.control, dict):
            raise ValueError("control must be a mapping or None")
        if len(set(self.cameras)) != len(self.cameras) or len(set(self.depth_cameras)) != len(self.depth_cameras):
            raise ValueError("camera names must be unique within each modality")
        if self.capture_cameras and set(self.preprocess) - set(self.capture_cameras):
            raise ValueError("preprocess names must refer to configured RGB/depth cameras")
        self.state_fields = tuple(self.state_fields)
        self.cameras = list(self.cameras)
        if self.image_size is not None:
            self.image_size = tuple(self.image_size)
        if self.action_mode not in ACTION_MODES:
            raise ValueError(
                f"action_mode {self.action_mode!r} invalid; choose from {list(ACTION_MODES)}"
            )
        if type(self.num_joints) is not int or self.num_joints <= 0:
            raise ValueError(f"num_joints must be positive, got {self.num_joints}")
        if self.joint_limits is not None:
            for pair in self.joint_limits:
                finite(pair, "joint_limits")
            shape_ok = len(self.joint_limits) == self.num_joints and all(
                len(pair) == 2 for pair in self.joint_limits
            )
            if not shape_ok:
                raise ValueError(
                    f"joint_limits must have shape ({self.num_joints}, 2) "
                    "([[low, high], ...] per joint)"
                )
            if any(low > high for low, high in self.joint_limits):
                raise ValueError("joint_limits entries must satisfy low <= high")
        if self.action_mode == "joint_abs" and self.joint_limits is None:
            raise ValueError(
                "action_mode 'joint_abs' maps [-1, 1] onto joint_limits; set joint_limits"
            )
        if self.safety_box is not None and len(self.safety_box) != _SAFETY_BOX_LENGTH:
            raise ValueError(
                "safety_box must be [xmin, ymin, zmin, xmax, ymax, zmax], "
                f"got {len(self.safety_box)} entries"
            )
        if self.safety_box is not None:
            finite(self.safety_box, "safety_box")
            if any(low > high for low, high in zip(self.safety_box[:3], self.safety_box[3:])):
                raise ValueError("safety_box entries must satisfy low <= high")
        unknown = [f for f in self.state_fields if f not in STATE_FIELDS]
        if unknown:
            raise ValueError(f"unknown state_fields {unknown}; choose from {list(STATE_FIELDS)}")
        if not self.state_fields:
            raise ValueError("state_fields must not be empty")
        if self.home_joints is not None and len(self.home_joints) != self.num_joints:
            raise ValueError(
                f"home_joints must have {self.num_joints} entries, got {len(self.home_joints)}"
            )
        if self.home_joints is not None:
            finite(self.home_joints, "home_joints")
        if self.image_size is not None and (
            len(self.image_size) != 2 or any(type(value) is not int or value <= 0 for value in self.image_size)
        ):
            raise ValueError(f"image_size must be a positive (H, W), got {self.image_size}")
        if self.remote is not None and self.capture_cameras and self.image_size is None:
            raise ValueError(
                "remote robots report camera sizes only after connect; set image_size "
                "so the observation space is known at construction"
            )
        if self.control_hz <= 0:
            raise ValueError(f"control_hz must be positive, got {self.control_hz}")
        if self.watchdog_s is not None:
            finite([self.watchdog_s], "watchdog_s")
            if self.watchdog_s <= 1.0 / self.control_hz:
                raise ValueError(
                    f"watchdog_s ({self.watchdog_s}) must exceed one control period "
                    f"(1/control_hz = {1.0 / self.control_hz:.4g} s), or be None to disable the watchdog"
                )
        if self.max_episode_steps is not None and (type(self.max_episode_steps) is not int or self.max_episode_steps <= 0):
            raise ValueError(
                f"max_episode_steps must be positive or None, got {self.max_episode_steps}"
            )

    @property
    def capture_cameras(self):
        return list(dict.fromkeys([*self.cameras, *self.depth_cameras]))

    def require_robot_source(self) -> None:
        if (self.robot is None) == (self.remote is None):
            raise ValueError(
                "set exactly one of `robot` (local RobotConfig dict) or `remote` "
                "(robot_infra daemon address)"
            )
