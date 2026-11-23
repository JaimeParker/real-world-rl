"""Robot composition: a typed config plus a PartGroup of arm, gripper, cameras and an optional teleop."""
from __future__ import annotations

import dataclasses
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, ClassVar, Mapping

import yaml

from real_world_rl.devices.arm import Arm
from real_world_rl.devices.base import PartGroup
from real_world_rl.devices.camera import Camera, CameraConfig
from real_world_rl.devices.gripper import Gripper, GripperMode
from real_world_rl.devices.teleop import Teleop, TeleopConfig

# The Franka Hand's maximum grasp force (HIL-SERL grasps with it).
MAX_GRASP_FORCE_N = 130.0


def _check_known_keys(config_cls: type, data: Mapping[str, Any], known: set[str]) -> None:
    for key in data:
        if key not in known:
            raise ValueError(f"unknown key '{key}' for {config_cls.__name__}; known: {sorted(known)}")


@dataclass
class GripperConfig:
    backend: str = "fake"
    max_width_m: float = 0.08
    mode: str = "binary"
    binary_threshold: float = 0.5
    shares_arm_session: bool = False
    # Franka Hand (franky backend) commands; other backends ignore them. None takes the backend
    # default, which for franky is HIL-SERL's franka_gripper_server: grasp to 0.01 m at 0.3 m/s
    # with 130 N, open fully (franky open = move to the hand's maximum width) at 0.3 m/s.
    speed_m_s: float | None = None
    grasp_force_n: float | None = None
    close_width_m: float | None = None
    open_width_m: float | None = None  # None: fully open

    def __post_init__(self) -> None:
        import math
        for name in ("speed_m_s", "grasp_force_n", "close_width_m", "open_width_m"):
            value = getattr(self, name)
            if value is not None and (isinstance(value, bool) or not isinstance(value, (int, float))
                                      or not math.isfinite(value)):
                raise ValueError(f"gripper {name} must be a finite number or null, got {value!r}")
        if self.speed_m_s is not None and self.speed_m_s <= 0:
            raise ValueError(f"gripper speed_m_s must be > 0, got {self.speed_m_s}")
        if self.grasp_force_n is not None and not 0 < self.grasp_force_n <= MAX_GRASP_FORCE_N:
            raise ValueError(f"gripper grasp_force_n must be in (0, {MAX_GRASP_FORCE_N:g}] N "
                             f"(the Franka Hand's maximum), got {self.grasp_force_n}")
        for name in ("close_width_m", "open_width_m"):
            value = getattr(self, name)
            if value is not None and not 0 <= value <= self.max_width_m:
                raise ValueError(f"gripper {name} must be in [0, max_width_m {self.max_width_m}], got {value}")

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> "GripperConfig":
        _check_known_keys(cls, data, {f.name for f in dataclasses.fields(cls)})
        return cls(**data)


@dataclass
class RobotConfig:
    """Base config. Subclasses set the ClassVars and add robot-specific fields."""

    TYPE: ClassVar[str] = ""
    NUM_JOINTS: ClassVar[int] = 0
    name: str = "robot"
    arm_backend: str = "fake"
    gripper: GripperConfig | None = field(default_factory=GripperConfig)
    cameras: list[CameraConfig] = field(default_factory=list)
    home_joints: list[float] | None = None
    limits: dict | None = None
    control: dict | None = None
    claim: bool = True
    teleop: TeleopConfig | None = None
    task_frame: dict | None = None

    def __post_init__(self) -> None:
        from real_world_rl.devices.control import ArmControlConfig
        from real_world_rl.devices.limits import ArmLimits, _array
        from real_world_rl.devices.workspace import task_transform
        task_transform(self.task_frame)
        if self.home_joints is not None:
            _array(self.home_joints, (self.NUM_JOINTS,), "home_joints")
        ArmLimits.from_config(self)
        if self.control is not None:
            self.control = ArmControlConfig.from_dict(self.control).to_dict()
        if self.gripper is not None:
            modes = [m.value for m in GripperMode]
            if self.gripper.mode not in modes:
                raise ValueError(f"gripper mode '{self.gripper.mode}' invalid; choose from {modes}")

    @property
    def joint_limits(self) -> list[list[float]] | None:
        value = (self.limits or {}).get("joints")
        return None if value is None else [list(row) for row in value]

    def kinematics(self):
        return None

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> "RobotConfig":
        for key in ("safety_box", "workspace", "joint_limits", "max_delta_per_step"):
            if key in data:
                raise ValueError(
                    f"top-level {key} was removed; use limits: instead; run "
                    "`rwrl limits migrate <yaml>`; see docs: "
                    "https://real-world-rl.readthedocs.io/en/latest/en/reference/devices.html"
                )
        values = {key: value for key, value in data.items() if key != "type"}
        _check_known_keys(cls, values, {f.name for f in dataclasses.fields(cls)})
        if "gripper" in values and values["gripper"] is not None:
            values["gripper"] = GripperConfig.from_dict(values["gripper"])
        if "cameras" in values:
            camera_keys = {f.name for f in dataclasses.fields(CameraConfig)}
            for camera in values["cameras"]:
                _check_known_keys(CameraConfig, camera, camera_keys)
            values["cameras"] = [CameraConfig(**camera) for camera in values["cameras"]]
        if values.get("teleop") is not None:
            values["teleop"] = TeleopConfig.from_dict(values["teleop"])
        return cls(**values)

    def to_dict(self) -> dict:
        return {**dataclasses.asdict(self), "type": self.TYPE}


class Robot(PartGroup, ABC):
    ROBOT_TYPE: ClassVar[str] = ""
    CONFIG: ClassVar[type[RobotConfig]] = RobotConfig
    _TYPES: ClassVar[dict[str, type["Robot"]]] = {}

    def __init__(self, config: RobotConfig) -> None:
        super().__init__(config.name)
        self.config = config
        self.build()

    @abstractmethod
    def build(self) -> None:
        """Add child parts; must not touch hardware."""

    @classmethod
    def register_type(cls, robot_cls: type["Robot"]) -> type["Robot"]:
        key = robot_cls.ROBOT_TYPE.lower()
        if not key:
            raise ValueError(f"{robot_cls.__name__} must set ROBOT_TYPE")
        if key in cls._TYPES:
            raise ValueError(f"robot type '{key}' is already registered")
        if robot_cls.CONFIG.TYPE != robot_cls.ROBOT_TYPE:
            raise ValueError(
                f"{robot_cls.__name__}.CONFIG.TYPE '{robot_cls.CONFIG.TYPE}' "
                f"!= ROBOT_TYPE '{robot_cls.ROBOT_TYPE}'"
            )
        cls._TYPES[key] = robot_cls
        return robot_cls

    @classmethod
    def types(cls) -> dict[str, type["Robot"]]:
        return dict(cls._TYPES)

    @classmethod
    def _type_class(cls, robot_type: str) -> type["Robot"]:
        try:
            return cls._TYPES[robot_type.lower()]
        except KeyError:
            raise ValueError(
                f"unknown robot type '{robot_type}'; registered: {sorted(cls._TYPES)}"
            ) from None

    @classmethod
    def from_config(cls, config: RobotConfig) -> "Robot":
        return cls._type_class(config.TYPE)(config)

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> "Robot":
        if "type" not in data:
            raise ValueError(
                f"robot config has no 'type'; add a 'type' key, e.g. type: franka "
                f"(registered: {sorted(cls._TYPES)})"
            )
        robot_cls = cls._type_class(data["type"])
        return robot_cls(robot_cls.CONFIG.from_dict(data))

    @classmethod
    def from_yaml(cls, path: str | Path) -> "Robot":
        data = yaml.safe_load(Path(path).read_text())
        if isinstance(data, Mapping) and "robot" in data:
            data = data["robot"]
        return cls.from_dict(data)

    @property
    def arm(self) -> Arm:
        return self.part("arm")

    @property
    def gripper(self) -> Gripper | None:
        return self.children.get("gripper")

    @property
    def cameras(self) -> dict[str, Camera]:
        group = self.children.get("cameras")
        return dict(group.children) if group is not None else {}

    @property
    def teleop(self) -> Teleop | None:
        return self.children.get("teleop")

    def describe(self) -> dict:
        return {**super().describe(), "type": self.ROBOT_TYPE}

    def describe_text(self) -> str:
        lines = [f"{self.name} ({self.ROBOT_TYPE})"]
        for name, child in self.children.items():
            lines.append(f"  {name}: {self._part_summary(child)}")
            if isinstance(child, PartGroup):
                lines.extend(
                    f"    {sub_name}: {self._part_summary(sub)}"
                    for sub_name, sub in child.children.items()
                )
        return "\n".join(lines)

    @staticmethod
    def _part_summary(part) -> str:
        return f"{type(part).__name__} [sdk={part.SDK}, connected={part.is_connected}]"

    def _build_standard(self) -> None:
        cfg = self.config
        arm = Arm.backend(cfg.arm_backend).from_config(cfg)
        arm.set_station_control(cfg.control)
        self.add("arm", arm)
        if cfg.gripper:
            owner = arm if cfg.gripper.shares_arm_session else None
            gripper_cls = Gripper.backend(cfg.gripper.backend)
            self.add("gripper", gripper_cls.from_config(cfg, owner=owner))
        if cfg.cameras:
            group = PartGroup("cameras")
            for camera_cfg in cfg.cameras:
                group.add(camera_cfg.name, Camera.backend(camera_cfg.backend)(camera_cfg))
            self.add("cameras", group)
        if cfg.teleop is not None:
            # Added last so it disconnects first and never delays releasing the arm.
            teleop_cls = Teleop.backend(cfg.teleop.backend)
            options = {"claim": cfg.claim} if cfg.teleop.backend == "spacemouse" else {}
            self.add("teleop", teleop_cls(cfg.teleop, owner=None, **options))
