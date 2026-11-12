"""Device layer: typed errors, connection/part abstractions, backend registry, claims."""
from real_world_rl.devices.arm import (
    ARM_STATE_FIELDS,
    Arm,
    ArmHealth,
    ControlMode,
    arm_state_shape,
)
from real_world_rl.devices.base import (
    Connection,
    ControllablePart,
    Features,
    FeatureSpec,
    Health,
    Part,
    PartGroup,
)
from real_world_rl.devices.camera import Camera, CameraConfig, Frame
from real_world_rl.devices.claims import DeviceClaim
from real_world_rl.devices.errors import (
    DeviceAlreadyConnected,
    DeviceBusy,
    DeviceError,
    DeviceFault,
    DeviceNotConnected,
    DeviceNotFound,
    SafetyViolation,
    SdkMissing,
    VersionMismatch,
)
from real_world_rl.devices.gripper import Gripper, GripperMode
from real_world_rl.devices.registry import BackendRegistry
from real_world_rl.devices.robot import GripperConfig, Robot, RobotConfig
from real_world_rl.devices.safety import ArmSafety
from real_world_rl.devices.teleop import RawReading, Teleop, TeleopConfig, TeleopHealth, TeleopSample

__all__ = [
    "ARM_STATE_FIELDS",
    "Arm",
    "ArmHealth",
    "ArmSafety",
    "Camera",
    "CameraConfig",
    "ControlMode",
    "Frame",
    "Gripper",
    "GripperConfig",
    "GripperMode",
    "arm_state_shape",
    "BackendRegistry",
    "Connection",
    "ControllablePart",
    "DeviceAlreadyConnected",
    "DeviceBusy",
    "DeviceClaim",
    "DeviceError",
    "DeviceFault",
    "DeviceNotConnected",
    "DeviceNotFound",
    "FeatureSpec",
    "Features",
    "Health",
    "Part",
    "PartGroup",
    "Robot",
    "RobotConfig",
    "SafetyViolation",
    "SdkMissing",
    "RawReading",
    "Teleop",
    "TeleopConfig",
    "TeleopHealth",
    "TeleopSample",
    "VersionMismatch",
]

from real_world_rl.devices import backends as _backends  # noqa: E402,F401
from real_world_rl.devices import robots as _robots  # noqa: E402,F401
