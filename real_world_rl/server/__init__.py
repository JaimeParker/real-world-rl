"""Robot daemon service layer: wire codec, transport-neutral ``RobotService``, transports and
the client-side ``RemoteRobot`` (importing this package registers robot type "remote")."""
from real_world_rl.server.codec import encode_jpeg, pack, unpack
from real_world_rl.server.service import METHODS, RobotService
from real_world_rl.server.remote import (
    RemoteArm,
    RemoteCamera,
    RemoteGripper,
    RemoteRobot,
    RemoteRobotConfig,
    RemoteTeleop,
)

__all__ = [
    "METHODS",
    "RemoteArm",
    "RemoteCamera",
    "RemoteGripper",
    "RemoteRobot",
    "RemoteRobotConfig",
    "RemoteTeleop",
    "RobotService",
    "encode_jpeg",
    "pack",
    "unpack",
]
