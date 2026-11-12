"""Importing this package registers every SDK-free backend."""
from real_world_rl.devices.backends.arms import fake as _fake_arm  # noqa: F401
from real_world_rl.devices.backends.cameras import fake as _fake_camera  # noqa: F401
from real_world_rl.devices.backends.grippers import fake as _fake_gripper  # noqa: F401
from real_world_rl.devices.backends.teleops import fake as _fake_teleop  # noqa: F401
from real_world_rl.teleop.spacemouse import backend as _spacemouse_teleop  # noqa: F401  (SDK import is lazy)
from real_world_rl.teleop.gello import backend as _gello_teleop  # noqa: F401  (SDK import is lazy)
from real_world_rl.devices.backends.cameras import realsense as _realsense_camera  # noqa: F401  (SDK import is lazy)
from real_world_rl.devices.backends.arms import franky as _franky_arm  # noqa: F401  (SDK import is lazy)
from real_world_rl.devices.backends.grippers import franky as _franky_gripper  # noqa: F401
from real_world_rl.devices.backends.arms import serl_http as _serl_http_arm  # noqa: F401
from real_world_rl.devices.backends.grippers import serl_http as _serl_http_gripper  # noqa: F401
from real_world_rl.devices.backends.arms import pyagxarm as _pyagxarm_arm  # noqa: F401  (SDK import is lazy)
from real_world_rl.devices.backends.grippers import pyagxarm as _pyagxarm_gripper  # noqa: F401
from real_world_rl.devices.backends.arms import piper_sdk as _piper_sdk_arm  # noqa: F401  (SDK import is lazy)
from real_world_rl.devices.backends.grippers import piper_sdk as _piper_sdk_gripper  # noqa: F401
