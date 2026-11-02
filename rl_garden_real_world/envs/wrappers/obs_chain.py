"""Observation-wrapper chain shared by HIL-SERL training and deploy.

Both must stack frames and convert quaternions identically, or a deployed policy sees a
different observation layout than it trained on.
"""
from __future__ import annotations

import logging
from typing import Optional, Sequence, Union

import gymnasium as gym

from rl_garden_real_world.envs.bimanual.config import BimanualRealRobotEnvConfig
from rl_garden_real_world.envs.real_robot.config import RealRobotEnvConfig
from rl_garden_real_world.envs.wrappers.rotvec_obs import QuatSlices, as_quat_slice_list

_LOGGER = logging.getLogger(__name__)
_TCP_POSE_FIELD = "tcp_pose"
_TCP_POSE_QUAT_OFFSET = 3
_QUAT_DIM = 4


def tcp_pose_quat_slice(config: RealRobotEnvConfig) -> Optional[tuple[int, int]]:
    """Where the xyzw quaternion sits in the flat ``state`` vector, or ``None`` without ``tcp_pose``."""
    from rl_garden_real_world.envs.real_robot.env import _state_field_shape

    offset = 0
    for field in config.state_fields:
        if field == _TCP_POSE_FIELD:
            start = offset + _TCP_POSE_QUAT_OFFSET
            return (start, start + _QUAT_DIM)
        offset += _state_field_shape(field, config.num_joints)[0]
    return None


def tcp_pose_quat_slices(
    config: Union[RealRobotEnvConfig, BimanualRealRobotEnvConfig],
) -> list[tuple[int, int]]:
    """Every arm's ``tcp_pose`` quaternion in the flat ``state`` vector, in arm order; empty
    without ``tcp_pose``. A two-arm state is the arms' blocks back to back, each laid out by the
    shared ``state_fields``, so arm ``i``'s slice is the single-arm one shifted by ``i`` blocks."""
    from rl_garden_real_world.envs.real_robot.env import _state_field_shape

    arm_configs = (
        list(config.arm_configs.values()) if isinstance(config, BimanualRealRobotEnvConfig) else [config]
    )
    slices = []
    offset = 0
    for arm_config in arm_configs:
        arm_slice = tcp_pose_quat_slice(arm_config)
        if arm_slice is not None:
            slices.append((offset + arm_slice[0], offset + arm_slice[1]))
        offset += sum(_state_field_shape(field, arm_config.num_joints)[0] for field in arm_config.state_fields)
    return slices


def apply_obs_chain(
    env: gym.Env,
    *,
    frame_stack: int,
    image_keys: Sequence[str],
    convert_obs_rotation: bool,
    quat_slice: Optional[QuatSlices],
) -> gym.Env:
    """Frame stacking (innermost) then rotvec conversion, each only when requested.

    ``image_keys`` empty means every non-``state`` observation key is a camera. ``quat_slice`` is
    one ``(start, end)`` pair or one per arm; ``None`` or empty means there is no quaternion.
    """
    from rl_garden.envs.wrappers.frame_stack import ImageFrameStackWrapper
    from rl_garden_real_world.envs.wrappers.rotvec_obs import RotvecObsWrapper

    if frame_stack > 1:
        # FrankaRealEnv/RealRobotVectorEnv have no _init_raw_obs/update_obs_space (ManiSkill-only
        # APIs) and their camera keys are not necessarily "rgb*"/"depth*"-prefixed, so
        # image_keys must be passed explicitly.
        stack_keys = tuple(image_keys) or tuple(
            k for k in env.single_observation_space.spaces if k != "state"
        )
        env = ImageFrameStackWrapper(env, frame_stack=frame_stack, image_keys=stack_keys)

    if convert_obs_rotation:
        if quat_slice is None or not as_quat_slice_list(quat_slice):
            _LOGGER.warning(
                "convert_obs_rotation skipped: state_fields has no tcp_pose, so there is no "
                "quaternion to convert"
            )
        else:
            env = RotvecObsWrapper(env, quat_slice=quat_slice)
    return env
