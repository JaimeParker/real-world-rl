"""Two-arm robot_infra real-robot env backend, registered as ``"real_robot_bimanual"``.

``req.backend_config`` is a ``BimanualRealRobotEnvConfig`` (one ``RealRobotEnvConfig`` per arm).
As for ``real_robot``, ``req.num_envs``/``req.num_eval_envs`` are ignored (the two arms are one
env, a batch of exactly one) and train and eval share one env instance per request.

``req.observation`` is honored or rejected as for ``real_robot``: depth, extra_state and frame
stacking raise ``ObservationContractError``; ``state`` toggles the flat ``state`` key (both arms,
arm order); ``image_size`` resizes every frame. ``rgb`` must list exactly the arms' cameras in arm
order (``--real_robot_bimanual.<arm>.cameras``): the backend cannot tell which arm's robot serves a
camera, so it never fills the per-arm camera lists from ``rgb``.
"""

from __future__ import annotations

import dataclasses
from typing import Any, ClassVar

from rl_garden.envs.backend_registry import EnvRequest, register_env_backend
from rl_garden.observations.schema import ObservationContractError

from real_world_rl.envs.backends.real_robot import RealRobotBackend


class RealRobotBimanualBackend(RealRobotBackend):
    """Reuses ``RealRobotBackend``'s shared-env-per-request bookkeeping with its own env cache."""

    api_version = 2
    config_field = "real_robot_bimanual"

    _envs_by_request: ClassVar[dict[int, Any]] = {}

    @classmethod
    def resolve_config(cls, req: EnvRequest, *, is_eval: bool):
        del is_eval
        from real_world_rl.envs.bimanual.config import BimanualRealRobotEnvConfig

        config = req.backend_config
        if not isinstance(config, BimanualRealRobotEnvConfig):
            raise ValueError(
                "real_robot_bimanual backend needs req.backend_config to be a "
                f"BimanualRealRobotEnvConfig (with each arm's `robot` or `remote` set), got "
                f"{type(config).__name__}"
            )
        config.require_robot_source()
        observation = req.observation
        unsupported = {
            "depth": observation.depth,
            "extra_state": observation.extra_state,
            "frame_stack>1": observation.frame_stack > 1,
        }
        requested = [name for name, value in unsupported.items() if value]
        if requested:
            raise ObservationContractError(
                f"real_robot_bimanual backend cannot produce {requested}; it supports state and "
                "rgb only"
            )
        cameras = list(observation.rgb)
        if config.cameras != cameras:
            per_arm = {arm: arm_config.cameras for arm, arm_config in config.arm_configs.items()}
            raise ObservationContractError(
                f"real_robot_bimanual requested rgb cameras {cameras} must equal the arms' cameras "
                f"in arm order, {config.cameras} (per arm: {per_arm}); set "
                "--real_robot_bimanual.<arm>.cameras to name each arm's cameras"
            )
        image_size = config.left.image_size
        if observation.image_size is not None:
            if image_size is not None and tuple(image_size) != tuple(observation.image_size):
                raise ObservationContractError(
                    f"real_robot_bimanual config image_size {image_size} conflicts with requested "
                    f"image_size {observation.image_size}"
                )
            image_size = observation.image_size
        return dataclasses.replace(
            config,
            left=dataclasses.replace(config.left, image_size=image_size),
            right=dataclasses.replace(config.right, image_size=image_size),
        )

    @classmethod
    def _make_env(cls, req: EnvRequest):
        from real_world_rl.envs.bimanual import (
            BimanualRealRobotEnv,
            BimanualRealRobotVectorEnv,
        )

        env = BimanualRealRobotVectorEnv(
            BimanualRealRobotEnv(cls.resolve_config(req, is_eval=False)),
            include_state=req.observation.state,
        )
        if req.reward_scale != 1.0 or req.reward_bias != 0.0:
            from rl_garden.envs.wrappers.reward_transform import RewardScaleBiasWrapper

            env = RewardScaleBiasWrapper(env, req.reward_scale, req.reward_bias)
        return env

    @classmethod
    def make_train_env(cls, req: EnvRequest):
        return cls._shared_env(req)

    @classmethod
    def make_eval_env(cls, req: EnvRequest):
        return cls._shared_env(req)


register_env_backend("real_robot_bimanual", RealRobotBimanualBackend)
