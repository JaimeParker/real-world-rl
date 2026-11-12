"""The follower side of a cartesian teleop source, read from the env it drives: the measured TCP
pose and the env's per-step scales, the safety box, and the station workspace/task frame.

``follower_kwargs_from_env`` fills the kwargs every ``TRACKS_CARTESIAN_TARGET`` source takes
from its env (``env_contract``, ``follower_pose_getter``, ``pos_scale_m``/``rot_scale_rad``,
``safety_box``), leaving the ones the caller passed.
"""
from __future__ import annotations

from typing import Any, Callable, Mapping, Optional

import numpy as np

from real_world_rl.teleop.core.part import _find_last_observation_owner

__all__ = [
    "PoseGetter",
    "find_in_wrapper_chain",
    "follower_kwargs_from_env",
    "follower_pose_from_env",
    "safety_box_from_env",
    "workspace_from_env",
]

_CARTESIAN_ACTION_MODE = "cartesian_delta"

# The follower's measured TCP pose ``xyz + quat_xyzw`` in the base frame (None before the env's
# first reset).
PoseGetter = Callable[[], Optional[np.ndarray]]


def find_in_wrapper_chain(env: Any, env_cls: type) -> Optional[Any]:
    """Walk ``.env`` down the wrapper chain to the first ``env_cls`` instance."""
    node, seen = env, set()
    while node is not None and id(node) not in seen:
        seen.add(id(node))
        if isinstance(node, env_cls):
            return node
        node = vars(node).get("env") if hasattr(node, "__dict__") else None
    return None


def follower_pose_from_env(env: Any, contract: Optional[Mapping[str, Any]]) -> tuple[PoseGetter, tuple]:
    """The follower TCP pose getter and ``(pos_scale_m, rot_scale_rad)`` of ``env``."""
    from real_world_rl.envs.franka_real.env import FrankaRealEnv

    franka_env = find_in_wrapper_chain(env, FrankaRealEnv)
    if franka_env is not None:
        return (lambda: franka_env.last_tcp_pose), tuple(franka_env.cfg.action_scale)
    real_env = _find_last_observation_owner(env)
    if real_env is not None and contract is not None and contract["action_mode"] == _CARTESIAN_ACTION_MODE:

        def tcp_pose_from_observation() -> Optional[np.ndarray]:
            observation = real_env.last_observation
            if observation is None:
                return None
            return np.asarray(observation["state"]["tcp_pose"], dtype=np.float64)

        return tcp_pose_from_observation, (contract["xyz_scale_m"], contract["rot_scale_rad"])
    raise ValueError(
        "the follower's TCP pose is read from a franka_real env (FrankaRealEnv.last_tcp_pose) or "
        "a real_robot env with action_mode cartesian_delta (and tcp_pose in its state_fields); "
        "otherwise pass follower_pose_getter=..."
    )


def safety_box_from_env(env: Any, contract: Optional[Mapping[str, Any]]) -> Optional[list]:
    """``[xmin, ymin, zmin, xmax, ymax, zmax]`` the env clips commands to, if configured."""
    from real_world_rl.envs.franka_real.env import FrankaRealEnv

    franka_env = find_in_wrapper_chain(env, FrankaRealEnv)
    if franka_env is not None:
        return [*franka_env.cfg.safety_box_low, *franka_env.cfg.safety_box_high]
    experiment = (contract or {}).get("experiment") or {}
    box = experiment.get("safety_box")
    return None if box is None else list(box)


def follower_kwargs_from_env(env: Any, kwargs: dict) -> dict:
    """``kwargs`` with the follower side read from ``env`` where not given: ``env_contract`` (the
    env's action contract, the legacy one without), ``follower_pose_getter`` with the env's
    ``pos_scale_m``/``rot_scale_rad``, and ``safety_box``. Updates and returns ``kwargs``."""
    from real_world_rl.action_contract import LEGACY_ACTION_CONTRACT, get_env_action_contract

    contract = get_env_action_contract(env)
    kwargs.setdefault("env_contract", LEGACY_ACTION_CONTRACT if contract is None else contract)
    if "follower_pose_getter" not in kwargs:
        getter, (pos_scale_m, rot_scale_rad) = follower_pose_from_env(env, contract)
        kwargs["follower_pose_getter"] = getter
        kwargs.setdefault("pos_scale_m", pos_scale_m)
        kwargs.setdefault("rot_scale_rad", rot_scale_rad)
    kwargs.setdefault("safety_box", safety_box_from_env(env, contract))
    return kwargs


def workspace_from_env(env: Any, kwargs: dict) -> dict:
    """``kwargs`` with the station ``workspace`` and ``task_frame`` of the ``RealRobotEnv`` behind
    ``env`` (its config's, else its local robot yaml's) where not given; unchanged without a
    ``RealRobotEnv``. Updates and returns ``kwargs``."""
    from real_world_rl.envs.real_robot.env import RealRobotEnv

    backend = find_in_wrapper_chain(env, RealRobotEnv)
    if backend is not None:
        config = backend.config
        local = config.robot or {}
        kwargs.setdefault("workspace", config.workspace if config.workspace is not None else local.get("workspace"))
        kwargs.setdefault("task_frame", config.task_frame if config.task_frame is not None else local.get("task_frame"))
    return kwargs
