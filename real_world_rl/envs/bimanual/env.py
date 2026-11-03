"""BimanualRealRobotEnv: two ``RealRobotEnv`` arms driven as one numpy gym.Env.

Each arm is its own ``RealRobotEnv`` (a local robot_infra robot or a daemon), built from
``BimanualRealRobotEnvConfig.arm_env_config``. The action follows the action-contract layout
(``real_world_rl.action_contract``): ``[arm_0 block, arm_1 block, gripper_0, gripper_1]``
in ``arm_names`` order, without the gripper dims when ``gripper_in_action`` is false. The
observation is ``{"state": {"<arm>_<field>": float32}, "frames": {cam: uint8 HxWx3}}``: every
state field carries its arm's name as a prefix, cameras keep their (unique) names.

``reset()`` and ``step()`` drive both arms concurrently, one thread per arm, and return once both
are done. Each arm paces itself to ``control_hz`` (``RealRobotEnv.step`` sleeps the rest of the
period since its previous command), so concurrent stepping keeps one control period per step. An
exception from either arm is re-raised after the other arm finished its call.

Safety is per arm: each arm's safety box and joint limits, tightened by its device layer. There is
no inter-arm collision check.
"""
from __future__ import annotations

import logging
from concurrent.futures import ThreadPoolExecutor
from typing import Any, Callable, Mapping, Optional, Sequence, TypeVar

import gymnasium as gym
import numpy as np
import torch
from gymnasium import spaces
from gymnasium.vector.utils import batch_space

from real_world_rl.action_contract import (
    action_dim_of,
    arm_block_slices,
    gripper_slice,
)
from real_world_rl.envs.bimanual.config import BimanualRealRobotEnvConfig
from real_world_rl.tasks import build_task
from real_world_rl.envs.real_robot.env import RealRobotEnv, RealRobotVectorEnv

_LOGGER = logging.getLogger(__name__)
_ArmResult = TypeVar("_ArmResult")


def _raise_first_failure(failures: Sequence[tuple[str, Exception]]) -> None:
    """Re-raise the first ``(arm, error)`` failure; the others are logged, not lost."""
    if not failures:
        return
    for arm, error in failures[1:]:
        _LOGGER.error("arm %r also failed: %r", arm, error)
    raise failures[0][1]


class BimanualRealRobotEnv(gym.Env):
    metadata = {"render_modes": ["rgb_array"]}

    def __init__(
        self, config: BimanualRealRobotEnvConfig, *, robots: Optional[Mapping[str, Any]] = None
    ) -> None:
        """``robots`` optionally injects an already-constructed robot per arm name."""
        config.require_robot_source()
        config.check_arms_consistent()
        robots = dict(robots or {})
        unknown = sorted(set(robots) - set(config.arm_names))
        if unknown:
            raise ValueError(f"robots given for unknown arms {unknown}; arms are {list(config.arm_names)}")
        self.config = config
        self.arms: dict[str, RealRobotEnv] = {
            arm: RealRobotEnv(config.arm_env_config(arm), robot=robots.get(arm))
            for arm in config.arm_names
        }
        self.task_runtime = build_task(config.task, config.arm_names)
        self.task_description: str = "" if config.task.name == "none" else config.task.name
        self.render_mode = "rgb_array"

        state_space = spaces.Dict(
            [
                (f"{arm}_{field}", space)
                for arm, env in self.arms.items()
                for field, space in env.observation_space["state"].spaces.items()
            ]
        )
        frame_space = spaces.Dict(
            [
                (cam, space)
                for env in self.arms.values()
                for cam, space in env.observation_space["frames"].spaces.items()
            ]
        )
        depth_space = spaces.Dict([(cam, space) for env in self.arms.values()
                                  for cam, space in env.observation_space.spaces.get("depth_frames", spaces.Dict()).spaces.items()])
        self.observation_space = spaces.Dict([("state", state_space), ("frames", frame_space)] + ([("depth_frames", depth_space)] if depth_space.spaces else []))
        contract = self.action_contract
        self.action_space = spaces.Box(-1.0, 1.0, shape=(action_dim_of(contract),), dtype=np.float32)
        self._arm_blocks = dict(zip(config.arm_names, arm_block_slices(contract)))
        self._gripper_slice = gripper_slice(contract)

        self._executor: Optional[ThreadPoolExecutor] = None
        self._last_obs: Optional[dict] = None
        self._elapsed_steps = 0

    # -- concurrency -----------------------------------------------------------------

    def _on_every_arm(self, call: Callable[[str, RealRobotEnv], _ArmResult]) -> dict[str, _ArmResult]:
        """``call(arm, env)`` on every arm concurrently. Waits for all of them, then re-raises the
        first failure in arm order."""
        if self._executor is None:
            self._executor = ThreadPoolExecutor(
                max_workers=len(self.arms), thread_name_prefix="bimanual_arm"
            )
        futures = {arm: self._executor.submit(call, arm, env) for arm, env in self.arms.items()}
        results: dict[str, _ArmResult] = {}
        failures: list[tuple[str, Exception]] = []
        for arm, future in futures.items():
            try:
                results[arm] = future.result()
            except Exception as error:
                failures.append((arm, error))
        _raise_first_failure(failures)
        return results

    # -- gym API ---------------------------------------------------------------------

    def reset(self, *, seed: Optional[int] = None, options: Optional[dict] = None):
        super().reset(seed=seed)
        if self.config.task.name != "none":
            results = self.task_runtime.reset(self, self.np_random)
        else:
            results = self._on_every_arm(lambda arm, env: env.reset(seed=seed, options=options))
        self._elapsed_steps = 0
        obs = self._merge_observations({arm: obs for arm, (obs, _) in results.items()})
        return obs, self._merge_infos({arm: info for arm, (_, info) in results.items()})

    def step(self, action):
        if self._last_obs is None:
            raise RuntimeError("BimanualRealRobotEnv.step() called before reset()")
        action = np.asarray(action, dtype=np.float64).reshape(-1)
        if action.shape != self.action_space.shape:
            raise ValueError(f"expected action shape {self.action_space.shape}, got {action.shape}")
        if not np.isfinite(action).all():
            raise ValueError("action must contain finite numbers")
        arm_actions = self.split_action(action)
        results = self._on_every_arm(lambda arm, env: env.step(arm_actions[arm]))

        self._elapsed_steps += 1
        obs = self._merge_observations({arm: result[0] for arm, result in results.items()})
        arm_infos = {arm: result[4] for arm, result in results.items()}
        info = self._merge_infos(arm_infos)
        info["task_tcp_positions"] = {arm: result[4]["task_tcp_positions"]["arm"]
                                      for arm, result in results.items() if result[4].get("task_tcp_positions")}
        rtts = [arm_info["rtt_ms"] for arm_info in arm_infos.values() if arm_info.get("rtt_ms") is not None]
        # The slower arm bounds the step's round trip.
        info["rtt_ms"] = max(rtts) if rtts else None
        info["elapsed_steps"] = self._elapsed_steps
        truncated = (
            self.config.max_episode_steps is not None
            and self._elapsed_steps >= self.config.max_episode_steps
        )
        return obs, 0.0, False, bool(truncated), info

    def split_action(self, action: np.ndarray) -> dict[str, np.ndarray]:
        """Each arm's own ``RealRobotEnv`` action, ``[arm block, its gripper]``, by arm name."""
        grippers = action[self._gripper_slice]
        return {
            arm: np.concatenate([action[block], grippers[index : index + 1]])
            for index, (arm, block) in enumerate(self._arm_blocks.items())
        }

    def _merge_observations(self, arm_observations: Mapping[str, dict]) -> dict:
        state = {
            f"{arm}_{field}": value
            for arm, obs in arm_observations.items()
            for field, value in obs["state"].items()
        }
        frames = {cam: frame for obs in arm_observations.values() for cam, frame in obs["frames"].items()}
        depths = {cam: value for obs in arm_observations.values() for cam, value in obs.get("depth_frames", {}).items()}
        self._last_obs = {"state": state, "frames": frames}
        if depths: self._last_obs["depth_frames"] = depths
        return self._last_obs

    @staticmethod
    def _merge_infos(arm_infos: Mapping[str, dict]) -> dict:
        return {f"{arm}_{key}": value for arm, info in arm_infos.items() for key, value in info.items()}

    def render(self):
        if self._last_obs is None or not self._last_obs["frames"]:
            return None
        return next(iter(self._last_obs["frames"].values()))

    def close(self) -> None:
        failures: list[tuple[str, Exception]] = []
        for arm, env in self.arms.items():
            try:
                env.close()
            except Exception as error:
                failures.append((arm, error))
        if self._executor is not None:
            self._executor.shutdown(wait=True)
            self._executor = None
        _raise_first_failure(failures)

    # -- properties mirrored from RealRobotEnv ------------------------------------------

    @property
    def arm_names(self) -> list[str]:
        return list(self.config.arm_names)

    @property
    def robot(self) -> dict[str, Any]:
        """Each arm's robot, by arm name."""
        return {arm: env.robot for arm, env in self.arms.items()}

    @property
    def action_mode(self) -> str:
        return self.config.left.action_mode

    @property
    def gripper_in_action(self) -> bool:
        return self.config.left.gripper_in_action

    @property
    def num_joints(self) -> int:
        return self.config.left.num_joints

    @property
    def arm_action_dim(self) -> int:
        """Action dims of one arm, excluding its gripper (the same for both arms)."""
        return next(iter(self.arms.values())).arm_action_dim

    @property
    def runtime_control(self):
        return {name: env._control_metadata for name, env in self.arms.items()}

    @property
    def requested_experiment(self):
        import dataclasses
        return {"task": dataclasses.asdict(self.config.task),
                "arm_names": self.config.arm_names, "max_episode_steps": self.config.max_episode_steps,
                "reset_settle_s": self.config.reset_settle_s,
                "arms": {name: env.requested_experiment for name, env in self.arms.items()}}

    @property
    def action_contract(self) -> dict:
        """The shared contract with ``num_arms=2``. ``joint_limits`` is per arm, in arm order
        (``(num_arms, num_joints, 2)``), or None when no arm sets them."""
        shared = self.config.left
        arm_configs = self.config.arm_configs.values()
        joint_limits = (
            None if shared.joint_limits is None else [config.joint_limits for config in arm_configs]
        )
        return {
            "experiment": self.requested_experiment,
            "action_mode": shared.action_mode,
            "gripper_in_action": shared.gripper_in_action,
            "arm_action_dim": self.arm_action_dim,
            "num_arms": len(self.arms),
            "arm_names": self.arm_names,
            "num_joints": shared.num_joints,
            "joint_limits": joint_limits,
            "xyz_scale_m": shared.xyz_scale_m,
            "rot_scale_rad": shared.rot_scale_rad,
            "joint_scale_rad": shared.joint_scale_rad,
            "state_fields": shared.state_fields,
        }


class BimanualRealRobotVectorEnv(RealRobotVectorEnv):
    """Batch-of-1 torch view of a ``BimanualRealRobotEnv``.

    ``state`` concatenates the arms in arm order, each arm's ``state_fields`` in config order
    (``(1, D)`` float32); each camera becomes ``rgb_<cam>`` (``(1, H, W, 3)`` uint8), in arm
    order. Tensors live on CPU and there is no autoreset, as in ``RealRobotVectorEnv``, whose
    delegating properties and ``reset``/``step``/``close`` this class reuses.
    """

    def __init__(self, env: BimanualRealRobotEnv, *, include_state: bool = True) -> None:
        self.env = env
        self.include_state = include_state
        entries: list[tuple[str, spaces.Space]] = [
            (f"rgb_{cam}", space) for cam, space in env.observation_space["frames"].spaces.items()
        ]
        entries.extend((f"depth_{cam}", space) for cam, space in env.observation_space.spaces.get("depth_frames", spaces.Dict()).spaces.items())
        if include_state:
            state_dim = sum(space.shape[0] for space in env.observation_space["state"].spaces.values())
            entries.append(("state", spaces.Box(-np.inf, np.inf, shape=(state_dim,), dtype=np.float32)))
        self.single_observation_space = spaces.Dict(entries)
        self.single_action_space = env.action_space
        self.observation_space = batch_space(self.single_observation_space, 1)
        self.action_space = batch_space(self.single_action_space, 1)
        self.metadata = env.metadata

    def _to_torch(self, obs: dict) -> dict[str, torch.Tensor]:
        out = {
            f"rgb_{cam}": torch.from_numpy(np.require(obs["frames"][cam], requirements=["C", "W"])).unsqueeze(0)
            for cam in self.env.observation_space["frames"].spaces
        }
        out.update({f"depth_{cam}": torch.from_numpy(np.ascontiguousarray(value)).unsqueeze(0)
                    for cam, value in obs.get("depth_frames", {}).items()})
        if self.include_state:
            state = np.concatenate([obs["state"][key] for key in self.env.observation_space["state"].spaces])
            out["state"] = torch.from_numpy(state.astype(np.float32, copy=False)).unsqueeze(0)
        return out
