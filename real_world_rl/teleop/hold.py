"""Contract-aware hold targets shared by intervention and recording."""
from typing import Any, Callable, Mapping
import numpy as np
import torch
from real_world_rl.action_contract import LEGACY_ACTION_CONTRACT, get_env_action_contract, num_arms_of, num_grippers_of
from real_world_rl.teleop.bimanual import find_bimanual_env, normalized_joint_target
from real_world_rl.teleop.part import _find_last_observation_owner
HoldActionFn = Callable[[Any], torch.Tensor]


def hold_action_fn_for(env: Any) -> HoldActionFn:
    """The action that keeps ``env``'s robot still, by the env's action contract: a zero arm
    action for ``joint_delta`` and ``cartesian_delta``, the follower's current ``joint_pos``
    normalised onto ``joint_limits`` for ``joint_abs`` (per arm, from each arm's own follower,
    on a two-arm env). The gripper dimensions, when the env has them (the last ``num_arms``
    dims), are the given gripper actions (the last ones executed, one per arm), since a fixed
    value would open or close the grippers."""
    contract = get_env_action_contract(env) or LEGACY_ACTION_CONTRACT
    num_arms = num_arms_of(contract)
    arm_dim = int(contract["arm_action_dim"])
    num_grippers = num_grippers_of(contract)

    def arm_action() -> np.ndarray:
        if contract["action_mode"] != "joint_abs":
            return np.zeros(num_arms * arm_dim)
        limits = contract.get("joint_limits")
        if limits is None:
            raise ValueError("a joint_abs env needs joint_limits in its action contract to hold the arm")
        if num_arms == 1:
            return normalized_joint_target(follower_joint_pos(env), limits)
        return np.concatenate([normalized_joint_target(follower_joint_pos(arm_env), per_arm)
                               for arm_env, per_arm in zip(_arm_envs_of(env, contract), limits)])

    def hold(gripper_actions: Any) -> torch.Tensor:
        parts = [arm_action()]
        if num_grippers:
            held = np.asarray(gripper_actions, dtype=np.float64).reshape(-1)
            parts.append(np.broadcast_to(held, (num_grippers,)))
        return torch.as_tensor(np.concatenate(parts), dtype=torch.float32)

    return hold


def follower_joint_pos(env: Any) -> np.ndarray:
    """``state["joint_pos"]`` of the latest raw observation of the ``RealRobotEnv`` behind ``env``."""
    owner = _find_last_observation_owner(env)
    if owner is None:
        raise ValueError(
            "reading the follower joints needs a real_robot env (RealRobotEnv.last_observation)"
        )
    observation = owner.last_observation
    if observation is None:
        raise RuntimeError("the follower joints are read before the env was reset")
    state = observation["state"]
    if "joint_pos" not in state:
        raise ValueError(
            "the follower joints are read from the observation's joint_pos; add joint_pos to "
            "--real-robot.state-fields"
        )
    return np.asarray(state["joint_pos"], dtype=np.float64)


def _arm_envs_of(env: Any, contract: Mapping[str, Any]) -> list:
    """Each arm's ``RealRobotEnv`` of the two-arm env behind ``env``, in ``arm_names`` order."""
    bimanual = find_bimanual_env(env)
    if bimanual is None:
        raise ValueError(
            f"reading each arm's follower joints (num_arms={num_arms_of(contract)}) needs a "
            "real_robot_bimanual env (BimanualRealRobotEnv) behind the wrapper chain"
        )
    return [bimanual.arms[arm] for arm in contract["arm_names"]]
