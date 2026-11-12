"""Contract-aware hold policies shared by intervention and recording.

A hold policy is called with the gripper action(s) to hold (the last executed, one per arm) and
returns the ``(A,)`` action that keeps the robot where it should stay. ``reset()`` (after every
env reset) and ``observe_step(intervened)`` (after every env step) are explicit hooks the caller
drives; the recorder calls both. ``as_hold_policy`` adapts a plain callable.
"""
from typing import Any, Callable, Mapping, Optional
import numpy as np
import torch
from real_world_rl.action_contract import LEGACY_ACTION_CONTRACT, get_env_action_contract, num_arms_of, num_grippers_of
from real_world_rl.teleop.core.bimanual import find_bimanual_env, normalized_joint_target
from real_world_rl.teleop.core.part import _find_last_observation_owner
HoldActionFn = Callable[[Any], torch.Tensor]

_CARTESIAN_ARM_DIM = 6


class HoldPolicy:
    """``hold(gripper_actions) -> (A,) float32 tensor`` with no-op lifecycle hooks."""

    def __call__(self, gripper_actions: Any) -> torch.Tensor:
        raise NotImplementedError

    def reset(self) -> None:
        """Called after every env reset."""

    def observe_step(self, intervened: bool) -> None:
        """Called after every env step with whether the operator's action was executed."""


class _FunctionHold(HoldPolicy):
    def __init__(self, hold: HoldActionFn) -> None:
        self._hold = hold

    def __call__(self, gripper_actions: Any) -> torch.Tensor:
        return self._hold(gripper_actions)


def as_hold_policy(hold: Any) -> HoldPolicy:
    """``hold`` itself when it is a ``HoldPolicy``, else a policy calling it with no-op hooks."""
    return hold if isinstance(hold, HoldPolicy) else _FunctionHold(hold)


class _ArmTargetHold(HoldPolicy):
    """A stateless arm action (``arm_action()``) followed by the held gripper action(s)."""

    def __init__(self, arm_action: Callable[[], np.ndarray], num_grippers: int) -> None:
        self._arm_action = arm_action
        self._num_grippers = num_grippers

    def __call__(self, gripper_actions: Any) -> torch.Tensor:
        return _with_grippers(self._arm_action(), gripper_actions, self._num_grippers)


class CartesianHold(HoldPolicy):
    """Keeps a single ``cartesian_delta`` arm on a fixed hold pose with a
    ``CartesianTargetTracker``: the tracker of the env's ``TRACKS_CARTESIAN_TARGET`` teleop
    source (``leader_cartesian``, ``spacemouse``) when there is one (found through the
    intervention wrapper in the chain; the source moves it to the
    pose measured at every hand-back), else one of its own that ``observe_step`` re-samples at
    the pose measured after every intervened step. ``reset()`` samples the hold pose at the env
    reset. The action moves the arm towards the hold pose by at most the tracker's lead cap
    (``reach_for_hold``, then ``action_for``): an own tracker caps at ``min(default cap, env
    scale)`` and the source's caps are at most the env scales, so an arm pushed far off the hold
    gets a saturated action back, never an error. While the shared tracker follows the
    operator, the arm action is zero: the operator's action replaces it.

    Until ``reset()`` was called (a caller that drives no hooks, such as the intervention
    wrapper's fill-in for the dims the operator does not drive), the arm action is the zero twist.
    """

    def __init__(self, env: Any, num_grippers: int) -> None:
        self._env = env
        self._num_grippers = num_grippers
        self._binding: Optional[tuple] = None  # (pose getter, (xyz_scale_m, rot_scale_rad), tracker, owns_tracker)
        self._managed = False

    @property
    def tracker(self):
        return self._bind()[2]

    def reset(self) -> None:
        _, _, tracker, _ = self._bind()
        tracker.reset(self._measured())
        self._managed = True

    def observe_step(self, intervened: bool) -> None:
        if not (self._managed and intervened):
            return
        _, _, tracker, owns_tracker = self._bind()
        if owns_tracker:
            tracker.hold_at(self._measured())

    def __call__(self, gripper_actions: Any) -> torch.Tensor:
        arm = np.zeros(_CARTESIAN_ARM_DIM)
        if self._managed:
            _, (xyz_scale_m, rot_scale_rad), tracker, _ = self._bind()
            if not tracker.following:
                measured = self._measured()
                tracker.reach_for_hold(measured)
                arm = tracker.action_for(measured, xyz_scale_m, rot_scale_rad)
        return _with_grippers(arm, gripper_actions, self._num_grippers)

    def _bind(self) -> tuple:
        if self._binding is None:
            from real_world_rl.teleop.core.targets.cartesian_tracker import (
                CartesianTargetTracker,
                CartesianTrackerConfig,
            )
            from real_world_rl.teleop.core.targets.follower import follower_pose_from_env

            getter, scales = follower_pose_from_env(self._env, get_env_action_contract(self._env))
            xyz_scale_m, rot_scale_rad = (float(scale) for scale in scales)
            tracker = shared_cartesian_tracker(self._env)
            owns_tracker = tracker is None
            if owns_tracker:
                # Caps within one env step, so action_for never has to refuse the hold action.
                defaults = CartesianTrackerConfig()
                tracker = CartesianTargetTracker(CartesianTrackerConfig(
                    lead_cap_m=min(defaults.lead_cap_m, xyz_scale_m),
                    lead_cap_rad=min(defaults.lead_cap_rad, rot_scale_rad),
                ))
            self._binding = (getter, (xyz_scale_m, rot_scale_rad), tracker, owns_tracker)
        return self._binding

    def _measured(self) -> np.ndarray:
        pose = self._bind()[0]()
        if pose is None:
            raise RuntimeError("the cartesian hold reads the follower's TCP pose before the env was reset")
        return np.asarray(pose, dtype=np.float64)


def hold_action_fn_for(env: Any) -> HoldPolicy:
    """The hold policy of ``env``'s robot, by the env's action contract: a ``CartesianHold`` for
    a single ``cartesian_delta`` arm, a zero arm action for ``joint_delta`` (and for two
    ``cartesian_delta`` arms), the follower's current ``joint_pos`` normalised onto
    ``joint_limits`` for ``joint_abs`` (per arm, from each arm's own follower, on a two-arm
    env). The gripper dimensions, when the env has them (the last ``num_arms`` dims), are the
    given gripper actions (the last ones executed, one per arm), since a fixed value would open or
    close the grippers."""
    contract = get_env_action_contract(env) or LEGACY_ACTION_CONTRACT
    num_arms = num_arms_of(contract)
    arm_dim = int(contract["arm_action_dim"])
    num_grippers = num_grippers_of(contract)
    if contract["action_mode"] == "cartesian_delta" and num_arms == 1:
        return CartesianHold(env, num_grippers)

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

    return _ArmTargetHold(arm_action, num_grippers)


def shared_cartesian_tracker(env: Any):
    """The ``CartesianTargetTracker`` of the ``TRACKS_CARTESIAN_TARGET`` source (a
    ``TrackedCartesianSource``, also as the motion half of a composite) of the intervention
    wrapper in ``env``'s chain, or None."""
    from real_world_rl.envs.wrappers.intervention import InterventionWrapper
    from real_world_rl.teleop.core.composite import CompositeTeleopSource
    from real_world_rl.teleop.core.contract import declares_capability

    node, seen = env, set()
    while node is not None and id(node) not in seen:
        seen.add(id(node))
        if isinstance(node, InterventionWrapper):
            source = node.source
            if isinstance(source, CompositeTeleopSource):
                source = source.motion
            tracked = declares_capability(source, "TRACKS_CARTESIAN_TARGET")
            return source.cartesian_tracker if tracked else None
        node = vars(node).get("env") if hasattr(node, "__dict__") else None
    return None


def _with_grippers(arm: np.ndarray, gripper_actions: Any, num_grippers: int) -> torch.Tensor:
    parts = [arm]
    if num_grippers:
        held = np.asarray(gripper_actions, dtype=np.float64).reshape(-1)
        parts.append(np.broadcast_to(held, (num_grippers,)))
    return torch.as_tensor(np.concatenate(parts), dtype=torch.float32)


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
