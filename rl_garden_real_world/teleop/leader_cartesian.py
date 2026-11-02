"""Cartesian intervention from a joint-space leader arm (GELLO) through Panda forward kinematics
(registered as ``device="leader_cartesian"``).

For envs that only take a cartesian twist: ``franka_real`` (the SERL HTTP bridge has no joint
command endpoint) and ``real_robot`` in ``cartesian_delta`` mode. The leader's joints ``q`` become
an end-effector target ``T* = (p*, R*)``, and the emitted arm action is the base-frame twist that
moves the follower's measured TCP pose ``(p, R)`` onto it, in the env's normalized units:

    a_pos = clip((p* - p) / pos_scale_m, -1, 1)
    a_rot = clip(rotvec(R* R^T) / rot_scale_rad, -1, 1)

This inverts the env's own update (``p + a_pos * pos_scale_m``, ``from_rotvec(a_rot *
rot_scale_rad) * R``: base-frame translation, left-multiplied rotation), so an unsaturated
action lands the follower on ``T*``. The twist is recomputed from the measured pose every poll,
so saturation only delays convergence; it never drifts.

Mappings:

- ``anchored`` (default): at the intervention onset (the first intervened poll after a
  non-intervened one, and the first after ``reset()``) the follower pose ``(p_f0, R_f0)`` and the
  leader pose ``(p_l0, R_l0) = FK(q_0)`` are stored; then the leader's base-frame displacement is
  applied to the follower, translation and rotation decoupled: ``p* = p_f0 + (p_l - p_l0)``,
  ``R* = (R_l R_l0^T) R_f0``. A leader rotation turns the follower in place (it never swings it
  about the base origin), the onset poll emits a zero twist, and FK errors only enter through
  their change since onset.
- ``absolute``: ``T* = FK(q)``. Needs ``FK`` to match the follower's TCP frame and the leader
  to be near the follower at onset, or the follower moves (rate-limited) to the leader's pose.

The gripper action comes from the leader's width target ``w`` in ``[0, 1]`` (1 = open): ``+1``
if ``w > gripper_binarize_threshold`` else ``-1`` (default 0.5: the trigger closes past half
travel), so the hybrid gripper records clean +-1 labels; ``None`` emits the continuous
``2w - 1``. A leader without a gripper holds the gripper action last sent to the env, as
``TeleopPartSource`` does.
"""
from __future__ import annotations

from typing import Any, Callable, Mapping, Optional, Sequence, Union

import numpy as np

from rl_garden_real_world.kinematics.panda import PandaKinematics
from rl_garden_real_world.kinematics.transforms import (
    matrix_to_rotvec,
    pose_to_transform,
)
from rl_garden_real_world.teleop.base import TeleopReading, TeleopSource
from rl_garden_real_world.teleop.part import (
    DEFAULT_READ_TIMEOUT_S,
    TeleopContract,
    TeleopResolver,
    _find_last_observation_owner,
    teleop_contract_of,
)
from rl_garden_real_world.teleop.registry import register_teleop_source

__all__ = ["LeaderArmCartesianSource", "MAPPINGS"]

MAPPINGS = ("anchored", "absolute")
_LEADER_ACTION_SPACE = "joint_abs"
_CARTESIAN_ACTION_MODE = "cartesian_delta"
_CARTESIAN_ARM_DIM = 6
_FORWARDED_EVENTS = ("success", "failure", "rerecord")
_DEFAULT_REMOTE_NAME = "leader"

PoseGetter = Callable[[], Optional[np.ndarray]]


@register_teleop_source("leader_cartesian")
class LeaderArmCartesianSource(TeleopSource):
    """A ``joint_abs`` leader arm mapped to the env's ``cartesian_delta`` twist (module docstring).

    The leader is either ``teleop`` (a ``Teleop``/``RemoteTeleop`` part or a zero-arg resolver
    returning one, as for ``TeleopPartSource``; the caller owns the part) or the teleop part of
    ``leader_robot``, a robot_infra robot this source owns: ``start()`` connects it and
    ``close()`` disconnects it (a ``RemoteRobot`` for a stub-robot daemon on the robot host, or a
    local robot built from a yaml). ``teleop_contract=(action_mode, arm_action_dim,
    has_gripper)`` optionally pins what the leader must declare; every resolved part must be
    ``joint_abs`` with ``kinematics.NUM_JOINTS`` joints.

    ``follower_pose_getter`` returns the follower's measured TCP pose ``xyz + quat_xyzw`` in the
    base frame (None before the env's first reset). ``pos_scale_m``/``rot_scale_rad`` are the
    env's per-step scales. Read failures propagate; nothing ever turns into an idle reading.
    """

    def __init__(
        self,
        teleop: Union[Any, TeleopResolver, None] = None,
        *,
        follower_pose_getter: PoseGetter,
        leader_robot: Optional[Any] = None,
        env_contract: Optional[Mapping[str, Any]] = None,
        teleop_contract: Optional[Sequence] = None,
        kinematics: Optional[PandaKinematics] = None,
        mapping: str = "anchored",
        pos_scale_m: float = 0.02,
        rot_scale_rad: float = 0.1,
        gripper_binarize_threshold: Optional[float] = 0.5,
        last_gripper_action_getter: Optional[Callable[[], float]] = None,
        read_timeout_s: float = DEFAULT_READ_TIMEOUT_S,
    ) -> None:
        from rl_garden_real_world.action_contract import LEGACY_ACTION_CONTRACT

        if (teleop is None) == (leader_robot is None):
            raise ValueError("pass exactly one leader: teleop=<part or resolver> or leader_robot=<robot>")
        if mapping not in MAPPINGS:
            raise ValueError(f"mapping {mapping!r} invalid; choose from {list(MAPPINGS)}")
        if not (pos_scale_m > 0 and rot_scale_rad > 0):
            raise ValueError(
                f"pos_scale_m and rot_scale_rad must be positive, got {pos_scale_m}, {rot_scale_rad}"
            )
        self._kinematics = PandaKinematics() if kinematics is None else kinematics
        self._mapping = mapping
        self._pos_scale_m = float(pos_scale_m)
        self._rot_scale_rad = float(rot_scale_rad)
        self._gripper_binarize_threshold = gripper_binarize_threshold
        self._follower_pose = follower_pose_getter
        self._last_gripper_action = last_gripper_action_getter
        self._read_timeout_s = read_timeout_s
        contract = LEGACY_ACTION_CONTRACT if env_contract is None else env_contract
        self._env_gripper = bool(contract["gripper_in_action"])

        self._leader_robot = leader_robot
        if leader_robot is not None:
            self._resolve: TeleopResolver = lambda: leader_robot.teleop
        else:
            self._resolve = teleop if callable(teleop) else (lambda: teleop)
        self._declared = None if teleop_contract is None else TeleopContract(*teleop_contract)
        problem = None if self._declared is None else self._contract_problem(self._declared)
        if problem is not None:
            raise ValueError(problem)
        self._verified: Any = None
        self._leader_has_gripper = None if self._declared is None else self._declared.has_gripper
        resolved = self._resolve()
        if resolved is not None:
            problem = self._part_problem(resolved)
            if problem is not None:
                raise ValueError(problem)
            self._accept(resolved)

        self._previously_intervened = False
        self._anchor: Optional[tuple[np.ndarray, np.ndarray]] = None  # (p_f0 - p_l0, R_l0^T R_f0)

    @property
    def action_mode(self) -> str:
        return _CARTESIAN_ACTION_MODE

    @property
    def arm_action_dim(self) -> int:
        return _CARTESIAN_ARM_DIM

    @property
    def has_gripper(self) -> bool:
        return self._env_gripper

    @property
    def leader_robot(self) -> Optional[Any]:
        """The robot this source owns (connected by ``start()``), or None for a caller-owned part."""
        return self._leader_robot

    @classmethod
    def from_env(
        cls,
        env: Any,
        *,
        leader: Union[Any, TeleopResolver, None] = None,
        remote_address: Optional[str] = None,
        remote_token: Optional[str] = None,
        remote_name: str = _DEFAULT_REMOTE_NAME,
        robot_yaml: Optional[str] = None,
        **kwargs: Any,
    ) -> "LeaderArmCartesianSource":
        """Source for ``env``. The leader is exactly one of ``leader`` (a part or resolver the
        caller owns), ``remote_address`` (a robot daemon on ``host:port`` serving the leader as
        its teleop part; a ``RemoteRobot`` is connected in ``start()``) or ``robot_yaml`` (a local
        robot yaml with a ``teleop:`` section, built here and connected in ``start()``).

        Unless passed, the follower pose and the scales come from the env: a ``FrankaRealEnv``
        found down the wrapper chain (``last_tcp_pose``, ``cfg.action_scale``), else a
        ``real_robot`` env in ``cartesian_delta`` mode (``last_observation["state"]["tcp_pose"]``,
        ``xyz_scale_m``/``rot_scale_rad`` of its action contract).
        """
        from rl_garden_real_world.action_contract import (
            LEGACY_ACTION_CONTRACT,
            get_env_action_contract,
        )

        candidates = {"leader": leader, "remote_address": remote_address, "robot_yaml": robot_yaml}
        given = [name for name, value in candidates.items() if value is not None]
        if len(given) != 1:
            raise ValueError(
                "device='leader_cartesian' needs exactly one leader: leader=<teleop part>, "
                "remote_address='host:port' (a robot daemon serving the leader as its teleop "
                f"part) or robot_yaml=<path> (a local robot yaml with a teleop: section); got {given}"
            )
        contract = get_env_action_contract(env)
        kwargs.setdefault("env_contract", LEGACY_ACTION_CONTRACT if contract is None else contract)
        if "follower_pose_getter" not in kwargs:
            getter, (pos_scale_m, rot_scale_rad) = _follower_pose_from_env(env, contract)
            kwargs["follower_pose_getter"] = getter
            kwargs.setdefault("pos_scale_m", pos_scale_m)
            kwargs.setdefault("rot_scale_rad", rot_scale_rad)
        if leader is not None:
            return cls(leader, **kwargs)
        if remote_address is not None:
            from robot_infra.server import RemoteRobot, RemoteRobotConfig

            robot = RemoteRobot(
                RemoteRobotConfig(name=remote_name, address=remote_address, token=remote_token)
            )
        else:
            import robot_infra.devices.backends  # noqa: F401  (registers backends; SDK imports are lazy)
            import robot_infra.devices.robots  # noqa: F401  (registers robot types)
            from robot_infra.devices import Robot

            robot = Robot.from_yaml(robot_yaml)
        return cls(leader_robot=robot, **kwargs)

    # -- source protocol -----------------------------------------------------------

    def start(self) -> None:
        """Connects the owned leader robot (if any) and checks its teleop part."""
        if self._leader_robot is None:
            return
        self._leader_robot.connect()
        try:
            self._require_part()
        except BaseException:
            self._leader_robot.disconnect()
            raise

    def reset(self) -> None:
        """Drops the anchor (the next intervened poll re-anchors) and the leader's latched events."""
        from robot_infra.devices.errors import DeviceNotConnected

        self._previously_intervened = False
        self._anchor = None
        teleop = self._resolve()
        if teleop is None:
            return
        self._verify(teleop)
        try:
            teleop.reset()
        except DeviceNotConnected:
            return

    def poll(self) -> TeleopReading:
        teleop = self._require_part()
        sample = teleop.read(self._read_timeout_s)
        if sample.action_space != _LEADER_ACTION_SPACE:
            raise self._device_fault(
                f"leader sample has action_space {sample.action_space!r}, expected "
                f"{_LEADER_ACTION_SPACE!r}",
                teleop,
            )
        q = np.asarray(sample.action, dtype=np.float64)
        intervened = bool(sample.intervening)
        onset = intervened and not self._previously_intervened
        arm = self._arm_action(q, onset) if intervened else np.zeros(_CARTESIAN_ARM_DIM)
        self._previously_intervened = intervened  # after the mapping: a failed onset retries
        parts = [arm]
        if self._env_gripper:
            parts.append(np.array([self._gripper_action(sample, teleop)]))
        return TeleopReading(
            action=np.concatenate(parts).astype(np.float32),
            intervened=intervened,
            episode_end=bool(sample.events.get("episode_end", False)),
            events={key: True for key in _FORWARDED_EVENTS if sample.events.get(key, False)},
            timestamp=float(getattr(sample, "timestamp", 0.0)),
        )

    def close(self) -> None:
        if self._leader_robot is not None:
            self._leader_robot.disconnect()

    # -- mapping -------------------------------------------------------------------

    def _arm_action(self, q: np.ndarray, onset: bool) -> np.ndarray:
        leader = self._kinematics.ee_transform(q)
        follower = self._follower_transform()
        leader_pos, leader_rot = leader[:3, 3], leader[:3, :3]
        if self._mapping == "absolute":
            target_pos, target_rot = leader_pos, leader_rot
        else:
            if onset or self._anchor is None:
                # p_f0 - p_l0 and R_l0^T R_f0: p* = p_l + (p_f0 - p_l0), R* = R_l (R_l0^T R_f0).
                self._anchor = (follower[:3, 3] - leader_pos, leader_rot.T @ follower[:3, :3])
            position_offset, rotation_offset = self._anchor
            target_pos, target_rot = leader_pos + position_offset, leader_rot @ rotation_offset
        pos = (target_pos - follower[:3, 3]) / self._pos_scale_m
        rot = matrix_to_rotvec(target_rot @ follower[:3, :3].T) / self._rot_scale_rad
        return np.clip(np.concatenate([pos, rot]), -1.0, 1.0)

    def _follower_transform(self) -> np.ndarray:
        pose = self._follower_pose()
        if pose is None:
            raise RuntimeError("leader_cartesian polled before the env was reset (no follower pose yet)")
        return pose_to_transform(pose)

    def _gripper_action(self, sample: Any, teleop: Any) -> float:
        if not self._leader_has_gripper:
            return float(self._last_gripper_action())
        if sample.gripper is None:
            raise self._device_fault("leader declares a gripper but its sample carries none", teleop)
        width = float(sample.gripper)
        if self._gripper_binarize_threshold is None:
            return float(np.clip(2.0 * width - 1.0, -1.0, 1.0))
        return 1.0 if width > self._gripper_binarize_threshold else -1.0

    # -- leader part checks --------------------------------------------------------

    def _require_part(self) -> Any:
        teleop = self._resolve()
        if teleop is None:
            raise self._device_fault(
                "the leader robot has no teleop part (not connected, or no teleop: section in its yaml)"
            )
        self._verify(teleop)
        return teleop

    def _contract_problem(self, contract: TeleopContract) -> Optional[str]:
        if contract.action_mode != _LEADER_ACTION_SPACE:
            return f"the leader must be a {_LEADER_ACTION_SPACE} teleop, got {contract}"
        if contract.arm_action_dim != self._kinematics.NUM_JOINTS:
            return (
                f"the leader must have {self._kinematics.NUM_JOINTS} joints for "
                f"{type(self._kinematics).__name__}, got {contract}"
            )
        if self._env_gripper and not contract.has_gripper and self._last_gripper_action is None:
            return (
                "a leader without a gripper on an env with gripper_in_action holds the last sent "
                "gripper action, so it needs last_gripper_action_getter"
            )
        return None

    def _part_problem(self, teleop: Any) -> Optional[str]:
        actual = teleop_contract_of(teleop)
        if self._declared is not None and actual != self._declared:
            return f"the leader declares {actual}, but this run was built for {self._declared}"
        return self._contract_problem(actual)

    def _accept(self, teleop: Any) -> None:
        self._leader_has_gripper = teleop_contract_of(teleop).has_gripper
        self._verified = teleop

    def _verify(self, teleop: Any) -> None:
        """Checks a newly resolved part (also after a reconnect); a mismatch is a ``DeviceFault``."""
        if teleop is self._verified:
            return
        problem = self._part_problem(teleop)
        if problem is not None:
            raise self._device_fault(problem, teleop)
        self._accept(teleop)

    @staticmethod
    def _device_fault(message: str, teleop: Any = None) -> Exception:
        from robot_infra.devices.errors import DeviceFault

        return DeviceFault(
            message,
            hint="check the leader robot yaml's teleop: section (backend gello, action_space "
            "joint_abs with 7 joints)",
            device=getattr(teleop, "name", "leader"),
        )


def _follower_pose_from_env(env: Any, contract: Optional[Mapping[str, Any]]) -> tuple[PoseGetter, tuple]:
    """The follower TCP pose getter and ``(pos_scale_m, rot_scale_rad)`` of ``env``."""
    from rl_garden_real_world.envs.franka_real.env import FrankaRealEnv

    franka_env = _find_in_wrapper_chain(env, FrankaRealEnv)
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
        "device='leader_cartesian' needs the follower's TCP pose: a franka_real env "
        "(FrankaRealEnv.last_tcp_pose) or a real_robot env with action_mode cartesian_delta; "
        "otherwise pass follower_pose_getter=..."
    )


def _find_in_wrapper_chain(env: Any, env_cls: type) -> Optional[Any]:
    """Walk ``.env`` down the wrapper chain to the first ``env_cls`` instance."""
    node, seen = env, set()
    while node is not None and id(node) not in seen:
        seen.add(id(node))
        if isinstance(node, env_cls):
            return node
        node = vars(node).get("env") if hasattr(node, "__dict__") else None
    return None
