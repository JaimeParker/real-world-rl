"""Cartesian intervention from a joint-space leader arm (GELLO) through Panda forward kinematics
(registered as ``device="leader_cartesian"``).

For envs that only take a cartesian twist: ``franka_real`` (the SERL HTTP bridge has no joint
command endpoint) and ``real_robot`` in ``cartesian_delta`` mode, whose step sends
``measured + action * scale``. The leader's joints ``q`` become a requested end-effector pose,
and a ``CartesianTargetTracker`` (``real_world_rl.teleop.core.targets.cartesian_tracker``) turns it into the
emitted arm action: the tracker's persistent command advances towards the request at
``target_speed_m_s``/``target_speed_rad_s``, never leads the follower's measured TCP pose by more
than ``lead_cap_m``/``lead_cap_rad`` (the excess is dropped, never queued), and the action is
``(command - measured)`` in the env's normalized units (``[xyz / pos_scale_m, rotvec(R_cmd
R_meas^T) / rot_scale_rad]``), which the env's own update lands back on the command. The scales
must be at least the lead caps (checked at construction), so the action never saturates.

Mappings of the leader pose ``(p_l, R_l) = FK(q)`` onto the requested pose:

- ``anchored`` (default): at the take-over the tracker's current command ``(p_c, R_c)`` (the hold
  pose: the follower does not jump) and the leader pose ``(p_l0, R_l0)`` are stored; then the
  leader's base-frame displacement is applied to it, translation and rotation decoupled:
  ``p* = p_c + (p_l - p_l0)``, ``R* = (R_l R_l0^T) R_c``. A leader rotation turns the follower in
  place, and FK errors only enter through their change since the take-over.
- ``absolute``: ``FK(q)``. Needs ``FK`` to match the follower's TCP frame; the command then moves
  from the hold pose to the leader's pose at the target speed.

The take-over is the rising edge of the operator's intervention signal (the part's
``intervening``, or the toggle ``CompositeTeleopSource`` hands to ``poll_with_toggle``). Its
falling edge (hand-back), an input or tracking failure and an episode end put the tracker in
hold at the measured pose. A stalled control loop (more than ``max_dt_s`` between two intervened
polls) counts as an input loss: the follower holds, a warning gives the gap, and the next poll
re-anchors the leader as at a take-over. A requested position outside ``safety_box`` does too, logs
``WORKSPACE HOLD: <axes>`` once, and reads as not intervened until the operator hands back and
takes over again. The tracker (``cartesian_tracker``) is shared with the recorder's hold policy
(``real_world_rl.teleop.core.hold``), which keeps the follower on the hold pose while nobody drives.

The gripper action comes from the leader's width target ``w`` in ``[0, 1]`` (1 = open): ``+1``
if ``w > gripper_binarize_threshold`` else ``-1`` (default 0.5: the trigger closes past half
travel), so the hybrid gripper records clean +-1 labels; ``None`` emits the continuous
``2w - 1``. With ``last_gripper_action_getter`` (the intervention wrapper passes it), a trigger
that disagrees at the take-over with the gripper's last commanded state (open: action > 0) is
ignored: the last commanded action is held until the trigger crosses to that state, so taking
over never drops a grasped object. A leader without a gripper always holds the last action.
"""
from __future__ import annotations

import logging
import time
from typing import Any, Callable, Mapping, Optional, Sequence, Union

import numpy as np

from real_world_rl.kinematics.panda import PandaKinematics
from real_world_rl.kinematics.transforms import (
    matrix_to_quat_xyzw,
    pose_to_transform,
)
from real_world_rl.teleop.core.contract import TeleopReading, held_gripper_action
from real_world_rl.teleop.core.targets.cartesian_tracker import (
    CartesianTargetTracker,
    CartesianTrackerConfig,
)
from real_world_rl.teleop.core.composite import ToggleableTeleopSource
from real_world_rl.teleop.core.part import (
    DEFAULT_READ_TIMEOUT_S,
    TeleopContract,
    TeleopResolver,
    _find_last_observation_owner,
    teleop_contract_of,
)
from real_world_rl.teleop.core.registry import register_teleop_source

__all__ = ["LeaderArmCartesianSource", "MAPPINGS", "follower_pose_from_env"]

MAPPINGS = ("anchored", "absolute")
_LEADER_ACTION_SPACE = "joint_abs"
_CARTESIAN_ACTION_MODE = "cartesian_delta"
_CARTESIAN_ARM_DIM = 6
_FORWARDED_EVENTS = ("success", "failure", "rerecord")
_DEFAULT_REMOTE_NAME = "leader"

PoseGetter = Callable[[], Optional[np.ndarray]]

logger = logging.getLogger(__name__)


@register_teleop_source("leader_cartesian")
class LeaderArmCartesianSource(ToggleableTeleopSource):
    """A ``joint_abs`` leader arm mapped to the env's ``cartesian_delta`` twist (module docstring).

    The leader is either ``teleop`` (a ``Teleop``/``RemoteTeleop`` part or a zero-arg resolver
    returning one, as for ``TeleopPartSource``; the caller owns the part) or the teleop part of
    ``leader_robot``, a device-layer robot this source owns: ``start()`` connects it and
    ``close()`` disconnects it (a ``RemoteRobot`` for a stub-robot daemon on the robot host, or a
    local robot built from a yaml). ``teleop_contract=(action_mode, arm_action_dim,
    has_gripper)`` optionally pins what the leader must declare; every resolved part must be
    ``joint_abs`` with ``kinematics.NUM_JOINTS`` joints.

    ``follower_pose_getter`` returns the follower's measured TCP pose ``xyz + quat_xyzw`` in the
    base frame (None before the env's first reset). ``pos_scale_m``/``rot_scale_rad`` are the
    env's per-step scales and must be at least ``lead_cap_m``/``lead_cap_rad``. ``clock`` (a
    monotonic clock in seconds) times the tracker's advances; more than ``max_dt_s`` between two
    intervened polls holds the follower and re-anchors at the next poll, a clock going backwards
    raises ``MissedControlDeadline``. Read failures propagate; nothing ever turns into an idle
    reading.
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
        pos_scale_m: float = 0.05,
        rot_scale_rad: float = 0.3,
        lead_cap_m: float = 0.05,
        lead_cap_rad: float = 0.3,
        target_speed_m_s: float = 0.05,
        target_speed_rad_s: float = 0.3,
        max_dt_s: float = 0.3,
        safety_box: Optional[Sequence[float]] = None,
        gripper_binarize_threshold: Optional[float] = 0.5,
        last_gripper_action_getter: Optional[Callable[[], Any]] = None,
        read_timeout_s: float = DEFAULT_READ_TIMEOUT_S,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        from real_world_rl.action_contract import LEGACY_ACTION_CONTRACT

        if (teleop is None) == (leader_robot is None):
            raise ValueError("pass exactly one leader: teleop=<part or resolver> or leader_robot=<robot>")
        if mapping not in MAPPINGS:
            raise ValueError(f"mapping {mapping!r} invalid; choose from {list(MAPPINGS)}")
        if not (pos_scale_m > 0 and rot_scale_rad > 0):
            raise ValueError(
                f"pos_scale_m and rot_scale_rad must be positive, got {pos_scale_m}, {rot_scale_rad}"
            )
        tracker_config = CartesianTrackerConfig(
            lead_cap_m=lead_cap_m,
            lead_cap_rad=lead_cap_rad,
            speed_m_s=target_speed_m_s,
            speed_rad_s=target_speed_rad_s,
            max_dt_s=max_dt_s,
        )
        if pos_scale_m < lead_cap_m or rot_scale_rad < lead_cap_rad:
            raise ValueError(
                "leader_cartesian needs xyz_scale_m >= lead_cap_m and rot_scale_rad >= lead_cap_rad, "
                "so one env step reaches every command the lead cap allows; got "
                f"xyz_scale_m={pos_scale_m}, lead_cap_m={lead_cap_m}, rot_scale_rad={rot_scale_rad}, "
                f"lead_cap_rad={lead_cap_rad}. Raise --real-robot.xyz-scale-m/--real-robot.rot-scale-rad "
                "or lower --teleop-lead-cap-m/--teleop-lead-cap-rad"
            )
        self._tracker = CartesianTargetTracker(tracker_config, safety_box=safety_box)
        self._kinematics = PandaKinematics() if kinematics is None else kinematics
        self._mapping = mapping
        self._pos_scale_m = float(pos_scale_m)
        self._rot_scale_rad = float(rot_scale_rad)
        self._gripper_binarize_threshold = gripper_binarize_threshold
        self._follower_pose = follower_pose_getter
        self._last_gripper_action = last_gripper_action_getter
        self._read_timeout_s = read_timeout_s
        self._clock = clock
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

        self._previously_requested = False
        # Set by a workspace rejection or an episode end: following again needs a new take-over.
        self._needs_new_takeover = False
        # Set by a stalled control loop: the next intervened poll re-anchors on its own.
        self._reanchor_pending = False
        self._anchor: Optional[tuple[np.ndarray, np.ndarray]] = None  # (p_c0 - p_l0, R_l0^T R_c0)
        self._last_advance_s: Optional[float] = None
        self._gripper_latched = False

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

    @property
    def cartesian_tracker(self) -> CartesianTargetTracker:
        """The follower's command tracker, shared with the hold policy."""
        return self._tracker

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

        Unless passed, the follower pose, the scales and the safety box come from the env: a
        ``FrankaRealEnv`` found down the wrapper chain (``last_tcp_pose``, ``cfg.action_scale``,
        ``cfg.safety_box_low/high``), else a ``real_robot`` env in ``cartesian_delta`` mode
        (``last_observation["state"]["tcp_pose"]``, ``xyz_scale_m``/``rot_scale_rad`` of its
        action contract, the configured ``safety_box`` of its experiment).
        """
        from real_world_rl.action_contract import (
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
            getter, (pos_scale_m, rot_scale_rad) = follower_pose_from_env(env, contract)
            kwargs["follower_pose_getter"] = getter
            kwargs.setdefault("pos_scale_m", pos_scale_m)
            kwargs.setdefault("rot_scale_rad", rot_scale_rad)
        kwargs.setdefault("safety_box", _safety_box_from_env(env, contract))
        if leader is not None:
            return cls(leader, **kwargs)
        if remote_address is not None:
            from real_world_rl.server import RemoteRobot, RemoteRobotConfig

            robot = RemoteRobot(
                RemoteRobotConfig(name=remote_name, address=remote_address, token=remote_token)
            )
        else:
            import real_world_rl.devices.backends  # noqa: F401  (registers backends; SDK imports are lazy)
            import real_world_rl.devices.robots  # noqa: F401  (registers robot types)
            from real_world_rl.devices import Robot

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
        """After the env reset: the tracker holds the follower's pose, the next intervened poll
        is a take-over, and the leader's latched events are dropped."""
        from real_world_rl.devices.errors import DeviceNotConnected

        self._previously_requested = False
        self._needs_new_takeover = False
        self._reanchor_pending = False
        self._anchor = None
        self._last_advance_s = None
        self._gripper_latched = False
        pose = self._follower_pose()
        if pose is not None:
            self._tracker.reset(pose)
        teleop = self._resolve()
        if teleop is None:
            return
        self._verify(teleop)
        try:
            teleop.reset()
        except DeviceNotConnected:
            return

    def poll(self) -> TeleopReading:
        """One leader read; the part's own ``intervening`` is the operator's signal."""
        return self._poll(toggle=None)

    def poll_with_toggle(self, toggle: bool) -> TeleopReading:
        return self._poll(toggle=bool(toggle))

    def close(self) -> None:
        if self._leader_robot is not None:
            self._leader_robot.disconnect()

    # -- polling -------------------------------------------------------------------

    def _poll(self, toggle: Optional[bool]) -> TeleopReading:
        teleop = self._require_part()
        try:
            sample = teleop.read(self._read_timeout_s)
            if sample.action_space != _LEADER_ACTION_SPACE:
                raise self._device_fault(
                    f"leader sample has action_space {sample.action_space!r}, expected "
                    f"{_LEADER_ACTION_SPACE!r}",
                    teleop,
                )
        except BaseException:
            self._hold_after_failure()
            raise
        q = np.asarray(sample.action, dtype=np.float64)
        requested = bool(sample.intervening) if toggle is None else toggle
        episode_end = bool(sample.events.get("episode_end", False))
        measured = self._measured_pose()
        if not self._tracker.initialized:
            self._tracker.reset(measured)
        takeover = requested and not self._previously_requested
        intervened, arm = self._arm_action(q, measured, requested, takeover)
        self._previously_requested = requested  # after the mapping: a failed take-over retries
        if episode_end and self._tracker.following:
            self._tracker.hold_at(measured)
            self._needs_new_takeover = True
            arm = self._tracker.action_for(measured, self._pos_scale_m, self._rot_scale_rad)
        parts = [arm]
        if self._env_gripper:
            parts.append(np.array([self._gripper_action(sample, teleop, takeover, requested)]))
        return TeleopReading(
            action=np.concatenate(parts).astype(np.float32),
            intervened=intervened,
            episode_end=episode_end,
            events={key: True for key in _FORWARDED_EVENTS if sample.events.get(key, False)},
            timestamp=float(getattr(sample, "timestamp", 0.0)),
        )

    def _arm_action(
        self, q: np.ndarray, measured: np.ndarray, requested: bool, takeover: bool
    ) -> tuple[bool, np.ndarray]:
        """``(intervened, arm action)``; the arm action only matters while intervened."""
        idle = np.zeros(_CARTESIAN_ARM_DIM)
        tracker = self._tracker
        if not requested:
            if tracker.following:
                tracker.hold_at(measured)  # hand-back
            self._needs_new_takeover = False
            self._reanchor_pending = False
            return False, idle
        if takeover or self._reanchor_pending:
            self._take_over(q)
        elif self._needs_new_takeover:
            return False, idle
        try:
            update = tracker.advance(self._requested_pose(q), measured, self._elapsed_s())
            if update.workspace_violations:
                self._needs_new_takeover = True
                logger.warning(
                    "WORKSPACE HOLD: %s; the follower holds its pose, hand back and take over "
                    "again to continue",
                    ", ".join(update.workspace_violations),
                )
                return False, idle
            if update.missed_deadline_s is not None:
                # Treated as an input loss: hold here, then re-anchor the leader as at a take-over.
                self._reanchor_pending = True
                logger.warning(
                    "control loop stalled: %.3f s between two leader commands (max %.3f s); the "
                    "follower holds its pose and the leader is re-anchored at the next poll",
                    update.missed_deadline_s,
                    tracker.config.max_dt_s,
                )
            return True, tracker.action_for(measured, self._pos_scale_m, self._rot_scale_rad)
        except BaseException:
            self._hold_after_failure()
            raise

    def _take_over(self, q: np.ndarray) -> None:
        self._tracker.take_over()
        self._needs_new_takeover = False
        self._reanchor_pending = False
        self._last_advance_s = None
        if self._mapping == "anchored":
            leader = self._kinematics.ee_transform(q)
            command = pose_to_transform(self._tracker.command)
            # p_c0 - p_l0 and R_l0^T R_c0: p* = p_l + (p_c0 - p_l0), R* = R_l (R_l0^T R_c0).
            self._anchor = (command[:3, 3] - leader[:3, 3], leader[:3, :3].T @ command[:3, :3])

    def _requested_pose(self, q: np.ndarray) -> np.ndarray:
        leader = self._kinematics.ee_transform(q)
        position, rotation = leader[:3, 3], leader[:3, :3]
        if self._mapping == "anchored":
            position_offset, rotation_offset = self._anchor
            position, rotation = position + position_offset, rotation @ rotation_offset
        return np.concatenate([position, matrix_to_quat_xyzw(rotation)])

    def _elapsed_s(self) -> float:
        """Seconds since the previous advance; 0 for the first one of a take-over."""
        now = float(self._clock())
        elapsed = 0.0 if self._last_advance_s is None else now - self._last_advance_s
        self._last_advance_s = now
        return elapsed

    def _measured_pose(self) -> np.ndarray:
        pose = self._follower_pose()
        if pose is None:
            raise RuntimeError("leader_cartesian polled before the env was reset (no follower pose yet)")
        return np.asarray(pose, dtype=np.float64)

    def _hold_after_failure(self) -> None:
        """Input or tracking failure: hold the measured pose; following needs a new take-over."""
        if not self._tracker.following:
            return
        pose = self._follower_pose()
        if pose is not None:
            self._tracker.hold_at(pose)
        self._needs_new_takeover = True

    # -- gripper -------------------------------------------------------------------

    def _gripper_action(self, sample: Any, teleop: Any, takeover: bool, requested: bool) -> float:
        if not self._leader_has_gripper:
            return held_gripper_action(self._last_gripper_action)
        if sample.gripper is None:
            raise self._device_fault("leader declares a gripper but its sample carries none", teleop)
        trigger = self._trigger_action(float(sample.gripper))
        if self._last_gripper_action is None:
            return trigger
        if not requested:
            self._gripper_latched = False
            return trigger
        held = held_gripper_action(self._last_gripper_action)
        if takeover and _gripper_open(trigger) != _gripper_open(held):
            self._gripper_latched = True
            logger.info(
                "leader gripper trigger disagrees with the gripper (%s); holding it until the "
                "trigger is %s too",
                "open" if _gripper_open(held) else "closed",
                "open" if _gripper_open(held) else "closed",
            )
        if self._gripper_latched:
            if _gripper_open(trigger) != _gripper_open(held):
                return held
            self._gripper_latched = False
        return trigger

    def _trigger_action(self, width: float) -> float:
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
        from real_world_rl.devices.errors import DeviceFault

        return DeviceFault(
            message,
            hint="check the leader robot yaml's teleop: section (backend gello, action_space "
            "joint_abs with 7 joints)",
            device=getattr(teleop, "name", "leader"),
        )


def _gripper_open(action: float) -> bool:
    return action > 0.0


def follower_pose_from_env(env: Any, contract: Optional[Mapping[str, Any]]) -> tuple[PoseGetter, tuple]:
    """The follower TCP pose getter and ``(pos_scale_m, rot_scale_rad)`` of ``env``."""
    from real_world_rl.envs.franka_real.env import FrankaRealEnv

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
        "the follower's TCP pose is read from a franka_real env (FrankaRealEnv.last_tcp_pose) or "
        "a real_robot env with action_mode cartesian_delta (and tcp_pose in its state_fields); "
        "otherwise pass follower_pose_getter=..."
    )


def _safety_box_from_env(env: Any, contract: Optional[Mapping[str, Any]]) -> Optional[list]:
    """``[xmin, ymin, zmin, xmax, ymax, zmax]`` the env clips commands to, if configured."""
    from real_world_rl.envs.franka_real.env import FrankaRealEnv

    franka_env = _find_in_wrapper_chain(env, FrankaRealEnv)
    if franka_env is not None:
        return [*franka_env.cfg.safety_box_low, *franka_env.cfg.safety_box_high]
    experiment = (contract or {}).get("experiment") or {}
    box = experiment.get("safety_box")
    return None if box is None else list(box)


def _find_in_wrapper_chain(env: Any, env_cls: type) -> Optional[Any]:
    """Walk ``.env`` down the wrapper chain to the first ``env_cls`` instance."""
    node, seen = env, set()
    while node is not None and id(node) not in seen:
        seen.add(id(node))
        if isinstance(node, env_cls):
            return node
        node = vars(node).get("env") if hasattr(node, "__dict__") else None
    return None
