"""One single-arm teleop source per arm acting as one source for a two-arm env
(``device="bimanual"``).

``BimanualTeleopSource`` holds a child source per arm, in the env's ``arm_names`` order, and
assembles their readings into the action-contract layout (``action_contract``)
``[arm_0 block, ..., arm_{n-1} block, gripper_0, ..., gripper_{n-1}]``. The operator overrides
the whole action with one ``intervened`` flag: true when any child intervenes
(``intervention_from="any"``, the default) or only when every child does (``"all"``).

Under ``"any"``, an arm whose child is not intervening while another child is holds still: its
arm block is the arm's hold action (``joint_abs``: the follower's ``joint_pos`` normalised onto
that arm's joint limits; ``joint_delta`` and ``cartesian_delta``: zero) and its gripper the
action last sent to it. An idle leader therefore never moves its follower, and an idle trigger
never opens a gripper that holds an object.

``from_env`` builds the children by registry name (``device_per_arm``) over each arm's own
``RealRobotEnv`` of a ``BimanualRealRobotEnv`` (``env.arms[arm]``), so every child reads its own
arm's follower and robot:

- ``leader_arm`` (default): the arm robot's teleop part read as a joint-space leader arm (a GELLO
  served by that arm's daemon), with the onset policy;
- ``robot``: the arm robot's teleop part, mapped as ``TeleopPartSource`` does;
- ``gello``: a GELLO plugged into this machine, one per arm (``per_arm_kwargs[arm]`` holds its
  ``port`` and calibration).

``leader_cartesian`` is not supported: it turns one leader into the twist of one cartesian arm.
"""
from __future__ import annotations

import contextlib
from typing import Any, Callable, Iterable, Mapping, Optional, Sequence

import numpy as np

from real_world_rl.action_contract import get_env_action_contract
from real_world_rl.teleop.base import (
    TeleopReading,
    TeleopSource,
    held_gripper_action,
)
from real_world_rl.teleop.composite import ToggleableTeleopSource
from real_world_rl.teleop.registry import (
    make_teleop_source,
    register_teleop_source,
)

__all__ = [
    "ARM_DEVICES",
    "INTERVENTION_FROM",
    "BimanualTeleopSource",
    "find_bimanual_env",
    "normalized_joint_target",
]

ARM_DEVICES = ("robot", "leader_arm", "gello")
INTERVENTION_FROM = ("any", "all")
_JOINT_ABS = "joint_abs"

# Zero-arg callable returning one arm's hold block (``arm_action_dim`` values).
ArmHoldAction = Callable[[], Sequence[float]]


def normalized_joint_target(joint_pos: Sequence[float], joint_limits: Sequence[Sequence[float]]) -> np.ndarray:
    """The ``joint_abs`` arm action commanding ``joint_pos``: normalised onto ``joint_limits``
    (``[[low, high]]`` per joint) and clipped to ``[-1, 1]``."""
    low, high = np.asarray(joint_limits, dtype=np.float64).T
    q = np.asarray(joint_pos, dtype=np.float64)
    return np.clip(2.0 * (q - low) / (high - low) - 1.0, -1.0, 1.0)


def find_bimanual_env(env: Any) -> Optional[Any]:
    """The ``BimanualRealRobotEnv`` behind ``env`` (walking ``.env`` down the wrapper chain), or
    ``None``."""
    from real_world_rl.envs.bimanual.env import BimanualRealRobotEnv

    node, seen = env, set()
    while node is not None and id(node) not in seen:
        seen.add(id(node))
        if isinstance(node, BimanualRealRobotEnv):
            return node
        node = vars(node).get("env") if hasattr(node, "__dict__") else None
    return None


@register_teleop_source("bimanual")
class BimanualTeleopSource(ToggleableTeleopSource):
    """One child source per arm (``sources``, in arm order) as one source; see the module
    docstring for the assembled action and the intervention rule.

    The children must each drive one arm and declare the same ``action_mode``,
    ``arm_action_dim`` and ``has_gripper``. ``last_gripper_action_getter`` returns the gripper
    actions last sent to the env, one per arm (``InterventionWrapper`` keeps them); it holds the
    gripper of an idle arm and pads the gripper dims when the children declare none.
    ``arm_hold_actions`` maps every arm to its hold block; it may be omitted in the delta modes
    (zero) and must be given in ``joint_abs`` with ``intervention_from="any"`` (``from_env``
    fills it from each arm's follower).

    ``events`` carry every child's fired event twice: as ``<arm>_<event>`` and as the plain
    ``<event>`` (fired on any arm); ``episode_end`` is ORed over the children.
    ``poll_with_toggle`` hands the toggle to every ``ToggleableTeleopSource`` child (so the
    onset policy runs per arm) and uses it as the ``intervened`` flag of the other children, so
    ``CompositeTeleopSource(motion=<this>, events=<keyboard>)`` toggles both arms at once.
    """

    def __init__(
        self,
        sources: Mapping[str, TeleopSource],
        *,
        intervention_from: str = "any",
        last_gripper_action_getter: Optional[Callable[[], Any]] = None,
        arm_hold_actions: Optional[Mapping[str, ArmHoldAction]] = None,
    ) -> None:
        if intervention_from not in INTERVENTION_FROM:
            raise ValueError(
                f"intervention_from must be one of {list(INTERVENTION_FROM)}, got {intervention_from!r}"
            )
        if not sources:
            raise ValueError("BimanualTeleopSource needs one teleop source per arm, got none")
        self.sources: dict[str, TeleopSource] = {str(arm): source for arm, source in sources.items()}
        multi_arm = [arm for arm, source in self.sources.items() if int(getattr(source, "num_arms", 1)) != 1]
        if multi_arm:
            raise ValueError(f"every arm's teleop source must drive one arm; {multi_arm} drive several")
        declarations = {
            arm: (source.action_mode, int(source.arm_action_dim), bool(source.has_gripper))
            for arm, source in self.sources.items()
        }
        if len(set(declarations.values())) != 1:
            described = ", ".join(
                f"{arm}: action_mode={mode!r} arm_action_dim={dim} has_gripper={gripper}"
                for arm, (mode, dim, gripper) in declarations.items()
            )
            raise ValueError(f"the arms' teleop sources must declare the same action space; got {described}")
        self._action_mode, self._arm_action_dim, self._children_have_gripper = next(
            iter(declarations.values())
        )
        self._intervention_from = intervention_from
        self._last_gripper_action = last_gripper_action_getter
        self._arm_hold_actions = self._checked_hold_actions(arm_hold_actions)

    def _checked_hold_actions(self, arm_hold_actions: Optional[Mapping[str, ArmHoldAction]]) -> dict:
        if arm_hold_actions is not None:
            given = {str(arm): hold for arm, hold in arm_hold_actions.items()}
            if set(given) != set(self.sources):
                raise ValueError(
                    f"arm_hold_actions names arms {sorted(given)}, the sources are for {list(self.sources)}"
                )
            return given
        if self._action_mode == _JOINT_ABS and self._intervention_from == "any" and len(self.sources) > 1:
            raise ValueError(
                "joint_abs with intervention_from='any' holds an idle arm at its follower's joints, "
                "so it needs arm_hold_actions (from_env builds them from each arm's follower)"
            )
        zeros = np.zeros(self._arm_action_dim)
        return {arm: (lambda: zeros) for arm in self.sources}

    @classmethod
    def from_env(
        cls,
        env: Any,
        *,
        device_per_arm: str = "leader_arm",
        per_arm_kwargs: Optional[Mapping[str, Mapping[str, Any]]] = None,
        common_kwargs: Optional[Mapping[str, Any]] = None,
        intervention_from: str = "any",
        last_gripper_action_getter: Optional[Callable[[], Any]] = None,
    ) -> "BimanualTeleopSource":
        """Builds one ``device_per_arm`` child per arm of the ``BimanualRealRobotEnv`` behind
        ``env``, with ``make_teleop_source(device_per_arm, env.arms[arm], **kwargs)``.

        Each child's kwargs are ``common_kwargs`` updated by ``per_arm_kwargs[arm]``, plus the
        arm contract's ``joint_limits``/``joint_scale_rad`` and a view of
        ``last_gripper_action_getter`` returning that arm's entry, unless given.
        """
        if device_per_arm not in ARM_DEVICES:
            raise ValueError(
                f"device_per_arm must be one of {list(ARM_DEVICES)}, got {device_per_arm!r} "
                "(leader_cartesian drives a single cartesian arm)"
            )
        bimanual = find_bimanual_env(env)
        if bimanual is None:
            raise ValueError(
                "device='bimanual' needs a real_robot_bimanual env (BimanualRealRobotEnv) behind "
                "the wrapper chain"
            )
        arm_names = list(bimanual.arm_names)
        per_arm_kwargs = {str(arm): dict(kwargs or {}) for arm, kwargs in (per_arm_kwargs or {}).items()}
        unknown = sorted(set(per_arm_kwargs) - set(arm_names))
        if unknown:
            raise ValueError(f"per_arm_kwargs names unknown arms {unknown}; the env's arms are {arm_names}")

        sources: dict[str, TeleopSource] = {}
        holds: dict[str, ArmHoldAction] = {}
        for index, arm in enumerate(arm_names):
            arm_env = bimanual.arms[arm]
            arm_contract = get_env_action_contract(arm_env)
            kwargs = {**dict(common_kwargs or {}), **per_arm_kwargs.get(arm, {})}
            kwargs.setdefault("joint_limits", arm_contract.get("joint_limits"))
            kwargs.setdefault("joint_scale_rad", arm_contract.get("joint_scale_rad"))
            if last_gripper_action_getter is not None:
                kwargs.setdefault(
                    "last_gripper_action_getter", _arm_gripper_view(last_gripper_action_getter, index)
                )
            sources[arm] = make_teleop_source(device_per_arm, arm_env, **kwargs)
            if intervention_from == "any":
                holds[arm] = _follower_hold_action(arm, arm_env, arm_contract)
        return cls(
            sources,
            intervention_from=intervention_from,
            last_gripper_action_getter=last_gripper_action_getter,
            arm_hold_actions=holds or None,
        )

    # -- declaration -------------------------------------------------------------------

    @property
    def action_mode(self) -> str:
        return self._action_mode

    @property
    def arm_action_dim(self) -> int:
        return self._arm_action_dim

    @property
    def has_gripper(self) -> bool:
        return self._children_have_gripper or self._last_gripper_action is not None

    @property
    def num_arms(self) -> int:
        return len(self.sources)

    @property
    def arm_names(self) -> list[str]:
        return list(self.sources)

    @property
    def intervention_from(self) -> str:
        return self._intervention_from

    # -- source protocol ---------------------------------------------------------------

    def start(self) -> None:
        """Starts every child in arm order; when one fails, the ones already started are closed."""
        started: list[TeleopSource] = []
        try:
            for source in self.sources.values():
                source.start()
                started.append(source)
        except BaseException:
            for source in reversed(started):
                with contextlib.suppress(Exception):
                    source.close()
            raise

    def poll(self) -> TeleopReading:
        """Polls every child; each child's own intervention signal decides its arm."""
        readings = [source.poll() for source in self.sources.values()]
        return self._assemble(readings, [bool(reading.intervened) for reading in readings])

    def poll_with_toggle(self, toggle: bool) -> TeleopReading:
        readings, flags = [], []
        for source in self.sources.values():
            if isinstance(source, ToggleableTeleopSource):
                reading = source.poll_with_toggle(bool(toggle))
                flags.append(bool(reading.intervened))
            else:
                reading = source.poll()
                flags.append(bool(toggle))
            readings.append(reading)
        return self._assemble(readings, flags)

    def reset(self) -> None:
        """Resets every child; the first failure is raised after all were reset."""
        _call_on_every_source(self.sources.values(), "reset")

    def close(self) -> None:
        """Closes every child; the first failure is raised after all were closed."""
        _call_on_every_source(self.sources.values(), "close")

    # -- assembly ----------------------------------------------------------------------

    def _assemble(self, readings: Sequence[TeleopReading], arm_intervened: Sequence[bool]) -> TeleopReading:
        combine = any if self._intervention_from == "any" else all
        intervened = combine(arm_intervened)
        child_dim = self._arm_action_dim + int(self._children_have_gripper)
        arm_blocks: list[np.ndarray] = []
        grippers: list[float] = []
        events: dict[str, bool] = {}
        for index, (arm, reading, arm_active) in enumerate(zip(self.sources, readings, arm_intervened)):
            action = np.asarray(reading.action, dtype=np.float64)
            if action.shape != (child_dim,):
                raise ValueError(f"arm {arm!r}: teleop reading has shape {action.shape}, expected ({child_dim},)")
            held = intervened and not arm_active
            arm_blocks.append(self._hold_block(arm) if held else action[: self._arm_action_dim])
            if self.has_gripper:
                grippers.append(self._gripper_action(index, action, held))
            for event, fired in reading.events.items():
                if fired:
                    events[f"{arm}_{event}"] = True
                    events[event] = True
        parts = arm_blocks + ([np.asarray(grippers, dtype=np.float64)] if self.has_gripper else [])
        return TeleopReading(
            action=np.concatenate(parts).astype(np.float32),
            intervened=bool(intervened),
            episode_end=any(bool(reading.episode_end) for reading in readings),
            events=events,
            timestamp=max(float(reading.timestamp) for reading in readings),
        )

    def _hold_block(self, arm: str) -> np.ndarray:
        block = np.asarray(self._arm_hold_actions[arm](), dtype=np.float64).reshape(-1)
        if block.shape != (self._arm_action_dim,):
            raise ValueError(
                f"arm {arm!r}: hold action has shape {block.shape}, expected ({self._arm_action_dim},)"
            )
        return block

    def _gripper_action(self, arm_index: int, action: np.ndarray, held: bool) -> float:
        """The child's gripper, unless the arm is held (or the child has none): then the gripper
        action last sent to that arm."""
        uses_child = self._children_have_gripper and (not held or self._last_gripper_action is None)
        if uses_child:
            return float(action[-1])
        return held_gripper_action(self._last_gripper_action, arm_index)


def _arm_gripper_view(last_gripper_action_getter: Callable[[], Any], arm_index: int) -> Callable[[], float]:
    """A single-arm getter over one entry of the per-arm gripper actions."""
    return lambda: held_gripper_action(last_gripper_action_getter, arm_index)


def _follower_hold_action(arm: str, arm_env: Any, arm_contract: Mapping[str, Any]) -> ArmHoldAction:
    """The block that keeps ``arm`` still: zero in the delta modes; in ``joint_abs``, the arm's
    latest observed ``joint_pos`` normalised onto its ``joint_limits``."""
    if arm_contract["action_mode"] != _JOINT_ABS:
        zeros = np.zeros(int(arm_contract["arm_action_dim"]))
        return lambda: zeros
    joint_limits = arm_contract.get("joint_limits")
    if joint_limits is None:
        raise ValueError(f"arm {arm!r}: joint_abs needs joint_limits to hold the arm while the other arm intervenes")
    state_fields = tuple(arm_contract.get("state_fields") or ())
    if state_fields and "joint_pos" not in state_fields:
        raise ValueError(
            f"arm {arm!r}: holding a joint_abs arm reads joint_pos from its observation, but "
            f"state_fields is {list(state_fields)}; add joint_pos to the arm's state_fields"
        )

    def hold() -> np.ndarray:
        observation = arm_env.last_observation
        if observation is None:
            raise RuntimeError(f"arm {arm!r}: teleop source polled before the env was reset")
        return normalized_joint_target(observation["state"]["joint_pos"], joint_limits)

    return hold


def _call_on_every_source(sources: Iterable[TeleopSource], method: str) -> None:
    first_error: Optional[BaseException] = None
    for source in sources:
        try:
            getattr(source, method)()
        except Exception as error:  # re-raised below, once every source was called
            if first_error is None:
                first_error = error
    if first_error is not None:
        raise first_error
