"""GELLO leader arm opened locally through the device layer (``device="gello"``)."""
from __future__ import annotations

from typing import Any, Callable, Mapping, Optional, Sequence

from real_world_rl.devices.teleop import Teleop, TeleopConfig

from real_world_rl.action_contract import (
    LEGACY_ACTION_CONTRACT,
    get_env_action_contract,
)
from real_world_rl.teleop.core.contract import TeleopReading
from real_world_rl.teleop.core.composite import ToggleableTeleopSource
from real_world_rl.teleop.leader.joint import (
    DEFAULT_ARM_TOLERANCE_RAD,
    DEFAULT_RAMP_STEP_RAD,
    LeaderArmPartSource,
)
from real_world_rl.teleop.core.part import (
    DEFAULT_READ_TIMEOUT_S,
    _find_last_observation_owner,
)
from real_world_rl.teleop.core.registry import register_teleop_source

_BACKEND = "gello"


def _gello_teleop_class() -> type:
    """The device layer's ``gello`` teleop backend (its Dynamixel SDK import is lazy)."""
    if _BACKEND not in Teleop.backends():
        import real_world_rl.teleop.gello.backend  # noqa: F401  (registers "gello")
    return Teleop.backend(_BACKEND)


def station_teleop_options(path: Optional[str], robot_name: str, robot: Any) -> dict:
    """A station robot's ``teleop:`` section as local GELLO source kwargs: ``options`` plus
    ``hold_s``. ``path`` names the station yaml in errors."""
    teleop = robot.get("teleop") if isinstance(robot, dict) else None
    if not isinstance(teleop, dict):
        raise ValueError(f"--teleop-options-yaml {path}: station robot {robot_name!r} has no teleop: section")
    backend = teleop.get("backend")
    if backend != _BACKEND:
        raise ValueError(
            f"--teleop-options-yaml {path}: station robot {robot_name!r} has a {backend!r} teleop; "
            f"only a {_BACKEND!r} section configures a local GELLO"
        )
    options = dict(teleop.get("options") or {})
    if "hold_s" in teleop:
        options["hold_s"] = teleop["hold_s"]
    return options


@register_teleop_source("gello")
class GelloTeleopSource(ToggleableTeleopSource):
    """A GELLO on this machine, read by the device layer's ``gello`` teleop (a poll thread over
    the Dynamixel bus feeding ``read()``) and mapped into the env's joint action space by
    ``LeaderArmPartSource`` with its ``onset`` policy.

    The device options (``port``, ``baudrate``, ``joint_ids``, ``joint_offsets_rad``,
    ``joint_signs``, ``wrap_centers_rad``, ``gripper_id``, ``gripper_open_rad``,
    ``gripper_closed_rad``, ``gripper_binarize_threshold``, ``intervention``, ``event_device``,
    ``event_keys``, ``poll_interval_s``) go to the backend unchanged; the optional ones are only
    sent when given, so the backend's defaults apply otherwise. ``python -m
    real_world_rl.teleop.gello.calibrate`` prints them for a unit.

    The SDK is imported when ``start()`` opens the device (the intervention wrapper calls it), so
    building the source works without it. ``close()`` disconnects the device.
    """

    USES_LAST_GRIPPER_ACTION = True
    DRIVES_ONE_ARM = True

    @classmethod
    def options_from_station(
        cls,
        station_path: str,
        station: Mapping[str, Any],
        *,
        robot: Optional[str] = None,
        profile: Optional[str] = None,
        provenance: Optional[dict] = None,
    ) -> dict:
        """The station robot's ``gello`` teleop section (``station_teleop_options``): the robot
        of a single-robot station yaml, or the only one of its ``robots:``. The station fixes the
        robot, so ``robot`` and ``profile`` are not used."""
        robots = station.get("robots")
        if not isinstance(robots, dict):
            return station_teleop_options(station_path, str(station.get("name") or "robot"), station)
        if len(robots) != 1:
            raise ValueError(
                f"--teleop-options-yaml {station_path} lists robots {sorted(robots)}; --teleop-device "
                f"{_BACKEND} drives one arm (use --teleop-device bimanual "
                f"--teleop-arm-device {_BACKEND} for two)"
            )
        ((name, robot_section),) = robots.items()
        return station_teleop_options(station_path, name, robot_section)

    def __init__(
        self,
        *,
        port: str,
        joint_ids: Sequence[int],
        joint_offsets_rad: Sequence[float],
        joint_signs: Sequence[int],
        baudrate: Optional[int] = None,
        wrap_centers_rad: Optional[Sequence[float]] = None,
        gripper_id: Optional[int] = None,
        gripper_open_rad: Optional[float] = None,
        gripper_closed_rad: Optional[float] = None,
        gripper_binarize_threshold: Optional[float] = None,
        intervention: Optional[str] = None,
        event_device: Optional[str] = None,
        event_keys: Optional[Mapping[str, str]] = None,
        poll_interval_s: Optional[float] = None,
        hold_s: float = 0.0,
        read_timeout_s: float = DEFAULT_READ_TIMEOUT_S,
        env_contract: Optional[Mapping[str, Any]] = None,
        joint_limits: Optional[Sequence[Sequence[float]]] = None,
        joint_scale_rad: Optional[float] = None,
        last_observation_getter: Optional[Callable[[], Optional[dict]]] = None,
        last_gripper_action_getter: Optional[Callable[[], Any]] = None,
        onset: str = "ramp",
        arm_tolerance_rad: float = DEFAULT_ARM_TOLERANCE_RAD,
        ramp_step_rad: float = DEFAULT_RAMP_STEP_RAD,
    ) -> None:
        options: dict = {
            "port": port,
            "joint_ids": list(joint_ids),
            "joint_offsets_rad": list(joint_offsets_rad),
            "joint_signs": list(joint_signs),
        }
        optional = {
            "baudrate": baudrate,
            "wrap_centers_rad": None if wrap_centers_rad is None else list(wrap_centers_rad),
            "gripper_id": gripper_id,
            "gripper_open_rad": gripper_open_rad,
            "gripper_closed_rad": gripper_closed_rad,
            "gripper_binarize_threshold": gripper_binarize_threshold,
            "intervention": intervention,
            "event_device": event_device,
            "event_keys": None if event_keys is None else dict(event_keys),
            "poll_interval_s": poll_interval_s,
        }
        options.update({key: value for key, value in optional.items() if value is not None})
        config = TeleopConfig(name=_BACKEND, backend=_BACKEND, hold_s=hold_s, options=options)
        self._part = _gello_teleop_class()(config, read_timeout_s=read_timeout_s)
        self._mapped = LeaderArmPartSource(
            self._part,
            env_contract=LEGACY_ACTION_CONTRACT if env_contract is None else env_contract,
            joint_limits=joint_limits,
            joint_scale_rad=joint_scale_rad,
            last_observation_getter=last_observation_getter,
            last_gripper_action_getter=last_gripper_action_getter,
            read_timeout_s=read_timeout_s,
            onset=onset,
            arm_tolerance_rad=arm_tolerance_rad,
            ramp_step_rad=ramp_step_rad,
        )

    @classmethod
    def from_env(cls, env: Any, **kwargs: Any) -> "GelloTeleopSource":
        """Fills the env side from ``env``: its action contract (with ``joint_limits`` and
        ``joint_scale_rad``) and the ``RealRobotEnv``'s latest observation."""
        contract = get_env_action_contract(env)
        kwargs.setdefault("env_contract", contract)
        if contract is not None:
            kwargs.setdefault("joint_limits", contract.get("joint_limits"))
            kwargs.setdefault("joint_scale_rad", contract.get("joint_scale_rad"))
        owner = _find_last_observation_owner(env)
        if owner is not None:
            kwargs.setdefault("last_observation_getter", lambda: owner.last_observation)
        return cls(**kwargs)

    @property
    def action_mode(self) -> str:
        return self._mapped.action_mode

    @property
    def arm_action_dim(self) -> int:
        return self._mapped.arm_action_dim

    @property
    def has_gripper(self) -> bool:
        return self._mapped.has_gripper

    def start(self) -> None:
        self._part.connect()

    def poll(self) -> TeleopReading:
        return self._mapped.poll()

    def poll_with_toggle(self, toggle: bool) -> TeleopReading:
        return self._mapped.poll_with_toggle(toggle)

    def reset(self) -> None:
        self._mapped.reset()

    def close(self) -> None:
        self._part.disconnect()
