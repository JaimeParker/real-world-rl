"""GELLO leader arm opened locally through the device layer (``device="gello"``)."""
from __future__ import annotations

from typing import Any, Callable, Mapping, Optional, Sequence

from robot_infra.devices.teleop import Teleop, TeleopConfig

from rl_garden_real_world.action_contract import (
    LEGACY_ACTION_CONTRACT,
    get_env_action_contract,
)
from rl_garden_real_world.teleop.base import TeleopReading
from rl_garden_real_world.teleop.composite import ToggleableTeleopSource
from rl_garden_real_world.teleop.leader_arm import (
    DEFAULT_ARM_TOLERANCE_RAD,
    DEFAULT_RAMP_STEP_RAD,
    LeaderArmPartSource,
)
from rl_garden_real_world.teleop.part import (
    DEFAULT_READ_TIMEOUT_S,
    _find_last_observation_owner,
)
from rl_garden_real_world.teleop.registry import register_teleop_source

_BACKEND = "gello"


def _gello_teleop_class() -> type:
    """The device layer's ``gello`` teleop backend (its Dynamixel SDK import is lazy)."""
    if _BACKEND not in Teleop.backends():
        import robot_infra.devices.backends.teleops.gello  # noqa: F401  (registers "gello")
    return Teleop.backend(_BACKEND)


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
    robot_infra.devices.backends.teleops.gello_calibrate`` prints them for a unit.

    The SDK is imported when ``start()`` opens the device (the intervention wrapper calls it), so
    building the source works without it. ``close()`` disconnects the device.
    """

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
        last_gripper_action_getter: Optional[Callable[[], float]] = None,
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
