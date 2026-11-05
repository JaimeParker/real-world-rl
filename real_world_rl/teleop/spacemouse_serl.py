"""HIL-SERL SpaceMouse intervention: raw base-frame twist and button levels."""
from __future__ import annotations

import logging
from typing import Any, Mapping, Optional, Sequence

import numpy as np

from real_world_rl.action_contract import (
    LEGACY_ACTION_CONTRACT,
    get_env_action_contract,
    num_arms_of,
)
from real_world_rl.teleop.base import TeleopReading, TeleopSource
from real_world_rl.teleop.part import DEFAULT_READ_TIMEOUT_S, teleop_contract_of

MOTION_THRESHOLD = 0.001
# These belong to the tracker path, never to native SERL intervention.
TRACKER_OPTIONS = frozenset({
    "follower_pose_getter", "last_gripper_action_getter", "pos_scale_m", "rot_scale_rad",
    "lead_cap_m", "lead_cap_rad", "target_speed_m_s", "target_speed_rad_s",
    "max_dt_s", "safety_box", "clock", "deadzone", "hold_s", "tracker",
    "cartesian_tracker",
})

logger = logging.getLogger(__name__)


class SpaceMouseSerlTeleopSource(TeleopSource):
    """Sample raw axes and levels each step, without toggle, deadzone or hold target.

    A caller-owned ``teleop`` is never connected or disconnected here. Its latched
    events and sticky gripper/intervention state have no control meaning in SERL.
    """

    def __init__(
        self,
        teleop: Optional[Any] = None,
        *,
        env_contract: Optional[Mapping[str, Any]] = None,
        device_index: int = 0,
        invert_axes: Sequence[int] = (),
        poll_interval_s: float = 0.005,
        read_timeout_s: float = DEFAULT_READ_TIMEOUT_S,
        rng: Optional[np.random.Generator] = None,
        **kwargs: Any,
    ) -> None:
        if kwargs:
            raise TypeError(
                "spacemouse serl mode does not accept tracker options or "
                f"last_gripper_action_getter (unsupported: {sorted(kwargs)})"
            )
        contract = LEGACY_ACTION_CONTRACT if env_contract is None else env_contract
        if (contract["action_mode"] != "cartesian_delta"
                or int(contract["arm_action_dim"]) != 6 or num_arms_of(contract) != 1):
            raise ValueError("spacemouse serl mode requires one cartesian_delta arm with 6 axes")
        self._env_gripper = bool(contract["gripper_in_action"])
        self._read_timeout_s = read_timeout_s
        self._rng = np.random.default_rng() if rng is None else rng
        self._owns_part = teleop is None
        self._previous_event_toggle = False
        if teleop is None:
            from robot_infra.devices.backends.teleops import spacemouse  # noqa: F401
            from robot_infra.devices.teleop import Teleop, TeleopConfig

            teleop = Teleop.backend("spacemouse")(TeleopConfig(
                name="spacemouse", backend="spacemouse", options={
                    "device_index": device_index, "invert_axes": list(invert_axes),
                    "poll_interval_s": poll_interval_s,
                },
            ), read_timeout_s=read_timeout_s)
        else:
            declared = teleop_contract_of(teleop)
            if (declared.action_mode, declared.arm_action_dim) != ("cartesian_delta", 6):
                raise ValueError("spacemouse serl mode requires a cartesian_delta teleop part with 6 axes")
        self._part = teleop

    @classmethod
    def from_env(cls, env: Any, **kwargs: Any) -> "SpaceMouseSerlTeleopSource":
        kwargs.setdefault("env_contract", get_env_action_contract(env) or LEGACY_ACTION_CONTRACT)
        return cls(**kwargs)

    @property
    def action_mode(self) -> str:
        return "cartesian_delta"

    @property
    def arm_action_dim(self) -> int:
        return 6

    @property
    def has_gripper(self) -> bool:
        return self._env_gripper

    def start(self) -> None:
        if self._owns_part:
            self._part.connect()

    def reset(self) -> None:
        from robot_infra.devices.errors import DeviceNotConnected

        self._previous_event_toggle = False
        try:
            self._part.reset()
        except DeviceNotConnected:
            return

    def close(self) -> None:
        if self._owns_part:
            self._part.disconnect()

    def ignore_event_toggle(self, toggle: bool) -> None:
        """Warn on changed t/h state, as the tracker source does for a composite."""
        toggle = bool(toggle)
        if toggle != self._previous_event_toggle:
            logger.warning(
                "the SpaceMouse serl mode ignores the event device's take-over/hand-back "
                "keys (t/h); release the puck and gripper buttons to hand back"
            )
        self._previous_event_toggle = toggle

    def poll(self) -> TeleopReading:
        sample = self._part.read(timeout_s=self._read_timeout_s)
        twist = np.asarray(sample.action)
        if twist.shape != (6,) or not np.all(np.isfinite(twist)):
            raise ValueError("spacemouse serl mode requires a finite raw six-axis sample")
        left = bool(sample.buttons.get("button_0", False))
        right = bool(sample.buttons.get("button_1", False))
        intervened = bool(np.linalg.norm(twist) > MOTION_THRESHOLD)
        action = twist
        if self.has_gripper:
            if left:
                gripper = self._rng.uniform(-1.0, -0.9, size=(1,))
                intervened = True
            elif right:
                gripper = self._rng.uniform(0.9, 1.0, size=(1,))
                intervened = True
            else:
                gripper = np.zeros(1)
            action = np.concatenate((twist, gripper))
        return TeleopReading(
            action=action, intervened=intervened,
            timestamp=float(sample.timestamp), buttons={"left": left, "right": right},
        )
