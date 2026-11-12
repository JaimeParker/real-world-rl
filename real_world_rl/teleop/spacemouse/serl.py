"""HIL-SERL SpaceMouse intervention: raw base-frame twist and button levels."""
from __future__ import annotations

import logging
from typing import Any, Mapping, Optional, Sequence

import numpy as np

from real_world_rl.teleop.core.targets.twist_input import rotate_input, diagnostic_input, input_sample_diagnostics

from real_world_rl.action_contract import (
    LEGACY_ACTION_CONTRACT,
    get_env_action_contract,
    num_arms_of,
)
from real_world_rl.teleop.core.contract import TeleopReading, TeleopSource
from real_world_rl.teleop.core.part import DEFAULT_READ_TIMEOUT_S, teleop_contract_of
from real_world_rl.teleop.spacemouse.options import SpaceMouseOptions

# The teleop_control block's version string (compared on resume; never change it).
SERL_CONTROL_VERSION = "spacemouse_serl_v1"
MOTION_THRESHOLD = 0.001
# These belong to the tracker path, never to native SERL intervention.
TRACKER_OPTIONS = frozenset({
    "follower_pose_getter", "last_gripper_action_getter", "pos_scale_m", "rot_scale_rad",
    "lead_cap_m", "lead_cap_rad", "target_speed_m_s", "target_speed_rad_s",
    "max_dt_s", "safety_box", "clock", "deadzone", "hold_s", "tracker",
    "cartesian_tracker",
})

logger = logging.getLogger(__name__)


class SpaceMouseSerlTeleopSource(SpaceMouseOptions, TeleopSource):
    """Sample raw axes and levels each step, without toggle, deadzone or hold target.

    A caller-owned ``teleop`` is never connected or disconnected here. Its latched
    events and sticky gripper/intervention state have no control meaning in SERL.
    Built by ``device="spacemouse"`` with ``mode="serl"``.
    """

    REQUIRES_FULL_ACTION = True

    @classmethod
    def control_spec(cls, *, tracker_options, source_options) -> dict:
        """The serl semantics: motion threshold, button gripper, inverted axes and the compared
        input frame (the diagnostics filter)."""
        options = source_options()
        spec = {"version": SERL_CONTROL_VERSION, "mode": "serl",
                "motion_threshold": MOTION_THRESHOLD, "gripper": "serl_buttons"}
        if options.get("invert_axes"):
            spec["invert_axes"] = list(options["invert_axes"])
        spec.update(cls.compared_input_frame(options))
        return spec

    def __init__(
        self,
        teleop: Optional[Any] = None,
        *,
        env_contract: Optional[Mapping[str, Any]] = None,
        device_index: int = 0,
        invert_axes: Sequence[int] = (),
        input_rotation=None,
        expected_device_identity=None,
        diagnostic_axes="all",
        diagnostic_gain=1.,
        poll_interval_s: float = 0.005,
        device_table: Optional[str] = None,
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
            from real_world_rl.teleop.spacemouse import backend  # noqa: F401
            from real_world_rl.devices.teleop import Teleop, TeleopConfig

            teleop = Teleop.backend("spacemouse")(TeleopConfig(
                name="spacemouse", backend="spacemouse", options={
                    "device_index": device_index, "invert_axes": list(invert_axes),
                    "poll_interval_s": poll_interval_s,
                    **({} if device_table is None else {"device_table": str(device_table)}),
                },
            ), read_timeout_s=read_timeout_s)
        else:
            declared = teleop_contract_of(teleop)
            if (declared.action_mode, declared.arm_action_dim) != ("cartesian_delta", 6):
                raise ValueError("spacemouse serl mode requires a cartesian_delta teleop part with 6 axes")
        self._part = teleop
        from real_world_rl.teleop.core.targets.twist_input import validate_input_rotation
        self._input_rotation = validate_input_rotation(input_rotation)
        diagnostic_input(np.zeros(6), diagnostic_axes, diagnostic_gain)
        self._diagnostic_axes, self._diagnostic_gain = diagnostic_axes, diagnostic_gain
        from real_world_rl.teleop.spacemouse.profiles import validate_device_identity
        self._expected_device_identity = validate_device_identity(expected_device_identity)

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
        from real_world_rl.teleop.spacemouse.profiles import start_profiled_source
        start_profiled_source(self)

    def session_input(self):
        """Consume fresh device input without advancing the motion target."""
        sample = self._part.read(timeout_s=self._read_timeout_s)
        return sample.action, sample.buttons

    def reset(self) -> None:
        from real_world_rl.devices.errors import DeviceNotConnected

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
        action = diagnostic_input(rotate_input(twist, self._input_rotation),
                                  self._diagnostic_axes, self._diagnostic_gain)
        diagnostics = input_sample_diagnostics(sample, action)
        if self.has_gripper:
            if left:
                gripper = self._rng.uniform(-1.0, -0.9, size=(1,))
                intervened = True
            elif right:
                gripper = self._rng.uniform(0.9, 1.0, size=(1,))
                intervened = True
            else:
                gripper = np.zeros(1)
            action = np.concatenate((action, gripper))
        return TeleopReading(
            diagnostics=diagnostics,
            action=action, intervened=intervened,
            timestamp=float(sample.timestamp), buttons={"left": left, "right": right},
        )
