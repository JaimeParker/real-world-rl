"""Hardware-free source returning scripted readings (tests, ``device="fake"``)."""
from __future__ import annotations

from typing import Any, Optional, Sequence

from real_world_rl.action_contract import (
    LEGACY_ACTION_CONTRACT,
    get_env_action_contract,
    num_arms_of,
)
from real_world_rl.teleop.base import TeleopReading, TeleopSource
from real_world_rl.teleop.registry import register_teleop_source


@register_teleop_source("fake")
class ScriptedTeleopSource(TeleopSource):
    """Returns ``readings`` in order and repeats the last one. Counts ``start``/``reset``/
    ``close`` calls (``start_calls``/``reset_calls``/``close_calls``).

    ``device="fake"`` builds one from the env's action contract (its arm count included) with a
    single idle reading of the full action width, so a run exercises the intervention path
    without an operator, on one arm or several; pass ``readings=[...]`` to script one.
    """

    def __init__(
        self,
        readings: Sequence[TeleopReading],
        *,
        action_mode: str = "cartesian_delta",
        arm_action_dim: int = 6,
        has_gripper: bool = True,
        num_arms: int = 1,
    ) -> None:
        if not readings:
            raise ValueError("ScriptedTeleopSource needs at least one reading")
        self._readings = list(readings)
        self._next = 0
        self._action_mode = action_mode
        self._arm_action_dim = arm_action_dim
        self._has_gripper = has_gripper
        self._num_arms = int(num_arms)
        self.start_calls = 0
        self.reset_calls = 0
        self.close_calls = 0

    @classmethod
    def from_env(
        cls, env: Any, *, readings: Optional[Sequence[TeleopReading]] = None, **kwargs: Any
    ) -> "ScriptedTeleopSource":
        contract = get_env_action_contract(env) or LEGACY_ACTION_CONTRACT
        declared = dict(
            action_mode=contract["action_mode"],
            arm_action_dim=int(contract["arm_action_dim"]),
            has_gripper=bool(contract["gripper_in_action"]),
            num_arms=num_arms_of(contract),
        )
        declared.update(kwargs)
        if readings is None:
            per_arm_dim = declared["arm_action_dim"] + int(declared["has_gripper"])
            readings = [TeleopReading.idle(declared["num_arms"] * per_arm_dim)]
        return cls(readings, **declared)

    @property
    def action_mode(self) -> str:
        return self._action_mode

    @property
    def arm_action_dim(self) -> int:
        return self._arm_action_dim

    @property
    def has_gripper(self) -> bool:
        return self._has_gripper

    @property
    def num_arms(self) -> int:
        return self._num_arms

    def start(self) -> None:
        self.start_calls += 1

    def poll(self) -> TeleopReading:
        reading = self._readings[min(self._next, len(self._readings) - 1)]
        self._next += 1
        return reading

    def reset(self) -> None:
        self.reset_calls += 1

    def close(self) -> None:
        self.close_calls += 1
