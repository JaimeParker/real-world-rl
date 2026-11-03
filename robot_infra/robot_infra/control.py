"""Torch-free impedance settings supported by the arm control contract.

Omitted settings keep the backend/SDK defaults. Configuration is applied while
IDLE and takes effect when the next servo tracker starts.
"""
from __future__ import annotations

import dataclasses
import math
from dataclasses import dataclass
from typing import Any, Mapping


@dataclass(frozen=True)
class ArmControlConfig:
    translational_stiffness: float | None = None
    rotational_stiffness: float | None = None
    cartesian_damping: list[float] | None = None
    joint_stiffness: list[float] | None = None
    joint_damping: list[float] | None = None
    translational_error_clip: list[float] | None = None
    rotational_error_clip: list[float] | None = None
    max_delta_tau: float | None = None
    gains_time_constant: float | None = None

    def __post_init__(self) -> None:
        vectors = {"cartesian_damping": 6, "translational_error_clip": 3, "rotational_error_clip": 3}
        for field in dataclasses.fields(self):
            value = getattr(self, field.name)
            if value is None:
                continue
            if field.name in vectors or field.name in ("joint_stiffness", "joint_damping"):
                if not isinstance(value, (list, tuple)) or not value:
                    raise ValueError(f"{field.name} must be a nonempty numeric vector")
                if field.name in vectors and len(value) != vectors[field.name]:
                    raise ValueError(f"{field.name} requires {vectors[field.name]} entries")
                values = value
            else:
                values = [value]
            positive = field.name in ("max_delta_tau", "gains_time_constant", "translational_error_clip", "rotational_error_clip")
            for item in values:
                if isinstance(item, bool) or not isinstance(item, (int, float)) or not math.isfinite(item) or (item <= 0 if positive else item < 0):
                    raise ValueError(f"{field.name} requires finite {'positive' if positive else 'nonnegative'} numbers")

    @classmethod
    def from_dict(cls, config: Mapping[str, Any] | "ArmControlConfig" | None) -> "ArmControlConfig":
        if isinstance(config, cls):
            return config
        config = dict(config or {})
        unknown = config.keys() - {field.name for field in dataclasses.fields(cls)}
        if unknown:
            raise ValueError(f"unknown arm control settings: {sorted(unknown)}")
        return cls(**config)

    def to_dict(self) -> dict[str, Any]:
        return {name: value for name, value in dataclasses.asdict(self).items() if value is not None}

    def for_mode(self, mode: str) -> dict[str, Any]:
        config = self.to_dict()
        result = {key: config[key] for key in ("max_delta_tau", "gains_time_constant") if key in config}
        if mode == "cartesian_servo":
            result.update({key: config[key] for key in ("translational_stiffness", "rotational_stiffness", "translational_error_clip", "rotational_error_clip") if key in config})
            if "cartesian_damping" in config:
                result["damping"] = config["cartesian_damping"]
        elif mode == "joint_servo":
            for source, target in (("joint_stiffness", "stiffness"), ("joint_damping", "damping")):
                if source in config:
                    result[target] = config[source]
        return result
