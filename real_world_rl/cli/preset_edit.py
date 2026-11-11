"""Read env presets and save only the resolved task's home joints."""
from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Any

import yaml

from .errors import CliError, USAGE_EXIT_CODE


def mapping(value: Any, label: str) -> dict:
    if not isinstance(value, dict):
        raise CliError(f"{label} must be a mapping", code=USAGE_EXIT_CODE)
    return value


def joints(value, count: int, label: str = "home_joints") -> list[float]:
    if not isinstance(value, (list, tuple)) or len(value) != count:
        raise CliError(f"{label} must contain {count} finite numbers")
    if any(isinstance(x, bool) or not isinstance(x, (int, float)) or not math.isfinite(x) for x in value):
        raise CliError(f"{label} must contain {count} finite numbers")
    return [float(x) for x in value]


def check_limits(values: list[float], limits, label: str) -> None:
    if limits is None:
        return
    if not isinstance(limits, (list, tuple)) or len(limits) != len(values):
        raise CliError(f"{label} must have {len(values)} [lo, hi] pairs")
    for i, (value, row) in enumerate(zip(values, limits)):
        low, high = joints(row, 2, label)
        if low > high:
            raise CliError(f"{label}[{i}] has lo > hi")
        if not low <= value <= high:
            raise CliError(f"joint {i} = {value} is outside {label}[{i}] = [{low}, {high}]")


@dataclass
class Preset:
    path: Path | None
    document: dict
    env: dict
    reset: dict
    raw: bytes | None = None

    @classmethod
    def load(cls, path: str | None) -> Preset:
        if path is None:
            return cls(None, {}, {}, {})
        preset_path = Path(path)
        raw = preset_path.read_bytes()
        document = mapping(yaml.safe_load(raw), "preset")
        # Training presets wrap env settings in real_robot; bare env mappings work too.
        env = mapping(document["real_robot"], "real_robot") if "real_robot" in document else document
        task = mapping({} if env.get("task") is None else env["task"], "task")
        reset = mapping({} if task.get("reset") is None else task["reset"], "task.reset")
        return cls(preset_path, document, env, reset, raw)

    def save_home(
        self, current: list[float], limits: list[tuple[str, Any]], *, num_joints: int
    ) -> dict:
        if self.path is None:
            raise CliError("home set requires --preset", code=USAGE_EXIT_CODE)
        current = joints(current, num_joints, "current joint_pos")
        updated = [round(value, 4) for value in current]
        limits = [*limits, ("preset joint_limits", self.env.get("joint_limits"))]
        robot_config = self.env.get("robot")
        if isinstance(robot_config, dict):
            limits.append(("preset robot joint_limits", robot_config.get("joint_limits")))
        for label, boundary in limits:
            check_limits(current, boundary, label)
            check_limits(updated, boundary, label)
        old = self.reset.get("home_joints")
        task = self.env.get("task")
        if task is None:
            task = self.env["task"] = {}
        reset = task.get("reset")
        if reset is None:
            reset = task["reset"] = {}
        reset["home_joints"] = updated
        from real_world_rl.common.yaml_edit import atomic_replace, render_changes
        text = render_changes(self.document, self.raw)
        saved = atomic_replace(self.path, text, self.raw)
        self.raw = text.encode("utf-8")
        self.reset = reset
        return {"old": old, "home_joints": updated, "backup": saved["backup"]}
