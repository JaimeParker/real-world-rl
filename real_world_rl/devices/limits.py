"""Unified station limits and servo projection, independent of SDKs and learning code."""
from __future__ import annotations

from collections.abc import Mapping
from copy import deepcopy
from dataclasses import dataclass
from typing import TYPE_CHECKING

import numpy as np

from real_world_rl.devices.errors import SafetyViolation
from real_world_rl.devices.workspace import task_transform

if TYPE_CHECKING:
    from real_world_rl.devices.robot import RobotConfig


def _keys(value, allowed, name):
    if not isinstance(value, Mapping):
        raise ValueError(f"{name} must be a mapping")
    unknown = set(value) - set(allowed)
    if unknown:
        raise ValueError(f"unknown {name} keys: {sorted(unknown)}")


def _array(value, shape, name):
    raw = np.asarray(value, dtype=object)
    if any(isinstance(x, (bool, str)) or not isinstance(x, (int, float, np.number))
           for x in raw.flat):
        raise ValueError(f"{name} must contain finite numbers")
    result = np.array(value, dtype=float, copy=True)
    if result.shape != shape or not np.isfinite(result).all():
        raise ValueError(f"{name} must have shape {shape} and contain finite numbers")
    result.setflags(write=False)
    return result


def _scalar(value, name, *, positive=False):
    if (isinstance(value, bool) or not isinstance(value, (int, float, np.number))
            or not np.isfinite(value) or (value <= 0 if positive else value < 0)):
        raise ValueError(f"{name} must be finite and {'positive' if positive else 'nonnegative'}")
    return float(value)


def _steps(value):
    if value is None:
        return {}
    _keys(value, ("xyz_m", "joint_rad"), "max_delta_per_step")
    return {key: _scalar(v, key, positive=True) for key, v in value.items()}


@dataclass(frozen=True)
class TcpBox:
    origin_xyz: np.ndarray
    axes_R: np.ndarray
    low: np.ndarray
    high: np.ndarray
    measured_margin_m: float = .003

    def __post_init__(self):
        for name, shape in (("origin_xyz", (3,)), ("axes_R", (3, 3)), ("low", (3,)), ("high", (3,))):
            object.__setattr__(self, name, _array(getattr(self, name), shape, name))
        if (not np.allclose(self.axes_R.T @ self.axes_R, np.eye(3), atol=1e-6, rtol=0)
                or not np.isclose(np.linalg.det(self.axes_R), 1, atol=1e-6, rtol=0)):
            raise ValueError("axes_R must be a proper rotation")
        if np.any(self.low >= 0) or np.any(self.high <= 0):
            raise ValueError("tcp bounds require low < 0 < high on every axis")
        object.__setattr__(self, "measured_margin_m", _scalar(self.measured_margin_m, "measured_margin_m"))

    def to_box(self, xyz) -> np.ndarray:
        return self.axes_R.T @ (_array(xyz, (3,), "xyz") - self.origin_xyz)

    def contains(self, xyz, margin=0.0) -> bool:
        return not self.violations(xyz, margin)

    def violations(self, xyz, margin=0.0) -> list[str]:
        margin = _scalar(margin, "margin")
        return [f"{axis}={v:+.4f} m not in [{lo:.4f}, {hi:.4f}] (box frame)"
                for axis, v, lo, hi in zip("xyz", self.to_box(xyz), self.low-margin, self.high+margin)
                if not lo <= v <= hi]

    def clip(self, xyz) -> np.ndarray:
        if self.contains(xyz):
            return np.asarray(xyz, dtype=float).copy()
        return self.origin_xyz + self.axes_R @ np.clip(self.to_box(xyz), self.low, self.high)

    def distance(self, xyz) -> float:
        position = self.to_box(xyz)
        return float(np.linalg.norm(position - np.clip(position, self.low, self.high)))

    def absolute_aabb(self) -> np.ndarray | None:
        if not np.array_equal(self.axes_R, np.eye(3)):
            return None
        return np.concatenate([self.origin_xyz + self.low, self.origin_xyz + self.high])

    def to_dict(self) -> dict:
        return {**{key: getattr(self, key).tolist() for key in ("origin_xyz", "axes_R", "low", "high")},
                "measured_margin_m": self.measured_margin_m}


@dataclass(frozen=True)
class ArmLimits:
    joints: np.ndarray | None = None
    tcp: TcpBox | None = None
    max_delta_xyz_m: float | None = None
    max_delta_joint_rad: float | None = None
    kinematics: dict | None = None

    def __post_init__(self):
        if self.joints is not None:
            raw = np.asarray(self.joints, dtype=object)
            if raw.ndim != 2 or raw.shape[1] != 2:
                raise ValueError("joints must be (n, 2) pairs")
            joints = _array(self.joints, raw.shape, "joints")
            if np.any(joints[:, 0] > joints[:, 1]):
                raise ValueError("joints require lo <= hi")
            object.__setattr__(self, "joints", joints)
        if self.tcp is not None and not isinstance(self.tcp, TcpBox):
            raise ValueError("tcp must be a TcpBox")
        for name in ("max_delta_xyz_m", "max_delta_joint_rad"):
            if getattr(self, name) is not None:
                object.__setattr__(self, name, _scalar(getattr(self, name), name, positive=True))
        if self.kinematics is not None:
            from real_world_rl.devices.robots.franka_kinematics import _tool_transform
            _keys(self.kinematics, ("model", "flange_T_ee"), "kinematics")
            if self.kinematics.get("model") not in ("panda", "fr3") or self.kinematics.get("flange_T_ee") is None:
                raise ValueError("kinematics requires model panda/fr3 and flange_T_ee")
            object.__setattr__(self, "kinematics", {
                "model": self.kinematics["model"],
                "flange_T_ee": _tool_transform(self.kinematics["flange_T_ee"]).tolist(),
            })

    @classmethod
    def from_config(cls, config: RobotConfig) -> ArmLimits | None:
        data = {} if config.limits is None else config.limits
        _keys(data, ("joints", "tcp", "max_delta_per_step"), "limits")
        joints = data.get("joints")
        if config.TYPE == "franka" and joints is None:
            # No limits.joints: the robot model's datasheet limits (also the setup wizard's default).
            from real_world_rl.devices.robots.franka import DATASHEET_JOINT_LIMITS
            joints = [list(pair) for pair in DATASHEET_JOINT_LIMITS[config.robot_model]]
        steps = _steps(data.get("max_delta_per_step"))
        tcp = None
        if data.get("tcp") is not None:
            spec = data["tcp"]
            _keys(spec, ("origin", "origin_xyz", "axes_R", "low", "high", "measured_margin_m"), "limits.tcp")
            origin = spec.get("origin")
            if isinstance(origin, str):
                if origin != "home":
                    raise ValueError("tcp.origin must be home or explicit XYZ")
                fk = config.kinematics()
                if config.home_joints is None or fk is None:
                    raise ValueError("origin: home requires home_joints and kinematics; use explicit origin XYZ")
                xyz = _array(spec.get("origin_xyz"), (3,), "origin_xyz")
                if np.linalg.norm(fk(config.home_joints) - xyz) > 1e-3:
                    raise ValueError("home_joints changed since limits were frozen; run `rwrl limits rebase`")
            else:
                xyz = _array(origin, (3,), "origin")
                if "origin_xyz" in spec and not np.array_equal(xyz, _array(spec["origin_xyz"], (3,), "origin_xyz")):
                    raise ValueError("explicit origin and origin_xyz must be equal")
            tcp = TcpBox(xyz, spec.get("axes_R"), spec.get("low"), spec.get("high"), spec.get("measured_margin_m", .003))
            if not np.allclose(tcp.axes_R, task_transform(config.task_frame)[:3, :3], atol=1e-6, rtol=0):
                raise ValueError("task_frame changed since limits were frozen; run `rwrl limits rebase`")
        kinematics = None
        if config.TYPE == "franka":
            from real_world_rl.devices.robots.franka_kinematics import _tool_transform
            kinematics = {"model": config.robot_model, "flange_T_ee": _tool_transform(config.flange_T_ee).tolist()}
        result = cls(joints, tcp, steps.get("xyz_m"), steps.get("joint_rad"), kinematics)
        if result.joints is not None:
            if result.joints.shape != (config.NUM_JOINTS, 2):
                raise ValueError(f"limits.joints must have {config.NUM_JOINTS} pairs")
            if config.home_joints is not None:
                home = np.asarray(config.home_joints)
                if np.any(home < result.joints[:, 0]) or np.any(home > result.joints[:, 1]):
                    raise ValueError("home_joints outside limits.joints")
        return None if result.is_unlimited else result

    @property
    def is_unlimited(self):
        return self.joints is None and self.tcp is None and self.max_delta_xyz_m is None and self.max_delta_joint_rad is None

    def to_dict(self) -> dict:
        steps = {key: value for key, value in (("xyz_m", self.max_delta_xyz_m), ("joint_rad", self.max_delta_joint_rad)) if value is not None}
        return {"version": "station_limits_v1", "joints": None if self.joints is None else self.joints.tolist(),
                "tcp": None if self.tcp is None else self.tcp.to_dict(), "max_delta_per_step": steps or None,
                "kinematics": deepcopy(self.kinematics)}

    def contract_dict(self) -> dict:
        """``to_dict()`` without ``kinematics``: the limit values an experiment contract compares.

        The FK description is device metadata, known only after connecting."""
        return {key: value for key, value in self.to_dict().items() if key != "kinematics"}

    @classmethod
    def from_dict(cls, data) -> ArmLimits:
        _keys(data, ("version", "joints", "tcp", "max_delta_per_step", "kinematics"), "station limits")
        if data.get("version") != "station_limits_v1":
            raise ValueError("expected station_limits_v1")
        tcp = data.get("tcp")
        if tcp is not None:
            _keys(tcp, ("origin_xyz", "axes_R", "low", "high", "measured_margin_m"), "tcp")
            tcp = TcpBox(tcp.get("origin_xyz"), tcp.get("axes_R"), tcp.get("low"), tcp.get("high"), tcp.get("measured_margin_m", .003))
        steps = _steps(data.get("max_delta_per_step"))
        return cls(data.get("joints"), tcp, steps.get("xyz_m"), steps.get("joint_rad"), data.get("kinematics"))

    def clip_pose(self, pose7):
        pose = _array(pose7, (7,), "pose7")
        result = pose.copy()
        if self.tcp is not None:
            result[:3] = self.tcp.clip(pose[:3])
        return result, not np.array_equal(result, pose)

    def limit_step_pose(self, pose7, current7):
        pose = _array(pose7, (7,), "pose7")
        current = _array(current7, (7,), "current7")
        step = pose[:3] - current[:3]
        distance = np.linalg.norm(step)
        if self.max_delta_xyz_m is None or distance <= self.max_delta_xyz_m:
            return pose, False
        return np.concatenate([current[:3] + step * self.max_delta_xyz_m / distance, pose[3:]]), True

    def clip_joints(self, q):
        q = self._q(q)
        result = q if self.joints is None else np.clip(q, self.joints[:, 0], self.joints[:, 1])
        return result, not np.array_equal(result, q)

    def _q(self, q):
        shape = (len(self.joints),) if self.joints is not None else np.asarray(q).shape
        if len(shape) != 1:
            raise ValueError("q must be a vector")
        return _array(q, shape, "q")

    def limit_step_joints(self, q, current_q):
        q = self._q(q)
        current = _array(current_q, q.shape, "current_q")
        if self.max_delta_joint_rad is None:
            return q, False
        result = np.clip(q, current-self.max_delta_joint_rad, current+self.max_delta_joint_rad)
        return result, not np.array_equal(result, q)

    def project_joints_to_box(self, q_target, q_current, fk):
        target = self._q(q_target)
        current = _array(q_current, target.shape, "q_current")
        if self.tcp is None or fk is None:
            return target, False
        target_xyz = fk(target)
        if target_xyz is None:
            return target, False
        current_xyz = fk(current)
        if not self.tcp.contains(current_xyz):
            if self.tcp.distance(target_xyz) < self.tcp.distance(current_xyz):
                return target, False
            return current, not np.array_equal(target, current)
        if self.tcp.contains(target_xyz):
            return target, False
        low, high = 0., 1.
        for _ in range(20):
            alpha = (low + high) / 2
            if self.tcp.contains(fk(current + alpha * (target-current))):
                low = alpha
            else:
                high = alpha
        return current + low * (target-current), True

    def require_pose_inside(self, pose7, device=""):
        pose = _array(pose7, (7,), "pose7")
        errors = [] if self.tcp is None else self.tcp.violations(pose[:3])
        if errors:
            raise SafetyViolation("target outside limits.tcp: " + "; ".join(errors), device=device, hint="change the target or the limits in the station yaml")

    def require_joints_inside(self, q, device="", fk=None):
        q = self._q(q)
        if self.joints is not None and (np.any(q < self.joints[:, 0]) or np.any(q > self.joints[:, 1])):
            raise SafetyViolation("target outside limits.joints", device=device, hint="change the target or the limits in the station yaml")
        if self.tcp is not None and fk is not None:
            xyz = fk(q)
            if xyz is not None:
                self.require_pose_inside([*xyz, 0, 0, 0, 1], device)

    def measured_violations(self, xyz) -> list[str]:
        return [] if self.tcp is None else self.tcp.violations(xyz, self.tcp.measured_margin_m)
