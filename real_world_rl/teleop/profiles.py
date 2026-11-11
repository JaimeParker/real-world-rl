"""Portable, device-free SpaceMouse profile loading shared by CLI and training."""
from __future__ import annotations

import math
import logging
from pathlib import Path
from collections.abc import Mapping

import yaml

from robot_infra.devices.workspace import task_transform
from robot_infra.discovery.autoconfig import StationRobot
from real_world_rl.teleop.calibration import input_rotation

AXIS_MAPPING_VERSION = "serl_hid_axes_v1"


def validate_profiles_reference(reference) -> dict[str, str]:
    """Validate the upper-layer station reference to a named profile file."""
    if (not isinstance(reference, dict) or set(reference) - {"file", "default"}
            or not isinstance(reference.get("file"), str) or not reference["file"]
            or not isinstance(reference.get("default"), str) or not reference["default"]):
        raise ValueError("teleop_profiles requires nonempty file and default names")
    return {"file": reference["file"], "default": reference["default"]}


def validate_device_identity(identity):
    if identity is None:
        return None
    if not isinstance(identity, Mapping) or set(identity) - {"vendor_id", "product_id", "serial_number", "model"}:
        raise ValueError("device_identity requires USB vendor_id/product_id and optional serial_number/model")
    result = dict(identity)
    for key in ("vendor_id", "product_id"):
        if type(result.get(key)) is not int or not 0 <= result[key] <= 65535:
            raise ValueError(f"device_identity {key} must be an integer USB identifier")
    for key in ("serial_number", "model"):
        if key in result and not isinstance(result[key], str):
            raise ValueError(f"device_identity {key} must be text")
    return result


def verify_device_identity(expected, actual):
    """Match USB IDs and a recorded nonempty serial; labels are not identifiers."""
    expected = validate_device_identity(expected)
    if expected is None:
        return
    if not isinstance(actual, Mapping):
        raise ValueError("connected SpaceMouse identity is unavailable")
    keys = ["vendor_id", "product_id"]
    if expected.get("serial_number"):
        keys.append("serial_number")
    mismatched = [key for key in keys if actual.get(key) != expected[key]]
    if mismatched:
        raise ValueError("connected SpaceMouse does not match profile: " + ", ".join(mismatched))


def start_profiled_source(source):
    """Connect only owned input, verify identity, and unwind on mismatch."""
    from robot_infra.devices.errors import DeviceFault
    if source._owns_part:
        source._part.connect()
    try:
        verify_device_identity(source._expected_device_identity,
                               getattr(source._part, "device_identity", None))
    except ValueError as error:
        if source._owns_part:
            try:
                source._part.disconnect()
            except Exception as cleanup_error:
                logging.getLogger(__name__).warning("input cleanup also failed: %s", cleanup_error, exc_info=True)
        raise DeviceFault(str(error), hint="select or calibrate a profile for the connected device") from error


def validate_profile(profile: Mapping) -> dict:
    if not isinstance(profile, Mapping):
        raise ValueError("profile must be a mapping")
    unknown = set(profile) - {"frame", "yaw_rad", "neutral_threshold", "axis_mapping_version",
                              "device_identity", "evidence", "horizontal_validated", "rotation_validated"}
    if unknown:
        raise ValueError(f"unknown profile fields: {sorted(unknown)}")
    result = dict(profile)
    if "device_identity" in result:
        result["device_identity"] = validate_device_identity(result["device_identity"])
    for key, default in (("yaw_rad", None), ("neutral_threshold", .05)):
        value = result.get(key, default)
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError(f"profile {key} must be finite")
        result[key] = float(value)
    if not 0 < result["neutral_threshold"] < 1:
        raise ValueError("neutral_threshold must be in (0, 1)")
    if not isinstance(result.get("frame"), str) or not result["frame"]:
        raise ValueError("profile frame is required")
    if result.get("axis_mapping_version") != AXIS_MAPPING_VERSION:
        raise ValueError(f"profile requires axis_mapping_version={AXIS_MAPPING_VERSION}")
    for key in ("horizontal_validated", "rotation_validated"):
        if key in result and not isinstance(result[key], bool):
            raise ValueError(f"{key} must be boolean")
    return result


def load_profiles(path: str | Path) -> dict:
    document = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    if not isinstance(document, dict) or set(document) != {"schema_version", "profiles"}:
        raise ValueError("profile file requires schema_version and profiles")
    if type(document["schema_version"]) is not int or document["schema_version"] != 1:
        raise ValueError("unsupported profile schema_version")
    profiles = document["profiles"]
    if not isinstance(profiles, dict) or any(not isinstance(k, str) or not k for k in profiles):
        raise ValueError("profiles must map nonempty names to profiles")
    return {name: validate_profile(value) for name, value in profiles.items()}


def resolve_profile(station_path: str | Path, robot: StationRobot, name: str | None = None) -> dict:
    reference = robot.sections.get("teleop_profiles")
    if not reference:
        raise ValueError("station robot has no teleop_profiles; run rwrl teleop calibrate")
    file = Path(reference["file"])
    if not file.is_absolute():
        file = Path(station_path).resolve().parent / file
    selected = name if name is not None else reference.get("default")
    profiles = load_profiles(file)
    if selected not in profiles:
        raise ValueError(f"profile {selected!r} not found; available: {sorted(profiles)}")
    profile = profiles[selected]
    transform = task_transform(robot.config.task_frame)
    if profile["frame"] == "base":
        import numpy as np
        transform = np.eye(4)
    elif not robot.config.task_frame or profile["frame"] != robot.config.task_frame["name"]:
        raise ValueError("profile frame does not match the station task_frame")
    return {"name": selected, "file": str(file), "profile": profile,
            "input_rotation": input_rotation(transform[:3, :3], profile["yaw_rad"]).tolist()}


def source_options(station_path, robot_name=None, profile_name=None, *, provenance=None) -> dict:
    from real_world_rl.station import load_station
    robot = load_station(station_path).select(robot_name)
    config = robot.config
    if config.teleop is None or config.teleop.backend != "spacemouse":
        raise ValueError("selected station robot must configure a spacemouse teleop")
    options = dict(config.teleop.options)
    if config.task_frame is not None and "input_rotation" in options:
        raise ValueError("input_rotation conflicts with station task_frame; migrate the rotation "
                         "to task_frame and select a named operator profile")
    if robot.sections.get("teleop_profiles") or profile_name is not None:
        if "input_rotation" in options:
            raise ValueError("input_rotation conflicts with a named profile")
        resolved = resolve_profile(station_path, robot, profile_name)
        options["input_rotation"] = resolved["input_rotation"]
        if provenance is not None:
            from real_world_rl.common.provenance import portable_path
            provenance.update(profile_name=resolved["name"], profile_file=portable_path(resolved["file"]),
                              yaw_rad=resolved["profile"]["yaw_rad"],
                              input_rotation=resolved["input_rotation"],
                              task_frame=({key: value for key, value in config.task_frame.items() if key != "source"}
                                          if config.task_frame else None))
        if resolved["profile"].get("device_identity") is not None:
            options["expected_device_identity"] = resolved["profile"]["device_identity"]
    return options
