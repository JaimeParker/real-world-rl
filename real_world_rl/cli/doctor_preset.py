"""Doctor checks that a preset's copy of the station hardware still matches the station."""

from __future__ import annotations

import json
import math
from pathlib import Path

PRESET_HARDWARE_FIELDS = ("robot_ip", "arm_backend", "gripper", "cameras", "home_joints",
                          "joint_limits", "safety_box", "task_frame", "workspace")
PRESET_CAMERA_FIELDS = ("serial", "width", "height", "fps", "enable_depth")
STALE_PRESET_HINT = "the preset copy of the station is stale; regenerate it or edit the field"
_BRIEF_LIST_LENGTH = 4


def same_value(first, second) -> bool:
    """Structural equality; numbers compare with math.isclose(abs_tol=1e-9)."""
    if isinstance(first, bool) or isinstance(second, bool):
        return type(first) is type(second) and first == second
    if isinstance(first, (int, float)) and isinstance(second, (int, float)):
        return math.isclose(first, second, abs_tol=1e-9)
    if isinstance(first, dict) and isinstance(second, dict):
        return first.keys() == second.keys() and all(same_value(first[key], second[key]) for key in first)
    if isinstance(first, (list, tuple)) and isinstance(second, (list, tuple)):
        return len(first) == len(second) and all(same_value(a, b) for a, b in zip(first, second))
    return first == second


def _camera_differences(station_cameras, preset_cameras):
    station = {camera["name"]: camera for camera in station_cameras}
    preset = {camera["name"]: camera for camera in preset_cameras}
    differences = []
    for name in [*station, *(name for name in preset if name not in station)]:
        if name not in station or name not in preset:
            summary = [None if camera is None else {key: camera[key] for key in PRESET_CAMERA_FIELDS}
                       for camera in (station.get(name), preset.get(name))]
            differences.append((f"cameras.{name}", *summary))
            continue
        differences.extend((f"cameras.{name}.{key}", station[name][key], preset[name][key])
                           for key in PRESET_CAMERA_FIELDS
                           if not same_value(station[name][key], preset[name][key]))
    return differences


def preset_station_differences(station_config, preset_robot: dict) -> list[tuple[str, object, object]]:
    """Hardware fields where a preset robot mapping disagrees with the station's robot config.

    The preset mapping is normalised through the station robot's config class, so omitted keys
    compare as the defaults the preset would run with.
    """
    preset_type = preset_robot.get("type", station_config.TYPE)
    if preset_type != station_config.TYPE:
        return [("type", station_config.TYPE, preset_type)]
    return config_differences(station_config, type(station_config).from_dict(preset_robot))


def config_differences(station_config, preset_config) -> list[tuple[str, object, object]]:
    """Hardware fields where two robot configs disagree, as (field, station value, preset value).

    robot_ip is compared only for robot types that have it. Cameras align by name; each differing
    camera field is its own entry, and a camera on one side only is a ``cameras.<name>`` entry.
    """
    if station_config.TYPE != preset_config.TYPE:
        return [("type", station_config.TYPE, preset_config.TYPE)]
    station = station_config.to_dict()
    preset = preset_config.to_dict()
    differences = []
    for field in PRESET_HARDWARE_FIELDS:
        if field not in station:
            continue
        if field == "cameras":
            differences.extend(_camera_differences(station[field], preset[field]))
        elif not same_value(station[field], preset[field]):
            differences.append((field, station[field], preset[field]))
    return differences


def brief(value) -> str:
    """Compact text for a check message: long lists show their first and last items and length."""
    if isinstance(value, (list, tuple)):
        if len(value) > _BRIEF_LIST_LENGTH:
            return f"[{brief(value[0])}, ..., {brief(value[-1])}] (length {len(value)})"
        return "[" + ", ".join(brief(item) for item in value) + "]"
    if isinstance(value, dict):
        return "{" + ", ".join(f"{key}: {brief(item)}" for key, item in value.items()) + "}"
    return json.dumps(value)


def preset_robot_reference(document):
    """The preset's robot entry: real_robot.robot in a training preset, robot in a bare env mapping."""
    if not isinstance(document, dict):
        return None
    environment = document.get("real_robot") if "real_robot" in document else document
    return environment.get("robot") if isinstance(environment, dict) else None


def comparison_checks(name: str, differences) -> list[tuple[str, str, str, str]]:
    if not differences:
        return [(f"{name}.preset", "PASS", "preset hardware fields match the station", "")]
    return [(f"{name}.preset:{field}", "WARN", f"station={brief(station)}; preset={brief(preset)}", STALE_PRESET_HINT)
            for field, station, preset in differences]


def preset_station_checks(args) -> list[tuple[str, str, str, str]]:
    """(id, status, message, hint) rows comparing a preset's robot with the commissioned station.

    Empty without both a station and a preset robot mapping or station path. An inline robot
    mapping is compared with the --robot robot or the station's only robot; a station path is
    compared robot by robot under the same names.
    """
    preset_path = getattr(args, "preset", None)
    if not preset_path:
        return []
    import yaml
    from real_world_rl.station import load_station
    from robot_infra.discovery.autoconfig import resolve_station_config_path

    reference = preset_robot_reference(yaml.safe_load(Path(preset_path).read_text(encoding="utf-8")))
    if not reference or not isinstance(reference, (dict, str)):
        return []
    try:
        station_path = resolve_station_config_path(getattr(args, "config", None))
    except FileNotFoundError:
        return []
    station = load_station(station_path)
    names = [args.robot] if getattr(args, "robot", None) else list(station.robots)
    if isinstance(reference, dict):
        if len(names) != 1:
            return [("preset.robot", "SKIPPED",
                     f"the preset's inline robot cannot be matched to one of {len(names)} station robots "
                     f"{sorted(names)}", "pass --robot to compare it with one station robot")]
        config = station.select(names[0]).config
        try:
            differences = preset_station_differences(config, reference)
        except (TypeError, ValueError) as error:
            return [(f"{names[0]}.preset", "WARN",
                     f"the preset's inline robot is not a valid {config.TYPE} config: {error}", STALE_PRESET_HINT)]
        return comparison_checks(names[0], differences)
    preset_station = load_station(reference)
    checks = []
    for name in names:
        entry = preset_station.robots.get(name)
        if entry is None:
            checks.append((f"{name}.preset", "SKIPPED",
                           f"preset station {reference} has no robot '{name}'; nothing to compare", ""))
        else:
            checks.extend(comparison_checks(name, config_differences(station.select(name).config, entry.config)))
    return checks
