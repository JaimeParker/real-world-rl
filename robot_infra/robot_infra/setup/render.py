"""Render a ``StationDraft`` into the annotated ``station.template.yaml`` (``{{ name }}``
placeholders, plain string substitution, so the comments survive) and write it atomically."""
from __future__ import annotations

import dataclasses
import json
import math
import numbers
import os
import re
import shutil
import tempfile
import textwrap
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable, Mapping

import yaml

from robot_infra.devices import CameraConfig, GripperConfig, TeleopConfig
from robot_infra.devices.robots.franka import FrankaConfig
from robot_infra.discovery.autoconfig import (
    DEFAULT_DAEMON_BIND,
    STATION_FILENAME,
    Station,
    default_station_config_dir,
    parse_station,
)
from robot_infra.setup.wizard import GELLO_OPTION_ORDER, ArmDraft, StationDraft

TEMPLATE_PATH = Path(__file__).with_name("station.template.yaml")
BACKUP_TIME_FORMAT = "%Y%m%dT%H%M%SZ"
_SECTION_MARKER = "#@ "
_PLACEHOLDER = re.compile(r"\{\{\s*(\w+)\s*\}\}")
_PLAIN_STRING = re.compile(r"[A-Za-z0-9_./+-][A-Za-z0-9_./+@:-]*")
_ROBOT_INDENT = "    "
# Every field each section of the template writes; rendering a config with another field fails
# instead of silently dropping it.
_ROBOT_FIELDS = {
    "name", "arm_backend", "robot_ip", "bridge_url", "realtime", "relative_dynamics_factor", "home_joints",
    "task_frame", "workspace", "safety_box", "joint_limits", "max_delta_per_step", "control", "claim", "gripper", "cameras", "teleop",
}
_GRIPPER_FIELDS = {"backend", "max_width_m", "mode", "binary_threshold", "shares_arm_session"}
_CAMERA_FIELDS = {"name", "serial", "width", "height", "fps", "enable_depth", "backend"}
_TELEOP_FIELDS = {"name", "backend", "hold_s", "options"}

Echo = Callable[[str], None]


# -- yaml values -----------------------------------------------------------------------------


def _float_text(value: float) -> str:
    if math.isnan(value):
        return ".nan"
    if math.isinf(value):
        return ".inf" if value > 0 else "-.inf"
    text = repr(value)
    if "e" in text:
        # PyYAML only reads exponent floats with a dot and a signed exponent (1.0e-05).
        mantissa, exponent = text.split("e")
        mantissa = mantissa if "." in mantissa else f"{mantissa}.0"
        exponent = exponent if exponent[0] in "+-" else f"+{exponent}"
        text = f"{mantissa}e{exponent}"
    return text


def _string_text(value: str, *, flow: bool) -> str:
    plain = bool(_PLAIN_STRING.fullmatch(value)) and not (flow and ":" in value)
    if plain:
        try:
            plain = yaml.safe_load(value) == value
        except yaml.YAMLError:
            plain = False
    return value if plain else json.dumps(value)


def yaml_value(value: Any, *, flow: bool = False) -> str:
    """One-line YAML for a scalar, list or mapping (flow style) that ``yaml.safe_load`` reads back
    as ``value``."""
    if value is None:
        return "null"
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, numbers.Integral):
        return str(int(value))
    if isinstance(value, numbers.Real):
        return _float_text(float(value))
    if isinstance(value, str):
        return _string_text(value, flow=flow)
    if isinstance(value, Mapping):
        items = (f"{yaml_value(key, flow=True)}: {yaml_value(item, flow=True)}" for key, item in value.items())
        return "{" + ", ".join(items) + "}"
    if isinstance(value, (list, tuple)):
        return "[" + ", ".join(yaml_value(item, flow=True) for item in value) + "]"
    raise TypeError(f"cannot render {type(value).__name__} {value!r} as YAML")


# -- template ----------------------------------------------------------------------------------


def load_template(text: str | None = None) -> dict[str, list[str]]:
    """Sections of the template: each starts at a ``#@ <name>`` line."""
    text = TEMPLATE_PATH.read_text() if text is None else text
    sections: dict[str, list[str]] = {}
    current: list[str] | None = None
    for line in text.splitlines():
        if line.startswith(_SECTION_MARKER):
            current = sections.setdefault(line[len(_SECTION_MARKER):].strip(), [])
        elif current is not None:
            current.append(line)
    for lines in sections.values():
        while lines and not lines[-1].strip():
            lines.pop()
    return sections


def _fill(lines: list[str], values: Mapping[str, str]) -> str:
    """Substitute the placeholders. A line holding only a placeholder takes a (multi-line) block
    and disappears when the block is empty."""

    def value_of(name: str) -> str:
        if name not in values:
            raise ValueError(f"template placeholder '{{{{{name}}}}}' has no value")
        return values[name]

    filled = []
    for line in lines:
        block = _PLACEHOLDER.fullmatch(line)
        if block is not None:
            text = value_of(block.group(1))
            if text:
                filled.append(text)
            continue
        filled.append(_PLACEHOLDER.sub(lambda match: value_of(match.group(1)), line))
    return "\n".join(filled)


def _check_fields(config: Any, expected: set[str]) -> None:
    actual = {config_field.name for config_field in dataclasses.fields(config)}
    if actual != expected:
        raise ValueError(
            f"the station template does not match {type(config).__name__}: fields {sorted(actual ^ expected)} differ"
        )


def _gripper_text(gripper: GripperConfig | None, sections: dict[str, list[str]]) -> str:
    if gripper is None:
        return "\n".join(sections["no_gripper"])
    _check_fields(gripper, _GRIPPER_FIELDS)
    return _fill(sections["gripper"], {name: yaml_value(getattr(gripper, name)) for name in _GRIPPER_FIELDS})


def _cameras_text(cameras: list[CameraConfig], sections: dict[str, list[str]]) -> str:
    if not cameras:
        return "\n".join(sections["no_cameras"])
    entries = []
    for camera in cameras:
        _check_fields(camera, _CAMERA_FIELDS)
        entries.append(_fill(sections["camera"], {name: yaml_value(getattr(camera, name)) for name in _CAMERA_FIELDS}))
    return _fill(sections["cameras"], {"entries": "\n".join(entries)})


def _teleop_text(teleop: TeleopConfig | None, gello_calibrated: bool, sections: dict[str, list[str]]) -> str:
    if teleop is None:
        return "\n".join(sections["no_teleop"])
    _check_fields(teleop, _TELEOP_FIELDS)
    common = {"name": yaml_value(teleop.name), "hold_s": yaml_value(teleop.hold_s)}
    options = dict(teleop.options)
    if teleop.backend == "gello" and list(options) == list(GELLO_OPTION_ORDER):
        values = {key: yaml_value(value) for key, value in options.items()}
        values["todo"] = "" if gello_calibrated else "\n".join(sections["gello_todo"])
        return _fill(sections["teleop_gello"], {**common, **values})
    return _fill(
        sections["teleop_other"], {**common, "backend": yaml_value(teleop.backend), "options": yaml_value(options)}
    )


def _robot_text(arm: ArmDraft, sections: dict[str, list[str]], *, with_daemon: bool) -> str:
    config = arm.config
    if not isinstance(config, FrankaConfig):
        raise ValueError(f"the station template renders Franka robots, '{arm.key}' is a {config.TYPE}")
    _check_fields(config, _ROBOT_FIELDS)
    scalars = _ROBOT_FIELDS - {"gripper", "cameras", "teleop"}
    values = {name: yaml_value(getattr(config, name)) for name in scalars}
    values["gripper"] = _gripper_text(config.gripper, sections)
    values["cameras"] = _cameras_text(config.cameras, sections)
    values["teleop"] = _teleop_text(config.teleop, arm.gello_calibrated, sections)
    values["daemon"] = (
        _fill(sections["daemon"], {"bind": yaml_value(arm.bind), "token": yaml_value(arm.token)}) if with_daemon else ""
    )
    return _fill(sections["robot"], values)


def render_station_yaml(draft: StationDraft) -> str:
    """One arm: the flat single-robot form (a ``daemon:`` block only when bind or token differ
    from the defaults). Several: ``robots: {<key>: ...}``, each with its ``daemon:`` block."""
    if not draft.arms:
        raise ValueError("a station needs at least one robot")
    sections = load_template()
    header = "\n".join(sections["header"])
    if not draft.multi_robot:
        arm = draft.arms[0]
        with_daemon = arm.bind != DEFAULT_DAEMON_BIND or arm.token is not None
        return f"{header}\n{_robot_text(arm, sections, with_daemon=with_daemon)}\n"
    parts = [header, "\n".join(sections["multi_header"])]
    for arm in draft.arms:
        parts.append(f"  {yaml_value(arm.key)}:")
        parts.append(textwrap.indent(_robot_text(arm, sections, with_daemon=True), _ROBOT_INDENT))
    return "\n".join(parts) + "\n"


def check_rendered_station(text: str, draft: StationDraft) -> Station:
    """Load the rendered text with ``parse_station`` and check it holds exactly the draft."""
    station = parse_station(yaml.safe_load(text))
    expected_keys = [arm.key for arm in draft.arms] if draft.multi_robot else [draft.arms[0].config.name]
    if list(station.robots) != expected_keys:
        raise ValueError(f"rendered station has robots {list(station.robots)}, expected {expected_keys}")
    for arm, robot in zip(draft.arms, station.robots.values()):
        if robot.config != arm.config or (robot.bind, robot.token) != (arm.bind, arm.token):
            raise ValueError(f"the rendered yaml does not reproduce robot '{arm.key}'")
    return station


# -- writing -----------------------------------------------------------------------------------


def _backup_path(path: Path, now: datetime) -> Path:
    backup = path.with_name(f"{path.name}.bak.{now.strftime(BACKUP_TIME_FORMAT)}")
    counter = 1
    while backup.exists():
        backup = path.with_name(f"{path.name}.bak.{now.strftime(BACKUP_TIME_FORMAT)}.{counter}")
        counter += 1
    return backup


def write_station_yaml(
    text: str,
    scope: str = "local",
    path_override: str | Path | None = None,
    *,
    cwd: str | Path | None = None,
    env: Mapping[str, str] | None = None,
    now: datetime | None = None,
    echo: Echo | None = None,
) -> Path:
    """Write ``text`` to ``path_override`` or ``<scope dir>/station.yaml``: creates the directory,
    copies an existing file to ``station.yaml.bak.<UTC %Y%m%dT%H%M%SZ>``, then replaces it atomically."""
    if path_override is not None:
        path = Path(path_override).expanduser()
    else:
        path = default_station_config_dir(scope, cwd=cwd, env=env) / STATION_FILENAME
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.exists():
        backup = _backup_path(path, now or datetime.now(timezone.utc))
        shutil.copy2(path, backup)
        if echo is not None:
            echo(f"backed up the previous {path.name} to {backup}")
    handle = tempfile.NamedTemporaryFile(
        "w", dir=path.parent, prefix=f".{path.name}.", suffix=".tmp", delete=False, encoding="utf-8"
    )
    try:
        with handle:
            handle.write(text)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(handle.name, path)
    except BaseException:
        Path(handle.name).unlink(missing_ok=True)
        raise
    return path
