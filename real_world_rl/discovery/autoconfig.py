"""Station config lookup and loading, plus filling empty config fields from environment
variables and unambiguous camera discovery.

A station yaml is either one robot mapping (optionally nested under ``robot:``) or
``{"robots": {name: <robot mapping>, ...}}``. Each robot mapping may carry a
``daemon: {bind, token}`` block, read by the daemon and stripped before ``RobotConfig.from_dict``."""
from __future__ import annotations

import dataclasses
import os
import types
import typing
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable, Mapping

import yaml

from real_world_rl.common.station_paths import (
    LEGACY_STATION_CONFIG_ENV,
    STATION_CONFIG_ENV,
    STATION_FILENAME,
    STATION_SCOPES,  # noqa: F401  re-exported for setup and the device table lookup
    station_lookup_dirs,
    station_path_from_env,
    station_scope_dir,
)
from real_world_rl.devices import Camera, Robot, RobotConfig

_TRUE_STRINGS = {"1", "true", "yes"}
DEFAULT_DAEMON_BIND = "0.0.0.0:50051"
_DAEMON_KEY = "daemon"
_DAEMON_KEYS = {"bind", "token"}
_ROBOTS_KEY = "robots"
SETUP_COMMAND = "rwrl setup"
# Where the upper-layer station sections (teleop_profiles) are registered and loaded.
SECTIONS_LOADER = "real_world_rl.station.load_station"


def _scalar_type(annotation: Any) -> type | None:
    if typing.get_origin(annotation) in (typing.Union, types.UnionType):
        members = [arg for arg in typing.get_args(annotation) if arg is not type(None)]
        annotation = members[0] if len(members) == 1 else None
    return annotation if annotation in (str, int, float, bool) else None


def _cast(value: str, target: type) -> Any:
    if target is bool:
        return value.strip().lower() in _TRUE_STRINGS
    return target(value)


def fill_from_env(config: RobotConfig, env: Mapping[str, str] | None = None) -> RobotConfig:
    env = os.environ if env is None else env
    hints = typing.get_type_hints(type(config))
    updates: dict[str, Any] = {}
    for config_field in dataclasses.fields(config):
        if config_field.name == "arm_backend":
            continue
        if getattr(config, config_field.name) not in (None, ""):
            continue
        target = _scalar_type(hints.get(config_field.name))
        raw = env.get(config_field.name.upper())
        if target is not None and raw is not None:
            updates[config_field.name] = _cast(raw, target)
    updates["cameras"] = [_with_discovered_serial(camera) for camera in config.cameras]
    return dataclasses.replace(config, **updates)


def _with_discovered_serial(camera):
    if camera.serial:
        return camera
    try:
        serials = Camera.backend(camera.backend).discover()
    except ValueError:
        return camera
    return dataclasses.replace(camera, serial=next(iter(serials))) if len(serials) == 1 else camera


def default_station_config_dir(
    scope: str = "local", *, cwd: str | Path | None = None, env: Mapping[str, str] | None = None
) -> Path:
    """Directory the setup wizard writes ``station.yaml`` to: ``<cwd>/local`` for ``local``,
    ``$XDG_CONFIG_HOME/rwrl`` (default ``~/.config/rwrl``) for ``user``."""
    return station_scope_dir(scope, cwd=cwd, env=env)


def station_config_search_path(
    *, cwd: str | Path | None = None, env: Mapping[str, str] | None = None
) -> list[Path]:
    """The well-known station files in lookup order (``$RWRL_STATION`` comes first): the two
    scopes, then the pre-0.1.0 ``~/.config/robot_infra/station.yaml``."""
    return [directory / STATION_FILENAME for directory in station_lookup_dirs(cwd=cwd, env=env)]


def resolve_station_config_path(
    explicit: str | Path | None = None,
    *,
    env: Mapping[str, str] | None = None,
    cwd: str | Path | None = None,
) -> Path:
    """``explicit`` if given; else ``$RWRL_STATION`` (or the legacy ``$ROBOT_INFRA_CONFIG``) if
    set; else the first existing of ``station_config_search_path``."""
    env = os.environ if env is None else env
    if explicit:
        return Path(explicit)
    from_env = station_path_from_env(env)
    if from_env is not None:
        return from_env
    candidates = station_config_search_path(cwd=cwd, env=env)
    for candidate in candidates:
        if candidate.is_file():
            return candidate
    locations = [f"${STATION_CONFIG_ENV}", f"${LEGACY_STATION_CONFIG_ENV}", *(str(candidate) for candidate in candidates)]
    raise FileNotFoundError(
        "no robot config given and no station config found; looked at " + ", ".join(locations)
        + f". Pass --station <yaml>, or run `{SETUP_COMMAND}` to write one"
    )


@dataclass
class StationRobot:
    """One robot of a station: its config and where its daemon listens."""

    config: RobotConfig
    bind: str = DEFAULT_DAEMON_BIND
    token: str | None = None
    # The yaml has a ``daemon:`` block: the robot is meant to be served (the doctor then needs grpc).
    daemon_declared: bool = field(default=False, compare=False)
    sections: dict[str, Any] = field(default_factory=dict)


@dataclass
class Station:
    """Every robot of a station yaml by name. ``multi_robot`` is True for the ``robots:`` form."""

    robots: dict[str, StationRobot]
    multi_robot: bool = False

    def select(self, name: str | None = None) -> StationRobot:
        """The robot called ``name``; without a name, the only robot (several raise ``ValueError``)."""
        if name is None:
            if len(self.robots) == 1:
                return next(iter(self.robots.values()))
            raise ValueError(
                f"the station has {len(self.robots)} robots {sorted(self.robots)}; pass --robot <name>"
            )
        try:
            return self.robots[name]
        except KeyError:
            raise ValueError(f"no robot '{name}' in the station; robots: {sorted(self.robots)}") from None


def _robot_config_from_mapping(
    data: Any, *, default_name: str | None = None,
    sections: Mapping[str, Callable[[Any], Any]] | None = None,
) -> StationRobot:
    if not isinstance(data, Mapping):
        raise ValueError(f"a robot config must be a mapping, got {type(data).__name__}")
    values = dict(data)
    daemon_declared = values.get(_DAEMON_KEY) is not None
    daemon = values.pop(_DAEMON_KEY, None) or {}
    if not isinstance(daemon, Mapping):
        raise ValueError(f"'daemon' must be a mapping with keys {sorted(_DAEMON_KEYS)}, got {daemon!r}")
    unknown = sorted(set(daemon) - _DAEMON_KEYS)
    if unknown:
        raise ValueError(f"unknown daemon keys {unknown}; known: {sorted(_DAEMON_KEYS)}")
    if "type" not in values:
        raise ValueError(f"robot config has no 'type'; registered: {sorted(Robot.types())}")
    if default_name is not None and "name" not in values:
        values["name"] = default_name
    validated_sections = {}
    for name, validator in (sections or {}).items():
        if name in values:
            try:
                validated_sections[name] = validator(values.pop(name))
            except ValueError as error:
                robot_name = values.get("name", "robot")
                raise ValueError(f"robot {robot_name!r}, section {name!r}: {error}") from error
    config = Robot.types()[str(values["type"]).lower()].CONFIG.from_dict(values)
    bind = daemon.get("bind")
    token = daemon.get("token")
    return StationRobot(
        config=config,
        bind=DEFAULT_DAEMON_BIND if bind is None else str(bind),
        token=None if token is None else str(token),
        daemon_declared=daemon_declared,
        sections=validated_sections,
    )


def parse_station(
    data: Any, *, sections: Mapping[str, Callable[[Any], Any]] | None = None,
) -> Station:
    """A station from parsed yaml: one robot mapping (optionally under ``robot:``), named by its
    ``name`` field, or ``{"robots": {name: mapping}}`` where a mapping without ``name`` takes its key.

    Like ``daemon:``, registered sections describe a robot but are not device config.
    Their keys are removed before device parsing; validators return the values stored
    in ``StationRobot.sections``. The device layer does not interpret their contents.
    Without sections (None or {}) parsing is strict device parsing; the upper layer passes its
    sections explicitly (``real_world_rl.station.load_station``).
    Unregistered unknown keys still fail device parsing.
    """
    sections = {} if sections is None else sections

    def parse_robot(mapping, *, default_name=None):
        try:
            return _robot_config_from_mapping(mapping, default_name=default_name, sections=sections)
        except ValueError as error:
            # Only enrich device-config unknown keys, not validator/nested config errors.
            config_names = [robot.CONFIG.__name__ for robot in Robot.types().values()]
            if str(error).startswith("unknown key '") and any(
                f" for {name};" in str(error) for name in config_names
            ):
                raise ValueError(
                    f"{error}; if this is an upper-layer station section, load the station with "
                    f"{SECTIONS_LOADER}; registered sections: {sorted(sections)}"
                ) from error
            raise

    if isinstance(data, Mapping) and _ROBOTS_KEY in data:
        unknown = sorted(set(data) - {_ROBOTS_KEY})
        if unknown:
            raise ValueError(f"unknown station keys {unknown}; a multi-robot station has only '{_ROBOTS_KEY}'")
        robots = data[_ROBOTS_KEY]
        if not isinstance(robots, Mapping) or not robots:
            raise ValueError(f"'{_ROBOTS_KEY}' must map at least one robot name to its config")
        return Station(
            {
                str(name): parse_robot(robot, default_name=str(name))
                for name, robot in robots.items()
            },
            multi_robot=True,
        )
    if isinstance(data, Mapping) and "robot" in data:
        data = data["robot"]
    robot = parse_robot(data)
    return Station({robot.config.name: robot})


def load_station(
    path: str | Path, *, sections: Mapping[str, Callable[[Any], Any]] | None = None,
) -> Station:
    """Parse a station yaml without building parts or reading the environment.

    A spacemouse teleop's local device table is resolved against the station's directory here:
    ``device_table`` becomes absolute, or is set to the station's ``spacemouse-devices.yaml``
    when that file exists, so the daemon, direct ``rwrl`` routes and the training side read the
    same table."""
    station = parse_station(yaml.safe_load(Path(path).read_text()), sections=sections)
    station_dir = Path(path).resolve().parent
    for robot in station.robots.values():
        teleop = robot.config.teleop
        if teleop is not None and teleop.backend == "spacemouse":
            from real_world_rl.teleop.spacemouse.hid import station_teleop_options

            options = station_teleop_options(teleop.options, station_dir)
            if options != teleop.options:
                robot.config = dataclasses.replace(
                    robot.config, teleop=dataclasses.replace(teleop, options=options))
    return station


def load_station_with_env(
    path: str | Path, *, sections: Mapping[str, Callable[[Any], Any]] | None = None,
) -> Station:
    """``load_station`` with ``fill_from_env`` applied to every robot."""
    station = load_station(path, sections=sections)
    for robot in station.robots.values():
        robot.config = fill_from_env(robot.config)
    return station


def load_config(
    path: str | Path, robot: str | None = None, *,
    sections: Mapping[str, Callable[[Any], Any]] | None = None,
) -> RobotConfig:
    """Parse a robot or station yaml into one robot's config without building parts, so an
    unregistered backend is reported by the doctor instead of failing at load time. A station
    with several robots needs ``robot``."""
    return load_station(path, sections=sections).select(robot).config


def load_config_with_env(
    path: str | Path, robot: str | None = None, *,
    sections: Mapping[str, Callable[[Any], Any]] | None = None,
) -> RobotConfig:
    """``load_config`` plus ``fill_from_env``: the one config view shared by daemon, doctor and discovery."""
    return fill_from_env(load_config(path, robot, sections=sections))
