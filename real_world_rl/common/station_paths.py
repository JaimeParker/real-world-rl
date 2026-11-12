"""Where station files are looked up. Standard library only, so the host doctor can ask before
any device-layer import.

Lookup order: ``$RWRL_STATION``, ``<cwd>/local/``, ``$XDG_CONFIG_HOME/rwrl/`` (default
``~/.config/rwrl/``). The pre-0.1.0 names, ``$ROBOT_INFRA_CONFIG`` and ``~/.config/robot_infra/``,
are still read for one release after their replacements; ``rwrl doctor`` reports their use."""
from __future__ import annotations

import os
from pathlib import Path
from typing import Mapping

STATION_FILENAME = "station.yaml"
STATION_CONFIG_ENV = "RWRL_STATION"
LEGACY_STATION_CONFIG_ENV = "ROBOT_INFRA_CONFIG"
STATION_SCOPES = ("local", "user")
LOCAL_CONFIG_DIRNAME = "local"
USER_CONFIG_DIRNAME = "rwrl"
LEGACY_USER_CONFIG_DIRNAME = "robot_infra"


def _environ(env: Mapping[str, str] | None) -> Mapping[str, str]:
    return os.environ if env is None else env


def user_config_home(env: Mapping[str, str] | None = None) -> Path:
    """``$XDG_CONFIG_HOME``, else ``$HOME/.config``."""
    env = _environ(env)
    if env.get("XDG_CONFIG_HOME"):
        return Path(env["XDG_CONFIG_HOME"])
    if env.get("HOME"):
        return Path(env["HOME"]) / ".config"
    return Path.home() / ".config"


def station_scope_dir(scope: str, *, cwd: str | Path | None = None,
                      env: Mapping[str, str] | None = None) -> Path:
    """The directory a station scope writes to: ``<cwd>/local`` or ``<config home>/rwrl``."""
    if scope == "local":
        return (Path.cwd() if cwd is None else Path(cwd)) / LOCAL_CONFIG_DIRNAME
    if scope == "user":
        return user_config_home(env) / USER_CONFIG_DIRNAME
    raise ValueError(f"unknown station config scope '{scope}'; choose from {list(STATION_SCOPES)}")


def legacy_user_config_dir(env: Mapping[str, str] | None = None) -> Path:
    return user_config_home(env) / LEGACY_USER_CONFIG_DIRNAME


def station_lookup_dirs(*, cwd: str | Path | None = None, env: Mapping[str, str] | None = None) -> list[Path]:
    """Directories searched for station-scope files, in order; the legacy user directory is last."""
    return [*(station_scope_dir(scope, cwd=cwd, env=env) for scope in STATION_SCOPES),
            legacy_user_config_dir(env)]


def station_path_from_env(env: Mapping[str, str] | None = None) -> Path | None:
    """``$RWRL_STATION``, else the legacy ``$ROBOT_INFRA_CONFIG``, else None."""
    env = _environ(env)
    for name in (STATION_CONFIG_ENV, LEGACY_STATION_CONFIG_ENV):
        if env.get(name):
            return Path(env[name])
    return None


def legacy_station_locations(*, env: Mapping[str, str] | None = None) -> list[tuple[str, str, str]]:
    """``(key, message, hint)`` per pre-0.1.0 name still in use; empty when nothing needs migrating."""
    env = _environ(env)
    found = []
    if env.get(LEGACY_STATION_CONFIG_ENV):
        if env.get(STATION_CONFIG_ENV):
            found.append((
                "station_env",
                f"${LEGACY_STATION_CONFIG_ENV} is set but ignored: ${STATION_CONFIG_ENV} takes precedence",
                f"unset {LEGACY_STATION_CONFIG_ENV}",
            ))
        else:
            found.append((
                "station_env",
                f"${LEGACY_STATION_CONFIG_ENV} is the pre-0.1.0 name and is read for one more release",
                f"export {STATION_CONFIG_ENV}=\"${LEGACY_STATION_CONFIG_ENV}\" and unset {LEGACY_STATION_CONFIG_ENV}",
            ))
    legacy = legacy_user_config_dir(env)
    if legacy.is_dir() and any(legacy.iterdir()):
        target = station_scope_dir("user", env=env)
        found.append((
            "user_config_dir",
            f"{legacy} is the pre-0.1.0 user config directory and is read for one more release",
            f"mkdir -p {target} && mv {legacy}/* {target}/",
        ))
    return found
