"""Where station files are looked up. Standard library only, so the host doctor can ask before
any device-layer import.

Lookup order: ``$RWRL_STATION``, the ``rwrl use station`` selection, ``<cwd>/local/``, ``$XDG_CONFIG_HOME/rwrl/`` (default
``~/.config/rwrl/``). The pre-0.1.0 names, ``$ROBOT_INFRA_CONFIG`` and ``~/.config/robot_infra/``,
are still read for one release after their replacements; ``rwrl doctor`` reports their use."""
from __future__ import annotations

import json
import os
import sys
from pathlib import Path
from typing import Mapping

STATION_FILENAME = "station.yaml"
STATION_CONFIG_ENV = "RWRL_STATION"
LEGACY_STATION_CONFIG_ENV = "ROBOT_INFRA_CONFIG"
# Set by `rwrl shell` for its commands, like $RWRL_STATION: the preset and robot of the selection.
PRESET_CONFIG_ENV = "RWRL_PRESET"
ROBOT_NAME_ENV = "RWRL_ROBOT"
SHELL_ENV = "RWRL_SHELL"  # marks a process started by `rwrl shell`
STATION_SCOPES = ("local", "user")
LOCAL_CONFIG_DIRNAME = "local"
USER_CONFIG_DIRNAME = "rwrl"
LEGACY_USER_CONFIG_DIRNAME = "robot_infra"
# ``rwrl use`` remembers a station and a preset per working directory, next to ./local/station.yaml.
SELECTION_FILENAME = ".rwrl-use.json"
SELECTION_KINDS = ("station", "preset")
SELECTION_COMMAND = "rwrl use"
# Selections a command fell back to in this process: kind -> path, each announced once on stderr.
_announced: dict[str, str] = {}


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


def selection_file(cwd: str | Path | None = None) -> Path:
    """``<cwd>/local/.rwrl-use.json``: where ``rwrl use`` keeps its selections."""
    return (Path.cwd() if cwd is None else Path(cwd)) / LOCAL_CONFIG_DIRNAME / SELECTION_FILENAME


def read_selection(cwd: str | Path | None = None) -> dict[str, str]:
    """The remembered ``{kind: path}``; empty when nothing was selected. Paths are as written by
    ``rwrl use``: absolute, or relative to the directory holding ``local/``."""
    path = selection_file(cwd)
    try:
        text = path.read_text(encoding="utf-8")
    except FileNotFoundError:
        return {}
    try:
        data = json.loads(text)
    except ValueError as error:
        raise ValueError(f"{path} is not valid JSON ({error}); run `{SELECTION_COMMAND} --clear` to reset it") from error
    if not isinstance(data, dict):
        raise ValueError(f"{path} must hold a JSON object; run `{SELECTION_COMMAND} --clear` to reset it")
    return {kind: str(data[kind]) for kind in SELECTION_KINDS if data.get(kind)}


def write_selection(updates: Mapping[str, str | None], cwd: str | Path | None = None) -> Path:
    """Set (a path) or forget (None) each kind; the file is removed once nothing is selected."""
    path = selection_file(cwd)
    try:
        data = read_selection(cwd)
    except ValueError:
        data = {}  # a damaged file is replaced rather than blocking --clear
    for kind, value in updates.items():
        if kind not in SELECTION_KINDS:
            raise ValueError(f"unknown selection '{kind}'; choose from {list(SELECTION_KINDS)}")
        if value is None:
            data.pop(kind, None)
        else:
            data[kind] = str(value)
    if data:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")
    else:
        path.unlink(missing_ok=True)
    return path


def selected_path(kind: str, cwd: str | Path | None = None) -> Path | None:
    """The path ``rwrl use <kind>`` remembered for this directory, else None."""
    value = read_selection(cwd).get(kind)
    if value is None:
        return None
    path = Path(value)
    return path if path.is_absolute() or cwd is None else Path(cwd) / path


def require_selected_file(kind: str, path: Path) -> Path:
    """``path`` if it exists; else an error naming how to fix the stale selection."""
    if not path.is_file():
        raise FileNotFoundError(
            f"{kind} {path} chosen with `{SELECTION_COMMAND} {kind}` does not exist; "
            f"run `{SELECTION_COMMAND} {kind} <yaml>` or `{SELECTION_COMMAND} --clear {kind}`"
        )
    return path


def announce_selection(kind: str, path: str | Path) -> None:
    """Say once per process, on stderr, that a command fell back to a remembered selection."""
    if _announced.get(kind) == str(path):
        return
    _announced[kind] = str(path)
    print(f"using {kind} {path} (from {SELECTION_COMMAND})", file=sys.stderr)


def announced_selections() -> dict[str, str]:
    """The selections this process fell back to, for a ``--json`` result."""
    return dict(_announced)
