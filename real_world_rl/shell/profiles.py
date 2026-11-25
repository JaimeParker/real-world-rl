"""Device profiles: every ``local/sites/<name>/`` with a ``station.yaml``, plus the station files
the plain commands find (``local/station.yaml``, ``~/.config/rwrl/station.yaml``)."""
from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from real_world_rl.common.station_paths import STATION_FILENAME, STATION_SCOPES, station_scope_dir

SITES_DIRNAME = "sites"
TASK_FILENAME = "task.yaml"
TASKS_DIRNAME = "tasks"


@dataclass
class Profile:
    name: str
    station: Path
    tasks: dict[str, Path] = field(default_factory=dict)
    # Why the station does not load; such a profile is listed but cannot be selected.
    error: str | None = None
    loaded: Any = None  # the parsed Station

    @property
    def robots(self) -> list[str]:
        return [] if self.loaded is None else list(self.loaded.robots)

    def robot(self, name: str):
        return self.loaded.robots[name]

    def summary(self) -> str:
        if self.error is not None:
            return "does not load: " + self.error
        types = sorted({entry.config.TYPE for entry in self.loaded.robots.values()})
        count = len(self.tasks)
        return f"{', '.join(types)}  {', '.join(self.robots)}  {count} task{'' if count == 1 else 's'}"


def find_tasks(directory: Path) -> dict[str, Path]:
    """``task.yaml`` and ``tasks/*.yaml`` by name; ``tasks/task.yaml`` is listed as ``tasks/task``
    when both exist."""
    tasks = {}
    if (directory / TASK_FILENAME).is_file():
        tasks["task"] = directory / TASK_FILENAME
    for path in sorted((directory / TASKS_DIRNAME).glob("*.yaml")):
        if path.is_file():
            tasks[path.stem if path.stem not in tasks else f"{TASKS_DIRNAME}/{path.stem}"] = path
    return tasks


def load_profile(name: str, station: Path) -> Profile:
    from real_world_rl.station import load_station
    profile = Profile(name, station, find_tasks(station.parent))
    try:
        profile.loaded = load_station(station)
    except Exception as error:  # any invalid file must stay listed with its reason
        profile.error = " ".join(str(error).split()) or type(error).__name__
    return profile


def discover(cwd: str | Path | None = None, env=None) -> list[Profile]:
    """Profiles by name, then the station files outside ``local/sites``. Absolute paths."""
    base = (Path.cwd() if cwd is None else Path(cwd)).resolve()
    sites = station_scope_dir("local", cwd=base) / SITES_DIRNAME
    found = [(directory.name, directory / STATION_FILENAME) for directory in sorted(sites.glob("*"))
             if (directory / STATION_FILENAME).is_file()]
    names = {name for name, _ in found}
    for scope in STATION_SCOPES:
        path = station_scope_dir(scope, cwd=base, env=env).resolve() / STATION_FILENAME
        if path.is_file():
            name = scope if scope not in names else f"{scope}/{STATION_FILENAME}"
            found.append((name, path))
    return [load_profile(name, path) for name, path in found]
