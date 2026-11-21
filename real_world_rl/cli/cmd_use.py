"""``rwrl use``: remember a station or preset so later commands can omit ``--station``/``--preset``."""
from __future__ import annotations

import os
from pathlib import Path

from .command import Result
from .errors import CliError, USAGE_EXIT_CODE


def stored_form(path: str) -> str:
    """Absolute paths stay absolute; relative ones are kept relative to this directory."""
    expanded = Path(path).expanduser()
    return str(expanded) if expanded.is_absolute() else os.path.normpath(expanded)


def validate(kind: str, path: str, station_loader) -> None:
    """Refuse to remember a file later commands could not load."""
    if not Path(path).is_file():
        raise CliError(f"{kind} {path} does not exist", code=USAGE_EXIT_CODE)
    try:
        if kind == "station":
            station_loader(path)
        else:
            from .preset_edit import Preset
            Preset.load(path)
    except CliError:
        raise
    except Exception as error:
        raise CliError(f"{kind} {path} does not load: {error}", code=USAGE_EXIT_CODE) from error


def current(cwd=None) -> dict:
    """``{kind: {"path", "source"}}`` for what a command run here would use without flags."""
    from real_world_rl.common.station_paths import (
        STATION_CONFIG_ENV,
        read_selection,
        station_lookup_dirs,
        STATION_FILENAME,
        station_path_from_env,
    )
    selection = read_selection(cwd)
    state = {}
    from_env = station_path_from_env()
    if from_env is not None:
        state["station"] = {"path": str(from_env), "source": "$" + STATION_CONFIG_ENV}
    elif "station" in selection:
        state["station"] = {"path": selection["station"], "source": "rwrl use"}
    else:
        found = next((directory / STATION_FILENAME for directory in station_lookup_dirs(cwd=cwd)
                      if (directory / STATION_FILENAME).is_file()), None)
        state["station"] = ({"path": str(found), "source": "lookup"} if found is not None
                            else {"path": None, "source": None})
    state["preset"] = ({"path": selection["preset"], "source": "rwrl use"} if "preset" in selection
                       else {"path": None, "source": None})
    if from_env is not None and "station" in selection:
        state["station"]["overrides"] = selection["station"]
    return state


def describe(state: dict) -> str:
    lines = []
    for kind, entry in state.items():
        if entry["path"] is None:
            hint = "pass --preset or run `rwrl use preset <yaml>`" if kind == "preset" else \
                "pass --station or run `rwrl use station <yaml>`"
            lines.append(f"{kind}: none ({hint})")
            continue
        line = f"{kind}: {entry['path']} ({entry['source']})"
        if "overrides" in entry:
            line += f"; the rwrl use selection {entry['overrides']} is ignored while it is set"
        lines.append(line)
    return "\n".join(lines)


class UseCommand:
    name = "use"
    help = "remember a station or preset for later commands in this directory"
    uses_selection = False

    def register(self, parser):
        parser.description = (
            "Remember a station or preset in ./local/.rwrl-use.json. Later commands run from this "
            "directory use it when --station/--preset is omitted ($RWRL_STATION still wins for the "
            "station). With no arguments, show what would be used.")
        parser.add_argument("kind", nargs="?", choices=("station", "preset"))
        parser.add_argument("path", nargs="?", metavar="PATH", help="the yaml to remember")
        parser.add_argument("--clear", action="store_true",
                            help="forget the selection of KIND (both when KIND is omitted)")

    def run(self, args, ctx):
        from real_world_rl.common.station_paths import selection_file, write_selection
        if args.clear:
            if args.path:
                raise CliError("--clear takes no PATH", code=USAGE_EXIT_CODE)
            kinds = [args.kind] if args.kind else ["station", "preset"]
            write_selection({kind: None for kind in kinds})
            action = "cleared " + ", ".join(kinds)
        elif args.path:
            validate(args.kind, args.path, ctx.station_loader or _load_station)
            write_selection({args.kind: stored_form(args.path)})
            action = f"remembered {args.kind} {stored_form(args.path)}"
        elif args.kind:
            raise CliError(f"rwrl use {args.kind} needs a PATH", code=USAGE_EXIT_CODE,
                           hint=f"rwrl use {args.kind} local/{'station' if args.kind == 'station' else 'task'}.yaml",
                           required_fields=["PATH"])
        else:
            action = None
        state = current()
        text = describe(state)
        if action:
            text = f"{action} in {shown(selection_file())}\n{text}"
        return Result({"command": "use", "file": str(selection_file()), **state}, text)


def shown(path: Path) -> str:
    """``path`` relative to the current directory when it lies below it."""
    try:
        return str(path.relative_to(Path.cwd()))
    except ValueError:
        return str(path)


def _load_station(path):
    from real_world_rl.station import load_station
    return load_station(path)


USE = UseCommand()
