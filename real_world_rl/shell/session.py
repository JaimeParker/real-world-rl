"""What a shell session has selected, and how a typed line becomes a command.

The selection lives in this process only: a child command receives it through ``$RWRL_STATION``,
``$RWRL_PRESET`` and ``$RWRL_ROBOT``, so other terminals and agents keep theirs."""
from __future__ import annotations

from dataclasses import dataclass, field
import json
import os
from pathlib import Path
import shlex
import sys

from real_world_rl.common.station_paths import (
    LEGACY_STATION_CONFIG_ENV,
    LOCAL_CONFIG_DIRNAME,
    PRESET_CONFIG_ENV,
    ROBOT_NAME_ENV,
    SHELL_ENV,
    STATION_CONFIG_ENV,
    write_selection,
)
from .profiles import Profile, discover, load_profile

STATE_DIRNAME = ".rwrl-shell"
# Learning commands whose --config is an env preset (deploy and deploy-openpi take none).
TASK_CONFIG_COMMANDS = ("train", "record", "collect-reward", "deploy-act")
# Commands that work without a selected device.
DEVICELESS_COMMANDS = ("setup", "discover", "archive", "dataset")
_VALUE_OPTIONS = ("--station", "--robot", "--preset")


class SessionError(Exception):
    """A line or selection the session refuses, with the message to show."""


@dataclass
class Run:
    argv: list[str]
    env: dict[str, str]
    warnings: list[str] = field(default_factory=list)
    # Robots whose daemon the command reaches by the preset's address, bypassing the CLI's own check.
    daemon_robots: list[str] = field(default_factory=list)


@dataclass
class Edit:
    """The completed line, offered for editing before it runs."""
    text: str


def state_dir(cwd: Path) -> Path:
    return cwd / LOCAL_CONFIG_DIRNAME / STATE_DIRNAME


def command_name(tokens: list[str]) -> str | None:
    """The command word of a line, after any global options."""
    index = 0
    while index < len(tokens):
        token = tokens[index]
        if not token.startswith("-"):
            return token
        index += 2 if token in _VALUE_OPTIONS else 1
    return None


def has_option(tokens: list[str], *names: str) -> bool:
    return any(token == name or token.startswith(name + "=") for token in tokens for name in names)


def option_value(tokens: list[str], name: str) -> str | None:
    for index, token in enumerate(tokens):
        if token.startswith(name + "="):
            return token.split("=", 1)[1]
        if token == name and index + 1 < len(tokens):
            return tokens[index + 1]
    return None


def split_line(line: str) -> list[str]:
    """The line's words; a leading ``rwrl`` is accepted and dropped."""
    try:
        tokens = shlex.split(line)
    except ValueError as error:
        raise SessionError(f"cannot read the line: {error}") from error
    return tokens[1:] if tokens[:1] == ["rwrl"] else tokens


def split_address(address: str) -> tuple[str, str]:
    host, _, port = address.rpartition(":")
    return ("127.0.0.1" if host == "localhost" else host), port


def preset_remotes(node) -> list[str]:
    """Every ``remote:`` address in a preset's real_robot settings."""
    if isinstance(node, dict):
        own = [node["remote"]] if isinstance(node.get("remote"), str) else []
        return own + [item for key, value in node.items() if key != "remote" for item in preset_remotes(value)]
    if isinstance(node, list):
        return [item for value in node for item in preset_remotes(value)]
    return []


class Session:
    def __init__(self, cwd: str | Path | None = None, env=None) -> None:
        self.cwd = (Path.cwd() if cwd is None else Path(cwd)).resolve()
        self.env = dict(os.environ if env is None else env)
        self.profile: Profile | None = None
        self.task: str | None = None
        self.robot: str | None = None
        self._stamp = None
        self.remembered = self._read_remembered()

    # -- last used, only to preselect ----------------------------------------------

    @property
    def _remembered_file(self) -> Path:
        return state_dir(self.cwd) / "last.json"

    def _read_remembered(self) -> dict:
        try:
            data = json.loads(self._remembered_file.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            return {}
        if not isinstance(data, dict):
            return {}
        return {key: value for key, value in data.items()
                if (key == "profile" and isinstance(value, str))
                or (key in ("tasks", "robots") and isinstance(value, dict))}

    def _remember(self) -> None:
        name = self.profile.name
        self.remembered["profile"] = name
        self.remembered.setdefault("tasks", {})[name] = self.task
        self.remembered.setdefault("robots", {})[name] = self.robot
        try:
            self._remembered_file.parent.mkdir(parents=True, exist_ok=True)
            self._remembered_file.write_text(json.dumps(self.remembered, indent=2) + "\n", encoding="utf-8")
        except OSError:
            pass  # only the preselection is lost

    # -- selection ---------------------------------------------------------------------

    def profiles(self) -> list[Profile]:
        return discover(self.cwd, self.env)

    def find(self, name: str) -> Profile:
        profiles = self.profiles()
        for profile in profiles:
            if profile.name == name:
                return profile
        known = ", ".join(profile.name for profile in profiles) or "none"
        raise SessionError(f"no device profile '{name}'; profiles: {known}")

    def select(self, profile: Profile) -> None:
        """Switch to ``profile`` with its last used task and robot."""
        if profile.error is not None:
            raise SessionError(f"{profile.name} does not load: {profile.error}")
        tasks = self.remembered.get("tasks", {})
        robots = self.remembered.get("robots", {})
        self.profile = profile
        task = tasks.get(profile.name)
        self.task = task if task in profile.tasks else next(iter(profile.tasks), None)
        robot = robots.get(profile.name)
        if len(profile.robots) == 1:
            robot = profile.robots[0]
        self.robot = robot if robot in profile.robots else None
        self._stamp = self._file_stamp()
        self._remember()

    def set_task(self, name: str | None) -> None:
        self.require_profile()
        if name is not None and name not in self.profile.tasks:
            known = ", ".join(self.profile.tasks) or "none"
            raise SessionError(f"{self.profile.name} has no task '{name}'; tasks: {known}")
        self.task = name
        self._remember()

    def set_robot(self, name: str | None) -> None:
        """``None`` addresses every robot (commands that need one then ask for ``--robot``)."""
        self.require_profile()
        if name is None and len(self.profile.robots) == 1:
            name = self.profile.robots[0]
        if name is not None and name not in self.profile.robots:
            raise SessionError(f"no robot '{name}'; robots: {', '.join(self.profile.robots)}")
        self.robot = name
        self._remember()

    def require_profile(self) -> Profile:
        if self.profile is None:
            raise SessionError("no device selected; type `switch`")
        return self.profile

    @property
    def task_path(self) -> Path | None:
        return None if self.profile is None or self.task is None else self.profile.tasks[self.task]

    def _file_stamp(self):
        stamps = []
        directory = self.profile.station.parent
        for path in (self.profile.station, directory / "task.yaml", directory / "tasks"):
            try:
                stat = path.stat()
                stamps.append((str(path), stat.st_mtime_ns, stat.st_size))
            except OSError:
                stamps.append((str(path), None, None))
        return tuple(stamps)

    def refresh(self) -> bool:
        """Reload the selected profile when its files changed; True when they did."""
        if self.profile is None or self._stamp == self._file_stamp():
            return False
        self.profile = load_profile(self.profile.name, self.profile.station)
        self._stamp = self._file_stamp()
        if self.task not in self.profile.tasks:
            self.task = None
        if self.profile.error is None and self.robot not in self.profile.robots:
            self.robot = self.profile.robots[0] if len(self.profile.robots) == 1 else None
        return True

    def save_default(self) -> Path:
        """Make the selection what plain ``rwrl`` commands in this directory use."""
        self.require_profile()
        return write_selection({"station": self.shown(self.profile.station),
                                "preset": None if self.task_path is None else self.shown(self.task_path)},
                               self.cwd)

    # -- display -----------------------------------------------------------------------

    def shown(self, path: Path) -> str:
        try:
            return str(path.relative_to(self.cwd))
        except ValueError:
            return str(path)

    def prompt_label(self) -> str:
        if self.profile is None:
            return "no device"
        label = self.profile.name + (f"/{self.task}" if self.task else "")
        return label + (f" [{self.robot}]" if self.robot else " [all robots]")

    def active_robots(self) -> list[str]:
        if self.profile is None or self.profile.error is not None:
            return []
        return [self.robot] if self.robot else self.profile.robots

    def info_rows(self) -> list[tuple[str, str, str, str]]:
        """What is selected, as ``(label, value, note, state)``; state is main, muted, bad or empty."""
        if self.profile is None:
            return [("device", "no device", "", "bad"), ("", "type `switch`", "", "")]
        rows = [("device", self.profile.name, "", "main"),
                ("station", self.shown(self.profile.station), "", "")]
        if self.profile.error:
            rows.append(("", "does not load: " + self.profile.error, "", "bad"))
        rows.append(("task", self.task, self.shown(self.task_path), "main") if self.task
                    else ("task", "none", "", "muted"))
        for name in self.active_robots():
            entry = self.profile.robot(name)
            address = getattr(entry.config, "robot_ip", None) or getattr(entry.config, "can_interface", None)
            route = f"daemon {entry.bind}" if entry.daemon_declared else "direct (no daemon: block)"
            rows.append(("robot", name, f"{entry.config.TYPE} {address or '(no address)'}, {route}", "main"))
        rows.append(("scope", "this terminal only", "`default` makes it the choice of plain rwrl", "muted"))
        return rows

    # -- commands ----------------------------------------------------------------------

    def child_env(self, tokens: list[str] = ()) -> dict[str, str]:
        """The environment of a child command. A line with its own ``--station`` gets none of the
        selection: the task and robot belong to the selected station."""
        env = dict(self.env)
        for name in (STATION_CONFIG_ENV, LEGACY_STATION_CONFIG_ENV, PRESET_CONFIG_ENV, ROBOT_NAME_ENV):
            env.pop(name, None)
        env[SHELL_ENV] = "1"
        if self.profile is not None and not has_option(tokens, "--station"):
            env[STATION_CONFIG_ENV] = str(self.profile.station)
            if self.task_path is not None:
                env[PRESET_CONFIG_ENV] = str(self.task_path)
            if self.robot is not None and len(self.profile.robots) > 1:
                env[ROBOT_NAME_ENV] = self.robot
        return env

    def prepare(self, line: str, commands) -> Run | Edit:
        """The child process for ``line``, or the line completed with the task's ``--config``."""
        tokens = split_line(line)
        command = command_name(tokens)
        helping = has_option(tokens, "--help", "-h")
        if command is None and not helping:
            raise SessionError("no command in the line; type `menu` or `help`")
        if command is not None and command not in commands:
            raise SessionError(f"unknown command '{command}'; type `menu` or `help`")
        if command == "shell":
            raise SessionError("already in the shell; type `switch` to change device")
        if command == "use" and not helping:
            raise SessionError("`use` would change every terminal here; type `switch`/`task` for this one, "
                               "or `default` to store this selection")
        migrating = command == "limits" and "migrate" in tokens
        if not helping and not migrating and command not in DEVICELESS_COMMANDS and not has_option(tokens, "--station"):
            profile = self.require_profile()
            if profile.error is not None:
                raise SessionError(f"{profile.name} no longer loads: {profile.error}")
        warnings, robots = [], []
        if command in TASK_CONFIG_COMMANDS and not helping:
            if not has_option(tokens, "--config", "--preset"):
                if self.task_path is None:
                    raise SessionError(f"{command} needs a task: type `task`, or pass --config <yaml>")
                return Edit(shlex.join([*tokens, "--config", self.shown(self.task_path)]))
            warnings, robots = self.check_config(option_value(tokens, "--config") or option_value(tokens, "--preset"))
        return Run([sys.executable, "-m", "real_world_rl.cli", *tokens], self.child_env(tokens), warnings, robots)

    def check_config(self, config: str | None) -> tuple[list[str], list[str]]:
        """Learning commands connect to the preset's own ``remote``: a warning for each that cannot be
        the selected station's daemon, and the station robots the others belong to."""
        if config is None or self.profile is None or self.profile.error is not None:
            return [], []
        import yaml
        path = Path(config).expanduser()
        try:
            document = yaml.safe_load((path if path.is_absolute() else self.cwd / path).read_bytes())
        except (OSError, yaml.YAMLError):
            return [], []  # the command reports an unreadable file itself
        if not isinstance(document, dict):
            return [], []
        binds = {name: entry.bind for name, entry in self.profile.loaded.robots.items()}
        warnings, robots = [], []
        for remote in dict.fromkeys(preset_remotes(document.get("real_robot", document))):
            matching = [name for name, bind in binds.items() if self._same_endpoint(remote, bind)]
            robots += [name for name in matching if self.profile.robot(name).daemon_declared]
            if not matching:
                stated = ", ".join(f"{name} {bind}" for name, bind in binds.items())
                warnings.append(f"{config} connects to {remote}, not to a daemon of {self.profile.name} ({stated}); "
                                "this command may drive another robot")
        return warnings, robots

    @staticmethod
    def _same_endpoint(remote: str, bind: str) -> bool:
        host, port = split_address(remote)
        bind_host, bind_port = split_address(bind)
        # A wildcard bind is reached by any of the robot host's addresses: only the port can differ.
        return port == bind_port and (bind_host in ("0.0.0.0", "[::]") or host == bind_host)
