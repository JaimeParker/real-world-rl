"""Conservative read-only Linux device ownership evidence shared by API and CLI."""
from __future__ import annotations

from collections.abc import Sequence
import os
from pathlib import Path
import re
import shutil
import subprocess

# Programs whose whole process holds the hardware connection.
CONTROL_PROGRAMS = {
    "robot_infra.server",
    "robot_infra.server.daemon",
    "record_real_demos.py",
    "train_real_world.py",
    "deploy_real_robot.py",
    "real_world_rl.training",
    "real_world_rl.demos",
    "real_world_rl.deploy",
    "real-world-rl-record",
    "real-world-rl-train",
    "real-world-rl-deploy",
}

# Command-line entry points that only own hardware while running a connecting subcommand;
# offline subcommands (`rwrl --help`, `rwrl workspace show`, `rwrl teleop inspect-stop`) do not.
SUBCOMMAND_PROGRAMS = {"rwrl", "robot-infra", "real_world_rl.cli", "robot_infra.cli"}
CONNECTING_SUBCOMMANDS = {
    "status", "q", "tcp", "home", "gripper", "camera", "stop", "recover", "daemon", "setup",
    "gello-calibrate", "teleop", "workspace", "train", "record", "deploy", "collect-reward",
}
# Subcommands that connect only for some of their own sub-subcommands.
CONNECTING_SUB_SUBCOMMANDS = {
    "teleop": {"run", "read", "calibrate", "diagnose"},
    "workspace": {"capture"},
}
# Global options that consume the next token; the remaining global options are flags.
GLOBAL_OPTIONS_WITH_VALUE = {"--station", "--robot", "--preset"}


def _positionals(tokens: Sequence[str]) -> list[str] | None:
    """Tokens without global options and their values; None when help was requested."""
    found = []
    skip_value = False
    for token in tokens:
        if skip_value:
            skip_value = False
        elif token in ("-h", "--help"):
            return None
        elif token.startswith("--"):
            skip_value = "=" not in token and token in GLOBAL_OPTIONS_WITH_VALUE
        else:
            found.append(token)
    return found


def controlling_command(argv: Sequence[str]) -> str | None:
    """Describe the hardware-controlling command in ``argv``, or None for an offline one.

    A whole-program match (see CONTROL_PROGRAMS) is always controlling. For the subcommand
    entry points the first token after the global options is the subcommand; ``teleop`` and
    ``workspace`` additionally need a connecting sub-subcommand. Help requests never control.
    """
    for index, arg in enumerate(argv):
        name = Path(arg).name
        if arg in CONTROL_PROGRAMS or name in CONTROL_PROGRAMS:
            return name
        if arg in SUBCOMMAND_PROGRAMS or name in SUBCOMMAND_PROGRAMS:
            positionals = _positionals(argv[index + 1:])
            if not positionals or positionals[0] not in CONNECTING_SUBCOMMANDS:
                return None
            subcommand = positionals[0]
            allowed = CONNECTING_SUB_SUBCOMMANDS.get(subcommand)
            if allowed is None:
                return f"{name} {subcommand}"
            if len(positionals) > 1 and positionals[1] in allowed:
                return f"{name} {subcommand} {positionals[1]}"
            return None
    return None


def process_owners(paths=(), proc: Path = Path("/proc")) -> list[str]:
    """Conservative process evidence, never a signal or a kill. Do not report raw argv."""
    found = []
    for folder in proc.glob("[0-9]*"):
        if folder.name == str(os.getpid()):
            continue
        try:
            argv = (
                (folder / "cmdline").read_bytes().decode(errors="replace").split("\0")
            )
        except OSError:
            continue
        control = controlling_command(argv)
        try:
            maps = (folder / "maps").read_text(encoding="utf-8")
        except (OSError, UnicodeError):
            maps = ""
        loaded_sdk = any(
            word in maps
            for word in (
                "/franky/",
                "/hid.cpython-",
                "/hidraw.cpython-",
                "/pyrealsense2/",
            )
        )
        if control or loaded_sdk:
            found.append(f"pid {folder.name}: {control or 'loaded hardware SDK'}")
    existing = [str(path) for path in paths if Path(path).exists()]
    if existing and shutil.which("fuser"):
        try:
            result = subprocess.run(
                ["fuser", *existing], capture_output=True, text=True, timeout=5
            )
            for pid in re.findall(r"\b\d+\b", result.stdout):
                if pid != str(os.getpid()):
                    found.append(f"pid {pid}: open target device")
        except (OSError, subprocess.TimeoutExpired):
            found.append("device ownership probe unavailable")
    return sorted(set(found))

