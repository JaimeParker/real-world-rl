"""Conservative read-only Linux device ownership evidence shared by API and CLI."""
from __future__ import annotations

import os
from pathlib import Path
import re
import shutil
import subprocess

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
    # rwrl runs the daemon and direct connections in its own process (rwrl daemon, rwrl --direct ...).
    "rwrl",
    "real_world_rl.cli",
}

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
        control = next(
            (
                Path(arg).name
                for arg in argv
                if arg in CONTROL_PROGRAMS or Path(arg).name in CONTROL_PROGRAMS
            ),
            None,
        )
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

