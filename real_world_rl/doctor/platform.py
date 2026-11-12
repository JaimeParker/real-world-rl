"""Discover host capabilities without assuming a machine model or Linux release."""

from __future__ import annotations

import os
import platform
import shutil
from dataclasses import dataclass
from pathlib import Path


def read_text(path: Path) -> str:
    try:
        return path.read_text(encoding="utf-8").strip()
    except (OSError, UnicodeError):
        return ""


def find_checkout() -> Path | None:
    """Find our source tree, never treating site-packages as a repository."""
    for start in (Path.cwd(), Path(__file__).resolve().parent):
        for candidate in (start, *start.parents):
            if (candidate / "pyproject.toml").is_file() and (
                candidate / "real_world_rl/doctor/report.py"
            ).is_file():
                return candidate
    return None


@dataclass(frozen=True)
class Capabilities:
    system: str
    distribution: str
    version: str
    architecture: str
    container: bool
    wsl: bool
    debian_family: bool
    etc: Path = Path("/etc")
    sysfs: Path = Path("/sys")
    runtime: Path = Path("/run")

    @classmethod
    def detect(
        cls,
        *,
        etc: Path = Path("/etc"),
        sysfs: Path = Path("/sys"),
        proc: Path = Path("/proc"),
        runtime: Path = Path("/run"),
        filesystem: Path = Path("/"),
    ) -> Capabilities:
        release = {}
        for line in read_text(etc / "os-release").splitlines():
            if "=" in line:
                key, value = line.split("=", 1)
                release[key] = value.strip("\"'")
        family = {release.get("ID", ""), *release.get("ID_LIKE", "").split()}
        cgroup = read_text(proc / "1/cgroup")
        container = (
            bool(os.environ.get("container"))
            or any(
                (filesystem / marker).exists()
                for marker in (".dockerenv", "run/.containerenv")
            )
            or any(
                word in cgroup
                for word in ("docker", "lxc", "kubepods", "containerd", "libpod")
            )
        )
        kernel = platform.release() + " " + read_text(proc / "sys/kernel/osrelease")
        return cls(
            platform.system(),
            release.get("ID", "unknown"),
            release.get("VERSION_ID", ""),
            platform.machine(),
            container,
            "microsoft" in kernel.lower(),
            bool(family & {"debian", "ubuntu"}),
            etc,
            sysfs,
            runtime,
        )

    @property
    def linux(self) -> bool:
        return self.system == "Linux"

    @property
    def device_probes(self) -> bool:
        return self.linux and not self.container and not self.wsl

    def block_reason(self, capability: str) -> str:
        """Check each repair independently; Python repairs do not need host privileges."""
        if capability == "python":
            return ""
        if not self.device_probes:
            return (
                "system repair requires a native Linux host outside containers and WSL"
            )
        if capability == "packages":
            if not self.debian_family or not shutil.which("apt-get"):
                return "automatic package installation needs Debian/Ubuntu-compatible apt-get; install these packages manually"
        elif capability == "udev":
            if not shutil.which("udevadm") or not (self.etc / "udev/rules.d").is_dir():
                return "udev rules directory and udevadm are required; install/configure udev first"
        elif capability == "groups":
            if not all(shutil.which(tool) for tool in ("groupadd", "usermod")):
                return "groupadd and usermod are required for user-group repairs"
        elif capability == "pam":
            if not (self.etc / "security/limits.d").is_dir():
                return "PAM limits.d is unavailable; configure realtime limits using this host's login system"
        elif capability == "systemd":
            if (
                not shutil.which("systemctl")
                or not (self.runtime / "systemd/system").is_dir()
            ):
                return "service repair requires a running systemd instance"
        elif capability != "system":
            return "unsupported repair capability: " + capability
        return ""
