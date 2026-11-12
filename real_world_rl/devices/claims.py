"""Advisory, node-local exclusive claims on device endpoints."""
from __future__ import annotations

import os
import re
from pathlib import Path
from types import TracebackType
from typing import TYPE_CHECKING

import filelock

from real_world_rl.devices.errors import DeviceBusy

if TYPE_CHECKING:
    from real_world_rl.devices.robot import Robot

# None resolves XDG_CACHE_HOME at construction; tests may override the directory explicitly.
_DEFAULT_LOCK_DIR: Path | None = None
_LEGACY_LOCK_DIR = Path.home() / ".cache" / "robot_infra" / "claims"


def _default_lock_dir() -> Path:
    cache = os.environ.get("XDG_CACHE_HOME")
    return (Path(cache) if cache else Path.home() / ".cache") / "rwrl" / "claims"


def claim_key(robot: "Robot") -> str:
    config = robot.config
    address = getattr(config, "robot_ip", None) or getattr(config, "can_interface", None) or config.name
    return f"{robot.ROBOT_TYPE}:{address}"


class DeviceClaim:
    """Advisory, node-local exclusive claim on a device endpoint (filelock, timeout=0)."""

    def __init__(self, key: str, lock_dir: Path | None = None) -> None:
        self.key = key
        lock_dir = Path(lock_dir) if lock_dir is not None else (_DEFAULT_LOCK_DIR or _default_lock_dir())
        lock_dir.mkdir(parents=True, exist_ok=True)
        slug = re.sub(r"[^A-Za-z0-9_.-]", "_", key)
        # thread_local=False: a daemon acquires and releases from different gRPC worker threads.
        self._lock = filelock.FileLock(str(lock_dir / f"{slug}.lock"), timeout=0, thread_local=False)
        self._legacy_path = _LEGACY_LOCK_DIR / f"{slug}.lock"
        self._legacy_lock = None

    def acquire(self) -> None:
        try:
            self._lock.acquire()
        except filelock.Timeout:
            raise self._busy() from None
        # One-release compatibility: never create the old directory or a missing old lock.
        # Keep an existing legacy lock held with the new one, so an older process cannot
        # acquire it between our compatibility check and release.
        if self._legacy_lock is None and self._legacy_path.exists() and (
            self._legacy_path.resolve() != Path(self._lock.lock_file).resolve()
        ):
            legacy = filelock.FileLock(str(self._legacy_path), timeout=0, thread_local=False)
            try:
                legacy.acquire()
            except filelock.Timeout:
                self._lock.release()
                raise self._busy(legacy=True) from None
            except BaseException:
                self._lock.release()
                raise
            self._legacy_lock = legacy

    def _busy(self, *, legacy: bool = False) -> DeviceBusy:
        hint = f"stop the other process using '{self.key}', or set `claim: false` in the robot config"
        if legacy:
            hint += f"; legacy claim is held at {self._legacy_path}"
        return DeviceBusy(
            f"device '{self.key}' is claimed by another process",
            hint=hint,
            device=self.key,
        )

    @property
    def held(self) -> bool:
        return self._lock.is_locked

    def release(self) -> None:
        self._lock.release()
        if not self._lock.is_locked and self._legacy_lock is not None:
            self._legacy_lock.release()
            self._legacy_lock = None

    def __enter__(self) -> "DeviceClaim":
        self.acquire()
        return self

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc: BaseException | None,
        tb: TracebackType | None,
    ) -> None:
        self.release()
