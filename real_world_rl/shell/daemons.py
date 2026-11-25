"""Local daemons started by ``rwrl shell``.

A started daemon outlives the shell (own session, output in a log file). It is recorded with its
boot id and process start time, so a later shell in this directory can stop it and a reused pid is
never signalled. Daemons started any other way are never touched."""
from __future__ import annotations

from contextlib import contextmanager
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import time

from .session import Session, SessionError, state_dir

STOP_WAIT_S = 15.0
START_WAIT_S = 90.0
POLL_S = 0.2


def _boot_id() -> str:
    try:
        return Path("/proc/sys/kernel/random/boot_id").read_text().strip()
    except OSError:
        return ""


def process_start(pid: int) -> int | None:
    """Start time in clock ticks since boot; None when the process is gone or a zombie."""
    try:
        text = Path(f"/proc/{pid}/stat").read_text()
    except OSError:
        return None
    fields = text.rpartition(")")[2].split()
    return None if fields[0] in ("Z", "X") else int(fields[19])


class DaemonManager:
    def __init__(self, session: Session) -> None:
        self.session = session
        self.directory = state_dir(session.cwd)
        self._children: dict[int, subprocess.Popen] = {}

    @property
    def _file(self) -> Path:
        return self.directory / "daemons.json"

    @contextmanager
    def _locked(self):
        from filelock import FileLock
        self.directory.mkdir(parents=True, exist_ok=True)
        with FileLock(str(self._file) + ".lock", timeout=5):
            yield

    def _read(self) -> list[dict]:
        try:
            data = json.loads(self._file.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            return []
        return [entry for entry in data if isinstance(entry, dict)] if isinstance(data, list) else []

    def _write(self, entries: list[dict]) -> None:
        self._file.write_text(json.dumps(entries, indent=2) + "\n", encoding="utf-8")

    def _alive(self, entry: dict) -> bool:
        child = self._children.get(entry.get("pid"))
        if child is not None and child.poll() is not None:
            return False
        return (entry.get("boot_id") == _boot_id() and entry.get("start_ticks") is not None
                and process_start(entry.get("pid", -1)) == entry["start_ticks"])

    def running(self) -> list[dict]:
        """Daemons started by a shell in this directory that still run."""
        return [entry for entry in self._read() if self._alive(entry)]

    def of_profile(self) -> list[dict]:
        station = str(self.session.require_profile().station)
        return [entry for entry in self.running() if entry["station"] == station]

    def last_log(self) -> Path | None:
        station = str(self.session.require_profile().station)
        logs = [Path(entry["log"]) for entry in self._read() if entry.get("station") == station]
        return logs[-1] if logs else None

    def addresses(self) -> dict[str, str]:
        from real_world_rl.cli.connect import client_address
        profile = self.session.require_profile()
        if profile.error is not None:
            raise SessionError(f"{profile.name} does not load: {profile.error}")
        return {name: client_address(entry.bind) for name, entry in profile.loaded.robots.items()
                if entry.daemon_declared}

    def start(self, extra: list[str] = ()) -> dict:
        """Start the selected station's daemon (every robot of it) detached; does not wait."""
        from real_world_rl.cli.connect import probe_daemon
        addresses = self.addresses()
        profile = self.session.profile
        if not addresses:
            raise SessionError(f"{profile.name} declares no daemon: commands connect directly. "
                               "Add a `daemon:` block to its station.yaml to use one")
        for name, address in addresses.items():
            if probe_daemon(address):
                raise SessionError(f"something already listens on {address} ({name}); nothing started")
        if self.of_profile():
            raise SessionError("its daemon is already starting; see `daemon log`")
        logs = self.directory / "logs"
        logs.mkdir(parents=True, exist_ok=True)
        slug = re.sub(r"[^A-Za-z0-9_.-]", "_", profile.name)
        log = logs / f"daemon-{slug}-{time.strftime('%Y%m%dT%H%M%S')}.log"
        argv = [sys.executable, "-m", "real_world_rl.cli", "daemon", "--station", str(profile.station), *extra]
        env = {**self.session.child_env(), "PYTHONUNBUFFERED": "1"}
        with self._locked(), open(log, "ab") as output:
            child = subprocess.Popen(argv, stdin=subprocess.DEVNULL, stdout=output, stderr=subprocess.STDOUT,
                                     start_new_session=True, cwd=self.session.cwd, env=env)
            self._children[child.pid] = child
            entry = {"pid": child.pid, "start_ticks": process_start(child.pid), "boot_id": _boot_id(),
                     "profile": profile.name, "station": str(profile.station), "log": str(log)}
            # Finished entries of other stations are dropped; this station keeps its newest log.
            self._write([item for item in self._read() if self._alive(item)] + [entry])
        return entry

    def wait_ready(self, entry: dict, timeout_s: float = START_WAIT_S) -> bool:
        """True once every address answers; False when the process ended or time ran out."""
        from real_world_rl.cli.connect import probe_daemon
        pending = list(self.addresses().values())
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline and self._alive(entry):
            pending = [address for address in pending if not probe_daemon(address)]
            if not pending:
                return True
            time.sleep(POLL_S)
        return False

    def alive(self, entry: dict) -> bool:
        return self._alive(entry)

    def stop(self, entry: dict, timeout_s: float = STOP_WAIT_S) -> bool:
        """SIGTERM (the daemon disconnects its robot); True once it ended. Never SIGKILL."""
        if not self._alive(entry):
            return True
        try:
            os.kill(entry["pid"], signal.SIGTERM)
        except ProcessLookupError:
            return True
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if not self._alive(entry):
                return True
            time.sleep(POLL_S)
        return False


def tail(path: Path, lines: int = 30) -> str:
    try:
        return "\n".join(path.read_text(encoding="utf-8", errors="replace").splitlines()[-lines:])
    except OSError as error:
        return f"cannot read {path}: {error}"
