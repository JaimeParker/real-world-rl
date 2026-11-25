"""Background daemon status for the toolbar: a TCP probe every few seconds; the daemon's identity
is asked once per selection and each time it comes up."""
from __future__ import annotations

import json
import subprocess
import sys
import threading

PERIOD_S = 3.0
IDENTITY_TIMEOUT_S = 15.0

DIRECT, DOWN, CHECKING, MATCH, MISMATCH, UNKNOWN = "direct", "down", "checking", "match", "mismatch", "unknown"


def query_identity(station: str, robot: str, env: dict, cwd) -> dict:
    try:
        result = subprocess.run([sys.executable, "-m", "real_world_rl.shell.identity", station, robot],
                                capture_output=True, text=True, timeout=IDENTITY_TIMEOUT_S, env=env, cwd=cwd,
                                stdin=subprocess.DEVNULL, start_new_session=True)
        return json.loads(result.stdout.strip().splitlines()[-1])
    except (OSError, subprocess.SubprocessError, ValueError, IndexError) as error:
        return {"error": str(error)}


class StatusMonitor:
    def __init__(self, session, on_change, *, query=query_identity, probe=None) -> None:
        self._session = session
        self._on_change = on_change
        self._query = query
        self._probe = probe
        self._wake = threading.Event()
        self._stopping = False
        self._recheck = True
        self.states: dict[str, tuple[str, str]] = {}  # robot -> (state, detail)
        self._target = None
        self._thread = threading.Thread(target=self._loop, name="rwrl-shell-status", daemon=True)

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stopping = True
        self._wake.set()

    def recheck(self) -> None:
        self._recheck = True
        self._wake.set()

    def _loop(self) -> None:
        while not self._stopping:
            self._wake.clear()
            try:
                self.update()
            except Exception:  # the toolbar must never take the shell down
                self.states = {}
            self._wake.wait(PERIOD_S)

    def update(self) -> None:
        from real_world_rl.cli.connect import client_address, probe_daemon
        probe = self._probe or probe_daemon
        profile = self._session.profile
        robots = self._session.active_robots()
        target = (id(profile), tuple(robots))
        fresh, self._recheck = self._recheck or target != self._target, False
        self._target = target
        previous, states = ({} if fresh else self.states), {}
        for name in robots:
            entry = profile.robot(name)
            if not entry.daemon_declared:
                states[name] = (DIRECT, "")
                continue
            address = client_address(entry.bind)
            if not probe(address):
                states[name] = (DOWN, address)
                continue
            before = previous.get(name, (DOWN, ""))
            if before[0] in (MATCH, MISMATCH):
                states[name] = before
                continue
            self._publish({**states, name: (CHECKING, address)}, target)
            answer = self._query(str(profile.station), name, self._session.child_env(), self._session.cwd)
            if "error" in answer:
                states[name] = (UNKNOWN, f"{address}: {answer['error']}")
            elif answer["mismatches"]:
                states[name] = (MISMATCH, f"{address}: " + "; ".join(answer["mismatches"]))
            else:
                states[name] = (MATCH, address)
        self._publish(states, target)

    def _publish(self, states, target) -> None:
        # A selection made meanwhile must not show the previous device's status.
        if target != (id(self._session.profile), tuple(self._session.active_robots())):
            self._recheck = True
            states = {}
        if states != self.states:
            self.states = states
            self._on_change()
