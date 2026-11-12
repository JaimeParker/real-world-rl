"""real_world_rl.supervise: the supervisor process, the child heartbeat and the in-process StepWatchdog.

Supervisor tests run the real ``rwrl supervise`` (``python -m real_world_rl.cli supervise``) on tiny child scripts, with
timeouts on the 0.2 s scale.
"""
from __future__ import annotations

import io
import json
import os
import signal
import subprocess
import sys
import textwrap
import threading
import time
from pathlib import Path

import pytest

import real_world_rl
from real_world_rl.supervise import (
    HEARTBEAT_FD_ENV,
    Heartbeat,
    StepWatchdog,
    StepWatchdogTripped,
    SupervisorLost,
    capture_heartbeats,
)
import real_world_rl.supervise.supervisor as supervisor_module
from real_world_rl.supervise.supervisor import ESCALATED_EXIT_CODE, HeartbeatMonitor, SupervisorOptions, parse_args

pytestmark = pytest.mark.skipif(not sys.platform.startswith("linux"), reason="real_world_rl.supervise is Linux only")

# The repository root, which holds the real_world_rl package.
_REPOSITORY_ROOT = Path(real_world_rl.__file__).resolve().parents[1]
_RUN_TIMEOUT_S = 30
_FAST = ["--heartbeat-timeout-s", "0.2", "--grace-s", "0.2", "--kill-s", "0.2", "--startup-s", "5"]

_BEATING_CHILD = """
    import sys, time
    from real_world_rl.supervise import Heartbeat
    heartbeat = Heartbeat.from_env()
    assert heartbeat.enabled
    heartbeat.begin_long_operation()
    time.sleep(0.4)  # a reset: longer than --heartbeat-timeout-s, shorter than --long-op-s
    for _ in range(10):
        heartbeat.beat()
        time.sleep(0.02)
    sys.exit(int(sys.argv[1]))
"""

_SILENT_STUBBORN_CHILD = """
    import signal, time
    from real_world_rl.supervise import Heartbeat
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    signal.signal(signal.SIGTERM, signal.SIG_IGN)
    Heartbeat.from_env().beat()
    time.sleep(30)
"""

_SILENT_GRACEFUL_CHILD = """
    import time
    from real_world_rl.supervise import Heartbeat
    Heartbeat.from_env().beat()
    try:
        time.sleep(30)
    except KeyboardInterrupt:
        raise SystemExit(0)
"""

_HUNG_AT_STARTUP_CHILD = """
    import time
    time.sleep(30)
"""

_OPERATOR_STOP_CHILD = """
    import signal, sys, time
    from real_world_rl.supervise import Heartbeat
    heartbeat = Heartbeat.from_env()
    stop_requested = []
    signal.signal(signal.SIGINT, lambda *_: stop_requested.append(True))
    heartbeat.beat()
    print("ready", flush=True)
    while not stop_requested:
        heartbeat.beat()
        time.sleep(0.02)
    print("child got SIGINT", flush=True)
    for _ in range(25):  # a cleanup that keeps beating for 0.5 s
        heartbeat.beat()
        time.sleep(0.02)
    sys.exit(5)
"""


def _child_env() -> dict[str, str]:
    env = {key: value for key, value in os.environ.items() if key != HEARTBEAT_FD_ENV}
    env["PYTHONPATH"] = os.pathsep.join(p for p in (str(_REPOSITORY_ROOT), env.get("PYTHONPATH", "")) if p)
    return env


def _command(tmp_path: Path, child_source: str, *options: str, child_args: tuple[str, ...] = ()) -> list[str]:
    script = tmp_path / "child.py"
    script.write_text(textwrap.dedent(child_source))
    return [sys.executable, "-m", "real_world_rl.cli", "supervise", *options, "--",
            sys.executable, str(script), *child_args]


def _run(command: list[str]) -> subprocess.CompletedProcess:
    return subprocess.run(command, env=_child_env(), capture_output=True, text=True, timeout=_RUN_TIMEOUT_S)


def _events(log_path: Path) -> list[dict]:
    return [json.loads(line) for line in log_path.read_text().splitlines()]


def _escalations(log_path: Path) -> list[dict]:
    return [event for event in _events(log_path) if event["event"] == "escalate"]


# -- supervisor process -------------------------------------------------------------


@pytest.mark.parametrize("exit_code", [0, 7])
def test_beating_child_exit_code_is_passed_through(tmp_path, exit_code):
    log = tmp_path / "supervise.jsonl"
    result = _run(_command(tmp_path, _BEATING_CHILD, *_FAST, "--long-op-s", "5", "--log", str(log),
                           child_args=(str(exit_code),)))
    assert result.returncode == exit_code, result.stderr
    events = _events(log)
    assert [event["event"] for event in events][0] == "spawn"
    assert not _escalations(log)
    assert events[-1]["event"] == "child_exit" and events[-1]["exit_code"] == exit_code


def test_silent_child_ignoring_int_and_term_is_killed(tmp_path):
    log = tmp_path / "supervise.jsonl"
    started = time.monotonic()
    result = _run(_command(tmp_path, _SILENT_STUBBORN_CHILD, *_FAST, "--log", str(log)))
    assert time.monotonic() - started < 10
    assert result.returncode == 128 + signal.SIGKILL
    escalations = _escalations(log)
    assert [event["signal"] for event in escalations] == ["SIGINT", "SIGTERM", "SIGKILL"]
    assert {event["reason"] for event in escalations} == {"heartbeat silent"}
    gaps = [later["t_monotonic"] - earlier["t_monotonic"] for earlier, later in zip(escalations, escalations[1:])]
    assert all(gap >= 0.19 for gap in gaps)
    assert _events(log)[-1]["signal"] == "SIGKILL"
    assert "heartbeat silent" in result.stderr


def test_escalated_child_that_exits_cleanly_still_fails_the_supervisor(tmp_path):
    log = tmp_path / "supervise.jsonl"
    result = _run(_command(tmp_path, _SILENT_GRACEFUL_CHILD, *_FAST, "--log", str(log)))
    assert result.returncode == ESCALATED_EXIT_CODE
    assert [event["signal"] for event in _escalations(log)] == ["SIGINT"]


def test_child_hung_before_the_first_heartbeat_is_killed_after_startup_s(tmp_path):
    log = tmp_path / "supervise.jsonl"
    options = ["--heartbeat-timeout-s", "0.2", "--grace-s", "0.2", "--kill-s", "0.2", "--startup-s", "0.3"]
    result = _run(_command(tmp_path, _HUNG_AT_STARTUP_CHILD, *options, "--log", str(log)))
    assert result.returncode != 0
    events = _events(log)
    escalations = _escalations(log)
    assert escalations[0]["reason"] == "no heartbeat within --startup-s"
    assert escalations[0]["t_monotonic"] - events[0]["t_monotonic"] >= 0.29


def test_supervisor_sigint_is_forwarded_and_the_supervisor_waits_for_the_child(tmp_path):
    log = tmp_path / "supervise.jsonl"
    command = _command(tmp_path, _OPERATOR_STOP_CHILD, "--heartbeat-timeout-s", "0.5", "--startup-s", "5",
                       "--log", str(log))
    supervisor = subprocess.Popen(command, env=_child_env(), stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    try:
        assert supervisor.stdout.readline().strip() == "ready"
        supervisor.send_signal(signal.SIGINT)  # only the supervisor: the child runs in its own session
        stdout, stderr = supervisor.communicate(timeout=_RUN_TIMEOUT_S)
    finally:
        if supervisor.poll() is None:
            supervisor.kill()
            supervisor.wait()
    assert supervisor.returncode == 5, stderr
    assert "child got SIGINT" in stdout
    events = _events(log)
    assert any(event["event"] == "operator_signal" and event["forwarded"] == "SIGINT" for event in events)
    assert not _escalations(log)


def test_missing_command_and_bad_options_are_usage_errors():
    with pytest.raises(SystemExit):
        parse_args(["--heartbeat-timeout-s", "1"])
    with pytest.raises(SystemExit):
        parse_args(["--grace-s", "0", "--", "true"])
    options, command, log = parse_args(["--kill-s", "0.5", "--", "python", "-c", "pass"])
    assert options == SupervisorOptions(kill_s=0.5) and command == ["python", "-c", "pass"] and log is None


def test_heartbeat_monitor_deadlines():
    options = SupervisorOptions(heartbeat_timeout_s=2.0, startup_s=30.0, long_op_s=60.0)
    monitor = HeartbeatMonitor(options, started_at=100.0)
    assert monitor.deadline == 130.0 and monitor.phase == "startup"
    assert monitor.observe(b"L", 101.0) == "first_message"
    assert monitor.deadline == 161.0 and monitor.phase == "long_operation"
    assert monitor.observe(b"L.", 120.0) is None
    assert monitor.deadline == 122.0 and monitor.phase == "heartbeat"


_CHILD_EXIT_PATHS = [
    pytest.param(True, id="pidfd", marks=pytest.mark.skipif(not hasattr(os, "pidfd_open"), reason="no os.pidfd_open")),
    pytest.param(False, id="waitid-thread"),
]


@pytest.fixture
def in_process_supervise(tmp_path, monkeypatch):
    """``supervise()`` called in the test process, so the child-exit path can be chosen."""
    monkeypatch.delenv(HEARTBEAT_FD_ENV, raising=False)
    monkeypatch.setenv("PYTHONPATH", _child_env()["PYTHONPATH"])

    def run(child_source: str, options: SupervisorOptions, *, use_pidfd: bool, child_args: tuple[str, ...] = ()):
        monkeypatch.setattr(supervisor_module, "_USE_PIDFD", use_pidfd)
        script = tmp_path / "child.py"
        script.write_text(textwrap.dedent(child_source))
        log = io.StringIO()
        exit_code = supervisor_module.supervise([sys.executable, str(script), *child_args], options, log=log)
        return exit_code, [json.loads(line) for line in log.getvalue().splitlines()]

    return run


@pytest.mark.parametrize("use_pidfd", _CHILD_EXIT_PATHS)
def test_child_exit_is_detected_on_both_paths(in_process_supervise, use_pidfd):
    options = SupervisorOptions(heartbeat_timeout_s=0.2, grace_s=0.2, kill_s=0.2, startup_s=5.0, long_op_s=5.0)
    exit_code, events = in_process_supervise(_BEATING_CHILD, options, use_pidfd=use_pidfd, child_args=("7",))
    assert exit_code == 7
    assert events[-1]["event"] == "child_exit" and events[-1]["escalation"] is None


@pytest.mark.parametrize("use_pidfd", _CHILD_EXIT_PATHS)
def test_kill_is_detected_on_both_paths(in_process_supervise, use_pidfd):
    options = SupervisorOptions(heartbeat_timeout_s=0.2, grace_s=0.2, kill_s=0.2, startup_s=5.0)
    started = time.monotonic()
    exit_code, events = in_process_supervise(_SILENT_STUBBORN_CHILD, options, use_pidfd=use_pidfd)
    assert time.monotonic() - started < 10
    assert exit_code == 128 + signal.SIGKILL
    assert [event["signal"] for event in events if event["event"] == "escalate"] == ["SIGINT", "SIGTERM", "SIGKILL"]


# -- child heartbeat ----------------------------------------------------------------


def test_heartbeat_without_the_env_var_is_a_no_op():
    heartbeat = Heartbeat.from_env({})
    assert not heartbeat.enabled
    heartbeat.beat()
    heartbeat.begin_long_operation()


def test_capture_heartbeats_records_the_messages():
    previous = os.environ.get(HEARTBEAT_FD_ENV)
    with capture_heartbeats() as received:
        heartbeat = Heartbeat.from_env()
        assert heartbeat.enabled
        heartbeat.begin_long_operation()
        heartbeat.beat()
        heartbeat.beat()
        assert received() == b"L.."
        assert received() == b""
    assert os.environ.get(HEARTBEAT_FD_ENV) == previous


def test_heartbeat_ignores_a_full_pipe():
    with capture_heartbeats():
        heartbeat = Heartbeat.from_env()
        for _ in range(200_000):  # well past the default 64 KiB pipe capacity
            heartbeat.beat()


def test_heartbeat_raises_when_the_supervisor_is_gone():
    read_fd, write_fd = os.pipe()
    os.close(read_fd)
    try:
        heartbeat = Heartbeat.from_env({HEARTBEAT_FD_ENV: str(write_fd)})
        with pytest.raises(SupervisorLost):
            heartbeat.beat()
    finally:
        os.close(write_fd)


def test_heartbeat_rejects_an_fd_that_is_not_an_inherited_pipe(tmp_path):
    with open(tmp_path / "file", "w") as regular_file:
        with pytest.raises(ValueError, match="not a pipe"):
            Heartbeat.from_env({HEARTBEAT_FD_ENV: str(regular_file.fileno())})
    with pytest.raises(ValueError, match="not an open file descriptor"):
        Heartbeat.from_env({HEARTBEAT_FD_ENV: "999999"})
    with pytest.raises(ValueError, match="not an open file descriptor"):
        Heartbeat.from_env({HEARTBEAT_FD_ENV: "three"})


# -- in-process StepWatchdog ------------------------------------------------------


class _FakeArm:
    def __init__(self, error: Exception | None = None) -> None:
        self.stops = 0
        self.stopped = threading.Event()
        self._error = error

    def stop(self) -> None:
        self.stops += 1
        self.stopped.set()
        if self._error is not None:
            raise self._error


def test_step_watchdog_stops_the_arm_once_and_raises_on_check_in():
    arm = _FakeArm()
    watchdog = StepWatchdog(arm, timeout_s=0.05)
    try:
        watchdog.activate()
        assert arm.stopped.wait(timeout=5.0)
        time.sleep(0.2)  # several more timeouts: still one stop
        assert arm.stops == 1 and watchdog.tripped and watchdog.trip_count == 1
        with pytest.raises(StepWatchdogTripped, match="arm.stop"):
            watchdog.check_in()
        with pytest.raises(StepWatchdogTripped):
            watchdog.check_in()
        watchdog.activate()  # the next reset clears the trip
        watchdog.check_in()
    finally:
        watchdog.close()


def test_step_watchdog_stays_quiet_while_checked_in_and_while_inactive():
    arm = _FakeArm()
    watchdog = StepWatchdog(arm, timeout_s=0.2)
    try:
        time.sleep(0.3)  # never activated
        watchdog.activate()
        for _ in range(30):
            watchdog.check_in()
            time.sleep(0.01)
        watchdog.deactivate()  # a reset motion
        time.sleep(0.3)
        assert arm.stops == 0
        watchdog.check_in()
    finally:
        watchdog.close()


def test_step_watchdog_reports_a_failing_stop():
    arm = _FakeArm(error=RuntimeError("stop failed"))
    watchdog = StepWatchdog(arm, timeout_s=0.05)
    watchdog.activate()
    assert arm.stopped.wait(timeout=5.0)
    watchdog.close()
    with pytest.raises(StepWatchdogTripped, match="stop failed"):
        watchdog.check_in()


def test_step_watchdog_rejects_a_non_positive_timeout():
    with pytest.raises(ValueError):
        StepWatchdog(_FakeArm(), timeout_s=0.0)
