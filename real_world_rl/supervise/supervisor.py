"""``rwrl supervise [options] -- <command...>``: run a robot process under a
separate watchdog process.

The command runs in its own session (its own process group) and receives the write end of a
heartbeat pipe (``RWRL_HEARTBEAT_FD``, see ``heartbeat.py``). When the heartbeat goes
silent the supervisor escalates SIGINT -> SIGTERM -> SIGKILL to the whole group. In the
single-process robot path (the env drives the arm through an in-process SDK such as franky) that
kill is the supervisor's only lever: the SDK's control connection drops and the robot stops at
the firmware level. Out-of-process ``stop()`` exists only on the daemon route
(``rwrl daemon --watchdog-s``).

The supervisor waits on the pipe, a child-exit fd and a signal wakeup fd with ``select``; it
never sleep-polls. The child-exit fd is a pidfd where ``os.pidfd_open`` works, else a pipe that
a thread writes to once ``waitid(..., WNOWAIT)`` returns. Linux only.
"""
from __future__ import annotations

from real_world_rl.cli.cmd_supervise import build_parser
import ctypes
import dataclasses
import json
import os
import select
import signal
import subprocess
import sys
import threading
import time
from dataclasses import dataclass
from typing import Optional, Sequence, TextIO

from real_world_rl.supervise.heartbeat import HEARTBEAT_FD_ENV, LONG_OPERATION_BYTE

_PR_SET_PDEATHSIG = 1
# Exit code when the child was escalated but still exited 0 (it handled the SIGINT gracefully).
from real_world_rl.cli.errors import ESCALATED_EXIT_CODE, LAUNCH_FAILURE_EXIT_CODE, USAGE_EXIT_CODE


class UnsupportedPlatform(RuntimeError):
    pass


def require_supported_platform() -> None:
    if not sys.platform.startswith("linux"):
        raise UnsupportedPlatform(
            f"real_world_rl.supervise needs Linux (process groups, prctl); this is {sys.platform}"
        )


# Tests switch this off to exercise the waitid-thread fallback where pidfd_open exists.
_USE_PIDFD = hasattr(os, "pidfd_open")


def _open_child_exit_fd(pid: int) -> int:
    """An fd that becomes readable once ``pid`` has exited, without reaping it: the zombie keeps
    its pid (= the group id) reserved until the leftover group members are killed."""
    if _USE_PIDFD:
        try:
            return os.pidfd_open(pid)
        except OSError:  # ENOSYS on kernels < 5.3 or under a seccomp profile without it
            pass
    exit_read, exit_write = os.pipe()

    def signal_exit() -> None:
        try:
            os.waitid(os.P_PID, pid, os.WEXITED | os.WNOWAIT)
        except ChildProcessError:
            pass  # already reaped: it has exited either way
        try:
            os.write(exit_write, b"x")
        except OSError:
            pass  # the supervisor already closed the read end
        finally:
            os.close(exit_write)

    threading.Thread(target=signal_exit, name="supervise-child-exit", daemon=True).start()
    return exit_read


@dataclass(frozen=True)
class SupervisorOptions:
    heartbeat_timeout_s: float = 2.0
    grace_s: float = 3.0
    kill_s: float = 2.0
    startup_s: float = 30.0
    long_op_s: float = 60.0

    def __post_init__(self) -> None:
        for name in ("heartbeat_timeout_s", "grace_s", "kill_s", "startup_s", "long_op_s"):
            value = getattr(self, name)
            if not (value > 0 and value != float("inf")):
                raise ValueError(f"{name} must be a positive finite number of seconds, got {value}")




def parse_args(argv: Sequence[str] | None = None) -> tuple[SupervisorOptions, list[str], Optional[str]]:
    parser = build_parser()
    return options_from_args(parser.parse_args(argv), parser)


def options_from_args(args, parser):
    command = list(args.child_command)
    if command and command[0] == "--":
        command = command[1:]
    if not command:
        parser.error("no command given; usage: rwrl supervise [options] -- <command...>")
    try:
        options = SupervisorOptions(
            heartbeat_timeout_s=args.heartbeat_timeout_s, grace_s=args.grace_s, kill_s=args.kill_s,
            startup_s=args.startup_s, long_op_s=args.long_op_s,
        )
    except ValueError as error:
        parser.error(str(error))
    return options, command, args.log


class HeartbeatMonitor:
    """When the child is next overdue, given the messages received so far."""

    def __init__(self, options: SupervisorOptions, started_at: float) -> None:
        self._options = options
        self.phase = "startup"
        self.deadline = started_at + options.startup_s

    def observe(self, messages: bytes, now: float) -> Optional[str]:
        """Account for ``messages`` received at ``now``; return a loggable event name or None."""
        previous_phase = self.phase
        if messages[-1:] == LONG_OPERATION_BYTE:
            self.phase, self.deadline = "long_operation", now + self._options.long_op_s
        else:
            self.phase, self.deadline = "heartbeat", now + self._options.heartbeat_timeout_s
        if previous_phase == "startup":
            return "first_message"
        if self.phase == "long_operation":
            return "long_operation"
        return None

    @property
    def overdue_reason(self) -> str:
        return {"startup": "no heartbeat within --startup-s", "heartbeat": "heartbeat silent",
                "long_operation": "long operation exceeded --long-op-s"}[self.phase]


class _EventLog:
    def __init__(self, stream: Optional[TextIO]) -> None:
        self._stream = stream

    def write(self, event: str, **fields) -> None:
        if self._stream is None:
            return
        record = {"event": event, "t_monotonic": time.monotonic(), "t_wall": time.time(), **fields}
        self._stream.write(json.dumps(record) + "\n")
        self._stream.flush()


def _say(message: str) -> None:
    print(f"real_world_rl.supervise: {message}", file=sys.stderr, flush=True)


def _shell_exit_code(returncode: int) -> int:
    """A child killed by signal N reports -N; report it the way a shell does (128 + N)."""
    return returncode if returncode >= 0 else 128 - returncode


def _signal_group(pgid: int, signum: int) -> bool:
    try:
        os.killpg(pgid, signum)
    except ProcessLookupError:
        return False
    return True


def _parent_death_signal_setter(signum: int, parent_pid: int):
    """``preexec_fn`` that makes the kernel send ``signum`` to the child when the supervisor dies."""
    libc = ctypes.CDLL(None, use_errno=True)

    def set_parent_death_signal() -> None:
        libc.prctl(_PR_SET_PDEATHSIG, signum, 0, 0, 0)
        if os.getppid() != parent_pid:  # the supervisor died before prctl took effect
            os._exit(128 + signum)

    return set_parent_death_signal


def supervise(command: Sequence[str], options: SupervisorOptions, *, log: Optional[TextIO] = None) -> int:
    """Run ``command`` until it exits; return its shell-style exit code (non-zero after an escalation)."""
    require_supported_platform()
    events = _EventLog(log)
    heartbeat_read, heartbeat_write = os.pipe()
    wakeup_read, wakeup_write = os.pipe()
    for fd in (heartbeat_read, wakeup_read, wakeup_write):
        os.set_blocking(fd, False)
    pending_signals: list[int] = []
    previous_handlers = {signum: signal.signal(signum, lambda signum, frame: pending_signals.append(signum))
                         for signum in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP)}
    previous_wakeup_fd = signal.set_wakeup_fd(wakeup_write)
    child: Optional[subprocess.Popen] = None
    child_exit_fd: Optional[int] = None
    try:
        try:
            child = subprocess.Popen(
                list(command),
                env={**os.environ, HEARTBEAT_FD_ENV: str(heartbeat_write)},
                pass_fds=(heartbeat_write,),
                start_new_session=True,  # own group: the ladder reaches every process it started
                preexec_fn=_parent_death_signal_setter(signal.SIGINT, os.getpid()),
            )
        finally:
            os.close(heartbeat_write)  # only the child holds the write end, so its exit gives EOF
        child_exit_fd = _open_child_exit_fd(child.pid)
        pgid = child.pid
        events.write("spawn", pid=child.pid, pgid=pgid, command=list(command), options=dataclasses.asdict(options))
        return _supervise_child(child, pgid, child_exit_fd, heartbeat_read, wakeup_read, pending_signals, options, events)
    except BaseException:
        if child is not None and child.poll() is None:
            _signal_group(child.pid, signal.SIGINT)
        raise
    finally:
        signal.set_wakeup_fd(previous_wakeup_fd)
        for signum, handler in previous_handlers.items():
            signal.signal(signum, handler)
        for fd in (heartbeat_read, wakeup_read, wakeup_write, child_exit_fd):
            if fd is not None:
                os.close(fd)


def _supervise_child(child, pgid, child_exit_fd, heartbeat_read, wakeup_read, pending_signals, options, events) -> int:
    monitor = HeartbeatMonitor(options, time.monotonic())
    ladder = [(signal.SIGINT, options.grace_s), (signal.SIGTERM, options.kill_s), (signal.SIGKILL, None)]
    escalation_reason: Optional[str] = None
    next_rung = 0
    rung_deadline: Optional[float] = None
    watched = [heartbeat_read, wakeup_read, child_exit_fd]
    while True:
        deadline = monitor.deadline if escalation_reason is None else rung_deadline
        timeout = None if deadline is None else max(0.0, deadline - time.monotonic())
        ready, _, _ = select.select(watched, [], [], timeout)
        now = time.monotonic()
        if wakeup_read in ready:
            _drain(wakeup_read)
        while pending_signals:
            signum = pending_signals.pop(0)
            name = signal.Signals(signum).name
            _say(f"received {name}; forwarding SIGINT to the robot process group {pgid} and waiting for it to exit")
            events.write("operator_signal", signal=name, forwarded="SIGINT")
            _signal_group(pgid, signal.SIGINT)
        if heartbeat_read in ready:
            messages = _drain(heartbeat_read)
            if messages is None:  # EOF: the child closed the pipe or exited
                watched.remove(heartbeat_read)
                events.write("heartbeat_pipe_closed")
            elif escalation_reason is None:
                event = monitor.observe(messages, now)
                if event is not None:
                    events.write(event)
        if child_exit_fd in ready:
            break
        if escalation_reason is None and now >= monitor.deadline:
            escalation_reason = monitor.overdue_reason
            _say(f"{escalation_reason}; escalating SIGINT -> SIGTERM -> SIGKILL on process group {pgid}")
        if escalation_reason is not None and (rung_deadline is None or now >= rung_deadline) and next_rung < len(ladder):
            signum, wait_s = ladder[next_rung]
            next_rung += 1
            rung_deadline = None if wait_s is None else now + wait_s
            events.write("escalate", reason=escalation_reason, signal=signum.name, phase=monitor.phase)
            _say(f"sending {signum.name} to process group {pgid}")
            _signal_group(pgid, signum)
    # The child is a zombie until reaped, so its pid (= pgid) cannot be recycled yet: kill whatever
    # it left in its group, so nothing it started keeps running unsupervised.
    _signal_group(pgid, signal.SIGKILL)
    returncode = child.wait()
    exit_code = _shell_exit_code(returncode)
    events.write("child_exit", returncode=returncode, exit_code=exit_code,
                 signal=signal.Signals(-returncode).name if returncode < 0 else None,
                 escalation=escalation_reason)
    if escalation_reason is not None:
        _say(f"robot process exited with code {exit_code} after escalation ({escalation_reason})")
        return exit_code or ESCALATED_EXIT_CODE
    return exit_code


def _drain(fd: int) -> Optional[bytes]:
    """Everything readable on ``fd`` now; None on EOF."""
    chunks = []
    while True:
        try:
            chunk = os.read(fd, 4096)
        except BlockingIOError:
            break
        if not chunk:
            return b"".join(chunks) if chunks else None
        chunks.append(chunk)
    return b"".join(chunks)


def main(argv: Sequence[str] | None = None) -> int:
    return run(build_parser().parse_args(argv), None)


def run(args, ctx) -> int:
    options, command, log_path = options_from_args(args, getattr(args, "_parser", None) or build_parser())
    try:
        require_supported_platform()
    except UnsupportedPlatform as error:
        _say(str(error))
        return USAGE_EXIT_CODE
    log = open(log_path, "a", encoding="utf-8") if log_path is not None else None
    try:
        return supervise(command, options, log=log)
    except (FileNotFoundError, PermissionError) as error:
        _say(f"cannot start {command[0]!r}: {error}")
        return LAUNCH_FAILURE_EXIT_CODE
    finally:
        if log is not None:
            log.close()
