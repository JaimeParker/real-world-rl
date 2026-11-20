"""Explicit, capability-aware repair plans and private audit reports."""

from __future__ import annotations

from real_world_rl.cli.errors import DOCTOR_UNRESOLVED_EXIT_CODE, SUCCESS_EXIT_CODE

import hashlib
import json
import os
import re
import shlex
import stat
import subprocess
import sys
import uuid
from contextlib import contextmanager
from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from tempfile import TemporaryFile

from real_world_rl.doctor.platform import Capabilities


@dataclass
class Finding:
    id: str
    status: str
    message: str
    hint: str = ""


@dataclass
class Action:
    id: str
    check: str
    title: str
    argv: list[str] = field(default_factory=list)
    target: str = ""
    content: str = ""
    dependencies: list[str] = field(default_factory=list)
    privileged: bool = False
    python: bool = False
    exclusive: bool = False
    pending: str = ""
    status: str = "PLANNED"
    detail: str = ""
    # SHA of the file inspected while planning. Never overwrite a later edit.
    before: str | None = None
    capability: str = "system"
    prerequisites: dict[str, str | None] = field(default_factory=dict)
    validate_argv: list[str] = field(default_factory=list)
    verify_argv: list[str] = field(default_factory=list)
    verify_line: str = ""
    after_argv: list[list[str]] = field(default_factory=list)
    advice: str = ""


def redact(value):
    if isinstance(value, dict):
        return {
            key: "<redacted>"
            if re.search(
                r"token|password|secret|credential|authorization|api[_-]?key",
                str(key),
                re.I,
            )
            else redact(item)
            for key, item in value.items()
        }
    if isinstance(value, (list, tuple)):
        result = []
        for index, item in enumerate(value):
            previous = value[index - 1] if index else ""
            sensitive_flag = isinstance(previous, str) and re.fullmatch(
                r"--(?:token|password|secret|api[_-]?key|authorization)", previous, re.I
            )
            result.append("<redacted>" if sensitive_flag else redact(item))
        return result
    if not isinstance(value, str):
        return value
    value = re.sub(r"(https?://)[^/\s@]+@", r"\1<redacted>@", value)
    value = re.sub(
        r"(?i)(authorization\s*[:=]\s*)(?:bearer|basic)\s+[^\s,;]+",
        r"\1<redacted>",
        value,
    )
    return re.sub(
        r"(?i)((?:token|password|secret|authorization)[\w-]*\s*[:=]\s*)[^\s,;]+",
        r"\1<redacted>",
        value,
    )


def project_environment() -> bool:
    if sys.prefix != sys.base_prefix:
        return True
    # A conda base environment is not a project environment.
    prefix = Path(sys.prefix).resolve()
    metadata = prefix / "conda-meta"
    if not metadata.is_dir() or os.environ.get("CONDA_DEFAULT_ENV") == "base":
        return False
    executable = os.environ.get("CONDA_EXE")
    if executable and Path(executable).resolve().parent.parent == prefix:
        return False
    # Prefix environments need not live under an envs/ directory. A conda package
    # record identifies a base/tooling environment even when it is not active.
    return not any(metadata.glob("conda-[0-9]*.json"))


def digest(path: Path) -> str | None:
    try:
        return hashlib.sha256(path.read_bytes()).hexdigest()
    except FileNotFoundError:
        return None


def state_directory() -> Path:
    if sys.platform == "win32":
        return (
            Path(os.environ.get("LOCALAPPDATA", Path.home() / "AppData/Local"))
            / "rwrl/doctor"
        )
    if sys.platform == "darwin":
        return Path.home() / "Library/Application Support/rwrl/doctor"
    state = Path(os.environ.get("XDG_STATE_HOME", Path.home() / ".local/state"))
    if not state.is_absolute():
        state = Path.home() / ".local/state"
    return state / "rwrl/doctor"


class Report:
    def __init__(self, root: Path | None = None, *, directory: Path | None = None):
        explicit = directory or os.environ.get("RWRL_DOCTOR_REPORT_DIR")
        candidates = (
            [Path(explicit).expanduser()]
            if explicit
            else (
                [root / "local/doctor", state_directory()]
                if root
                else [state_directory()]
            )
        )
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S")
        for index, base in enumerate(candidates):
            try:
                base.mkdir(parents=True, exist_ok=True)
                self.path = base / f"{stamp}-{uuid.uuid4().hex[:8]}"
                self.path.mkdir(mode=0o700)
                self.path.chmod(0o700)
                break
            except OSError:
                if index == len(candidates) - 1:
                    raise
        self.log = self.path / "commands.log"
        self.log.touch(mode=0o600)

    def save(self, name: str, payload) -> None:
        path = self.path / name
        temp = path.with_suffix(path.suffix + ".tmp")
        with temp.open("w", encoding="utf-8") as stream:
            os.chmod(temp, 0o600)
            json.dump(redact(payload), stream, indent=2, ensure_ascii=False)
            stream.write("\n")
        temp.replace(path)

    def run(
        self,
        argv: list[str],
        *,
        input_text: str | None = None,
        timeout: int = 300,
        stdout_only: bool = False,
    ):
        # Only developer-built argv is accepted. No shell expansion or stdin passwords.
        with self.log.open("a", encoding="utf-8") as stream:
            stream.write("$ " + shlex.join(redact(argv)) + "\n")
            try:
                result = subprocess.run(
                    argv,
                    input=input_text,
                    capture_output=True,
                    text=True,
                    encoding="utf-8",
                    errors="replace",
                    timeout=timeout,
                )
            except (OSError, subprocess.TimeoutExpired) as error:
                stream.write(redact(str(error)) + "\n")
                return 1, str(error)
            output = result.stdout + result.stderr
            stream.write(redact(output) + f"\nexit={result.returncode}\n")
        return (
            result.returncode,
            result.stdout if stdout_only and result.returncode == 0 else output,
        )


    @contextmanager
    def background(self, argv):
        """Log a short-lived probe and always reap the child we started."""
        with TemporaryFile() as output:
            with self.log.open("a", encoding="utf-8") as stream:
                stream.write("$ " + shlex.join(redact(argv)) + "\n")
            process = subprocess.Popen(
                argv, stdin=subprocess.DEVNULL, stdout=output, stderr=output,
            )

            def diagnostics():
                return os.pread(output.fileno(), 8192, 0).decode("utf-8", errors="replace")

            try:
                yield process, diagnostics
            finally:
                if process.poll() is None:
                    process.terminate()
                try:
                    process.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=2)
                with self.log.open("a", encoding="utf-8") as stream:
                    stream.write(redact(diagnostics()) + f"\nexit={process.returncode}\n")


def user_service_directory() -> Path:
    config = Path(os.environ.get("XDG_CONFIG_HOME") or Path.home() / ".config").expanduser()
    if not config.is_absolute():
        config = Path.home() / ".config"
    return config / "systemd/user"


def _allowed_destination(target: Path, action: Action, capabilities: Capabilities) -> bool:
    if action.privileged:
        return (
            str(target).startswith("/etc/udev/rules.d/99-rwrl-")
            or str(target) == "/etc/security/limits.d/99-rwrl-realtime.conf"
            or target == capabilities.etc / "ssh/sshd_config.d/99-rwrl.conf"
        )
    return (
        target.parent == user_service_directory()
        and re.fullmatch(r"rwrl-tunnel-[a-z0-9-]+\.service", target.name) is not None
    )


class ConcurrentEdit(RuntimeError):
    """A dedicated repair file changed while being staged."""


def _publish(report, prefix, source, target, *, expected, mode):
    temporary = str(target) + ".rwrl-" + report.path.name
    code, output = report.run(prefix + ["install", "-m", f"{mode:04o}", "--", str(source), temporary])
    if code:
        return code, output
    try:
        if target.is_symlink() or digest(target) != expected:
            raise ConcurrentEdit("configuration changed while staging; inspect again")
    except (ConcurrentEdit, OSError):
        report.run(prefix + ["rm", "-f", "--", temporary])
        raise
    code, output = report.run(prefix + ["mv", "-fT", "--", temporary, str(target)])
    if code:
        report.run(prefix + ["rm", "-f", "--", temporary])
    return code, output


def file_action(
    check: str,
    title: str,
    target: str,
    content: str,
    *,
    pending: str = "",
    exclusive: bool = True,
) -> Action:
    return Action(
        "file:" + target,
        check,
        title,
        target=target,
        content=content,
        privileged=True,
        pending=pending,
        exclusive=exclusive,
        before=digest(Path(target)),
        capability="udev"
        if "/udev/" in target
        else "pam"
        if "/security/limits.d/" in target
        else "system",
    )


def execute(
    actions: list[Action],
    report: Report,
    *,
    allowed: bool,
    busy,
    assume_yes: bool,
    sdk_only: bool = False,
    capabilities: Capabilities | None = None,
    command=None,
) -> None:
    capabilities = capabilities or Capabilities.detect()
    run_command = command or report.run
    done: dict[str, Action] = {}
    for action in actions:
        done[action.id] = action
        if sdk_only and not action.id.startswith("sdk:"):
            action.status, action.detail = (
                "SKIPPED",
                "--install-sdk selects Franky wheel replacement only",
            )
            continue
        if not allowed:
            action.status, action.detail = (
                "BLOCKED",
                "repairs disabled for this invocation (including --mock)",
            )
            continue
        reason = capabilities.block_reason(
            "python" if action.python else action.capability
        )
        if reason:
            action.status, action.detail = "BLOCKED", reason
            continue
        if any(
            dep not in done or done[dep].status not in ("PASS", "PENDING_ACTION")
            for dep in action.dependencies
        ):
            action.status, action.detail = (
                "BLOCKED",
                "a prerequisite repair did not complete",
            )
            continue
        if action.python and not project_environment():
            action.status, action.detail = (
                "BLOCKED",
                "activate a project venv/conda environment; sudo pip is forbidden",
            )
            continue
        occupied = busy() if action.exclusive else []
        if occupied:
            action.status, action.detail = (
                "BLOCKED",
                f"control processes still running: {occupied}",
            )
            continue
        prefix: list[str] = []
        if action.privileged and getattr(os, "geteuid", lambda: -1)() != 0:
            # Never prompt on a pipe. Interactive sudo reads the terminal directly.
            if not assume_yes and sys.stdin.isatty():
                try:
                    code = subprocess.run(["sudo", "-v"]).returncode
                except OSError:
                    code = 1
            else:
                code, _ = report.run(["sudo", "-n", "true"])
            if code:
                action.status, action.detail = (
                    "BLOCKED",
                    "sudo authorization required; run sudo -v in a terminal, then retry",
                )
                continue
            prefix = ["sudo", "-n"]
        try:
            changed = any(digest(Path(path)) != original for path, original in action.prerequisites.items())
            target = Path(action.target) if action.target else None
            if target is not None:
                changed |= target.is_symlink() or digest(target) != action.before
        except OSError as error:
            action.status, action.detail = "BLOCKED", f"cannot verify configuration: {error}"
            continue
        if changed:
            action.status, action.detail = (
                "BLOCKED", "configuration changed since inspection, or destination is a symlink; inspect again",
            )
            continue
        if target is not None:
            if not _allowed_destination(target, action, capabilities):
                action.status, action.detail = "BLOCKED", "destination outside the repair allowlist"
                continue
            backup = None
            mode = 0o644
            try:
                if target.exists():
                    mode = stat.S_IMODE(target.stat().st_mode) & 0o777
                    backup = report.path / (target.name + ".before")
                    backup.write_bytes(target.read_bytes())
                    backup.chmod(0o600)
                if not action.privileged:
                    target.parent.mkdir(parents=True, exist_ok=True)
                staged = report.path / (target.name + ".new")
                staged.write_text(action.content, encoding="utf-8")
                staged.chmod(0o600)
            except OSError as error:
                action.status, action.detail = "FAIL", str(error)
                continue
            try:
                code, output = _publish(report, prefix, staged, target, expected=action.before, mode=mode)
            except (ConcurrentEdit, OSError) as error:
                action.status, action.detail = "BLOCKED", str(error)
                continue
            published = not code
            if published and action.validate_argv:
                code, output = run_command(prefix + action.validate_argv, timeout=15)
            if not code and action.verify_argv:
                code, output = run_command(prefix + action.verify_argv, timeout=15)
                if not code and action.verify_line not in output.splitlines():
                    code, output = 1, f"effective config did not contain {action.verify_line!r}; review Include order"
            if published and code:
                try:
                    staged_hash = digest(staged)
                    if target.is_symlink() or digest(target) != staged_hash:
                        raise ConcurrentEdit("configuration changed during validation; rollback refused")
                    rollback_code, rollback_output = (
                        _publish(report, prefix, backup, target, expected=staged_hash, mode=mode)
                        if backup else report.run(prefix + ["rm", "-f", "--", str(target)])
                    )
                    output += ("\nrollback failed: " + rollback_output if rollback_code
                               else "\nprevious configuration restored")
                except (ConcurrentEdit, OSError) as error:
                    output += "\n" + str(error)
        else:
            if action.python and action.argv[:3] != [sys.executable, "-m", "pip"]:
                action.status, action.detail = (
                    "BLOCKED",
                    "Python repair must use the running interpreter",
                )
                continue
            code, output = run_command(
                prefix + action.argv, timeout=1800 if action.python else 300
            )
        for argv in action.after_argv:
            if code:
                break
            code, output = run_command(prefix + argv, timeout=30)
        if code:
            action.status, action.detail = "FAIL", redact(output[-2000:])
        else:
            action.status = "PENDING_ACTION" if action.pending else "PASS"
            action.detail = (
                action.pending
                or "completed; see post-repair inspection for verification"
            )


def payload(
    findings: list[Finding], actions: list[Action], report: Report, **extra
) -> dict:
    return {
        "schema_version": 1,
        "report_directory": str(report.path),
        "checks": [asdict(item) for item in findings],
        "repairs": [asdict(item) for item in actions],
        **extra,
    }


def exit_status(findings: list[Finding], actions: list[Action]) -> int:
    unresolved = {"FAIL", "BLOCKED", "PENDING_ACTION"}
    return (DOCTOR_UNRESOLVED_EXIT_CODE if any(item.status in unresolved for item in [*findings, *actions])
            else SUCCESS_EXIT_CODE)
