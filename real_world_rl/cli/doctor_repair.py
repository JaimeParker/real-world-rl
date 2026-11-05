"""Explicit, capability-aware repair plans and private audit reports."""

from __future__ import annotations

import hashlib
import json
import os
import re
import shlex
import subprocess
import sys
import uuid
from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
from pathlib import Path

from .doctor_platform import Capabilities


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
    prefix = Path(sys.prefix)
    return (prefix / "conda-meta").is_dir() and prefix.parent.name == "envs"


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
) -> None:
    capabilities = capabilities or Capabilities.detect()
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
        if action.target:
            target = Path(action.target)
            if target.is_symlink() or digest(target) != action.before:
                action.status, action.detail = (
                    "BLOCKED",
                    "configuration changed since inspection, or destination is a symlink; inspect again",
                )
                continue
            # This CLI owns only its dedicated configuration files.
            if not (
                str(target).startswith("/etc/udev/rules.d/99-rwrl-")
                or str(target) == "/etc/security/limits.d/99-rwrl-realtime.conf"
            ):
                action.status, action.detail = (
                    "BLOCKED",
                    "destination outside the repair allowlist",
                )
                continue
            if target.exists():
                try:
                    backup = report.path / (target.name + ".before")
                    backup.write_bytes(target.read_bytes())
                    backup.chmod(0o600)
                except OSError as error:
                    action.status, action.detail = "FAIL", str(error)
                    continue
            staged = report.path / (target.name + ".new")
            staged.write_text(action.content, encoding="utf-8")
            staged.chmod(0o600)
            # install writes a temporary file beside the destination; mv publishes atomically.
            temporary = str(target) + ".rwrl-" + report.path.name
            code, output = report.run(
                prefix + ["install", "-m", "0644", "--", str(staged), temporary]
            )
            if not code:
                code, output = report.run(
                    prefix + ["mv", "-f", "--", temporary, str(target)]
                )
        else:
            if action.python and action.argv[:3] != [sys.executable, "-m", "pip"]:
                action.status, action.detail = (
                    "BLOCKED",
                    "Python repair must use the running interpreter",
                )
                continue
            code, output = report.run(
                prefix + action.argv, timeout=1800 if action.python else 300
            )
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
    return int(any(item.status in unresolved for item in [*findings, *actions]))
