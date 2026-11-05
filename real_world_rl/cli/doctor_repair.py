"""Explicit, dependency-aware NUC repair plans and private audit reports."""
from __future__ import annotations

from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import re
import shlex
import subprocess
import sys
import uuid


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


def redact(value):
    if isinstance(value, dict):
        return {key: "<redacted>" if re.search(r"token|password|secret|credential", str(key), re.I)
                else redact(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [redact(item) for item in value]
    if not isinstance(value, str):
        return value
    value = re.sub(r"(https?://)[^/\s@]+@", r"\1<redacted>@", value)
    return re.sub(r"(?i)((?:token|password|secret|authorization)[\w-]*\s*[:=]\s*)[^\s,;]+", r"\1<redacted>", value)


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


class Report:
    def __init__(self, root: Path):
        base = Path(os.environ.get("RWRL_DOCTOR_REPORT_DIR", root / "local/doctor"))
        base.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S")
        self.path = base / f"{stamp}-{uuid.uuid4().hex[:8]}"
        self.path.mkdir(mode=0o700)
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

    def run(self, argv: list[str], *, input_text: str | None = None, timeout: int = 300, stdout_only: bool = False):
        # Only developer-built argv is accepted. No shell expansion or stdin passwords.
        with self.log.open("a", encoding="utf-8") as stream:
            stream.write("$ " + shlex.join(redact(argv)) + "\n")
            try:
                result = subprocess.run(argv, input=input_text, capture_output=True, text=True, timeout=timeout)
            except (OSError, subprocess.TimeoutExpired) as error:
                stream.write(redact(str(error)) + "\n")
                return 1, str(error)
            output = result.stdout + result.stderr
            stream.write(redact(output) + f"\nexit={result.returncode}\n")
        return result.returncode, result.stdout if stdout_only and result.returncode == 0 else output


def file_action(check: str, title: str, target: str, content: str, *, pending: str = "", exclusive: bool = True) -> Action:
    return Action("file:" + target, check, title, target=target, content=content,
                  privileged=True, pending=pending, exclusive=exclusive, before=digest(Path(target)))


def execute(actions: list[Action], report: Report, *, allowed: bool, busy, assume_yes: bool,
            sdk_only: bool = False) -> None:
    done: dict[str, Action] = {}
    for action in actions:
        done[action.id] = action
        if not allowed:
            action.status, action.detail = "BLOCKED", "automatic repair only supports Ubuntu 22.04 x86_64 outside containers"
            continue
        if sdk_only and not action.id.startswith("sdk:"):
            action.status, action.detail = "SKIPPED", "--install-sdk selects Franky wheel replacement only"
            continue
        if any(dep not in done or done[dep].status not in ("PASS", "PENDING_ACTION") for dep in action.dependencies):
            action.status, action.detail = "BLOCKED", "a prerequisite repair did not complete"
            continue
        if action.python and not project_environment():
            action.status, action.detail = "BLOCKED", "activate a project venv/conda environment; sudo pip is forbidden"
            continue
        occupied = busy() if action.exclusive else []
        if occupied:
            action.status, action.detail = "BLOCKED", f"control processes still running: {occupied}"
            continue
        prefix: list[str] = []
        if action.privileged:
            # Never prompt on a pipe. Interactive sudo reads the terminal directly.
            if not assume_yes and sys.stdin.isatty():
                try:
                    code = subprocess.run(["sudo", "-v"]).returncode
                except OSError:
                    code = 1
            else:
                code, _ = report.run(["sudo", "-n", "true"])
            if code:
                action.status, action.detail = "BLOCKED", "sudo authorization required; run sudo -v in a terminal, then retry"
                continue
            prefix = ["sudo", "-n"]
        if action.target:
            target = Path(action.target)
            if target.is_symlink() or digest(target) != action.before:
                action.status, action.detail = "BLOCKED", "configuration changed since inspection, or destination is a symlink; inspect again"
                continue
            # This CLI owns only its dedicated configuration files.
            if not (str(target).startswith("/etc/udev/rules.d/99-rwrl-")
                    or str(target) == "/etc/security/limits.d/99-rwrl-realtime.conf"):
                action.status, action.detail = "BLOCKED", "destination outside the repair allowlist"
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
            code, output = report.run(prefix + ["install", "-m", "0644", "--", str(staged), temporary])
            if not code:
                code, output = report.run(prefix + ["mv", "-f", "--", temporary, str(target)])
        else:
            if action.python and action.argv[:3] != [sys.executable, "-m", "pip"]:
                action.status, action.detail = "BLOCKED", "Python repair must use the running interpreter"
                continue
            code, output = report.run(prefix + action.argv, timeout=1800 if action.python else 300)
        if code:
            action.status, action.detail = "FAIL", redact(output[-2000:])
        else:
            action.status = "PENDING_ACTION" if action.pending else "PASS"
            action.detail = action.pending or "completed; see post-repair inspection for verification"


def payload(findings: list[Finding], actions: list[Action], report: Report, **extra) -> dict:
    return {"schema_version": 1, "report_directory": str(report.path),
            "checks": [asdict(item) for item in findings], "repairs": [asdict(item) for item in actions], **extra}


def exit_status(findings: list[Finding], actions: list[Action]) -> int:
    unresolved = {"FAIL", "BLOCKED", "PENDING_ACTION"}
    return int(any(item.status in unresolved for item in [*findings, *actions]))
