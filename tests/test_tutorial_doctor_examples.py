"""The doctor outputs shown in the tutorial match a real ``--mock`` run, line for line.

A ```text block whose fence follows the line ``<!-- doctor-example: mock -->`` is the expected
output of the ``rwrl ... doctor ... --mock`` command in the nearest earlier ```bash block of the
same page. The command runs in-process with colour off, from an empty directory with no station
or pre-0.1.0 configuration, so only the station the command names is checked. Three values
depend on the machine and are normalised on both sides before comparing: the ``host.platform``
and ``host.python`` messages and the ``Report:`` directory. The device probe of a station runs
its fake backends in a subprocess. CPU only, no network, no hardware.
"""
from __future__ import annotations

import os
import re
import shlex
from dataclasses import dataclass
from pathlib import Path

import pytest

from real_world_rl.cli import main

REPO_ROOT = Path(__file__).resolve().parents[1]
TUTORIAL_DIR = REPO_ROOT / "docs" / "tutorial"
EXAMPLE_MARKER = "<!-- doctor-example: mock -->"
# The pages whose doctor output the tutorial teaches; each must keep at least one example.
PAGES_WITH_EXAMPLES = {"01-install.md", "03-franka-bringup.md"}

_MACHINE_DEPENDENT = [
    (re.compile(r"^(\S+ +host\.platform: ).*$", re.M), r"\1<platform>"),
    (re.compile(r"^(\S+ +host\.python: ).*$", re.M), r"\1<python>"),
    (re.compile(r"^Report: .*$", re.M), "Report: <dir>"),
]


@dataclass(frozen=True)
class DoctorExample:
    page: str
    line: int
    argv: tuple[str, ...]
    expected: str

    @property
    def id(self) -> str:
        return f"{self.page}:{self.line}"


def _fenced_block(lines: list[str], start: int) -> list[str]:
    body = []
    index = start + 1
    while index < len(lines) and lines[index].strip() != "```":
        body.append(lines[index])
        index += 1
    if index == len(lines):
        raise ValueError(f"unterminated block at line {start + 1}")
    return body


def _doctor_command(lines: list[str], before: int, page: str) -> tuple[str, ...]:
    """The ``rwrl ... doctor ... --mock`` command of the nearest ```bash block above ``before``."""
    for index in range(before - 1, -1, -1):
        if lines[index].strip() != "```bash":
            continue
        joined = re.sub(r"\\\n", " ", "\n".join(_fenced_block(lines, index)))
        commands = [
            tuple(shlex.split(line, comments=True)) for line in joined.splitlines()
            if line.strip() and not line.strip().startswith("#")
        ]
        commands = [argv for argv in commands if argv[:1] == ("rwrl",) and "doctor" in argv and "--mock" in argv]
        if len(commands) != 1:
            raise ValueError(f"{page}:{index + 1}: expected one 'rwrl ... doctor ... --mock' command, found {commands}")
        return commands[0]
    raise ValueError(f"{page}:{before + 1}: no ```bash block before the doctor example")


def doctor_examples(page: Path) -> list[DoctorExample]:
    lines = page.read_text(encoding="utf-8").splitlines()
    examples = []
    for index, line in enumerate(lines):
        if line.strip() != EXAMPLE_MARKER:
            continue
        fence = index + 1
        if fence >= len(lines) or lines[fence].strip() != "```text":
            raise ValueError(f"{page.name}:{index + 1}: {EXAMPLE_MARKER} must directly precede a ```text block")
        expected = "\n".join(_fenced_block(lines, fence))
        examples.append(DoctorExample(page.name, fence + 1, _doctor_command(lines, index, page.name), expected))
    return examples


EXAMPLES = [example for page in sorted(TUTORIAL_DIR.glob("*.md")) for example in doctor_examples(page)]


def normalised(text: str) -> str:
    text = text.rstrip("\n")
    for pattern, replacement in _MACHINE_DEPENDENT:
        text = pattern.sub(replacement, text)
    return text


def repository_paths(argv: tuple[str, ...]) -> list[str]:
    """``argv`` after ``rwrl`` with repository-relative paths made absolute (the run's cwd is empty)."""
    return [str(REPO_ROOT / token) if (REPO_ROOT / token).is_file() else token for token in argv[1:]]


@pytest.fixture
def isolated_host(tmp_path, monkeypatch, doctor_report_dir):
    """No station, legacy configuration, device claim or colour from the machine running the test."""
    # The device probe subprocess must import the same sources after the chdir below.
    entries = [str(Path(entry).resolve()) for entry in os.environ.get("PYTHONPATH", "").split(os.pathsep) if entry]
    if entries:
        monkeypatch.setenv("PYTHONPATH", os.pathsep.join(entries))
    for name in ("HOME", "XDG_CONFIG_HOME", "XDG_CACHE_HOME", "XDG_STATE_HOME"):
        monkeypatch.setenv(name, str(tmp_path / name.lower()))
    monkeypatch.setenv("NO_COLOR", "1")
    monkeypatch.chdir(tmp_path)
    # The interpreter running the tests need not be a project environment; the example shows one.
    monkeypatch.setattr("real_world_rl.doctor.host.project_environment", lambda: True)


def test_the_tutorial_shows_doctor_examples():
    assert PAGES_WITH_EXAMPLES <= {example.page for example in EXAMPLES}, (
        f"every page of {sorted(PAGES_WITH_EXAMPLES)} needs a {EXAMPLE_MARKER} block"
    )


def test_normalisation_keeps_everything_but_the_machine_values():
    text = ("PASS           host.platform: Linux x\nWARN           host.python: /usr/bin/python3\n"
            "PASS           arm.handshake: robot connected\nReport: /tmp/r/1\n")
    assert normalised(text) == ("PASS           host.platform: <platform>\nWARN           host.python: <python>\n"
                                "PASS           arm.handshake: robot connected\nReport: <dir>")


@pytest.mark.parametrize("example", EXAMPLES, ids=[example.id for example in EXAMPLES])
def test_doctor_example_matches_a_mock_run(example: DoctorExample, isolated_host, capsys):
    code = main(repository_paths(example.argv))
    captured = capsys.readouterr()
    expected_code = 0 if example.expected.rstrip().endswith("Result: no blocking findings; review SKIPPED/WARN coverage") else 1
    assert code == expected_code, captured.err
    assert "\x1b[" not in captured.out
    assert normalised(captured.out) == normalised(example.expected), (
        f"{example.page}:{example.line}: update the example to the output of {shlex.join(example.argv)}"
    )
