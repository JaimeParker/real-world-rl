"""The doctor outputs shown in the tutorial match a real ``--mock`` run, line for line.

Marked reStructuredText literalincludes contain complete expected output. The nearest earlier
bash code block or literalinclude supplies the mock command.
The command runs in-process with colour off, from an empty directory with no station
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
EXAMPLE_MARKER = ".. doctor-example: mock"
# The pages whose doctor output the tutorial teaches; each must keep at least one example.
PAGES_WITH_EXAMPLES = {f"{language}tutorials/{page}.rst"
                       for language in ("", "en/") for page in ("install", "station")}

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


def _block(lines: list[str], start: int) -> list[str]:
    body = []
    for line in lines[start + 1:]:
        if line and not line.startswith("   "):
            break
        body.append(line[3:] if line.startswith("   ") else line)
    return body


def _included(page: Path, lines: list[str], index: int) -> str:
    path = page.parent / lines[index].split("::", 1)[1].strip()
    text = path.read_text(encoding="utf-8")
    options = dict(re.findall(r"^:([a-z-]+): (.*)$", "\n".join(_block(lines, index)), re.M))
    if "start-after" in options:
        text = text.split(options["start-after"], 1)[1].lstrip("\n")
    if "end-before" in options:
        text = text.split(options["end-before"], 1)[0]
    if "lines" in options:
        first, last = map(int, options["lines"].split("-"))
        text = "\n".join(text.splitlines()[first - 1:last])
    return text


def _doctor_command(page: Path, lines: list[str], before: int) -> tuple[str, ...]:
    for index in range(before - 1, -1, -1):
        if lines[index].startswith(".. literalinclude::"):
            if ":language: bash" not in _block(lines, index):
                continue
            text = _included(page, lines, index)
        elif lines[index] == ".. code-block:: bash":
            text = "\n".join(_block(lines, index))
        else:
            continue
        joined = re.sub(r"\\\n", " ", text)
        commands = [tuple(shlex.split(line, comments=True)) for line in joined.splitlines()]
        commands = [argv for argv in commands
                    if argv[:1] == ("rwrl",) and "doctor" in argv and "--mock" in argv]
        if commands:
            assert len(commands) == 1, (page, commands)
            return commands[0]
    raise ValueError(f"{page}:{before + 1}: no mock doctor command before example")


def doctor_examples(page: Path) -> list[DoctorExample]:
    lines = page.read_text(encoding="utf-8").splitlines()
    examples = []
    for index, line in enumerate(lines):
        if line != EXAMPLE_MARKER:
            continue
        include = index + 1
        assert lines[include].startswith(".. literalinclude::"), (page, include)
        examples.append(DoctorExample(
            page.relative_to(REPO_ROOT / "docs").as_posix(), include + 1,
            _doctor_command(page, lines, index), _included(page, lines, include),
        ))
    return examples


EXAMPLES = [example for name in sorted(PAGES_WITH_EXAMPLES)
            for example in doctor_examples(REPO_ROOT / "docs" / name)]


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
