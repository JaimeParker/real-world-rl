"""Terminal colour policy and the plain-text invariant of the shared status renderer."""
from __future__ import annotations

import io

import pytest

from real_world_rl.cli.render import (
    STATUS_STYLES,
    pad_then_paint,
    paint,
    status_style,
    strip_ansi,
    supports_color,
)


class Terminal(io.StringIO):
    def isatty(self):
        return True


@pytest.mark.parametrize(
    "terminal,environ,expected",
    [
        (True, {"TERM": "xterm-256color"}, True),
        (True, {}, True),
        (False, {"TERM": "xterm-256color"}, False),
        (True, {"TERM": "xterm-256color", "NO_COLOR": "1"}, False),
        # NO_COLOR disables colour whatever its value, the empty string included.
        (True, {"TERM": "xterm-256color", "NO_COLOR": ""}, False),
        (False, {"NO_COLOR": "1"}, False),
        (True, {"TERM": "dumb"}, False),
    ],
)
def test_colour_needs_a_terminal_without_no_color(terminal, environ, expected):
    stream = Terminal() if terminal else io.StringIO()
    assert supports_color(stream, environ, platform="linux") is expected


def test_windows_console_stays_plain():
    assert supports_color(Terminal(), {"TERM": "xterm"}, platform="win32") is False


def test_streams_that_cannot_answer_isatty_stay_plain():
    closed = io.StringIO()
    closed.close()
    assert supports_color(closed, {}, platform="linux") is False
    assert supports_color(object(), {}, platform="linux") is False


def test_default_environment_is_the_process_environment(monkeypatch):
    monkeypatch.setenv("TERM", "xterm")
    monkeypatch.setenv("NO_COLOR", "1")
    assert supports_color(Terminal(), platform="linux") is False
    monkeypatch.delenv("NO_COLOR")
    assert supports_color(Terminal(), platform="linux") is True


@pytest.mark.parametrize(
    "status,code",
    [
        ("FAIL", "31"),
        ("BLOCKED", "1;35"),
        ("WARN", "33"),
        ("PENDING_ACTION", "33"),
        ("PASS", "32"),
        ("OK", "32"),
        ("SKIPPED", "90"),
        ("SKIP", "90"),
    ],
)
def test_each_status_has_its_style(status, code):
    assert status_style(status) == code
    assert pad_then_paint(status, 14) == f"\x1b[{code}m{status:14}\x1b[0m"


def test_unknown_status_and_disabled_colour_are_plain():
    assert status_style("PLAN") is None
    assert pad_then_paint("PLAN", 14) == f"{'PLAN':14}"
    assert pad_then_paint("FAIL", 14, enabled=False) == f"{'FAIL':14}"
    assert paint("text", "1", enabled=False) == "text"
    assert paint("text", None) == "text"
    assert paint("", "1") == ""


@pytest.mark.parametrize("status", [*STATUS_STYLES, "PLAN", "PLANNED"])
@pytest.mark.parametrize("enabled", [True, False])
def test_stripping_escape_codes_restores_the_plain_text(status, enabled):
    assert strip_ansi(pad_then_paint(status, 14, enabled=enabled)) == f"{status:14}"
    assert strip_ansi(paint(f"Result: {status}", "1;31", enabled=enabled)) == f"Result: {status}"


def test_columns_line_up_the_same_with_and_without_colour():
    def table(enabled):
        return [f"{pad_then_paint(status, 14, enabled=enabled)} check.{status.lower()}: evidence"
                for status in STATUS_STYLES]

    plain, coloured = table(False), table(True)
    assert coloured != plain
    assert [strip_ansi(line) for line in coloured] == plain
    assert {strip_ansi(line).index("check.") for line in coloured} == {15}
