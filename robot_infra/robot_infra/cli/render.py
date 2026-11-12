"""Terminal colour for status text, dependency-free.

Colour is decided per output stream: only a terminal gets escape codes, and ``NO_COLOR`` (any value,
https://no-color.org) turns it off. JSON and anything written to a file or pipe stay plain. A styled
string with its escape codes removed is exactly the plain string, so alignment never changes.
"""
from __future__ import annotations

import os
import re
import sys
from typing import Mapping

BOLD = "1"
RED = "31"
GREEN = "32"
YELLOW = "33"
CYAN = "36"
GREY = "90"
BOLD_RED = "1;31"
BOLD_GREEN = "1;32"
BOLD_MAGENTA = "1;35"

STATUS_STYLES = {
    "FAIL": RED,
    "BLOCKED": BOLD_MAGENTA,
    "WARN": YELLOW,
    "PENDING_ACTION": YELLOW,
    "PASS": GREEN,
    "OK": GREEN,
    "SKIPPED": GREY,
    "SKIP": GREY,
}

_ANSI_SGR = re.compile(r"\x1b\[[0-9;]*m")


def supports_color(stream, environ: Mapping[str, str] | None = None, platform: str | None = None) -> bool:
    """True only for a terminal stream, without ``NO_COLOR``, on a terminal that renders SGR codes."""
    environ = os.environ if environ is None else environ
    platform = sys.platform if platform is None else platform
    if "NO_COLOR" in environ or environ.get("TERM") == "dumb" or platform == "win32":
        return False
    isatty = getattr(stream, "isatty", None)
    if isatty is None:
        return False
    try:
        return bool(isatty())
    except (ValueError, OSError):  # closed or detached stream
        return False


def paint(text: str, style: str | None, *, enabled: bool = True) -> str:
    """Wrap ``text`` in an SGR style; unchanged when disabled, unstyled or empty."""
    if not enabled or not style or not text:
        return text
    return f"\x1b[{style}m{text}\x1b[0m"


def status_style(status: str) -> str | None:
    return STATUS_STYLES.get(status)


def pad_then_paint(status: str, width: int, *, enabled: bool = True) -> str:
    """Left-align ``status`` to ``width`` first, then style the padded field, keeping columns aligned."""
    return paint(f"{status:{width}}", status_style(status), enabled=enabled)


def strip_ansi(text: str) -> str:
    return _ANSI_SGR.sub("", text)
