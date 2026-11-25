"""Colours and layout of ``rwrl shell``: rich for printed output, one matching prompt_toolkit style
for the prompt, toolbar, lists and completion menu.

Cyan marks what is selected, grey is secondary, and green/yellow/red only ever mean a state. Only
ANSI colours are used, so the terminal's own theme applies; ``NO_COLOR`` turns them off. Messages
are never parsed as markup: they contain brackets and paths."""
from __future__ import annotations

import sys

from prompt_toolkit.styles import Style
from rich import box
from rich.console import Console
from rich.panel import Panel
from rich.table import Table
from rich.text import Text

STYLE = Style.from_dict({
    "device": "ansicyan bold", "task": "ansicyan", "robot": "ansibrightblack", "arrow": "ansicyan bold",
    "good": "ansigreen", "bad": "ansired bold", "warn": "ansiyellow", "muted": "ansibrightblack",
    "bottom-toolbar": "noreverse bg:ansiblack ansiwhite", "bar.device": "bold ansicyan", "bar.key": "ansibrightblack",
    "mark": "ansigreen bold", "title": "bold", "filter": "ansiyellow", "pick": "ansicyan bold",  # not "selected": prompt_toolkit reverses that one
   
    "label": "bold", "group": "ansimagenta", "detail": "ansibrightblack", "disabled": "ansibrightblack",
    "error": "ansired",
    "completion-menu.completion": "bg:ansiblack ansiwhite",
    "completion-menu.completion.current": "noreverse bg:ansicyan ansiblack bold",
    "completion-menu.meta.completion": "bg:ansiblack ansibrightblack",
    "completion-menu.meta.completion.current": "noreverse bg:ansicyan ansiblack",
    "scrollbar.background": "bg:ansiblack", "scrollbar.button": "bg:ansibrightblack",
})
LOG_STYLES = {"PASS": "green", "FAIL": "bold red", "BLOCKED": "bold magenta", "WARN": "yellow",
              "PENDING_ACTION": "yellow", "SKIPPED": "bright_black", "error:": "bold red"}
SYMBOLS = {"arrow": ("❯", ">"), "dot": ("●", "*"), "cross": ("✗", "x"), "tick": ("✓", "+"), "sep": ("│", "|"),
           "warn": ("▲", "!")}
console = Console(highlight=False, markup=False, emoji=False)


def symbol(name: str) -> str:
    """The symbol, or its ASCII stand-in when the terminal's encoding lacks it."""
    fancy, plain = SYMBOLS[name]
    try:
        fancy.encode(getattr(sys.stdout, "encoding", None) or "ascii")
    except (UnicodeEncodeError, LookupError):
        return plain
    return fancy


def say(text: str, style: str = "") -> None:
    # Long paths are never broken, so they can be copied.
    console.print(Text(text, style=style), soft_wrap=True)


def _marked(mark: str, word: str, text: str, style: str) -> None:
    console.print(Text.assemble((f"{symbol(mark)} {word}", f"bold {style}"), (" " + text) if text else ""),
                  soft_wrap=True)


def error(text: str) -> None:
    _marked("cross", "error:", text, "red")


def warning(text: str) -> None:
    _marked("warn", "warning:", text, "yellow")


def success(text: str) -> None:
    console.print(Text.assemble((symbol("tick") + " ", "bold green"), text), soft_wrap=True)


def ended(text: str) -> None:
    _marked("cross", text, "", "red")


def ask(question: str) -> str:
    return console.input(Text.assemble((question, "bold"), (" [y/N] ", "bold yellow")))


def gap() -> None:
    console.print()


def info(rows) -> None:
    """``(label, value, note, state)`` rows in a panel titled with the first value."""
    table = Table.grid(padding=(0, 2))
    table.add_column(style="bright_black", no_wrap=True)
    table.add_column(overflow="fold")
    title = ""
    for label, value, note, state in rows:
        if label == "device":
            title = value
            continue
        style = {"bad": "bold red", "main": "bold", "muted": "bright_black"}.get(state, "")
        table.add_row(label, Text.assemble((value, style), (f"  {note}", "bright_black") if note else ""))
    console.print(Panel(table, title=Text(title, style="bold cyan"), title_align="left", border_style="bright_black",
                        box=box.ROUNDED, expand=False, padding=(0, 1)))


def command_list(sections) -> None:
    """``(title, {name: help})`` sections as one aligned list."""
    table = Table.grid(padding=(0, 2))
    table.add_column(no_wrap=True, style="cyan")
    table.add_column()
    for title, commands in sections:
        table.add_row(Text(title, style="bold default"), "")
        for name, text in commands.items():
            table.add_row("  " + name, Text(" ".join(text.split()) or "-", style=""))
    console.print(table)


def daemon_row(name: str, address: str, text: str, state: str) -> None:
    style = {"good": "green", "bad": "bold red", "warn": "yellow"}[state]
    console.print(Text.assemble((symbol("dot") + " ", style), (name, "bold"), (f" {address}  ", "bright_black"), (text, style)),
                  soft_wrap=True)


def log(path, text: str) -> None:
    say(str(path), "bright_black")
    for line in text.splitlines():
        word = line.split(" ", 1)[0]
        console.print(Text.assemble((word, LOG_STYLES.get(word, "")), line[len(word):]), soft_wrap=True)
