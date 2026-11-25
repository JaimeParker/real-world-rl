"""Tab completion from the ``rwrl`` parser: commands, actions, options, choices and paths."""
from __future__ import annotations

import argparse
import re

from prompt_toolkit.completion import Completer, Completion, PathCompleter
from prompt_toolkit.document import Document

from .session import TASK_CONFIG_COMMANDS

BUILTINS = {
    "switch": "change device", "task": "change task", "robot": "change robot", "menu": "pick a command",
    "info": "show the selected files", "default": "make this selection the plain rwrl default",
    "daemon": "local daemon: status, start, stop, log, run", "help": "list commands", "exit": "leave the shell",
}
# The parser lists these without a help text.
PRIMITIVE_HELP = {
    "status": "connection, arm health and devices", "q": "joint positions", "tcp": "TCP pose",
    "stop": "stop the arm", "recover": "recover the arm from an error",
}
DAEMON_ACTIONS = {"status": "daemon state and identity", "start": "start in the background",
                  "stop": "stop a daemon started here", "log": "show its log", "run": "run in this terminal"}


def subparsers_of(parser):
    return next((action for action in parser._actions if isinstance(action, argparse._SubParsersAction)), None)


def command_help(parser) -> dict[str, str]:
    """Command name -> help, in registration order."""
    action = subparsers_of(parser)
    helps = {choice.dest: choice.help for choice in action._choices_actions}
    return {name: helps.get(name) or PRIMITIVE_HELP.get(name, "") for name in action.choices}


class ShellCompleter(Completer):
    def __init__(self, session, parser) -> None:
        self._session = session
        self._parser = parser
        self._help = command_help(parser)
        self._paths = PathCompleter(expanduser=True, get_paths=lambda: [str(session.cwd)])

    def get_completions(self, document, event):
        words = document.text_before_cursor.split()
        current = "" if not words or document.text_before_cursor[-1:].isspace() else words.pop()
        if words[:1] == ["rwrl"]:
            words = words[1:]
        if not words:
            return self._match(current, {**BUILTINS, **{name: text for name, text in self._help.items()
                                                        if name not in ("use", "shell")}})
        return self._arguments(words, current, event)

    def _match(self, current, options):
        for name, meta in options.items():
            if name.startswith(current):
                yield Completion(name, start_position=-len(current), display_meta=re.sub(r"\s+", " ", meta or ""))

    def _path(self, current, event):
        yield from self._paths.get_completions(Document(current), event)

    def _arguments(self, words, current, event):
        head, profile = words[0], self._session.profile
        if head == "switch":
            return self._match(current, {item.name: "" for item in self._session.profiles()}) if len(words) == 1 else iter(())
        if head == "task":
            return self._match(current, {name: "" for name in profile.tasks} if profile and len(words) == 1 else {})
        if head == "robot":
            names = {"all": "no single robot", **{name: "" for name in profile.robots}} if profile else {}
            return self._match(current, names if len(words) == 1 else {})
        if head == "daemon" and len(words) == 1 and not current.startswith("-"):
            return self._match(current, DAEMON_ACTIONS)
        if head == "help":
            return self._match(current, self._help if len(words) == 1 else {})
        return self._parsed(words, current, event)

    def _parsed(self, words, current, event):
        parser, previous = self._parser, words[-1]
        for word in words:
            action = subparsers_of(parser)
            if action is not None and word in action.choices:
                parser = action.choices[word]
        if previous in ("--config", "--preset", "--station"):
            return self._path(current, event)
        option = parser._option_string_actions.get(previous)
        if option is not None and option.nargs != 0:
            if option.choices:
                return self._match(current, {str(choice): "" for choice in option.choices})
            return self._path(current, event)
        if current.startswith("-"):
            options = {name: action.help if action.help != argparse.SUPPRESS else ""
                       for name, action in parser._option_string_actions.items() if name.startswith("--")}
            if words[0] in TASK_CONFIG_COMMANDS:
                options["--config"] = "env preset yaml"
            return self._match(current, options)
        action = subparsers_of(parser)
        if action is not None:
            return self._match(current, {name: "" for name in action.choices})
        positional = next((item for item in parser._actions if not item.option_strings and item.choices), None)
        if positional is not None:
            return self._match(current, {str(choice): "" for choice in positional.choices})
        return self._path(current, event)
