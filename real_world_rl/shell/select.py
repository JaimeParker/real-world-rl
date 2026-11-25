"""An inline list: arrow keys, typing filters, Enter selects, Esc or Ctrl-C cancels."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

from prompt_toolkit.application import Application
from prompt_toolkit.key_binding import KeyBindings
from prompt_toolkit.keys import Keys
from prompt_toolkit.layout import Layout, Window
from prompt_toolkit.layout.controls import FormattedTextControl
from prompt_toolkit.layout.dimension import Dimension

from .view import symbol

VISIBLE_ROWS = 14


@dataclass
class Item:
    value: Any
    label: str
    detail: str = ""
    group: str = field(default="", kw_only=True)
    disabled: bool = field(default=False, kw_only=True)


class Selector:
    def __init__(self, title: str, items: list[Item], *, preselect=None) -> None:
        self.title = title
        self.items = items
        self.filter = ""
        self.index = next((i for i, item in enumerate(items) if item.value == preselect and not item.disabled),
                          self._step(-1, 1, items))

    @staticmethod
    def _step(index: int, direction: int, items) -> int:
        for offset in range(1, len(items) + 1):
            candidate = (index + direction * offset) % len(items)
            if not items[candidate].disabled:
                return candidate
        return 0

    @property
    def matches(self) -> list[Item]:
        wanted = self.filter.lower()
        return [item for item in self.items if wanted in f"{item.group} {item.label} {item.detail}".lower()]

    def move(self, direction: int) -> None:
        if self.matches:
            self.index = self._step(self.index, direction, self.matches)

    def set_filter(self, text: str) -> None:
        self.filter = text
        self.index = self._step(-1, 1, self.matches) if self.matches else 0

    def current(self) -> Item | None:
        matches = self.matches
        item = matches[self.index] if self.index < len(matches) else None
        return None if item is None or item.disabled else item

    def fragments(self):
        matches = self.matches
        width = max((len(item.label) for item in self.items), default=0)
        group_width = max((len(item.group) for item in self.items), default=0)
        first = max(0, min(self.index - VISIBLE_ROWS // 2, len(matches) - VISIBLE_ROWS))
        lines = [("class:mark", "? "), ("class:title", self.title),
                 ("class:filter", f"  {self.filter}\n") if self.filter else ("class:detail", "  type to filter\n")]
        for i, item in enumerate(matches[first:first + VISIBLE_ROWS], start=first):
            chosen = i == self.index and not item.disabled
            mark = symbol("arrow") if chosen else symbol("cross") if item.disabled else " "
            lines.append(("class:error" if item.disabled else "class:pick", f" {mark} "))
            if group_width:
                lines.append(("class:group", f"{item.group:{group_width}}  "))
            lines.append(("class:disabled" if item.disabled else "class:pick" if chosen else "class:label",
                          f"{item.label:{width}}"))
            lines.append(("class:error" if item.disabled else "class:detail", f"  {item.detail}\n"))
        if not matches:
            lines.append(("class:warn", "   no match\n"))
        more = f"{self.index + 1}/{len(matches)}  " if len(matches) > VISIBLE_ROWS else ""
        lines.append(("class:detail", f"   {more}up/down move, Enter selects, Esc cancels"))
        return lines

    def run(self, *, style=None, **app_options):
        """The selected value, or None when cancelled."""
        keys = KeyBindings()
        keys.add("up")(lambda event: self.move(-1))
        keys.add("down")(lambda event: self.move(1))
        keys.add("backspace")(lambda event: self.set_filter(self.filter[:-1]))
        keys.add("escape", eager=True)(lambda event: event.app.exit())
        keys.add("c-c")(lambda event: event.app.exit())
        keys.add("c-d")(lambda event: event.app.exit())

        @keys.add("enter")
        def accept(event):
            item = self.current()
            if item is not None:
                event.app.exit(result=item.value)

        @keys.add(Keys.Any)
        def typed(event):
            if event.data.isprintable():
                self.set_filter(self.filter + event.data)

        window = Window(FormattedTextControl(self.fragments, focusable=True, show_cursor=False),
                        height=Dimension(max=VISIBLE_ROWS + 4), always_hide_cursor=True, wrap_lines=False)
        app = Application(layout=Layout(window), key_bindings=keys, style=style, erase_when_done=True, **app_options)
        app.ttimeoutlen = 0.05  # a lone Esc reacts at once, arrow keys still parse
        return app.run()


def choose(title: str, items: list[Item], *, preselect=None, **options):
    return Selector(title, items, preselect=preselect).run(**options) if items else None
