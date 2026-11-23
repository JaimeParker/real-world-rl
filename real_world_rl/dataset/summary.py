"""Plain-text overview of a demo dataset: one line per episode and a totals footer.

The table is what ``rwrl dataset list`` prints. Columns: episode index, ``kept`` or
``discarded``, the discard reason, success (``yes``/``no``/``-`` when unknown), steps, the
share of steps the operator drove, duration in seconds and the number of review notes.
Only plain Python is used, so listing 1000 episodes is instant.
"""
from __future__ import annotations

from typing import Any

_COLUMNS = (("episode", 7), ("status", 9), ("reason", 20), ("success", 7), ("steps", 6),
            ("intervened", 10), ("duration_s", 10), ("notes", 5))


def _success_text(success: Any) -> str:
    if success is None:
        return "-"
    return "yes" if success else "no"


def episode_rows(dataset: Any) -> list[dict]:
    """One dict per episode (sorted by index) with the values the table prints."""
    rows = []
    for info in dataset.episodes():
        meta = info.meta
        steps = int(meta.get("steps", 0))
        intervened = int(meta.get("intervened_steps", 0))
        rows.append({
            "index": info.index,
            "status": meta.get("status"),
            "discard_reason": meta.get("discard_reason"),
            "success": meta.get("success"),
            "steps": steps,
            "intervened_steps": intervened,
            "intervened_pct": 100.0 * intervened / steps if steps else 0.0,
            "duration_s": float(meta.get("duration_s") or 0.0),
            "notes": len(meta.get("notes") or []),
        })
    return rows


def _format_row(row: dict) -> str:
    cells = (f"{row['index']:06d}", str(row["status"]), row["discard_reason"] or "-",
             _success_text(row["success"]), str(row["steps"]), f"{row['intervened_pct']:.0f}%",
             f"{row['duration_s']:.1f}", str(row["notes"]))
    return "  ".join(cell.ljust(width) for cell, (_, width) in zip(cells, _COLUMNS)).rstrip()


def format_episode_table(dataset: Any) -> str:
    """The episode table plus a footer: episodes, kept, discarded and kept transitions.

    Kept transitions is the number of intervened steps summed over kept episodes, i.e. the
    rows training will see.
    """
    rows = episode_rows(dataset)
    header = "  ".join(name.ljust(width) for name, width in _COLUMNS).rstrip()
    lines = [header, "-" * len(header)] + [_format_row(row) for row in rows]
    kept = [row for row in rows if row["status"] == "kept"]
    transitions = sum(row["intervened_steps"] for row in kept)
    lines += ["-" * len(header),
              f"episodes: {len(rows)}  kept: {len(kept)}  discarded: {len(rows) - len(kept)}  "
              f"kept transitions: {transitions}"]
    return "\n".join(lines)
