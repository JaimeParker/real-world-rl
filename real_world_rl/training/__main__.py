"""``python -m real_world_rl.training <method> ...`` (console script ``real-world-rl-train``).

Runs the real-world training registry's CLI: one subcommand per method in
``real_world_rl/training/entrypoints/``.
"""
from __future__ import annotations

from typing import Sequence

from real_world_rl.training import registry


def main(argv: Sequence[str] | None = None) -> None:
    registry.run_cli(argv)


if __name__ == "__main__":
    main()
