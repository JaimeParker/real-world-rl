"""``python -m rl_garden_real_world.training <method> ...`` (console script ``rlgarden-real-train``).

Runs the real-world training registry's CLI: one subcommand per method in
``rl_garden_real_world/training/entrypoints/``.
"""
from __future__ import annotations

from rl_garden_real_world.training import registry


def main() -> None:
    registry.run_cli()


if __name__ == "__main__":
    main()
