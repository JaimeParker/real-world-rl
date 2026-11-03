"""``python -m real_world_rl.training <method> ...`` (console script ``real-world-rl-train``).

Runs the real-world training registry's CLI: one subcommand per method in
``real_world_rl/training/entrypoints/``.
"""
from __future__ import annotations

from real_world_rl.training import registry


def main() -> None:
    registry.run_cli()


if __name__ == "__main__":
    main()
