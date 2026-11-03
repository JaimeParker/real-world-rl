"""``python -m rl_garden_real_world.demos ...`` (console script ``rlgarden-real-record``).

Records teleoperated demos on the real robot; see ``rl_garden_real_world/demos/record.py``
and ``docs/recording-demos.md``.
"""
from __future__ import annotations

from typing import Optional, Sequence

from rl_garden_real_world.demos.record import parse_record_args, run_recorder


def main(argv: Optional[Sequence[str]] = None) -> None:
    run_recorder(parse_record_args(argv))


if __name__ == "__main__":
    main()
