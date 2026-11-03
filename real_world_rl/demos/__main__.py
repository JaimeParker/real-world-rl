"""``python -m real_world_rl.demos ...`` (console script ``real-world-rl-record``).

Records teleoperated demos on the real robot; see ``real_world_rl/demos/record.py``
and ``docs/recording-demos.md``.
"""
from __future__ import annotations

from typing import Optional, Sequence

from real_world_rl.demos.record import parse_record_args, run_recorder


def main(argv: Optional[Sequence[str]] = None) -> None:
    run_recorder(parse_record_args(argv))


if __name__ == "__main__":
    main()
