"""Record teleoperated demos on the real robot for hil_serl / residual / SiLRI / BC.

    python examples/record_real_demos.py --config hil.yaml --teleop-device leader_arm \\
        --robot-teleop-action-mode joint_abs --teleop-event-device keyboard \\
        --output-path demos/task_20.pkl --num-successes 20

Takes the env, teleop and observation flags (and the ``--config`` preset) of the HIL training
entrypoints; pass the ones of the run that will load the demos (``--demo-dataset-paths
demos/task_20.pkl``). Logic lives in ``rl_garden_real_world/demos/record.py``; see
``docs/recording-demos.md``.
"""
from __future__ import annotations

from typing import Optional, Sequence

from rl_garden_real_world.demos.record import parse_record_args, run_recorder


def main(argv: Optional[Sequence[str]] = None) -> None:
    run_recorder(parse_record_args(argv))


if __name__ == "__main__":
    main()
