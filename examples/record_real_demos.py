"""Record teleoperated demos on the real robot for hil_serl / residual / SiLRI / BC.

    python examples/record_real_demos.py --config hil.yaml --teleop-device leader_arm \\
        --robot-teleop-action-mode joint_abs --teleop-event-device keyboard \\
        --output-path demos/task_20.pkl --num-successes 20

Takes the env, teleop and observation flags (and the ``--config`` preset) of the HIL training
entrypoints; pass the ones of the run that will load the demos (``--demo-dataset-paths
demos/task_20.pkl``). Logic lives in ``real_world_rl/demos/record.py``; see
``docs/recording-demos.md``.
"""
from __future__ import annotations

from real_world_rl.demos.__main__ import main

if __name__ == "__main__":
    main()
