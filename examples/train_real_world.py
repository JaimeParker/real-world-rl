"""Real-robot RL entry point.

Algorithm is selected via a subcommand, mirroring ``examples/train_online.py``
-- see ``real_world_rl/training/entrypoints/<method>.py`` for each registered
training method (``serl``, ``hil_serl``, ``residual_hil_serl``, ``silri``, ``real_world_bc``).

Run as two separate processes (optionally on two separate machines):

    # On the GPU training machine:
    python examples/train_real_world.py serl --role learner --sync_host 0.0.0.0 \\
        --sync_port 6000 --env_id <task> --real-robot.remote robot-pc:50051

    # On the robot control machine:
    python examples/train_real_world.py serl --role actor --sync_host <learner-ip> \\
        --sync_port 6000 --env_id <task> --real-robot.remote localhost:50051

Both processes use the same real_robot configuration so their observation/action
spaces match. The station YAML selects a serl_http (SERL bridge) or franky arm.
"""
from __future__ import annotations

from real_world_rl.training.__main__ import main

if __name__ == "__main__":
    main()
