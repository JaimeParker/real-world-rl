"""Reward-classifier data collection, porting HIL-SERL's own mechanism
(``3rd_party/hil-serl/examples/record_success_fail.py``): continuously step
the real robot (a human can intervene via ``TeleopInterventionWrapper``),
globally listen for the spacebar via ``pynput`` to mark the *most recently
collected* transition as a success, everything else defaults to failure.
Saves two pickle files under ``--output_dir`` on exit, matching
``rl_garden.models.reward.success.data.SuccessClassifierDataset``'s expected
``classifier_data/*_{success,failure}_*.pkl`` naming and ``{"obs": obs}``
entry shape.

Camera capture is caller-supplied (see
``real_world_rl/envs/franka_real/env.py``'s module docstring -- camera SDK
dependencies are deliberately kept out of the env itself and out of the
``franka_real`` backend registration, which always passes
``camera_capture=None``): the CLI entrypoint below has no camera wired up,
run it as a library (``main(camera_capture=...)``) with a cell-specific
capture callable to actually record images.

With ``--env-backend real_robot`` on the command line the collector runs on a robot_infra robot
instead (``real_world_rl.reward.collect_real_robot``): the HIL actor's env and flags
(``--teleop-device leader_arm --teleop-event-device keyboard``, ``--real-robot.*``, ``--obs.*``,
``--config``), cameras from the robot config, and frames labelled with the ``s``/``f`` events
rather than the spacebar; ``--env-backend real_robot --help`` lists its flags.
"""
from __future__ import annotations

import datetime
import os
import pickle
import sys
from dataclasses import dataclass, field
from typing import Literal, Optional, Sequence

import torch
import tyro

from real_world_rl.common.tree import index_tree


@dataclass
class Args:
    output_dir: str
    successes_needed: int = 200
    exp_name: str = "franka_real"

    bridge_url: str = "http://localhost:5000"
    action_scale_pos: float = 0.02
    action_scale_rot: float = 0.1
    gripper_threshold: float = 0.5
    max_episode_steps: int = 100
    camera_keys: tuple[str, ...] = field(default_factory=tuple)
    camera_height: int = 128
    camera_width: int = 128

    teleop_device: str = "pico"
    spacemouse_mode: Literal["tracker", "serl"] = "tracker"


_ENV_BACKEND_FLAGS = ("--env-backend", "--env_backend")


def selects_real_robot(argv: Sequence[str]) -> bool:
    """Whether the command line names an env backend (``--env-backend real_robot``): ``Args`` has
    no such flag, so it selects the ``real_robot`` collector, whose parser checks the value."""
    return any(token.split("=", 1)[0] in _ENV_BACKEND_FLAGS for token in argv)


def main(camera_capture=None, argv: Optional[Sequence[str]] = None) -> None:
    cli_args = list(sys.argv[1:] if argv is None else argv)
    if selects_real_robot(cli_args):
        if camera_capture is not None:
            raise ValueError(
                "the real_robot collector reads its cameras from the robot config; drop camera_capture"
            )
        from real_world_rl.reward.collect_real_robot import (
            parse_collect_args,
            run_real_robot_collector,
        )

        run_real_robot_collector(parse_collect_args(cli_args))
        return
    args = tyro.cli(Args, args=argv)
    if args.spacemouse_mode == "serl":
        raise ValueError(
            "SpaceMouse serl mode requires the shared env configuration; use "
            "python -m real_world_rl.reward.collect_real_robot --spacemouse-mode serl "
            "--teleop-device spacemouse with a configured real_robot preset"
        )

    from pynput import keyboard

    from real_world_rl.envs.franka_real import FrankaRealEnvConfig, make_franka_real_env
    from real_world_rl.envs.wrappers.intervention import TeleopInterventionWrapper

    success_key = [False]

    def on_press(key):
        if str(key) == "Key.space":
            success_key[0] = True

    listener = keyboard.Listener(on_press=on_press)
    listener.start()

    cfg = FrankaRealEnvConfig(
        bridge_url=args.bridge_url,
        action_scale=(args.action_scale_pos, args.action_scale_rot),
        gripper_threshold=args.gripper_threshold,
        max_episode_steps=args.max_episode_steps,
        camera_keys=args.camera_keys,
        camera_height=args.camera_height,
        camera_width=args.camera_width,
    )
    env = make_franka_real_env(cfg, camera_capture=camera_capture)
    env = TeleopInterventionWrapper(env, device=args.teleop_device)
    image_keys = [k for k in env.single_observation_space.spaces if k != "state"]

    successes: list[dict] = []
    failures: list[dict] = []

    obs, _ = env.reset()
    try:
        while len(successes) < args.successes_needed:
            action = torch.zeros(env.action_space.shape)
            next_obs, _reward, terminated, truncated, info = env.step(action)

            image_obs = {k: index_tree(obs[k], 0) for k in image_keys}
            entry = {"obs": image_obs}
            if success_key[0]:
                successes.append(entry)
                success_key[0] = False
                print(f"success {len(successes)}/{args.successes_needed}", flush=True)
            else:
                failures.append(entry)

            obs = next_obs
            if bool(terminated) or bool(truncated):
                obs, _ = env.reset()
    finally:
        listener.stop()
        env.close()

    save_classifier_data(args.output_dir, args.exp_name, args.successes_needed, successes, failures)


def save_classifier_data(
    output_dir: str, exp_name: str, success_count: int, successes: list, failures: list
) -> tuple[str, str]:
    """Pickles ``successes`` and ``failures`` to ``<exp_name>_<success_count>_success_images_<time>.pkl``
    and ``<exp_name>_failure_images_<time>.pkl`` under ``output_dir``; returns both paths."""
    os.makedirs(output_dir, exist_ok=True)
    uuid = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    success_path = os.path.join(output_dir, f"{exp_name}_{success_count}_success_images_{uuid}.pkl")
    failure_path = os.path.join(output_dir, f"{exp_name}_failure_images_{uuid}.pkl")
    with open(success_path, "wb") as f:
        pickle.dump(successes, f)
    with open(failure_path, "wb") as f:
        pickle.dump(failures, f)
    print(f"saved {len(successes)} successes to {success_path}", flush=True)
    print(f"saved {len(failures)} failures to {failure_path}", flush=True)
    return success_path, failure_path


if __name__ == "__main__":
    main()
