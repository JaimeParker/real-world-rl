"""Reward-classifier collection on a device-layer robot, with success/failure pickle output."""
from __future__ import annotations

import datetime
import os
import pickle
from typing import Optional, Sequence


def main(camera_capture=None, argv: Optional[Sequence[str]] = None) -> None:
    from real_world_rl.reward.collect_real_robot import parse_collect_args, run_real_robot_collector

    if camera_capture is not None:
        raise ValueError("the real_robot collector reads its cameras from the robot config; drop camera_capture")
    run_real_robot_collector(parse_collect_args(argv))


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


def main_cli(argv: Optional[Sequence[str]] = None) -> None:
    """rwrl adapter preserving the collector parser."""
    return main(argv=argv)


if __name__ == "__main__":
    main()
