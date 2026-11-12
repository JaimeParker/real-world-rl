"""Run a task preset using fake devices, without constructing or training an algorithm.

PYTHONPATH=.:rl-garden python examples/task_smoke.py --config examples/presets/pick_and_place_fake.yaml
"""
from __future__ import annotations
import argparse
from pathlib import Path
import torch
import yaml
from rl_garden.common.effective_config import apply_strict_mapping
from real_world_rl.training.entrypoints.hil_serl import HilSerlArgs
from real_world_rl.training.env_building import build_hil_env
from real_world_rl.training.evaluation import evaluate_policy
from real_world_rl.action_contract import experiment_hash


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True)
    parser.add_argument("--episodes", type=int, default=2)
    parsed = parser.parse_args()
    args = HilSerlArgs()
    apply_strict_mapping(args, yaml.safe_load(Path(parsed.config).read_text()))
    configs = args.real_robot_bimanual.arm_configs.values() if args.env_backend == "real_robot_bimanual" else [args.real_robot]
    if any(config.robot is None or config.robot.get("arm_backend") != "fake" or config.remote is not None for config in configs):
        raise ValueError("task_smoke only accepts local fake arm configurations")
    env = build_hil_env(args, role="eval")
    print(f"experiment_hash={experiment_hash(env)}")
    class HoldPolicy:
        def predict(self, obs, deterministic=True):
            return torch.zeros((1, *env.single_action_space.shape))
    evaluate_policy(env, HoldPolicy(), episodes=parsed.episodes, seed=args.seed)


if __name__ == "__main__": main()
