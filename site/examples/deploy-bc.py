"""Inspect a BC deployment offline, or record one explicitly requested robot episode."""
import argparse
from dataclasses import asdict
import json
from pathlib import Path
import time

import numpy as np
import torch

from real_world_rl.deploy.checkpoint import (
    DeployArgs, build_deploy_env, resolve_deploy_robot_config,
    restore_policy, restore_training_args, run_deploy,
)
from real_world_rl.envs.real_robot.config import RealRobotEnvConfig


def serial(value):
    if isinstance(value, torch.Tensor):
        return value.detach().cpu().tolist()
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    raise TypeError(type(value).__name__)


class RecordedEnv:
    """Keep proposals before step and feedback after step, flushing each record."""

    def __init__(self, env, stream):
        self.env, self.stream = env, stream
        self.index = 0

    def __getattr__(self, name):
        return getattr(self.env, name)

    def write(self, **record):
        self.stream.write(json.dumps(record, default=serial, allow_nan=False) + "\n")
        self.stream.flush()

    def reset(self, **kwargs):
        obs, info = self.env.reset(**kwargs)
        self.write(event="reset", state=obs.get("state"), tcp_pose=info.get("tcp_pose"))
        return obs, info

    def step(self, action):
        self.write(event="proposal", step=self.index, time_ns=time.time_ns(), policy_action=action)
        obs, reward, terminated, truncated, info = self.env.step(action)
        self.write(event="feedback", step=self.index, time_ns=time.time_ns(),
                   state=obs.get("state"), tcp_pose=info.get("tcp_pose"),
                   reward=reward, terminated=terminated, truncated=truncated)
        self.index += 1
        return obs, reward, terminated, truncated, info

    def close(self):
        self.env.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("checkpoint")
    parser.add_argument("--remote", default="127.0.0.1:50061")
    parser.add_argument("--xyz-scale-m", type=float)
    parser.add_argument("--run", action="store_true")
    parser.add_argument("--output", type=Path, default=Path("local/deploy-bc-episode"))
    options = parser.parse_args()
    config = RealRobotEnvConfig(remote=options.remote)
    if options.xyz_scale_m is not None:
        config.xyz_scale_m = options.xyz_scale_m
    args = DeployArgs(checkpoint=options.checkpoint, algo="real_world_bc", device="cpu",
                      real_robot=config, episodes=1, max_steps_per_episode=10,
                      enable_teleop=False)
    training = restore_training_args(args)
    resolved = resolve_deploy_robot_config(args, training)
    print("obs:", json.dumps(asdict(training.obs), sort_keys=True))
    print("real_robot:", json.dumps(asdict(resolved), sort_keys=True))
    if not options.run:
        print("offline config check: no reset, step or device connection")
        return
    # This tutorial deliberately covers the route-two task=none episode only.
    if resolved.task.name != "none":
        raise ValueError("this tutorial recorder requires task.name='none'")
    options.output.mkdir(parents=True, exist_ok=False)
    with (options.output / "steps.jsonl").open("x") as stream:
        env = RecordedEnv(build_deploy_env(args, training), stream)
        try:
            policy = restore_policy(args, training, env)
            results = run_deploy(args, env=env, policy=policy)
            with (options.output / "result.json").open("x") as result_file:
                json.dump([asdict(result) for result in results], result_file, indent=2,
                          default=serial, allow_nan=False)
        except BaseException as error:
            env.write(event="interrupted", error=type(error).__name__, message=str(error))
            raise
        finally:
            env.close()
    print("record:", options.output)


if __name__ == "__main__":
    main()
