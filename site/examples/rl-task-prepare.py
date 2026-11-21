"""Preserve a reviewed recorder session's contract for the remote HIL route."""
import argparse
from dataclasses import fields
from pathlib import Path

import yaml
from real_world_rl.training.args import TeleopArgs

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("session", type=Path)
parser.add_argument("output", type=Path)
parser.add_argument("--remote", required=True, help="actor-visible daemon HOST:PORT")
args = parser.parse_args()
recorded = yaml.safe_load(args.session.read_text())
config = yaml.safe_load(Path("site/examples/rl-task.yaml").read_text())
if recorded["env_backend"] != "real_robot":
    raise ValueError("this route requires single-arm real_robot")
for key in ("env_backend", "env_id", "obs", "real_robot", "convert_obs_rotation"):
    if key in recorded:
        config[key] = recorded[key]
for field in fields(TeleopArgs):
    if field.name in recorded:
        config[field.name] = recorded[field.name]
if config["real_robot"].get("task", {}).get("name", "none") != "none":
    raise ValueError("this route uses the preceding chapters' task.name=none")
if config["real_robot"]["action_mode"] != "cartesian_delta":
    raise ValueError("use the preceding chapters' Cartesian session, not the joint extension")
robot = config["real_robot"]
robot.pop("robot", None)
robot["remote"] = args.remote
# Preserve source options and calibration, but use terminal labels and episode hold.
config.update(teleop_event_device="stdin", episode_end_behavior="hold")
args.output.parent.mkdir(parents=True, exist_ok=True)
with args.output.open("x") as stream:
    yaml.safe_dump(config, stream, sort_keys=False)
print(f"config={args.output}")
print(f"remote={robot['remote']} action_mode={robot['action_mode']}")
print(f"teleop_device={config['teleop_device']} reward_from_teleop_events=True")
