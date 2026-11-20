"""Evaluate a trajectory-trained ACT checkpoint with the recording station's settings."""
import json
from dataclasses import dataclass
from pathlib import Path

import torch
from rl_garden.common.cli_args import ObservationArgs
from rl_garden.common.env_args import EnvBackendArgs, EnvRunArgs

from real_world_rl.contracts.action import demo_contract_of, validate_transitions_against_contract
from real_world_rl.demos.record import parse_args_with_preset
from real_world_rl.deploy.checkpoint import DeployArgs, run_deploy
from real_world_rl.policies.base_provider import ACTBaseProvider
from real_world_rl.training.args import HilEnvArgs, RealWorldFrankaArgs
from real_world_rl.training.env_building import build_hil_env
from real_world_rl.training.evaluation import close_env


@dataclass
class ActDeployArgs(HilEnvArgs, RealWorldFrankaArgs, EnvRunArgs, ObservationArgs, EnvBackendArgs):
    """Use the same observation, action and intervention settings as the demonstrations."""

    checkpoint: str = ""
    # The .contract.json sidecar of the demonstrations used for training.
    demo_contract: str = ""
    episodes: int = 1
    max_steps_per_episode: int | None = None
    device: str = "cpu"
    temporal_agg: bool = True
    enable_teleop: bool = True
    reward_from_teleop_events: bool = True
    eval_freq: int = 0


class ActPolicy(torch.nn.Module):
    def __init__(self, provider: ACTBaseProvider):
        super().__init__()
        self.provider = provider

    def reset_episode(self):
        self.provider.reset()

    def predict(self, obs, *, deterministic):
        return self.provider.select_action(obs)


def main_cli(argv=None):
    args = parse_args_with_preset(ActDeployArgs, argv)
    if not args.checkpoint or not args.demo_contract:
        raise ValueError("deploy-act requires --checkpoint and --demo-contract")
    if args.episodes < 1 or (args.max_steps_per_episode is not None and args.max_steps_per_episode < 1):
        raise ValueError("episode and step limits must be positive")
    contract = json.loads(Path(args.demo_contract).read_text(encoding="utf-8"))
    if not isinstance(contract, dict) or "action_mode" not in contract:
        raise ValueError("--demo-contract must be the training demonstration's action contract")
    env = build_hil_env(args, role="eval", enable_teleop=args.enable_teleop)
    try:
        validate_transitions_against_contract([], demo_contract_of(env), source=args.demo_contract, saved=contract)
        # Loading on meta avoids allocating a second copy of the network just to read camera order.
        metadata = torch.load(args.checkpoint, map_location="meta", weights_only=True)
        cameras = metadata.get("camera_keys")
        if cameras:
            actual = sorted(key for key in env.single_observation_space.spaces if key.startswith("rgb_"))
            if list(cameras) != actual:
                raise ValueError(f"ACT camera mismatch: checkpoint={cameras}, environment={actual}")
        provider = ACTBaseProvider(
            args.checkpoint,
            observation_space=env.single_observation_space,
            action_space=env.single_action_space,
            device=args.device,
            temporal_agg=args.temporal_agg,
        )
    except BaseException:
        close_env(env)
        raise
    run_deploy(DeployArgs(checkpoint=args.checkpoint, device=args.device, seed=args.seed,
                          episodes=args.episodes, max_steps_per_episode=args.max_steps_per_episode),
               env=env, policy=ActPolicy(provider))
    return 0
