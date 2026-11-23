"""Write one synthetic episode as a demo dataset; no reset, step, simulation or hardware."""
import argparse
import dataclasses
from pathlib import Path

import torch
import yaml

from real_world_rl.contracts.action import demo_contract_of
from real_world_rl.dataset import DatasetWriter
from real_world_rl.training.env_building import build_offline_experiment_env
from real_world_rl.training.registry import registry
from rl_garden.envs.backend_registry import make_training_envs
from rl_garden.training.offline.runner import _eval_env_request


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    config = Path("docs/examples/bc-train.yaml")
    parsed = registry.parse_command(["real_world_bc", "--config", str(config)]).args
    import real_world_rl.envs.backends.real_robot  # noqa: F401

    request = _eval_env_request(parsed)
    observation = request.observation
    request.observation = dataclasses.replace(observation, frame_stack=1)
    env, _ = make_training_envs(parsed.env_backend, request)
    env = build_offline_experiment_env(env, request.backend_config, observation)
    try:
        torch.manual_seed(1)
        spaces = env.single_observation_space.spaces
        obs_spec = {key: {"shape": list(space.shape),
                          "dtype": "uint8" if key.startswith("rgb_") else "float32"}
                    for key, space in spaces.items()}
        steps = 8
        writer = DatasetWriter.open(
            args.output / "demos", control_hz=float(parsed.real_robot.control_hz), obs_spec=obs_spec,
            action_dim=7, action_contract=demo_contract_of(env))
        episode = writer.begin_episode()
        for index in range(steps + 1):
            obs = {}
            for key, space in spaces.items():
                shape = (1, *space.shape)
                obs[key] = (torch.randint(0, 256, shape, dtype=torch.uint8)
                            if key.startswith("rgb_") else torch.randn(shape) * 0.01)
            # Identity quaternion; synthetic, not a measured pose.
            obs["state"][0, 3:7] = torch.tensor([0., 0., 0., 1.])
            episode.add_observation(obs, time_s=1000.0 + index / parsed.real_robot.control_hz)
            if index == steps:
                break
            action = torch.tensor([[0.2, 0., 0., 0., 0., 0., 1.]])
            mask = torch.tensor([[True] * 6 + [index % 2 == 0]])
            episode.add_step(action, action, mask, mask.clone(), True, 1.0, False, False, False, {})
        episode.finish(status="kept", success=True)
        path = args.output / "demos"
        settings = yaml.safe_load(config.read_text())
        session = {key: settings[key] for key in ("env_backend", "env_id", "obs", "real_robot")}
        (args.output / "resolved-config.yaml").write_text(yaml.safe_dump(session, sort_keys=False))
        print(f"synthetic transitions: {steps}; demo dataset: {path}")
    finally:
        env.close()


if __name__ == "__main__":
    main()
