"""Write test-style tensor transitions; no reset, step, simulation or hardware."""
import argparse
import dataclasses
from pathlib import Path
import pickle

import torch
import yaml

from real_world_rl.contracts.action import demo_contract_of, write_contract_sidecar
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
        rows = []
        # Same list-of-tensor-dicts format as tests/test_real_world_bc.py.
        for index in range(8):
            obs = {}
            for key, space in env.single_observation_space.spaces.items():
                shape = (1, *space.shape)
                obs[key] = (torch.randint(0, 256, shape, dtype=torch.uint8)
                            if key.startswith("rgb_") else torch.randn(shape) * 0.01)
            # Identity quaternion; synthetic, not a measured pose.
            obs["state"][0, 3:7] = torch.tensor([0., 0., 0., 1.])
            action = torch.tensor([[0.2, 0., 0., 0., 0., 0., 1.]])
            mask = torch.tensor([[True] * 6 + [index % 2 == 0]])
            rows.append(dict(obs=obs, next_obs={k: v.clone() for k, v in obs.items()},
                             action=action, reward=torch.ones(1), done=torch.zeros(1),
                             intervened=True, human_action_mask=mask,
                             override_mask=mask.clone()))
        path = args.output / "demos.pkl"
        with path.open("xb") as stream:
            pickle.dump(rows, stream)
        write_contract_sidecar(path, demo_contract_of(env))
        settings = yaml.safe_load(config.read_text())
        session = {key: settings[key] for key in ("env_backend", "env_id", "obs", "real_robot")}
        (args.output / "resolved-config.yaml").write_text(yaml.safe_dump(session, sort_keys=False))
        print(f"synthetic transitions: {len(rows)}; demo: {path}")
    finally:
        env.close()


if __name__ == "__main__":
    main()
