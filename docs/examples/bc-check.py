"""Load a trusted BC checkpoint and compare one saved observation on CPU."""
import argparse
import dataclasses
import json
from pathlib import Path

import torch

from real_world_rl.contracts.action import (
    demo_contract_of, validate_experiment, validate_transitions_against_contract,
)
from real_world_rl.dataset import Dataset, iter_transitions
from real_world_rl.deploy.checkpoint import (
    DeployArgs, restore_policy, restore_training_args,
)
from real_world_rl.training.env_building import build_offline_experiment_env
from rl_garden.envs.backend_registry import make_training_envs
from rl_garden.training.offline.runner import _eval_env_request


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("checkpoint", type=Path)
    parser.add_argument("demo", type=Path, help="demo dataset directory")
    parser.add_argument("--row", type=int, default=0)
    parser.add_argument("--record", type=Path, required=True)
    args = parser.parse_args()
    run = args.checkpoint.parent.parent
    if not (run / "config.json").is_file():
        raise FileNotFoundError(run / "config.json")
    metadata = json.loads((run / "experiment.json").read_text())
    dataset = Dataset(args.demo)
    contract = dataset.header["action_contract"]
    rows = list(iter_transitions(dataset, frame_stack=1))
    row = rows[args.row]
    deploy = DeployArgs(checkpoint=str(args.checkpoint), algo="real_world_bc", device="cpu")
    training = restore_training_args(deploy)
    training.device = "cpu"
    import real_world_rl.envs.backends.real_robot  # noqa: F401

    request = _eval_env_request(training)
    observation = request.observation
    request.observation = dataclasses.replace(observation, frame_stack=1)
    env, _ = make_training_envs(training.env_backend, request)
    env = build_offline_experiment_env(env, request.backend_config, observation, contract)
    try:
        validate_transitions_against_contract(
            rows, demo_contract_of(env), source=str(args.demo), saved=contract)
        validate_experiment(env, metadata["experiment_contract"])
        policy = restore_policy(deploy, training, env)
        policy.eval()
        # start inference
        obs = {key: torch.as_tensor(value).cpu() for key, value in row["obs"].items()}
        with torch.no_grad():
            predicted = policy.predict(obs, deterministic=True).cpu()
        target = torch.as_tensor(row["action"]).cpu()
        mask = torch.as_tensor(row["human_action_mask"]).bool().cpu()
        if predicted.shape != target.shape or mask.shape != target.shape:
            raise ValueError("prediction/action/mask shape mismatch")
        if not torch.isfinite(predicted).all() or not mask.any():
            raise ValueError("non-finite prediction or no supervised dimensions")
        mse = ((predicted - target).square() * mask).sum() / mask.sum()
        # end inference
        torch.set_printoptions(precision=4, sci_mode=False)
        print("observation shapes:", {key: list(value.shape) for key, value in obs.items()})
        print("demo action:", target)
        print("predicted action:", predicted)
        print("human_action_mask:", mask.to(torch.int))
        print(f"masked MSE: {mse.item():.6f}")
        record = dict(
            checkpoint=str(args.checkpoint), demo=str(args.demo), row=args.row,
            algorithm="real_world_bc", deterministic=True,
            observation_shapes={key: list(value.shape) for key, value in obs.items()},
            observation_dtypes={key: str(value.dtype) for key, value in obs.items()},
            state_fields=list(training.real_robot.state_fields),
            cameras=list(training.obs.rgb),
            action_conventions={key: getattr(training.real_robot, key) for key in (
                "action_mode", "cartesian_semantics", "xyz_scale_m", "rot_scale_rad",
                "joint_scale_rad", "limits", "gripper_in_action", "control_hz")},
            action_contract=contract,
            training_experiment=metadata,
            demo_action=target.tolist(), predicted_action=predicted.tolist(),
            human_action_mask=mask.tolist(), masked_mse=mse.item(),
        )
        args.record.parent.mkdir(parents=True, exist_ok=True)
        with args.record.open("x") as stream:
            json.dump(record, stream, indent=2, ensure_ascii=False, allow_nan=False)
        print("record:", args.record)
    finally:
        env.close()


if __name__ == "__main__":
    main()
