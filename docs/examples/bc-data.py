"""Inspect the training view of a single-arm demo dataset, without connecting any devices."""
import argparse
from collections import Counter
import json
from pathlib import Path

import torch

from real_world_rl.contracts.action import validate_transitions_against_contract
from real_world_rl.dataset import Dataset, iter_transitions


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("demo", type=Path, help="demo dataset directory")
    args = parser.parse_args()
    dataset = Dataset(args.demo)
    rows = list(iter_transitions(dataset, frame_stack=1))
    if not rows:
        raise ValueError("empty demonstration: the dataset has no kept episode with intervened steps")
    contract = dataset.header["action_contract"]
    validate_transitions_against_contract(rows, contract, source=str(args.demo), saved=contract)
    masks = Counter()
    reference = rows[0]["obs"]
    entries = contract.get("experiment", {}).get("observation", {}).get("entries", {})
    if entries:
        actual = {key: {"shape": list(value.shape[1:]),
                        "dtype": str(value.dtype).removeprefix("torch.")}
                  for key, value in reference.items()}
        if actual != entries:
            raise ValueError("observation shape/dtype differs from contract")
    width = contract["arm_action_dim"] + int(contract["gripper_in_action"])
    for index, row in enumerate(rows):
        action = torch.as_tensor(row["action"])
        mask = torch.as_tensor(row["human_action_mask"])
        if action.shape != (1, width) or mask.shape != action.shape:
            raise ValueError(f"row {index}: action/mask shape mismatch")
        if not torch.isfinite(action).all() or (action.abs() > 1).any():
            raise ValueError(f"row {index}: action outside finite [-1, 1]")
        if not ((mask == 0) | (mask == 1)).all():
            raise ValueError(f"row {index}: mask is not binary")
        for column in ("obs", "next_obs"):
            if set(row[column]) != set(reference):
                raise ValueError(f"row {index}: observation keys differ")
            for key, value in row[column].items():
                value = torch.as_tensor(value)
                if value.shape != reference[key].shape or not torch.isfinite(value).all():
                    raise ValueError(f"row {index}: invalid {column}.{key}")
        masks["".join(str(int(x)) for x in mask.flatten())] += 1
    print("transitions:", len(rows))
    print("observations:", {k: list(v.shape) for k, v in reference.items()})
    print("dtypes:", {k: str(v.dtype) for k, v in reference.items()})
    print("action dimension:", width)
    print("human masks:", dict(sorted(masks.items())))
    print("first action:", rows[0]["action"].tolist())
    print("contract:")
    print(json.dumps(contract, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
