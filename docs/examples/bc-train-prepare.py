"""Combine recorded observation/control settings with the tutorial's BC settings."""
import argparse
from pathlib import Path
import pickle

import yaml


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("session_config", type=Path)
    parser.add_argument("demo", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    recorded = yaml.safe_load(args.session_config.read_text())
    if recorded["env_backend"] != "real_robot":
        raise ValueError("this tutorial requires the single-arm real_robot backend")
    config = yaml.safe_load(Path("docs/examples/bc-train.yaml").read_text())
    for key in ("env_backend", "env_id", "obs", "real_robot"):
        config[key] = recorded[key]
    config["demo_dataset_paths"] = [str(args.demo)]
    with args.demo.open("rb") as stream:
        count = len(pickle.load(stream))
    if count == 0:
        raise ValueError("empty demonstration")
    # Buffer must hold all selected rows, otherwise it silently retains only a suffix.
    config["buffer_size"] = max(config["buffer_size"], count)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("x") as stream:
        yaml.safe_dump(config, stream, sort_keys=False)
    print(f"config: {args.output}")
    print(f"demo: {args.demo}")
    print(f"transitions: {count}; buffer_size: {config['buffer_size']}")


if __name__ == "__main__":
    main()
