"""Print BC scalars from the TensorBoard event directory."""
import argparse
from pathlib import Path

from tensorboard.backend.event_processing.event_accumulator import EventAccumulator


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("events", type=Path)
    args = parser.parse_args()
    events = EventAccumulator(str(args.events)).Reload()
    for tag in ("losses/loss", "losses/behavior_mse"):
        print(tag)
        for event in events.Scalars(tag):
            print(f"  step={event.step} value={event.value:.6f}")


if __name__ == "__main__":
    main()
