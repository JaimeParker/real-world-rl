"""Print the last real TensorBoard values; does not connect to a service."""
import argparse
from pathlib import Path
from tensorboard.backend.event_processing.event_accumulator import EventAccumulator

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("root", type=Path)
args = parser.parse_args()
files = sorted(args.root.rglob("events.out.tfevents.*"))
if not files:
    raise SystemExit("no TensorBoard events found")
for path in files:
    events = EventAccumulator(str(path)).Reload()
    for tag in sorted(events.Tags()["scalars"]):
        if tag.startswith(("learner/", "actor/", "episodes/", "transport/", "losses/")):
            last = events.Scalars(tag)[-1]
            print(f"{path}: {tag} step={last.step} value={last.value}")
