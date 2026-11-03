"""Inspect a recorder SQLite archive without modifying it; optionally export step evidence."""
from __future__ import annotations

import argparse
from collections import Counter
from contextlib import closing
import csv
import json
from pathlib import Path
import sqlite3

import torch

from real_world_rl.loops.durable import decode


def json_value(value):
    if isinstance(value, torch.Tensor):
        return value.detach().cpu().tolist()
    raise TypeError(f"unsupported archived value {type(value).__name__}")


def packed(value):
    return json.dumps(value, default=json_value, ensure_ascii=False, separators=(",", ":"))


def mask_bits(value):
    return "".join("1" if bit else "0" for bit in torch.as_tensor(value).reshape(-1).tolist())


def inspect_archive(path: Path, csv_path: Path | None = None):
    path = path.resolve(strict=True)
    if csv_path is not None and csv_path.suffix.lower() != ".csv":
        raise ValueError("step export must use a new .csv path")
    fields = ["episode_id", "step_index", "timestamp_s", "run_id", "human_action_mask", "override_mask",
              "action", "policy_action", "state", "next_state", "teleop_events", "terminated", "truncated"]
    stream = None
    summary = {"archive": str(path), "records": 0, "episode_starts": 0, "episode_ends": 0,
               "transitions": 0, "human_mask_counts": Counter(), "outcomes": []}
    try:
        # mode=ro prevents schema creation and interrupted-episode recovery writes.
        with closing(sqlite3.connect(path.as_uri() + "?mode=ro", uri=True)) as connection:
            connection.execute("PRAGMA query_only=ON")
            cursor = connection.execute("SELECT payload FROM messages ORDER BY seq")
            if csv_path is not None:
                stream = csv_path.open("x", encoding="utf-8-sig", newline="")
                writer = csv.DictWriter(stream, fieldnames=fields)
                writer.writeheader()
            for (payload,) in cursor:
                record = decode(payload)
                summary["records"] += 1
                kind = record["record_type"]
                if kind == "episode_start":
                    summary["episode_starts"] += 1
                elif kind == "episode_end":
                    summary["episode_ends"] += 1
                    summary["outcomes"].append(record.get("outcome"))
                elif kind == "transition":
                    summary["transitions"] += 1
                    bits = mask_bits(record["human_action_mask"])
                    summary["human_mask_counts"][bits] += 1
                    if stream is not None:
                        row = {key: packed(record.get(key)) for key in fields}
                        row.update(human_action_mask=bits, override_mask=mask_bits(record["override_mask"]),
                                   state=packed(record["obs"].get("state")),
                                   next_state=packed(record["next_obs"].get("state")))
                        writer.writerow(row)
    finally:
        if stream is not None:
            stream.close()
    summary["human_mask_counts"] = dict(summary["human_mask_counts"])
    return summary


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("archive", type=Path)
    parser.add_argument("--csv", type=Path, help="new output CSV; source SQLite is opened read-only")
    args = parser.parse_args(argv)
    print(json.dumps(inspect_archive(args.archive, args.csv), ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
