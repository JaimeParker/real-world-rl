"""Read-only inspection of a trusted actor outbox, audit copy or learner journal."""
import argparse
import json
from pathlib import Path
import sqlite3

import torch
from real_world_rl.loops.durable import decode


def plain(value):
    if isinstance(value, torch.Tensor):
        return value.tolist()
    return value


def records(path):
    connection = sqlite3.connect(path.resolve().as_uri() + "?mode=ro", uri=True)
    try:
        for seq, identity, payload in connection.execute(
            "SELECT seq,message_id,payload FROM messages ORDER BY seq"
        ):
            value = decode(payload)
            if "kind" in value:
                if value["kind"] != "transition":
                    continue
                value = value["data"]
            yield seq, identity, value
    finally:
        connection.close()


parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("database", type=Path)
parser.add_argument("--id", help="exact record_id from the other machine")
parser.add_argument("--compare", type=Path, help="other database copied locally")
args = parser.parse_args()
found = False
for seq, identity, row in records(args.database):
    if args.id and identity != args.id:
        continue
    found = True
    keys = ("record_type", "record_id", "episode_id", "step_index", "policy_action",
            "action", "executed_action", "human_action_mask", "override_mask", "syncing",
            "reward", "terminated", "truncated", "outcome_source", "outcome",
            "policy_session_id", "policy_version", "global_update", "policy_phase")
    result = {key: plain(row[key]) for key in keys if key in row}
    result["journal_or_outbox_seq"] = seq
    print(json.dumps(result, ensure_ascii=False, sort_keys=True))
    if args.compare:
        other = next((value for _, rid, value in records(args.compare) if rid == identity), None)
        if other is None:
            raise ValueError(f"matching record missing: {identity}")
        for key in ("policy_action", "action", "human_action_mask", "reward"):
            if key in row:
                torch.testing.assert_close(row[key], other[key])
        print("matching_id_and_action_mask_reward=True")
if not found:
    raise SystemExit("no matching records (ACKed outbox rows are removed; inspect the learner journal)")
