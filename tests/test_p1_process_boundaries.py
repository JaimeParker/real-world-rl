"""Abrupt process exits at ACK, replay-consume and manifest publication seams."""
import json
import os
import subprocess
import sys
from pathlib import Path

import pytest
import torch

from real_world_rl.loops.checkpoint import save_bundle, restore_bundle
from real_world_rl.loops.durable import DurableStore
from test_p0_process_recovery import TinyAgent, loop, transition

ROOT = Path(__file__).resolve().parents[1]
HEADER = '''
import os, sys
from pathlib import Path
sys.path.insert(0, str(Path.cwd() / "tests"))
from test_p0_process_recovery import TinyAgent, loop, transition
from real_world_rl.loops.durable import DurableStore
from real_world_rl.loops.checkpoint import save_bundle, restore_bundle
'''


def child(script, *args, code=73):
    result = subprocess.run([sys.executable, "-c", HEADER + script, *map(str, args)],
                            cwd=ROOT, env=os.environ.copy(), capture_output=True,
                            text=True, timeout=30)
    assert result.returncode == code, result.stderr


def state(learner, store):
    return {"journal_cursor": learner._journal_cursor, "received": learner._received,
            "journal_identity": store.identity, "experiment_hash": "experiment-A",
            "experiment_contract": {"task": "test"},
            "effective_experiment": {"runtime_control": {"arm": {"sdk_version": "test-sdk"}}},
            "seed_datasets": [{"path": "seed.pkl", "sha256": "seed-hash", "bytes": 1}]}


def test_commit_then_process_exit_before_ack_retries_one_row(tmp_path):
    path = tmp_path / "journal.sqlite3"
    child('''
store = DurableStore(sys.argv[1])
store.bind_experiment("experiment-A", {"task": "test"})
learner = loop(TinyAgent(), store)
learner.experiment_hash = "experiment-A"
record = transition(1)
record["experiment_hash"] = "experiment-A"
learner._admit_transition(record)
os._exit(73)  # committed receiver never returns ACK
''', path)
    store = DurableStore(path)
    try:
        store.bind_experiment("experiment-A", {"task": "test"})
        learner = loop(TinyAgent(), store)
        learner.experiment_hash = "experiment-A"
        record = transition(1)
        record["experiment_hash"] = "experiment-A"
        learner._admit_transition(record)
        assert len(store) == 1
        assert learner._drain_journal() == 1
        assert learner._drain_journal() == 0
        assert len(learner.agent.replay_buffer) == 1
    finally:
        store.close()


def test_consume_then_process_exit_before_bundle_replays_tail_once(tmp_path):
    path = tmp_path / "journal.sqlite3"
    store = DurableStore(path)
    store.bind_experiment("experiment-A")
    original = loop(TinyAgent(), store)
    original._admit_transition(transition(1))
    original._drain_journal()
    manifest = save_bundle(original.agent, tmp_path / "checkpoints", state(original, store))
    original._admit_transition(transition(2))
    store.close()
    child('''
store = DurableStore(sys.argv[1])
learner = loop(TinyAgent(), store)
state = restore_bundle(learner.agent, sys.argv[2])
learner._journal_cursor = state["journal_cursor"]
learner._received = state["received"]
assert learner._drain_journal() == 1
assert len(learner.agent.replay_buffer) == 2
os._exit(73)  # replay mutated, no new bundle published
''', path, manifest)
    store = DurableStore(path)
    try:
        recovered = loop(TinyAgent(), store)
        saved = restore_bundle(recovered.agent, manifest)
        assert saved["effective_experiment"]["runtime_control"]["arm"]["sdk_version"] == "test-sdk"
        assert saved["seed_datasets"][0]["sha256"] == "seed-hash"
        recovered._journal_cursor = saved["journal_cursor"]
        recovered._received = saved["received"]
        assert recovered._drain_journal() == 1
        assert recovered._drain_journal() == 0
        assert recovered._received == len(recovered.agent.replay_buffer) == 2
        torch.testing.assert_close(recovered.agent.replay_buffer.rewards[:2, 0], torch.tensor([1., 2.]))
    finally:
        store.close()


@pytest.mark.parametrize("boundary, expected", [("before", 1), ("after", 2)])
def test_process_exit_at_manifest_replace_loads_complete_old_or_new_generation(tmp_path, boundary, expected):
    store = DurableStore(tmp_path / "journal.sqlite3")
    learner = loop(TinyAgent(), store)
    learner._admit_transition(transition(1)); learner._drain_journal()
    learner.agent.global_update = 1
    manifest = save_bundle(learner.agent, tmp_path / "checkpoints", state(learner, store))
    identity = store.identity
    store.close()
    child('''
import real_world_rl.loops.checkpoint as checkpoint
agent = TinyAgent()
old = restore_bundle(agent, sys.argv[1])
record = transition(2)
agent.replay_buffer.add(record["obs"], record["next_obs"], record["action"], record["reward"], record["done"])
agent.global_update = 2
old.update(journal_cursor=2, received=2)
replace = checkpoint.os.replace
def crash_replace(source, destination):
    if sys.argv[2] == "before":
        os._exit(73)
    replace(source, destination)
    os._exit(73)
checkpoint.os.replace = crash_replace
save_bundle(agent, Path(sys.argv[1]).parent, old)
''', manifest, boundary)
    target = TinyAgent()
    saved = restore_bundle(target, manifest, expected_journal_identity=identity,
                           expected_experiment_hash="experiment-A")
    assert target.global_update == len(target.replay_buffer) == expected
    assert saved["journal_cursor"] == saved["received"] == expected
    generation = manifest.parent / json.loads(manifest.read_text())["generation"]
    metadata = json.loads((generation / "experiment.json").read_text())
    assert metadata["experiment_hash"] == "experiment-A"
    assert metadata["effective_experiment"]["runtime_control"]["arm"]["sdk_version"] == "test-sdk"
