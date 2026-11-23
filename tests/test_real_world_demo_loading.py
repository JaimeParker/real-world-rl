"""Demo loading: legacy flat-observation wrapping in pkls, and demo dataset directories."""
from __future__ import annotations

import pickle

import torch

from real_world_rl.loops.hil_serl import HilSerlLearnerLoop
from real_world_rl.training.demo_loading import as_dict_observation, load_demo_pkls


def _transition(obs, next_obs) -> dict:
    return {
        "obs": obs,
        "next_obs": next_obs,
        "action": torch.zeros(1, 3),
        "reward": torch.zeros(1),
        "done": torch.zeros(1),
    }


def test_as_dict_observation_wraps_flat_tensor_as_state():
    obs = torch.randn(1, 4)
    wrapped = as_dict_observation(obs)
    assert set(wrapped) == {"state"}
    assert wrapped["state"] is obs


def test_as_dict_observation_leaves_dict_untouched():
    obs = {"state": torch.randn(1, 4), "rgb_wrist": torch.zeros(1, 8, 8, 3, dtype=torch.uint8)}
    assert as_dict_observation(obs) is obs


def test_load_demo_pkls_wraps_legacy_flat_obs_and_keeps_dict_obs(tmp_path):
    flat = _transition(torch.randn(1, 4), torch.randn(1, 4))
    dict_obs = {"state": torch.randn(1, 4)}
    dict_next = {"state": torch.randn(1, 4)}
    nested = _transition(dict_obs, dict_next)
    with open(tmp_path / "demo.pkl", "wb") as f:
        pickle.dump([flat, nested], f)

    received = []
    count = load_demo_pkls(
        [str(tmp_path / "*.pkl")], lambda *a, **kw: received.append(a), device="cpu"
    )

    assert count == 2
    (obs0, next0, *_), (obs1, next1, *_) = received
    assert torch.equal(obs0["state"], flat["obs"]) and torch.equal(next0["state"], flat["next_obs"])
    assert set(obs1) == {"state"} and torch.equal(obs1["state"], dict_obs["state"])
    assert torch.equal(next1["state"], dict_next["state"])


def test_hil_serl_add_transition_wraps_legacy_flat_obs():
    received = []
    transition = _transition(torch.randn(1, 4), torch.randn(1, 4))
    HilSerlLearnerLoop._add_transition(lambda *a, **kw: received.append(a), transition, "cpu")
    obs, next_obs = received[0][:2]
    assert torch.equal(obs["state"], transition["obs"])
    assert torch.equal(next_obs["state"], transition["next_obs"])


def test_loader_deduplicates_globs_and_projects_explicit_termination(tmp_path):
    row = _transition(torch.zeros(1, 4), torch.ones(1, 4))
    row.update(done=torch.ones(1, dtype=torch.bool), terminated=False, truncated=True)
    path = tmp_path / "one.pkl"
    path.write_bytes(pickle.dumps([row]))
    received = []
    assert load_demo_pkls([str(path), str(tmp_path / "*.pkl")],
                          lambda *args, **kwargs: received.append(args), "cpu") == 1
    assert not received[0][4]


def _dataset(root, *, keep_syncing_steps=False):
    """A two-step kept episode (step 0 a syncing step) under a ``joint_delta`` contract."""
    import numpy as np

    from real_world_rl.dataset import DatasetWriter

    contract = {"action_mode": "joint_delta", "arm_action_dim": 2, "gripper_in_action": True}
    writer = DatasetWriter.open(root, control_hz=10.0, obs_spec={"state": {"shape": [4], "dtype": "float32"}},
                                action_dim=3, action_contract=contract,
                                config={"keep_syncing_steps": keep_syncing_steps})
    episode = writer.begin_episode()
    episode.add_observation({"state": np.zeros((1, 4), np.float32)}, time_s=0.0)
    for step in range(2):
        episode.add_step(np.full(3, step, np.float32), np.zeros(3, np.float32), np.ones(3, bool), np.ones(3, bool),
                         True, 0.0, False, step == 1, step == 0, {})
        episode.add_observation({"state": np.full((1, 4), step + 1, np.float32)}, time_s=0.1 * (step + 1))
    episode.finish(status="kept", success=True)
    return contract


def test_a_dataset_directory_loads_with_its_header_contract_and_recorded_syncing_choice(tmp_path):
    import pytest

    from real_world_rl.training.demo_loading import demo_source_digest, read_demo_contract

    contract = _dataset(tmp_path / "plain")
    _dataset(tmp_path / "with_sync", keep_syncing_steps=True)
    actions = []
    add = lambda obs, next_obs, action, reward, done, **extra: actions.append(float(action[0, 0]))
    assert load_demo_pkls([str(tmp_path / "plain")], add, "cpu", expected_contract=contract) == 1
    assert load_demo_pkls([str(tmp_path / "with_sync")], add, "cpu", expected_contract=contract) == 2
    assert actions == [1.0, 0.0, 1.0]
    assert read_demo_contract([str(tmp_path / "pl*")]) == contract
    with pytest.raises(ValueError, match="action_mode"):
        load_demo_pkls([str(tmp_path / "plain")], add, "cpu", expected_contract={**contract, "action_mode": "joint_abs"})
    with pytest.raises(ValueError, match="arm_action_dim"):
        load_demo_pkls([str(tmp_path / "plain")], add, "cpu", expected_contract={**contract, "arm_action_dim": 3})
    (tmp_path / "empty").mkdir()
    with pytest.raises(ValueError, match="without a dataset.json"):
        load_demo_pkls([str(tmp_path / "empty")], add, "cpu")

    digest = demo_source_digest(str(tmp_path / "plain"))
    assert digest["bytes"] > 0 and digest == demo_source_digest(str(tmp_path / "plain"))
    assert digest["sha256"] != demo_source_digest(str(tmp_path / "with_sync"))["sha256"]
