"""Real-world replay checkpoints preserve extension labels and FWBW state."""
import numpy as np
import pytest
import torch
from gymnasium import spaces
from rl_garden.buffers.mc_buffer import MCReplayBuffer
from rl_garden.buffers.replay_buffer import ReplayBuffer
from real_world_rl.buffers.intervention_replay_buffer import InterventionReplayBuffer
from real_world_rl.loops.replay_state import serialize_buffer, restore_buffer

OBS = spaces.Dict({"state": spaces.Box(-1, 1, (2,), dtype=np.float32)})
ACT = spaces.Box(-1, 1, (3,), dtype=np.float32)


def make(cls=InterventionReplayBuffer, **kw):
    return cls(OBS, ACT, num_envs=1, buffer_size=4,
               storage_device="cpu", sample_device="cpu", **kw)


def add(buffer, index):
    obs = {"state": torch.full((1, 2), float(index))}
    action = torch.full((1, 3), float(index))
    extras = {}
    if isinstance(buffer, InterventionReplayBuffer):
        extras = dict(intervened=index % 2 == 0, policy_action=action + .1,
                      base_actions=action + .2, next_base_actions=action + .3,
                      intervention_onset=index % 3 == 0)
    if buffer.store_grasp_penalty:
        extras["grasp_penalty"] = torch.tensor([-float(index)])
    buffer.add(obs, obs, action, torch.tensor([float(index)]), torch.tensor([index % 2 == 0]), **extras)


@pytest.mark.parametrize("rows", [2, 7])
def test_all_extension_columns_and_ring_cursor_roundtrip(rows):
    config = dict(store_base_actions=True, store_onset=True, store_grasp_penalty=True)
    source, target = make(**config), make(**config)
    for index in range(rows):
        add(source, index)
    state = serialize_buffer(source)
    restore_buffer(target, state)
    assert (target.pos, target.full) == (source.pos, source.full)
    indices = torch.arange(len(source))
    envs = torch.zeros_like(indices)
    first, second = source._index_batch(indices, envs), target._index_batch(indices, envs)
    for key in ("actions", "rewards", "dones", *source.slice_keys()):
        torch.testing.assert_close(getattr(first, key), getattr(second, key))
    source.actions.zero_()
    assert torch.count_nonzero(state["actions"]) > 0
    target.actions.zero_()
    assert torch.count_nonzero(state["actions"]) > 0


def test_missing_legacy_labels_fail_before_mutation():
    config = dict(store_base_actions=True, store_onset=True)
    source, target = make(**config), make(**config)
    add(source, 1)
    state = serialize_buffer(source)
    del state["intervened"]
    with pytest.raises(ValueError, match="intervened"):
        restore_buffer(target, state)
    assert target.pos == 0


def test_grasp_penalty_on_plain_replay_is_preserved():
    source = make(ReplayBuffer, store_grasp_penalty=True)
    target = make(ReplayBuffer, store_grasp_penalty=True)
    add(source, 2)
    restore_buffer(target, serialize_buffer(source))
    torch.testing.assert_close(target.grasp_penalty, source.grasp_penalty)


def test_fwbw_episode_boundaries_and_external_mc_survive():
    source, target = make(MCReplayBuffer), make(MCReplayBuffer)
    for index in range(3):
        add(source, index)
    source._episode_end[1] = True
    source._externally_valid[1] = True
    source._external_mc[1] = 9
    restore_buffer(target, serialize_buffer(source))
    for name in ("_episode_end", "_externally_valid", "_external_mc"):
        torch.testing.assert_close(getattr(target, name), getattr(source, name))
    assert target._valid_table is None


def test_demo_projection_keeps_legacy_origin_unknown_and_drops_archive_metadata(tmp_path):
    import pickle
    from real_world_rl.training.demo_loading import load_demo_pkls
    path = tmp_path / "legacy.pkl"
    transition = dict(obs=torch.zeros(1, 2), next_obs=torch.ones(1, 2),
                      action=torch.zeros(1, 3), reward=torch.zeros(1), done=torch.zeros(1),
                      episode_id="episode-1", actor_id="actor-a", source="unknown",
                      teleop_events={"success": True})
    path.write_bytes(pickle.dumps([transition]))
    received = []
    def add(obs, next_obs, action, reward, done, **extra):
        received.append((obs, extra))
    assert load_demo_pkls([str(path)], add, "cpu") == 1
    assert set(received[0][0]) == {"state"}
    assert received[0][1] == {"intervened": True}  # Explicit legacy demo dataset intent; provenance metadata stays absent.


def test_demo_projection_targets_plain_buffer_interface():
    from real_world_rl.training.demo_loading import training_extras
    buffer = make(ReplayBuffer, store_grasp_penalty=True)
    result = training_extras(dict(intervened=True, policy_action=torch.zeros(1, 3),
                                  grasp_penalty=torch.zeros(1), episode_id="x"), buffer.add)
    assert set(result) == {"grasp_penalty"}


def test_legacy_native_replay_keeps_rows_without_inventing_expert_labels():
    from rl_garden.common.checkpoint import replay_buffer_state_dict
    source = make(store_base_actions=True, store_onset=True)
    target = make(store_base_actions=True, store_onset=True)
    add(source, 2)
    state = replay_buffer_state_dict(source)
    with pytest.warns(UserWarning, match="provenance"):
        restore_buffer(target, state)
    assert target.pos == source.pos
    assert target.legacy_provenance_unknown
    assert not target.intervened.any()
    assert not target.intervention_onset.any()
    torch.testing.assert_close(target.policy_actions, target.actions)
    torch.testing.assert_close(target.base_actions, source.base_actions)


def test_legacy_plain_replay_remains_readable():
    from rl_garden.common.checkpoint import replay_buffer_state_dict
    source, target = make(ReplayBuffer), make(ReplayBuffer)
    add(source, 1)
    restore_buffer(target, replay_buffer_state_dict(source))
    assert target.pos == 1
    assert not target.legacy_provenance_unknown
