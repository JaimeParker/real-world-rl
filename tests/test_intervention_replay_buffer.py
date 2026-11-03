"""InterventionReplayBuffer: round trips for every flag combination, the
(T, N) layout, interop with rl-garden's prior-data mixing and high-UTD slicing,
and the error cases."""
from __future__ import annotations

import itertools
from types import SimpleNamespace

import numpy as np
import pytest
import torch
from gymnasium import spaces

from rl_garden.algorithms.sac_core import SACCore
from rl_garden.buffers.prior_data_replay import PriorDataReplayMixin
from rl_garden.common.types import ReplayBufferSample
from real_world_rl.buffers.intervention_replay_buffer import (
    InterventionReplayBuffer,
    InterventionSample,
)

ACTION_DIM = 3
OBS_SPACE = spaces.Dict({"state": spaces.Box(-1.0, 1.0, (4,), dtype=np.float32)})
ACTION_SPACE = spaces.Box(-1.0, 1.0, (ACTION_DIM,), dtype=np.float32)
FLAG_COMBINATIONS = list(itertools.product([False, True], repeat=3))


def _buffer(buffer_size=8, **kwargs) -> InterventionReplayBuffer:
    return InterventionReplayBuffer(
        OBS_SPACE,
        ACTION_SPACE,
        num_envs=1,
        buffer_size=buffer_size,
        storage_device="cpu",
        sample_device="cpu",
        **kwargs,
    )


def _add(buffer: InterventionReplayBuffer, value: float, **extras) -> None:
    """Row ``value``: every stored number derives from it so rows are identifiable."""
    full = torch.full
    kwargs = dict(
        intervened=torch.tensor([int(value) % 2 == 0]),
        policy_action=full((1, ACTION_DIM), value + 0.1),
    )
    if buffer.store_base_actions:
        kwargs["base_actions"] = full((1, ACTION_DIM), value + 0.2)
        kwargs["next_base_actions"] = full((1, ACTION_DIM), value + 0.3)
    if buffer.store_onset:
        kwargs["intervention_onset"] = torch.tensor([int(value) % 3 == 0])
    if buffer.store_grasp_penalty:
        kwargs["grasp_penalty"] = torch.tensor([-value])
    kwargs.update(extras)
    buffer.add(
        {"state": full((1, 4), value)},
        {"state": full((1, 4), value + 0.5)},
        full((1, ACTION_DIM), value),
        torch.tensor([value]),
        torch.tensor([0.0]),
        **kwargs,
    )


@pytest.mark.parametrize("store_base_actions,store_onset,store_grasp_penalty", FLAG_COMBINATIONS)
def test_round_trip_for_every_flag_combination(store_base_actions, store_onset, store_grasp_penalty):
    buffer = _buffer(
        store_base_actions=store_base_actions,
        store_onset=store_onset,
        store_grasp_penalty=store_grasp_penalty,
    )
    for value in range(5):
        _add(buffer, float(value))

    rows = torch.arange(5)
    sample = buffer._index_batch(rows, torch.zeros(5, dtype=torch.long))

    assert type(sample) is InterventionSample
    torch.testing.assert_close(sample.rewards, torch.arange(5, dtype=torch.float32))
    torch.testing.assert_close(sample.actions[:, 0], torch.arange(5, dtype=torch.float32))
    torch.testing.assert_close(sample.obs["state"][:, 0], torch.arange(5, dtype=torch.float32))
    torch.testing.assert_close(sample.next_obs["state"][:, 0], torch.arange(5) + 0.5)
    assert sample.intervened.dtype == torch.bool and sample.intervened.shape == (5,)
    assert sample.intervened.tolist() == [True, False, True, False, True]
    assert sample.policy_actions.dtype == torch.float32
    assert sample.policy_actions.shape == (5, ACTION_DIM)
    torch.testing.assert_close(sample.policy_actions[:, 0], torch.arange(5) + 0.1)

    if store_base_actions:
        assert sample.base_actions.shape == (5, ACTION_DIM)
        torch.testing.assert_close(sample.base_actions[:, 0], torch.arange(5) + 0.2)
        torch.testing.assert_close(sample.next_base_actions[:, 0], torch.arange(5) + 0.3)
    else:
        assert sample.base_actions is None and sample.next_base_actions is None
    if store_onset:
        assert sample.intervention_onset.dtype == torch.bool
        assert sample.intervention_onset.tolist() == [True, False, False, True, False]
    else:
        assert sample.intervention_onset is None
    if store_grasp_penalty:
        torch.testing.assert_close(sample.grasp_penalty, -torch.arange(5, dtype=torch.float32))
    else:
        assert sample.grasp_penalty is None


def test_sample_returns_an_intervention_sample_of_the_requested_size():
    buffer = _buffer(store_base_actions=True, store_onset=True)
    for value in range(6):
        _add(buffer, float(value))

    sample = buffer.sample(4)

    assert isinstance(sample, InterventionSample) and isinstance(sample, ReplayBufferSample)
    assert sample.rewards.shape == (4,)
    assert sample.intervened.shape == (4,)
    assert sample.policy_actions.shape == (4, ACTION_DIM)
    assert sample.base_actions.shape == (4, ACTION_DIM)
    # Columns stay aligned with their row.
    torch.testing.assert_close(sample.policy_actions[:, 0], sample.rewards + 0.1)
    torch.testing.assert_close(sample.base_actions[:, 0], sample.rewards + 0.2)


def test_storage_layout_is_time_by_env_with_one_env():
    buffer = _buffer(buffer_size=8, store_base_actions=True, store_onset=True)
    assert buffer.num_envs == 1 and buffer.per_env_buffer_size == 8
    assert buffer.intervened.shape == (8, 1) and buffer.intervened.dtype == torch.bool
    assert buffer.policy_actions.shape == (8, 1, ACTION_DIM)
    assert buffer.base_actions.shape == (8, 1, ACTION_DIM)
    assert buffer.next_base_actions.shape == (8, 1, ACTION_DIM)
    assert buffer.intervention_onset.shape == (8, 1)
    assert buffer.policy_actions.dtype == torch.float32


def test_unused_columns_are_not_allocated():
    buffer = _buffer()
    assert not hasattr(buffer, "base_actions")
    assert not hasattr(buffer, "next_base_actions")
    assert not hasattr(buffer, "intervention_onset")


def test_policy_action_defaults_to_the_stored_action():
    buffer = _buffer()
    buffer.add(
        {"state": torch.zeros(1, 4)},
        {"state": torch.zeros(1, 4)},
        torch.full((1, ACTION_DIM), 0.7),
        torch.tensor([1.0]),
        torch.tensor([0.0]),
        intervened=torch.tensor([True]),
    )
    torch.testing.assert_close(buffer.policy_actions[0], buffer.actions[0])


def test_python_bool_flags_are_accepted():
    buffer = _buffer(store_onset=True)
    _add(buffer, 1.0, intervened=True, intervention_onset=False)
    assert bool(buffer.intervened[0, 0]) is True
    assert bool(buffer.intervention_onset[0, 0]) is False


def test_extras_are_written_at_the_slot_the_base_columns_use_and_wrap_with_it():
    buffer = _buffer(buffer_size=4, store_base_actions=True)
    for value in range(6):
        _add(buffer, float(value))

    assert buffer.full and len(buffer) == 4
    # Rows 4 and 5 overwrote slots 0 and 1; extras follow the same slots.
    torch.testing.assert_close(buffer.rewards[:, 0], torch.tensor([4.0, 5.0, 2.0, 3.0]))
    torch.testing.assert_close(buffer.policy_actions[:, 0, 0], torch.tensor([4.1, 5.1, 2.1, 3.1]))
    torch.testing.assert_close(buffer.base_actions[:, 0, 0], torch.tensor([4.2, 5.2, 2.2, 3.2]))


def test_slice_keys_lists_the_extra_fields_present():
    assert _buffer().slice_keys() == ("intervened", "policy_actions", "human_action_mask", "override_mask")
    assert _buffer(store_base_actions=True).slice_keys() == (
        "intervened",
        "policy_actions",
        "human_action_mask",
        "override_mask",
        "base_actions",
        "next_base_actions",
    )
    assert _buffer(store_base_actions=True, store_onset=True, store_grasp_penalty=True).slice_keys() == (
        "intervened",
        "policy_actions",
        "human_action_mask",
        "override_mask",
        "base_actions",
        "next_base_actions",
        "intervention_onset",
        "grasp_penalty",
    )


def _filled_pair(**kwargs):
    first, second = _buffer(buffer_size=16, **kwargs), _buffer(buffer_size=16, **kwargs)
    for value in range(5):
        _add(first, float(value))
    for value in range(10, 13):
        _add(second, float(value))
    return first, second


def test_concat_and_shuffle_from_prior_data_replay_keep_rows_aligned():
    first, second = _filled_pair(store_base_actions=True, store_onset=True, store_grasp_penalty=True)
    mixin = PriorDataReplayMixin()

    combined = mixin._concat_replay_samples(first.sample(4), second.sample(3))
    assert type(combined) is InterventionSample
    assert combined.rewards.shape == (7,) and combined.base_actions.shape == (7, ACTION_DIM)

    shuffled = mixin._shuffle_batch(combined, 7)
    assert type(shuffled) is InterventionSample
    torch.testing.assert_close(shuffled.policy_actions[:, 0], shuffled.rewards + 0.1)
    torch.testing.assert_close(shuffled.next_base_actions[:, 0], shuffled.rewards + 0.3)
    torch.testing.assert_close(shuffled.grasp_penalty, -shuffled.rewards)
    assert shuffled.intervened.tolist() == [int(r) % 2 == 0 for r in shuffled.rewards.tolist()]
    assert shuffled.intervention_onset.tolist() == [int(r) % 3 == 0 for r in shuffled.rewards.tolist()]


def test_concat_and_shuffle_handle_absent_optional_columns():
    first, second = _filled_pair()
    mixin = PriorDataReplayMixin()
    combined = mixin._concat_replay_samples(first.sample(4), second.sample(3))
    shuffled = mixin._shuffle_batch(combined, 7)
    assert shuffled.base_actions is None and shuffled.intervention_onset is None


def test_high_utd_slice_keeps_the_extras_when_the_keys_are_listed():
    buffer = _buffer(buffer_size=16, store_base_actions=True, store_onset=True, store_grasp_penalty=True)
    for value in range(8):
        _add(buffer, float(value))
    batch = buffer.sample(8)
    algorithm = SimpleNamespace(_extra_batch_slice_keys=buffer.slice_keys())

    minibatch = SACCore._slice_batch(algorithm, batch, 2, 3)

    assert type(minibatch) is InterventionSample
    assert minibatch.rewards.shape == (3,)
    for name in buffer.slice_keys():
        assert getattr(minibatch, name).shape[0] == 3
        torch.testing.assert_close(getattr(minibatch, name), getattr(batch, name)[2:5])


def test_high_utd_slice_without_the_keys_fails_on_the_required_fields():
    buffer = _buffer()
    _add(buffer, 0.0)
    algorithm = SimpleNamespace(_extra_batch_slice_keys=())
    with pytest.raises(TypeError):
        SACCore._slice_batch(algorithm, buffer.sample(1), 0, 1)


def test_mmap_dir_is_rejected(tmp_path):
    with pytest.raises(ValueError, match="mmap_dir"):
        _buffer(mmap_dir=tmp_path / "mmap")


@pytest.mark.parametrize(
    "extras,match",
    [
        ({"base_actions": None}, "store_base_actions=True"),
        ({"next_base_actions": None}, "store_base_actions=True"),
        ({"intervention_onset": None}, "store_onset=True"),
    ],
)
def test_missing_required_extras_raise_without_writing(extras, match):
    buffer = _buffer(store_base_actions=True, store_onset=True)
    with pytest.raises(ValueError, match=match):
        _add(buffer, 1.0, **extras)
    assert buffer.pos == 0 and len(buffer) == 0
    assert not buffer.intervened.any()


@pytest.mark.parametrize(
    "extras,match",
    [
        ({"base_actions": torch.zeros(1, ACTION_DIM)}, "store_base_actions=False"),
        ({"next_base_actions": torch.zeros(1, ACTION_DIM)}, "store_base_actions=False"),
        ({"intervention_onset": torch.tensor([True])}, "store_onset=False"),
        ({"grasp_penalty": torch.zeros(1)}, "store_grasp_penalty=False"),
    ],
)
def test_extras_the_buffer_does_not_store_raise(extras, match):
    buffer = _buffer()
    with pytest.raises(ValueError, match=match):
        _add(buffer, 1.0, **extras)
    assert buffer.pos == 0


def test_intervened_is_required():
    buffer = _buffer()
    with pytest.raises(TypeError):
        buffer.add(
            {"state": torch.zeros(1, 4)},
            {"state": torch.zeros(1, 4)},
            torch.zeros(1, ACTION_DIM),
            torch.zeros(1),
            torch.zeros(1),
        )
