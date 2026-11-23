"""The recorder's dataset gives the training rows the old recorder pickled.

The recorder runs over a scripted joint-space env with hold gaps, leader-sync steps and a
rerecorded attempt; a spy around the env rebuilds, from the same steps, the rows the old recorder
put into its pkl. ``iter_transitions`` must give those rows, a dataset directory must load
through ``load_demo_pkls`` exactly as that pkl does, and as the dataset's own ``export_pkl`` does
at frame stacks 1 and 3."""
from __future__ import annotations

import pickle

import numpy as np
import pytest
import torch
from _leader_teleop_fakes import JointEnv, executed_joints, normalized_joints
from _recorder_dataset import record_into_dataset

from real_world_rl.contracts.action import demo_contract_of, write_contract_sidecar
from real_world_rl.dataset import Dataset, export_pkl, iter_transitions
from real_world_rl.demos.record import RecordDemosArgs
from real_world_rl.envs.wrappers.intervention import InterventionWrapper
from real_world_rl.teleop import TeleopReading
from real_world_rl.teleop.core.fake import ScriptedTeleopSource
from real_world_rl.teleop.leader.joint import TELEOP_SYNCING_EVENT
from real_world_rl.training.demo_loading import load_demo_pkls

Q1 = [0.2, -0.1, 0.0, 0.3, 0.0, 0.1, 0.0]
Q2 = [0.25, -0.1, 0.05, 0.3, 0.0, 0.1, 0.0]
OPEN, CLOSED = 1.0, -1.0
ACTION_DIM = 8
SPEC = {"state": {"shape": [4], "dtype": "float32"}, "rgb_wrist": {"shape": [6, 5, 3], "dtype": "uint8"}}


class CameraJointEnv(JointEnv):
    """A ``joint_abs`` env whose follower tracks the executed targets, observing a random
    ``state`` and ``rgb_wrist`` frame per step; the success step terminates with reward 1."""

    def __init__(self):
        super().__init__("joint_abs")
        self.rng = np.random.default_rng(0)
        self.steps = 0

    def _observe(self):
        return {"state": torch.from_numpy(self.rng.standard_normal((1, 4)).astype(np.float32)),
                "rgb_wrist": torch.from_numpy(self.rng.integers(0, 256, (1, 6, 5, 3), dtype=np.uint8))}

    def reset(self, **kwargs):
        super().reset(**kwargs)
        return self._observe(), {}

    def step(self, action):
        self.step_actions.append(action)
        self.steps += 1
        self.follower = executed_joints(torch.as_tensor(action).reshape(-1)[:7].numpy())
        return self._observe(), torch.tensor([0.5 * self.steps]), torch.tensor([False]), torch.tensor([False]), {}


class OldRowsSpy:
    """Delegates to ``env`` and rebuilds, from every step, the transition dicts the old recorder
    pickled: a step is a row when ``info["intervened"]`` (ramp-onset ``teleop_syncing`` steps
    only with ``keep_syncing_steps``), rows of kept episodes only, ``training_segment_end`` on a
    row whose successor is not the next step."""

    def __init__(self, env, keep_syncing_steps):
        self.env = env
        self.keep_syncing_steps = keep_syncing_steps
        self.rows, self._episode, self._obs, self._step = [], [], None, 0

    def __getattr__(self, name):
        return getattr(self.env, name)

    def reset(self, **kwargs):
        self._obs, info = self.env.reset(**kwargs)
        self._episode, self._step = [], 0
        return self._obs, info

    def step(self, action):
        next_obs, reward, terminated, truncated, info = self.env.step(action)
        events = info.get("teleop_events") or {}
        if info["intervened"] and (self.keep_syncing_steps or not events.get(TELEOP_SYNCING_EVENT, False)):
            row = {"obs": self._obs, "next_obs": next_obs, "action": info.get("intervene_action", action),
                   "reward": reward, "done": terminated, "terminated": terminated, "truncated": truncated,
                   "syncing": False, "step_index": self._step}
            for key in ("grasp_penalty", "human_action_mask", "override_mask", "policy_action", "intervened"):
                if key in info:
                    row[key] = info[key]
            self._episode.append(row)
        self._obs, self._step = next_obs, self._step + 1
        return next_obs, reward, terminated, truncated, info

    def on_episode(self, episode):
        if episode.kept:
            for index, row in enumerate(self._episode):
                following = self._episode[index + 1] if index + 1 < len(self._episode) else None
                if following is None or following["step_index"] != row["step_index"] + 1:
                    row["training_segment_end"] = True
            self.rows.extend(self._episode)


def _reading(q, *, intervened=True, gripper=CLOSED, episode_end=False, events=()):
    return TeleopReading(action=np.append(normalized_joints(q), gripper).astype(np.float32),
                         intervened=intervened, episode_end=episode_end, events={name: True for name in events})


SYNC = _reading([0.0] * 7, intervened=False, gripper=OPEN)
READINGS = [
    # A rerecorded attempt (discarded).
    SYNC, _reading(Q1), _reading(Q1, events=("rerecord",)),
    # The kept attempt: hold, sync catch-up, intervene, hold, intervene with success.
    _reading(Q1, intervened=False),
    _reading(Q1, intervened=False, gripper=OPEN),
    _reading(Q1, events=(TELEOP_SYNCING_EVENT,)),
    _reading(Q1, events=(TELEOP_SYNCING_EVENT,)),
    _reading(Q2),
    _reading(Q2),
    _reading(Q2, intervened=False),
    _reading(Q1, events=("success",), episode_end=True),
]


def _run(tmp_path, keep_syncing_steps):
    inner = InterventionWrapper(CameraJointEnv(), ScriptedTeleopSource(READINGS, action_mode="joint_abs", arm_action_dim=7))
    spy = OldRowsSpy(inner, keep_syncing_steps)
    args = RecordDemosArgs(num_successes=1, control_hz=1e6, keep_syncing_steps=keep_syncing_steps,
                           episode_end_behavior="auto")
    record_into_dataset(spy, args, tmp_path / "ds", obs_spec=SPEC, on_episode=spy.on_episode)
    return spy, spy.rows, Dataset(tmp_path / "ds")


def _assert_same_value(new, old, key):
    if isinstance(old, torch.Tensor):
        assert isinstance(new, torch.Tensor), key
        assert (new.dtype, new.shape) == (old.dtype, old.shape), key
        assert torch.equal(new, old), key
    elif isinstance(old, dict):
        assert set(new) == set(old), key
        for name in old:
            _assert_same_value(new[name], old[name], f"{key}.{name}")
    else:
        assert type(new) is type(old) and new == old, key


@pytest.mark.parametrize("keep_syncing_steps, steps", [(False, [3, 4, 6]), (True, [1, 2, 3, 4, 6])])
def test_training_view_equals_the_old_recorders_rows(tmp_path, keep_syncing_steps, steps):
    _, old_rows, dataset = _run(tmp_path, keep_syncing_steps)
    assert [info.status for info in dataset.episodes()] == ["discarded", "kept"]
    kept = dataset.episode(1)
    assert kept.data["syncing"].tolist() == [False, True, True, False, False, False, False]
    assert kept.meta["segments"] == [{"start": 0, "end": 1, "kind": "hold"}, {"start": 1, "end": 5, "kind": "intervene"},
                                     {"start": 5, "end": 6, "kind": "hold"}, {"start": 6, "end": 7, "kind": "intervene"}]

    rows = list(iter_transitions(dataset, frame_stack=1, keep_syncing_steps=keep_syncing_steps))
    assert [row["step_index"] for row in old_rows] == [row["step_index"] for row in rows] == steps
    for new, old in zip(rows, old_rows):
        assert set(new) - {"training_segment_end", "episode_id"} <= set(old)
        assert ("training_segment_end" in new) == ("training_segment_end" in old)
        for key in new:
            if key != "episode_id":
                _assert_same_value(new[key], old[key], key)
        assert new["episode_id"] == kept.meta["episode_id"]
    assert [row.get("training_segment_end", False) for row in rows] == [
        row.get("training_segment_end", False) for row in old_rows]


def _old_pkl(rows, path, contract):
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "wb") as stream:
        pickle.dump(rows, stream)
    write_contract_sidecar(path, contract)
    return path


def _adder_calls(path, contract, frame_stack=1):
    calls = []
    count = load_demo_pkls([str(path)], lambda *columns, **extra: calls.append((columns, extra)), "cpu",
                           expected_contract=contract, frame_stack=frame_stack)
    assert count == len(calls)
    return calls


def _assert_same_calls(new_calls, old_calls):
    assert len(new_calls) == len(old_calls)
    for (new_columns, new_extra), (old_columns, old_extra) in zip(new_calls, old_calls):
        for new, old in zip(new_columns, old_columns):
            _assert_same_value(new, old, "column")
        assert set(new_extra) == set(old_extra)
        for key in old_extra:
            _assert_same_value(torch.as_tensor(new_extra[key]), torch.as_tensor(old_extra[key]), key)


@pytest.mark.parametrize("keep_syncing_steps", [False, True])
def test_the_dataset_loads_as_the_old_pkl_does(tmp_path, keep_syncing_steps):
    spy, old_rows, dataset = _run(tmp_path, keep_syncing_steps)
    contract = demo_contract_of(spy)
    old_path = _old_pkl(old_rows, tmp_path / "old.pkl", contract)
    new_path = export_pkl(dataset, tmp_path / "new" / "demos.pkl", frame_stack=1, keep_syncing_steps=keep_syncing_steps)
    assert new_path.with_suffix(".contract.json").is_file()

    old_calls = _adder_calls(old_path, contract)
    assert len(old_calls) == len(old_rows)
    _assert_same_calls(_adder_calls(new_path, contract), old_calls)
    # The dataset directory itself; its recorded keep_syncing_steps decides the syncing rows.
    _assert_same_calls(_adder_calls(dataset.root, contract), old_calls)


@pytest.mark.parametrize("frame_stack", [1, 3])
def test_loading_the_dataset_directory_equals_loading_its_export(tmp_path, frame_stack):
    spy, _, dataset = _run(tmp_path, False)
    contract = demo_contract_of(spy)
    exported = export_pkl(dataset, tmp_path / "export" / "demos.pkl", frame_stack=frame_stack)
    from_dir = _adder_calls(dataset.root, contract, frame_stack=frame_stack)
    assert len(from_dir) == 3
    image = from_dir[0][0][0]["rgb_wrist"]
    assert image.shape == ((1, 6, 5, 3) if frame_stack == 1 else (1, frame_stack, 6, 5, 3))
    _assert_same_calls(from_dir, _adder_calls(exported, contract))
