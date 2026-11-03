"""TeleopEventRewardWrapper: success/failure events to reward and episode end (tensor, numpy and
python outputs, never lowering what the inner env set), behind the intervention wrapper, and the
HIL entrypoints' warning when a run has no reward source."""
from __future__ import annotations

import logging

import gymnasium as gym
import numpy as np
import pytest
import torch
from _teleop_fakes import ContractEnv

from real_world_rl.envs.wrappers import TeleopEventRewardWrapper, teleop_event_reward
from real_world_rl.envs.wrappers.intervention import InterventionWrapper
from real_world_rl.teleop import TeleopReading
from real_world_rl.teleop.fake import ScriptedTeleopSource
from real_world_rl.training import env_building


class _StepEnv(gym.Env):
    """Returns the given ``(reward, terminated, truncated)`` and ``info`` from every step."""

    num_envs = 1

    def __init__(self, reward, terminated, truncated, info=None):
        self.result = (reward, terminated, truncated)
        self.info = dict(info or {})

    def reset(self, **kwargs):
        return {"state": torch.zeros(1, 4)}, {}

    def step(self, action):
        reward, terminated, truncated = self.result
        return {"state": torch.ones(1, 4)}, reward, terminated, truncated, dict(self.info)


def _events(*names) -> dict:
    return {"teleop_events": {name: True for name in names}}


def _step(env, **kwargs):
    return TeleopEventRewardWrapper(env, **kwargs).step(torch.zeros(1, 8))


def _tensors(reward=0.0, terminated=False, truncated=False, dtype=torch.float32):
    return torch.tensor([reward], dtype=dtype), torch.tensor([terminated]), torch.tensor([truncated])


def test_the_package_exports_the_wrapper():
    assert TeleopEventRewardWrapper is teleop_event_reward.TeleopEventRewardWrapper


def test_success_pays_the_success_reward_and_terminates():
    _, reward, terminated, truncated, info = _step(_StepEnv(*_tensors(), info=_events("success")))
    assert reward.dtype == torch.float32 and reward.tolist() == [1.0]
    assert terminated.dtype == torch.bool and terminated.tolist() == [True]
    assert truncated.tolist() == [False]
    assert info["teleop_events"] == {"success": True}


def test_the_success_reward_is_configurable_and_keeps_dtype_and_device():
    # A device other than the default one, without needing an accelerator.
    meta = torch.device("meta")
    outputs = (
        torch.zeros(1, dtype=torch.float64, device=meta),
        torch.zeros(1, dtype=torch.bool, device=meta),
        torch.zeros(1, dtype=torch.bool, device=meta),
    )
    _, reward, terminated, truncated, _ = _step(_StepEnv(*outputs, info=_events("success")), success_reward=5.0)
    assert (reward.dtype, reward.device) == (torch.float64, meta)
    assert (terminated.dtype, terminated.device) == (torch.bool, meta)
    assert truncated is outputs[2]

    _, reward, _, _, _ = _step(_StepEnv(*_tensors(dtype=torch.float64), info=_events("success")), success_reward=5.0)
    assert reward.dtype == torch.float64 and reward.tolist() == [5.0]


def test_success_never_lowers_what_the_inner_env_set():
    # e.g. a reward classifier that already paid more and terminated.
    env = _StepEnv(*_tensors(reward=3.0, terminated=True), info=_events("success"))
    _, reward, terminated, _, _ = _step(env)
    assert reward.tolist() == [3.0] and terminated.tolist() == [True]


def test_failure_truncates_and_leaves_reward_and_terminated():
    outputs = _tensors(reward=0.5)
    _, reward, terminated, truncated, _ = _step(_StepEnv(*outputs, info=_events("failure")))
    assert reward is outputs[0] and terminated is outputs[1]
    assert truncated.dtype == torch.bool and truncated.tolist() == [True]


def test_a_failure_in_the_same_step_as_a_success_wins():
    outputs = _tensors()
    _, reward, terminated, truncated, _ = _step(_StepEnv(*outputs, info=_events("success", "failure")))
    assert reward.tolist() == [0.0] and terminated.tolist() == [False] and truncated.tolist() == [True]


@pytest.mark.parametrize("info", [{}, _events("rerecord"), {"teleop_events": {}}])
def test_other_steps_pass_through_unchanged(info):
    outputs = _tensors(reward=0.25)
    _, *passed, _ = _step(_StepEnv(*outputs, info=info))
    assert all(got is sent for got, sent in zip(passed, outputs))


def test_python_and_numpy_outputs():
    _, reward, terminated, truncated, _ = _step(_StepEnv(0.0, False, False, info=_events("success")))
    assert (reward, terminated, truncated) == (1.0, True, False)
    _, _, _, truncated, _ = _step(_StepEnv(0.0, False, False, info=_events("failure")))
    assert truncated is True

    arrays = (np.zeros(1, dtype=np.float32), np.zeros(1, dtype=bool), np.zeros(1, dtype=bool))
    _, reward, terminated, _, _ = _step(_StepEnv(*arrays, info=_events("success")))
    assert reward.dtype == np.float32 and reward.tolist() == [1.0] and terminated.tolist() == [True]


def test_reads_the_events_the_intervention_wrapper_writes():
    def reading(*events):
        return TeleopReading(action=np.zeros(7, dtype=np.float32), intervened=True, events={e: True for e in events})

    source = ScriptedTeleopSource([reading(), reading("success"), reading("failure")])
    env = TeleopEventRewardWrapper(InterventionWrapper(ContractEnv(), source), success_reward=1.0)
    env.reset()
    results = [env.step(torch.zeros(1, 7))[1:4] for _ in range(3)]

    rewards = [float(reward) for reward, _, _ in results]
    ends = [(bool(terminated), bool(truncated)) for _, terminated, truncated in results]
    assert rewards == [0.0, 1.0, 0.0]
    assert ends == [(False, False), (True, False), (False, True)]


# -- the HIL entrypoints' warning ---------------------------------------------------------


def _hil_serl():
    from real_world_rl.training.entrypoints import hil_serl

    return hil_serl, hil_serl.run_hil_serl, hil_serl.HilSerlArgs


def _residual():
    from real_world_rl.training.entrypoints import residual_hil_serl

    def args(**kwargs):
        return residual_hil_serl.ResidualHilSerlArgs(residual_variant="hybrid", base_provider="zero", **kwargs)

    return residual_hil_serl, residual_hil_serl.run_residual_hil_serl, args


def _silri():
    from real_world_rl.training.entrypoints import silri

    return silri, silri.run_silri, silri.SilriArgs


METHODS = {"hil_serl": _hil_serl, "residual_hil_serl": _residual, "silri": _silri}


@pytest.fixture(params=list(METHODS))
def method(request, monkeypatch):
    """``(name, run function, Args factory)`` with every role's body replaced by a no-op."""
    module, run, make_args = METHODS[request.param]()
    for role in ("_run_actor", "_run_learner", "_run_eval"):
        monkeypatch.setattr(module, role, lambda args: None)
    return request.param, run, make_args


def _warnings(caplog, run, args) -> list[str]:
    caplog.clear()
    with caplog.at_level(logging.WARNING, logger=env_building.__name__):
        run(args)
    return [record.getMessage() for record in caplog.records if record.name == env_building.__name__]


@pytest.mark.parametrize("role", ["actor", "learner"])
def test_a_run_without_a_reward_source_warns_naming_both_flags(method, caplog, role):
    name, run, make_args = method
    (message,) = _warnings(caplog, run, make_args(role=role))
    assert message.startswith(f"{name} {role}: no reward source")
    assert "--classifier-checkpoint" in message and "--reward-from-teleop-events" in message


@pytest.mark.parametrize(
    "fields",
    [{"role": "eval"}, {"reward_from_teleop_events": True}, {"classifier_checkpoint": "/tmp/classifier.pt"}],
)
def test_no_warning_with_a_reward_source_or_in_eval(method, caplog, fields):
    _, run, make_args = method
    assert _warnings(caplog, run, make_args(**{"role": "learner", **fields})) == []
