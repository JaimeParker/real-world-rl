"""Tests for ActorLoop against a fake env/policy/sync client.

The fakes below only implement the surface ActorLoop actually calls
(``env.reset``/``env.step``/``env.num_envs``, ``policy.eval``/``predict``/
``load_state_dict``, ``sync_client.start``/``stop``/``push_transition``/
``latest_policy_params``) so a test failure here means ActorLoop itself
reached for something outside its documented contract.
"""
from __future__ import annotations

import pytest
import torch

from real_world_rl.loops.actor import ActorLoop


class _FakeEnv:
    num_envs = 1

    def __init__(self, terminate_every: int | None = None):
        self.terminate_every = terminate_every
        self.reset_calls = 0
        self.step_calls = 0
        self._t = 0

    def reset(self, *, seed=None):
        del seed
        self.reset_calls += 1
        self._t = 0
        return torch.zeros(1, 4), {}

    def step(self, action):
        self.step_calls += 1
        self._t += 1
        next_obs = torch.full((1, 4), float(self.step_calls))
        reward = torch.tensor([1.0])
        terminated = torch.tensor(
            [self.terminate_every is not None and self._t % self.terminate_every == 0]
        )
        truncated = torch.tensor([False])
        return next_obs, reward, terminated, truncated, {}


class _FakePolicy:
    def __init__(self):
        self.eval_called = False
        self.loaded_state_dicts = []
        self.predict_calls = 0

    def eval(self):
        self.eval_called = True

    def load_state_dict(self, sd):
        self.loaded_state_dicts.append(sd)

    def predict(self, obs, deterministic=False):
        del obs, deterministic
        self.predict_calls += 1
        return torch.ones(1, 2)


class _FakeSyncClient:
    def __init__(self, params_to_serve=None):
        self.started = False
        self.stopped = False
        self.pushed = []
        self._params = params_to_serve

    def start(self):
        self.started = True

    def stop(self):
        self.stopped = True

    def push_transition(self, transition):
        self.pushed.append(transition)

    def latest_policy_params(self):
        return self._params


def test_actor_loop_steps_env_and_pushes_transitions():
    env = _FakeEnv()
    policy = _FakePolicy()
    sync_client = _FakeSyncClient()
    loop = ActorLoop(env, policy, sync_client, control_hz=1_000_000.0)

    loop.run(total_steps=3)

    assert env.reset_calls == 1
    assert env.step_calls == 3
    assert len(sync_client.pushed) == 3
    assert sync_client.started and sync_client.stopped
    assert policy.eval_called
    torch.testing.assert_close(sync_client.pushed[0]["obs"], torch.zeros(1, 4))
    torch.testing.assert_close(sync_client.pushed[0]["next_obs"], torch.full((1, 4), 1.0))


def test_actor_loop_resets_env_after_termination():
    env = _FakeEnv(terminate_every=2)
    policy = _FakePolicy()
    sync_client = _FakeSyncClient()
    loop = ActorLoop(env, policy, sync_client, control_hz=1_000_000.0)

    loop.run(total_steps=2)

    # The completed final step releases the robot without an extra reset.
    assert env.reset_calls == 1


def test_actor_loop_stops_after_the_episode_the_operator_quit():
    class _QuittingEnv(_FakeEnv):
        """The operator presses q on the second step (episode end plus ``operator_quit``)."""

        def step(self, action):
            next_obs, reward, terminated, truncated, info = super().step(action)
            if self.step_calls == 2:
                return next_obs, reward, terminated, torch.tensor([True]), {"teleop_events": {"operator_quit": True}}
            return next_obs, reward, terminated, truncated, info

    env = _QuittingEnv()
    sync_client = _FakeSyncClient()
    ActorLoop(env, _FakePolicy(), sync_client, control_hz=1_000_000.0).run(total_steps=10)

    assert env.step_calls == 2 and env.reset_calls == 1
    assert len(sync_client.pushed) == 2 and sync_client.stopped


def test_actor_loop_loads_unchanged_policy_params_once():
    env = _FakeEnv()
    policy = _FakePolicy()
    params = {"w": torch.tensor([1.0])}
    sync_client = _FakeSyncClient(params_to_serve=params)
    loop = ActorLoop(env, policy, sync_client, control_hz=1_000_000.0)

    loop.run(total_steps=2)

    assert len(policy.loaded_state_dicts) == 1
    assert policy.loaded_state_dicts[0] is params


def test_actor_loop_records_intervene_action_when_present():
    class _InterveningEnv(_FakeEnv):
        def step(self, action):
            next_obs, reward, terminated, truncated, _ = super().step(action)
            return next_obs, reward, terminated, truncated, {"intervene_action": torch.full((1, 2), 5.0)}

    env = _InterveningEnv()
    policy = _FakePolicy()
    sync_client = _FakeSyncClient()
    loop = ActorLoop(env, policy, sync_client, control_hz=1_000_000.0)

    loop.run(total_steps=1)

    torch.testing.assert_close(sync_client.pushed[0]["action"], torch.full((1, 2), 5.0))


def test_actor_loop_rejects_multi_env():
    class _MultiEnv(_FakeEnv):
        num_envs = 2

    with pytest.raises(ValueError, match="num_envs"):
        ActorLoop(_MultiEnv(), _FakePolicy(), _FakeSyncClient())


def test_actor_loop_beats_the_supervisor_heartbeat_per_step_and_announces_resets():
    from robot_infra.supervise import BEAT_BYTE, LONG_OPERATION_BYTE, capture_heartbeats

    env = _FakeEnv(terminate_every=2)
    loop = ActorLoop(env, _FakePolicy(), _FakeSyncClient(), control_hz=1_000_000.0)
    with capture_heartbeats() as received:
        loop.run(total_steps=4)
        messages = received()

    # One long-operation announcement per reset and one at cleanup, one beat per step and per reset.
    assert messages.count(LONG_OPERATION_BYTE) == env.reset_calls + 1
    assert messages.count(BEAT_BYTE) == 4 + env.reset_calls
    assert messages[0] == LONG_OPERATION_BYTE[0]  # the first reset is announced before it runs


def test_actor_holding_waits_for_next_without_adding_transitions():
    from types import SimpleNamespace
    import numpy as np
    from real_world_rl.teleop.composite import CompositeTeleopSource

    class SessionEnv(_FakeEnv):
        def __init__(self):
            super().__init__(terminate_every=1)
            events = iter([{}, {}, {'next_episode': True}, {}, {}, {'operator_quit': True}])
            self.source = CompositeTeleopSource(
                SimpleNamespace(session_input=lambda: (np.zeros(6), {})),
                SimpleNamespace(poll=lambda: SimpleNamespace(events=next(events))))
            self.hold_ticks = []

        def session_hold_target(self):
            return np.array([self.reset_calls, 0, 0, 0, 0, 0, 1.])

        session_start_target = session_hold_target

        def session_observe(self):
            return torch.full((1, 4), 42.)

        def session_hold_tick(self, target):
            self.hold_ticks.append((self.step_calls, self.reset_calls, target.copy()))

    env = SessionEnv()
    sync = _FakeSyncClient()
    loop = ActorLoop(env, _FakePolicy(), sync, control_hz=1_000_000.)
    loop.configure_episode_session('hold', neutral_stable_s=0.)
    loop.run(total_steps=2)
    assert env.step_calls == 2
    assert env.reset_calls == 2
    assert len(sync.pushed) == 2
    assert [(step, reset) for step, reset, _ in env.hold_ticks] == [(0, 1), (1, 1), (1, 1), (1, 2), (2, 2)]
    assert sync.stopped


def test_actor_hold_rejects_unsupported_environment_before_reset():
    env = _FakeEnv()
    sync = _FakeSyncClient()
    loop = ActorLoop(env, _FakePolicy(), sync)
    loop.configure_episode_session('hold')
    with pytest.raises(ValueError, match='Cartesian env'):
        loop.run(total_steps=1)
    assert env.reset_calls == 0
    assert env.step_calls == 0
    assert sync.stopped


def test_actor_step_budget_holds_without_reset_or_an_extra_step():
    from types import SimpleNamespace
    import numpy as np
    from real_world_rl.teleop.composite import CompositeTeleopSource

    class BudgetEnv(_FakeEnv):
        def __init__(self):
            super().__init__()
            self.ticks = 0
            events = iter([{}, {'next_episode': True}, {'operator_quit': True}])
            self.source = CompositeTeleopSource(
                SimpleNamespace(session_input=lambda: (np.zeros(6), {})),
                SimpleNamespace(poll=lambda: SimpleNamespace(events=next(events))))

        def session_start_target(self):
            return np.array([0, 0, 0, 0, 0, 0, 1.])

        session_hold_target = session_start_target

        def session_observe(self):
            return torch.full((1, 4), 42.)

        def session_hold_tick(self, target):
            self.ticks += 1

    env = BudgetEnv()
    sync = _FakeSyncClient()
    loop = ActorLoop(env, _FakePolicy(), sync, control_hz=1_000_000.)
    loop.configure_episode_session('hold', neutral_stable_s=0.)
    loop.run(total_steps=1)
    assert env.step_calls == env.reset_calls == len(sync.pushed) == 1
    assert env.ticks == 2  # startup gate, then ignored next at budget


def test_two_interventions_restore_policy_in_one_archived_episode(tmp_path):
    from types import SimpleNamespace
    from test_teleop_intervention_wrapper import _FakeEnv as DeviceEnv, _FakeTeleop, _sample
    from real_world_rl.envs.wrappers.intervention import TeleopInterventionWrapper
    from real_world_rl.trajectory import DurableArchive

    device = DeviceEnv()
    wrapped = TeleopInterventionWrapper(device, teleop=_FakeTeleop([
        _sample(True, 5.), _sample(False), _sample(True, 7.), _sample(False), _sample(False)]))
    class Policy(_FakePolicy):
        def predict(self, obs, deterministic=False):
            self.predict_calls += 1
            return torch.ones(1, 7)
    class Sync(_FakeSyncClient):
        def __init__(self):
            super().__init__()
            self.run_id, self._actor_id, self.session_id = 'run', 'actor', 'session'
            self._sequence = 0
            self.experiment_hash = 'unknown'
            self._outbox = SimpleNamespace(path=tmp_path / 'outbox.sqlite3')
            self.archive = DurableArchive(tmp_path / 'trajectory.sqlite3')
        def bind_experiment(self, digest, contract):
            self.experiment_hash = digest
        def _new_message_id(self):
            self._sequence += 1
            return str(self._sequence)
        def push_record(self, record):
            self.archive.append(record)
        def push_transition(self, transition):
            self.archive.append(transition)
        def transport_metrics(self):
            return {}
    sync, policy = Sync(), Policy()
    try:
        ActorLoop(wrapped, policy, sync, control_hz=1_000_000.).run(total_steps=5)
        rows = list(sync.archive)
        transitions = [row for row in rows if row['record_type'] == 'transition']
        assert len({row['episode_id'] for row in rows}) == 1
        assert [row['record_type'] for row in rows] == ['episode_start'] + ['transition']*5 + ['episode_end']
        assert [row['step_index'] for row in transitions] == list(range(5))
        assert [bool(torch.as_tensor(row['human_action_mask']).any()) for row in transitions] == [True, False, True, False, False]
        for action, expected in zip(device.step_actions, [5., 1., 7., 1., 1.]):
            torch.testing.assert_close(action, torch.full((1, 7), expected))
        assert device.reset_calls == 1
        assert policy.predict_calls == 5
    finally:
        sync.archive.close()


@pytest.mark.parametrize('rollout_fault', [False, True])
def test_cleanup_heartbeat_failure_still_closes_actor(monkeypatch, rollout_fault):
    from types import SimpleNamespace
    from robot_infra.supervise import Heartbeat
    env = _FakeEnv()
    closed = []
    env.close = lambda: closed.append(True)
    if rollout_fault:
        def fail_step(action):
            raise ValueError('original rollout')
        env.step = fail_step
    calls = []
    def announce():
        calls.append(True)
        if len(calls) > 1:
            raise RuntimeError('cleanup heartbeat')
    monkeypatch.setattr(Heartbeat, 'from_env', lambda: SimpleNamespace(begin_long_operation=announce, beat=lambda: None))
    sync = _FakeSyncClient()
    with pytest.raises(ValueError if rollout_fault else RuntimeError,
                       match='original rollout' if rollout_fault else 'cleanup heartbeat'):
        ActorLoop(env, _FakePolicy(), sync, control_hz=1_000_000.).run(total_steps=1)
    assert closed == [True]
    assert sync.stopped


def test_actor_policy_and_transition_use_post_gate_observation():
    from types import SimpleNamespace
    import numpy as np
    from real_world_rl.teleop.composite import CompositeTeleopSource
    class SessionEnv(_FakeEnv):
        def __init__(self):
            super().__init__(terminate_every=1)
            events = iter([{}, {'operator_quit': True}])
            self.source = CompositeTeleopSource(
                SimpleNamespace(session_input=lambda: (np.zeros(6), {})),
                SimpleNamespace(poll=lambda: SimpleNamespace(events=next(events))))
            self.gated = False
        def session_start_target(self):
            return np.array([0, 0, 0, 0, 0, 0, 1.])
        session_hold_target = session_start_target
        def session_hold_tick(self, target):
            self.gated = True
        def session_observe(self):
            assert self.gated
            return torch.full((1, 4), 42.)
    class Policy(_FakePolicy):
        def predict(self, obs, deterministic=False):
            torch.testing.assert_close(obs, torch.full((1, 4), 42.))
            return super().predict(obs, deterministic)
    sync = _FakeSyncClient()
    loop = ActorLoop(SessionEnv(), Policy(), sync, control_hz=1e6)
    loop.configure_episode_session('hold', neutral_stable_s=0.)
    loop.run(total_steps=1)
    torch.testing.assert_close(sync.pushed[0]['obs'], torch.full((1, 4), 42.))
