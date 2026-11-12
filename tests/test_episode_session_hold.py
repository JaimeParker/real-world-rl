from types import SimpleNamespace

import numpy as np
import pytest

from real_world_rl.teleop.composite import CompositeTeleopSource
from real_world_rl.teleop.session import wait_after_episode
from real_world_rl.supervise import capture_heartbeats


class Backend:
    def __init__(self):
        self.target = np.array([.5, 0, .3, 0, 0, 0, 1.])
        self.ticks = []

    def session_hold_target(self):
        return self.target.copy()

    def session_hold_tick(self, target):
        self.ticks.append(target.copy())

    def step(self, *_):
        raise AssertionError("terminated episode must not be stepped")

    def reset(self, *_):
        raise AssertionError("holding must not reset")


def source(events, inputs):
    event_iterator = iter(events)
    input_iterator = iter(inputs)
    return CompositeTeleopSource(
        SimpleNamespace(session_input=lambda: (np.full(6, next(input_iterator)), {})),
        SimpleNamespace(poll=lambda: SimpleNamespace(events=next(event_iterator))))


def test_hold_ignores_motion_and_requires_neutral_only_for_next():
    env = Backend()
    src = source([{"next_episode": True}, {}, {}, {}, {"operator_quit": True}], [.8, .02, .02, .02])
    clock = iter([0., .01, .1, .11, .2, .21, .3, .31])
    with capture_heartbeats() as received:
        result = wait_after_episode(env, src, allow_next=True, control_hz=10,
                                    neutral_stable_s=.19, clock=lambda: next(clock), sleep=lambda _: None)
        assert received() == b"...."
    assert result == "next"
    assert len(env.ticks) == 4
    for target in env.ticks:
        np.testing.assert_array_equal(target, env.target)


def test_exhausted_budget_ignores_next_until_quit():
    env = Backend()
    src = source([{"next_episode": True}, {"operator_quit": True}], [0])
    assert wait_after_episode(env, src, allow_next=False, control_hz=20, sleep=lambda _: None) == "quit"
    assert len(env.ticks) == 1


def test_device_failure_is_not_silently_held_forever():
    env = Backend()
    src = source([{}], [float("nan")])
    with pytest.raises(ValueError, match="invalid session input"):
        wait_after_episode(env, src, allow_next=True, control_hz=20)
    assert not env.ticks


def test_startup_holds_post_reset_pose_without_next_event():
    from real_world_rl.teleop.session import wait_before_episode
    env = Backend()
    measured = env.target + np.array([.1, 0, 0, 0, 0, 0, 0])
    env.session_start_target = lambda: measured.copy()
    src = source([{}, {}, {}, {}], [.7, .01, .01, .01])
    clock = iter([0., .01, .1, .11, .2, .21, .3, .31])
    assert wait_before_episode(env, src, control_hz=10, neutral_stable_s=.19,
                               clock=lambda: next(clock), sleep=lambda _: None) == "next"
    for target in env.ticks:
        np.testing.assert_array_equal(target, measured)


def test_startup_quit_does_not_send_target():
    from real_world_rl.teleop.session import wait_before_episode
    env = Backend()
    env.session_start_target = env.session_hold_target
    src = source([{"operator_quit": True}], [])
    assert wait_before_episode(env, src, control_hz=20) == "quit"
    assert not env.ticks


def test_hold_emits_machine_readable_phase_without_transitions():
    env = Backend()
    phases = []
    env.session_event = lambda phase, reason: phases.append((phase, reason))
    src = source([{'operator_quit': True}], [])
    assert wait_after_episode(env, src, allow_next=False, control_hz=20) == 'quit'
    assert phases == [('EPISODE_HOLD', 'budget_exhausted'), ('STOPPING', 'operator_quit')]
    assert not env.ticks


def test_next_key_is_registered_only_for_session_mode():
    from real_world_rl.teleop.keyboard import OperatorKeySource
    source = OperatorKeySource()
    source._keys.press('n')
    assert 'next_episode' not in source.poll().events
    source.enable_session_events()
    source._keys.press('n')
    assert source.poll().events['next_episode']


def test_session_refresh_replaces_all_stacked_images():
    import gymnasium as gym
    import torch
    from gymnasium import spaces
    from gymnasium.vector.utils import batch_space
    from real_world_rl.envs.wrappers.obs_chain import apply_obs_chain
    from real_world_rl.teleop.session import session_observe
    class CameraEnv(gym.Env):
        num_envs = 1
        single_observation_space = spaces.Dict({'rgb_wrist': spaces.Box(0, 255, (2, 2, 3), dtype='uint8')})
        observation_space = batch_space(single_observation_space, 1)
        single_action_space = spaces.Box(-1, 1, (1,), dtype='float32')
        action_space = batch_space(single_action_space, 1)
        def reset(self, **kwargs):
            return {'rgb_wrist': torch.zeros((1, 2, 2, 3), dtype=torch.uint8)}, {}
        def session_observe(self):
            return {'rgb_wrist': torch.full((1, 2, 2, 3), 42, dtype=torch.uint8)}
    env = apply_obs_chain(CameraEnv(), frame_stack=3, image_keys=['rgb_wrist'],
                          convert_obs_rotation=False, quat_slice=None)
    env.reset()
    observed = session_observe(env)
    assert observed['rgb_wrist'].shape == (1, 3, 2, 2, 3)
    assert torch.all(observed['rgb_wrist'] == 42)


@pytest.mark.parametrize('options,key', [
    ({'neutral_threshold': 0.}, 'neutral_threshold'),
    ({'neutral_threshold': 1.}, 'neutral_threshold'),
    ({'neutral_threshold': float('nan')}, 'neutral_threshold'),
    ({'neutral_stable_s': -1.}, 'neutral_stable_s'),
    ({'neutral_stable_s': float('nan')}, 'neutral_stable_s'),
])
def test_neutral_option_errors_match_record_actor_and_wait(options, key):
    from real_world_rl.demos.record import RecordDemosArgs, _validate_episode_limit
    from real_world_rl.loops.actor import ActorLoop
    with pytest.raises(ValueError, match=key):
        _validate_episode_limit(RecordDemosArgs(**options))
    loop = object.__new__(ActorLoop)
    with pytest.raises(ValueError, match=key):
        loop.configure_episode_session('hold', **options)
    with pytest.raises(ValueError, match=key):
        wait_after_episode(Backend(), source([], []), allow_next=True, control_hz=10, **options)


def test_actor_episode_end_reaches_hold_target_without_stopping_the_arm(tmp_path, monkeypatch):
    """WO-17: ActorLoop's episode-end path never stops the arm before the HOLD target is read,
    so the device's last accepted target (cleared by any stop) is what HOLD re-sends."""
    import torch
    import real_world_rl.devices.claims as claims
    from real_world_rl.loops.actor import ActorLoop
    from test_real_robot_env import _Rig
    from test_real_world_actor_loop import _FakePolicy, _FakeSyncClient

    monkeypatch.setattr(claims, "_DEFAULT_LOCK_DIR", tmp_path / "claims")
    rig = _Rig("local")
    real = rig.env(max_episode_steps=1)
    arm = rig.arm
    record = {}

    class BatchedRobotEnv:
        num_envs = 1

        def __init__(self):
            self.env = real
            self.source = source([{}, {}, {"operator_quit": True}], [0., 0.])

        def reset(self, *, seed=None):
            real.reset(seed=seed)
            return torch.zeros(1, 4), {}

        def step(self, action):
            del action
            _, reward, terminated, truncated, _ = real.step(np.array([.5, 0, 0, 0, 0, 0, 0]))
            record["episode_end"] = len(arm.call_log)
            record["accepted"] = arm.last_cartesian_target
            return (torch.zeros(1, 4), torch.tensor([float(reward)]), torch.tensor([terminated]),
                    torch.tensor([truncated]), {})

        def session_observe(self):
            real.session_observe()
            return torch.zeros(1, 4)

        def close(self):
            real.close()

    read_target = real.session_hold_target
    def spy_target():
        record["calls_before_hold"] = list(arm.call_log[record["episode_end"]:])
        return read_target()
    monkeypatch.setattr(real, "session_hold_target", spy_target)
    held = []
    tick = real.session_hold_tick
    def spy_tick(target):
        if "episode_end" in record:
            held.append((target.copy(), list(arm.call_log[record["episode_end"]:])))
        return tick(target)
    monkeypatch.setattr(real, "session_hold_tick", spy_tick)

    loop = ActorLoop(BatchedRobotEnv(), _FakePolicy(), _FakeSyncClient(), control_hz=1_000_000.)
    loop.configure_episode_session("hold", neutral_stable_s=0.)
    try:
        loop.run(total_steps=1)
    finally:
        rig.close()
    assert record["accepted"] is not None
    assert record["calls_before_hold"] == []
    assert len(held) == 1
    target, calls_until_tick = held[0]
    np.testing.assert_array_equal(target, record["accepted"])
    assert "stop" not in calls_until_tick
