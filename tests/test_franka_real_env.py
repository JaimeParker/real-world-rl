"""Tests for FrankaRealEnv against a fake bridge client (no HTTP/ROS/hardware)."""
from __future__ import annotations

import numpy as np
import pytest
import torch
from scipy.spatial.transform import Rotation

from real_world_rl.envs.franka_real.bridge_client import FrankaBridgeClient
from real_world_rl.envs.franka_real.config import FrankaRealEnvConfig
from real_world_rl.envs.franka_real.env import FrankaRealEnv


class _FakeBridgeClient(FrankaBridgeClient):
    """Overrides every network call with an in-memory fake state."""

    def __init__(self, initial_pose=None):
        self.reset_calls = 0
        self.stop_calls = 0
        self.sent_poses = []
        self.gripper_calls = []
        self._pose = (
            np.array(initial_pose, dtype=np.float64)
            if initial_pose is not None
            else np.array([0.5, 0.0, 0.2, 0.0, 0.0, 0.0, 1.0])
        )

    def reset_joints(self):
        self.reset_calls += 1

    def stop_impedance(self):
        self.stop_calls += 1

    def get_state(self):
        return {
            "pose": self._pose.tolist(),
            "vel": [0.0] * 6,
            "gripper_pos": 0.0,
            "force": [0.0, 0.0, 0.0],
            "torque": [0.0, 0.0, 0.0],
        }

    def send_pose(self, pose):
        self.sent_poses.append(np.asarray(pose, dtype=np.float64))
        self._pose = np.asarray(pose, dtype=np.float64)

    def send_gripper(self, open_gripper):
        self.gripper_calls.append(bool(open_gripper))


def _make_env(camera_capture=None, **cfg_kwargs) -> tuple[FrankaRealEnv, _FakeBridgeClient]:
    bridge = _FakeBridgeClient()
    cfg = FrankaRealEnvConfig(bridge_url="http://unused", **cfg_kwargs)
    env = FrankaRealEnv(cfg, bridge_client=bridge, camera_capture=camera_capture)
    return env, bridge


class _SerlClock:
    def __init__(self):
        self.now = 0.0
        self.events = []

    def monotonic(self):
        return self.now

    def sleep(self, seconds):
        self.events.append(("sleep", seconds))
        self.now += seconds


@pytest.fixture
def serl_clock(monkeypatch):
    import real_world_rl.envs.franka_real.env as env_module

    clock = _SerlClock()
    monkeypatch.setattr(env_module, "time", clock)
    return clock


def _make_serl_env(**overrides):
    values = dict(cartesian_semantics="serl", safety_box_low=(-1,) * 3, safety_box_high=(1,) * 3,
                  safety_box_rpy_low=(0, -np.pi, -np.pi), safety_box_rpy_high=(np.pi,) * 3)
    values.update(overrides)
    return _make_env(**values)


def test_serl_target_uses_latest_measured_pose(serl_clock, monkeypatch):
    env, bridge = _make_serl_env()
    env.reset()
    read_state = bridge.get_state

    def lagged_state():
        state = read_state()
        state["pose"] = [0.505, 0.0, 0.2, 0, 0, 0, 1]
        return state

    monkeypatch.setattr(bridge, "get_state", lagged_state)
    env.step([3, 0, 0, 0, 0, 0, 0])
    env.step([3, 0, 0, 0, 0, 0, 0])
    assert bridge.sent_poses[0][0] == pytest.approx(0.52)
    assert bridge.sent_poses[1][0] == pytest.approx(0.525)


def test_serl_rotation_left_multiplies_and_clips_roll_magnitude(serl_clock):
    env, bridge = _make_serl_env(safety_box_rpy_low=(2.5, -0.1, -0.15),
                                safety_box_rpy_high=(2.7, 0.1, 0.15))
    bridge._pose[3:] = Rotation.from_euler("xyz", [-2.9, 0.2, 0.3]).as_quat()
    env.reset()
    env.step([0, 0, 0, 0.2, -0.4, 0.6, 0])
    euler = Rotation.from_quat(bridge.sent_poses[0][3:]).as_euler("xyz")
    np.testing.assert_allclose(euler, [-2.7, 0.1, 0.15])


def test_serl_gripper_zero_sends_no_command(serl_clock):
    env, bridge = _make_serl_env()
    env.reset()
    serl_clock.now = 1.0
    env.step(torch.zeros(1, 7))
    assert bridge.gripper_calls == []


@pytest.mark.parametrize("action,width,elapsed,expected", [
    (-0.5, 1.0, 0.61, [False]), (0.5, 0.0, 0.61, [True]),
    (-0.49, 1.0, 0.61, []), (0.49, 0.0, 0.61, []),
    (-1.0, 0.85, 0.61, []), (1.0, 0.85, 0.61, []),
    (-1.0, 0.0, 0.61, []), (1.0, 1.0, 0.61, []),
    (-1.0, 1.0, 0.6, []), (1.0, 0.0, 0.6, []),
    (-1.0, 1.0, 0.0, []), (1.0, 0.0, 0.0, []),
])
def test_serl_gripper_threshold_width_and_time_boundaries(
    serl_clock, monkeypatch, action, width, elapsed, expected,
):
    env, bridge = _make_serl_env()
    read_state = bridge.get_state
    monkeypatch.setattr(bridge, "get_state", lambda: {**read_state(), "gripper_pos": width})
    env.reset()
    serl_clock.now = elapsed
    env.step([0, 0, 0, 0, 0, 0, action])
    assert bridge.gripper_calls == expected


def test_serl_gripper_sleep_precedes_pose_send(serl_clock, monkeypatch):
    env, bridge = _make_serl_env()
    env.reset()
    send_pose = bridge.send_pose
    get_state = bridge.get_state

    def record_pose(pose):
        serl_clock.events.append(("pose",))
        send_pose(pose)

    def read():
        serl_clock.events.append(("read",))
        return get_state()

    monkeypatch.setattr(bridge, "send_pose", record_pose)
    monkeypatch.setattr(bridge, "send_gripper", lambda command: serl_clock.events.append(("gripper", command)))
    monkeypatch.setattr(bridge, "get_state", read)
    serl_clock.now = 1.0
    env.step([0, 0, 0, 0, 0, 0, 0.5])
    assert serl_clock.events == [
        ("gripper", True), ("sleep", 0.6), ("pose",), ("sleep", 0.0), ("read",),
    ]
    assert env._last_gripper_act == 1.0


def test_serl_gripper_retries_when_measured_width_still_requires_it(serl_clock):
    env, bridge = _make_serl_env()
    env.reset()  # fake reports closed even after send_gripper(True)
    serl_clock.now = 1.0
    env.step([0, 0, 0, 0, 0, 0, 1])
    serl_clock.now += 1.0
    env.step([0, 0, 0, 0, 0, 0, 1])
    assert bridge.gripper_calls == [True, True]


@pytest.mark.parametrize("hz", [5.0, 20.0])
def test_serl_hz_does_not_rescale_delta(serl_clock, hz):
    env, bridge = _make_serl_env(control_hz=hz)
    env.reset()
    env.step([1, 0, 0, 0, 0, 1, 0])
    assert bridge.sent_poses[0][0] == pytest.approx(0.52)
    np.testing.assert_allclose(
        Rotation.from_quat(bridge.sent_poses[0][3:]).as_rotvec(), [0, 0, 0.1], atol=1e-12,
    )
    assert serl_clock.events == [("sleep", 1.0 / hz)]


def test_serl_overrun_does_not_add_a_period(serl_clock, monkeypatch):
    env, bridge = _make_serl_env()
    env.reset()

    def slow_pose(pose):
        serl_clock.now += 0.2

    monkeypatch.setattr(bridge, "send_pose", slow_pose)
    env.step(np.zeros(7))
    assert serl_clock.events == [("sleep", 0.0)]


@pytest.mark.parametrize("missing", [
    "safety_box_low", "safety_box_high", "safety_box_rpy_low", "safety_box_rpy_high",
])
def test_serl_requires_complete_pose_limits(missing):
    with pytest.raises(ValueError, match="safety_box"):
        _make_serl_env(**{missing: None})


@pytest.mark.parametrize("overrides", [
    {"safety_box_low": (0, 0)}, {"safety_box_rpy_high": (1, 1)},
    {"safety_box_rpy_low": (4, -1, -1)}, {"safety_box_high": (float("inf"), 1, 1)},
    {"control_hz": 0}, {"control_hz": float("nan")},
    {"gripper_sleep_s": -1}, {"gripper_action_scale": -1},
    {"action_scale": (-0.1, 0.1)},
])
def test_serl_rejects_invalid_config(overrides):
    with pytest.raises(ValueError):
        _make_serl_env(**overrides)


def test_serl_without_gripper_needs_no_feedback(serl_clock, monkeypatch):
    env, bridge = _make_serl_env(gripper_in_action=False)
    read_state = bridge.get_state

    def without_gripper():
        state = read_state()
        del state["gripper_pos"]
        return state

    monkeypatch.setattr(bridge, "get_state", without_gripper)
    env.reset()
    obs, *_ = env.step(np.zeros(6))
    assert env.single_action_space.shape == (6,)
    assert obs["state"].shape == (1, 20)
    assert bridge.gripper_calls == []


def test_current_env_defaults_unchanged(monkeypatch):
    from real_world_rl.action_contract import get_env_action_contract
    import real_world_rl.envs.franka_real.env as env_module

    env, bridge = _make_env()
    assert env.cfg.cartesian_semantics == "current"
    assert env.cfg.gripper_action_scale == 1.0
    assert env.cfg.gripper_sleep_s == 0.6
    assert env.cfg.control_hz == 10.0
    assert get_env_action_contract(env) is None
    clock = _SerlClock()
    monkeypatch.setattr(env_module, "time", clock)
    env.reset()
    env.step(np.zeros(7))
    assert bridge.gripper_calls == [False]
    assert clock.events == []


def test_reset_calls_bridge_and_returns_batched_state_obs():
    env, bridge = _make_env()
    obs, info = env.reset()
    assert bridge.reset_calls == 1
    assert obs["state"].shape == (1, 20)
    assert obs["state"].dtype == torch.float32
    assert info == {}


def test_step_before_reset_raises():
    env, _ = _make_env()
    with pytest.raises(RuntimeError, match="reset"):
        env.step(torch.zeros(1, 7))


def test_step_scales_action_into_position_delta():
    env, bridge = _make_env(safety_box_low=(-10, -10, -10), safety_box_high=(10, 10, 10))
    env.reset()
    action = torch.zeros(1, 7)
    action[0, 0] = 1.0  # +x
    env.step(action)

    sent = bridge.sent_poses[0]
    # action_scale defaults to (0.02, 0.1); starting pose x = 0.5.
    assert sent[0] == pytest.approx(0.5 + 0.02)
    assert sent[1] == pytest.approx(0.0)
    assert sent[2] == pytest.approx(0.2)


def test_step_clips_target_position_to_safety_box():
    env, bridge = _make_env(safety_box_low=(0.0, -0.1, 0.0), safety_box_high=(0.51, 0.1, 0.3))
    env.reset()
    action = torch.zeros(1, 7)
    action[0, 0] = 1.0  # would push x to 0.52, past the 0.51 box limit
    env.step(action)

    assert bridge.sent_poses[0][0] == pytest.approx(0.51)


def test_step_composes_delta_rotation_with_current_orientation():
    env, bridge = _make_env(safety_box_low=(-10, -10, -10), safety_box_high=(10, 10, 10))
    env.reset()
    action = torch.zeros(1, 7)
    action[0, 5] = 1.0  # rotation about z, scaled by rot_scale=0.1
    env.step(action)

    sent_quat = bridge.sent_poses[0][3:7]
    expected = Rotation.from_rotvec([0.0, 0.0, 0.1]) * Rotation.from_quat([0.0, 0.0, 0.0, 1.0])
    np.testing.assert_allclose(sent_quat, expected.as_quat(), atol=1e-6)


def test_step_opens_and_closes_gripper_based_on_threshold():
    env, bridge = _make_env(gripper_threshold=0.5)
    env.reset()

    open_action = torch.zeros(1, 7)
    open_action[0, 6] = 0.9
    env.step(open_action)
    assert bridge.gripper_calls[-1] is True

    close_action = torch.zeros(1, 7)
    close_action[0, 6] = -0.9
    env.step(close_action)
    assert bridge.gripper_calls[-1] is False


def test_step_truncates_after_max_episode_steps():
    env, _ = _make_env(max_episode_steps=2)
    env.reset()
    action = torch.zeros(1, 7)
    _, _, _, truncated, _ = env.step(action)
    assert not bool(truncated[0])
    _, _, _, truncated, _ = env.step(action)
    assert bool(truncated[0])


def test_camera_keys_are_included_in_observation_via_capture_callable():
    capture_calls = []

    def _capture():
        capture_calls.append(1)
        return {"wrist": np.zeros((8, 8, 3), dtype=np.uint8)}

    env, _ = _make_env(
        camera_capture=_capture, camera_keys=("wrist",), camera_height=8, camera_width=8
    )
    obs, _ = env.reset()
    assert "wrist" in obs
    assert obs["wrist"].shape == (1, 8, 8, 3)
    assert obs["wrist"].dtype == torch.uint8
    assert len(capture_calls) == 1


def test_camera_keys_without_capture_callable_raises():
    with pytest.raises(ValueError, match="camera_capture"):
        _make_env(camera_keys=("wrist",))


def test_observation_and_action_spaces_are_batch_of_one():
    env, _ = _make_env()
    assert env.num_envs == 1
    assert env.action_space.shape == (1, 7)
    assert env.observation_space["state"].shape == (1, 20)


def test_step_does_not_clip_orientation_when_rpy_bounds_unset():
    env, bridge = _make_env(safety_box_low=(-10, -10, -10), safety_box_high=(10, 10, 10))
    env.reset()
    action = torch.zeros(1, 7)
    action[0, 3] = 1.0  # large rotation about x, scaled by rot_scale=0.1
    env.step(action)

    sent_quat = bridge.sent_poses[0][3:7]
    expected = Rotation.from_rotvec([0.1, 0.0, 0.0]) * Rotation.from_quat([0.0, 0.0, 0.0, 1.0])
    np.testing.assert_allclose(sent_quat, expected.as_quat(), atol=1e-6)


def test_safety_box_rpy_bounds_must_be_set_together():
    with pytest.raises(ValueError, match="safety_box_rpy"):
        _make_env(safety_box_rpy_low=(-0.1, -0.1, -0.1))
    with pytest.raises(ValueError, match="safety_box_rpy"):
        _make_env(safety_box_rpy_high=(0.1, 0.1, 0.1))


def test_step_clips_target_orientation_to_safety_box_rpy():
    env, bridge = _make_env(
        safety_box_low=(-10, -10, -10),
        safety_box_high=(10, 10, 10),
        safety_box_rpy_low=(-0.2, -0.2, -0.2),
        safety_box_rpy_high=(0.2, 0.2, 0.2),
    )
    env.reset()
    action = torch.zeros(1, 7)
    action[0, 5] = 1.0  # rotation about z scaled by rot_scale=0.1 -> within bounds
    env.step(action)
    sent_quat = bridge.sent_poses[0][3:7]
    euler = Rotation.from_quat(sent_quat).as_euler("xyz")
    np.testing.assert_allclose(euler, [0.0, 0.0, 0.1], atol=1e-6)

    # A rotation well past the 0.2 rad bound gets clipped, sign preserved --
    # exercises the same "clip by magnitude" formula HIL-SERL's
    # clip_safety_box() uses for the discontinuous first Euler angle. Fresh
    # env/bridge so this delta composes with an identity current orientation
    # (Euler decomposition of a compound rotation on top of an already
    # z-rotated pose wouldn't isolate cleanly to one axis).
    env2, bridge2 = _make_env(
        safety_box_low=(-10, -10, -10),
        safety_box_high=(10, 10, 10),
        safety_box_rpy_low=(-0.2, -0.2, -0.2),
        safety_box_rpy_high=(0.2, 0.2, 0.2),
    )
    env2.reset()
    action = torch.zeros(1, 7)
    action[0, 3] = 10.0  # would rotate ~1.0 rad about x, well past the bound
    env2.step(action)
    sent_quat = bridge2.sent_poses[0][3:7]
    euler = Rotation.from_quat(sent_quat).as_euler("xyz")
    assert euler[0] == pytest.approx(0.2, abs=1e-6)


def test_last_tcp_pose_is_none_before_reset_then_a_copy_of_the_measured_pose():
    env, bridge = _make_env(safety_box_low=(-10, -10, -10), safety_box_high=(10, 10, 10))
    assert env.last_tcp_pose is None

    env.reset()
    np.testing.assert_allclose(env.last_tcp_pose, [0.5, 0.0, 0.2, 0.0, 0.0, 0.0, 1.0])
    assert env.last_tcp_pose.dtype == np.float64

    action = torch.zeros(1, 7)
    action[0, 1] = 1.0  # +y
    env.step(action)
    np.testing.assert_allclose(env.last_tcp_pose, bridge.sent_poses[-1])

    pose = env.last_tcp_pose
    pose[:] = 0.0  # a caller mutating its copy leaves the env's step origin alone
    assert env.last_tcp_pose[1] == pytest.approx(0.02)


def test_close_stops_only_owned_control_and_is_idempotent():
    env, bridge = _make_env()
    env.close()
    assert bridge.stop_calls == 0
    env, bridge = _make_env()
    env.reset()
    env.close()
    env.close()
    assert bridge.stop_calls == 1
    with pytest.raises(RuntimeError, match="closed"):
        env.reset()


def test_failed_reset_retains_control_cleanup_responsibility(monkeypatch):
    env, bridge = _make_env()
    def fail_reset():
        raise TimeoutError("reset acknowledgement lost")
    monkeypatch.setattr(bridge, "reset_joints", fail_reset)
    with pytest.raises(TimeoutError):
        env.reset()
    env.close()
    assert bridge.stop_calls == 1


def test_bridge_command_text_acknowledgements_and_slow_reset_timeout(monkeypatch):
    import requests
    calls = []
    class Response:
        content = b"Reset Joint"
        def raise_for_status(self):
            pass
        def json(self):
            raise ValueError("plain text acknowledgement")
    def post(url, **kwargs):
        calls.append((url, kwargs))
        return Response()
    monkeypatch.setattr(requests, "post", post)
    client = FrankaBridgeClient("http://unused", timeout=5)
    client.reset_joints()
    client.stop_impedance()
    assert calls[0][1]["timeout"] == 90
    assert calls[1][0].endswith("/stopimp")
    with pytest.raises(ValueError):
        client.get_state()
