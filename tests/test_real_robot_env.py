"""RealRobotEnv over device-layer fake devices, both local and through RemoteRobot + LocalClient."""
from __future__ import annotations

import time

import numpy as np
import pytest

import real_world_rl.devices.backends  # noqa: F401  (registers the fake backends)
import real_world_rl.devices.claims as claims
from real_world_rl.devices import CameraConfig, DeviceFault, GripperConfig
from real_world_rl.devices.robots.franka import FrankaConfig, FrankaRobot
from real_world_rl.server import RemoteRobot
from real_world_rl.server.service import RobotService
from real_world_rl.server.transport import LocalClient

from real_world_rl.envs.real_robot import RealRobotEnv, RealRobotEnvConfig, RuntimeReportPaths

ZERO_HOME = [0.0] * 7
FRAME_HW = (24, 32)


def _fake_franka(**robot_overrides) -> FrankaRobot:
    return FrankaRobot(
        FrankaConfig(
            arm_backend="fake",
            gripper=GripperConfig(backend="fake"),
            cameras=[CameraConfig(name="wrist", backend="fake", width=32, height=24, fps=60)],
            **robot_overrides,
        )
    )


@pytest.fixture(autouse=True)
def _claim_dir(tmp_path, monkeypatch):
    monkeypatch.setattr(claims, "_DEFAULT_LOCK_DIR", tmp_path / "claims")


class _Rig:
    """A robot to inject plus the fake arm/gripper that actually execute its commands."""

    def __init__(self, mode: str, **robot_overrides) -> None:
        self.mode = mode
        self.service = None
        if mode == "local":
            self.robot = _fake_franka(**robot_overrides)
            self.device_robot = self.robot
        else:
            self.service = RobotService(_fake_franka(**robot_overrides), watchdog_s=30.0)
            self.robot = RemoteRobot.from_client(LocalClient(self.service))
            self.device_robot = self.service.robot
        self.envs: list[RealRobotEnv] = []

    @property
    def arm(self):
        return self.device_robot.arm

    def config(self, **overrides) -> RealRobotEnvConfig:
        values = dict(cameras=["wrist"], home_joints=ZERO_HOME, reset_settle_s=0.0)
        if self.mode == "local":
            values["robot"] = {"type": "franka", "arm_backend": "fake"}
        else:
            values.update(remote="local", image_size=FRAME_HW)
        values.update(overrides)
        return RealRobotEnvConfig(**values)

    def env(self, *, report_paths=None, **overrides) -> RealRobotEnv:
        env = RealRobotEnv(self.config(**overrides), robot=self.robot, report_paths=report_paths)
        self.envs.append(env)
        return env

    def spy_commands(self, monkeypatch) -> list[dict]:
        """Record every action dict the env sends to the robot."""
        sent: list[dict] = []
        name = "send_action" if self.mode == "local" else "step"
        original = getattr(self.robot, name)

        def spy(action):
            sent.append(action)
            return original(action)

        monkeypatch.setattr(self.robot, name, spy)
        return sent

    def close(self) -> None:
        for env in self.envs:
            env.close()
        if self.robot.is_connected:
            self.robot.disconnect()
        if self.service is not None:
            self.service.close()


@pytest.fixture(params=["local", "remote"])
def rig(request):
    rig = _Rig(request.param)
    yield rig
    rig.close()


def _is_subsequence(expected: list[str], log: list[str]) -> bool:
    it = iter(log)
    return all(item in it for item in expected)


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
    import real_world_rl.envs.real_robot.env as env_module

    clock = _SerlClock()
    monkeypatch.setattr(env_module, "time", clock)
    return clock


def _serl_env(rig, **overrides):
    values = dict(cartesian_semantics="serl", safety_box=[-1] * 3 + [1] * 3,
                  safety_box_rpy_low=[0, -np.pi, -np.pi], safety_box_rpy_high=[np.pi] * 3)
    values.update(overrides)
    return rig.env(**values)


def test_serl_target_uses_latest_measured_pose(rig, serl_clock, monkeypatch):
    env = _serl_env(rig)
    env.reset()
    measured = np.array([0.3, 0, 0.2, 0, 0, 0, 1.0])
    env._last_raw["arm"]["tcp_pose"] = measured.copy()
    sent = []
    send = rig.robot.send_action

    def record(command):
        sent.append(command["arm"]["tcp_pose"].copy())
        send(command)

    monkeypatch.setattr(rig.robot, "send_action", record)
    action = np.array([2, 0, 0, 0, 0, 0, 0])
    env.step(action)
    assert sent[0][0] == pytest.approx(0.32)  # action clipped to one
    feedback = float(env._last_raw["arm"]["tcp_pose"][0])
    assert feedback != pytest.approx(sent[0][0])
    env.step(action)
    assert sent[1][0] == pytest.approx(feedback + 0.02)


def test_serl_rotation_left_multiplies_and_clips_roll_magnitude(rig, serl_clock, monkeypatch):
    from scipy.spatial.transform import Rotation

    env = _serl_env(rig, safety_box_rpy_low=[2.5, -0.1, -0.15],
                    safety_box_rpy_high=[2.7, 0.1, 0.15])
    env.reset()
    env._last_raw["arm"]["tcp_pose"] = np.r_[
        [0, 0, 0], Rotation.from_euler("xyz", [-2.9, 0.2, 0.3]).as_quat()
    ]
    sent = []
    monkeypatch.setattr(rig.robot, "send_action", lambda command: sent.append(command))
    env.step([0, 0, 0, 0.2, -0.4, 0.6, 0])
    euler = Rotation.from_quat(sent[0]["arm"]["tcp_pose"][3:]).as_euler("xyz")
    np.testing.assert_allclose(euler, [-2.7, 0.1, 0.15])


def test_serl_gripper_zero_sends_no_command(rig, serl_clock):
    env = _serl_env(rig)
    env.reset()
    rig.device_robot.gripper.call_log.clear()
    serl_clock.now = 1.0
    env.step(np.zeros(7))
    assert rig.device_robot.gripper.call_log == []


def test_serl_gripper_sleep_precedes_pose_send(rig, serl_clock, monkeypatch):
    env = _serl_env(rig)
    env.reset()
    send = rig.robot.send_action
    grip = rig.robot.gripper.command_binary
    observe = rig.robot.get_observation

    def send_pose(command):
        serl_clock.events.append(("pose", command))
        send(command)

    def send_gripper(open, *, force):
        serl_clock.events.append(("gripper", open, force))
        grip(open, force=force)

    def read():
        serl_clock.events.append(("read",))
        return observe()

    monkeypatch.setattr(rig.robot, "send_action", send_pose)
    monkeypatch.setattr(rig.robot.gripper, "command_binary", send_gripper)
    monkeypatch.setattr(rig.robot, "get_observation", read)
    serl_clock.now = 1.0
    serl_clock.events.clear()
    env.step([0, 0, 0, 0, 0, 0, -0.5])
    events = serl_clock.events
    assert events[0] == ("gripper", False, True)
    assert events[1] == ("sleep", 0.6)
    assert events[2][0] == "pose"
    assert "gripper" not in events[2][1]
    assert events[3:] == [("sleep", 0.0), ("read",)]
    assert env._last_gripper_act == 1.0


def test_serl_gripper_retries_when_measured_width_still_requires_it(rig, serl_clock, monkeypatch):
    env = _serl_env(rig)
    env.reset()
    # A stuck physical gripper keeps reporting open despite repeated close commands.
    gripper = rig.device_robot.gripper
    monkeypatch.setattr(gripper, "close", lambda: gripper.call_log.append("close"))
    gripper.call_log.clear()
    serl_clock.now = 1.0
    env.step([0, 0, 0, 0, 0, 0, -1])
    serl_clock.now += 1.0
    env.step([0, 0, 0, 0, 0, 0, -1])
    assert gripper.call_log == ["close", "close"]


def test_serl_gripper_initial_cooldown(rig, serl_clock):
    env = _serl_env(rig)
    env.reset()
    gripper = rig.device_robot.gripper
    gripper.call_log.clear()
    serl_clock.now = 0.6
    env.step([0, 0, 0, 0, 0, 0, -1])
    assert gripper.call_log == []
    serl_clock.now = 0.71
    env.step([0, 0, 0, 0, 0, 0, -1])
    assert gripper.call_log == ["close"]


@pytest.mark.parametrize("hz", [5.0, 20.0])
def test_serl_hz_does_not_rescale_delta(rig, serl_clock, monkeypatch, hz):
    from scipy.spatial.transform import Rotation

    env = _serl_env(rig, control_hz=hz)
    env.reset()
    before = env._last_raw["arm"]["tcp_pose"].copy()
    sent = []
    monkeypatch.setattr(rig.robot, "send_action", lambda command: sent.append(command))
    serl_clock.events.clear()
    env.step([1, 0, 0, 0, 0, 1, 0])
    assert sent[0]["arm"]["tcp_pose"][0] - before[0] == pytest.approx(0.02)
    np.testing.assert_allclose(
        Rotation.from_quat(sent[0]["arm"]["tcp_pose"][3:]).as_rotvec(), [0, 0, 0.1], atol=1e-12,
    )
    assert serl_clock.events == [("sleep", 1.0 / hz)]


@pytest.mark.parametrize("missing", ["safety_box", "safety_box_rpy_low", "safety_box_rpy_high"])
def test_serl_requires_complete_pose_limits(missing):
    values = dict(cartesian_semantics="serl", safety_box=[-1] * 3 + [1] * 3,
                  safety_box_rpy_low=[0, -1, -1], safety_box_rpy_high=[3, 1, 1])
    values[missing] = None
    with pytest.raises(ValueError, match="safety_box"):
        RealRobotEnvConfig(**values)


@pytest.mark.parametrize("overrides", [
    {"safety_box": [-1] * 3 + [1] * 2},
    {"safety_box_rpy_low": [0, 0]},
    {"safety_box_rpy_high": [3, float("nan"), 1]},
    {"safety_box_rpy_low": [4, -1, -1]},
    {"gripper_sleep_s": -1}, {"gripper_action_scale": float("inf")},
    {"action_mode": "joint_delta"},
])
def test_serl_rejects_invalid_config(overrides):
    values = dict(cartesian_semantics="serl", safety_box=[-1] * 3 + [1] * 3,
                  safety_box_rpy_low=[0, -1, -1], safety_box_rpy_high=[3, 1, 1])
    values.update(overrides)
    with pytest.raises(ValueError):
        RealRobotEnvConfig(**values)


def test_serl_without_gripper_needs_no_feedback(rig, serl_clock):
    env = _serl_env(rig, gripper_in_action=False, state_fields=("tcp_pose",))
    env.reset()
    env._last_raw.pop("gripper")
    env.step(np.zeros(6))


def test_current_env_defaults_unchanged(rig):
    env = rig.env()
    assert env.config.cartesian_semantics == "current"
    assert env.config.gripper_action_scale == 1.0
    assert env.config.gripper_sleep_s == 0.6
    assert env.config.safety_box_rpy_low is env.config.safety_box_rpy_high is None
    assert env.action_space.shape == (7,)
    assert "cartesian_semantics" not in env.requested_experiment
    assert "serl" not in env.requested_experiment
    assert "safety_box_rpy_low" not in env.requested_experiment


# -- config -------------------------------------------------------------------------


@pytest.mark.parametrize(
    "kwargs, match",
    [
        (dict(action_mode="velocity"), "action_mode"),
        (dict(num_joints=0), "num_joints"),
        (dict(joint_limits=[[-1.0, 1.0]] * 6), "joint_limits"),
        (dict(joint_limits=[[-1.0, 1.0, 2.0]] * 7), "joint_limits"),
        (dict(safety_box=[0.0] * 5), "safety_box"),
        (dict(state_fields=("tcp_pose", "ee_wrench")), "state_fields"),
        (dict(action_mode="joint_abs"), "joint_limits"),
        (dict(home_joints=[0.0] * 6), "home_joints"),
    ],
)
def test_config_validation_errors(kwargs, match):
    if "remote" not in kwargs and kwargs:
        kwargs = {"robot": {"type": "franka"}, **kwargs}
    with pytest.raises(ValueError, match=match):
        RealRobotEnvConfig(**kwargs)


def test_config_default_constructs_and_parses_from_empty_cli():
    import tyro

    assert RealRobotEnvConfig().robot is None
    assert tyro.cli(RealRobotEnvConfig, args=[]) == RealRobotEnvConfig()


@pytest.mark.parametrize('reports_enabled', [False, True])
def test_config_tyro_parse_and_serialization_exclude_runtime_paths(tmp_path, monkeypatch, reports_enabled):
    import dataclasses
    import json
    import tyro
    from rl_garden.common.effective_config import json_value

    destinations = {
        'RWRL_STOP_TRACE_PATH': str(tmp_path / 'stop.json'),
        'RWRL_DIAGNOSTICS_PATH': str(tmp_path / 'diagnostics.jsonl'),
        'RWRL_SESSION_EVENTS_PATH': str(tmp_path / 'session.jsonl'),
    }
    for variable, path in destinations.items():
        if reports_enabled:
            monkeypatch.setenv(variable, path)
        else:
            monkeypatch.delenv(variable, raising=False)
    config = tyro.cli(RealRobotEnvConfig, args=[])
    assert config == RealRobotEnvConfig()
    runtime_keys = {'stop_trace_path', 'diagnostics_path', 'session_events_path',
                    'stop_trace', 'diagnostics', 'session_events', 'report_paths'}
    assert not runtime_keys & config.__annotations__.keys()
    assert not any(hasattr(config, name) for name in runtime_keys)
    for serialized in (dataclasses.asdict(config), json_value(config)):
        assert not runtime_keys & serialized.keys()
        assert not any(path in json.dumps(serialized) for path in destinations.values())


@pytest.mark.parametrize('reports_enabled', [False, True])
def test_runtime_report_paths_from_environ_and_explicit_override(tmp_path, monkeypatch, reports_enabled):
    values = {name: str(tmp_path / name) for name in ('stop_trace', 'diagnostics', 'session_events')}
    for name, path in values.items():
        variable = f'RWRL_{name.upper()}_PATH'
        if reports_enabled:
            monkeypatch.setenv(variable, path)
        else:
            monkeypatch.delenv(variable, raising=False)
    expected = RuntimeReportPaths(**values) if reports_enabled else RuntimeReportPaths()
    assert RuntimeReportPaths.from_environ() == expected
    rig = _Rig('local')
    try:
        assert rig.env()._report_paths == expected
        explicit = RuntimeReportPaths(diagnostics=str(tmp_path / 'explicit.jsonl'))
        assert rig.env(report_paths=explicit)._report_paths == explicit
        # An explicit empty value disables ambient destinations rather than inheriting them.
        assert rig.env(report_paths=RuntimeReportPaths())._report_paths == RuntimeReportPaths()
        assert not rig.robot.is_connected
    finally:
        rig.close()


@pytest.mark.parametrize(
    "kwargs", [dict(), dict(robot={"type": "franka"}, remote="host:1")]
)
def test_env_requires_exactly_one_robot_source(kwargs):
    with pytest.raises(ValueError, match="exactly one"):
        RealRobotEnv(RealRobotEnvConfig(**kwargs), robot=object())


def test_config_remote_with_cameras_requires_image_size():
    with pytest.raises(ValueError, match="image_size"):
        RealRobotEnvConfig(remote="host:1", cameras=["wrist"])
    RealRobotEnvConfig(remote="host:1", cameras=["wrist"], image_size=(16, 16))


# -- spaces -------------------------------------------------------------------------


def test_spaces_for_seven_joint_cartesian(rig):
    env = rig.env()
    assert env.action_space.shape == (7,)
    assert env.action_space.dtype == np.float32
    state = env.observation_space["state"]
    assert list(state.spaces) == list(env.config.state_fields)
    assert state["tcp_pose"].shape == (7,)
    assert state["joint_pos"].shape == (7,)
    assert state["gripper_pos"].shape == (1,)
    assert state["tcp_pose"].dtype == np.float32
    assert np.all(np.isinf(state["tcp_pose"].high))
    frame = env.observation_space["frames"]["wrist"]
    assert frame.shape == (*FRAME_HW, 3) and frame.dtype == np.uint8


def test_spaces_for_joint_delta_and_no_gripper(rig):
    assert rig.env(action_mode="joint_delta").action_space.shape == (8,)
    assert rig.env(gripper_in_action=False).action_space.shape == (6,)


def test_construction_does_not_connect(rig):
    rig.env()
    assert not rig.robot.is_connected


# -- reset ----------------------------------------------------------------------------


@pytest.mark.parametrize(
    "action_mode, servo", [("cartesian_delta", "enter:cartesian_servo"), ("joint_delta", "enter:joint_servo")]
)
def test_reset_idles_homes_opens_and_enters_servo(rig, action_mode, servo):
    env = rig.env(action_mode=action_mode)
    env.reset()
    env.step(np.zeros(env.action_space.shape, dtype=np.float32))
    rig.arm.call_log.clear()
    rig.device_robot.gripper.close()

    obs, info = env.reset()

    assert _is_subsequence(["stop", "enter:idle", "move_to_joints", servo], rig.arm.call_log)
    assert rig.arm.call_log.index("move_to_joints") < rig.arm.call_log.index(servo)
    assert rig.arm.control_mode.value == servo.split(":")[1]
    assert rig.device_robot.gripper.width == 1.0
    np.testing.assert_allclose(obs["state"]["joint_pos"], ZERO_HOME)
    assert info == {}


def test_reset_rejects_joint_count_mismatch_with_hint(rig):
    env = rig.env(num_joints=6, home_joints=None)
    with pytest.raises(DeviceFault) as excinfo:
        env.reset()
    assert "num_joints=7" in excinfo.value.hint
    assert not rig.robot.is_connected


def test_reset_rejects_unknown_camera_with_hint(rig):
    env = rig.env(cameras=["front"], image_size=FRAME_HW)
    with pytest.raises(DeviceFault, match="front") as excinfo:
        env.reset()
    assert excinfo.value.hint


def test_reset_without_any_home_joints_raises_hint():
    service = RobotService(_fake_franka(home_joints=None), watchdog_s=30.0)
    robot = RemoteRobot.from_client(LocalClient(service))
    env = RealRobotEnv(
        RealRobotEnvConfig(remote="local", reset_settle_s=0.0), robot=robot
    )
    try:
        with pytest.raises(DeviceFault) as excinfo:
            env.reset()
        assert "home_joints" in excinfo.value.hint
    finally:
        env.close()
        service.close()


def test_local_reset_falls_back_to_robot_home_joints():
    robot = _fake_franka()
    env = RealRobotEnv(
        RealRobotEnvConfig(robot={"type": "franka"}, reset_settle_s=0.0), robot=robot
    )
    try:
        obs, _ = env.reset()
        np.testing.assert_allclose(obs["state"]["joint_pos"], robot.config.home_joints, atol=1e-6)
    finally:
        env.close()


# -- step -------------------------------------------------------------------------


def test_cartesian_step_target(rig, monkeypatch):
    env = rig.env()
    env.reset()
    sent = rig.spy_commands(monkeypatch)

    env.step(np.array([1, 0, 0, 0, 0, 0, 1], dtype=np.float32))

    command = sent[-1]
    pose = np.asarray(command["arm"]["tcp_pose"])
    np.testing.assert_allclose(pose[:3], [env.config.xyz_scale_m, 0.0, 0.0])
    np.testing.assert_allclose(pose[3:], [0.0, 0.0, 0.0, 1.0], atol=1e-12)
    assert float(np.asarray(command["gripper"]["gripper"])[0]) == 1.0
    np.testing.assert_allclose(rig.arm._pose_target[:3], [env.config.xyz_scale_m, 0.0, 0.0])


def test_cartesian_rotation_composes_delta_on_current(rig, monkeypatch):
    from scipy.spatial.transform import Rotation

    env = rig.env(rot_scale_rad=0.5)
    env.reset()
    sent = rig.spy_commands(monkeypatch)
    env.step(np.array([0, 0, 1, 0, 0, 1, -1], dtype=np.float32))
    pose = np.asarray(sent[-1]["arm"]["tcp_pose"])
    expected = Rotation.from_rotvec([0, 0, 0.5]).as_quat()
    np.testing.assert_allclose(pose[3:], expected, atol=1e-9)
    assert float(np.asarray(sent[-1]["gripper"]["gripper"])[0]) == 0.0


def test_cartesian_target_clipped_to_safety_box(rig, monkeypatch):
    env = rig.env(safety_box=[-0.005, -1.0, -1.0, 0.005, 1.0, 0.01])
    env.reset()
    sent = rig.spy_commands(monkeypatch)
    env.step(np.array([1, 0, 1, 0, 0, 0, 1], dtype=np.float32))
    np.testing.assert_allclose(np.asarray(sent[-1]["arm"]["tcp_pose"])[:3], [0.005, 0.0, 0.01])
    env.step(np.array([-1, 0, 0, 0, 0, 0, 1], dtype=np.float32))
    assert np.asarray(sent[-1]["arm"]["tcp_pose"])[0] == pytest.approx(-0.005)


def test_out_of_range_action_is_clipped(rig, monkeypatch):
    env = rig.env()
    env.reset()
    sent = rig.spy_commands(monkeypatch)
    env.step(np.array([5, 0, 0, 0, 0, 0, 3], dtype=np.float32))
    assert np.asarray(sent[-1]["arm"]["tcp_pose"])[0] == pytest.approx(env.config.xyz_scale_m)
    assert float(np.asarray(sent[-1]["gripper"]["gripper"])[0]) == 1.0


def test_joint_delta_clipped_to_joint_limits(rig, monkeypatch):
    env = rig.env(action_mode="joint_delta", joint_limits=[[-0.01, 0.01]] * 7)
    env.reset()
    sent = rig.spy_commands(monkeypatch)
    env.step(np.array([1, -1, 0.1, 0, 0, 0, 0, 1], dtype=np.float32))
    q = np.asarray(sent[-1]["arm"]["joint_pos"])
    np.testing.assert_allclose(q, [0.01, -0.01, 0.005, 0, 0, 0, 0])
    np.testing.assert_allclose(rig.arm._joint_target, q)


def test_joint_delta_without_limits_is_unclipped(rig, monkeypatch):
    env = rig.env(action_mode="joint_delta")
    env.reset()
    sent = rig.spy_commands(monkeypatch)
    env.step(np.ones(8, dtype=np.float32))
    np.testing.assert_allclose(np.asarray(sent[-1]["arm"]["joint_pos"]), [env.config.joint_scale_rad] * 7)


@pytest.fixture(params=["local", "remote"])
def step_limited_rig(request):
    """A robot whose yaml sets max_delta_per_step.joint_rad, as joint_abs requires."""
    rig = _Rig(request.param, max_delta_per_step={"joint_rad": 10.0})
    yield rig
    rig.close()


def test_joint_abs_maps_unit_range_onto_limits(step_limited_rig, monkeypatch):
    rig = step_limited_rig
    limits = [[-1.0, 1.0], [0.0, 2.0], [-2.0, -1.0], [0.0, 1.0], [0.0, 1.0], [0.0, 1.0], [0.0, 1.0]]
    env = rig.env(action_mode="joint_abs", joint_limits=limits)
    env.reset()
    sent = rig.spy_commands(monkeypatch)
    env.step(np.array([-1, 0, 1, 0.5, -0.5, 0, 0, 1], dtype=np.float32))
    np.testing.assert_allclose(
        np.asarray(sent[-1]["arm"]["joint_pos"]), [-1.0, 1.0, -1.0, 0.75, 0.25, 0.5, 0.5]
    )


def test_step_before_reset_raises(rig):
    with pytest.raises(RuntimeError, match="reset"):
        rig.env().step(np.zeros(7, dtype=np.float32))


def test_step_paces_to_control_hz(rig):
    env = rig.env(control_hz=50.0)
    env.reset()
    action = np.zeros(7, dtype=np.float32)
    start = time.monotonic()
    env.step(action)
    env.step(action)
    assert time.monotonic() - start >= 0.02


def test_truncated_at_max_episode_steps(rig):
    env = rig.env(max_episode_steps=2)
    env.reset()
    action = np.zeros(7, dtype=np.float32)
    _, reward, terminated, truncated, info = env.step(action)
    assert (reward, terminated, truncated, info["elapsed_steps"]) == (0.0, False, False, 1)
    _, _, _, truncated, info = env.step(action)
    assert truncated is True and info["elapsed_steps"] == 2
    env.reset()
    _, _, _, truncated, _ = env.step(action)
    assert truncated is False


def test_step_info_carries_tcp_pose_and_rtt(rig):
    env = rig.env()
    env.reset()
    _, _, _, _, info = env.step(np.zeros(7, dtype=np.float32))
    assert info["tcp_pose"].shape == (7,) and info["tcp_pose"].dtype == np.float32
    if rig.mode == "remote":
        assert isinstance(info["rtt_ms"], float)
    else:
        assert info["rtt_ms"] is None


def test_observation_dtypes_and_shapes(rig):
    env = rig.env()
    obs, _ = env.reset()
    assert env.observation_space.contains(obs)
    assert obs["state"]["gripper_pos"].tolist() == [1.0]
    obs, *_ = env.step(np.zeros(7, dtype=np.float32))
    assert env.observation_space.contains(obs)
    for field, value in obs["state"].items():
        assert value.dtype == np.float32, field
    assert obs["frames"]["wrist"].shape == (*FRAME_HW, 3)
    assert obs["frames"]["wrist"].dtype == np.uint8
    assert env.render() is obs["frames"]["wrist"]


def test_frames_resized_to_image_size(rig):
    pytest.importorskip("PIL")
    env = rig.env(image_size=(16, 16))
    obs, _ = env.reset()
    assert obs["frames"]["wrist"].shape == (16, 16, 3)
    assert obs["frames"]["wrist"].dtype == np.uint8
    assert env.observation_space.contains(obs)


def test_piper_style_state_fields(rig):
    env = rig.env(state_fields=("tcp_pose", "joint_pos", "joint_vel", "gripper_pos"))
    obs, _ = env.reset()
    assert list(obs["state"]) == ["tcp_pose", "joint_pos", "joint_vel", "gripper_pos"]


def test_close_disconnects_and_task_description_defaults_empty(rig):
    env = rig.env()
    assert env.task_description == ""
    assert env.render() is None
    env.reset()
    assert rig.robot.is_connected
    env.close()
    assert not rig.robot.is_connected
    env.close()


def test_last_observation_is_none_before_reset_and_dict_after(rig):
    env = rig.env()
    assert env.last_observation is None
    env.reset()
    assert set(env.last_observation) == {"state", "frames"}
    env.close()


# -- device-layer safety inheritance --------------------------------------------------

DEVICE_BOX = [-0.01, -0.01, -0.01, 0.01, 0.01, 0.01]
DEVICE_JOINT_LIMITS = [[-2.5, 2.5]] * 7
DEVICE_HOME = [0.1] * 7
CARTESIAN_PUSH = np.array([1, 0, 0, 0, 0, 0, 1], dtype=np.float32)  # +xyz_scale_m = 0.02 in x


@pytest.fixture(params=["local", "remote"])
def limited_rig(request):
    rig = _Rig(
        request.param,
        safety_box=DEVICE_BOX,
        joint_limits=DEVICE_JOINT_LIMITS,
        home_joints=DEVICE_HOME,
        max_delta_per_step={"joint_rad": 0.5},
    )
    yield rig
    rig.close()


def _first_target_x(rig, monkeypatch, **env_overrides) -> float:
    env = rig.env(**env_overrides)
    env.reset()
    sent = rig.spy_commands(monkeypatch)
    env.step(CARTESIAN_PUSH)
    return float(np.asarray(sent[-1]["arm"]["tcp_pose"])[0])


def test_unset_safety_box_is_inherited_from_device(limited_rig, monkeypatch):
    assert _first_target_x(limited_rig, monkeypatch) == pytest.approx(0.01)


def test_tighter_env_safety_box_wins(limited_rig, monkeypatch):
    tighter = [-0.005, -0.005, -0.005, 0.005, 0.005, 0.005]
    assert _first_target_x(limited_rig, monkeypatch, safety_box=tighter) == pytest.approx(0.005)


def test_looser_env_safety_box_faults_on_first_reset(limited_rig):
    env = limited_rig.env(safety_box=[-0.02, -0.01, -0.01, 0.02, 0.01, 0.01])
    with pytest.raises(DeviceFault) as excinfo:
        env.reset()
    assert "0.02" in str(excinfo.value) and "0.01" in str(excinfo.value)
    assert "tighten" in excinfo.value.hint
    assert not limited_rig.robot.is_connected


def test_unset_joint_limits_are_inherited_and_tighter_ones_win(limited_rig, monkeypatch):
    action = np.array([1, 0, 0, 0, 0, 0, 0, 1], dtype=np.float32)

    def first_joint_target(**env_overrides) -> float:
        env = limited_rig.env(action_mode="joint_delta", joint_scale_rad=5.0, **env_overrides)
        env.reset()
        sent = limited_rig.spy_commands(monkeypatch)
        env.step(action)
        return float(np.asarray(sent[-1]["arm"]["joint_pos"])[0])

    assert first_joint_target() == pytest.approx(2.5)
    assert first_joint_target(joint_limits=[[-1.0, 1.0]] * 7) == pytest.approx(1.0)


def test_looser_env_joint_limits_fault_on_first_reset(limited_rig):
    env = limited_rig.env(joint_limits=[[-3.0, 3.0]] * 7)
    with pytest.raises(DeviceFault, match="joint_limits"):
        env.reset()


def test_device_without_safety_leaves_env_config_unchecked(rig, monkeypatch):
    from real_world_rl.server.remote import RemoteArm

    arm_class = RemoteArm if rig.mode == "remote" else type(rig.arm)
    monkeypatch.setattr(arm_class, "safety_limits", property(lambda self: None))
    assert _first_target_x(rig, monkeypatch, safety_box=[-1.0, -1.0, -1.0, 1.0, 1.0, 1.0]) == (
        pytest.approx(0.02)
    )


def test_unset_home_joints_are_inherited_from_device(limited_rig):
    env = limited_rig.env(home_joints=None)
    obs, _ = env.reset()
    np.testing.assert_allclose(obs["state"]["joint_pos"], DEVICE_HOME, atol=1e-6)


def test_joint_abs_without_device_joint_step_limit_faults_with_hint(rig):
    env = rig.env(action_mode="joint_abs", joint_limits=[[-1.0, 1.0]] * 7)
    with pytest.raises(DeviceFault, match="max_delta_per_step.joint_rad") as excinfo:
        env.reset()
    assert "robot yaml" in excinfo.value.hint
    assert not rig.robot.is_connected


def test_joint_abs_with_device_unaware_of_safety_limits_faults(limited_rig, monkeypatch):
    from real_world_rl.server.remote import RemoteArm

    arm_class = RemoteArm if limited_rig.mode == "remote" else type(limited_rig.arm)
    monkeypatch.setattr(arm_class, "safety_limits", property(lambda self: None))  # older daemon
    env = limited_rig.env(action_mode="joint_abs", joint_limits=[[-1.0, 1.0]] * 7)
    with pytest.raises(DeviceFault, match="max_delta_per_step.joint_rad"):
        env.reset()


def test_joint_abs_with_device_joint_step_limit_passes(limited_rig):
    env = limited_rig.env(action_mode="joint_abs", joint_limits=[[-1.0, 1.0]] * 7)
    env.reset()


def test_joint_delta_does_not_need_device_joint_step_limit(rig):
    rig.env(action_mode="joint_delta").reset()


def test_step_info_carries_safety_clips(limited_rig):
    env = limited_rig.env()
    env.reset()
    _, _, _, _, info = env.step(CARTESIAN_PUSH)
    assert info["safety_clips"] == 0


# -- robot with a teleop part ---------------------------------------------------------


@pytest.fixture(params=["local", "remote"])
def teleop_rig(request):
    from real_world_rl.devices.teleop import TeleopConfig

    rig = _Rig(request.param, teleop=TeleopConfig(name="leader", backend="fake"))
    yield rig
    rig.close()


def test_teleop_part_does_not_leak_into_env_observation(teleop_rig):
    env = teleop_rig.env()
    obs, _ = env.reset()
    assert "teleop" in teleop_rig.robot.get_observation()
    obs, *_ = env.step(np.zeros(env.action_space.shape, dtype=np.float32))
    assert set(obs) == {"state", "frames"}
    assert list(obs["state"]) == list(env.config.state_fields)
    assert set(obs["frames"]) == {"wrist"}


@pytest.mark.parametrize('mode', ['local', 'remote'])
def test_workspace_contract_requires_matching_enforcement_before_reset(mode):
    workspace = {'frame': 'base', 'low': [-2., -2., -2.], 'high': [2., 2., 2.]}
    rig = _Rig(mode, workspace=workspace)
    try:
        env = rig.env(workspace=workspace)
        assert 'spatial_constraints' in env.requested_experiment
        env.reset()
        assert 'task_workspace_v1' in env.robot.arm.safety_capabilities
    finally:
        rig.close()
    rig = _Rig(mode, workspace=workspace)
    try:
        wrong = {**workspace, 'high': [3., 3., 3.]}
        env = rig.env(workspace=wrong)
        with pytest.raises(DeviceFault, match='differs'):
            env.reset()
        assert not env._validated
    finally:
        rig.close()


def test_old_daemon_cannot_claim_workspace_support():
    workspace = {'frame': 'base', 'low': [-2.]*3, 'high': [2.]*3}
    rig = _Rig('remote', workspace=workspace)
    original = rig.service._methods['robot.describe']
    def old_describe():
        result = original()
        result['arm'].pop('safety_capabilities')
        return result
    rig.service._methods['robot.describe'] = old_describe
    try:
        env = rig.env(workspace=workspace)
        with pytest.raises(DeviceFault, match='does not advertise'):
            env.reset()
    finally:
        rig.close()


@pytest.mark.parametrize('mode', ['local', 'remote'])
@pytest.mark.parametrize('semantics', ['current', 'serl'])
def test_measured_workspace_violation_stops_without_recovery_command(mode, semantics):
    from real_world_rl.devices.errors import SafetyViolation
    workspace = {'frame': 'base', 'low': [-1.]*3, 'high': [1.]*3}
    rig = _Rig(mode, workspace=workspace)
    try:
        options = dict(workspace=workspace, cartesian_semantics=semantics)
        if semantics == 'serl':
            options.update(safety_box=[-1.]*3+[1.]*3,
                           safety_box_rpy_low=[-3.14]*3, safety_box_rpy_high=[3.14]*3)
        env = rig.env(**options)
        env.reset()
        rig.arm._tcp_pose[0] = 2.
        rig.arm._pose_target = None
        env._last_raw['arm']['tcp_pose'][0] = 2.
        before = list(rig.arm.call_log)
        with pytest.raises(SafetyViolation, match='measured TCP'):
            env.step(np.zeros(7))
        assert rig.arm.call_log[len(before):] == ['stop']
        with pytest.raises(SafetyViolation, match='measured TCP'):
            env.reset()
        assert rig.arm.call_log[len(before):] == ['stop']
    finally:
        rig.close()


def test_joint_mode_refuses_cartesian_workspace_before_connect():
    workspace = {'frame': 'base', 'low': [-1.]*3, 'high': [1.]*3}
    rig = _Rig('local', workspace=workspace)
    try:
        with pytest.raises(ValueError, match='requires cartesian_delta'):
            rig.env(workspace=workspace, action_mode='joint_delta')
        assert not rig.robot.is_connected
    finally:
        rig.close()


def test_measured_workspace_stop_failure_preserves_violation(monkeypatch):
    from real_world_rl.devices.errors import SafetyViolation
    workspace = {'frame': 'base', 'low': [-1.]*3, 'high': [1.]*3}
    rig = _Rig('local', workspace=workspace)
    try:
        env = rig.env(workspace=workspace)
        env.reset()
        rig.arm._tcp_pose[2] = 2.
        rig.arm._pose_target = None
        env._last_raw['arm']['tcp_pose'][2] = 2.
        def fail_stop():
            raise RuntimeError('stop failed independently')
        with monkeypatch.context() as patch:
            patch.setattr(rig.arm, 'stop', fail_stop)
            with pytest.raises(SafetyViolation, match='measured TCP'):
                env.step(np.zeros(7))
    finally:
        rig.close()


def test_feedback_drift_after_send_is_detected(monkeypatch):
    from real_world_rl.devices.errors import SafetyViolation
    workspace = {'frame': 'base', 'low': [-1.]*3, 'high': [1.]*3}
    rig = _Rig('local', workspace=workspace)
    try:
        env = rig.env(workspace=workspace)
        env.reset()
        original = rig.robot.get_observation
        def drifted():
            raw = original()
            raw['arm']['tcp_pose'][0] = 2.
            return raw
        monkeypatch.setattr(rig.robot, 'get_observation', drifted)
        before = len(rig.arm.call_log)
        with pytest.raises(SafetyViolation, match='measured TCP'):
            env.step(np.zeros(7))
        assert rig.arm.call_log[before:] == ['servo_cartesian', 'stop']
    finally:
        rig.close()


@pytest.mark.parametrize("reason", [None, "operator_quit"])
def test_close_exports_normal_stop_before_releasing_connection(tmp_path, monkeypatch, reason):
    import json
    rig = _Rig('local')
    try:
        path = tmp_path / 'trace.json'
        report_paths = RuntimeReportPaths()
        env = rig.env(report_paths=report_paths)
        env.reset()
        report_paths.stop_trace = str(path)
        path = env.output_path('stop_trace')
        trace = {'schema_version': 1, 'samples': []}
        def stop_trace():
            assert rig.robot.is_connected
            return {'trace': trace, 'assessment': {'status': 'BLOCKED'}}
        monkeypatch.setattr(rig.arm, 'stop_with_trace', stop_trace, raising=False)
        if reason:
            env.session_event('STOPPING', reason)
        env.close()
        assert not rig.robot.is_connected
        assert json.loads(path.read_text()) == trace
        context = json.loads(path.read_text())['metadata']['session_exit']
        assert context['reason'] == (reason or 'normal_close')
        assert len(context['env_config_sha256']) == 64
        assert json.loads(path.with_name(path.stem + '.assessment.json').read_text())['status'] == 'BLOCKED'
    finally:
        rig.close()


def test_exception_close_does_not_wait_for_normal_stop_trace(monkeypatch):
    rig = _Rig('local')
    try:
        report_paths = RuntimeReportPaths()
        env = rig.env(report_paths=report_paths)
        env.reset()
        report_paths.stop_trace = 'unused.json'
        def forbidden():
            raise AssertionError('fault cleanup must not perform normal observation')
        monkeypatch.setattr(rig.arm, 'stop_with_trace', forbidden, raising=False)
        try:
            raise ValueError('rollout fault')
        except ValueError:
            env.close()
        assert not rig.robot.is_connected
    finally:
        rig.close()


def test_failed_stop_acceptance_exports_evidence_and_releases(tmp_path, monkeypatch):
    rig = _Rig('local')
    try:
        report_paths = RuntimeReportPaths()
        env = rig.env(report_paths=report_paths)
        env.reset()
        path = tmp_path / 'stop.json'
        report_paths.stop_trace = str(path)
        path = env.output_path('stop_trace')
        monkeypatch.setattr(rig.arm, 'stop_with_trace', lambda: {
            'trace': {'samples': []}, 'assessment': {'status': 'FAIL'}}, raising=False)
        with pytest.raises(RuntimeError, match='acceptance failed'):
            env.close()
        assert path.exists()
        assert not rig.robot.is_connected
    finally:
        rig.close()


@pytest.mark.parametrize('mode', ['local', 'remote'])
def test_control_diagnostics_distinguish_requested_limited_accepted_and_measured(mode, tmp_path):
    import json
    rig = _Rig(mode, max_delta_per_step={'xyz_m': .01})
    try:
        path = tmp_path / 'diagnostics.jsonl'
        env = rig.env(report_paths=RuntimeReportPaths(diagnostics=str(path)), safety_box=[-.015]*3+[.015]*3,
                      xyz_scale_m=.02)
        path = env.output_path('diagnostics')
        env.reset()
        env.step(np.array([1., 0, 0, 0, 0, 0, 0]))
        row = json.loads(path.read_text().splitlines()[-1])
        assert row['requested_pose'][0] == pytest.approx(.02)
        assert row['env_limited_pose'][0] == pytest.approx(.015)
        assert row['device_accepted_pose'][0] == pytest.approx(.01)
        assert row['measured_base_pose'][0] < row['device_accepted_pose'][0]
        assert row['limit_reasons'] == ['env_pose_limits', 'device_limits']
    finally:
        rig.close()


def test_teleop_evidence_attaches_once_to_control_row(tmp_path):
    import json
    rig = _Rig('local')
    try:
        path = tmp_path / 'diagnostics.jsonl'
        env = rig.env(report_paths=RuntimeReportPaths(diagnostics=str(path)))
        path = env.output_path('diagnostics')
        env.reset()
        data = {'input': {'sample_seq': 42, 'decoded_axes': [1, 2, 3, 4, 5, 6]}, 'intervened': True}
        env.set_teleop_diagnostics(data)
        data['input']['sample_seq'] = 99
        env.step(np.zeros(7))
        env.step(np.zeros(7))
        rows = [json.loads(line) for line in path.read_text().splitlines()]
        assert rows[0]['teleop']['input']['sample_seq'] == 42
        assert rows[1]['teleop'] is None
    finally:
        rig.close()


@pytest.mark.parametrize('active_error', [False, True])
def test_close_releases_even_when_watchdog_and_stream_close_fail(monkeypatch, active_error):
    from types import SimpleNamespace
    rig = _Rig('local')
    try:
        env = rig.env()
        env.reset()
        calls = []
        def watchdog_close():
            raise RuntimeError('watchdog cleanup fault')
        def stream_close():
            calls.append('stream')
            raise OSError('stream cleanup fault')
        monkeypatch.setattr(env, '_close_step_watchdog', watchdog_close)
        env._diagnostic_stream = SimpleNamespace(close=stream_close)
        if active_error:
            with pytest.raises(ValueError, match='original rollout'):
                try:
                    raise ValueError('original rollout')
                finally:
                    env.close()
        else:
            with pytest.raises(RuntimeError, match='watchdog cleanup fault'):
                env.close()
        assert not rig.robot.is_connected
        assert calls == ['stream']
        assert env._diagnostic_stream is None
    finally:
        monkeypatch.undo()
        rig.close()


@pytest.mark.parametrize('mode', ['local', 'remote'])
def test_hold_session_capability_rejected_before_homing(mode, monkeypatch):
    from real_world_rl.teleop.session import session_backend
    from real_world_rl.devices.errors import DeviceFault
    rig = _Rig(mode)
    try:
        env = rig.env()
        session_backend(env)
        monkeypatch.setattr(type(rig.arm), 'safety_capabilities', property(lambda self: ()))
        with pytest.raises(DeviceFault, match='accepted_cartesian_target_v1'):
            env.reset()
        assert 'move_to_joints' not in rig.arm.call_log
        assert 'servo_cartesian' not in rig.arm.call_log
        assert not env.robot.is_connected
    finally:
        rig.close()


def test_spacemouse_from_env_inherits_task_workspace_without_connecting():
    from real_world_rl.teleop.spacemouse import SpaceMouseTeleopSource
    rig = _Rig('local')
    try:
        frame = {'name': 'table', 'base_T_frame': np.eye(4).tolist()}
        workspace = {'frame': 'table', 'low': [-1]*3, 'high': [1]*3}
        env = rig.env(task_frame=frame, workspace=workspace, xyz_scale_m=.05, rot_scale_rad=.3)
        source = SpaceMouseTeleopSource.from_env(env, last_gripper_action_getter=lambda: 1.)
        assert source.cartesian_tracker.workspace.frame == 'table'
        np.testing.assert_array_equal(source.cartesian_tracker.workspace.high, [1]*3)
        assert not rig.robot.is_connected
    finally:
        rig.close()


def test_session_phase_file_closes_after_device_release(tmp_path):
    import json
    rig = _Rig('local')
    try:
        path = tmp_path / 'session.jsonl'
        env = rig.env(report_paths=RuntimeReportPaths(session_events=str(path)))
        path = env.output_path('session_events')
        env.require_session_hold()
        env.reset()
        env.session_event('ACTIVE', 'neutral_confirmed')
        env.close()
        rows = [json.loads(line) for line in path.read_text().splitlines()]
        assert [row['phase'] for row in rows] == ['READY', 'ACTIVE', 'STOPPING', 'CLOSED']
        assert [row['sequence'] for row in rows] == list(range(4))
        env.close()
        assert len(path.read_text().splitlines()) == 4
        assert not rig.robot.is_connected
    finally:
        rig.close()


def test_session_event_write_failure_cannot_prevent_disconnect(tmp_path, monkeypatch):
    rig = _Rig('local')
    try:
        env = rig.env(report_paths=RuntimeReportPaths(session_events=str(tmp_path / 'events.jsonl')))
        env.require_session_hold()
        env.reset()
        def fail(*args, **kwargs):
            raise OSError('event disk failure')
        monkeypatch.setattr(env._session_events, 'emit', fail)
        with pytest.raises(OSError, match='event disk failure'):
            env.close()
        assert not rig.robot.is_connected
    finally:
        monkeypatch.undo()
        rig.close()


@pytest.mark.parametrize('mode', ['local', 'remote'])
def test_hold_diagnostics_do_not_advance_episode_or_invent_input(mode, tmp_path):
    import json
    rig = _Rig(mode)
    try:
        path = tmp_path / 'control.jsonl'
        env = rig.env(report_paths=RuntimeReportPaths(diagnostics=str(path)))
        path = env.output_path('diagnostics')
        env.reset()
        env.step(np.zeros(7))
        target = env.session_hold_target()
        before = env._elapsed_steps
        for _ in range(2):
            env.session_hold_tick(target)
        rows = [json.loads(line) for line in path.read_text().splitlines()]
        assert [row['kind'] for row in rows] == ['step', 'hold', 'hold']
        assert env._elapsed_steps == before
        for row in rows[1:]:
            assert row['input_action'] is None and row['teleop'] is None
            assert row['measurement_phase'] == 'before_command'
            assert row['step'] == before
            np.testing.assert_array_equal(row['device_accepted_pose'], target)
    finally:
        rig.close()


@pytest.mark.parametrize('logging_failure', [False, True])
def test_command_failure_diagnostics_preserve_device_fault(tmp_path, monkeypatch, logging_failure):
    import json
    from types import SimpleNamespace
    from real_world_rl.devices.errors import DeviceFault
    rig = _Rig('local')
    try:
        path = tmp_path / 'failed.jsonl'
        env = rig.env(report_paths=RuntimeReportPaths(diagnostics=str(path)))
        path = env.output_path('diagnostics')
        env.reset()
        if logging_failure:
            env._diagnostic_stream.close()
            def fail_write(value):
                raise OSError('disk full')
            env._diagnostic_stream = SimpleNamespace(write=fail_write, close=lambda: None)
        def fail_send(command):
            raise DeviceFault('original command fault')
        monkeypatch.setattr(env, '_send_and_observe', fail_send)
        with pytest.raises(DeviceFault, match='original command fault'):
            env.step(np.zeros(7))
        assert env._elapsed_steps == 0
        if not logging_failure:
            row = json.loads(path.read_text())
            assert row['kind'] == 'command_failed'
            assert row['execution_status'] == 'unknown'
            assert row['device_accepted_pose'] is None and row['measured_base_pose'] is None
            assert row['requested_pose'] is not None
    finally:
        rig.close()


def _hold_source(events):
    from types import SimpleNamespace
    from real_world_rl.teleop.composite import CompositeTeleopSource
    events = iter(events)
    return CompositeTeleopSource(SimpleNamespace(session_input=lambda: (np.zeros(6), {})),
                                 SimpleNamespace(poll=lambda: SimpleNamespace(events=next(events))))


@pytest.mark.parametrize('mode', ['local', 'remote'])
def test_episode_end_to_hold_never_stops_the_arm_and_holds_the_last_accepted_target(mode, monkeypatch):
    """WO-17: the HOLD target is read with no arm stop after the episode's final step."""
    from real_world_rl.teleop.session import wait_after_episode
    rig = _Rig(mode)
    try:
        env = rig.env(max_episode_steps=1)
        env.reset()
        *_, truncated, _ = env.step(np.array([.5, 0, 0, 0, 0, 0, 0]))
        assert truncated
        episode_end = len(rig.arm.call_log)
        accepted = rig.arm.last_cartesian_target
        assert accepted is not None
        seen = {}
        read_target = env.session_hold_target
        def spy_target():
            seen['calls_before_hold'] = list(rig.arm.call_log[episode_end:])
            return read_target()
        monkeypatch.setattr(env, 'session_hold_target', spy_target)
        held = []
        tick = env.session_hold_tick
        monkeypatch.setattr(env, 'session_hold_tick', lambda target: (held.append(target.copy()), tick(target))[1])
        assert wait_after_episode(env, _hold_source([{}, {'operator_quit': True}]), allow_next=False,
                                  control_hz=1000., sleep=lambda _: None) == 'quit'
        assert seen['calls_before_hold'] == []
        assert 'stop' not in rig.arm.call_log[episode_end:]
        assert len(held) == 1
        np.testing.assert_array_equal(held[0], accepted)
    finally:
        rig.close()


@pytest.mark.parametrize('mode', ['local', 'remote'])
def test_hold_after_a_plain_arm_stop_refuses_instead_of_resending_the_old_target(mode):
    rig = _Rig(mode)
    try:
        env = rig.env()
        env.reset()
        env.step(np.array([.5, 0, 0, 0, 0, 0, 0]))
        assert rig.arm.last_cartesian_target is not None
        rig.arm.stop()  # e.g. a daemon/operator stop between the episode end and HOLD
        after_stop = len(rig.arm.call_log)
        with pytest.raises(RuntimeError, match='no accepted Cartesian target to hold'):
            env.session_hold_target()
        assert 'servo_cartesian' not in rig.arm.call_log[after_stop:]
    finally:
        rig.close()


def test_hold_target_after_step_watchdog_trip_reports_the_trip():
    from real_world_rl.supervise import StepWatchdogTripped
    rig = _Rig('local')
    try:
        env = rig.env()
        env.reset()
        env.step(np.array([.5, 0, 0, 0, 0, 0, 0]))
        message = 'no env step for 0.900s (watchdog limit 0.5s); arm.stop() was called'

        class TrippedWatchdog:
            tripped, trip_message = True, message
            def check_in(self):
                raise StepWatchdogTripped(message)
            def activate(self): pass
            def deactivate(self): pass
            def close(self): pass

        rig.arm.stop()  # what the watchdog thread did when it tripped
        env._step_watchdog = TrippedWatchdog()
        with pytest.raises(StepWatchdogTripped, match='watchdog limit'):
            env.session_hold_target()
    finally:
        rig.close()


@pytest.mark.parametrize('with_workspace', [False, True])
def test_hold_allows_measured_half_mm_below_base_box(with_workspace):
    """B1: exercise the real HOLD method against a fake device, not a fake backend."""
    workspace = {'low': [-1., -1., 0.], 'high': [1., 1., 1.]} if with_workspace else None
    rig = _Rig('local', safety_box=[-1., -1., 0., 1., 1., 1.], workspace=workspace)
    try:
        env = rig.env(workspace=workspace)
        env.reset()
        env.step(np.zeros(7))
        target = env.session_hold_target()
        rig.arm._tcp_pose[2] = -.0005
        rig.arm._pose_target = None
        before = env._elapsed_steps
        env.session_hold_tick(target)
        np.testing.assert_array_equal(rig.arm.last_cartesian_target, target)
        assert env._elapsed_steps == before
    finally:
        rig.close()


@pytest.mark.parametrize('mode', ['local', 'remote'])
@pytest.mark.parametrize('outside_m,should_fault', [(.001, False), (.004, True)])
def test_measured_workspace_margin_distinguishes_one_and_four_mm(mode, outside_m, should_fault):
    from real_world_rl.devices.errors import SafetyViolation
    workspace = {'low': [-1., -1., 0.], 'high': [1., 1., 1.]}
    rig = _Rig(mode, workspace=workspace)
    try:
        env = rig.env(workspace=workspace)
        env.reset()
        rig.arm._tcp_pose[2] = -outside_m
        rig.arm._pose_target = None
        if should_fault:
            with pytest.raises(SafetyViolation, match='measured TCP'):
                env.session_hold_tick([0, 0, 0, 0, 0, 0, 1])
            assert 'stop' in rig.arm.call_log
        else:
            env.session_hold_tick([0, 0, 0, 0, 0, 0, 1])
            assert env._workspace_fault is None
    finally:
        rig.close()


@pytest.mark.parametrize('mode', ['local', 'remote'])
@pytest.mark.parametrize('semantics', ['current', 'serl'])
def test_policy_target_projects_position_and_orientation_into_workspace(mode, semantics):
    from scipy.spatial.transform import Rotation
    workspace = {'low': [-.01]*3, 'high': [.01]*3,
                 'orientation_xyzw': [0, 0, 0, 1], 'max_angle_rad': .04}
    rig = _Rig(mode, workspace=workspace)
    try:
        options = dict(workspace=workspace, cartesian_semantics=semantics)
        if semantics == 'serl':
            options.update(safety_box=[-.015]*3+[.015]*3,
                           safety_box_rpy_low=[0, -.02, -.03],
                           safety_box_rpy_high=[.1, .02, .03])
        env = rig.env(**options)
        env.reset()
        env.step(np.array([1., 1., 1., 1., 1., 0., 0.]))
        target = rig.arm.last_cartesian_target
        np.testing.assert_allclose(target[:3], [.01]*3)
        assert Rotation.from_quat(target[3:]).magnitude() == pytest.approx(.04)
        assert rig.arm.health().safety_rejects == 0
    finally:
        rig.close()


def test_workspace_step_uses_cached_feedback_without_extra_arm_read(monkeypatch):
    workspace = {'low': [-1.]*3, 'high': [1.]*3}
    rig = _Rig('remote', workspace=workspace)
    try:
        env = rig.env(workspace=workspace)
        env.reset()
        calls = []
        original = rig.robot.arm.read_state
        monkeypatch.setattr(rig.robot.arm, 'read_state', lambda: (calls.append('read'), original())[1])
        env.step(np.zeros(7))
        assert calls == []  # robot.step already returns the post-command observation
    finally:
        rig.close()


def test_runtime_report_paths_are_not_serialized_and_close_is_idempotent(tmp_path, monkeypatch):
    import dataclasses
    from rl_garden.common.effective_config import json_value
    names = {'stop_trace_path', 'diagnostics_path', 'session_events_path'}
    config = RealRobotEnvConfig(robot={'type': 'franka'})
    assert not names & dataclasses.asdict(config).keys()
    assert not names & json_value(config).keys()
    rig = _Rig('local')
    try:
        report_paths = RuntimeReportPaths(diagnostics=str(tmp_path / 'control.jsonl'),
                                          session_events=str(tmp_path / 'events.jsonl'))
        env = rig.env(report_paths=report_paths)
        env.require_session_hold()
        env.reset()
        report_paths.stop_trace = str(tmp_path / 'stop.json')
        monkeypatch.setattr(rig.arm, 'stop_with_trace', lambda: {
            'trace': {'samples': []}, 'assessment': {'status': 'BLOCKED'}}, raising=False)
        env.close()
        env.close()
        assert not names & dataclasses.asdict(env.config).keys()
        assert not names & json_value(env.config).keys()
        assert env.output_path('stop_trace').exists()
        assert len([p for p in tmp_path.glob('stop-*.json') if not p.name.endswith('.assessment.json')]) == 1
        assert not (tmp_path / 'control.jsonl').exists()
        another = RealRobotEnv(config, robot=rig.robot, report_paths=report_paths)
        assert another.output_path('stop_trace') != env.output_path('stop_trace')
    finally:
        rig.close()


def test_frame_alone_is_display_only_and_workspace_compares_frame():
    frame = {'name': 'table', 'base_T_frame': np.eye(4).tolist()}
    rig = _Rig('remote')
    try:
        env = rig.env(task_frame=frame)
        assert 'spatial_constraints' not in env.requested_experiment
        env.reset()
    finally:
        rig.close()
    workspace = {'frame': 'base', 'low': [-1]*3, 'high': [1]*3}
    rig = _Rig('remote', workspace=workspace, task_frame=frame)
    try:
        different = {'name': 'table', 'base_T_frame': np.eye(4).tolist()}
        different['base_T_frame'][0][3] = .1
        env = rig.env(workspace=workspace, task_frame=different)
        with pytest.raises(DeviceFault, match='differs'):
            env.reset()
    finally:
        rig.close()


def test_operator_quit_is_not_followed_by_duplicate_stopping_event(tmp_path):
    import json
    rig = _Rig('local')
    try:
        env = rig.env(report_paths=RuntimeReportPaths(session_events=str(tmp_path / 'events.jsonl')))
        env.require_session_hold()
        env.reset()
        env.session_event('STOPPING', 'operator_quit')
        env.close()
        rows = [json.loads(line) for line in env.output_path('session_events').read_text().splitlines()]
        assert [row['reason'] for row in rows if row['phase'] == 'STOPPING'] == ['operator_quit']
    finally:
        rig.close()


def test_active_exception_event_records_type(tmp_path):
    import json
    rig = _Rig('local')
    try:
        env = rig.env(report_paths=RuntimeReportPaths(session_events=str(tmp_path / 'events.jsonl')))
        env.require_session_hold()
        env.reset()
        try:
            raise LookupError('rollout')
        except LookupError:
            env.close()
        rows = [json.loads(line) for line in env.output_path('session_events').read_text().splitlines()]
        assert rows[-1]['phase'] == 'FAULT'
        assert rows[-1]['reason'] == 'active_exception:LookupError'
        assert rows[-2]['reason'] == 'active_exception:LookupError'
    finally:
        rig.close()
