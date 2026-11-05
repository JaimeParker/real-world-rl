"""RealRobotEnv over robot_infra fake devices, both local and through RemoteRobot + LocalClient."""
from __future__ import annotations

import time

import numpy as np
import pytest

import robot_infra.devices.backends  # noqa: F401  (registers the fake backends)
import robot_infra.devices.claims as claims
from robot_infra.devices import CameraConfig, DeviceFault, GripperConfig
from robot_infra.devices.robots.franka import FrankaConfig, FrankaRobot
from robot_infra.server import RemoteRobot
from robot_infra.server.service import RobotService
from robot_infra.server.transport import LocalClient

from real_world_rl.envs.real_robot import RealRobotEnv, RealRobotEnvConfig

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

    def env(self, **overrides) -> RealRobotEnv:
        env = RealRobotEnv(self.config(**overrides), robot=self.robot)
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
    from robot_infra.server.remote import RemoteArm

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
    from robot_infra.server.remote import RemoteArm

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
    from robot_infra.devices.teleop import TeleopConfig

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
