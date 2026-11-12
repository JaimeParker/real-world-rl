"""BimanualRealRobotEnv over two device-layer fake Frankas: config, spaces, action split,
concurrent stepping, observation/info prefixes, truncation, failure propagation, contract."""
from __future__ import annotations

import threading
import time

import numpy as np
import pytest
import tyro
from _bimanual_fakes import (
    ARMS,
    FRAME_HW,
    ZERO_HOME,
    arm_config,
    bimanual_config,
    camera_name,
    fake_franka,
)

import real_world_rl.devices.claims as claims
from real_world_rl.server import RemoteRobot
from real_world_rl.server.service import RobotService
from real_world_rl.server.transport import LocalClient

from real_world_rl.envs.bimanual import BimanualRealRobotEnv, BimanualRealRobotEnvConfig
from real_world_rl.envs.real_robot import RealRobotEnv

CARTESIAN_DIM = 2 * 6 + 2


@pytest.fixture(autouse=True)
def _claim_dir(tmp_path, monkeypatch):
    monkeypatch.setattr(claims, "_DEFAULT_LOCK_DIR", tmp_path / "claims")


@pytest.fixture
def make_env():
    """Builds two-arm envs over injected fake robots; closes them after the test."""
    envs: list[BimanualRealRobotEnv] = []

    def make(config: BimanualRealRobotEnvConfig | None = None, **config_kwargs) -> BimanualRealRobotEnv:
        config = config or bimanual_config(**config_kwargs)
        env = BimanualRealRobotEnv(
            config, robots={arm: fake_franka(side) for arm, side in zip(config.arm_names, ARMS)}
        )
        envs.append(env)
        return env

    yield make
    for env in envs:
        env.close()


def _spy_arm_commands(env: BimanualRealRobotEnv, monkeypatch) -> dict[str, list[dict]]:
    """Record every action dict each arm's env sends to its (local) robot."""
    sent: dict[str, list[dict]] = {arm: [] for arm in ARMS}
    for arm, arm_env in env.arms.items():
        original = arm_env.robot.send_action

        def spy(action, _arm=arm, _original=original):
            sent[_arm].append(action)
            return _original(action)

        monkeypatch.setattr(arm_env.robot, "send_action", spy)
    return sent


# -- config --------------------------------------------------------------------------


@pytest.mark.parametrize(
    "shared_field, left_value",
    [
        ("action_mode", "joint_delta"),
        ("num_joints", 6),
        ("control_hz", 20.0),
        ("gripper_in_action", False),
        ("xyz_scale_m", 0.05),
        ("rot_scale_rad", 0.2),
        ("joint_scale_rad", 0.1),
        ("state_fields", ("tcp_pose", "joint_pos")),
        ("image_size", (16, 16)),
    ],
)
def test_mismatched_shared_fields_are_rejected(shared_field, left_value):
    left = {shared_field: left_value}
    if shared_field == "num_joints":
        left["home_joints"] = [0.0] * 6
    with pytest.raises(ValueError, match=shared_field):
        bimanual_config(left=left)


def test_duplicate_camera_names_are_rejected():
    with pytest.raises(ValueError, match="unique across arms"):
        bimanual_config(right={"cameras": [camera_name("left")]})


def test_joint_limits_on_one_arm_only_are_rejected():
    with pytest.raises(ValueError, match="joint_limits"):
        bimanual_config(left={"joint_limits": [[-1.0, 1.0]] * 7})


def test_per_arm_max_episode_steps_is_rejected():
    with pytest.raises(ValueError, match="max_episode_steps"):
        bimanual_config(right={"max_episode_steps": 5})


@pytest.mark.parametrize("arm_names", [("left", "left"), ("left",), ("left", "")])
def test_arm_names_must_be_two_distinct_names(arm_names):
    with pytest.raises(ValueError, match="arm_names"):
        bimanual_config(arm_names=arm_names)


def test_require_robot_source_names_the_arm():
    config = BimanualRealRobotEnvConfig(left=arm_config("left"), right=arm_config("right", robot=None))
    with pytest.raises(ValueError, match="arm 'right'.*exactly one"):
        config.require_robot_source()
    with pytest.raises(ValueError, match="exactly one"):
        BimanualRealRobotEnv(config)


def test_config_default_constructs_and_parses_nested_arm_fields_from_cli():
    assert tyro.cli(BimanualRealRobotEnvConfig, args=[]) == BimanualRealRobotEnvConfig()
    config = tyro.cli(
        BimanualRealRobotEnvConfig,
        args=["--left.remote", "10.0.0.2:50051", "--right.remote", "10.0.0.3:50051", "--max_episode_steps", "50"],
    )
    assert config.left.remote == "10.0.0.2:50051" and config.right.remote == "10.0.0.3:50051"
    assert config.max_episode_steps == 50
    config.require_robot_source()


def test_arm_env_config_disables_per_arm_truncation_and_shares_settle_time():
    config = bimanual_config(max_episode_steps=3, reset_settle_s=0.25)
    for arm in ARMS:
        arm_env_config = config.arm_env_config(arm)
        assert arm_env_config.max_episode_steps is None
        assert arm_env_config.reset_settle_s == 0.25


def test_robots_for_unknown_arms_are_rejected():
    with pytest.raises(ValueError, match="unknown arms"):
        BimanualRealRobotEnv(bimanual_config(), robots={"middle": fake_franka("left")})


# -- spaces and contract ------------------------------------------------------------


def test_spaces_carry_arm_prefixes_and_unique_cameras(make_env):
    env = make_env()
    assert env.action_space.shape == (CARTESIAN_DIM,) and env.action_space.dtype == np.float32
    fields = env.config.left.state_fields
    assert list(env.observation_space["state"].spaces) == [f"{arm}_{field}" for arm in ARMS for field in fields]
    assert env.observation_space["state"]["right_joint_pos"].shape == (7,)
    assert list(env.observation_space["frames"].spaces) == ["left_wrist", "right_wrist"]
    assert env.observation_space["frames"]["right_wrist"].shape == (*FRAME_HW, 3)
    assert all(isinstance(arm_env, RealRobotEnv) for arm_env in env.arms.values())
    assert not any(robot.is_connected for robot in env.robot.values())


def test_joint_delta_without_grippers_action_width(make_env):
    env = make_env(shared={"action_mode": "joint_delta", "gripper_in_action": False})
    assert env.action_space.shape == (14,)
    assert env.arm_action_dim == 7 and env.num_joints == 7 and not env.gripper_in_action


def test_action_contract_is_shared_with_two_arms(make_env):
    env = make_env(shared={"action_mode": "joint_delta"})
    contract = env.action_contract
    assert contract["num_arms"] == 2 and contract["arm_names"] == ["left", "right"]
    assert contract["action_mode"] == "joint_delta" and contract["arm_action_dim"] == 7
    assert contract["gripper_in_action"] is True and contract["num_joints"] == 7
    assert contract["joint_limits"] is None
    assert contract["state_fields"] == env.config.left.state_fields
    assert env.action_mode == "joint_delta"


def test_action_contract_carries_joint_limits_per_arm(make_env):
    left_limits, right_limits = [[-1.0, 1.0]] * 7, [[-0.5, 0.5]] * 7
    env = make_env(
        shared={"action_mode": "joint_delta"},
        left={"joint_limits": left_limits},
        right={"joint_limits": right_limits},
        arm_names=("a", "b"),
    )
    assert env.action_contract["joint_limits"] == [left_limits, right_limits]
    assert env.action_contract["arm_names"] == ["a", "b"]
    assert list(env.observation_space["state"].spaces)[0].startswith("a_")


# -- reset and step -----------------------------------------------------------------


def test_reset_merges_prefixed_observations(make_env):
    env = make_env()
    obs, info = env.reset()
    assert env.observation_space.contains(obs)
    assert info == {}
    for arm in ARMS:
        np.testing.assert_allclose(obs["state"][f"{arm}_joint_pos"], ZERO_HOME, atol=1e-6)
        assert obs["state"][f"{arm}_gripper_pos"].tolist() == [1.0]
    assert set(obs["frames"]) == {"left_wrist", "right_wrist"}
    assert all(robot.is_connected for robot in env.robot.values())
    assert env.render() is obs["frames"]["left_wrist"]


def test_step_splits_action_into_arm_blocks_and_grippers(make_env, monkeypatch):
    env = make_env()
    env.reset()
    sent = _spy_arm_commands(env, monkeypatch)
    action = np.zeros(CARTESIAN_DIM, dtype=np.float32)
    action[0] = 1.0  # left +x
    action[6 + 1] = -1.0  # right -y
    action[12:] = [1.0, -1.0]  # left open, right closed

    env.step(action)

    scale = env.config.left.xyz_scale_m
    assert len(sent["left"]) == 1 and len(sent["right"]) == 1
    np.testing.assert_allclose(np.asarray(sent["left"][0]["arm"]["tcp_pose"])[:3], [scale, 0.0, 0.0])
    np.testing.assert_allclose(np.asarray(sent["right"][0]["arm"]["tcp_pose"])[:3], [0.0, -scale, 0.0])
    assert float(np.asarray(sent["left"][0]["gripper"]["gripper"])[0]) == 1.0
    assert float(np.asarray(sent["right"][0]["gripper"]["gripper"])[0]) == 0.0


def test_split_action_layout_for_joint_modes(make_env):
    env = make_env(shared={"action_mode": "joint_delta"})
    action = np.arange(16, dtype=np.float64)
    split = env.split_action(action)
    np.testing.assert_array_equal(split["left"], list(range(7)) + [14])
    np.testing.assert_array_equal(split["right"], list(range(7, 14)) + [15])


def test_step_rejects_wrong_width_and_requires_reset(make_env):
    env = make_env()
    with pytest.raises(RuntimeError, match="reset"):
        env.step(np.zeros(CARTESIAN_DIM, dtype=np.float32))
    env.reset()
    with pytest.raises(ValueError, match="action shape"):
        env.step(np.zeros(7, dtype=np.float32))


def test_both_arms_step_concurrently_once_per_step(make_env, monkeypatch):
    env = make_env()
    env.reset()
    # Each arm waits for the other inside its step: a sequential env would time out here.
    barrier = threading.Barrier(2, timeout=5.0)
    calls: dict[str, list[str]] = {arm: [] for arm in ARMS}
    for arm, arm_env in env.arms.items():
        original = arm_env.step

        def step(action, _arm=arm, _original=original):
            calls[_arm].append(threading.current_thread().name)
            barrier.wait()
            return _original(action)

        monkeypatch.setattr(arm_env, "step", step)

    env.step(np.zeros(CARTESIAN_DIM, dtype=np.float32))
    env.step(np.zeros(CARTESIAN_DIM, dtype=np.float32))

    assert [len(calls[arm]) for arm in ARMS] == [2, 2]
    assert calls["left"][0] != calls["right"][0]


def test_concurrent_step_paces_to_control_hz(make_env):
    env = make_env(shared={"control_hz": 20.0})
    env.reset()
    action = np.zeros(CARTESIAN_DIM, dtype=np.float32)
    env.step(action)
    start = time.monotonic()
    env.step(action)
    env.step(action)
    elapsed = time.monotonic() - start
    # Each step waits out the rest of its arm's 50 ms period since that arm's previous command.
    assert 0.08 <= elapsed < 0.5


def test_step_info_is_arm_prefixed(make_env):
    env = make_env()
    env.reset()
    obs, reward, terminated, truncated, info = env.step(np.zeros(CARTESIAN_DIM, dtype=np.float32))
    assert (reward, terminated, truncated) == (0.0, False, False)
    assert env.observation_space.contains(obs)
    for arm in ARMS:
        assert info[f"{arm}_tcp_pose"].shape == (7,)
        assert info[f"{arm}_elapsed_steps"] == 1
        assert f"{arm}_safety_clips" in info and f"{arm}_rtt_ms" in info
    assert info["elapsed_steps"] == 1
    assert info["rtt_ms"] is None  # local robots report no round trip


def test_truncated_at_max_episode_steps(make_env):
    env = make_env(max_episode_steps=2)
    env.reset()
    action = np.zeros(CARTESIAN_DIM, dtype=np.float32)
    assert env.step(action)[3] is False
    _, _, _, truncated, info = env.step(action)
    assert truncated is True and info["elapsed_steps"] == 2
    env.reset()
    assert env.step(action)[3] is False


def test_failure_in_one_arm_is_raised_after_the_other_finished(make_env, monkeypatch):
    env = make_env()
    env.reset()
    left_finished = threading.Event()
    original_left_step = env.arms["left"].step

    def slow_left_step(action):
        time.sleep(0.05)
        result = original_left_step(action)
        left_finished.set()
        return result

    def failing_right_step(action):
        raise RuntimeError("right arm faulted")

    monkeypatch.setattr(env.arms["left"], "step", slow_left_step)
    monkeypatch.setattr(env.arms["right"], "step", failing_right_step)

    with pytest.raises(RuntimeError, match="right arm faulted"):
        env.step(np.zeros(CARTESIAN_DIM, dtype=np.float32))
    assert left_finished.is_set()


def test_reset_failure_in_one_arm_propagates(make_env, monkeypatch):
    env = make_env()

    def failing_reset(**kwargs):
        raise RuntimeError("left arm cannot home")

    monkeypatch.setattr(env.arms["left"], "reset", failing_reset)
    with pytest.raises(RuntimeError, match="cannot home"):
        env.reset()


def test_close_disconnects_both_arms_and_env_can_reset_again(make_env):
    env = make_env()
    env.reset()
    env.close()
    assert not any(robot.is_connected for robot in env.robot.values())
    env.reset()
    env.step(np.zeros(CARTESIAN_DIM, dtype=np.float32))


def test_one_local_and_one_daemon_arm():
    service = RobotService(fake_franka("right"), watchdog_s=30.0)
    robots = {"left": fake_franka("left"), "right": RemoteRobot.from_client(LocalClient(service))}
    config = bimanual_config(
        shared={"image_size": FRAME_HW},
        right={"robot": None, "remote": "local"},
    )
    env = BimanualRealRobotEnv(config, robots=robots)
    try:
        env.reset()
        _, _, _, _, info = env.step(np.zeros(CARTESIAN_DIM, dtype=np.float32))
        assert info["left_rtt_ms"] is None and isinstance(info["right_rtt_ms"], float)
        assert info["rtt_ms"] == info["right_rtt_ms"]
    finally:
        env.close()
        service.close()
