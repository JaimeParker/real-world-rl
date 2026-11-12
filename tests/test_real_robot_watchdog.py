"""RealRobotEnv's optional in-process step watchdog (``RealRobotEnvConfig.watchdog_s``) over fake devices."""
from __future__ import annotations

import time

import numpy as np
import pytest

import real_world_rl.devices.backends  # noqa: F401  (registers the fake backends)
import real_world_rl.devices.claims as claims
from real_world_rl.devices import CameraConfig, GripperConfig
from real_world_rl.devices.robots.franka import FrankaConfig, FrankaRobot
from real_world_rl.supervise import StepWatchdogTripped

from real_world_rl.envs.real_robot import RealRobotEnv, RealRobotEnvConfig

_CONTROL_HZ = 50.0


@pytest.fixture(autouse=True)
def _claim_dir(tmp_path, monkeypatch):
    monkeypatch.setattr(claims, "_DEFAULT_LOCK_DIR", tmp_path / "claims")


@pytest.fixture
def make_env():
    envs: list[RealRobotEnv] = []

    def make(**overrides) -> RealRobotEnv:
        robot = FrankaRobot(FrankaConfig(
            arm_backend="fake", gripper=GripperConfig(backend="fake"),
            cameras=[CameraConfig(name="wrist", backend="fake", width=32, height=24, fps=60)],
        ))
        values = dict(robot={"type": "franka", "arm_backend": "fake"}, cameras=["wrist"],
                      home_joints=[0.0] * 7, reset_settle_s=0.0, control_hz=_CONTROL_HZ)
        values.update(overrides)
        env = RealRobotEnv(RealRobotEnvConfig(**values), robot=robot)
        envs.append(env)
        return env

    yield make
    for env in envs:
        env.close()


def _zero_action(env: RealRobotEnv) -> np.ndarray:
    return np.zeros(env.action_space.shape, dtype=np.float32)


def test_watchdog_is_off_by_default(make_env):
    env = make_env()
    env.reset()
    env.step(_zero_action(env))
    assert env._step_watchdog is None


def test_watchdog_stops_the_arm_after_silence_and_the_next_step_raises(make_env):
    env = make_env(watchdog_s=0.1)
    env.reset()
    for _ in range(3):
        env.step(_zero_action(env))
    arm = env.robot.arm
    stops_before = arm.call_log.count("stop")

    time.sleep(0.4)  # the caller stalls for 4x watchdog_s

    assert arm.call_log.count("stop") == stops_before + 1
    with pytest.raises(StepWatchdogTripped):
        env.step(_zero_action(env))

    env.reset()  # a reset re-arms the watchdog and clears the trip
    env.step(_zero_action(env))


def test_watchdog_is_paused_during_a_reset(make_env, monkeypatch):
    env = make_env(watchdog_s=0.1)
    env.reset()
    env.step(_zero_action(env))
    arm = env.robot.arm

    def stops_during_reset_and_step() -> int:
        stops_before = arm.call_log.count("stop")
        env.reset()
        env.step(_zero_action(env))
        return arm.call_log.count("stop") - stops_before

    fast_reset_stops = stops_during_reset_and_step()
    original_move = arm.move_to_joints

    def slow_reset_motion(*args, **kwargs):
        time.sleep(0.4)
        return original_move(*args, **kwargs)

    monkeypatch.setattr(arm, "move_to_joints", slow_reset_motion)
    assert stops_during_reset_and_step() == fast_reset_stops  # the 0.4 s motion did not trip it


def test_watchdog_does_not_change_the_experiment_contract(make_env):
    assert make_env(watchdog_s=0.3).action_contract == make_env().action_contract


@pytest.mark.parametrize("watchdog_s", [0.02, 0.0, -1.0, float("nan")])
def test_watchdog_must_exceed_one_control_period(watchdog_s):
    with pytest.raises(ValueError, match="watchdog_s"):
        RealRobotEnvConfig(robot={"type": "franka"}, control_hz=_CONTROL_HZ, watchdog_s=watchdog_s)
