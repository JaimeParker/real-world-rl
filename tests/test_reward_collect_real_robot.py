"""Reward-classifier data collection on the real_robot backend: the ``--env-backend`` dispatch of
``collect_data``, its flags, and a collection over a fake camera env with a scripted teleop source
(labels from the success/failure events, the hold action, saving, early stop)."""
from __future__ import annotations

import pickle

import numpy as np
import pytest
import torch
from _teleop_fakes import ContractEnv
from gymnasium import spaces

from rl_garden_real_world.envs.wrappers.intervention import InterventionWrapper
from rl_garden_real_world.reward import collect_data, collect_real_robot
from rl_garden_real_world.reward.collect_real_robot import (
    RealRobotCollectArgs,
    run_real_robot_collector,
)
from rl_garden_real_world.teleop import TeleopReading
from rl_garden_real_world.teleop.fake import ScriptedTeleopSource
from rl_garden_real_world.training import env_building

OPEN, CLOSED = 1.0, -1.0


class CameraEnv(ContractEnv):
    """A ``joint_delta`` arm (7 joints) plus gripper whose observations are ``state`` and a
    ``rgb_wrist`` image filled with the global step count (from 1); raises ``KeyboardInterrupt``
    at ``interrupt_step``."""

    def __init__(self, interrupt_step=None):
        super().__init__("joint_delta", arm_action_dim=7)
        self.single_observation_space = spaces.Dict(
            {
                "state": spaces.Box(-1.0, 1.0, (4,), dtype=np.float32),
                "rgb_wrist": spaces.Box(0, 255, (2, 2, 3), dtype=np.uint8),
            }
        )
        self.single_action_space = spaces.Box(-1.0, 1.0, (8,), dtype=np.float32)
        self.interrupt_step = interrupt_step
        self.steps = 0

    def _obs(self):
        return {"state": torch.zeros(1, 4), "rgb_wrist": torch.full((1, 2, 2, 3), self.steps, dtype=torch.uint8)}

    def reset(self, **kwargs):
        super().reset(**kwargs)
        return self._obs(), {}

    def step(self, action):
        self.step_actions.append(action)
        self.steps += 1
        if self.steps == self.interrupt_step:
            raise KeyboardInterrupt
        return self._obs(), torch.tensor([0.0]), torch.tensor([False]), torch.tensor([False]), {}


def _reading(*events, intervened=True, episode_end=False):
    return TeleopReading(
        action=np.append(np.zeros(7), CLOSED).astype(np.float32),
        intervened=intervened,
        episode_end=episode_end,
        events={name: True for name in events},
    )


def _wrapped(env, readings):
    return InterventionWrapper(env, ScriptedTeleopSource(readings, action_mode="joint_delta", arm_action_dim=7))


def _args(tmp_path, **overrides) -> RealRobotCollectArgs:
    return RealRobotCollectArgs(**{"output_dir": str(tmp_path), "control_hz": 1e6, **overrides})


def _saved(tmp_path, label: str) -> list:
    (path,) = tmp_path.glob(f"real_robot_*{label}_images_*.pkl")
    with open(path, "rb") as stream:
        return pickle.load(stream)


def _steps_of(entries) -> list[int]:
    assert all(set(entry["obs"]) == {"rgb_wrist"} for entry in entries)  # images only, no state
    return [int(entry["obs"]["rgb_wrist"].reshape(-1)[0]) for entry in entries]


# -- command line --------------------------------------------------------------------------


@pytest.mark.parametrize(
    "argv, real_robot",
    [
        (["--output-dir", "x"], False),
        (["--env-backend", "real_robot"], True),
        (["--env-backend=real_robot"], True),
        (["--env_backend", "real_robot"], True),
        (["--teleop-device", "pico"], False),
    ],
)
def test_an_env_backend_flag_selects_the_real_robot_collector(argv, real_robot):
    assert collect_data.selects_real_robot(argv) is real_robot


def test_main_parses_the_hil_flags_for_the_real_robot_backend(tmp_path, monkeypatch):
    received = []
    monkeypatch.setattr(collect_real_robot, "run_real_robot_collector", received.append)
    collect_data.main(
        argv=[
            "--env-backend", "real_robot",
            "--output-dir", str(tmp_path),
            "--successes-needed", "50",
            "--teleop-device", "leader_arm",
            "--teleop-event-device", "keyboard",
            "--classifier-image-keys", "rgb_wrist",
            "--real_robot.remote", "10.0.0.2:50051",
        ]
    )
    (args,) = received
    assert isinstance(args, RealRobotCollectArgs)
    assert (args.env_backend, args.output_dir, args.successes_needed) == ("real_robot", str(tmp_path), 50)
    assert (args.teleop_device, args.teleop_event_device) == ("leader_arm", "keyboard")
    assert tuple(args.classifier_image_keys) == ("rgb_wrist",) and args.real_robot.remote == "10.0.0.2:50051"
    assert args.exp_name == "real_robot" and args.eval_freq == 0


@pytest.mark.parametrize(
    "argv, flags",
    [
        (["--help"], ["--bridge-url", "--successes-needed"]),
        (["--env-backend", "real_robot", "--help"], ["--successes-needed", "--teleop-event-device"]),
    ],
)
def test_help_lists_each_paths_flags(argv, flags, capsys):
    with pytest.raises(SystemExit) as exit_info:
        collect_data.main(argv=argv)
    assert exit_info.value.code == 0
    out = capsys.readouterr().out
    assert all(flag in out for flag in flags)


def test_the_real_robot_collector_takes_no_camera_capture():
    with pytest.raises(ValueError, match="camera_capture"):
        collect_data.main(camera_capture=lambda: None, argv=["--env-backend", "real_robot"])


# -- collection ----------------------------------------------------------------------------


def test_frames_are_labelled_by_the_success_and_failure_events(tmp_path, monkeypatch):
    env = CameraEnv()
    readings = [
        _reading(intervened=False),  # 1 failure: nothing pressed yet
        _reading("success"),  # 2 success: labelling switches to success
        _reading("success", "failure"),  # 3 failure: a failure in the same step wins
        _reading(),  # 4 failure
        _reading("success"),  # 5 success
        _reading(episode_end=True),  # 6 success, then the episode ends and labelling resets
        _reading(),  # 7 failure
        _reading("success"),  # 8 success: the 4th, collection stops
    ]
    wrapped = _wrapped(env, readings)
    monkeypatch.setattr(env_building, "build_hil_env", lambda args, *, role, **kwargs: wrapped)

    success_path, failure_path = run_real_robot_collector(_args(tmp_path, successes_needed=4))

    assert _steps_of(_saved(tmp_path, "success")) == [2, 5, 6, 8]
    assert _steps_of(_saved(tmp_path, "failure")) == [1, 3, 4, 7]
    assert "real_robot_4_success_images_" in success_path and "real_robot_failure_images_" in failure_path
    assert env.reset_calls == 2 and env.steps == 8 and env.close_calls == 1
    # Nobody intervened in step 1: the env got the hold action, a zero arm and an open gripper.
    np.testing.assert_allclose(torch.as_tensor(env.step_actions[0]).reshape(-1).numpy(), [0.0] * 7 + [OPEN])


def test_a_stopped_collection_saves_what_it_has(tmp_path, monkeypatch):
    env = CameraEnv(interrupt_step=3)
    wrapped = _wrapped(env, [_reading(intervened=False), _reading("success")])
    monkeypatch.setattr(env_building, "build_hil_env", lambda args, **kwargs: wrapped)

    with pytest.raises(KeyboardInterrupt):
        run_real_robot_collector(_args(tmp_path))

    assert _steps_of(_saved(tmp_path, "success")) == [2]
    assert _steps_of(_saved(tmp_path, "failure")) == [1]
    assert env.close_calls == 1


@pytest.mark.parametrize("overrides, message", [({"output_dir": None}, "--output-dir"), ({"fwbw": True}, "--fwbw")])
def test_missing_output_and_fwbw_are_rejected(tmp_path, monkeypatch, overrides, message):
    def never_built(*args, **kwargs):
        raise AssertionError("the env must not be built")

    monkeypatch.setattr(env_building, "build_hil_env", never_built)
    with pytest.raises(ValueError, match=message):
        run_real_robot_collector(_args(tmp_path, **overrides))


def test_a_presets_reward_sources_are_dropped_from_the_env(tmp_path, monkeypatch, capsys):
    # Both would end the episode on the first success; a training preset may carry them.
    env = CameraEnv()
    wrapped = _wrapped(env, [_reading("success")])
    built = []

    def build(args, **kwargs):
        built.append(args)
        return wrapped

    monkeypatch.setattr(env_building, "build_hil_env", build)
    args = _args(
        tmp_path, successes_needed=1, reward_from_teleop_events=True, classifier_checkpoint="/tmp/classifier.pt"
    )
    run_real_robot_collector(args)

    (env_args,) = built
    assert not env_args.reward_from_teleop_events and env_args.classifier_checkpoint is None
    assert "ignoring --reward-from-teleop-events and --classifier-checkpoint" in capsys.readouterr().out
    assert _steps_of(_saved(tmp_path, "success")) == [1]
