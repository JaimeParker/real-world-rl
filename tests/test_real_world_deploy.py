"""Tests for the eval-only real-robot deploy entrypoint (real_world_rl/deploy.py)."""
from __future__ import annotations

import gymnasium as gym
import pytest
import torch
import tyro
from gymnasium import spaces
from gymnasium.vector.utils import batch_space

import robot_infra.devices.claims as claims
from robot_infra.devices.errors import DeviceFault

from real_world_rl.deploy import (
    DEVICE_ERROR_EXIT_CODE,
    DeployArgs,
    EpisodeResult,
    build_deploy_env,
    main,
    resolve_deploy_robot_config,
    restore_policy,
    restore_training_args,
    run_deploy,
)
from real_world_rl.envs.real_robot import RealRobotEnvConfig

FAKE_FRANKA = {
    "type": "franka",
    "arm_backend": "fake",
    "gripper": {"backend": "fake"},
}


class _FakeHealth:
    ok = True


class _FakeRobot:
    def health(self):
        return _FakeHealth()


class _FakeEnv(gym.Env):
    """Batch-of-1 torch env shaped like RealRobotVectorEnv; truncates after ``episode_len``."""

    num_envs = 1

    def __init__(self, episode_len: int = 3, fail_on_step: int | None = None) -> None:
        self.single_observation_space = spaces.Dict(
            {"state": spaces.Box(-1, 1, (4,), dtype="float32")}
        )
        self.single_action_space = spaces.Box(-1, 1, (2,), dtype="float32")
        self.observation_space = batch_space(self.single_observation_space, 1)
        self.action_space = batch_space(self.single_action_space, 1)
        self.robot = _FakeRobot()
        self.episode_len = episode_len
        self.fail_on_step = fail_on_step
        self.total_steps = 0
        self.resets = 0
        self.closed = False
        self._t = 0

    def _obs(self):
        return {"state": torch.zeros(1, 4)}

    def reset(self, *, seed=None, options=None):
        self.resets += 1
        self._t = 0
        return self._obs(), {}

    def step(self, action):
        assert action.shape == (1, 2)
        self.total_steps += 1
        if self.fail_on_step is not None and self.total_steps == self.fail_on_step:
            raise DeviceFault("arm faulted", hint="clear the fault in Desk", device="arm")
        self._t += 1
        return (
            self._obs(),
            torch.tensor([1.0]),
            torch.tensor([False]),
            torch.tensor([self._t >= self.episode_len]),
            {"rtt_ms": 2.0, "elapsed_steps": self._t},
        )

    def close(self):
        self.closed = True


class _FakePolicy:
    def __init__(self) -> None:
        self.calls: list[bool] = []
        self.evaluated = False

    def eval(self):
        self.evaluated = True

    def predict(self, obs, deterministic=False):
        self.calls.append(deterministic)
        return torch.zeros(1, 2)


def test_run_deploy_runs_episodes_and_prints_summary(capsys):
    env, policy = _FakeEnv(episode_len=3), _FakePolicy()
    args = DeployArgs(checkpoint="unused.pt", episodes=2)
    results = run_deploy(args, env=env, policy=policy)

    from real_world_rl.metrics import EpisodeOutcome
    assert results == [
        EpisodeResult(steps=3, episode_return=3.0, truncated=True, rtt_ms_mean=2.0, health_ok=True,
                      outcome=EpisodeOutcome(None, False, 3, 0, end_reason="timeout"))
    ] * 2
    assert env.resets == 2 and env.closed
    assert policy.evaluated and policy.calls == [True] * 6
    out = capsys.readouterr().out
    assert out.count("[deploy] episode=") == 2
    assert "[deploy] summary episodes=2 mean_steps=3.0 mean_return=3.000" in out


def test_max_steps_per_episode_caps_episode():
    args = DeployArgs(checkpoint="unused.pt", episodes=1, max_steps_per_episode=2, deterministic=False)
    policy = _FakePolicy()
    (result,) = run_deploy(args, env=_FakeEnv(episode_len=10), policy=policy)
    assert result.steps == 2 and result.truncated
    assert policy.calls == [False, False]


def test_device_error_prints_hint_closes_env_and_returns_2(capsys):
    env = _FakeEnv(fail_on_step=2)
    code = main(DeployArgs(checkpoint="unused.pt", episodes=2), env=env, policy=_FakePolicy())
    assert code == DEVICE_ERROR_EXIT_CODE
    assert env.closed
    err = capsys.readouterr().err
    assert "arm faulted" in err and "clear the fault in Desk" in err


def test_main_returns_zero_on_success():
    env = _FakeEnv()
    assert main(DeployArgs(checkpoint="unused.pt", episodes=1), env=env, policy=_FakePolicy()) == 0
    assert env.closed


def test_deploy_args_parse():
    args = tyro.cli(
        DeployArgs,
        args=[
            "--checkpoint", "ckpt.pt",
            "--algo", "rlpd_hybrid",
            "--real_robot.remote", "127.0.0.1:1",
            "--real_robot.num-joints", "6",
            "--episodes", "3",
            "--no-deterministic",
            "--max-steps-per-episode", "50",
        ],
    )
    assert args.checkpoint == "ckpt.pt" and args.algo == "rlpd_hybrid"
    assert args.real_robot.remote == "127.0.0.1:1" and args.real_robot.num_joints == 6
    assert args.episodes == 3 and not args.deterministic and args.max_steps_per_episode == 50
    assert args.device == "cpu"


def test_deploy_spacemouse_mode_defaults_and_override_parse():
    assert DeployArgs(checkpoint="unused").spacemouse_mode is None
    args = tyro.cli(DeployArgs, args=[
        "--checkpoint", "unused", "--teleop-device", "spacemouse", "--spacemouse-mode", "serl",
    ])
    assert args.spacemouse_mode == "serl"


@pytest.mark.parametrize("override", [None, "serl"])
def test_deploy_inherits_or_overrides_serl_mode_before_env_construction(override, monkeypatch):
    import dataclasses
    from real_world_rl.action_contract import demo_contract_of, effective_experiment_of
    from real_world_rl.training.env_building import build_hil_env
    from real_world_rl.training.entrypoints.hil_serl import HilSerlArgs

    cli = _deploy_args("unused", enable_teleop=False, spacemouse_mode=override)
    trained = HilSerlArgs(env_backend="real_robot", teleop_device="spacemouse",
                         spacemouse_mode="serl")
    trained.real_robot = dataclasses.replace(
        cli.real_robot, safety_box=[-1] * 3 + [1] * 3,
        safety_box_rpy_low=[0, -1, -1], safety_box_rpy_high=[3, 1, 1],
    )
    training_env = build_hil_env(trained, role="learner", enable_teleop=False)
    try:
        saved = effective_experiment_of(training_env)
    finally:
        training_env.close()
    monkeypatch.setattr("real_world_rl.deploy._saved_experiment", lambda _: saved)
    env = build_deploy_env(cli, trained)
    try:
        contract = demo_contract_of(env)
        assert contract["experiment"]["cartesian_semantics"] == "serl"
        assert contract["experiment"]["sources"]["teleop_control"]["version"] == "spacemouse_serl_v1"
    finally:
        env.close()


def test_deploy_without_training_config_defaults_to_tracker():
    from real_world_rl.action_contract import demo_contract_of

    cli = _deploy_args("unused", enable_teleop=False)
    env = build_deploy_env(cli, None)
    try:
        assert "cartesian_semantics" not in demo_contract_of(env)["experiment"]
    finally:
        env.close()


@pytest.mark.parametrize("provided", ["unset", "matching"])
def test_serl_deploy_restores_all_safety_boxes(provided):
    import dataclasses
    from types import SimpleNamespace

    cli = _deploy_args("unused", enable_teleop=False)
    boxes = {
        "safety_box": [-1] * 3 + [1] * 3,
        "safety_box_rpy_low": [0, -1, -1],
        "safety_box_rpy_high": [3, 1, 1],
    }
    trained = dataclasses.replace(cli.real_robot, cartesian_semantics="serl", **boxes)
    if provided == "matching":
        cli.real_robot = dataclasses.replace(cli.real_robot, **boxes)
    resolved = resolve_deploy_robot_config(cli, SimpleNamespace(real_robot=trained))
    assert resolved.cartesian_semantics == "serl"
    for name, value in boxes.items():
        assert getattr(resolved, name) == value


@pytest.mark.parametrize("field,value", [
    ("safety_box", [-0.5, -1, -1, 1, 1, 1]),
    ("safety_box_rpy_low", [0, -0.5, -1]),
    ("safety_box_rpy_high", [3, 0.5, 1]),
])
def test_serl_deploy_rejects_different_cli_safety_box(field, value):
    import dataclasses
    from types import SimpleNamespace

    cli = _deploy_args("unused", enable_teleop=False)
    trained = dataclasses.replace(
        cli.real_robot, cartesian_semantics="serl", safety_box=[-1] * 3 + [1] * 3,
        safety_box_rpy_low=[0, -1, -1], safety_box_rpy_high=[3, 1, 1],
    )
    cli.real_robot = dataclasses.replace(cli.real_robot, **{field: value})
    with pytest.raises(ValueError, match="contradicts the training run") as error:
        resolve_deploy_robot_config(cli, SimpleNamespace(real_robot=trained))
    assert f"{field}: training=" in str(error.value)
    assert "cli=" in str(error.value)


@pytest.mark.parametrize("cli_box", [None, [-0.5, -1, -1, 1, 1, 1]])
def test_current_deploy_keeps_cli_xyz_safety_box(cli_box):
    import dataclasses
    from types import SimpleNamespace

    cli = _deploy_args("unused", enable_teleop=False)
    trained = dataclasses.replace(
        cli.real_robot, safety_box=[-1] * 3 + [1] * 3,
        safety_box_rpy_low=[0, -1, -1], safety_box_rpy_high=[3, 1, 1],
    )
    cli.real_robot = dataclasses.replace(cli.real_robot, safety_box=cli_box)
    resolved = resolve_deploy_robot_config(cli, SimpleNamespace(real_robot=trained))
    assert resolved.cartesian_semantics == "current"
    assert resolved.safety_box == cli_box
    assert resolved.safety_box_rpy_low == trained.safety_box_rpy_low
    assert resolved.safety_box_rpy_high == trained.safety_box_rpy_high


def test_saved_current_semantics_rejects_serl_override_before_construction(monkeypatch):
    import dataclasses
    from types import SimpleNamespace
    from real_world_rl.deploy import _restore_real_world_fields

    trained = SimpleNamespace()
    cli = _deploy_args("unused", enable_teleop=False, spacemouse_mode="serl")
    saved = {"env_backend": "real_robot", "teleop_device": "spacemouse", "spacemouse_mode": "tracker",
             "real_robot": dataclasses.asdict(cli.real_robot)}
    _restore_real_world_fields(trained, saved)
    assert trained.spacemouse_mode == "tracker" and trained.teleop_device == "spacemouse"

    def forbidden_build(*args, **kwargs):
        raise AssertionError("an incompatible override must fail before env construction")

    monkeypatch.setattr("rl_garden.envs.backend_registry.make_training_envs", forbidden_build)
    with pytest.raises(ValueError, match="conflicts.*current"):
        build_deploy_env(cli, trained)


def test_deploy_preserves_saved_current_without_mode_override(monkeypatch):
    import dataclasses
    from real_world_rl.action_contract import demo_contract_of
    from real_world_rl.deploy import _restore_real_world_fields
    from real_world_rl.training.env_building import build_hil_env
    from real_world_rl.training.entrypoints.hil_serl import HilSerlArgs
    from real_world_rl.action_contract import effective_experiment_of

    cli = _deploy_args("unused", enable_teleop=False)
    # Restored semantics are authoritative even for an older inconsistent config.
    saved = {"env_backend": "real_robot", "teleop_device": "spacemouse", "spacemouse_mode": "serl",
             "real_robot": dataclasses.asdict(cli.real_robot)}
    trained = HilSerlArgs(env_backend="real_robot")
    _restore_real_world_fields(trained, saved)
    baseline = HilSerlArgs(env_backend="real_robot", teleop_device="fake")
    baseline.real_robot = cli.real_robot
    env = build_hil_env(baseline, role="learner", enable_teleop=False)
    try:
        metadata = effective_experiment_of(env)
        from real_world_rl.training.env_building import teleop_control_spec
        metadata["experiment"]["sources"]["teleop_control"] = teleop_control_spec(trained)
    finally:
        env.close()
    monkeypatch.setattr("real_world_rl.deploy._saved_experiment", lambda _: metadata)
    env = build_deploy_env(cli, trained)
    try:
        assert "cartesian_semantics" not in demo_contract_of(env)["experiment"]
    finally:
        env.close()


# -- checkpoint restore against a real (fake-hardware) real_robot env ------------------


@pytest.fixture
def _claim_dir(tmp_path, monkeypatch):
    monkeypatch.setattr(claims, "_DEFAULT_LOCK_DIR", tmp_path / "claims")


def _deploy_args(checkpoint, **overrides) -> DeployArgs:
    real_robot = RealRobotEnvConfig(
        robot=FAKE_FRANKA, home_joints=[0.0] * 7, reset_settle_s=0.0, control_hz=1000.0
    )
    return DeployArgs(checkpoint=str(checkpoint), real_robot=real_robot, **overrides)


def test_restores_saved_rlpd_checkpoint_and_deploys(tmp_path, _claim_dir, capsys):
    from rl_garden.training.online.entrypoints.rlpd import build_rlpd

    checkpoint = tmp_path / "run" / "checkpoints" / "checkpoint_0.pt"
    checkpoint.parent.mkdir(parents=True)
    args = _deploy_args(checkpoint, episodes=1, max_steps_per_episode=2)

    # Train-side: an RLPD agent on the same env spaces, saved with its weights perturbed.
    training_args = _scratch_rlpd_args()
    env = build_deploy_env(args, training_args)
    saved = build_rlpd(training_args, env, None, logger=None, checkpoint_dir=None)
    with torch.no_grad():
        for param in saved.policy.parameters():
            param.add_(0.5)
    saved.save(checkpoint)
    env.close()

    restored_args = restore_training_args(args)
    env = build_deploy_env(args, restored_args)
    try:
        policy = restore_policy(args, restored_args, env)
        expected = saved.policy.state_dict()
        actual = policy.state_dict()
        assert actual.keys() == expected.keys()
        for key, value in expected.items():
            torch.testing.assert_close(actual[key], value.to(actual[key].device))
        assert all(p.device.type == "cpu" for p in policy.parameters())
    finally:
        env.close()

    (result,) = run_deploy(args)
    assert result.steps == 2 and result.truncated and result.health_ok is True
    assert "[deploy] summary episodes=1" in capsys.readouterr().out


def _scratch_rlpd_args():
    from rl_garden.training.online.entrypoints.rlpd import RLPDArgs

    return RLPDArgs(num_envs=1, num_eval_envs=1, buffer_size=8, buffer_device="cpu")


def test_deploy_interrupted_episode_is_reported_and_env_closed(capsys):
    class InterruptedEnv(_FakeEnv):
        def step(self, action):
            raise KeyboardInterrupt()
    env = InterruptedEnv()
    with pytest.raises(KeyboardInterrupt):
        run_deploy(DeployArgs(checkpoint="unused.pt"), env=env, policy=_FakePolicy())
    assert env.closed
    output = capsys.readouterr().out
    assert "interrupted_episodes=1" in output
    assert "completed_episodes=0" in output


@pytest.mark.parametrize('embedded', [False, True])
def test_deploy_inherits_workspace_and_rejects_different_cli_geometry(embedded):
    from types import SimpleNamespace
    frame = {'name': 'table', 'base_T_frame': [[1, 0, 0, .4], [0, 1, 0, 0],
                                               [0, 0, 1, .2], [0, 0, 0, 1]]}
    workspace = {'frame': 'table', 'low': [-.1]*3, 'high': [.1]*3}
    geometry = dict(task_frame=frame, workspace=workspace)
    trained = RealRobotEnvConfig(**({'robot': {'type': 'franka', **geometry}} if embedded else geometry))
    args = DeployArgs(checkpoint='unused.pt', real_robot=RealRobotEnvConfig(remote='robot:50051'))
    resolved = resolve_deploy_robot_config(args, SimpleNamespace(real_robot=trained))
    assert resolved.workspace == workspace and resolved.task_frame == frame
    assert resolved.remote == 'robot:50051'
    args.real_robot.workspace = {**workspace, 'high': [.2]*3}
    with pytest.raises(ValueError, match='workspace'):
        resolve_deploy_robot_config(args, SimpleNamespace(real_robot=trained))
