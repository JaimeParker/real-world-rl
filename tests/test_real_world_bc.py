"""Tests for the real-world BC entrypoint. Mirrors
test_hil_serl_training_entrypoints.py's style (mock make_training_envs, real
Logger/persist_resolved_config with log_type="none") -- note franka_real
backend registration is a known pre-existing environment quirk (also
affecting several hil_serl entrypoint tests): these pass as part of the full
real_world/tests/ suite (another test module's import registers the backend
as a side effect) but fail if this file is run in isolation.
"""
from __future__ import annotations

import pickle
from pathlib import Path

import pytest

import gymnasium as gym
import torch
from gymnasium import spaces
from gymnasium.vector.utils import batch_space

from real_world_rl.training.entrypoints.bc import RealWorldBCArgs, run_real_world_bc


@pytest.mark.parametrize("teleop_flags", [
    [],
    ["--teleop-device", "None"],
    ["--teleop-device", "spacemouse", "--spacemouse-mode", "serl"],
])
def test_bc_parses_gello_station_preset_without_resolving_teleop(monkeypatch, teleop_flags):
    from real_world_rl.training import args as shared_args, env_building
    from real_world_rl.training.registry import registry

    def unexpected_teleop(*args, **kwargs):
        raise AssertionError("offline BC must not require or resolve a teleop device")

    monkeypatch.setattr(shared_args, "require_teleop_device", unexpected_teleop)
    monkeypatch.setattr(env_building, "resolve_teleop_input", unexpected_teleop)
    monkeypatch.setattr(env_building, "resolve_spacemouse_mode", unexpected_teleop)
    preset = Path(__file__).resolve().parents[1] / "examples/presets/gello_cartesian.yaml"
    command = registry.parse_command([
        "real_world_bc", "--config", str(preset), "--buffer-device", "cpu", *teleop_flags,
    ])

    assert isinstance(command.args, RealWorldBCArgs)
    expected_device = teleop_flags[1] if teleop_flags else "leader_cartesian"
    assert command.args.teleop_device == (None if expected_device == "None" else expected_device)
    assert command.args.teleop_event_device == "stdin"
    assert command.args.teleop_options_yaml == "gello_leader.yaml"
    assert command.args.teleop_lead_cap_m == 0.05
    assert command.args.teleop_lead_cap_rad == 0.3
    assert command.args.teleop_target_speed_m_s == 0.05
    assert command.args.teleop_target_speed_rad_s == 0.3
    assert command.args.env_backend == "real_robot"
    assert command.args.real_robot.action_mode == "cartesian_delta"


class _FakeEnv(gym.Env):
    num_envs = 1

    def __init__(self):
        self.single_observation_space = spaces.Box(-1, 1, (4,), dtype="float32")
        self.single_action_space = spaces.Box(-1, 1, (3,), dtype="float32")
        self.observation_space = batch_space(self.single_observation_space, 1)
        self.action_space = batch_space(self.single_action_space, 1)


def _write_demo_pkl(path, n=8):
    transitions = [
        {
            "obs": torch.randn(1, 4),
            "next_obs": torch.randn(1, 4),
            "action": torch.rand(1, 3) * 2 - 1,
            "reward": torch.ones(1),
            "done": torch.zeros(1),
        }
        for _ in range(n)
    ]
    with open(path, "wb") as f:
        pickle.dump(transitions, f)


def _base_args(tmp_path, **overrides) -> RealWorldBCArgs:
    kwargs = dict(
        log_dir=str(tmp_path / "logs"),
        log_type="none",
        num_offline_steps=2,
        checkpoint_freq=0,
        save_final_checkpoint=False,
        batch_size=4,
        buffer_size=64,
        buffer_device="cpu",
        device="cpu",
    )
    kwargs.update(overrides)
    return RealWorldBCArgs(**kwargs)


def test_run_real_world_bc_trains_on_demo_pkls(monkeypatch, tmp_path):
    monkeypatch.setattr(
        "rl_garden.envs.backend_registry.make_training_envs",
        lambda backend, req: (_FakeEnv(), None),
    )
    demo_path = tmp_path / "demos.pkl"
    _write_demo_pkl(demo_path)

    args = _base_args(tmp_path, demo_dataset_paths=[str(demo_path)])
    run_real_world_bc(args)  # must not raise


def test_run_real_world_bc_raises_when_no_demos_loaded(monkeypatch, tmp_path):
    monkeypatch.setattr(
        "rl_garden.envs.backend_registry.make_training_envs",
        lambda backend, req: (_FakeEnv(), None),
    )
    args = _base_args(tmp_path, demo_dataset_paths=[str(tmp_path / "nonexistent_*.pkl")])
    try:
        run_real_world_bc(args)
        assert False, "expected ValueError for zero loaded demo transitions"
    except ValueError as e:
        assert "No demo transitions" in str(e)


def test_run_real_world_bc_saves_periodic_and_final_checkpoints(monkeypatch, tmp_path):
    monkeypatch.setattr(
        "rl_garden.envs.backend_registry.make_training_envs",
        lambda backend, req: (_FakeEnv(), None),
    )
    demo_path = tmp_path / "demos.pkl"
    _write_demo_pkl(demo_path)

    checkpoint_dir = tmp_path / "ckpt"
    args = _base_args(
        tmp_path,
        demo_dataset_paths=[str(demo_path)],
        num_offline_steps=4,
        checkpoint_freq=2,
        save_final_checkpoint=True,
        checkpoint_dir=str(checkpoint_dir),
    )
    run_real_world_bc(args)

    assert (checkpoint_dir / "checkpoint_2.pt").exists()
    assert (checkpoint_dir / "checkpoint_4.pt").exists()
    assert (checkpoint_dir / "final.pt").exists()



def test_run_real_world_bc_logs_metrics_at_log_freq(monkeypatch, tmp_path):
    """The offline loop logs through Logger.log_metrics; a log step must not raise."""
    monkeypatch.setattr(
        "rl_garden.envs.backend_registry.make_training_envs",
        lambda backend, req: (_FakeEnv(), None),
    )
    demo_path = tmp_path / "demos.pkl"
    _write_demo_pkl(demo_path)
    logged: list[tuple[dict, int]] = []
    from rl_garden.common.logger import Logger

    monkeypatch.setattr(Logger, "log_metrics", lambda self, metrics, step, **kw: logged.append((dict(metrics), step)))
    args = _base_args(tmp_path, demo_dataset_paths=[str(demo_path)], num_offline_steps=4, log_freq=2)

    run_real_world_bc(args)

    assert [step for _, step in logged] == [2, 4]
    assert all(metrics for metrics, _ in logged)


@pytest.mark.parametrize("failure", ["read", "train"])
def test_bc_releases_env_and_logger_on_data_or_training_error(tmp_path, monkeypatch, failure):
    from types import SimpleNamespace
    from rl_garden.common import Logger
    import real_world_rl.training.entrypoints.bc as entrypoint
    env = _FakeEnv()
    closed = []
    env.close = lambda: closed.append("env")
    logger = SimpleNamespace(close=lambda: closed.append("logger"))
    monkeypatch.setattr(Logger, "create", lambda **kwargs: logger)
    monkeypatch.setattr("rl_garden.envs.backend_registry.make_training_envs", lambda *args: (env, None))
    path = tmp_path / "demo.pkl"
    _write_demo_pkl(path)
    if failure == "read":
        path.write_bytes(b"broken pickle")
        error = pickle.UnpicklingError
    else:
        class Agent:
            buffer_device = "cpu"
            replay_buffer = SimpleNamespace(add=lambda *args, **kwargs: None)
            def train(self, *args, **kwargs): raise RuntimeError("training failed")
        monkeypatch.setattr(entrypoint, "build_real_world_bc", lambda *args, **kwargs: Agent())
        error = RuntimeError
    with pytest.raises(error):
        run_real_world_bc(_base_args(tmp_path, demo_dataset_paths=[str(path)]))
    assert closed == ["env", "logger"]


def test_bc_metadata_roundtrips_classifier_path_provenance_with_semantic_identity(tmp_path, monkeypatch):
    import json
    from _residual_fakes import DictEnv
    from real_world_rl.action_contract import demo_contract_of, write_contract_sidecar, validate_experiment
    from real_world_rl.envs.wrappers.experiment import apply_experiment_contract
    from real_world_rl.deploy import DeployArgs, _saved_experiment
    sources = {"classifier": {"path": "/recording-machine/model.pt", "content_sha256": "a" * 64, "threshold": .5}}
    physical = dict(action_mode="joint_delta", arm_action_dim=3, gripper_in_action=False,
                    num_joints=3, experiment={"task": {"name": "none"}})
    raw = DictEnv(action_dim=3)
    raw.action_contract = physical
    recorded = apply_experiment_contract(raw, source_config=sources)
    saved = demo_contract_of(recorded)
    saved["effective_control"] = {"arm": {"sdk_version": "recording-sdk"}}
    path = tmp_path / "demo.pkl"
    _write_demo_pkl(path)
    write_contract_sidecar(path, saved)
    fresh = DictEnv(action_dim=3)
    fresh.action_contract = physical
    monkeypatch.setattr("rl_garden.envs.backend_registry.make_training_envs", lambda *args: (fresh, None))
    checkpoint_dir = tmp_path / "bc" / "checkpoints"
    run_real_world_bc(_base_args(tmp_path, demo_dataset_paths=[str(path)], checkpoint_dir=str(checkpoint_dir),
                                num_offline_steps=1, save_final_checkpoint=True))
    metadata = json.loads((checkpoint_dir.parent / "experiment.json").read_text())
    classifier = metadata["experiment_contract"]["experiment"]["sources"]["classifier"]
    assert "path" not in classifier and classifier["content_sha256"] == "a" * 64
    assert metadata["effective_experiment"]["provenance"]["sources"]["classifier"]["path"] == sources["classifier"]["path"]
    assert metadata["effective_experiment"]["runtime_control"] == saved["effective_control"]
    assert metadata["experiment_hash"]
    deploy = DeployArgs(checkpoint=str(checkpoint_dir / "final.pt"), algo="real_world_bc", config_path=str(checkpoint_dir.parent / "config.json"))
    restored = _saved_experiment(deploy)
    target_raw = DictEnv(action_dim=3)
    target_raw.action_contract = physical
    target = apply_experiment_contract(target_raw, source_config={"classifier": {**sources["classifier"], "path": "/deployment-machine/model.pt"}})
    validate_experiment(target, restored)
    assert target_raw.resets == 0


def test_bc_runner_resume_restores_partial_replay_without_original_demos(monkeypatch, tmp_path):
    monkeypatch.setattr("rl_garden.envs.backend_registry.make_training_envs",
                        lambda backend, req: (_FakeEnv(), None))
    demo = tmp_path / "seed.pkl"
    _write_demo_pkl(demo)
    rows = pickle.loads(demo.read_bytes())
    for row in rows:
        row.update(intervened=True, human_action_mask=torch.tensor([[True, False, False]]),
                   override_mask=torch.tensor([[True, True, False]]),
                   policy_action=torch.full((1, 3), 0.25))
    demo.write_bytes(pickle.dumps(rows))
    first = tmp_path / "first" / "checkpoints"
    run_real_world_bc(_base_args(tmp_path, demo_dataset_paths=[str(demo)],
                                checkpoint_dir=str(first), num_offline_steps=1,
                                save_final_checkpoint=True, save_replay_buffer=True))
    replay_one = torch.load(first / "replay_buffer_final.pt", weights_only=False)
    assert replay_one["real_world_replay_version"] == 1
    demo.unlink()
    second = tmp_path / "second" / "checkpoints"
    run_real_world_bc(_base_args(tmp_path, demo_dataset_paths=[str(demo)],
                                load_checkpoint=str(first / "final.pt"), load_replay_buffer=True,
                                checkpoint_dir=str(second), num_offline_steps=1,
                                save_final_checkpoint=True, save_replay_buffer=True))
    replay_two = torch.load(second / "replay_buffer_final.pt", weights_only=False)
    assert replay_one["pos"] == replay_two["pos"] == len(rows)
    for column in ("actions", "intervened", "human_action_mask", "override_mask", "policy_actions"):
        torch.testing.assert_close(replay_one[column], replay_two[column])
    checkpoint = torch.load(second / "final.pt", weights_only=False)
    assert checkpoint["metadata"]["global_update"] == 2
