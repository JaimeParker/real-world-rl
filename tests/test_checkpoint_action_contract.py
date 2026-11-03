"""Actual algorithm checkpoint loads preserve policy action interpretation."""
import dataclasses

import gymnasium as gym
import pytest
import torch
from gymnasium import spaces
from gymnasium.vector.utils import batch_space

from real_world_rl.checkpoint_contract import load_policy_state_with_action_contract


class SpaceEnv(gym.Env):
    num_envs = 1

    def __init__(self):
        self.single_observation_space = spaces.Dict({"state": spaces.Box(-1, 1, (4,), dtype="float32")})
        self.single_action_space = spaces.Box(-1, 1, (7,), dtype="float32")
        self.observation_space = batch_space(self.single_observation_space, 1)
        self.action_space = batch_space(self.single_action_space, 1)
        self.resets = 0

    def reset(self, **kwargs):
        self.resets += 1
        raise AssertionError("checkpoint construction must not reset")


@pytest.mark.parametrize("method", ["hil", "residual_continuous", "residual_hybrid", "silri_continuous", "silri_hybrid"])
def test_actual_builders_accept_legacy_checkpoint_and_reject_loaded_action_buffers(tmp_path, method):
    from real_world_rl.training.entrypoints.hil_serl import HilSerlArgs, build_hil_agent
    from real_world_rl.training.entrypoints.residual_hil_serl import ResidualHilSerlArgs, build_residual_hil_serl
    from real_world_rl.training.entrypoints.silri import SilriArgs, build_silri
    common = dict(hidden_dim=8, discrete_hidden_dim=8, buffer_size=8, buffer_device="cpu")
    if method == "hil":
        args, build = HilSerlArgs(**common), build_hil_agent
    elif method.startswith("residual"):
        args = ResidualHilSerlArgs(**common, base_provider="zero", residual_variant=method.split("_")[1])
        build = build_residual_hil_serl
    else:
        args = SilriArgs(**common, feature_dim=8, behavior_hidden_dims=(8,), lambda_hidden_dims=(8,),
                         discrete_gripper=method.endswith("hybrid"))
        build = build_silri
    env = SpaceEnv()
    agent = build(args, env, None, None, None)
    path = tmp_path / "legacy.pt"
    agent.save(path)
    restored = build(dataclasses.replace(args, load_checkpoint=str(path), load_replay_buffer=False), env, None, None, None)
    expected = agent.policy.state_dict()
    for key, value in restored.policy.state_dict().items():
        torch.testing.assert_close(value, expected[key])
    payload = torch.load(path, weights_only=False)
    state = payload["state"]["policy"]
    if method == "silri_continuous":
        # DDPG's actor has no serialized scaling: tanh followed by SiLRI's
        # fixed clamp defines the normalized bounds, even for extreme weights.
        assert not any(key.endswith(("action_scale", "action_bias", "action_low", "action_high")) for key in state)
        key = next(key for key in state if key.startswith("actor.policy.") and key.endswith("bias") and state[key].shape == (7,))
        state[key].fill_(1e6)
        torch.save(payload, path)
        bounded = build(dataclasses.replace(args, load_checkpoint=str(path), load_replay_buffer=False), env, None, None, None)
        device = next(bounded.policy.parameters()).device
        action = bounded.policy.predict({"state": torch.zeros(1, 4, device=device)}, deterministic=True)
        assert torch.isfinite(action).all()
        assert torch.all(action > .99)
        assert torch.all(action < 1) and torch.all(action > -1)
        assert torch.all(action <= torch.as_tensor(env.single_action_space.high, device=device))
        assert torch.all(action >= torch.as_tensor(env.single_action_space.low, device=device))
        assert env.resets == 0
        return
    key = ("_discrete_action_values" if method == "silri_hybrid"
           else next(key for key in state if key.endswith("action_scale")))
    state[key] = state[key] * 2
    torch.save(payload, path)
    with pytest.raises(ValueError, match="action bounds/scales"):
        build(dataclasses.replace(args, load_checkpoint=str(path), load_replay_buffer=False), env, None, None, None)
    assert env.resets == 0


def test_snapshot_guard_uses_policy_partial_head_shape_and_detects_bias():
    class Policy(torch.nn.Module):
        def __init__(self):
            super().__init__()
            self.actor = torch.nn.Module()
            self.actor.register_buffer("action_scale", torch.ones(6))
            self.actor.register_buffer("action_bias", torch.zeros(6))
    policy = Policy()
    state = {key: value.clone() for key, value in policy.state_dict().items()}
    load_policy_state_with_action_contract(policy, state)
    state["actor.action_bias"][0] = .25
    with pytest.raises(ValueError, match="action bounds/scales"):
        load_policy_state_with_action_contract(policy, state)


def test_fwbw_shared_explicit_root_keeps_actual_learner_paths_separate(tmp_path, monkeypatch):
    import json
    from real_world_rl.training.entrypoints import hil_serl
    from real_world_rl.training.entrypoints.hil_serl import HilSerlArgs
    captured = []
    monkeypatch.setattr(hil_serl, "build_hil_env", lambda *a, **k: SpaceEnv())
    class Loop:
        def __init__(self, agent, **kwargs):
            captured.append((agent.checkpoint_dir, kwargs["journal_path"], kwargs["port"]))
        def run(self):
            pass
    monkeypatch.setattr("real_world_rl.loops.hil_serl.HilSerlLearnerLoop", Loop)
    root = tmp_path / "shared"
    args = HilSerlArgs(fwbw=True, exp_name="experiment", checkpoint_dir=str(root),
                      log_dir=str(tmp_path), log_type="none", hidden_dim=8, discrete_hidden_dim=8,
                      buffer_size=8, demo_buffer_size=8, buffer_device="cpu")
    for direction in ("forward", "backward"):
        hil_serl._run_learner(dataclasses.replace(args, fwbw_direction=direction))
        config = json.loads((tmp_path / f"experiment_{direction}" / "config.json").read_text())
        assert config["args"]["fwbw_direction"] == direction
    assert [str(value[0]) for value in captured] == [str(root / "forward"), str(root / "backward")]
    assert [value[1] for value in captured] == [str(root / direction / "journal.sqlite3") for direction in ("forward", "backward")]
    assert captured[0][2] != captured[1][2]


def test_fwbw_direction_leaf_and_ambiguous_legacy_directory(tmp_path):
    from real_world_rl.training.entrypoints.hil_serl import HilSerlArgs
    from real_world_rl.training.runtime_paths import hil_learner_paths
    leaf = tmp_path / "forward"
    leaf.mkdir()
    (leaf / "journal.sqlite3").write_bytes(b"existing journal")
    args = HilSerlArgs(fwbw=True, fwbw_direction="forward", checkpoint_dir=str(leaf))
    assert hil_learner_paths(args, "run")[1] == str(leaf)
    with pytest.raises(ValueError, match="other direction"):
        hil_learner_paths(dataclasses.replace(args, fwbw_direction="backward"), "run")
    flat = tmp_path / "legacy"
    flat.mkdir()
    journal = flat / "journal.sqlite3"
    journal.write_bytes(b"ambiguous existing journal")
    with pytest.raises(ValueError, match="ambiguous direction ownership"):
        hil_learner_paths(dataclasses.replace(args, checkpoint_dir=str(flat)), "run")
    assert journal.read_bytes() == b"ambiguous existing journal"


@pytest.mark.parametrize("method", ["serl", "hil_serl", "residual_hil_serl", "silri"])
def test_resolved_frequency_is_persisted_before_logger_or_env_creation(tmp_path, monkeypatch, method):
    import importlib
    import json
    from rl_garden.common import Logger
    from real_world_rl.envs.real_robot import RealRobotEnvConfig
    module = importlib.import_module(f"real_world_rl.training.entrypoints.{method}")
    cls = getattr(module, {"serl": "SerlArgs", "hil_serl": "HilSerlArgs",
                          "residual_hil_serl": "ResidualHilSerlArgs", "silri": "SilriArgs"}[method])
    args = cls(env_backend="real_robot", real_robot=RealRobotEnvConfig(), control_hz=20,
               exp_name="frequency", log_dir=str(tmp_path), log_type="none")
    def stop_after_persistence(**kwargs):
        raise RuntimeError("stop after config persistence")
    monkeypatch.setattr(Logger, "create", stop_after_persistence)
    with pytest.raises(RuntimeError, match="stop after config"):
        module._run_learner(args)
    config = json.loads((tmp_path / "frequency" / "config.json").read_text())
    assert config["args"]["control_hz"] == config["args"]["real_robot"]["control_hz"] == 20
