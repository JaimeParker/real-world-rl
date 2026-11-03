"""Evaluation consumes bundle weights and releases every owned resource."""
import json
from types import SimpleNamespace
import pytest
import torch
from _residual_fakes import DictEnv
from _silri_fakes import silri_agent
from real_world_rl.loops.checkpoint import save_bundle
from real_world_rl.training.evaluation import resolve_eval_checkpoint, evaluate_policy


def test_eval_resolves_manifest_and_actually_loads_agent_weights(tmp_path):
    source = silri_agent()
    manifest = save_bundle(source, tmp_path, {})
    expected = {key: value.clone() for key, value in source.policy.state_dict().items()}
    target = silri_agent()
    with torch.no_grad():
        for parameter in target.policy.parameters(): parameter.add_(3)
    checkpoint = resolve_eval_checkpoint(SimpleNamespace(load_checkpoint=str(manifest)), DictEnv())
    assert checkpoint.endswith("agent.pt")
    target.load(checkpoint, load_replay_buffer=False)
    for key, value in target.policy.state_dict().items():
        torch.testing.assert_close(value, expected[key])


def test_incompatible_saved_experiment_fails_before_any_reset(tmp_path):
    source = silri_agent()
    manifest = save_bundle(source, tmp_path, {})
    generation = tmp_path / json.loads(manifest.read_text())["generation"]
    (generation / "experiment.json").write_text(json.dumps({"experiment_contract": {
        "schema_version": 2, "action": {"action_mode": "wrong"}, "experiment": {}}}))
    env = DictEnv()
    with pytest.raises(ValueError, match="experiment"):
        resolve_eval_checkpoint(SimpleNamespace(load_checkpoint=str(manifest)), env)
    assert env.resets == 0 and not env.actions


@pytest.mark.parametrize("env_close_fails", [False, True])
def test_provider_cleanup_failure_preserves_rollout_error_and_releases_env(env_close_fails):
    class Env(DictEnv):
        def step(self, action): raise ValueError("original rollout failure")
        def close(self):
            self.closed = True
            if env_close_fails: raise OSError("env close failure")
    class Provider:
        closed = False
        def close(self):
            self.closed = True
            raise RuntimeError("provider close failure")
    env, provider = Env(), Provider()
    policy = SimpleNamespace(provider=provider, predict=lambda obs, deterministic: torch.zeros(1, 3))
    with pytest.raises(ValueError, match="original rollout failure"):
        evaluate_policy(env, policy, episodes=1, seed=1)
    assert env.closed and provider.closed
