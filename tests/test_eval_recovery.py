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


@pytest.mark.parametrize("episode_steps", [1, 2])
@pytest.mark.parametrize("elapsed, expected", [(.02, .08), (.15, 0.)])
def test_eval_paces_policy_steps(monkeypatch, elapsed, expected, episode_steps):
    import real_world_rl.training.evaluation as evaluation
    calls = []
    clock = iter([0., elapsed, 1., 1. + elapsed])
    monkeypatch.setattr(evaluation, "time", SimpleNamespace(
        perf_counter=lambda: next(clock), sleep=calls.append), raising=False)

    class Env:
        steps = 0
        def reset(self, **kwargs):
            self.steps = 0
            return torch.zeros(1, 3), {}
        def step(self, action):
            self.steps += 1
            return torch.zeros(1, 3), 0., self.steps == episode_steps, False, {}
        def close(self): pass

    policy = SimpleNamespace(predict=lambda obs, deterministic: torch.zeros(1, 3))
    evaluate_policy(Env(), policy, episodes=2 // episode_steps, seed=1, control_hz=10)
    assert calls == pytest.approx([expected, expected] if expected else [])


@pytest.mark.parametrize("quit_at", ["start", "end", "budget"])
def test_eval_hold_gates_reset_and_refreshes_observation(monkeypatch, quit_at):
    from real_world_rl.teleop.core import session
    calls = []
    source = object()
    monkeypatch.setattr(session, "session_backend", lambda env: calls.append("validate"))
    monkeypatch.setattr(session, "session_source", lambda env: source)
    def before(env, src, **kwargs):
        assert src is source and kwargs["control_hz"] == 20
        calls.append("before")
        return "quit" if quit_at == "start" else "next"
    def after(env, src, *, allow_next, **kwargs):
        calls.append(("after", allow_next))
        return "quit" if quit_at == "end" or not allow_next else "next"
    monkeypatch.setattr(session, "wait_before_episode", before)
    monkeypatch.setattr(session, "wait_after_episode", after)
    monkeypatch.setattr(session, "session_observe", lambda env: torch.ones(1, 3))
    class Env:
        def reset(self, **kwargs):
            calls.append("reset")
            return torch.zeros(1, 3), {}
        def step(self, action):
            calls.append("step")
            return torch.zeros(1, 3), 0., True, False, {}
        def close(self): calls.append("close")
    def predict(obs, deterministic):
        assert torch.all(obs == 1)
        return torch.zeros(1, 3)
    evaluate_policy(Env(), SimpleNamespace(predict=predict), episodes=2, seed=1,
                    control_hz=20, episode_end_behavior="hold")
    expected = ["validate", "reset", "before"]
    if quit_at != "start": expected += ["step", ("after", True)]
    if quit_at == "budget": expected += ["reset", "before", "step", ("after", False)]
    assert calls == expected + ["close"]


def test_full_bundle_restores_seed_demo_buffer(tmp_path):
    from _silri_fakes import fill
    from real_world_rl.loops.checkpoint import restore_bundle
    source = silri_agent()
    source.init_demo_buffer(16, .5)
    fill(source, 3, demo=True)
    manifest = save_bundle(source, tmp_path, {})
    target = silri_agent()
    target.init_demo_buffer(16, .5)
    restore_bundle(target, manifest)
    assert len(target.offline_replay_buffer) == 3
    torch.testing.assert_close(target.offline_replay_buffer.actions[:3],
                               source.offline_replay_buffer.actions[:3])


def test_eval_quit_during_episode_does_not_reset_again():
    class Env:
        resets = 0
        closed = False
        def reset(self, **kwargs):
            self.resets += 1
            return torch.zeros(1, 3), {}
        def step(self, action):
            return torch.zeros(1, 3), 0., False, False, {"teleop_events": {"operator_quit": True}}
        def close(self): self.closed = True
    env = Env()
    summary = evaluate_policy(env, SimpleNamespace(predict=lambda obs, deterministic: torch.zeros(1, 3)),
                              episodes=3, seed=1)
    assert env.resets == 1 and env.closed
    assert summary["interrupted_episodes"] == 1
