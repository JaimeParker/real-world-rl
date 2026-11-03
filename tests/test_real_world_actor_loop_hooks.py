"""ActorLoop's overridable per-step hooks: documented arguments, call order and
the default transition dict (which must stay what ``run`` pushed before the
hooks existed)."""
from __future__ import annotations

import torch

from real_world_rl.loops.actor import ActorLoop


class _Env:
    num_envs = 1

    def __init__(self, terminate_every=None, info=None):
        self.terminate_every = terminate_every
        self.info = info or {}
        self.reset_calls = 0
        self.step_calls = 0
        self.actions_received = []

    def reset(self, *, seed=None):
        del seed
        self.reset_calls += 1
        return torch.zeros(1, 4), {}

    def step(self, action):
        self.step_calls += 1
        self.actions_received.append(action)
        terminated = torch.tensor(
            [self.terminate_every is not None and self.step_calls % self.terminate_every == 0]
        )
        return (
            torch.full((1, 4), float(self.step_calls)),
            torch.tensor([1.0]),
            terminated,
            torch.tensor([False]),
            dict(self.info),
        )


class _Policy:
    def eval(self):
        pass

    def load_state_dict(self, state_dict):
        del state_dict

    def predict(self, obs, deterministic=False):
        del obs, deterministic
        return torch.ones(1, 2)


class _SyncClient:
    def __init__(self, params=None):
        self.params = params
        self.pushed = []

    def start(self):
        pass

    def stop(self):
        pass

    def push_transition(self, transition):
        self.pushed.append(transition)

    def latest_policy_params(self):
        return self.params


def _loop(cls=ActorLoop, env=None, sync_client=None):
    return cls(
        env or _Env(),
        _Policy(),
        sync_client or _SyncClient(),
        control_hz=1_000_000.0,
    )


def test_default_act_returns_the_prediction_and_an_empty_context():
    action, ctx = _loop()._act(torch.zeros(1, 4))
    torch.testing.assert_close(action, torch.ones(1, 2))
    assert ctx == {}


def test_default_build_transition_is_the_literal_dict_run_used_to_push():
    loop = _loop()
    obs, next_obs = torch.zeros(1, 4), torch.ones(1, 4)
    action, reward = torch.full((1, 2), 0.5), torch.tensor([1.0])
    terminated, truncated = torch.tensor([True]), torch.tensor([False])

    transition = loop._build_transition(
        obs, next_obs, action, reward, terminated, truncated, {}, {"ignored": 1}
    )

    assert list(transition) == ["obs", "next_obs", "action", "reward", "done"]
    assert transition["obs"] is obs
    assert transition["next_obs"] is next_obs
    assert transition["action"] is action
    assert transition["reward"] is reward
    assert transition["done"] is terminated


def test_default_build_transition_records_the_executed_intervention_action():
    loop = _loop()
    executed = torch.full((1, 2), 5.0)
    transition = loop._build_transition(
        torch.zeros(1, 4),
        torch.zeros(1, 4),
        torch.ones(1, 2),
        torch.tensor([1.0]),
        torch.tensor([False]),
        torch.tensor([False]),
        {"intervene_action": executed},
        {},
    )
    assert transition["action"] is executed


def test_default_build_transition_merges_extra_transition_fields():
    class _Tagging(ActorLoop):
        def _extra_transition_fields(self, info):
            return {"tag": info["tag"]}

    transition = _loop(_Tagging)._build_transition(
        torch.zeros(1, 4),
        torch.zeros(1, 4),
        torch.ones(1, 2),
        torch.tensor([1.0]),
        torch.tensor([False]),
        torch.tensor([False]),
        {"tag": "x"},
        {},
    )
    assert transition["tag"] == "x"


def test_default_run_pushes_the_pre_hook_transition_dict():
    sync_client = _SyncClient()
    _loop(sync_client=sync_client).run(total_steps=1)

    (pushed,) = sync_client.pushed
    assert set(pushed) == {"obs", "next_obs", "action", "reward", "done"}
    torch.testing.assert_close(pushed["obs"], torch.zeros(1, 4))
    torch.testing.assert_close(pushed["next_obs"], torch.full((1, 4), 1.0))
    torch.testing.assert_close(pushed["action"], torch.ones(1, 2))
    torch.testing.assert_close(pushed["reward"], torch.tensor([1.0]))
    torch.testing.assert_close(pushed["done"], torch.tensor([False]))


def test_hooks_receive_the_documented_arguments_in_order():
    events = []
    info = {"marker": 1}
    env = _Env(terminate_every=2, info=info)
    sync_client = _SyncClient()

    class _Hooked(ActorLoop):
        def _act(self, obs):
            events.append(("act", obs.clone()))
            return torch.full((1, 2), 3.0), {"step_ctx": len(events)}

        def _build_transition(
            self, obs, next_obs, policy_action, reward, terminated, truncated, info, ctx
        ):
            events.append(("build", obs.clone(), next_obs.clone(), policy_action, ctx, info))
            return {"built": len(events)}

        def _push(self, transition):
            events.append(("push", transition))

        def _on_episode_end(self, info):
            events.append(("episode_end", info, env.reset_calls))

    _loop(_Hooked, env=env, sync_client=sync_client).run(total_steps=2)

    names = [event[0] for event in events]
    assert names == ["act", "build", "push", "act", "build", "push", "episode_end"]
    # The env action is what _act returned.
    torch.testing.assert_close(env.actions_received[0], torch.full((1, 2), 3.0))
    _, obs, next_obs, policy_action, ctx, built_info = events[1]
    torch.testing.assert_close(obs, torch.zeros(1, 4))
    torch.testing.assert_close(next_obs, torch.full((1, 4), 1.0))
    torch.testing.assert_close(policy_action, torch.full((1, 2), 3.0))
    assert ctx == {"step_ctx": 1}
    assert built_info == info
    assert events[2] == ("push", {"built": 2})
    # _on_episode_end gets the last step's info and runs before env.reset.
    assert events[-1][1] == info and events[-1][2] == 1
    assert env.reset_calls == 2
    # _push replaced the sync client push.
    assert sync_client.pushed == []


def test_default_push_goes_to_the_sync_client():
    sync_client = _SyncClient()
    _loop(sync_client=sync_client)._push({"a": 1})
    assert sync_client.pushed == [{"a": 1}]


def test_on_episode_end_is_not_called_while_the_episode_continues():
    ended = []

    class _Hooked(ActorLoop):
        def _on_episode_end(self, info):
            ended.append(info)

    _loop(_Hooked, env=_Env(terminate_every=None)).run(total_steps=3)
    assert ended == []


def test_on_params_refreshed_counts_distinct_payloads_only():
    versions = []

    class _Hooked(ActorLoop):
        def _on_params_refreshed(self, version):
            versions.append(version)

    sync_client = _SyncClient()
    loop = _loop(_Hooked, sync_client=sync_client)

    loop._maybe_refresh_policy()
    assert versions == []  # nothing downloaded yet

    first = {"w": torch.tensor([1.0])}
    sync_client.params = first
    loop._maybe_refresh_policy()
    loop._maybe_refresh_policy()  # the same cached payload served again
    assert versions == [1]

    sync_client.params = {"w": torch.tensor([2.0])}
    loop._maybe_refresh_policy()
    assert versions == [1, 2]
    assert loop.params_version == 2


def test_on_params_refreshed_runs_after_the_policy_loaded_the_params():
    seen = []

    class _LoadingPolicy(_Policy):
        loaded = None

        def load_state_dict(self, state_dict):
            self.loaded = state_dict

    policy = _LoadingPolicy()
    params = {"w": torch.tensor([1.0])}

    class _Hooked(ActorLoop):
        def _on_params_refreshed(self, version):
            seen.append(policy.loaded)

    loop = _Hooked(_Env(), policy, _SyncClient(params), control_hz=1_000_000.0)
    loop._maybe_refresh_policy()
    assert seen == [params]
