"""ResidualActorLoop (warm-up gate, intervention recording, base-action caching across
steps and episodes, base provider resync on intervened steps) and ResidualLearnerLoop
(extras kept, demo pkls filled, snapshots)."""
from __future__ import annotations

import pickle

import pytest
import torch
from _residual_fakes import (
    ACTION_DIM,
    STATE_DIM,
    DictEnv,
    FakeSyncClient,
    RecordingProvider,
    residual_agent,
    residual_row,
)
from rl_garden_real_world.algorithms.residual import ResidualRLPD, ResidualRLPDHybrid
from rl_garden_real_world.hil_serl import HilSerlActorLoop, HilSerlLearnerLoop
from rl_garden_real_world.residual import ResidualActorLoop, ResidualLearnerLoop


def _actor_loop(policy, *, env=None, sync_client=None, provider=None, **kwargs) -> ResidualActorLoop:
    return ResidualActorLoop(
        env or DictEnv(),
        policy,
        sync_client or FakeSyncClient(),
        provider or RecordingProvider(),
        control_hz=1_000_000.0,
        deterministic=True,
        **kwargs,
    )


def _obs(marker: float) -> dict:
    return {"state": torch.full((1, STATE_DIM), marker)}


def test_loops_extend_the_hil_serl_loops():
    assert issubclass(ResidualActorLoop, HilSerlActorLoop)
    assert issubclass(ResidualLearnerLoop, HilSerlLearnerLoop)


@pytest.mark.parametrize("cls", [ResidualRLPD, ResidualRLPDHybrid])
def test_actor_acts_with_the_pure_base_until_the_first_parameter_refresh(cls):
    policy = residual_agent(cls).policy
    with torch.no_grad():
        policy.actor.fc_mean.bias.fill_(3.0)  # a trained policy would add a large residual
    sync_client = FakeSyncClient(params=None)
    loop = _actor_loop(policy, sync_client=sync_client)
    base = torch.full((1, ACTION_DIM), RecordingProvider.value_for(200.0))

    loop._maybe_refresh_policy()
    action, ctx = loop._act(_obs(200.0))
    assert loop.params_version == 0
    assert torch.equal(action, base)
    assert torch.equal(ctx["unit"], torch.zeros(1, policy.residual_dim))
    assert torch.equal(ctx["base_actions"], base)
    assert ctx["policy_action"] is action

    sync_client.params = policy.state_dict()
    loop._maybe_refresh_policy()
    action, ctx = loop._act(_obs(200.0))
    assert loop.params_version == 1
    expected_arm = (base[:, :2] + 0.1 * torch.tanh(torch.tensor(3.0))).clamp(-1.0, 1.0)
    torch.testing.assert_close(action[:, :2], expected_arm)


def test_warmup_unit_residual_stays_inside_its_box():
    policy = residual_agent(ResidualRLPD).policy
    loop = _actor_loop(policy, warmup_unit_residual=0.5)
    for _ in range(20):
        action, ctx = loop._act(_obs(300.0))
        base = ctx["base_actions"]
        assert float((action - base).abs().max()) <= 0.5 * 0.1 + 1e-7
        assert float(ctx["unit"].abs().max()) <= 0.5
        assert float(action[0, -1]) == float(base[0, -1])  # gripper scale 0


def test_hybrid_warmup_keeps_the_base_gripper():
    policy = residual_agent(ResidualRLPDHybrid).policy
    loop = _actor_loop(policy, provider=RecordingProvider())
    action, ctx = loop._act(_obs(-1000.0))  # base -1.0 on every dim
    assert action.shape == (1, ACTION_DIM)
    assert float(action[0, -1]) == -1.0


def test_warmup_unit_residual_outside_unit_interval_is_rejected():
    with pytest.raises(ValueError, match="warmup_unit_residual"):
        _actor_loop(residual_agent(ResidualRLPD).policy, warmup_unit_residual=1.5)


def test_intervened_transition_stores_the_human_action_the_base_and_the_policy_action():
    provider = RecordingProvider()
    loop = _actor_loop(residual_agent(ResidualRLPD).policy, provider=provider)
    obs, next_obs = _obs(10.0), _obs(20.0)
    action, ctx = loop._act(obs)
    human = torch.full((1, ACTION_DIM), 0.7)
    penalty = torch.tensor([-0.05])
    info = {"intervene_action": human, "intervened": True, "grasp_penalty": penalty}

    transition = loop._build_transition(
        obs, next_obs, action, torch.ones(1), torch.tensor([False]), torch.tensor([False]), info, ctx
    )

    assert transition["action"] is human
    assert transition["policy_action"] is action
    assert transition["base_actions"] is ctx["base_actions"]
    assert transition["intervened"] is True
    assert transition["grasp_penalty"] is penalty
    torch.testing.assert_close(
        transition["next_base_actions"], torch.full((1, ACTION_DIM), RecordingProvider.value_for(20.0))
    )
    assert provider.seen == [10.0, 20.0]


def test_run_reuses_next_base_within_episodes_and_resets_the_provider_at_episode_end():
    env = DictEnv(terminate_every=2)
    provider = RecordingProvider()
    sync_client = FakeSyncClient()
    _actor_loop(residual_agent(ResidualRLPD).policy, env=env, sync_client=sync_client, provider=provider).run(
        total_steps=4
    )

    # Reset obs 101, steps 1-2 (2 terminal, pre-reset), reset obs 102, steps 3-4.
    assert provider.seen == [101.0, 1.0, 2.0, 102.0, 3.0, 4.0]
    assert provider.reset_calls == 2

    def base(marker):
        return torch.full((1, ACTION_DIM), RecordingProvider.value_for(marker))

    pushed = sync_client.pushed
    assert len(pushed) == 4
    expected = [(101.0, 1.0), (1.0, 2.0), (102.0, 3.0), (3.0, 4.0)]
    for transition, (base_marker, next_marker) in zip(pushed, expected):
        torch.testing.assert_close(transition["base_actions"], base(base_marker))
        torch.testing.assert_close(transition["next_base_actions"], base(next_marker))
        assert transition["intervened"] is False
        assert set(transition) >= {"obs", "next_obs", "action", "reward", "done", "policy_action"}
    # The env executed exactly the pure base actions (warm-up, no refresh).
    for executed, (base_marker, _) in zip(env.actions, expected):
        torch.testing.assert_close(executed, base(base_marker))


# -- base resync on intervention ---------------------------------------------------------

# Episode 1 is steps 1-5 (5 terminal) with the human acting on steps 2 and 3; step 6 is
# the first step of episode 2.
_INTERVENE_STEPS = (2, 3)
_INTERVENED_FLAGS = [False, True, True, False, False, False]
# Reset obs 101, steps 1-5 (5 terminal, pre-reset), reset obs 102, step 6.
_PROVIDER_QUERIES = [101.0, 1.0, 2.0, 3.0, 4.0, 5.0, 102.0, 6.0]


def _run_with_interventions(**loop_kwargs) -> tuple[RecordingProvider, list[dict]]:
    provider = RecordingProvider()
    sync_client = FakeSyncClient()
    env = DictEnv(terminate_every=5, intervene_steps=_INTERVENE_STEPS)
    _actor_loop(
        residual_agent(ResidualRLPD).policy, env=env, sync_client=sync_client, provider=provider, **loop_kwargs
    ).run(total_steps=6)
    return provider, sync_client.pushed


def test_base_resync_on_intervention_defaults_on_and_must_be_a_bool():
    policy = residual_agent(ResidualRLPD).policy
    assert _actor_loop(policy).base_resync_on_intervention is True
    with pytest.raises(TypeError, match="base_resync_on_intervention"):
        _actor_loop(policy, base_resync_on_intervention="false")


def test_intervened_steps_reset_the_provider_before_the_base_of_their_next_obs():
    provider, pushed = _run_with_interventions()

    assert [t["intervened"] for t in pushed] == _INTERVENED_FLAGS
    assert provider.seen == _PROVIDER_QUERIES
    # One reset after each intervened step plus the episode-end reset after step 5.
    assert provider.reset_calls == len(_INTERVENE_STEPS) + 1
    # The bases of obs 2 and obs 3 (the next obs of intervened steps 2 and 3) are computed
    # after that step's reset; the onset step's own base (obs 1) predates the takeover.
    assert provider.reset_calls_at_select == [0, 0, 1, 2, 2, 2, 3, 3]


def test_disabled_resync_resets_the_provider_at_episode_ends_only():
    provider, pushed = _run_with_interventions(base_resync_on_intervention=False)

    assert [t["intervened"] for t in pushed] == _INTERVENED_FLAGS
    assert provider.seen == _PROVIDER_QUERIES
    assert provider.reset_calls == 1
    assert provider.reset_calls_at_select == [0, 0, 0, 0, 0, 0, 1, 1]


def test_resync_leaves_the_stored_fields_of_a_stateless_provider_unchanged():
    _, resynced = _run_with_interventions(base_resync_on_intervention=True)
    _, not_resynced = _run_with_interventions(base_resync_on_intervention=False)

    def base(marker):
        return torch.full((1, ACTION_DIM), RecordingProvider.value_for(marker))

    expected = [(101.0, 1.0), (1.0, 2.0), (2.0, 3.0), (3.0, 4.0), (4.0, 5.0), (102.0, 6.0)]
    assert len(resynced) == len(not_resynced) == len(expected)
    for with_resync, without_resync, (base_marker, next_marker) in zip(resynced, not_resynced, expected):
        assert with_resync["intervened"] is without_resync["intervened"]
        for key in ("action", "policy_action", "base_actions", "next_base_actions"):
            assert torch.equal(with_resync[key], without_resync[key]), key
        torch.testing.assert_close(with_resync["base_actions"], base(base_marker))
        torch.testing.assert_close(with_resync["next_base_actions"], base(next_marker))
    human = torch.full((1, ACTION_DIM), DictEnv.HUMAN_ACTION_VALUE)
    for transition, intervened in zip(resynced, _INTERVENED_FLAGS):
        assert torch.equal(transition["action"], human) is intervened


# -- learner loop ----------------------------------------------------------------------


def _residual_transition(intervened=False, **extra) -> dict:
    row = residual_row(intervened=intervened, base=0.3)
    row.update(extra)
    return row


def _learner_loop(agent, tmp_path, **kwargs) -> ResidualLearnerLoop:
    return ResidualLearnerLoop(agent, "127.0.0.1", 0, checkpoint_dir=str(tmp_path), **kwargs)


def _demo_agent(cls=ResidualRLPD):
    agent = residual_agent(cls)
    agent.init_demo_buffer(buffer_size=16, demo_data_ratio=0.5)
    return agent


def test_learner_buffer_kwargs_keep_every_extra_but_actor_id():
    loop = ResidualLearnerLoop.__new__(ResidualLearnerLoop)
    transition = {**_residual_transition(intervened=True), "actor_id": "robot_a"}
    core, extra = loop._buffer_kwargs(transition)
    assert set(core) == {"obs", "next_obs", "action", "reward", "done"}
    assert set(extra) == {"intervened", "policy_action", "base_actions", "next_base_actions"}


def test_learner_routes_residual_transitions_into_both_buffers(tmp_path):
    agent = _demo_agent()
    loop = _learner_loop(agent, tmp_path, buffer_period=1000)
    loop._on_transition(_residual_transition(intervened=False))
    loop._on_transition({**_residual_transition(intervened=True), "actor_id": "robot_a"})

    assert len(agent.replay_buffer) == 2
    assert len(agent.offline_replay_buffer) == 1
    assert agent.replay_buffer.intervened[:2, 0].tolist() == [False, True]
    torch.testing.assert_close(agent.replay_buffer.base_actions[1, 0], torch.full((ACTION_DIM,), 0.3))
    assert all("actor_id" not in t and "intervened" in t for t in loop._pending_online)


def test_learner_snapshots_keep_the_extras_across_a_restart(tmp_path):
    agent = _demo_agent()
    loop = _learner_loop(agent, tmp_path, buffer_period=2)
    loop._on_transition(_residual_transition(intervened=False))
    loop._on_transition(_residual_transition(intervened=True))

    reloaded = _demo_agent()
    reloaded_loop = _learner_loop(reloaded, tmp_path, buffer_period=2)
    assert reloaded_loop.received_transitions == 2
    assert len(reloaded.replay_buffer) == 2
    assert len(reloaded.offline_replay_buffer) == 1
    torch.testing.assert_close(reloaded.replay_buffer.next_base_actions[:2], agent.replay_buffer.next_base_actions[:2])
    assert reloaded.replay_buffer.intervened[:2, 0].tolist() == [False, True]


def _write_demo_pkl(demo_dir, keys=("obs", "next_obs", "action", "reward", "done")):
    demo_dir.mkdir()
    rows = []
    for _ in range(3):
        row = residual_row(intervened=True, base=0.2, human_offset=0.4)
        rows.append({key: row[key] for key in keys})
    with open(demo_dir / "session.pkl", "wb") as f:
        pickle.dump(rows, f)
    return str(demo_dir / "*.pkl")


def test_base_less_demo_pkls_are_rejected_by_default(tmp_path):
    pattern = _write_demo_pkl(tmp_path / "demos")
    with pytest.raises(ValueError, match=r"session\.pkl.*base_actions.*next_base_actions"):
        _learner_loop(_demo_agent(), tmp_path / "ckpt", demo_dataset_paths=[pattern])


def test_base_less_demo_pkls_load_with_zero_base_when_allowed(tmp_path):
    pattern = _write_demo_pkl(tmp_path / "demos")
    agent = _demo_agent()
    with pytest.warns(UserWarning, match="demo transitions without") as record:
        _learner_loop(agent, tmp_path / "ckpt", demo_dataset_paths=[pattern], allow_base_less_demos=True)

    assert sum("demo transitions without" in str(w.message) for w in record) == 1
    demo = agent.offline_replay_buffer
    assert len(demo) == 3
    assert bool(demo.intervened[:3].all())
    torch.testing.assert_close(demo.policy_actions[:3], demo.actions[:3])
    assert torch.equal(demo.base_actions[:3], torch.zeros(3, 1, ACTION_DIM))
    assert torch.equal(demo.next_base_actions[:3], torch.zeros(3, 1, ACTION_DIM))


def test_demo_pkls_with_base_actions_load_as_human_actions_without_a_warning(tmp_path, recwarn):
    keys = ("obs", "next_obs", "action", "reward", "done", "base_actions", "next_base_actions")
    pattern = _write_demo_pkl(tmp_path / "demos", keys)
    agent = _demo_agent()
    _learner_loop(agent, tmp_path / "ckpt", demo_dataset_paths=[pattern])

    demo = agent.offline_replay_buffer
    assert len(demo) == 3
    assert bool(demo.intervened[:3].all())
    torch.testing.assert_close(demo.policy_actions[:3], demo.actions[:3])
    torch.testing.assert_close(demo.base_actions[:3], torch.full((3, 1, ACTION_DIM), 0.2))
    assert not any("demo transitions without" in str(w.message) for w in recwarn)
