"""Partial imitation supervises only observed human action dimensions."""
from types import SimpleNamespace
import numpy as np
import pytest
import torch
from gymnasium import spaces
from real_world_rl.algorithms.masked_imitation import masked_sum_mean, marginal_gripper_ce
from real_world_rl.buffers.intervention_replay_buffer import InterventionReplayBuffer
from real_world_rl.teleop.action_parts import action_parts, part_mask
from real_world_rl.trajectory import begin_episode, transition_record, end_episode, validate_record


@pytest.mark.parametrize("width", [1, 7, 14, 16])
def test_full_mask_preserves_sum_nll_and_gradients(width):
    values = torch.randn(3, width, requires_grad=True)
    old = values.sum(-1).mean()
    new = masked_sum_mean(values, torch.ones_like(values, dtype=torch.bool))
    torch.testing.assert_close(new, old)
    torch.testing.assert_close(torch.autograd.grad(new, values, retain_graph=True)[0],
                               torch.autograd.grad(old, values)[0])


def test_partial_and_empty_masks_have_no_unobserved_label_gradient():
    values = torch.randn(2, 4, requires_grad=True)
    mask = torch.tensor([[True, False, True, False], [False] * 4])
    loss = masked_sum_mean(values, mask)
    gradient, = torch.autograd.grad(loss, values)
    torch.testing.assert_close(gradient, torch.tensor([[2., 0., 2., 0.], [0.] * 4]))
    empty = masked_sum_mean(values, torch.zeros_like(mask))
    assert empty.requires_grad and empty.item() == 0
    assert not torch.autograd.grad(empty, values)[0].any()


def test_joint_gripper_partial_ce_matches_enumerated_probability():
    from rl_garden.policies.rlpd_hybrid_policy import gripper_index_digits
    logits = torch.randn(2, 9, requires_grad=True)
    labels = torch.tensor([5, 7])
    mask = torch.tensor([[True, False], [False, True]])
    digits = gripper_index_digits(torch.arange(9), 2)
    targets = gripper_index_digits(labels, 2)
    terms = []
    for row in range(2):
        matches = (digits[:, mask[row]] == targets[row, mask[row]]).all(-1)
        terms.append(-logits[row].softmax(-1)[matches].sum().log())
    torch.testing.assert_close(marginal_gripper_ce(logits, labels, mask, 2), torch.stack(terms).mean())
    torch.testing.assert_close(marginal_gripper_ce(logits, labels, torch.ones_like(mask), 2),
                               torch.nn.functional.cross_entropy(logits, labels))
    empty = marginal_gripper_ce(logits, labels, torch.zeros_like(mask), 2)
    assert empty.item() == 0
    assert not torch.autograd.grad(empty, logits)[0].any()


@pytest.mark.parametrize("grippers", [False, True])
def test_named_parts_preserve_bimanual_layout(grippers):
    contract = dict(num_arms=2, arm_names=["left", "right"], arm_action_dim=7,
                    gripper_in_action=grippers, action_mode="joint_delta")
    layout = action_parts(contract)
    assert layout["right.arm"] == slice(7, 14)
    if grippers:
        assert layout["left.gripper"] == slice(14, 15)
    assert part_mask(contract, ["left.arm"]).sum() == 7
    with pytest.raises(ValueError, match="unknown action part"):
        part_mask(contract, ["head.arm"])


def test_explicit_empty_mask_overrides_legacy_intervened_true_and_survives_sample():
    obs = spaces.Dict({"state": spaces.Box(-1, 1, (2,), dtype=np.float32)})
    action = spaces.Box(-1, 1, (3,), dtype=np.float32)
    replay = InterventionReplayBuffer(obs, action, num_envs=1, buffer_size=4, storage_device="cpu", sample_device="cpu")
    state = {"state": torch.zeros(1, 2)}
    replay.add(state, state, torch.zeros(1, 3), torch.zeros(1), torch.zeros(1),
               intervened=True, human_action_mask=torch.zeros(1, 3, dtype=torch.bool),
               override_mask=torch.ones(1, 3, dtype=torch.bool))
    sample = replay._index_batch(torch.tensor([0]), torch.tensor([0]))
    assert not sample.intervened.any() and not sample.human_action_mask.any()
    assert sample.override_mask.all()
    assert "human_action_mask" in replay.slice_keys()


def test_archive_records_preserve_boundaries_and_mask_dimensions():
    row = dict(obs={"state": torch.zeros(1, 2)}, next_obs={"state": torch.ones(1, 2)},
               action=torch.zeros(1, 3), reward=torch.zeros(1), done=torch.zeros(1))
    records = [begin_episode("episode"), transition_record(row, episode_id="episode", step_index=0,
               human_action_mask=torch.tensor([[True, False, False]])),
               end_episode("episode", step_count=1, interrupted=True)]
    for record in records:
        validate_record(record)
    assert records[-1]["interrupted"]
    with pytest.raises(ValueError, match="human_action_mask"):
        transition_record(row, episode_id="episode", step_index=1, human_action_mask=torch.ones(4))

@pytest.mark.parametrize("grippers", [False, True])
@pytest.mark.parametrize("idle", ["hold", "policy"])
def test_partial_takeover_preserves_policy_or_holds_unselected_parts(grippers, idle):
    import gymnasium as gym
    from real_world_rl.teleop.base import TeleopSource, TeleopReading
    from real_world_rl.envs.wrappers.intervention import InterventionWrapper
    width = 6 if grippers else 4
    class Env(gym.Env):
        num_envs = 1
        action_contract = dict(action_mode="joint_delta", arm_action_dim=2, num_arms=2,
                               arm_names=["left", "right"], gripper_in_action=grippers)
        single_action_space = spaces.Box(-1, 1, (width,), dtype=np.float32)
        action_space = single_action_space
        observation_space = spaces.Dict({"state": spaces.Box(-1, 1, (2,), dtype=np.float32)})
        def reset(self, **kwargs): return {"state": torch.zeros(1, 2)}, {}
        def step(self, action):
            self.sent = action
            return {"state": torch.zeros(1, 2)}, torch.zeros(1), False, False, {}
    class Source(TeleopSource):
        action_mode = "joint_delta"
        arm_action_dim = 2
        has_gripper = grippers
        num_arms = 2
        arm_names = ["left", "right"]
        def poll(self): return TeleopReading(np.full(width, .8), True)
    raw = Env()
    env = InterventionWrapper(raw, Source(), selected_parts=["left.arm"], idle_behavior=idle)
    env.reset()
    _, _, _, _, info = env.step(torch.full((1, width), -.4))
    torch.testing.assert_close(raw.sent[0, :2], torch.full((2,), .8))
    expected = torch.full((width - 2,), -.4 if idle == "policy" else 0.)
    if grippers and idle == "hold": expected[-2:] = 1
    torch.testing.assert_close(raw.sent[0, 2:], expected)
    assert info["human_action_mask"].sum() == 2
    assert info["override_mask"].sum() == (width if idle == "hold" else 2)


@pytest.mark.parametrize("squash", [False, True])
def test_local_bc_full_mask_matches_old_loss_and_actor_gradient(squash):
    from real_world_rl.algorithms.bc import MaskedBC
    from rl_garden.policies.bc_policy import BCPolicy
    from rl_garden.encoders.base import BaseFeaturesExtractor
    from torch import nn
    class Extractor(BaseFeaturesExtractor):
        def __init__(self, space):
            super().__init__(space, features_dim=2)
        def forward(self, obs): return obs["state"]
    obs_space = spaces.Dict({"state": spaces.Box(-1, 1, (2,), dtype=np.float32)})
    policy = BCPolicy(obs_space, spaces.Box(-1, 1, (3,), dtype=np.float32),
                      actor_extractor=Extractor(obs_space), net_arch=(8,), tanh_squash=squash)
    data = SimpleNamespace(obs={"state": torch.randn(4, 2)}, actions=torch.rand(4, 3) * 1.8 - .9,
                           intervened=torch.ones(4, dtype=torch.bool), human_action_mask=torch.ones(4, 3, dtype=torch.bool))
    agent = MaskedBC.__new__(MaskedBC)
    agent.policy, agent.tanh_squash = policy, squash
    old_log_prob, _ = policy.behavior_log_prob(data.obs, data.actions, stop_gradient=False)
    old = -old_log_prob.mean()
    new, _ = agent._compute_losses(data)
    torch.testing.assert_close(new, old)
    params = tuple(policy.parameters())
    first = torch.autograd.grad(new, params, retain_graph=True, allow_unused=True)
    second = torch.autograd.grad(old, params, allow_unused=True)
    for a, b in zip(first, second):
        if a is not None: torch.testing.assert_close(a, b)


def test_durable_archive_reopens_unfinished_episode_as_interrupted(tmp_path):
    from real_world_rl.trajectory import DurableArchive
    path = tmp_path / "archive.sqlite3"
    archive = DurableArchive(path)
    archive.append(begin_episode("episode"))
    archive.close()
    recovered = DurableArchive(path)
    try:
        assert [row["record_type"] for row in recovered] == ["episode_start", "episode_end"]
        assert recovered[-1]["interrupted"]
    finally:
        recovered.close()


def test_sqlite_demo_view_uses_only_complete_successful_nonsync_human_rows(tmp_path):
    from real_world_rl.trajectory import DurableArchive
    from real_world_rl.training.demo_loading import load_demo_pkls
    path = tmp_path / "demos.trajectory.sqlite3"
    archive = DurableArchive(path)
    for episode, success, interrupted in (("good", True, False), ("bad", False, False), ("partial", True, True)):
        archive.append(begin_episode(episode))
        for step in range(2):
            row = dict(obs={"state": torch.zeros(1, 2)}, next_obs={"state": torch.ones(1, 2)},
                       action=torch.zeros(1, 3), reward=torch.zeros(1), done=torch.zeros(1))
            archive.append(transition_record(row, episode_id=episode, step_index=step,
                human_action_mask=torch.tensor([[True, False, False]]), syncing=step == 1))
        archive.append(end_episode(episode, step_count=2, interrupted=interrupted,
            outcome=dict(task_success=success, completed=not interrupted, interrupted=interrupted, discarded=False)))
    archive.close()
    received = []
    def add(obs, next_obs, action, reward, done, **extra): received.append(extra)
    assert load_demo_pkls([str(path)], add, "cpu") == 1
    assert received[0]["human_action_mask"].sum() == 1


def test_syncing_projection_disables_supervision_without_mutating_archive():
    from real_world_rl.trajectory import training_projection
    record = dict(intervened=True, syncing=True, human_action_mask=torch.ones(1, 3, dtype=torch.bool),
                  override_mask=torch.ones(1, 3, dtype=torch.bool))
    projected = training_projection(record)
    assert not projected["human_action_mask"].any() and not projected["intervened"]
    assert record["human_action_mask"].all() and record["intervened"]


def test_truncation_training_boundary_keeps_bootstrap_done_false():
    from real_world_rl.trajectory import training_projection
    projected = training_projection(dict(done=torch.tensor([False]), terminated=False, truncated=True))
    assert projected["episode_end"] and not projected["done"].any()
    segment = training_projection(dict(done=False, terminated=False, truncated=False, training_segment_end=True))
    assert segment["episode_end"] and not segment["done"]


def test_masked_bc_loads_legacy_bc_checkpoint(tmp_path):
    from rl_garden.algorithms.bc import BC
    from real_world_rl.algorithms.bc import MaskedBC
    env = SimpleNamespace(num_envs=1,
        single_observation_space=spaces.Dict({"state": spaces.Box(-1, 1, (2,), dtype=np.float32)}),
        single_action_space=spaces.Box(-1, 1, (3,), dtype=np.float32))
    kwargs = dict(env=env, buffer_size=8, buffer_device="cpu", device="cpu", net_arch=(8,), batch_size=2)
    old = BC(**kwargs)
    path = tmp_path / "legacy.pt"
    old.save(path, include_replay_buffer=False)
    current = MaskedBC(**kwargs)
    current.load(path, load_replay_buffer=False)
    obs = {"state": torch.randn(2, 2)}
    torch.testing.assert_close(current.policy.predict(obs, deterministic=True), old.policy.predict(obs, deterministic=True))


@pytest.mark.parametrize("failure", ["validate", "start"])
@pytest.mark.parametrize("close_fails", [False, True])
def test_intervention_construction_releases_source_and_env_preserving_error(failure, close_fails):
    import gymnasium as gym
    from real_world_rl.teleop.base import TeleopSource
    from real_world_rl.envs.wrappers.intervention import InterventionWrapper
    class Env(gym.Env):
        num_envs = 1
        action_contract = dict(action_mode="joint_delta", arm_action_dim=2, gripper_in_action=False)
        single_action_space = spaces.Box(-1, 1, (2,), dtype=np.float32)
        action_space = single_action_space
        observation_space = spaces.Dict({"state": spaces.Box(-1, 1, (2,), dtype=np.float32)})
        closed = False
        def close(self): self.closed = True
    class Source(TeleopSource):
        action_mode = "joint_abs" if failure == "validate" else "joint_delta"
        arm_action_dim = 2
        has_gripper = False
        opened = closed = False
        def poll(self): raise AssertionError("must not poll")
        def start(self):
            self.opened = True
            raise RuntimeError("start failed after opening")
        def close(self):
            self.closed = True
            if close_fails: raise OSError("secondary cleanup failure")
    env, source = Env(), Source()
    with pytest.raises(ValueError if failure == "validate" else RuntimeError,
                       match="action_mode" if failure == "validate" else "start failed"):
        InterventionWrapper(env, source)
    assert source.closed and env.closed
    assert source.opened == (failure == "start")


def test_no_gripper_continuous_hil_keeps_growing_demo_buffer():
    from _residual_fakes import DictEnv
    from real_world_rl.algorithms.hil_continuous import ContinuousHilRLPD
    env = DictEnv(action_dim=2)
    agent = ContinuousHilRLPD(env=env, device="cpu", buffer_device="cpu", buffer_size=16,
        batch_size=2, learning_starts=1, net_arch=[8], n_critics=2, utd=1, log_freq=0, eval_freq=0)
    agent.init_demo_buffer(8, .5)
    for index in range(4):
        obs = {"state": torch.rand(1, 4)}
        args = (obs, obs, torch.rand(1, 2), torch.ones(1), torch.zeros(1))
        agent.replay_buffer.add(*args)
        agent.add_demo_transition(*args)
    assert len(agent.offline_replay_buffer) == 4
    agent.train(1)
    assert agent.policy.predict({"state": torch.rand(1, 4)}, deterministic=True).shape == (1, 2)


def test_explicit_termination_controls_bootstrap_and_legacy_done_survives():
    from real_world_rl.trajectory import training_projection
    assert not training_projection(dict(done=True, terminated=False, truncated=True))["done"]
    assert training_projection(dict(done=False, terminated=True, truncated=False))["done"]
    assert training_projection(dict(done=True))["done"]


def test_durable_archive_sequence_increases_after_restart(tmp_path):
    from real_world_rl.trajectory import DurableArchive, begin_episode, end_episode
    path = tmp_path / "sequence.trajectory.sqlite3"
    archive = DurableArchive(path)
    archive.append(begin_episode("one"))
    archive.close()
    archive = DurableArchive(path)
    archive.append(begin_episode("two"))
    archive.append(end_episode("two", step_count=0))
    rows = list(archive)
    archive.close()
    assert [row["sequence"] for row in rows] == list(range(len(rows)))


def test_masked_bc_empty_batch_preserves_adam_momentum(monkeypatch):
    from real_world_rl.algorithms.bc import MaskedBC
    from _silri_fakes import SpacesEnv
    agent = MaskedBC(SpacesEnv(), device="cpu", buffer_device="cpu", buffer_size=8,
                     batch_size=2, std_log=False, log_freq=0)
    data = SimpleNamespace(obs={"state": torch.randn(2, 4)}, actions=torch.zeros(2, 3),
                           intervened=torch.ones(2, dtype=torch.bool),
                           human_action_mask=torch.ones(2, 3, dtype=torch.bool))
    monkeypatch.setattr(agent, "_sample_train_batch", lambda: data)
    agent.train(1)
    params = [param.detach().clone() for param in agent.policy.parameters()]
    steps = [state["step"].clone() for state in agent.actor_optimizer.state.values()]
    data.human_action_mask.zero_()
    assert agent.train(1)["loss"] == 0
    for param, expected in zip(agent.policy.parameters(), params):
        torch.testing.assert_close(param, expected, rtol=0, atol=0)
    for state, expected in zip(agent.actor_optimizer.state.values(), steps):
        torch.testing.assert_close(state["step"], expected, rtol=0, atol=0)


@pytest.mark.parametrize("dtype", [torch.bool, torch.float32])
def test_explicit_termination_projection_preserves_tensor_replay_contract(dtype):
    from real_world_rl.trajectory import training_projection
    done = torch.ones(1, dtype=dtype)
    projected = training_projection(dict(done=done, terminated=False, truncated=True))
    assert isinstance(projected["done"], torch.Tensor)
    assert projected["done"].shape == done.shape
    assert projected["done"].dtype == done.dtype
    assert projected["done"].device == done.device
    assert not projected["done"].any()
    assert projected["episode_end"].any()
