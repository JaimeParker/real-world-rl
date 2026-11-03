"""HIL-SERL's ActorLoop/LearnerLoop/sync classes -- mirrors
test_real_world_serl_loops.py (base-class behavior is covered by
test_real_world_actor_loop.py / test_real_world_learner_loop.py /
test_real_world_sync.py). Unlike SERL, HilSerlActorLoop/HilSerlLearnerLoop
override behavior (intervention tagging, demo-buffer routing, crash-recovery
snapshotting), covered below."""
from __future__ import annotations

import glob
import os
import pickle

import torch
from gymnasium import spaces

from rl_garden.algorithms.rlpd_hybrid import RLPDHybrid
from real_world_rl import (
    ActorSyncClient,
    LearnerSyncServer,
)
from real_world_rl.loops.actor import ActorLoop
from real_world_rl.loops.hil_serl import (
    HilSerlActorLoop,
    HilSerlActorSyncClient,
    HilSerlLearnerLoop,
    HilSerlLearnerSyncServer,
)
from real_world_rl.loops.learner import LearnerLoop


class _DummyVecEnv:
    num_envs = 1

    def __init__(self) -> None:
        import numpy as np

        self.single_observation_space = spaces.Box(-1.0, 1.0, (4,), dtype=np.float32)
        self.single_action_space = spaces.Box(-1.0, 1.0, (3,), dtype=np.float32)
        self.action_space = spaces.Box(
            low=self.single_action_space.low[None],
            high=self.single_action_space.high[None],
            dtype=np.float32,
        )

    def reset(self, seed=None):
        del seed
        return torch.zeros(1, 4), {}

    def step(self, actions):
        return (
            torch.randn(1, 4),
            torch.ones(1),
            torch.zeros(1, dtype=torch.bool),
            torch.zeros(1, dtype=torch.bool),
            {},
        )

    def close(self) -> None:
        return None


def _agent() -> RLPDHybrid:
    return RLPDHybrid(
        env=_DummyVecEnv(),
        device="cpu",
        buffer_device="cpu",
        buffer_size=32,
        batch_size=4,
        learning_starts=1,
        training_freq=4,
        eval_freq=0,
        log_freq=0,
        net_arch=[8],
        discrete_hidden_dim=8,
    )


def _transition(intervened: bool = False) -> dict:
    return {
        "obs": torch.zeros(1, 4),
        "next_obs": torch.zeros(1, 4),
        "action": torch.zeros(1, 3),
        "reward": torch.ones(1),
        "done": torch.zeros(1, dtype=torch.bool),
        "intervened": intervened,
    }


def test_hil_serl_loop_and_sync_classes_subclass_the_algorithm_agnostic_bases():
    assert issubclass(HilSerlActorLoop, ActorLoop)
    assert issubclass(HilSerlLearnerLoop, LearnerLoop)
    assert issubclass(HilSerlActorSyncClient, ActorSyncClient)
    assert issubclass(HilSerlLearnerSyncServer, LearnerSyncServer)


def test_hil_serl_sync_client_constructs_like_the_base():
    client = HilSerlActorSyncClient("http://127.0.0.1:6000")
    assert client._base_url == "http://127.0.0.1:6000"


def test_hil_serl_actor_loop_tags_intervened_from_info():
    loop = HilSerlActorLoop.__new__(HilSerlActorLoop)
    assert loop._extra_transition_fields({"intervene_action": torch.zeros(3)}) == {
        "intervened": True
    }
    assert loop._extra_transition_fields({}) == {"intervened": False}


def test_hil_serl_actor_loop_forwards_grasp_penalty_when_present():
    loop = HilSerlActorLoop.__new__(HilSerlActorLoop)
    penalty = torch.tensor([-0.05])
    fields = loop._extra_transition_fields({"grasp_penalty": penalty})
    assert fields["intervened"] is False
    torch.testing.assert_close(fields["grasp_penalty"], penalty)
    assert "grasp_penalty" not in loop._extra_transition_fields({})


def test_hil_serl_learner_loop_routes_by_intervened_flag(tmp_path):
    """Matches HIL-SERL's own actor loop (train_rlpd.py:178-193): every
    transition -- intervened or not -- goes into the online replay buffer;
    intervened transitions are *additionally* copied into the demo buffer.
    The demo buffer is a duplicated subset of the online buffer, not a
    disjoint partition of it."""
    agent = _agent()
    agent.init_demo_buffer(buffer_size=16, demo_data_ratio=0.5)
    loop = HilSerlLearnerLoop(agent, "127.0.0.1", 0, checkpoint_dir=str(tmp_path), buffer_period=1000)

    loop._on_transition(_transition(intervened=False))
    loop._on_transition(_transition(intervened=True))
    loop._on_transition(_transition(intervened=True))

    assert len(agent.replay_buffer) == 3
    assert len(agent.offline_replay_buffer) == 2
    assert loop.received_transitions == 3


def test_hil_serl_learner_loop_strips_actor_id_before_buffer_and_snapshot(tmp_path):
    agent = _agent()
    agent.init_demo_buffer(buffer_size=16, demo_data_ratio=0.5)
    loop = HilSerlLearnerLoop(agent, "127.0.0.1", 0, checkpoint_dir=str(tmp_path), buffer_period=1000)

    loop._on_transition({**_transition(intervened=True), "actor_id": "robot_a"})
    loop._on_transition({**_transition(intervened=False), "actor_id": "robot_b"})

    assert loop.received_by_actor == {"robot_a": 1, "robot_b": 1}
    # actor_id must never survive into the pending-snapshot lists (it would
    # otherwise get pickled into crash-recovery buffer/demo_buffer files).
    assert all("actor_id" not in t for t in loop._pending_online)
    assert all("actor_id" not in t for t in loop._pending_demo)


def test_hil_serl_learner_loop_snapshots_and_reloads_on_restart(tmp_path):
    agent = _agent()
    agent.init_demo_buffer(buffer_size=16, demo_data_ratio=0.5)
    loop = HilSerlLearnerLoop(agent, "127.0.0.1", 0, checkpoint_dir=str(tmp_path), buffer_period=2)

    loop._on_transition(_transition(intervened=False))
    loop._on_transition(_transition(intervened=True))  # crosses buffer_period=2 -> snapshot

    assert glob.glob(os.path.join(str(tmp_path), "buffer", "*.pkl"))
    assert glob.glob(os.path.join(str(tmp_path), "demo_buffer", "*.pkl"))

    reloaded_agent = _agent()
    reloaded_agent.init_demo_buffer(buffer_size=16, demo_data_ratio=0.5)
    reloaded_loop = HilSerlLearnerLoop(
        reloaded_agent, "127.0.0.1", 0, checkpoint_dir=str(tmp_path), buffer_period=2
    )

    # "buffer" snapshot holds both transitions (unconditional); "demo_buffer"
    # holds the one intervened transition, duplicated -- not a remainder.
    assert len(reloaded_agent.replay_buffer) == 2
    assert len(reloaded_agent.offline_replay_buffer) == 1
    # Received-count is off the "buffer" reload only, so the intervened
    # transition isn't counted twice.
    assert reloaded_loop.received_transitions == 2


def test_hil_serl_learner_loop_preloads_demo_dataset_paths(tmp_path):
    demo_dir = tmp_path / "demos"
    demo_dir.mkdir()
    demo_transitions = [_transition() for _ in range(3)]
    for t in demo_transitions:
        t.pop("intervened")
    with open(demo_dir / "session1.pkl", "wb") as f:
        pickle.dump(demo_transitions, f)

    agent = _agent()
    agent.init_demo_buffer(buffer_size=16, demo_data_ratio=0.5)
    loop = HilSerlLearnerLoop(
        agent,
        "127.0.0.1",
        0,
        checkpoint_dir=str(tmp_path / "ckpt"),
        buffer_period=1000,
        demo_dataset_paths=[str(demo_dir / "*.pkl")],
    )

    assert len(agent.offline_replay_buffer) == 3
    # Preloaded demo transitions don't count toward received_transitions --
    # that gates learning_starts, and HIL-SERL's own analogous gate never
    # counts --demo_path preload either, only genuine online transitions.
    assert loop.received_transitions == 0

    # Growing via live intervention keeps adding to the same buffer/slot.
    loop._on_transition(_transition(intervened=True))
    assert len(agent.offline_replay_buffer) == 4
    assert loop.received_transitions == 1


def test_hil_serl_learner_loop_reloads_demo_dataset_paths_on_every_restart(tmp_path):
    demo_dir = tmp_path / "demos"
    demo_dir.mkdir()
    demo_transitions = [_transition() for _ in range(2)]
    for t in demo_transitions:
        t.pop("intervened")
    with open(demo_dir / "session1.pkl", "wb") as f:
        pickle.dump(demo_transitions, f)

    checkpoint_dir = str(tmp_path / "ckpt")
    agent = _agent()
    agent.init_demo_buffer(buffer_size=16, demo_data_ratio=0.5)
    HilSerlLearnerLoop(
        agent,
        "127.0.0.1",
        0,
        checkpoint_dir=checkpoint_dir,
        buffer_period=1000,
        demo_dataset_paths=[str(demo_dir / "*.pkl")],
    )
    assert len(agent.offline_replay_buffer) == 2

    # Restart: mirrors HIL-SERL's own always-reload-demo_path behavior.
    reloaded_agent = _agent()
    reloaded_agent.init_demo_buffer(buffer_size=16, demo_data_ratio=0.5)
    HilSerlLearnerLoop(
        reloaded_agent,
        "127.0.0.1",
        0,
        checkpoint_dir=checkpoint_dir,
        buffer_period=1000,
        demo_dataset_paths=[str(demo_dir / "*.pkl")],
    )
    assert len(reloaded_agent.offline_replay_buffer) == 2


def test_hil_serl_learner_loop_learner_scalars_count_demo_rows_and_intervened_transitions(tmp_path):
    agent = _agent()
    agent.init_demo_buffer(buffer_size=16, demo_data_ratio=0.5)
    loop = HilSerlLearnerLoop(agent, "127.0.0.1", 0, checkpoint_dir=str(tmp_path), buffer_period=1000)

    loop._on_transition(_transition(intervened=False))
    loop._on_transition(_transition(intervened=True))
    loop._on_transition(_transition(intervened=True))

    assert {key: loop._learner_scalars()[key] for key in ("received", "update", "buffer_size", "demo_buffer_size", "intervened_received")} == {
        "received": 3,
        "update": 0,
        "buffer_size": 3,
        "demo_buffer_size": 2,
        "intervened_received": 2,
    }


# -- actor episode statistics ------------------------------------------------------------


class _ScriptedEpisodeEnv:
    """Steps through ``script`` rows ``(intervened, terminated, human_end, safety_clips,
    rtt_ms)``; a human end truncates the episode, as ``InterventionWrapper`` does."""

    num_envs = 1

    def __init__(self, script):
        self.script = list(script)
        self.steps = 0

    def reset(self, *, seed=None):
        del seed
        return torch.zeros(1, 4), {}

    def step(self, action):
        intervened, terminated, human_end, safety_clips, rtt_ms = self.script[self.steps]
        self.steps += 1
        info = {"human_episode_end": human_end, "safety_clips": safety_clips, "rtt_ms": rtt_ms}
        if intervened:
            info["intervene_action"] = torch.full((1, 3), 0.5)
        return (
            torch.zeros(1, 4),
            torch.ones(1),
            torch.tensor([terminated]),
            torch.tensor([human_end]),
            info,
        )


class _ConstantPolicy:
    def eval(self):
        pass

    def load_state_dict(self, state_dict):
        del state_dict

    def predict(self, obs, deterministic=False):
        del obs, deterministic
        return torch.zeros(1, 3)


class _StatsSyncClient:
    dropped_transitions = 4

    def __init__(self):
        self.pushed: list[dict] = []
        self.posted: list[dict] = []

    def start(self):
        pass

    def stop(self):
        pass

    def push_transition(self, transition):
        self.pushed.append(transition)

    def post_stats(self, stats):
        self.posted.append(stats)

    def latest_policy_params(self):
        return {"weights": torch.zeros(1)}


def expected_keys(stats, timing_keys):
    outcome_keys = {"success_rate", "autonomous_success_rate", "task_success_rate",
                    "episode_intervention_rate", "step_intervention_rate", "completed_episodes",
                    "unknown_episodes", "interrupted_episodes", "discarded_episodes", "tcp_proxy_success_rate",
                    "task_success", "outcome_source", "end_reason"}
    return stats.keys() - timing_keys - outcome_keys


def test_hil_serl_actor_loop_posts_episode_statistics_at_every_episode_end():
    script = [
        # Episode 1: takeovers on steps 2-3 and 5, success on step 5.
        (False, False, False, 3, 10.0),
        (True, False, False, 3, 20.0),
        (True, False, False, 3, 30.0),
        (False, False, False, 3, 40.0),
        (True, True, False, 4, 50.0),
        # Episode 2: no takeover, the operator ends it on step 2 (Esc).
        (False, False, False, 4, None),
        (False, False, True, 6, None),
    ]
    sync_client = _StatsSyncClient()
    loop = HilSerlActorLoop(
        _ScriptedEpisodeEnv(script), _ConstantPolicy(), sync_client, control_hz=1_000_000.0
    )

    loop.run(total_steps=len(script))

    assert len(sync_client.pushed) == 7
    first, second = sync_client.posted
    timing_keys = {
        "step_time_ms_mean",
        "step_time_ms_max",
        "param_refresh_ms_mean",
        "param_refresh_ms_max",
    }
    assert {key: first[key] for key in expected_keys(first, timing_keys)} == {
        "episode": 1,
        "return": 5.0,
        "length": 5,
        "intervened_steps": 3,
        "intervention_count": 2,
        "intervention_rate": 0.6,
        "terminated": 1.0,
        "human_episode_end": 0.0,
        "safety_clips": 1,
        "safety_clips_total": 4,
        "rtt_ms_mean": 30.0,
        "rtt_ms_max": 50.0,
        "dropped_transitions": 4,
    }
    assert {key: second[key] for key in expected_keys(second, timing_keys)} == {
        "episode": 2,
        "return": 2.0,
        "length": 2,
        "intervened_steps": 0,
        "intervention_count": 0,
        "intervention_rate": 0.0,
        "terminated": 0.0,
        "human_episode_end": 1.0,
        # Measured from the last reading of episode 1.
        "safety_clips": 2,
        "safety_clips_total": 6,
        "dropped_transitions": 4,
    }
    assert first["task_success"] is None  # Termination alone is not a task label.
    assert first["autonomous_success_rate"] == 0
    assert first["tcp_proxy_success_rate"] == 0
    assert first["episode_intervention_rate"] == 1
    assert second["unknown_episodes"] == 2
    for stats in (first, second):
        assert timing_keys <= set(stats)
        assert 0.0 <= stats["param_refresh_ms_max"] <= stats["step_time_ms_max"]
        assert stats["step_time_ms_mean"] <= stats["step_time_ms_max"]


def test_hil_serl_actor_loop_skips_posting_for_a_client_without_post_stats():
    class _PushOnlyClient(_StatsSyncClient):
        post_stats = None

    sync_client = _PushOnlyClient()
    loop = HilSerlActorLoop(
        _ScriptedEpisodeEnv([(False, True, False, None, None)]),
        _ConstantPolicy(),
        sync_client,
        control_hz=1_000_000.0,
    )

    loop.run(total_steps=1)

    assert len(sync_client.pushed) == 1
    assert sync_client.posted == []
