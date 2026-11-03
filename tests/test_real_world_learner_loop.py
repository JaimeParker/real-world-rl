"""Tests for LearnerLoop against a fake OffPolicyAlgorithm double.

LearnerLoop is meant to be algorithm-agnostic: it only ever touches
``replay_buffer.add``, ``train``, ``learning_starts``, ``utd``,
``buffer_device``, ``policy.state_dict``, and the checkpoint-related surface
(``checkpoint_dir``, ``checkpoint_freq``, ``save_replay_buffer``,
``save_final_checkpoint``, ``global_update``, ``save(...)``) -- the fake
below exposes exactly that surface, nothing more, so these tests double as a
check that LearnerLoop doesn't reach for anything beyond the documented
public interface of ``OffPolicyAlgorithm``.
"""
from __future__ import annotations

import threading
import time

import torch

from real_world_rl.loops.learner import LearnerLoop
from real_world_rl.loops.sync import ActorSyncClient


class _FakeReplayBuffer:
    def __init__(self):
        self.add_calls = []

    def add(self, obs, next_obs, action, reward, done, **kwargs):
        self.add_calls.append(
            dict(obs=obs, next_obs=next_obs, action=action, reward=reward, done=done, **kwargs)
        )

    def __len__(self):
        return len(self.add_calls)


class _FakePolicy:
    def __init__(self):
        self.version = 0

    def state_dict(self):
        return {"version": torch.tensor([self.version])}


class _FakeAgent:
    def __init__(
        self,
        learning_starts: int = 2,
        utd: float = 1.0,
        checkpoint_dir: str | None = None,
        checkpoint_freq: int = 0,
        save_replay_buffer: bool = False,
        save_final_checkpoint: bool = True,
    ):
        self.buffer_device = "cpu"
        self.replay_buffer = _FakeReplayBuffer()
        self.learning_starts = learning_starts
        self.utd = utd
        self.policy = _FakePolicy()
        self.train_calls: list[int] = []
        self.checkpoint_dir = checkpoint_dir
        self.checkpoint_freq = checkpoint_freq
        self.save_replay_buffer = save_replay_buffer
        self.save_final_checkpoint = save_final_checkpoint
        self.global_update = 0
        self.save_calls: list[tuple[str, bool]] = []

    def train(self, gradient_steps: int, compute_info: bool = False):
        del compute_info
        self.train_calls.append(gradient_steps)
        self.global_update += gradient_steps
        self.policy.version += 1
        return {}

    def save(self, path, include_replay_buffer: bool = False):
        self.save_calls.append((str(path), include_replay_buffer))


def _transition(value: float = 0.0) -> dict[str, torch.Tensor]:
    return {
        "obs": torch.full((1, 4), value),
        "next_obs": torch.full((1, 4), value + 1),
        "action": torch.zeros(1, 2),
        "reward": torch.tensor([1.0]),
        "done": torch.tensor([False]),
    }


def test_on_transition_adds_to_replay_buffer_with_device_cast():
    agent = _FakeAgent()
    loop = LearnerLoop(agent, "127.0.0.1", 0)
    loop._on_transition(_transition(1.0))
    assert len(agent.replay_buffer.add_calls) == 1
    call = agent.replay_buffer.add_calls[0]
    torch.testing.assert_close(call["obs"], torch.full((1, 4), 1.0))
    assert call["obs"].device == torch.device("cpu")
    assert loop.received_transitions == 1


def test_on_transition_passes_extra_kwargs_through():
    agent = _FakeAgent()
    loop = LearnerLoop(agent, "127.0.0.1", 0)
    transition = _transition(0.0)
    transition["discounts"] = torch.tensor([0.99])
    loop._on_transition(transition)
    assert "discounts" in agent.replay_buffer.add_calls[0]


def test_on_transition_strips_actor_id_before_reaching_buffer():
    # actor_id is observability-only metadata -- DictReplayBuffer.add()/
    # TensorReplayBuffer.add() have fixed signatures with no **kwargs, so a
    # raw actor_id reaching .add() would TypeError.
    agent = _FakeAgent()
    loop = LearnerLoop(agent, "127.0.0.1", 0)
    transition = _transition(0.0)
    transition["actor_id"] = "robot_a"
    loop._on_transition(transition)
    assert "actor_id" not in agent.replay_buffer.add_calls[0]


def test_received_by_actor_tallies_per_actor_and_unknown():
    agent = _FakeAgent()
    loop = LearnerLoop(agent, "127.0.0.1", 0)
    loop._on_transition({**_transition(), "actor_id": "robot_a"})
    loop._on_transition({**_transition(), "actor_id": "robot_a"})
    loop._on_transition({**_transition(), "actor_id": "robot_b"})
    loop._on_transition(_transition())  # no actor_id -- default client
    assert loop.received_by_actor == {"robot_a": 2, "robot_b": 1, "unknown": 1}
    assert loop.received_transitions == 4


def test_run_waits_for_learning_starts_before_training():
    agent = _FakeAgent(learning_starts=3, utd=1.0)
    loop = LearnerLoop(agent, "127.0.0.1", 0, train_freq=1, publish_freq=1, idle_poll_interval=0.02)
    loop._on_transition(_transition())
    loop._on_transition(_transition())  # only 2 of 3 required

    thread = threading.Thread(target=loop.run, daemon=True)
    thread.start()
    try:
        time.sleep(0.2)
        assert agent.train_calls == [], "train() must not run before learning_starts"
        loop._on_transition(_transition())  # crosses learning_starts == 3
        deadline = time.time() + 5.0
        while not agent.train_calls and time.time() < deadline:
            time.sleep(0.02)
        assert agent.train_calls, "train() never ran after learning_starts was reached"
        assert agent.train_calls[0] == max(1, int(1 * agent.utd))
    finally:
        loop.stop()
        thread.join(timeout=5.0)


def test_refresh_offline_data_default_is_a_harmless_noop():
    agent = _FakeAgent()
    loop = LearnerLoop(agent, "127.0.0.1", 0)
    assert loop._refresh_offline_data() is None


def test_refresh_offline_data_hook_is_called_each_training_iteration():
    calls = []

    class _CountingLearnerLoop(LearnerLoop):
        def _refresh_offline_data(self) -> None:
            calls.append(1)

    agent = _FakeAgent(learning_starts=1, utd=1.0)
    loop = _CountingLearnerLoop(agent, "127.0.0.1", 0, train_freq=1, publish_freq=1, idle_poll_interval=0.02)
    loop._on_transition(_transition())

    thread = threading.Thread(target=loop.run, daemon=True)
    thread.start()
    try:
        deadline = time.time() + 5.0
        while not agent.train_calls and time.time() < deadline:
            time.sleep(0.02)
        assert agent.train_calls, "learner never trained"
        assert len(calls) >= 1
        assert len(calls) == len(agent.train_calls)
    finally:
        loop.stop()
        thread.join(timeout=5.0)


def test_run_publishes_params_reachable_by_a_real_actor_client():
    agent = _FakeAgent(learning_starts=1, utd=1.0)
    loop = LearnerLoop(agent, "127.0.0.1", 0, train_freq=1, publish_freq=1, idle_poll_interval=0.02)
    loop._on_transition(_transition())

    thread = threading.Thread(target=loop.run, daemon=True)
    thread.start()
    try:
        deadline = time.time() + 5.0
        while not agent.train_calls and time.time() < deadline:
            time.sleep(0.02)
        assert agent.train_calls, "learner never trained"

        host, port = loop._server.server_address
        client = ActorSyncClient(f"http://{host}:{port}", poll_interval=10.0)
        client._poll_once()
        params = client.latest_policy_params()
        assert params is not None
        assert params["version"].item() >= 1
    finally:
        loop.stop()
        thread.join(timeout=5.0)


def test_train_step_saves_periodic_checkpoint_keyed_by_global_update():
    agent = _FakeAgent(checkpoint_dir="/tmp/ckpt", checkpoint_freq=2, save_replay_buffer=True)
    loop = LearnerLoop(agent, "127.0.0.1", 0, train_freq=2, publish_freq=1)

    loop._train_step()  # gradient_steps = 2 -> crosses checkpoint_freq=2
    assert agent.save_calls == [("/tmp/ckpt/checkpoint_2.pt", True)]

    loop._train_step()  # global_update=4 -> crosses next multiple
    assert agent.save_calls == [
        ("/tmp/ckpt/checkpoint_2.pt", True),
        ("/tmp/ckpt/checkpoint_4.pt", True),
    ]


def test_train_step_does_not_save_when_checkpointing_disabled():
    agent = _FakeAgent(checkpoint_dir=None, checkpoint_freq=2)
    loop = LearnerLoop(agent, "127.0.0.1", 0, train_freq=2, publish_freq=1)
    loop._train_step()
    assert agent.save_calls == []

    agent = _FakeAgent(checkpoint_dir="/tmp/ckpt", checkpoint_freq=0)
    loop = LearnerLoop(agent, "127.0.0.1", 0, train_freq=2, publish_freq=1)
    loop._train_step()
    assert agent.save_calls == []


def test_run_saves_final_checkpoint_on_stop_when_enabled():
    agent = _FakeAgent(
        learning_starts=1,
        utd=1.0,
        checkpoint_dir="/tmp/ckpt",
        checkpoint_freq=0,
        save_final_checkpoint=True,
    )
    loop = LearnerLoop(agent, "127.0.0.1", 0, train_freq=1, publish_freq=1, idle_poll_interval=0.02)
    loop._on_transition(_transition())

    thread = threading.Thread(target=loop.run, daemon=True)
    thread.start()
    try:
        deadline = time.time() + 5.0
        while not agent.train_calls and time.time() < deadline:
            time.sleep(0.02)
        assert agent.train_calls, "learner never trained"
    finally:
        loop.stop()
        thread.join(timeout=5.0)

    assert ("/tmp/ckpt/final.pt", False) in agent.save_calls


def test_run_does_not_save_final_checkpoint_when_disabled():
    agent = _FakeAgent(
        learning_starts=1,
        utd=1.0,
        checkpoint_dir="/tmp/ckpt",
        checkpoint_freq=0,
        save_final_checkpoint=False,
    )
    loop = LearnerLoop(agent, "127.0.0.1", 0, train_freq=1, publish_freq=1, idle_poll_interval=0.02)
    loop._on_transition(_transition())

    thread = threading.Thread(target=loop.run, daemon=True)
    thread.start()
    try:
        deadline = time.time() + 5.0
        while not agent.train_calls and time.time() < deadline:
            time.sleep(0.02)
        assert agent.train_calls, "learner never trained"
    finally:
        loop.stop()
        thread.join(timeout=5.0)

    assert agent.save_calls == []


# -- metrics -----------------------------------------------------------------------------


class _RecordingLogger:
    def __init__(self):
        self.scalars: list[tuple[str, float, int]] = []

    def add_scalar(self, tag, value, step):
        self.scalars.append((tag, value, step))

    def tags(self) -> set[str]:
        return {tag for tag, _, _ in self.scalars}

    def value(self, tag: str) -> tuple[float, int]:
        """The latest ``(value, step)`` logged under ``tag``."""
        matches = [(value, step) for name, value, step in self.scalars if name == tag]
        assert matches, tag
        return matches[-1]


class _LoggingFakeAgent(_FakeAgent):
    """``_FakeAgent`` plus the logging surface of ``BaseAlgorithm``: ``logger``,
    ``log_freq`` and ``_log_update_metrics`` (recorded)."""

    def __init__(self, log_freq: int = 3, logger=None, **kwargs):
        super().__init__(**kwargs)
        self.log_freq = log_freq
        self.logger = logger
        self.compute_info_calls: list[bool] = []
        self.update_metric_calls: list[tuple[dict, int]] = []

    def train(self, gradient_steps: int, compute_info: bool = False):
        super().train(gradient_steps)
        self.compute_info_calls.append(compute_info)
        return {"critic_loss": 0.5} if compute_info else {}

    def _log_update_metrics(self, metrics, step):
        self.update_metric_calls.append((metrics, step))


def _receive_and_train(loop: LearnerLoop, transitions: int, iterations_per_transition: int = 2):
    for _ in range(transitions):
        loop._on_transition(_transition())
        for _ in range(iterations_per_transition):
            loop._train_iteration(loop.received_transitions)


def test_update_metrics_are_logged_once_per_log_freq_boundary_of_received_transitions():
    logger = _RecordingLogger()
    agent = _LoggingFakeAgent(log_freq=3, logger=logger)
    loop = LearnerLoop(agent, "127.0.0.1", 0)

    _receive_and_train(loop, 7)

    assert agent.update_metric_calls == [({"critic_loss": 0.5}, 3), ({"critic_loss": 0.5}, 6)]
    assert agent.compute_info_calls.count(True) == 2
    assert len(agent.compute_info_calls) == 14
    assert logger.value("learner/received") == (6.0, 6)
    assert logger.value("learner/buffer_size") == (6.0, 6)
    update, _ = logger.value("learner/update")
    assert update == 11.0  # two iterations per transition, the 11th trained at step 6
    assert {tag for tag in logger.tags() if tag.startswith("learner/")} == {
        "learner/received",
        "learner/update",
        "learner/buffer_size",
        "learner/train_time_s",
    }
    assert [step for tag, _, step in logger.scalars if tag == "learner/train_time_s"] == [3, 6]


def test_no_metrics_without_a_logger_and_train_never_computes_info():
    agent = _LoggingFakeAgent(log_freq=1, logger=None)
    loop = LearnerLoop(agent, "127.0.0.1", 0)

    _receive_and_train(loop, 4)

    assert agent.update_metric_calls == []
    assert agent.compute_info_calls == [False] * 8


def test_log_freq_zero_disables_update_metrics():
    agent = _LoggingFakeAgent(log_freq=0, logger=_RecordingLogger())
    loop = LearnerLoop(agent, "127.0.0.1", 0)

    _receive_and_train(loop, 4)

    assert agent.update_metric_calls == []
    assert agent.logger.scalars == []


def test_log_freq_falls_back_to_the_default_for_an_agent_without_one():
    assert LearnerLoop(_FakeAgent(), "127.0.0.1", 0).log_freq == 1_000
    assert LearnerLoop(_LoggingFakeAgent(log_freq=7), "127.0.0.1", 0).log_freq == 7


def test_actor_stats_are_logged_under_actor_with_the_actor_id_as_a_sub_prefix():
    logger = _RecordingLogger()
    loop = LearnerLoop(_LoggingFakeAgent(logger=logger), "127.0.0.1", 0)
    loop._server._record_stats({"return": 2.5, "success": 1.0, "note": "text"})
    loop._server._record_stats({"return": 1.0, "actor_id": "robot_a"})

    loop._log_actor_stats(42)

    assert sorted(logger.scalars) == [
        ("actor/return", 2.5, 42),
        ("actor/robot_a/return", 1.0, 42),
        ("actor/success", 1.0, 42),
    ]
    assert loop._server.drain_stats() == []


def test_actor_stats_stay_queued_without_a_logger():
    loop = LearnerLoop(_LoggingFakeAgent(logger=None), "127.0.0.1", 0)
    loop._server._record_stats({"return": 2.5})

    loop._log_actor_stats(1)

    assert loop._server.drain_stats() == [{"return": 2.5}]


def test_run_logs_actor_stats_while_waiting_for_learning_starts():
    logger = _RecordingLogger()
    agent = _LoggingFakeAgent(logger=logger, learning_starts=100)
    loop = LearnerLoop(agent, "127.0.0.1", 0, idle_poll_interval=0.02)
    loop._on_transition(_transition())
    loop._server._record_stats({"return": 3.0})

    thread = threading.Thread(target=loop.run, daemon=True)
    thread.start()
    try:
        deadline = time.time() + 5.0
        while "actor/return" not in logger.tags() and time.time() < deadline:
            time.sleep(0.02)
        assert logger.value("actor/return") == (3.0, 1)
        assert agent.train_calls == []
    finally:
        loop.stop()
        thread.join(timeout=5.0)
