"""SERL's ActorLoop/LearnerLoop/sync classes are currently empty subclasses of
the algorithm-agnostic base classes -- this only checks the inheritance
relationship and importability, since there's no behavior of their own to
test (base-class behavior is covered by test_real_world_actor_loop.py /
test_real_world_learner_loop.py / test_real_world_sync.py)."""
from __future__ import annotations

from real_world_rl import (
    ActorSyncClient,
    LearnerSyncServer,
)
from real_world_rl.loops.actor import ActorLoop
from real_world_rl.loops.learner import LearnerLoop
from real_world_rl.loops.serl import (
    SerlActorLoop,
    SerlActorSyncClient,
    SerlLearnerLoop,
    SerlLearnerSyncServer,
)


def test_serl_loop_and_sync_classes_subclass_the_algorithm_agnostic_bases():
    assert issubclass(SerlActorLoop, ActorLoop)
    assert issubclass(SerlLearnerLoop, LearnerLoop)
    assert issubclass(SerlActorSyncClient, ActorSyncClient)
    assert issubclass(SerlLearnerSyncServer, LearnerSyncServer)


def test_serl_sync_client_constructs_like_the_base():
    client = SerlActorSyncClient("http://127.0.0.1:6000")
    assert client._base_url == "http://127.0.0.1:6000"
