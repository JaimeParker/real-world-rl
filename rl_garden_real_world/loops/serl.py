"""SERL's actor, learner and sync classes.

SERL drives the base loops unchanged; each class is kept as its own subclass for
naming symmetry with ``rl_garden_real_world.loops.hil_serl``.
"""
from __future__ import annotations

from rl_garden_real_world.loops.actor import ActorLoop
from rl_garden_real_world.loops.learner import LearnerLoop
from rl_garden_real_world.loops.sync import ActorSyncClient, LearnerSyncServer

__all__ = [
    "SerlActorLoop",
    "SerlLearnerLoop",
    "SerlActorSyncClient",
    "SerlLearnerSyncServer",
]


class SerlActorLoop(ActorLoop):
    """SERL's ActorLoop -- currently identical to the base; kept as its own
    subclass for naming symmetry with HIL-SERL's own (future) override.
    """


class SerlLearnerLoop(LearnerLoop):
    """SERL's LearnerLoop -- no offline-data refresh needed (SERL doesn't use a
    growing on-disk demo dataset), so this leaves ``_refresh_offline_data()`` as
    the base class's no-op default.
    """


class SerlActorSyncClient(ActorSyncClient):
    """SERL's actor sync client -- currently identical to the base; kept as its own
    subclass for naming symmetry with HIL-SERL's own (future) override.
    """


class SerlLearnerSyncServer(LearnerSyncServer):
    """SERL's learner sync server -- currently identical to the base; kept as its own
    subclass for naming symmetry with HIL-SERL's own (future) override.
    """
