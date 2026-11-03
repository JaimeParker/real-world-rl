"""Real-world RL orchestration for rl-garden: actor/learner loops, real-robot env backends,
teleop and the real-world training entrypoints.

The top level exports the method-agnostic loop classes of :mod:`rl_garden_real_world.loops`;
per-method loops live in ``rl_garden_real_world.loops.{serl,hil_serl,residual,silri}``.
"""
from importlib.metadata import PackageNotFoundError, version

from rl_garden_real_world.loops import (
    ActorLoop,
    ActorSyncClient,
    FWBWActorLoop,
    LearnerLoop,
    LearnerSyncServer,
)

try:
    __version__ = version("rlgarden-real-world")
except PackageNotFoundError:
    __version__ = "0.0.0+unknown"

__all__ = [
    "ActorLoop",
    "FWBWActorLoop",
    "LearnerLoop",
    "ActorSyncClient",
    "LearnerSyncServer",
]
