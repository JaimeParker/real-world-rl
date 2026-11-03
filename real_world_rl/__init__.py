"""Real-world RL orchestration for rl-garden: actor/learner loops, real-robot env backends,
teleop and the real-world training entrypoints.

The top level exports the method-agnostic loop classes of :mod:`real_world_rl.loops`;
per-method loops live in ``real_world_rl.loops.{serl,hil_serl,residual,silri}``.
"""
from importlib.metadata import PackageNotFoundError, version

from real_world_rl.loops import (
    ActorLoop,
    ActorSyncClient,
    FWBWActorLoop,
    LearnerLoop,
    LearnerSyncServer,
)

try:
    __version__ = version("real-world-rl")
except PackageNotFoundError:
    __version__ = "0.0.0+unknown"

__all__ = [
    "ActorLoop",
    "FWBWActorLoop",
    "LearnerLoop",
    "ActorSyncClient",
    "LearnerSyncServer",
]
