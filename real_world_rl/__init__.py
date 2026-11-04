"""Real-world RL orchestration for rl-garden: actor/learner loops, real-robot env backends,
teleop and the real-world training entrypoints.

The top level exports the method-agnostic loop classes of :mod:`real_world_rl.loops`;
per-method loops live in ``real_world_rl.loops.{serl,hil_serl,residual,silri}``.
"""
from importlib.metadata import PackageNotFoundError, version
from importlib import import_module

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


def __getattr__(name: str):
    if name == "loops":
        value = import_module("real_world_rl.loops")
    elif name in __all__:
        value = getattr(import_module("real_world_rl.loops"), name)
    else:
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
    globals()[name] = value
    return value


def __dir__() -> list[str]:
    return sorted(set(globals()) | set(__all__) | {"loops"})
