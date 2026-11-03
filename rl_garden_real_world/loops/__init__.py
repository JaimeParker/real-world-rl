"""Actor/learner/sync loops of real-robot training.

``actor``, ``learner`` and ``sync`` hold the method-agnostic base classes; ``serl``,
``hil_serl``, ``residual`` and ``silri`` hold each method's actor, learner and sync
subclasses. The base classes are imported eagerly; a per-method class is imported on
first attribute access, so importing this package does not import
``rl_garden_real_world.training`` or ``rl_garden_real_world.policies``.
"""
from __future__ import annotations

import importlib
from typing import Any

from rl_garden_real_world.loops.actor import ActorLoop, FWBWActorLoop
from rl_garden_real_world.loops.learner import LearnerLoop
from rl_garden_real_world.loops.sync import ActorSyncClient, LearnerSyncServer

_METHOD_CLASS_MODULES = {
    "SerlActorLoop": "serl",
    "SerlLearnerLoop": "serl",
    "SerlActorSyncClient": "serl",
    "SerlLearnerSyncServer": "serl",
    "HilSerlActorLoop": "hil_serl",
    "HilSerlLearnerLoop": "hil_serl",
    "HilSerlActorSyncClient": "hil_serl",
    "HilSerlLearnerSyncServer": "hil_serl",
    "ResidualActorLoop": "residual",
    "ResidualLearnerLoop": "residual",
    "SilriActorLoop": "silri",
    "SilriLearnerLoop": "silri",
}


def __getattr__(name: str) -> Any:
    module_name = _METHOD_CLASS_MODULES.get(name)
    if module_name is None:
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
    module = importlib.import_module(f"{__name__}.{module_name}")
    return getattr(module, name)


__all__ = [
    "ActorLoop",
    "FWBWActorLoop",
    "LearnerLoop",
    "ActorSyncClient",
    "LearnerSyncServer",
    *_METHOD_CLASS_MODULES,
]
