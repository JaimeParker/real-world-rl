"""Demo recording on the real robot (see ``docs/recording-demos.md``).

The recorder exports load on first access, so torch-free submodules (such as the archive
inspector's arguments) import without the recorder's dependencies.
"""
from importlib import import_module

__all__ = [
    "HoldActionFn",
    "LeaderFollowerGapError",
    "RecordDemosArgs",
    "RecordedEpisode",
    "check_leader_near_follower",
    "hold_action_fn_for",
    "parse_record_args",
    "record_demos",
    "run_recorder",
    "save_demos",
]


def __getattr__(name: str):
    if name not in __all__:
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
    value = getattr(import_module("real_world_rl.demos.record"), name)
    globals()[name] = value
    return value


def __dir__() -> list[str]:
    return sorted(set(globals()) | set(__all__))
