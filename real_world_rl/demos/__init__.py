"""Demo recording on the real robot (see ``docs/recording-demos.md``)."""
from real_world_rl.demos.record import (
    HoldActionFn,
    LeaderFollowerGapError,
    RecordDemosArgs,
    RecordedEpisode,
    check_leader_near_follower,
    hold_action_fn_for,
    parse_record_args,
    record_demos,
    run_recorder,
    save_demos,
)

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
