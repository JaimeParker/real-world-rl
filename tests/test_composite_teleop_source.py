"""CompositeTeleopSource: motion from one source, intervention and events from another."""
from __future__ import annotations

import numpy as np
import pytest
import torch
from _leader_teleop_fakes import JointEnv, connected_joint_part
from _teleop_fakes import ContractEnv

from real_world_rl.envs.wrappers.intervention import TeleopInterventionWrapper
from real_world_rl.teleop import TeleopReading, make_teleop_source
from real_world_rl.teleop.core.composite import (
    CompositeTeleopSource,
    ToggleableTeleopSource,
)
from real_world_rl.teleop.core.fake import ScriptedTeleopSource
from real_world_rl.teleop.leader.joint import LeaderArmPartSource


def _reading(value=0.0, intervened=False, episode_end=False, events=(), dim=8) -> TeleopReading:
    return TeleopReading(
        action=np.full(dim, value),
        intervened=intervened,
        episode_end=episode_end,
        events={key: True for key in events},
    )


def _motion(*readings) -> ScriptedTeleopSource:
    return ScriptedTeleopSource(list(readings), action_mode="joint_delta", arm_action_dim=7, has_gripper=True)


def _events(*readings) -> ScriptedTeleopSource:
    return ScriptedTeleopSource(list(readings))  # cartesian 6 + gripper: never validated


class _ToggleRecordingMotion(ToggleableTeleopSource):
    """Records the toggles it receives and only intervenes on every second one (a gate)."""

    action_mode, arm_action_dim, has_gripper = "joint_abs", 7, True

    def __init__(self) -> None:
        self.toggles = []
        self.polls = 0

    def poll(self) -> TeleopReading:
        self.polls += 1
        return _reading(0.5, intervened=True)

    def poll_with_toggle(self, toggle: bool) -> TeleopReading:
        self.toggles.append(toggle)
        return _reading(0.5, intervened=toggle and len(self.toggles) % 2 == 0)


def test_declares_the_motion_sources_action_space():
    source = CompositeTeleopSource(_motion(_reading()), _events(_reading(dim=7)))
    assert (source.action_mode, source.arm_action_dim, source.has_gripper) == ("joint_delta", 7, True)


def test_action_from_motion_intervention_from_events_and_ored_events():
    motion = _motion(_reading(0.5, intervened=False, events=("success",)))
    events = _events(_reading(0.0, intervened=True, episode_end=True, events=("rerecord",), dim=7))
    reading = CompositeTeleopSource(motion, events).poll()
    np.testing.assert_allclose(reading.action, np.full(8, 0.5))
    assert reading.intervened and reading.episode_end
    assert dict(reading.events) == {"success": True, "rerecord": True}


def test_intervention_from_motion():
    motion = _motion(_reading(0.5, intervened=True, episode_end=True))
    events = _events(_reading(intervened=False, dim=7))
    reading = CompositeTeleopSource(motion, events, intervention_from="motion").poll()
    assert reading.intervened and reading.episode_end


def test_unknown_intervention_from_is_rejected():
    with pytest.raises(ValueError, match="intervention_from"):
        CompositeTeleopSource(_motion(_reading()), _events(_reading()), intervention_from="both")


def test_toggleable_motion_receives_the_event_toggle_and_decides():
    motion = _ToggleRecordingMotion()
    events = _events(_reading(intervened=True, dim=7))
    source = CompositeTeleopSource(motion, events)
    assert [source.poll().intervened for _ in range(3)] == [False, True, False]
    assert motion.toggles == [True, True, True] and motion.polls == 0

    by_motion = CompositeTeleopSource(motion, events, intervention_from="motion")
    assert by_motion.poll().intervened and motion.polls == 1


def test_leader_arm_onset_applies_to_the_event_toggle():
    near, far = [0.05] + [0.0] * 6, [0.5] + [0.0] * 6
    # An "always" leader: the part reports intervening on every read.
    part = connected_joint_part([{"action": a, "active": True} for a in (near, far, near, far)])
    try:
        env = JointEnv("joint_abs")
        motion = LeaderArmPartSource(
            part,
            env_contract=env.action_contract,
            joint_limits=env.action_contract["joint_limits"],
            last_observation_getter=lambda: env.last_observation,
            onset="arm",
        )
        toggles = [False, True, True, True]
        events = _events(*[_reading(intervened=t, dim=8) for t in toggles])
        source = CompositeTeleopSource(motion, events)
        # toggle off; toggle on but the leader is far; near: armed; far: stays armed.
        assert [source.poll().intervened for _ in toggles] == [False, False, True, True]
    finally:
        part.disconnect()


def test_start_reset_close_reach_both_halves():
    motion, events = _motion(_reading()), _events(_reading())
    source = CompositeTeleopSource(motion, events)
    source.start()
    source.reset()
    source.close()
    for half in (motion, events):
        assert (half.start_calls, half.reset_calls, half.close_calls) == (1, 1, 1)


class _Failing(ScriptedTeleopSource):
    def __init__(self, failing: str) -> None:
        super().__init__([_reading()])
        self.failing = failing

    def start(self) -> None:
        super().start()
        if self.failing == "start":
            raise OSError("no device")

    def reset(self) -> None:
        super().reset()
        if self.failing == "reset":
            raise OSError("reset failed")

    def close(self) -> None:
        super().close()
        if self.failing == "close":
            raise OSError("close failed")


def test_close_and_reset_reach_the_events_half_even_if_motion_raises():
    motion, events = _Failing("close"), _events(_reading())
    with pytest.raises(OSError, match="close failed"):
        CompositeTeleopSource(motion, events).close()
    assert events.close_calls == 1

    motion, events = _Failing("reset"), _events(_reading())
    with pytest.raises(OSError, match="reset failed"):
        CompositeTeleopSource(motion, events).reset()
    assert events.reset_calls == 1


def test_events_half_failing_to_start_closes_the_motion_half():
    motion, events = _motion(_reading()), _Failing("start")
    with pytest.raises(OSError, match="no device"):
        CompositeTeleopSource(motion, events).start()
    assert motion.close_calls == 1


def test_from_env_builds_both_halves_through_the_registry():
    motion_reading = _reading(0.25, dim=8)
    source = make_teleop_source(
        "composite",
        ContractEnv("joint_delta", 7),
        motion="fake",
        events="fake",
        motion_kwargs={"readings": [motion_reading]},
        intervention_from="motion",
    )
    assert isinstance(source, CompositeTeleopSource) and source.intervention_from == "motion"
    assert isinstance(source.motion, ScriptedTeleopSource) and isinstance(source.events, ScriptedTeleopSource)
    assert (source.events.action_mode, source.events.arm_action_dim) == ("joint_delta", 7)
    assert source.poll().action[0] == pytest.approx(0.25)


def test_wrapper_builds_a_composite_by_name():
    env = ContractEnv()
    wrapped = TeleopInterventionWrapper(
        env,
        device="composite",
        motion="fake",
        events="fake",
        motion_kwargs={"readings": [_reading(0.5, dim=7)]},
        events_kwargs={"readings": [_reading(intervened=True, episode_end=True, dim=7)]},
    )
    _, _, _, truncated, info = wrapped.step(torch.zeros(1, 7))
    assert info["intervened"] and info["human_episode_end"] and bool(truncated)
    np.testing.assert_allclose(info["intervene_action"].numpy()[0], np.full(7, 0.5))
    wrapped.close()
    assert wrapped.source.motion.close_calls == 1 and wrapped.source.events.close_calls == 1
