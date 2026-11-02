"""Generic InterventionWrapper over ScriptedTeleopSource: action replacement, info keys, lifecycle."""
from __future__ import annotations

import numpy as np
import pytest
import torch
from _teleop_fakes import ContractEnv, FakeEnv, TwoArmContractEnv

from rl_garden_real_world.envs.wrappers.intervention import (
    InterventionWrapper,
    TeleopInterventionWrapper,
)
from rl_garden_real_world.teleop import TeleopReading
from rl_garden_real_world.teleop.fake import ScriptedTeleopSource


def _reading(intervened: bool, value: float = 5.0, episode_end: bool = False, events=None) -> TeleopReading:
    return TeleopReading(
        action=np.full(7, value, dtype=np.float32),
        intervened=intervened,
        episode_end=episode_end,
        events={} if events is None else events,
    )


def _wrap(readings, env=None) -> tuple[InterventionWrapper, ScriptedTeleopSource, FakeEnv]:
    env = FakeEnv() if env is None else env
    source = ScriptedTeleopSource(readings)
    return InterventionWrapper(env, source), source, env


def test_passthrough_keeps_the_policy_action_and_reports_not_intervened():
    wrapped, _, env = _wrap([_reading(False)])
    policy_action = torch.full((1, 7), 1.0)

    _, _, _, truncated, info = wrapped.step(policy_action)

    assert env.step_actions[0] is policy_action
    assert "intervene_action" not in info and "teleop_events" not in info
    assert info["policy_action"] is policy_action
    assert info["intervened"] is False and info["intervention_onset"] is False
    assert info["human_episode_end"] is False and bool(truncated) is False


def test_intervention_replaces_the_action_and_keeps_the_policy_action():
    wrapped, _, env = _wrap([_reading(True, value=0.5)])
    policy_action = torch.full((1, 7), 1.0)

    _, _, _, _, info = wrapped.step(policy_action)

    torch.testing.assert_close(env.step_actions[0], torch.full((1, 7), 0.5))
    torch.testing.assert_close(info["intervene_action"], torch.full((1, 7), 0.5))
    torch.testing.assert_close(info["policy_action"], torch.full((1, 7), 1.0))
    assert info["policy_action"] is policy_action
    assert info["intervened"] is True and info["intervention_onset"] is True
    assert env.step_actions[0].dtype == torch.float32 and env.step_actions[0].shape == (1, 7)


def test_intervention_matches_the_policy_action_device_and_always_sets_policy_action():
    wrapped, _, env = _wrap([_reading(False), _reading(True)])
    for _ in range(2):
        proposed = torch.zeros(1, 7)
        _, _, _, _, info = wrapped.step(proposed)
        assert info["policy_action"] is proposed
        assert env.step_actions[-1].device == proposed.device


def test_intervention_onset_marks_only_rising_edges():
    flags = [False, True, True, False, True]
    wrapped, _, _ = _wrap([_reading(flag) for flag in flags])

    infos = [wrapped.step(torch.zeros(1, 7))[4] for _ in flags]

    assert [info["intervened"] for info in infos] == flags
    assert [info["intervention_onset"] for info in infos] == [False, True, False, False, True]


def test_onset_restarts_after_reset():
    wrapped, _, _ = _wrap([_reading(True), _reading(True), _reading(True)])
    assert wrapped.step(torch.zeros(1, 7))[4]["intervention_onset"] is True
    assert wrapped.step(torch.zeros(1, 7))[4]["intervention_onset"] is False
    wrapped.reset()
    assert wrapped.step(torch.zeros(1, 7))[4]["intervention_onset"] is True


def test_teleop_events_are_forwarded_only_when_some_fired():
    wrapped, _, _ = _wrap([_reading(True, events={"success": True}), _reading(True)])
    assert wrapped.step(torch.zeros(1, 7))[4]["teleop_events"] == {"success": True}
    assert "teleop_events" not in wrapped.step(torch.zeros(1, 7))[4]


def test_episode_end_truncates_with_or_without_intervention():
    for intervened in (False, True):
        wrapped, _, _ = _wrap([_reading(intervened, episode_end=True), _reading(intervened)])
        _, _, terminated, truncated, info = wrapped.step(torch.zeros(1, 7))
        assert info["human_episode_end"] is True
        assert truncated.dtype == torch.bool and bool(truncated) and not bool(terminated)
        assert bool(wrapped.step(torch.zeros(1, 7))[3]) is False


def test_last_sent_gripper_action_tracks_the_executed_action_and_resets_to_open():
    wrapped, _, _ = _wrap([_reading(False), _reading(True, value=-0.5)])
    assert wrapped._last_gripper_action == 1.0
    policy_action = torch.zeros(1, 7)
    policy_action[0, -1] = -3.0  # clipped by the env to -1
    wrapped.step(policy_action)
    assert wrapped._last_gripper_action == -1.0
    wrapped.step(policy_action)
    assert wrapped._last_gripper_action == -0.5
    wrapped.reset()
    assert wrapped._last_gripper_action == 1.0


def test_source_is_started_once_and_exposed_as_source_and_teleop():
    wrapped, source, _ = _wrap([_reading(False)])
    assert source.start_calls == 1
    assert wrapped.source is source and wrapped.teleop is source


def test_a_source_that_does_not_fit_the_env_is_rejected_before_it_is_started():
    source = ScriptedTeleopSource([_reading(False)], action_mode="joint_delta")
    with pytest.raises(ValueError, match="action_mode"):
        InterventionWrapper(FakeEnv(), source)
    assert source.start_calls == 0

    with pytest.raises(ValueError, match="action_mode"):
        InterventionWrapper(ContractEnv("joint_delta", 6), ScriptedTeleopSource([_reading(False)]))


def test_reset_resets_the_env_before_the_source():
    order = []
    env = FakeEnv(call_order=order)
    source = ScriptedTeleopSource([_reading(False)])
    source.reset = lambda: order.append("source.reset")
    InterventionWrapper(env, source).reset()
    assert order == ["env.reset", "source.reset"]


def test_close_closes_the_source_then_the_env():
    order = []
    env = FakeEnv(call_order=order)
    source = ScriptedTeleopSource([_reading(False)])
    source.close = lambda: order.append("source.close")
    InterventionWrapper(env, source).close()
    assert order == ["source.close", "env.close"]


def test_env_is_closed_even_if_the_source_fails_to_close():
    env = FakeEnv()
    source = ScriptedTeleopSource([_reading(False)])

    def _fail():
        raise RuntimeError("device gone")

    source.close = _fail
    with pytest.raises(RuntimeError, match="device gone"):
        InterventionWrapper(env, source).close()
    assert env.close_calls == 1


def test_attribute_access_is_forwarded_to_the_env():
    wrapped, _, _ = _wrap([_reading(False)])
    assert wrapped.num_envs == 1


# -- TeleopInterventionWrapper (device by name) -------------------------------------------


def test_device_fake_builds_an_idle_source_from_the_env_contract():
    wrapped = TeleopInterventionWrapper(ContractEnv("cartesian_delta", 6), device="fake")
    _, _, _, _, info = wrapped.step(torch.zeros(1, 7))
    assert info["intervened"] is False
    assert isinstance(wrapped.source, ScriptedTeleopSource) and wrapped.source.start_calls == 1


def test_teleop_argument_wins_over_device_and_scripted_readings_pass_through():
    source = ScriptedTeleopSource([_reading(True, value=0.25)])
    wrapped = TeleopInterventionWrapper(FakeEnv(), teleop=source, device="spacemouse")
    assert wrapped.source is source
    assert wrapped.step(torch.zeros(1, 7))[4]["intervened"] is True


def test_unknown_device_lists_the_registered_names():
    with pytest.raises(ValueError, match="pico"):
        TeleopInterventionWrapper(FakeEnv(), device="joystick")


# -- two arms: one gripper per arm, the last two action dims ------------------------------


class _TwoArmScriptedSource(ScriptedTeleopSource):
    num_arms = 2


def _two_arm_reading(intervened: bool, grippers=(1.0, 1.0)) -> TeleopReading:
    return TeleopReading(action=np.array([0.0] * 12 + list(grippers), dtype=np.float32), intervened=intervened)


def test_two_arm_wrapper_tracks_both_executed_grippers_and_resets_them_to_open():
    wrapped = InterventionWrapper(
        TwoArmContractEnv(), _TwoArmScriptedSource([_reading(False), _two_arm_reading(True, (-0.5, 0.25))])
    )
    torch.testing.assert_close(wrapped._last_gripper_action, torch.tensor([1.0, 1.0]))
    policy_action = torch.zeros(1, 14)
    policy_action[0, -2:] = torch.tensor([-3.0, 0.5])  # the env clips to [-1, 1]
    wrapped.step(policy_action)
    torch.testing.assert_close(wrapped._last_gripper_action, torch.tensor([-1.0, 0.5]))
    _, _, _, _, info = wrapped.step(policy_action)
    assert info["intervened"] is True and info["intervene_action"].shape == (1, 14)
    torch.testing.assert_close(wrapped._last_gripper_action, torch.tensor([-0.5, 0.25]))
    wrapped.reset()
    torch.testing.assert_close(wrapped._last_gripper_action, torch.tensor([1.0, 1.0]))


def test_single_arm_wrapper_keeps_a_one_entry_gripper_vector():
    wrapped, _, _ = _wrap([_reading(False)])
    assert wrapped._last_gripper_action.shape == (1,)


def test_a_single_arm_source_on_a_two_arm_env_is_rejected():
    source = ScriptedTeleopSource([_reading(False)])
    with pytest.raises(ValueError, match="drives 1 arm.*num_arms=2"):
        InterventionWrapper(TwoArmContractEnv(), source)
    assert source.start_calls == 0


def test_a_two_arm_source_of_the_wrong_width_is_rejected():
    source = _TwoArmScriptedSource([_reading(False)], arm_action_dim=7)
    with pytest.raises(ValueError, match="16-D actions"):
        InterventionWrapper(TwoArmContractEnv(), source)
