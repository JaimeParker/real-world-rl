"""LeaderArmPartSource over a fake joint_abs leader part: env-mode mapping, the ramp and arm onset
policies, the external toggle, the loose joint-space declaration and the registry entry."""
from __future__ import annotations

import logging
import types

import numpy as np
import pytest
import torch
from _leader_teleop_fakes import (
    JOINT_SCALE_RAD,
    JointEnv,
    connected_joint_part,
    executed_joints,
    normalized_joints,
)
from _teleop_fakes import ContractEnv
from real_world_rl.devices import DeviceFault

from real_world_rl.envs.wrappers.intervention import (
    InterventionWrapper,
    TeleopInterventionWrapper,
)
from real_world_rl.teleop import TeleopReading, make_teleop_source
from real_world_rl.teleop.leader import joint as leader_arm
from real_world_rl.teleop.leader.joint import (
    TELEOP_SYNCING_EVENT,
    LeaderArmPartSource,
)

FAR = [0.3, -0.02, 0.0, 0.0, 0.0, 0.0, 0.12]
NEAR = [0.05, -0.02, 0.0, 0.0, 0.0, 0.0, 0.0]
STEP = 0.05


@pytest.fixture
def make_part():
    parts = []

    def make(script, **options):
        part = connected_joint_part(script, **options)
        parts.append(part)
        return part

    yield make
    for part in parts:
        part.disconnect()


def _source(part, env, **kwargs) -> LeaderArmPartSource:
    contract = env.action_contract
    kwargs.setdefault("joint_limits", contract["joint_limits"])
    kwargs.setdefault("joint_scale_rad", contract["joint_scale_rad"])
    kwargs.setdefault("last_observation_getter", lambda: env.last_observation)
    return LeaderArmPartSource(part, env_contract=contract, **kwargs)


# -- ramp ---------------------------------------------------------------------------------


def test_ramp_rate_limits_joint_abs_targets_and_flags_syncing_until_caught_up(make_part):
    env = JointEnv("joint_abs")
    part = make_part([{"action": FAR, "gripper": 1.0, "active": True}])
    source = _source(part, env, ramp_step_rad=STEP)
    assert (source.action_mode, source.arm_action_dim, source.has_gripper) == ("joint_abs", 7, True)

    reading = source.poll()
    first_target = np.array([0.05, -0.02, 0.0, 0.0, 0.0, 0.0, 0.05])
    assert reading.intervened
    # The recorded action is the command the env executes: the rate-limited target.
    np.testing.assert_allclose(executed_joints(reading.action[:7]), first_target, atol=1e-6)
    assert reading.action[7] == pytest.approx(1.0)
    assert dict(reading.events) == {TELEOP_SYNCING_EVENT: True}

    env.follower = first_target
    reading = source.poll()
    np.testing.assert_allclose(
        executed_joints(reading.action[:7]), [0.1, -0.02, 0, 0, 0, 0, 0.1], atol=1e-6
    )
    assert TELEOP_SYNCING_EVENT in reading.events

    env.follower = np.array(FAR) - 0.01
    reading = source.poll()
    np.testing.assert_allclose(reading.action[:7], normalized_joints(FAR), atol=1e-6)
    assert dict(reading.events) == {}


def test_ramp_is_inactive_while_not_intervening(make_part):
    env = JointEnv("joint_abs")
    source = _source(make_part([{"action": FAR, "active": False}]), env)
    reading = source.poll()
    assert not reading.intervened and dict(reading.events) == {}
    np.testing.assert_allclose(reading.action[:7], normalized_joints(FAR), atol=1e-6)


def test_joint_delta_ramp_is_the_parents_saturation(make_part):
    env = JointEnv("joint_delta")
    env.follower = np.full(7, 0.1)
    source = _source(make_part([{"action": FAR, "active": True}]), env, ramp_step_rad=STEP)
    assert source.action_mode == "joint_delta"
    reading = source.poll()
    expected = np.clip((np.array(FAR) - env.follower) / JOINT_SCALE_RAD, -1.0, 1.0)
    np.testing.assert_allclose(reading.action[:7], expected, atol=1e-6)
    # The takeover's first reading, still far from the follower: the catch-up.
    assert reading.intervened and dict(reading.events) == {TELEOP_SYNCING_EVENT: True}


def test_joint_delta_flags_syncing_only_until_the_takeover_catches_up(make_part):
    env = JointEnv("joint_delta")
    script = [{"action": FAR, "active": True}] * 4 + [
        {"action": FAR, "active": False},
        {"action": FAR, "active": True},
    ]
    source = _source(make_part(script), env)
    far = np.array(FAR)
    # (follower before the read, expected syncing): catch-up, caught up, the operator outpacing
    # the follower, toggle released, a new takeover catching up again.
    steps = [(far - 0.3, True), (far - 0.1, True), (far - 0.03, False), (far - 0.2, False), (far - 0.2, False),
             (far - 0.2, True)]
    readings = []
    for follower, syncing in steps:
        env.follower = follower
        reading = source.poll()
        readings.append(reading)
        assert (TELEOP_SYNCING_EVENT in reading.events) == syncing
    assert [r.intervened for r in readings] == [True, True, True, True, False, True]
    # The outpaced step is ordinary teaching: the saturated delta, every dim supervised.
    np.testing.assert_allclose(readings[3].action[:7], np.ones(7), atol=1e-6)
    assert readings[3].human_action_mask.all()


# -- arm ----------------------------------------------------------------------------------


def test_arm_waits_for_the_leader_then_stays_armed_until_the_toggle_releases(make_part):
    env = JointEnv("joint_abs")
    script = [
        {"action": FAR, "active": True},
        {"action": NEAR, "active": True},
        {"action": FAR, "active": True},
        {"action": FAR, "active": False},
        {"action": FAR, "active": True},
    ]
    source = _source(make_part(script), env, onset="arm", arm_tolerance_rad=0.1)
    readings = [source.poll() for _ in script]
    assert [r.intervened for r in readings] == [False, True, True, False, False]
    # Armed, the leader is followed directly (no ramp in arm mode).
    np.testing.assert_allclose(readings[2].action[:7], normalized_joints(FAR), atol=1e-6)
    assert all(TELEOP_SYNCING_EVENT not in r.events for r in readings)


def test_arm_logs_the_joint_error_at_most_once_per_second(make_part, monkeypatch, caplog):
    clock = types.SimpleNamespace(now=0.0)
    monkeypatch.setattr(leader_arm, "_now", lambda: clock.now)
    env = JointEnv("joint_abs")
    source = _source(make_part([{"action": FAR, "active": True}]), env, onset="arm")
    caplog.set_level(logging.WARNING, logger=leader_arm.__name__)
    for now in (0.0, 0.5, 0.9, 1.2):
        clock.now = now
        assert not source.poll().intervened
    records = [r for r in caplog.records if r.name == leader_arm.__name__]
    assert len(records) == 2
    assert "0.3" in records[0].getMessage()


def test_reset_disarms(make_part):
    env = JointEnv("joint_abs")
    script = [{"action": NEAR, "active": True}, {"action": FAR, "active": True}]
    source = _source(make_part(script), env, onset="arm")
    assert source.poll().intervened
    assert source.poll().intervened  # armed: stays on while the leader moves away
    source.reset()
    assert not source.poll().intervened


def test_poll_with_toggle_replaces_the_parts_intervening(make_part):
    env = JointEnv("joint_abs")
    source = _source(make_part([{"action": FAR, "active": False}]), env)
    reading = source.poll_with_toggle(True)
    assert reading.intervened and TELEOP_SYNCING_EVENT in reading.events
    assert not source.poll_with_toggle(False).intervened


def test_arm_counts_the_external_toggle(make_part):
    env = JointEnv("joint_abs")
    script = [{"action": NEAR, "active": True}, {"action": NEAR, "active": True}]
    source = _source(make_part(script), env, onset="arm")
    assert not source.poll_with_toggle(False).intervened  # the part's own toggle is ignored
    assert source.poll_with_toggle(True).intervened


# -- declaration and validation -----------------------------------------------------------


def test_declared_mode_only_has_to_be_a_joint_space(make_part):
    env = JointEnv("joint_delta")
    part = make_part([{"action": NEAR, "active": True}])
    source = _source(part, env, teleop_contract=("joint_delta", 7, True))
    assert source.action_mode == "joint_delta" and source.poll().intervened
    with pytest.raises(ValueError, match="does not match"):
        _source(part, env, teleop_contract=("joint_delta", 6, True))


def test_resolved_part_is_checked_against_the_declaration(make_part):
    env = JointEnv("joint_delta")
    holder = {"part": None}
    source = _source(lambda: holder["part"], env, teleop_contract=("joint_delta", 7, True))
    holder["part"] = make_part([{"action": NEAR, "active": True}])  # a joint_abs leader
    assert source.poll().intervened
    holder["part"] = make_part([{"action": [0.0] * 6, "active": True}], num_joints=6)
    with pytest.raises(DeviceFault, match="declares"):
        source.poll()


def test_cartesian_part_is_rejected(make_part):
    part = make_part([], action_space="cartesian_delta", num_joints=None)
    with pytest.raises(ValueError, match="joint positions"):
        _source(part, JointEnv("joint_abs"))


def test_cartesian_env_is_rejected_by_the_wrapper_naming_both(make_part):
    env = ContractEnv()
    source = LeaderArmPartSource(make_part([]), env_contract=env.action_contract)
    with pytest.raises(ValueError, match="joint_abs.*cartesian_delta"):
        InterventionWrapper(env, source)


def test_joint_abs_onset_needs_the_follower(make_part):
    env = JointEnv("joint_abs")
    part = make_part([])
    with pytest.raises(ValueError, match="last_observation_getter"):
        _source(part, env, last_observation_getter=None)
    no_joints = JointEnv("joint_abs", state_fields=("gripper_pos",))
    with pytest.raises(ValueError, match="joint_pos"):
        _source(part, no_joints)


@pytest.mark.parametrize(
    "kwargs, match",
    [({"onset": "snap"}, "onset"), ({"arm_tolerance_rad": 0.0}, "> 0"), ({"ramp_step_rad": -1.0}, "> 0")],
)
def test_invalid_onset_arguments_are_rejected(make_part, kwargs, match):
    with pytest.raises(ValueError, match=match):
        _source(make_part([]), JointEnv("joint_abs"), **kwargs)


# -- registry and wrapper -----------------------------------------------------------------


def test_registered_as_leader_arm_over_the_envs_robot_teleop(make_part):
    part = make_part([{"action": NEAR, "active": True}])
    env = JointEnv("joint_delta", robot=types.SimpleNamespace(teleop=part))
    source = make_teleop_source("leader_arm", env, joint_scale_rad=JOINT_SCALE_RAD, onset="arm")
    assert isinstance(source, LeaderArmPartSource)
    reading = source.poll()  # the follower comes from the env's last observation
    assert reading.intervened
    np.testing.assert_allclose(reading.action[:7], np.clip(np.array(NEAR) / JOINT_SCALE_RAD, -1, 1), atol=1e-6)


def test_wrapper_records_the_ramped_action_and_the_syncing_event(make_part):
    env = JointEnv("joint_abs")
    source = _source(make_part([{"action": FAR, "gripper": 0.0, "active": True}]), env)
    wrapper = InterventionWrapper(env, source)
    wrapper.reset()
    _, _, _, _, info = wrapper.step(torch.zeros(1, 8))
    executed = info["intervene_action"].numpy()[0]
    np.testing.assert_allclose(
        executed_joints(executed[:7]), [0.05, -0.02, 0, 0, 0, 0, 0.05], atol=1e-6
    )
    assert executed[7] == pytest.approx(-1.0)
    assert info["teleop_events"] == {TELEOP_SYNCING_EVENT: True}


@pytest.mark.parametrize("device", ["leader_arm", "composite"])
def test_gripperless_leader_holds_the_gripper_action_last_sent_through_the_wrapper(make_part, device):
    script = [{"action": NEAR, "active": False}, {"action": NEAR, "active": True}]
    part = make_part(script, has_gripper=False)
    env = JointEnv("joint_abs", robot=types.SimpleNamespace(teleop=part))
    leader_kwargs = {"joint_limits": env.action_contract["joint_limits"]}
    if device == "leader_arm":
        wrapper = TeleopInterventionWrapper(env, device="leader_arm", **leader_kwargs)
    else:
        toggles = [TeleopReading.idle(8), TeleopReading(action=np.zeros(8), intervened=True)]
        wrapper = TeleopInterventionWrapper(
            env,
            device="composite",
            motion="leader_arm",
            events="fake",
            motion_kwargs=leader_kwargs,
            events_kwargs={"readings": toggles},
        )
    wrapper.reset()
    close = torch.zeros(1, 8)
    close[0, -1] = -1.0
    _, _, _, _, info = wrapper.step(close)  # the policy closes the gripper
    assert not info["intervened"]
    _, _, _, _, info = wrapper.step(torch.zeros(1, 8))
    assert info["intervened"]
    assert info["intervene_action"][0, -1].item() == pytest.approx(-1.0)
