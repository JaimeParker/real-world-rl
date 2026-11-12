"""LeaderArmCartesianSource: a joint_abs leader arm mapped through Panda FK onto a requested pose,
which a CartesianTargetTracker turns into the cartesian twist of franka_real / real_robot cartesian
envs. Covers both mappings, the take-over and hand-back, the target speed and lead cap, the
workspace hold, the gripper (and its take-over latch), the action-contract checks, the flags and
the experiment-contract block, and the leader served by a stub-robot daemon (RemoteRobot +
LocalClient)."""
from __future__ import annotations

import dataclasses
import logging
import math
from pathlib import Path

import numpy as np
import pytest
import torch
import yaml
from _teleop_fakes import ContractEnv
from scipy.spatial.transform import Rotation

import real_world_rl.devices.backends  # noqa: F401  (registers the fake backends)
import real_world_rl.devices.claims as claims
from real_world_rl.devices import DeviceFault
from real_world_rl.devices.robots.franka import FrankaConfig, FrankaRobot
from real_world_rl.devices.teleop import EVENT_KEYS, TeleopConfig, TeleopSample
from real_world_rl.server import RemoteRobot
from real_world_rl.server.service import RobotService
from real_world_rl.server.transport import LocalClient

from real_world_rl.action_contract import (
    describe_contract,
    validate_transitions_against_contract,
)
from real_world_rl.envs.franka_real.bridge_client import FrankaBridgeClient
from real_world_rl.envs.franka_real.config import FrankaRealEnvConfig
from real_world_rl.envs.franka_real.env import make_franka_real_env
from real_world_rl.envs.wrappers.intervention import (
    InterventionWrapper,
    TeleopInterventionWrapper,
)
from real_world_rl.kinematics import PandaKinematics
from real_world_rl.teleop.core.contract import TeleopReading, TeleopSource
from real_world_rl.teleop.core.targets.cartesian_tracker import (
    CartesianTrackerConfig,
    MissedControlDeadline,
)
from real_world_rl.teleop.core.composite import CompositeTeleopSource
from real_world_rl.teleop.leader.cartesian import LeaderArmCartesianSource
from real_world_rl.training import env_building
from real_world_rl.training.entrypoints.hil_serl import HilSerlArgs

READY_Q = [0.0, -math.pi / 4, 0.0, -3 * math.pi / 4, 0.0, math.pi / 2, math.pi / 4]
SMALL_DQ = np.array([0.006, -0.004, 0.006, 0.004, -0.006, 0.004, 0.006])
KINEMATICS = PandaKinematics()
SCALE_M, SCALE_RAD = 0.05, 0.3
# Seconds between two polls of the fake clock: the longest gap allowed, so the command moves at
# most 0.015 m / 0.09 rad per poll at the default target speed.
STEP_S = 0.3
PRESET = Path(__file__).resolve().parents[1] / "examples" / "presets" / "gello_cartesian.yaml"
GELLO_OPTION_KEYS = {
    "port",
    "baudrate",
    "joint_ids",
    "joint_offsets_rad",
    "joint_signs",
    "wrap_centers_rad",
    "gripper_id",
    "gripper_open_rad",
    "gripper_closed_rad",
    "gripper_binarize_threshold",
    "intervention",
    "event_device",
    "event_keys",
    "poll_interval_s",
}


@pytest.fixture(autouse=True)
def _claim_dir(tmp_path, monkeypatch):
    monkeypatch.setattr(claims, "_DEFAULT_LOCK_DIR", tmp_path / "claims")


class _Leader:
    """A joint_abs leader part: each ``read()`` returns the current ``q``/``gripper``/``intervening``
    (joint angles float32, as a device sample carries them)."""

    action_space = "joint_abs"
    action_dim = 7
    name = "leader"

    def __init__(self, q=READY_Q, *, gripper=1.0, intervening=True, has_gripper=True):
        self.q = np.asarray(q, dtype=np.float32)
        self.has_gripper = has_gripper
        self.gripper = gripper if has_gripper else None
        self.intervening = intervening
        self.events = {}
        self.reset_calls = 0
        self._seq = 0

    def read(self, timeout_s=None):
        self._seq += 1
        events = {key: bool(self.events.get(key, False)) for key in EVENT_KEYS}
        self.events = {}
        return TeleopSample(
            action=self.q.copy(),
            action_space=self.action_space,
            gripper=self.gripper,
            buttons={},
            events=events,
            intervening=self.intervening,
            timestamp=0.0,
            seq=self._seq,
        )

    def reset(self):
        self.reset_calls += 1


class _Clock:
    """A monotonic clock that advances ``step_s`` on every read. Differences of its readings carry
    float rounding (1.2 - 0.9 is 0.3000000000000002), as a real clock's do: at ``STEP_S`` they sit
    on the tracker's ``max_dt_s`` bound, which tolerates that rounding."""

    def __init__(self, step_s=STEP_S):
        self.now = 0.0
        self.step_s = step_s

    def __call__(self):
        self.now += self.step_s
        return self.now


def franka_real_update(pose, action, pos_scale=SCALE_M, rot_scale=SCALE_RAD):
    """The env's cartesian_delta target pose (without the safety box): base-frame translation and
    a left-multiplied rotation vector, quaternions xyzw."""
    action = np.asarray(action, dtype=np.float64)
    position = pose[:3] + action[:3] * pos_scale
    rotation = Rotation.from_rotvec(action[3:6] * rot_scale) * Rotation.from_quat(pose[3:7])
    return np.concatenate([position, rotation.as_quat()])


class _Follower:
    """Follower TCP pose, moved by ``franka_real_update``."""

    def __init__(self, pose):
        self.pose = np.asarray(pose, dtype=np.float64)

    def __call__(self):
        return self.pose.copy()

    def apply(self, action):
        self.pose = franka_real_update(self.pose, action)


def _pose(position, rotation: Rotation) -> np.ndarray:
    return np.concatenate([position, rotation.as_quat()])


def _ee_pose(q) -> np.ndarray:
    transform = KINEMATICS.ee_transform(q)
    return _pose(transform[:3, 3], Rotation.from_matrix(transform[:3, :3]))


def _offset_pose(q, offset_m, rotvec) -> np.ndarray:
    """The leader's FK pose of ``q`` moved by ``offset_m`` and rotated by ``rotvec`` (base frame)."""
    pose = _ee_pose(q)
    return _pose(pose[:3] + offset_m, Rotation.from_rotvec(rotvec) * Rotation.from_quat(pose[3:]))


def _source(leader, follower, **kwargs) -> LeaderArmCartesianSource:
    kwargs.setdefault("clock", _Clock())
    return LeaderArmCartesianSource(leader, follower_pose_getter=follower, **kwargs)


def _expected_twist(target: np.ndarray, follower_pose: np.ndarray, pos_scale=SCALE_M, rot_scale=SCALE_RAD):
    position = (target[:3, 3] - follower_pose[:3]) / pos_scale
    rotation = Rotation.from_matrix(target[:3, :3]) * Rotation.from_quat(follower_pose[3:]).inv()
    return np.concatenate([position, rotation.as_rotvec() / rot_scale])


def _rotation_error(pose, target_quat) -> float:
    return (Rotation.from_quat(pose[3:7]) * Rotation.from_quat(target_quat).inv()).magnitude()


def _transform(pose) -> np.ndarray:
    transform = np.eye(4)
    transform[:3, :3] = Rotation.from_quat(pose[3:]).as_matrix()
    transform[:3, 3] = pose[:3]
    return transform


def _anchored_target(q_onset, q_now, command_pose) -> np.ndarray:
    """The leader displacement since the take-over applied to ``command_pose`` (decoupled)."""
    leader_onset, leader_now = KINEMATICS.ee_transform(q_onset), KINEMATICS.ee_transform(q_now)
    target = np.eye(4)
    target[:3, 3] = command_pose[:3] + (leader_now[:3, 3] - leader_onset[:3, 3])
    target[:3, :3] = leader_now[:3, :3] @ leader_onset[:3, :3].T @ _transform(command_pose)[:3, :3]
    return target


def _within_one_step(target, pose) -> bool:
    """Whether ``target`` is close enough to ``pose`` for one advance at the default speed."""
    rotation = Rotation.from_matrix(target[:3, :3]) * Rotation.from_quat(pose[3:]).inv()
    return np.linalg.norm(target[:3, 3] - pose[:3]) < 0.05 * STEP_S and rotation.magnitude() < 0.3 * STEP_S


# -- anchored mapping -------------------------------------------------------------------


def test_anchored_take_over_emits_a_zero_twist_then_tracks_the_leader_displacement():
    leader = _Leader(intervening=False)
    follower_pose = _offset_pose(READY_Q, [0.05, 0.03, -0.04], [0.1, -0.05, 0.2])
    source = _source(leader, _Follower(follower_pose))

    reading = source.poll()
    assert reading.intervened is False
    np.testing.assert_array_equal(reading.action[:6], 0.0)

    leader.intervening = True
    reading = source.poll()  # take-over: the command is the hold pose, where the follower is
    assert reading.intervened is True
    np.testing.assert_allclose(reading.action[:6], 0.0, atol=1e-9)

    q_onset = leader.q.astype(np.float64)
    leader.q = (q_onset + SMALL_DQ).astype(np.float32)
    reading = source.poll()
    target = _anchored_target(q_onset, leader.q, follower_pose)
    assert _within_one_step(target, follower_pose)
    expected = _expected_twist(target, follower_pose)
    assert np.abs(expected).max() > 1e-3
    np.testing.assert_allclose(reading.action[:6], expected, atol=1e-6)


def test_take_over_starts_from_the_hold_pose_without_a_jump():
    """The follower drifted since the hold was sampled at reset: the take-over keeps sending the
    hold action (back onto the hold pose), it does not jump to the leader."""
    leader = _Leader(intervening=False)
    hold_pose = _offset_pose(READY_Q, [0.03, -0.02, 0.01], [0.1, 0.0, -0.1])
    follower = _Follower(hold_pose)
    source = _source(leader, follower)
    source.reset()
    follower.pose = _offset_pose(READY_Q, [0.034, -0.02, 0.01], [0.1, 0.0, -0.1])

    leader.intervening = True
    reading = source.poll()
    assert reading.intervened
    np.testing.assert_allclose(reading.action[:6], _expected_twist(_transform(hold_pose), follower.pose), atol=1e-6)
    command = source.cartesian_tracker.command
    np.testing.assert_allclose(command[:3], hold_pose[:3], atol=1e-12)
    assert _rotation_error(command, hold_pose[3:]) < 1e-9


def test_the_anchor_maps_the_leaders_take_over_pose_onto_the_command():
    leader = _Leader()
    hold_pose = _offset_pose(READY_Q, [0.05, 0.02, -0.03], [0.0, 0.2, 0.1])
    source = _source(leader, _Follower(hold_pose))
    source.poll()  # take-over
    q_onset = leader.q.astype(np.float64)
    leader.q = (q_onset + SMALL_DQ).astype(np.float32)
    source.poll()

    target = _anchored_target(q_onset, leader.q, hold_pose)
    assert _within_one_step(target, hold_pose)
    command = source.cartesian_tracker.command
    np.testing.assert_allclose(command[:3], target[:3, 3], atol=1e-9)
    assert _rotation_error(command, Rotation.from_matrix(target[:3, :3]).as_quat()) < 1e-9


def test_anchored_leader_rotation_turns_the_follower_in_place():
    """A pure leader rotation (joint 7 spins the end effector about its own axis) must not
    translate a follower whose pose differs from the leader's at the take-over."""
    leader = _Leader()
    follower = _Follower(_offset_pose(READY_Q, [0.1, 0.05, -0.05], [0.2, -0.1, 0.3]))
    source = _source(leader, follower)
    source.poll()  # take-over
    leader_onset = KINEMATICS.ee_transform(leader.q)
    leader.q = leader.q + np.array([0, 0, 0, 0, 0, 0, 0.05], dtype=np.float32)
    leader_now = KINEMATICS.ee_transform(leader.q)
    np.testing.assert_allclose(leader_now[:3, 3], leader_onset[:3, 3], atol=1e-12)

    action = source.poll().action
    np.testing.assert_allclose(action[:3], 0.0, atol=1e-6)
    leader_rotation = Rotation.from_matrix(leader_now[:3, :3] @ leader_onset[:3, :3].T).as_rotvec()
    assert np.linalg.norm(leader_rotation) == pytest.approx(0.05, abs=1e-6)
    np.testing.assert_allclose(action[3:6], leader_rotation / SCALE_RAD, atol=1e-6)


def test_anchored_equals_absolute_when_the_follower_starts_at_the_leader_pose():
    follower = _Follower(_ee_pose(np.asarray(READY_Q, dtype=np.float32)))
    anchored_leader, absolute_leader = _Leader(), _Leader()
    anchored = _source(anchored_leader, follower)
    absolute = _source(absolute_leader, follower, mapping="absolute")
    anchored.poll()  # take-overs
    absolute.poll()
    for leader in (anchored_leader, absolute_leader):
        leader.q = (np.asarray(READY_Q) + SMALL_DQ).astype(np.float32)
    np.testing.assert_allclose(anchored.poll().action, absolute.poll().action, atol=1e-6)


def test_hand_back_resamples_the_hold_and_the_next_take_over_re_anchors():
    leader = _Leader()
    follower = _Follower(_offset_pose(READY_Q, [0.02, 0.0, 0.0], [0.0, 0.0, 0.1]))
    source = _source(leader, follower)
    tracker = source.cartesian_tracker
    source.poll()  # take-over
    leader.q = leader.q + SMALL_DQ.astype(np.float32)
    action = source.poll().action
    assert np.abs(action[:6]).max() > 1e-3
    follower.apply(action)  # the follower lands on the command

    leader.intervening = False
    assert not source.poll().intervened  # hand-back: the hold is re-sampled where the follower is
    assert not tracker.following
    np.testing.assert_allclose(tracker.hold_pose, follower.pose, atol=1e-12)

    leader.intervening = True
    np.testing.assert_allclose(source.poll().action[:6], 0.0, atol=1e-9)  # re-anchored at the moved leader
    leader.q = leader.q + SMALL_DQ.astype(np.float32)
    assert np.abs(source.poll().action[:6]).max() > 1e-3

    source.reset()
    assert leader.reset_calls == 1 and not tracker.following
    np.testing.assert_allclose(tracker.hold_pose, follower.pose, atol=1e-12)
    np.testing.assert_allclose(source.poll().action[:6], 0.0, atol=1e-9)


# -- absolute mapping, target speed and lead cap -------------------------------------------


def test_absolute_mapping_converges_at_the_target_speed_through_the_env_update_math():
    q = np.asarray([0.3, -0.5, 0.2, -2.0, 0.1, 1.8, -0.4], dtype=np.float32)
    target = _ee_pose(q)
    follower = _Follower(_offset_pose(q, [0.1, -0.06, 0.08], [0.25, -0.15, 0.2]))
    source = _source(_Leader(q), follower, mapping="absolute")

    previous = follower.pose
    for _ in range(40):
        follower.apply(source.poll().action)
        assert np.linalg.norm(follower.pose[:3] - previous[:3]) <= 0.05 * STEP_S + 1e-9
        assert _rotation_error(follower.pose, previous[3:]) <= 0.3 * STEP_S + 1e-9
        previous = follower.pose
    np.testing.assert_allclose(follower.pose[:3], target[:3], atol=1e-8)
    assert _rotation_error(follower.pose, target[3:]) < 1e-7


def test_absolute_take_over_holds_then_a_step_within_the_speed_lands_on_the_target():
    q = np.asarray(READY_Q, dtype=np.float32)
    target = _ee_pose(q)
    follower = _Follower(_offset_pose(q, [0.008, -0.005, 0.01], [0.03, 0.05, -0.04]))
    source = _source(_Leader(q), follower, mapping="absolute")
    np.testing.assert_allclose(source.poll().action[:6], 0.0, atol=1e-9)  # no jump at the take-over
    action = source.poll().action
    assert np.abs(action[:6]).max() < 1.0
    follower.apply(action)
    np.testing.assert_allclose(follower.pose[:3], target[:3], atol=1e-8)
    assert _rotation_error(follower.pose, target[3:]) < 1e-6


def test_a_far_target_leads_by_at_most_the_lead_cap_and_never_saturates():
    q = np.asarray(READY_Q, dtype=np.float32)
    follower = _Follower(_offset_pose(q, [0.5, 0.0, 0.0], [0.0, 0.0, math.pi / 2]))  # stuck
    source = _source(_Leader(q), follower, mapping="absolute")
    for _ in range(10):
        action = source.poll().action
        assert np.all(np.abs(action[:6]) <= 1.0)
        command = source.cartesian_tracker.command
        assert np.linalg.norm(command[:3] - follower.pose[:3]) <= 0.05 + 1e-9
        assert _rotation_error(command, follower.pose[3:]) <= 0.3 + 1e-9
    np.testing.assert_allclose(action[:6], [-1.0, 0.0, 0.0, 0.0, 0.0, -1.0], atol=1e-6)


@pytest.mark.parametrize(
    "scales, match",
    [((0.02, 0.3), r"xyz_scale_m=0\.02, lead_cap_m=0\.05"), ((0.05, 0.1), r"rot_scale_rad=0\.1, lead_cap_rad=0\.3")],
)
def test_scales_below_the_lead_caps_are_rejected(scales, match):
    follower = _Follower(_ee_pose(READY_Q))
    with pytest.raises(ValueError, match=match):
        _source(_Leader(), follower, pos_scale_m=scales[0], rot_scale_rad=scales[1])
    _source(_Leader(), follower, pos_scale_m=0.02, rot_scale_rad=0.1, lead_cap_m=0.02, lead_cap_rad=0.1)


def test_a_request_outside_the_safety_box_holds_until_a_new_take_over(caplog):
    q = np.asarray(READY_Q, dtype=np.float32)
    follower_pose = _ee_pose(q)
    box = [*(follower_pose[:3] - 0.01), *(follower_pose[:3] + 0.01)]
    leader = _Leader(q)
    source = _source(leader, _Follower(follower_pose), safety_box=box)
    tracker = source.cartesian_tracker
    assert source.poll().intervened  # take-over

    leader.q = q + np.array([0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float32)  # several cm out
    with caplog.at_level(logging.WARNING, logger="real_world_rl.teleop.leader.cartesian"):
        reading = source.poll()
    assert not reading.intervened and "WORKSPACE HOLD" in caplog.text
    assert not tracker.following
    np.testing.assert_allclose(tracker.hold_pose, follower_pose, atol=1e-12)

    leader.q = q  # back inside: still held, the operator hands back and takes over again
    assert not source.poll().intervened
    leader.intervening = False
    source.poll()
    leader.intervening = True
    assert source.poll().intervened and tracker.following


def test_a_stalled_control_loop_holds_and_re_anchors_at_the_next_poll(caplog):
    clock = _Clock()
    q = np.asarray(READY_Q, dtype=np.float32)
    leader = _Leader(q)
    follower = _Follower(_offset_pose(q, [0.02, 0.0, 0.0], [0.0, 0.0, 0.1]))
    source = _source(leader, follower, clock=clock)
    tracker = source.cartesian_tracker
    source.poll()  # take-over
    leader.q = q + SMALL_DQ.astype(np.float32)
    follower.apply(source.poll().action)  # the follower lands on the command

    leader.q = q + 2 * SMALL_DQ.astype(np.float32)  # the leader moves while the loop stalls
    clock.step_s = 1.0
    with caplog.at_level(logging.WARNING, logger="real_world_rl.teleop.leader.cartesian"):
        reading = source.poll()
    assert reading.intervened  # no exception: an input loss, the follower holds where it is
    np.testing.assert_allclose(reading.action[:6], 0.0, atol=1e-9)
    assert "stalled: 1.000 s" in caplog.text and caplog.text.count("stalled") == 1
    assert not tracker.following
    np.testing.assert_allclose(tracker.hold_pose, follower.pose, atol=1e-12)

    clock.step_s = STEP_S
    reading = source.poll()  # re-anchored at the leader's current pose: no jump
    assert reading.intervened and tracker.following
    np.testing.assert_allclose(reading.action[:6], 0.0, atol=1e-9)
    leader.q = q + 3 * SMALL_DQ.astype(np.float32)
    assert np.abs(source.poll().action[:6]).max() > 1e-3


def test_a_clock_going_backwards_raises_and_holds():
    clock = _Clock()
    source = _source(_Leader(), _Follower(_ee_pose(READY_Q)), clock=clock)
    source.poll()  # take-over
    source.poll()
    clock.step_s = -0.1
    with pytest.raises(MissedControlDeadline, match="clock"):
        source.poll()
    assert not source.cartesian_tracker.following


def test_the_event_devices_take_over_and_hand_back_drive_the_tracker():
    class _Toggle(TeleopSource):
        action_mode = "cartesian_delta"
        arm_action_dim = 6
        has_gripper = True

        def __init__(self):
            self.intervened = False

        def poll(self):
            return TeleopReading(np.zeros(7, dtype=np.float32), self.intervened)

    source = _source(_Leader(intervening=False), _Follower(_ee_pose(READY_Q)))
    events = _Toggle()
    composite = CompositeTeleopSource(source, events)
    assert not composite.poll().intervened
    events.intervened = True  # the part's own signal is ignored under the composite
    assert composite.poll().intervened and source.cartesian_tracker.following
    events.intervened = False
    assert not composite.poll().intervened and not source.cartesian_tracker.following


# -- gripper --------------------------------------------------------------------------------


@pytest.mark.parametrize(
    "width, expected", [(1.0, 1.0), (0.8, 1.0), (0.51, 1.0), (0.5, -1.0), (0.3, -1.0), (0.0, -1.0)]
)
def test_gripper_width_is_binarized_at_half_travel(width, expected):
    follower = _Follower(_ee_pose(np.asarray(READY_Q, dtype=np.float32)))
    reading = _source(_Leader(gripper=width), follower).poll()
    assert reading.action.shape == (7,)
    assert reading.action[6] == expected


def test_gripper_threshold_none_keeps_the_continuous_value():
    follower = _Follower(_ee_pose(np.asarray(READY_Q, dtype=np.float32)))
    reading = _source(_Leader(gripper=0.7), follower, gripper_binarize_threshold=None).poll()
    assert reading.action[6] == pytest.approx(0.4)


def _executing(source, last):
    """Polls ``source``, keeping ``last[0]`` as the executed gripper action (as the wrapper does)."""

    def poll():
        reading = source.poll()
        if reading.intervened:
            last[0] = float(reading.action[6])
        return reading

    return poll


def test_take_over_keeps_the_gripper_until_the_trigger_matches_it():
    leader = _Leader(gripper=1.0, intervening=False)  # the trigger is open
    last = [-1.0]  # the gripper is closed (on an object)
    source = _source(leader, _Follower(_ee_pose(READY_Q)), last_gripper_action_getter=lambda: last[0])
    poll = _executing(source, last)
    assert poll().action[6] == 1.0  # not intervened: the trigger (unused)
    leader.intervening = True
    assert poll().action[6] == -1.0  # take-over: the open trigger is ignored
    assert poll().action[6] == -1.0
    leader.gripper = 0.2  # the trigger closes: it matches the gripper
    assert poll().action[6] == -1.0
    leader.gripper = 1.0  # from here on the trigger drives the gripper
    assert poll().action[6] == 1.0

    leader.intervening = False  # hand-back, then a take-over that agrees: no latch
    poll()
    leader.intervening = True
    assert poll().action[6] == 1.0
    leader.gripper = 0.0
    assert poll().action[6] == -1.0


def test_leader_without_gripper_holds_the_last_sent_gripper_action():
    follower = _Follower(_ee_pose(np.asarray(READY_Q, dtype=np.float32)))
    leader = _Leader(has_gripper=False)
    reading = _source(leader, follower, last_gripper_action_getter=lambda: -0.3).poll()
    assert reading.action[6] == pytest.approx(-0.3)
    with pytest.raises(ValueError, match="last_gripper_action_getter"):
        _source(_Leader(has_gripper=False), follower)


def test_leader_without_gripper_reads_its_arm_from_a_one_entry_gripper_vector():
    follower = _Follower(_ee_pose(np.asarray(READY_Q, dtype=np.float32)))
    leader = _Leader(has_gripper=False)
    reading = _source(leader, follower, last_gripper_action_getter=lambda: torch.tensor([-0.25])).poll()
    assert reading.action[6] == pytest.approx(-0.25)


def test_events_are_forwarded():
    leader = _Leader()
    source = _source(leader, _Follower(_ee_pose(leader.q)))
    leader.events = {"episode_end": True, "success": True}
    reading = source.poll()
    assert reading.episode_end is True and dict(reading.events) == {"success": True}
    reading = source.poll()
    assert reading.episode_end is False and not reading.events


# -- flags and the experiment contract --------------------------------------------------------

_TRACKER_FLAGS = dict(
    teleop_lead_cap_m=0.04, teleop_lead_cap_rad=0.2, teleop_target_speed_m_s=0.03, teleop_target_speed_rad_s=0.1
)
_TRACKER_KWARGS = {"lead_cap_m": 0.04, "lead_cap_rad": 0.2, "target_speed_m_s": 0.03, "target_speed_rad_s": 0.1}


def _cartesian_args(**overrides) -> HilSerlArgs:
    return HilSerlArgs(env_backend="real_robot", teleop_device="leader_cartesian", **overrides)


def test_tracker_flags_have_the_field_defaults():
    args = HilSerlArgs()
    assert (args.teleop_lead_cap_m, args.teleop_lead_cap_rad) == (0.05, 0.3)
    assert (args.teleop_target_speed_m_s, args.teleop_target_speed_rad_s) == (0.05, 0.3)


def test_tracker_flags_reach_the_source_and_the_contract_block():
    args = _cartesian_args(**_TRACKER_FLAGS)
    kwargs = env_building.teleop_wrapper_kwargs(args)
    assert kwargs == {"device": "leader_cartesian", **_TRACKER_KWARGS}
    composite = env_building.teleop_wrapper_kwargs(dataclasses.replace(args, teleop_event_device="stdin"))
    assert composite["events"] == "stdin" and composite["motion_kwargs"] == _TRACKER_KWARGS

    block = env_building.reward_sources_spec(args)["teleop_control"]
    assert block == {"version": "cartesian_tracker_v1", "lead_cap_m": 0.04, "lead_cap_rad": 0.2,
                     "speed_m_s": 0.03, "speed_rad_s": 0.1}
    source = _source(_Leader(), _Follower(_ee_pose(READY_Q)), **_TRACKER_KWARGS)
    assert source.cartesian_tracker.describe() == block
    assert "teleop_control" not in env_building.reward_sources_spec(HilSerlArgs(teleop_device="pico"))


def test_options_yaml_may_not_override_the_tracker(tmp_path):
    options = tmp_path / "leader.yaml"
    options.write_text("robot_yaml: gello.yaml\nlead_cap_m: 0.1\n", encoding="utf-8")
    with pytest.raises(ValueError, match="--teleop-lead-cap-m"):
        env_building.teleop_wrapper_kwargs(_cartesian_args(teleop_options_yaml=str(options)))
    options.write_text("robot_yaml: gello.yaml\n", encoding="utf-8")
    kwargs = env_building.teleop_wrapper_kwargs(_cartesian_args(teleop_options_yaml=str(options)))
    assert kwargs["robot_yaml"] == "gello.yaml" and kwargs["lead_cap_m"] == 0.05


_OBSERVATION = {"entries": {}, "camera_order": [], "frame_stack": 1, "convert_obs_rotation": False}
_BLOCK = CartesianTrackerConfig().describe()


def _contract(sources, *, with_observation=True) -> dict:
    experiment = {"sources": dict(sources)}
    if with_observation:
        experiment["observation"] = dict(_OBSERVATION)
    return {"action_mode": "cartesian_delta", "arm_action_dim": 6, "gripper_in_action": True,
            "experiment": experiment}


def test_demos_without_the_teleop_control_block_mismatch_a_run_with_it():
    with_block, without = _contract({"teleop_control": _BLOCK}), _contract({})
    with pytest.raises(ValueError, match="teleop_control"):
        validate_transitions_against_contract([], with_block, source="old.pkl", saved=without)
    with pytest.raises(ValueError, match="teleop_control"):
        validate_transitions_against_contract([], without, source="new.pkl", saved=with_block)
    other = _contract({"teleop_control": {**_BLOCK, "lead_cap_m": 0.04}})
    with pytest.raises(ValueError, match="teleop_control"):
        validate_transitions_against_contract([], with_block, source="other.pkl", saved=other)
    with pytest.warns(UserWarning, match="legacy"), pytest.raises(ValueError, match="teleop_control"):
        validate_transitions_against_contract(
            [], with_block, source="legacy.pkl", saved=_contract({}, with_observation=False)
        )

    validate_transitions_against_contract([], with_block, source="new.pkl", saved=with_block)
    validate_transitions_against_contract([], without, source="old.pkl", saved=without)  # old pkls stay readable


def test_describe_contract_shows_the_teleop_control_block():
    assert "teleop_control" in describe_contract(_contract({"teleop_control": _BLOCK}))
    assert "teleop_control" not in describe_contract(_contract({}))


def test_gello_cartesian_preset():
    from real_world_rl.demos.record import parse_record_args

    args = parse_record_args(["--config", str(PRESET), "--output-path", "demos/try.pkl"])
    assert args.teleop_device == "leader_cartesian" and args.teleop_event_device == "stdin"
    robot = args.real_robot
    assert robot.action_mode == "cartesian_delta" and robot.control_hz == 50
    assert (robot.xyz_scale_m, robot.rot_scale_rad) == (0.05, 0.3) and robot.gripper_in_action
    assert tuple(robot.state_fields) == ("tcp_pose", "tcp_vel", "joint_pos", "joint_vel")
    assert list(robot.safety_box) == [0.25, -0.40, 0.10, 0.80, 0.20, 0.65]
    assert robot.robot["realtime"] == "ignore" and robot.robot["relative_dynamics_factor"] == 0.1
    assert args.obs.state and tuple(args.obs.rgb) == () and tuple(args.obs.depth) == ()
    assert env_building.teleop_control_spec(args) == _BLOCK


# -- contracts and failures -------------------------------------------------------------------


def test_intervention_wrapper_accepts_it_on_a_cartesian_env_and_rejects_joint_envs():
    leader = _Leader()
    wrapper = InterventionWrapper(ContractEnv(), _source(leader, _Follower(_ee_pose(leader.q))))
    wrapper.reset()
    _, _, _, _, info = wrapper.step(torch.zeros(1, 7))
    assert info["intervened"] is True and info["intervene_action"].shape == (1, 7)

    with pytest.raises(ValueError, match="cartesian_delta.*joint_abs"):
        InterventionWrapper(
            ContractEnv("joint_abs", 7), _source(_Leader(), _Follower(_ee_pose(leader.q)))
        )


def test_registered_as_leader_cartesian():
    leader = _Leader()
    wrapper = TeleopInterventionWrapper(
        ContractEnv(),
        device="leader_cartesian",
        leader=leader,
        follower_pose_getter=_Follower(_ee_pose(leader.q)),
    )
    assert isinstance(wrapper.source, LeaderArmCartesianSource)
    assert wrapper.source.action_mode == "cartesian_delta" and wrapper.source.arm_action_dim == 6


@pytest.mark.parametrize(
    "attributes, match",
    [({"action_space": "cartesian_delta", "action_dim": 6}, "joint_abs"), ({"action_dim": 6}, "7 joints")],
)
def test_rejects_leaders_that_are_not_7_joint_abs(attributes, match):
    leader = _Leader()
    for key, value in attributes.items():
        setattr(leader, key, value)
    with pytest.raises(ValueError, match=match):
        _source(leader, _Follower(_ee_pose(READY_Q)))


def test_declared_contract_must_match_the_part():
    with pytest.raises(ValueError, match="declares"):
        _source(
            _Leader(has_gripper=False),
            _Follower(_ee_pose(READY_Q)),
            teleop_contract=("joint_abs", 7, True),
            last_gripper_action_getter=lambda: 1.0,
        )
    with pytest.raises(ValueError, match="joint_abs"):
        _source(lambda: None, _Follower(_ee_pose(READY_Q)), teleop_contract=("joint_delta", 7, True))


def test_resolved_part_is_checked_when_it_appears():
    parts = [None]
    source = _source(lambda: parts[0], _Follower(_ee_pose(READY_Q)))  # nothing to check yet
    with pytest.raises(DeviceFault, match="no teleop part"):
        source.poll()
    parts[0] = _Leader()
    parts[0].action_dim = 6
    with pytest.raises(DeviceFault, match="7 joints"):
        source.poll()


def test_failures_propagate_instead_of_idling():
    leader = _Leader()
    source = _source(leader, lambda: None)
    with pytest.raises(RuntimeError, match="before the env was reset"):
        source.poll()

    def failing_read(timeout_s=None):
        raise DeviceFault("bus error", device="leader")

    leader.read = failing_read
    with pytest.raises(DeviceFault, match="bus error"):
        source.poll()


def test_an_input_loss_while_following_holds_the_measured_pose():
    leader = _Leader()
    follower = _Follower(_ee_pose(READY_Q))
    source = _source(leader, follower)
    source.poll()  # take-over
    assert source.cartesian_tracker.following

    def failing_read(timeout_s=None):
        raise DeviceFault("bus error", device="leader")

    leader.read = failing_read
    with pytest.raises(DeviceFault):
        source.poll()
    assert not source.cartesian_tracker.following
    np.testing.assert_allclose(source.cartesian_tracker.hold_pose, follower.pose)


def test_exactly_one_leader():
    with pytest.raises(ValueError, match="exactly one"):
        LeaderArmCartesianSource(follower_pose_getter=lambda: None)
    with pytest.raises(ValueError, match="exactly one leader"):
        LeaderArmCartesianSource.from_env(ContractEnv(), follower_pose_getter=lambda: None)
    with pytest.raises(ValueError, match="exactly one leader"):
        LeaderArmCartesianSource.from_env(
            ContractEnv(),
            leader=_Leader(),
            remote_address="10.0.0.2:50051",
            follower_pose_getter=lambda: None,
        )


# -- follower discovery from the env --------------------------------------------------------


class _FakeBridge(FrankaBridgeClient):
    """In-memory bridge: the robot reaches every sent pose."""

    def __init__(self, pose):
        self.pose = np.asarray(pose, dtype=np.float64)
        self.gripper_calls = []

    def reset_joints(self):
        pass

    def get_state(self):
        return {
            "pose": self.pose.tolist(),
            "vel": [0.0] * 6,
            "gripper_pos": 0.0,
            "force": [0.0] * 3,
            "torque": [0.0] * 3,
        }

    def send_pose(self, pose):
        self.pose = np.asarray(pose, dtype=np.float64)

    def send_gripper(self, open_gripper):
        self.gripper_calls.append(bool(open_gripper))


def test_franka_real_pose_scales_and_box_found_through_the_wrapper_chain_and_followed():
    q = np.asarray(READY_Q, dtype=np.float32)
    target = _ee_pose(q)
    bridge = _FakeBridge(_offset_pose(q, [0.06, -0.004, 0.05], [0.0, 0.0, 0.2]))
    cfg = FrankaRealEnvConfig(
        bridge_url="http://unused",
        action_scale=(SCALE_M, SCALE_RAD),
        reward_scale=2.0,  # adds a RewardScaleBiasWrapper above FrankaRealEnv
        max_episode_steps=1000,
        safety_box_low=(-10.0, -10.0, -10.0),
        safety_box_high=(10.0, 10.0, 10.0),
    )
    env = TeleopInterventionWrapper(
        make_franka_real_env(cfg, bridge_client=bridge),
        device="leader_cartesian",
        leader=_Leader(q, gripper=1.0),
        mapping="absolute",
        clock=_Clock(),
    )
    assert env.source.cartesian_tracker.safety_box.tolist() == [-10.0] * 3 + [10.0] * 3
    env.reset()
    _, _, _, _, info = env.step(torch.zeros(1, 7))
    np.testing.assert_allclose(info["intervene_action"][0, :6].numpy(), 0.0, atol=1e-6)  # no jump
    for _ in range(30):
        env.step(torch.zeros(1, 7))
    np.testing.assert_allclose(bridge.pose[:3], target[:3], atol=1e-8)
    assert _rotation_error(bridge.pose, target[3:]) < 1e-7
    assert bridge.gripper_calls[-1] is True


_ENV_BOX = [-1.0, -1.0, -1.0, 1.0, 1.0, 1.5]


class _CartesianRealEnv(ContractEnv):
    """``RealRobotEnv``-shaped: a ``last_observation`` property and a cartesian contract with
    scales and a configured safety box."""

    def __init__(self, tcp_pose):
        super().__init__()
        self.action_contract.update(xyz_scale_m=SCALE_M, rot_scale_rad=SCALE_RAD,
                                    experiment={"safety_box": list(_ENV_BOX)})
        self._tcp_pose = np.asarray(tcp_pose, dtype=np.float32)
        self._observation = None

    @property
    def last_observation(self):
        return self._observation

    def reset(self, **kwargs):
        self._observation = {"state": {"tcp_pose": self._tcp_pose.copy()}}
        return super().reset(**kwargs)


def test_real_robot_cartesian_env_pose_and_box_come_from_the_last_observation_and_contract():
    q = np.asarray(READY_Q, dtype=np.float32)
    env = _CartesianRealEnv(_offset_pose(q, [0.004, 0.0, 0.0], [0.0, 0.0, 0.0]))
    source = LeaderArmCartesianSource.from_env(env, leader=_Leader(q), mapping="absolute", clock=_Clock())
    assert source.cartesian_tracker.safety_box.tolist() == _ENV_BOX
    with pytest.raises(RuntimeError, match="before the env was reset"):
        source.poll()
    env.reset()
    np.testing.assert_allclose(source.poll().action[:6], 0.0, atol=1e-6)  # take-over at the hold pose
    np.testing.assert_allclose(source.poll().action[:6], [-0.08, 0.0, 0.0, 0.0, 0.0, 0.0], atol=1e-4)


def test_the_recorders_hold_shares_the_sources_tracker():
    from real_world_rl.teleop.core.hold import CartesianHold, hold_action_fn_for

    q = np.asarray(READY_Q, dtype=np.float32)
    env = _CartesianRealEnv(_ee_pose(q))
    leader = _Leader(q, intervening=False)
    source = LeaderArmCartesianSource.from_env(env, leader=leader, clock=_Clock())
    wrapper = InterventionWrapper(env, source)
    hold = hold_action_fn_for(wrapper)
    assert isinstance(hold, CartesianHold) and hold.tracker is source.cartesian_tracker
    wrapper.reset()
    hold.reset()

    reset_pose = env.last_observation["state"]["tcp_pose"].astype(np.float64)
    drifted = reset_pose.copy()
    drifted[0] += 0.003
    env._observation["state"]["tcp_pose"] = drifted.astype(np.float32)
    np.testing.assert_allclose(hold(1.0).numpy(), [-0.06, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0], atol=1e-4)

    leader.intervening = True
    wrapper.step(torch.zeros(1, 7))  # take-over: the operator's action replaces the hold's
    np.testing.assert_allclose(hold(1.0).numpy()[:6], 0.0)
    leader.intervening = False
    wrapper.step(torch.zeros(1, 7))  # hand-back: the source re-samples the shared hold here
    np.testing.assert_allclose(source.cartesian_tracker.hold_pose, drifted, atol=1e-6)
    np.testing.assert_allclose(hold(1.0).numpy()[:6], 0.0, atol=1e-4)

    from real_world_rl.teleop.core.fake import ScriptedTeleopSource

    composite = CompositeTeleopSource(
        LeaderArmCartesianSource.from_env(env, leader=_Leader(q)), ScriptedTeleopSource([TeleopReading.idle(7)])
    )
    assert hold_action_fn_for(InterventionWrapper(env, composite)).tracker is composite.motion.cartesian_tracker


def test_env_without_a_follower_pose_is_rejected():
    with pytest.raises(ValueError, match="follower's TCP pose"):
        LeaderArmCartesianSource.from_env(ContractEnv(), leader=_Leader())


# -- leader on a stub robot (daemon on the robot host, or a local yaml) ------------------------


def _leader_teleop_config(script) -> TeleopConfig:
    return TeleopConfig(
        name="gello",
        backend="fake",
        options={"action_space": "joint_abs", "num_joints": 7, "script": script, "on_end": "hold_last"},
    )


def _gello_station(script) -> FrankaRobot:
    return FrankaRobot(
        FrankaConfig(
            name="gello_station",
            arm_backend="fake",
            gripper=None,
            cameras=[],
            teleop=_leader_teleop_config(script),
        )
    )


def test_remote_stub_robot_leader_is_connected_by_start_and_released_by_close():
    moved = (np.asarray(READY_Q) + SMALL_DQ).tolist()
    script = [
        {"action": READY_Q, "gripper": 1.0, "active": False},
        {"action": READY_Q, "gripper": 0.0, "active": True},
        {"action": moved, "active": True},
    ]
    service = RobotService(_gello_station(script), watchdog_s=30.0)
    robot = RemoteRobot.from_client(LocalClient(service))
    follower = _Follower(_offset_pose(READY_Q, [0.02, 0.01, -0.01], [0.0, 0.1, 0.0]))
    try:
        source = LeaderArmCartesianSource(leader_robot=robot, follower_pose_getter=follower, clock=_Clock())
        assert not robot.is_connected and source.leader_robot is robot
        wrapper = InterventionWrapper(ContractEnv(), source)  # validates, then start() connects
        assert robot.is_connected
        wrapper.reset()

        _, _, _, _, info = wrapper.step(torch.zeros(1, 7))
        assert info["intervened"] is False

        _, _, _, _, info = wrapper.step(torch.zeros(1, 7))
        assert info["intervention_onset"] is True
        np.testing.assert_allclose(info["intervene_action"][0, :6].numpy(), 0.0, atol=1e-6)
        assert info["intervene_action"][0, 6].item() == -1.0

        _, _, _, _, info = wrapper.step(torch.zeros(1, 7))
        assert np.abs(info["intervene_action"][0, :6].numpy()).max() > 1e-3

        wrapper.close()
        assert not robot.is_connected
    finally:
        if robot.is_connected:
            robot.disconnect()
        service.close()


def test_start_releases_the_leader_robot_when_its_teleop_part_is_not_a_leader_arm():
    station = FrankaRobot(
        FrankaConfig(
            name="gello_station",
            arm_backend="fake",
            gripper=None,
            cameras=[],
            teleop=TeleopConfig(name="spacemouse", backend="fake"),  # cartesian_delta
        )
    )
    service = RobotService(station, watchdog_s=30.0)
    robot = RemoteRobot.from_client(LocalClient(service))
    try:
        source = LeaderArmCartesianSource(leader_robot=robot, follower_pose_getter=lambda: None)
        with pytest.raises(DeviceFault, match="joint_abs"):
            source.start()
        assert not robot.is_connected
    finally:
        if robot.is_connected:
            robot.disconnect()
        service.close()


def test_from_env_with_remote_address_builds_an_unconnected_remote_robot():
    source = LeaderArmCartesianSource.from_env(
        ContractEnv(), remote_address="10.0.0.2:50051", follower_pose_getter=lambda: None
    )
    robot = source.leader_robot
    assert isinstance(robot, RemoteRobot)
    assert robot.config.address == "10.0.0.2:50051" and not robot.is_connected
    source.close()  # never connected: a no-op


def test_from_env_with_robot_yaml_builds_and_owns_a_local_robot(tmp_path):
    robot_yaml = tmp_path / "gello_station.yaml"
    teleop = _leader_teleop_config([{"action": READY_Q, "gripper": 1.0, "active": True}])
    robot_yaml.write_text(
        yaml.safe_dump(
            {
                "type": "franka",
                "name": "gello_station",
                "arm_backend": "fake",
                "gripper": None,
                "cameras": [],
                "teleop": {"name": teleop.name, "backend": teleop.backend, "options": teleop.options},
            }
        )
    )
    source = LeaderArmCartesianSource.from_env(
        ContractEnv(), robot_yaml=str(robot_yaml), follower_pose_getter=_Follower(_ee_pose(READY_Q))
    )
    try:
        source.start()
        reading = source.poll()
        assert reading.intervened is True and reading.action[6] == 1.0
    finally:
        source.close()
    assert not source.leader_robot.is_connected


def test_gello_station_example_yaml_is_a_stub_robot_with_every_gello_option():
    path = Path(__file__).resolve().parents[1] / "examples" / "stations" / "robot.gello_station.example.yaml"
    data = yaml.safe_load(path.read_text())
    assert data["type"] == "franka"
    config = FrankaConfig.from_dict(data)
    assert config.name == "gello_station" and config.arm_backend == "fake"
    assert config.gripper is None and config.cameras == []
    assert config.teleop.backend == "gello"
    assert set(config.teleop.options) == GELLO_OPTION_KEYS
    assert len(config.teleop.options["joint_ids"]) == 7
