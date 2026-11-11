"""CartesianTargetTracker: a fixed hold pose, a persistent command advanced at a bounded rate and
capped to lead the measured pose, and the cartesian_delta action that lands the arm on it."""
from __future__ import annotations

import math

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from real_world_rl.teleop.cartesian_tracker import (
    TRACKER_VERSION,
    CartesianTargetTracker,
    CartesianTrackerConfig,
    CommandOutOfRange,
    MissedControlDeadline,
)

XYZ_SCALE_M, ROT_SCALE_RAD = 0.05, 0.3
DT_50_HZ = 0.02
START = np.array([0.5, 0.0, 0.4])


def _pose(position, rotvec=(0.0, 0.0, 0.0), base_rotvec=(math.pi, 0.0, 0.0)) -> np.ndarray:
    """``position`` with the rotation ``rotvec`` applied (base frame) to a downward-facing TCP."""
    rotation = Rotation.from_rotvec(rotvec) * Rotation.from_rotvec(base_rotvec)
    return np.concatenate([np.asarray(position, dtype=np.float64), rotation.as_quat()])


def _env_update(measured, action) -> np.ndarray:
    """``RealRobotEnv``'s cartesian_delta target: measured + action * scale (rotation left-multiplied)."""
    position = measured[:3] + action[:3] * XYZ_SCALE_M
    rotation = Rotation.from_rotvec(action[3:6] * ROT_SCALE_RAD) * Rotation.from_quat(measured[3:])
    return np.concatenate([position, rotation.as_quat()])


def _angle(a, b) -> float:
    return (Rotation.from_quat(a[3:]) * Rotation.from_quat(b[3:]).inv()).magnitude()


def _following(start=START, **config) -> CartesianTargetTracker:
    tracker = CartesianTargetTracker(CartesianTrackerConfig(**config))
    tracker.reset(_pose(start))
    tracker.take_over()
    return tracker


# -- hold ---------------------------------------------------------------------------------


def test_hold_is_exactly_the_hold_pose_under_drifting_measurements():
    hold = _pose(START)
    tracker = CartesianTargetTracker()
    tracker.reset(hold)
    rng = np.random.default_rng(0)
    for _ in range(20):
        drifted = _pose(START + rng.uniform(-0.02, 0.02, 3), rng.uniform(-0.1, 0.1, 3))
        action = tracker.action_for(drifted, XYZ_SCALE_M, ROT_SCALE_RAD)
        target = _env_update(drifted, action)
        np.testing.assert_allclose(target[:3], hold[:3], atol=1e-12)
        assert _angle(target, hold) < 1e-9
        np.testing.assert_array_equal(tracker.hold_pose, hold)
        np.testing.assert_array_equal(tracker.command, hold)


def test_an_impedance_lagging_arm_has_no_net_drift_after_many_polls():
    """The arm moves halfway to the commanded equilibrium per step and is pushed ~2 mm every
    step; with the fixed hold its error settles at two pushes (a zero action, which re-anchors
    the equilibrium at the measured pose, would accumulate 0.2 m)."""
    hold = _pose(START)
    tracker = CartesianTargetTracker()
    tracker.reset(hold)
    measured = hold.copy()
    push = np.array([0.002, -0.001, 0.0])
    for _ in range(100):
        target = _env_update(measured, tracker.action_for(measured, XYZ_SCALE_M, ROT_SCALE_RAD))
        measured = measured.copy()
        measured[:3] = measured[:3] + 0.5 * (target[:3] - measured[:3]) + push
    assert np.linalg.norm(measured[:3] - hold[:3]) <= 2 * np.linalg.norm(push) + 1e-9


def test_hand_back_resamples_the_hold_at_the_measured_pose():
    tracker = _following()
    moved = _pose(START + [0.01, 0.0, 0.0])
    tracker.advance(_pose(START + [0.2, 0.0, 0.0]), moved, 0.3)
    assert tracker.following
    tracker.hold_at(moved)
    assert not tracker.following
    np.testing.assert_array_equal(tracker.hold_pose, moved)
    np.testing.assert_array_equal(tracker.command, moved)
    np.testing.assert_allclose(tracker.action_for(moved, XYZ_SCALE_M, ROT_SCALE_RAD), 0.0, atol=1e-12)


def test_take_over_starts_from_the_current_command_without_a_jump():
    tracker = CartesianTargetTracker()
    hold = _pose(START)
    tracker.reset(hold)
    drifted = _pose(START + [0.004, 0.0, 0.0])
    held_action = tracker.action_for(drifted, XYZ_SCALE_M, ROT_SCALE_RAD)
    tracker.take_over()
    update = tracker.advance(tracker.command, drifted, 0.0)
    np.testing.assert_array_equal(update.command, hold)
    np.testing.assert_array_equal(tracker.action_for(drifted, XYZ_SCALE_M, ROT_SCALE_RAD), held_action)


# -- advance --------------------------------------------------------------------------------


def test_rate_limit_moves_a_millimetre_per_step_at_50_hz():
    tracker = _following()
    requested = _pose(START + [1.0, 0.0, 0.0], rotvec=(0.0, 0.0, 1.0))
    measured = _pose(START)
    previous = tracker.command
    for _ in range(10):
        command = tracker.advance(requested, measured, DT_50_HZ).command
        assert np.linalg.norm(command[:3] - previous[:3]) == pytest.approx(0.05 * DT_50_HZ, abs=1e-12)
        assert _angle(command, previous) == pytest.approx(0.3 * DT_50_HZ, abs=1e-9)
        measured, previous = command, command  # the arm lands on every command


def test_lead_cap_bounds_the_command_and_the_action_when_the_leader_jumps():
    tracker = _following()
    measured = _pose(START)  # the arm does not move
    requested = _pose(START + [0.3, 0.0, 0.0], rotvec=(0.0, 1.2, 0.0))
    for _ in range(20):
        command = tracker.advance(requested, measured, 0.3).command
        assert np.linalg.norm(command[:3] - measured[:3]) <= 0.05 + 1e-12
        assert _angle(command, measured) <= 0.3 + 1e-9
        action = tracker.action_for(measured, XYZ_SCALE_M, ROT_SCALE_RAD)
        assert np.all(np.abs(action) <= 1.0)
    np.testing.assert_allclose(action, [1.0, 0.0, 0.0, 0.0, 1.0, 0.0], atol=1e-9)


def test_excess_is_dropped_so_a_returning_leader_causes_no_overshoot():
    tracker = _following()
    measured = _pose(START)
    for _ in range(10):  # the leader jumps 0.3 m while the arm is stuck: the command caps at 0.05 m
        tracker.advance(_pose(START + [0.3, 0.0, 0.0]), measured, 0.3)
    returned = _pose(START + [0.02, 0.0, 0.0])  # the leader comes back before the arm caught up
    furthest = 0.0
    for _ in range(50):
        command = tracker.advance(returned, measured, 0.3).command
        measured = _env_update(measured, tracker.action_for(measured, XYZ_SCALE_M, ROT_SCALE_RAD))
        furthest = max(furthest, measured[0] - START[0])
        np.testing.assert_allclose(measured, command, atol=1e-12)
    np.testing.assert_allclose(measured[:3], returned[:3], atol=1e-12)
    assert furthest <= 0.05 + 1e-12


def test_workspace_rejection_locks_the_hold_at_the_measured_pose():
    tracker = CartesianTargetTracker(safety_box=[0.25, -0.4, 0.1, 0.8, 0.2, 0.65])
    tracker.reset(_pose(START))
    tracker.take_over()
    measured = _pose(START + [0.01, 0.0, 0.0])
    update = tracker.advance(_pose([0.9, 0.3, 0.4]), measured, DT_50_HZ)
    assert len(update.workspace_violations) == 2
    assert update.workspace_violations[0].startswith("x=0.9000 outside [0.2500, 0.8000]")
    assert update.workspace_violations[1].startswith("y=0.3000")
    assert not tracker.following
    np.testing.assert_array_equal(tracker.hold_pose, measured)
    with pytest.raises(RuntimeError, match="take_over"):
        tracker.advance(_pose(START), measured, DT_50_HZ)
    pushed = _pose(START + [0.03, 0.0, 0.0])
    target = _env_update(pushed, tracker.action_for(pushed, XYZ_SCALE_M, ROT_SCALE_RAD))
    np.testing.assert_allclose(target[:3], measured[:3], atol=1e-12)


@pytest.mark.parametrize("dt", [-0.01, math.inf, math.nan])
def test_a_broken_clock_raises(dt):
    tracker = _following()
    with pytest.raises(MissedControlDeadline, match="clock"):
        tracker.advance(_pose(START), _pose(START), dt)


def test_a_missed_deadline_holds_at_the_measured_pose():
    tracker = _following()
    measured = _pose(START + [0.01, 0.0, 0.0])
    update = tracker.advance(_pose(START + [0.3, 0.0, 0.0]), measured, 0.31)
    assert update.missed_deadline_s == pytest.approx(0.31) and update.held
    assert not update.workspace_violations
    assert not tracker.following
    np.testing.assert_array_equal(tracker.hold_pose, measured)
    np.testing.assert_array_equal(update.command, measured)
    tracker.take_over()  # following again from the hold pose
    assert tracker.advance(_pose(START), measured, 0.02).missed_deadline_s is None


@pytest.mark.parametrize("dt", [0.0, 0.3, 0.9 - 0.6, 1.2 - 0.8999999999999999])
def test_the_deadline_bounds_are_inclusive(dt):
    """The last two are 0.3 plus clock-difference rounding (0.30000000000000004, ...16): not a stall."""
    update = _following().advance(_pose(START), _pose(START), dt)
    assert update.missed_deadline_s is None and not update.held


def test_reach_for_hold_returns_to_the_hold_one_capped_step_at_a_time():
    """A cap of one env step (0.02 m): an arm pushed 0.05 m off the hold gets a saturated action
    back, never an error, and the command converges onto the hold pose."""
    xyz_scale_m = 0.02
    tracker = CartesianTargetTracker(CartesianTrackerConfig(lead_cap_m=xyz_scale_m))
    hold = _pose(START)
    tracker.reset(hold)
    measured = _pose(START + [0.05, 0.0, 0.0])
    actions = []
    for _ in range(4):
        tracker.reach_for_hold(measured)
        action = tracker.action_for(measured, xyz_scale_m, ROT_SCALE_RAD)
        actions.append(action[0])
        position = measured[:3] + action[:3] * xyz_scale_m
        measured = np.concatenate([position, measured[3:]])
    np.testing.assert_allclose(actions, [-1.0, -1.0, -0.5, 0.0], atol=1e-9)
    np.testing.assert_allclose(tracker.command, hold, atol=1e-12)
    np.testing.assert_array_equal(tracker.hold_pose, hold)
    tracker.take_over()
    with pytest.raises(RuntimeError, match="hold"):
        tracker.reach_for_hold(measured)


# -- action -----------------------------------------------------------------------------------


@pytest.mark.parametrize(
    "measured",
    [_pose(START + [0.051, 0.0, 0.0]), _pose(START, rotvec=(0.0, 0.0, 0.31))],
)
def test_action_for_raises_instead_of_clipping(measured):
    tracker = CartesianTargetTracker()
    tracker.reset(_pose(START))
    with pytest.raises(CommandOutOfRange, match="refusing to clip"):
        tracker.action_for(measured, XYZ_SCALE_M, ROT_SCALE_RAD)


def test_action_for_inverts_the_env_update():
    tracker = _following()
    measured = _pose(START + [0.01, -0.02, 0.005], rotvec=(0.05, -0.1, 0.02))
    command = tracker.advance(_pose(START + [0.01, 0.0, 0.0]), measured, 0.3).command
    target = _env_update(measured, tracker.action_for(measured, XYZ_SCALE_M, ROT_SCALE_RAD))
    np.testing.assert_allclose(target[:3], command[:3], atol=1e-12)
    assert _angle(target, command) < 1e-9


def test_the_tracker_needs_a_reset_first():
    tracker = CartesianTargetTracker()
    assert not tracker.initialized
    with pytest.raises(RuntimeError, match="reset"):
        tracker.action_for(_pose(START), XYZ_SCALE_M, ROT_SCALE_RAD)
    with pytest.raises(RuntimeError, match="reset"):
        tracker.take_over()


# -- config -----------------------------------------------------------------------------------


@pytest.mark.parametrize("field", ["lead_cap_m", "lead_cap_rad", "speed_m_s", "speed_rad_s", "max_dt_s"])
@pytest.mark.parametrize("value", [0.0, -0.1, math.nan, math.inf])
def test_config_values_must_be_positive_and_finite(field, value):
    with pytest.raises(ValueError, match=field):
        CartesianTrackerConfig(**{field: value})


def test_config_defaults_and_contract_block():
    config = CartesianTrackerConfig()
    assert (config.lead_cap_m, config.lead_cap_rad, config.speed_m_s, config.speed_rad_s, config.max_dt_s) == (
        0.05, 0.3, 0.05, 0.3, 0.3
    )
    assert CartesianTargetTracker(config).describe() == {
        "version": TRACKER_VERSION,
        "lead_cap_m": 0.05,
        "lead_cap_rad": 0.3,
        "speed_m_s": 0.05,
        "speed_rad_s": 0.3,
    }
    assert TRACKER_VERSION == "cartesian_tracker_v1"


@pytest.mark.parametrize("box", [[0.0] * 5, [0.5, 0.0, 0.0, 0.4, 1.0, 1.0], [0.0, 0.0, 0.0, math.nan, 1.0, 1.0]])
def test_safety_box_is_validated(box):
    with pytest.raises(ValueError, match="safety_box"):
        CartesianTargetTracker(safety_box=box)


def test_task_workspace_rejection_holds_measured_and_requires_takeover():
    transform = np.eye(4)
    transform[:3, :3] = Rotation.from_euler('x', 40, degrees=True).as_matrix()
    transform[:3, 3] = [.5, 0, .3]
    tracker = CartesianTargetTracker(workspace={'frame': 'table', 'low': [-.1]*3, 'high': [.1]*3},
        task_frame={'name': 'table', 'base_T_frame': transform.tolist()})
    measured = np.r_[transform[:3, 3], [0, 0, 0, 1]]
    tracker.reset(measured)
    tracker.take_over()
    requested = measured.copy()
    requested[:3] += transform[:3, :3] @ [0, 0, .11]
    result = tracker.advance(requested, measured, .02)
    assert result.workspace_violations
    assert not tracker.following
    np.testing.assert_array_equal(tracker.command, measured)
    tracker.take_over()
    assert not tracker.advance(measured, measured, .02).workspace_violations


def test_task_workspace_measured_violation_is_fault_not_recovery():
    from robot_infra.devices.errors import SafetyViolation
    tracker = CartesianTargetTracker(workspace={'frame': 'base', 'low': [-1]*3, 'high': [1]*3})
    pose = np.array([0, 0, 0, 0, 0, 0, 1.])
    tracker.reset(pose)
    tracker.take_over()
    measured = pose.copy()
    measured[2] = 2
    with pytest.raises(SafetyViolation, match='measured TCP'):
        tracker.advance(pose, measured, .02)
    np.testing.assert_array_equal(tracker.command, pose)


def test_base_box_only_does_not_reject_lead_capped_candidate_below_face():
    """B2: HEAD checks requested against the base box, not the candidate."""
    from real_world_rl.teleop.cartesian_tracker import _limited_towards
    tracker = CartesianTargetTracker(CartesianTrackerConfig(speed_m_s=1.),
                                     safety_box=[-1, -1, 0, 1, 1, 1])
    start = np.array([0, 0, 0, 0, 0, 0, 1.])
    measured = start.copy()
    measured[2] = -.0005
    requested = start.copy()
    requested[0] = .3
    tracker.reset(start)
    tracker.take_over()
    advanced = _limited_towards(start, requested, .1, .03)
    expected = _limited_towards(measured, advanced, .05, .3)
    assert expected[2] < 0
    update = tracker.advance(requested, measured, .1)
    assert not update.held
    assert tracker.following
    np.testing.assert_array_equal(update.command, expected)


@pytest.mark.parametrize('outside,raises', [(.001, False), (.004, True)])
def test_tracker_measured_workspace_margin(outside, raises):
    from robot_infra.devices.errors import SafetyViolation
    tracker = CartesianTargetTracker(workspace={'low': [-1, -1, 0], 'high': [1]*3})
    pose = np.array([0, 0, 0, 0, 0, 0, 1.])
    tracker.reset(pose)
    tracker.take_over()
    measured = pose.copy()
    measured[2] = -outside
    if raises:
        with pytest.raises(SafetyViolation, match='measured TCP'):
            tracker.advance(pose, measured, .01)
    else:
        assert not tracker.advance(pose, measured, .01).held
