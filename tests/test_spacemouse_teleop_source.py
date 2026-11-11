"""SpaceMouseTeleopSource: the puck as a velocity input over the cartesian command tracker,
button 0 as the take-over / hand-back toggle, button 1 as the gripper toggle. Most tests drive a
fake ``cartesian_delta`` teleop part (no thread, a scripted clock); the device-layer tests use a
mocked ``hid`` module (no hardware). Covers the velocity integration and the lead cap,
the release, the button edges, the gripper, the workspace and stall holds, the flags and the
experiment-contract block, the composite with the operator keys, one recorder run and the
intervention wrapper over the real device layer. The last test covers ``spacemouse_twist``, the
old raw-twist source kept for the sim examples."""
from __future__ import annotations

import dataclasses
import logging
import sys

import numpy as np
import pytest
import torch
from _teleop_fakes import HOME_TCP, CartesianPoseEnv, ContractEnv, FakeEnv, make_hid
from scipy.spatial.transform import Rotation

from robot_infra.devices import DeviceFault, SdkMissing
from robot_infra.devices.teleop import EVENT_KEYS, TeleopSample

from real_world_rl.demos.record import RecordDemosArgs, record_demos
from real_world_rl.envs.wrappers.intervention import TeleopInterventionWrapper
from real_world_rl.teleop import TeleopReading, make_teleop_source
from real_world_rl.teleop.cartesian_tracker import CartesianTrackerConfig
from real_world_rl.teleop.composite import CompositeTeleopSource
from real_world_rl.teleop.hold import CartesianHold, hold_action_fn_for
from real_world_rl.teleop.keyboard import OperatorKeySource
from real_world_rl.teleop.spacemouse import (
    DEFAULT_DEADZONE,
    SpaceMouseTeleopSource,
    SpaceMouseTwistTeleopSource,
)
from real_world_rl.training import env_building
from real_world_rl.training.entrypoints.hil_serl import HilSerlArgs

SCALE_M, SCALE_RAD = 0.05, 0.3
# Seconds between two polls of the scripted clock.
STEP_S = 0.1
SPEED_M_S, SPEED_RAD_S = 0.05, 0.3  # the tracker's default target speeds
LEAD_CAP_M = 0.05
OPEN, CLOSED = 1.0, -1.0
FORWARD_X = np.array([1.0, 0.0, 0.0, 0.0, 0.0, 0.0])
NO_GRIPPER = {"action_mode": "cartesian_delta", "arm_action_dim": 6, "gripper_in_action": False}


class _Part:
    """A ``cartesian_delta`` teleop part read without a thread: the next ``read()`` returns
    ``twist`` and the button ``levels``; ``press(index)`` latches a press event, as the device
    layer does for a press released before the read. ``intervening`` (the device layer's
    ``active``) is set whenever the puck moves, which the source must ignore."""

    action_space = "cartesian_delta"
    action_dim = 6
    has_gripper = True
    name = "spacemouse"

    def __init__(self):
        self.twist = np.zeros(6)
        self.levels = [False, False]
        self.pressed: set = set()
        self.reset_calls = 0
        self._seq = 0

    def press(self, *buttons):
        self.pressed.update(buttons)

    def read(self, timeout_s=None):
        self._seq += 1
        events = {key: False for key in EVENT_KEYS}
        events.update({f"button_{index}": True for index in self.pressed})
        self.pressed = set()
        return TeleopSample(
            action=np.asarray(self.twist, dtype=np.float32),
            action_space=self.action_space,
            gripper=1.0,
            buttons={f"button_{index}": level for index, level in enumerate(self.levels)},
            events=events,
            intervening=bool(np.any(self.twist)) or any(self.levels),
            timestamp=0.0,
            seq=self._seq,
        )

    def reset(self):
        self.reset_calls += 1
        self.pressed = set()


class _ScriptedPart(_Part):
    """A ``_Part`` that applies ``script[read index]`` (``twist`` and/or ``press``) before each read."""

    def __init__(self, script):
        super().__init__()
        self.script = dict(script)
        self.reads = 0

    def read(self, timeout_s=None):
        step = self.script.get(self.reads, {})
        if "twist" in step:
            self.twist = np.asarray(step["twist"], dtype=np.float64)
        self.press(*step.get("press", ()))
        self.reads += 1
        return super().read(timeout_s)


class _Clock:
    def __init__(self, step_s=STEP_S):
        self.now = 0.0
        self.step_s = step_s

    def __call__(self):
        self.now += self.step_s
        return self.now


class _Follower:
    """The follower TCP pose; ``apply`` lands it on ``pose + action * scale`` unless ``moves`` is
    false (a robot that does not follow)."""

    def __init__(self, pose=HOME_TCP, *, moves=True):
        self.pose = np.asarray(pose, dtype=np.float64).copy()
        self.moves = moves

    def __call__(self):
        return self.pose.copy()

    def apply(self, action):
        if not self.moves:
            return
        action = np.asarray(action, dtype=np.float64)
        rotation = Rotation.from_rotvec(action[3:6] * SCALE_RAD) * Rotation.from_quat(self.pose[3:])
        self.pose = np.concatenate([self.pose[:3] + action[:3] * SCALE_M, rotation.as_quat()])


class _Session:
    """A source over a ``_Part``, polled as the intervention wrapper does: an intervened action is
    executed on the follower and its gripper becomes the last sent gripper action."""

    def __init__(self, follower=None, **kwargs):
        self.part = _Part()
        self.follower = _Follower() if follower is None else follower
        self.last = [OPEN]
        kwargs.setdefault("clock", _Clock())
        self.source = SpaceMouseTeleopSource(
            self.part,
            follower_pose_getter=self.follower,
            last_gripper_action_getter=lambda: self.last[0],
            **kwargs,
        )
        self.source.reset()

    @property
    def tracker(self):
        return self.source.cartesian_tracker

    def poll(self) -> TeleopReading:
        reading = self.source.poll()
        if reading.intervened:
            self.follower.apply(reading.action[:6])
            self.last[0] = float(reading.action[6])
        return reading

    def take_over(self) -> TeleopReading:
        self.part.press(0)
        reading = self.poll()
        assert reading.intervened
        return reading


def _source(part=None, **kwargs) -> SpaceMouseTeleopSource:
    kwargs.setdefault("follower_pose_getter", _Follower())
    kwargs.setdefault("last_gripper_action_getter", lambda: OPEN)
    return SpaceMouseTeleopSource(_Part() if part is None else part, **kwargs)


def test_declares_the_cartesian_twist_plus_gripper_space():
    source = _Session().source
    assert (source.action_mode, source.arm_action_dim, source.has_gripper) == ("cartesian_delta", 6, True)
    gripperless = _source(env_contract=NO_GRIPPER, last_gripper_action_getter=None)
    assert gripperless.has_gripper is False


# -- motion ----------------------------------------------------------------------------------


def test_moving_the_puck_alone_never_intervenes():
    session = _Session()
    session.part.twist = np.array([1.0, -1.0, 1.0, 1.0, 1.0, 1.0])
    for _ in range(5):
        reading = session.poll()
        assert not reading.intervened and not reading.action[:6].any()
    assert not session.tracker.following
    np.testing.assert_array_equal(session.follower.pose, HOME_TCP)


@pytest.mark.parametrize("axis, polls", [(0, 4), (2, 6)])
def test_a_constant_twist_advances_the_command_by_speed_times_dt_per_poll(axis, polls):
    session = _Session()
    np.testing.assert_allclose(session.take_over().action[:6], 0.0, atol=1e-9)  # dt 0: no jump
    twist = np.zeros(6)
    twist[axis] = 1.0
    session.part.twist = twist
    for _ in range(polls):
        reading = session.poll()
        assert reading.intervened
        # One poll moves the command speed * dt = 5 mm ahead of the arm: 0.1 of the env scale.
        np.testing.assert_allclose(reading.action[axis], SPEED_M_S * STEP_S / SCALE_M, atol=1e-6)
    expected = HOME_TCP.copy()
    expected[axis] += SPEED_M_S * STEP_S * polls
    np.testing.assert_allclose(session.tracker.command, expected, atol=1e-6)
    np.testing.assert_allclose(session.follower.pose, expected, atol=1e-6)


def test_a_constant_rotation_twist_turns_the_command_at_the_rotation_speed():
    session = _Session()
    session.take_over()
    session.part.twist = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.5])
    for _ in range(4):
        session.poll()
    command = session.tracker.command
    turned = Rotation.from_quat(command[3:]) * Rotation.from_quat(HOME_TCP[3:]).inv()
    np.testing.assert_allclose(turned.as_rotvec(), [0.0, 0.0, 0.5 * SPEED_RAD_S * STEP_S * 4], atol=1e-6)
    np.testing.assert_allclose(command[:3], HOME_TCP[:3], atol=1e-9)


def test_a_robot_that_does_not_follow_caps_the_command_and_a_release_stops_within_the_cap():
    session = _Session(follower=_Follower(moves=False))
    session.take_over()
    session.part.twist = FORWARD_X
    for _ in range(30):  # 0.15 m requested in total
        reading = session.poll()
    command = session.tracker.command
    np.testing.assert_allclose(command[:3] - HOME_TCP[:3], [LEAD_CAP_M, 0.0, 0.0], atol=1e-9)
    np.testing.assert_allclose(reading.action[:6], [1.0, 0.0, 0.0, 0.0, 0.0, 0.0], atol=1e-6)

    session.part.twist = np.zeros(6)  # the puck is released, then the arm catches up
    session.follower.moves = True
    for _ in range(5):
        assert session.poll().intervened
    np.testing.assert_allclose(session.tracker.command, command, atol=1e-12)  # nothing queued
    np.testing.assert_allclose(session.follower.pose, command, atol=1e-6)


def test_rest_noise_within_the_deadzone_does_not_move_the_command():
    assert DEFAULT_DEADZONE == 0.05
    session = _Session()
    session.take_over()
    session.part.twist = np.full(6, 0.04)
    for _ in range(5):
        np.testing.assert_allclose(session.poll().action[:6], 0.0, atol=1e-9)
    np.testing.assert_array_equal(session.tracker.command, HOME_TCP)
    session.part.twist = np.array([0.04, 0.0, 0.5, 0.0, 0.0, 0.0])
    action = session.poll().action
    assert action[0] == 0.0
    assert action[2] == pytest.approx(0.5 * SPEED_M_S * STEP_S / SCALE_M, abs=1e-6)
    with pytest.raises(ValueError, match="deadzone"):
        _source(deadzone=1.0)


# -- buttons ---------------------------------------------------------------------------------


def test_button_0_takes_over_and_the_next_press_hands_back_at_the_measured_pose():
    session = _Session()
    session.take_over()
    assert session.tracker.following
    session.part.twist = FORWARD_X
    for _ in range(3):
        session.poll()
    session.part.twist = np.zeros(6)
    session.part.press(0)
    reading = session.poll()
    assert not reading.intervened and not session.tracker.following
    np.testing.assert_allclose(session.tracker.hold_pose, session.follower.pose)  # re-sampled here
    assert session.tracker.hold_pose[0] - HOME_TCP[0] == pytest.approx(3 * SPEED_M_S * STEP_S, abs=1e-6)
    session.part.press(0)
    assert session.poll().intervened


def test_a_held_button_toggles_once_and_a_press_between_two_reads_still_counts():
    session = _Session()
    session.poll()  # the level baseline
    session.part.levels[0] = True  # seen as a level only
    assert session.poll().intervened
    for _ in range(3):
        assert session.poll().intervened  # still held: not a new press
    session.part.levels[0] = False
    assert session.poll().intervened
    session.part.press(0)  # pressed and released between two reads: only its latched event shows
    assert not session.poll().intervened
    session.part.levels[0] = True
    session.part.press(0)  # one press seen both ways: one toggle
    assert session.poll().intervened


def test_a_button_held_through_the_env_reset_does_not_take_over():
    session = _Session()
    session.part.levels[0] = True
    session.source.reset()
    assert session.part.reset_calls == 2  # the device's latched presses are dropped at each reset
    assert not session.poll().intervened
    assert not session.poll().intervened


def test_take_over_starts_from_the_last_sent_gripper_action_and_button_1_toggles_it():
    session = _Session()
    session.last[0] = CLOSED  # the policy closed the gripper on an object
    assert session.take_over().action[6] == CLOSED  # taking over does not open it
    assert session.poll().action[6] == CLOSED
    session.part.press(1)
    assert session.poll().action[6] == OPEN
    assert session.poll().action[6] == OPEN  # sticky
    session.part.press(1)
    assert session.poll().action[6] == CLOSED


def test_a_continuous_last_gripper_action_is_kept_until_the_first_press():
    session = _Session()
    session.last[0] = 0.3
    assert session.take_over().action[6] == pytest.approx(0.3)
    session.part.press(1)
    assert session.poll().action[6] == CLOSED  # 0.3 counts as open: the press closes


def test_button_1_without_a_take_over_leaves_the_gripper_alone():
    session = _Session()
    session.last[0] = CLOSED
    session.part.press(1)
    reading = session.poll()
    assert not reading.intervened and reading.action[6] == CLOSED
    assert session.take_over().action[6] == CLOSED


def test_a_gripper_env_needs_the_last_gripper_action_getter():
    with pytest.raises(ValueError, match="last_gripper_action_getter"):
        _source(last_gripper_action_getter=None)
    part = _Part()
    source = _source(part, env_contract=NO_GRIPPER, last_gripper_action_getter=None, clock=_Clock())
    source.reset()
    part.press(0)
    reading = source.poll()
    assert reading.intervened and reading.action.shape == (6,)


# -- holds and failures ----------------------------------------------------------------------


def test_a_request_outside_the_safety_box_hands_back_until_the_next_press(caplog):
    session = _Session(safety_box=[0.0, -1.0, 0.0, 0.512, 1.0, 1.0])  # x up to 12 mm ahead
    session.take_over()
    session.part.twist = FORWARD_X
    with caplog.at_level(logging.WARNING, logger="real_world_rl.teleop.spacemouse"):
        readings = [session.poll() for _ in range(3)]
    assert [reading.intervened for reading in readings] == [True, True, False]
    assert "WORKSPACE HOLD: x=" in caplog.text
    assert not session.tracker.following
    np.testing.assert_allclose(session.tracker.hold_pose, session.follower.pose)
    session.part.twist = np.zeros(6)
    assert not session.poll().intervened  # stays handed back
    session.part.press(0)
    assert session.poll().intervened  # one press takes over again


def test_a_stalled_control_loop_holds_and_the_next_poll_continues(caplog):
    clock = _Clock()
    session = _Session(clock=clock)
    session.take_over()
    session.part.twist = FORWARD_X
    session.poll()
    clock.step_s = 1.0
    with caplog.at_level(logging.WARNING, logger="real_world_rl.teleop.spacemouse"):
        reading = session.poll()
    assert reading.intervened and "stalled: 1.000 s" in caplog.text
    np.testing.assert_allclose(reading.action[:6], 0.0, atol=1e-9)
    assert not session.tracker.following
    np.testing.assert_allclose(session.tracker.hold_pose, session.follower.pose)

    clock.step_s = STEP_S
    reading = session.poll()  # following again from the hold, without a jump
    assert reading.intervened and session.tracker.following
    np.testing.assert_allclose(reading.action[:6], 0.0, atol=1e-9)
    assert session.poll().action[0] == pytest.approx(SPEED_M_S * STEP_S / SCALE_M, abs=1e-6)


def test_a_read_failure_propagates_and_hands_back():
    session = _Session()
    session.take_over()
    read = session.part.read

    def failing_read(timeout_s=None):
        raise DeviceFault("hid error", device="spacemouse")

    session.part.read = failing_read
    with pytest.raises(DeviceFault, match="hid error"):
        session.poll()
    assert not session.tracker.following
    session.part.read = read
    assert not session.poll().intervened
    session.last[0] = CLOSED
    assert session.take_over().action[6] == CLOSED  # a new take-over: the gripper restarts too


def test_polling_before_the_env_reset_raises():
    with pytest.raises(RuntimeError, match="before the env was reset"):
        _source(follower_pose_getter=lambda: None).poll()


def test_rejects_a_part_that_is_not_a_cartesian_twist_and_scales_below_the_lead_cap():
    part = _Part()
    part.action_space, part.action_dim = "joint_abs", 7
    with pytest.raises(ValueError, match="cartesian_delta"):
        _source(part)
    with pytest.raises(ValueError, match="xyz_scale_m >= lead_cap_m"):
        _source(pos_scale_m=0.02)


# -- the env, the flags and the experiment contract ------------------------------------------


def test_registered_as_spacemouse_and_built_from_a_real_robot_env():
    box = [-1.0, -1.0, -1.0, 1.0, 1.0, 1.5]
    env = CartesianPoseEnv(safety_box=box)
    part = _Part()
    source = make_teleop_source(
        "spacemouse", env, teleop=part, last_gripper_action_getter=lambda: OPEN, clock=_Clock()
    )
    assert isinstance(source, SpaceMouseTeleopSource)
    assert source.cartesian_tracker.safety_box.tolist() == box
    with pytest.raises(RuntimeError, match="before the env was reset"):
        source.poll()
    env.reset()
    source.reset()
    part.press(0)
    assert source.poll().intervened
    np.testing.assert_allclose(source.cartesian_tracker.hold_pose, HOME_TCP)


def test_env_without_a_follower_pose_is_rejected():
    with pytest.raises(ValueError, match="follower's TCP pose"):
        make_teleop_source("spacemouse", ContractEnv(), teleop=_Part(), last_gripper_action_getter=lambda: OPEN)


_TRACKER_FLAGS = dict(
    teleop_lead_cap_m=0.04, teleop_lead_cap_rad=0.2, teleop_target_speed_m_s=0.03, teleop_target_speed_rad_s=0.1
)
_TRACKER_KWARGS = {"lead_cap_m": 0.04, "lead_cap_rad": 0.2, "target_speed_m_s": 0.03, "target_speed_rad_s": 0.1}
_DEFAULT_TRACKER_KWARGS = {"lead_cap_m": 0.05, "lead_cap_rad": 0.3, "target_speed_m_s": 0.05, "target_speed_rad_s": 0.3}


def test_tracker_flags_reach_the_source_and_the_contract_block():
    args = HilSerlArgs(
        env_backend="real_robot", teleop_device="spacemouse", teleop_event_device="stdin", **_TRACKER_FLAGS
    )
    assert env_building.teleop_wrapper_kwargs(args) == {
        "device": "composite",
        "motion": "spacemouse",
        "events": "stdin",
        "intervention_from": "events",
        "motion_kwargs": _TRACKER_KWARGS,
    }
    block = env_building.reward_sources_spec(args)["teleop_control"]
    assert block == CartesianTrackerConfig(lead_cap_m=0.04, lead_cap_rad=0.2, speed_m_s=0.03, speed_rad_s=0.1).describe()
    assert block == {"version": "cartesian_tracker_v1", "lead_cap_m": 0.04, "lead_cap_rad": 0.2,
                     "speed_m_s": 0.03, "speed_rad_s": 0.1}
    # The block is the tracker's alone: the same for a GELLO through leader_cartesian.
    gello = dataclasses.replace(args, teleop_device="leader_cartesian")
    assert env_building.reward_sources_spec(gello)["teleop_control"] == block
    assert _source(**_TRACKER_KWARGS).cartesian_tracker.describe() == block


def test_options_yaml_configures_the_device_but_not_the_tracker(tmp_path):
    options = tmp_path / "spacemouse.yaml"
    options.write_text("device_index: 1\ndeadzone: 0.08\nlead_cap_m: 0.1\n", encoding="utf-8")
    args = HilSerlArgs(teleop_device="spacemouse", teleop_options_yaml=str(options))
    with pytest.raises(ValueError, match="--teleop-lead-cap-m"):
        env_building.teleop_wrapper_kwargs(args)
    options.write_text("device_index: 1\ndeadzone: 0.08\n", encoding="utf-8")
    assert env_building.teleop_wrapper_kwargs(args) == {
        "device": "spacemouse", **_DEFAULT_TRACKER_KWARGS, "device_index": 1, "deadzone": 0.08,
    }


@pytest.mark.parametrize('mode', ['tracker', 'serl'])
@pytest.mark.parametrize('validation', ['teleop_wrapper_kwargs', 'resolve_teleop_input'])
def test_failed_options_validation_leaves_teleop_input_unfrozen(tmp_path, mode, validation):
    options = tmp_path / 'spacemouse.yaml'
    options.write_text('device_index: 1\nlead_cap_m: 0.1\n', encoding='utf-8')
    args = HilSerlArgs(teleop_device='spacemouse', spacemouse_mode=mode,
                       teleop_options_yaml=str(options))
    with pytest.raises(ValueError, match='lead_cap_m'):
        getattr(env_building, validation)(args)
    assert args.teleop_input is None

    options.write_text('device_index: 2\n', encoding='utf-8')
    assert env_building._single_arm_yaml_options(args) == {'device_index': 2}
    env_building.teleop_wrapper_kwargs(args)
    assert args.teleop_input is None  # successful reads also leave freezing to explicit resolution
    env_building.resolve_teleop_input(args)
    assert args.teleop_input['source_options'] == {'device_index': 2}
    options.unlink()
    env_building.resolve_teleop_input(args)  # restored numeric options need no YAML
    assert args.teleop_input['source_options'] == {'device_index': 2}


def test_the_spacemouse_is_no_longer_an_event_device():
    (field,) = [field for field in dataclasses.fields(HilSerlArgs) if field.name == "teleop_event_device"]
    assert "spacemouse" not in str(field.type) and "stdin" in str(field.type)
    args = HilSerlArgs(teleop_device="leader_cartesian", teleop_event_device="spacemouse")
    with pytest.raises(ValueError, match="--teleop-event-device spacemouse was removed.*stdin"):
        env_building.teleop_wrapper_kwargs(args)


# -- with the operator keys ------------------------------------------------------------------


class _ScriptedKeys(OperatorKeySource):
    """The operator keys, typed just before the poll they are listed for (``{poll index: keys}``)."""

    def __init__(self, script, **kwargs):
        super().__init__(**kwargs)
        self._script = dict(script)
        self.polls = 0

    def poll(self):
        for key in self._script.get(self.polls, ()):
            self._keys.press(key)
        self.polls += 1
        return super().poll()


def test_composite_takes_the_labels_from_the_keys_and_the_take_over_from_button_0(caplog):
    session = _Session()
    composite = CompositeTeleopSource(session.source, _ScriptedKeys({1: ["t"], 3: ["s"], 4: ["esc"]}))
    assert not composite.poll().intervened
    with caplog.at_level(logging.WARNING, logger="real_world_rl.teleop.spacemouse"):
        reading = composite.poll()  # t: ignored, with a warning
    assert not reading.intervened and "ignores the event device's take-over" in caplog.text
    session.part.press(0)
    assert composite.poll().intervened
    reading = composite.poll()
    assert reading.intervened and dict(reading.events) == {"success": True}
    reading = composite.poll()
    assert reading.intervened and reading.episode_end


def test_recording_a_demo_with_the_spacemouse_and_scripted_operator_events():
    """One recorder episode through the intervention wrapper's registry path: take over with
    button 0, move 3 polls along x, close the gripper with button 1, then success + episode end
    from the event device. The recorder's hold shares the source's tracker."""
    env = CartesianPoseEnv()
    part = _ScriptedPart({
        1: {"press": (0,)},
        2: {"twist": FORWARD_X},
        4: {"press": (1,)},
        5: {"twist": np.zeros(6)},
    })
    end = TeleopReading(np.zeros(7, dtype=np.float32), intervened=False, episode_end=True, events={"success": True})
    wrapper = TeleopInterventionWrapper(
        env,
        device="composite",
        motion="spacemouse",
        events="fake",
        motion_kwargs={"teleop": part, "clock": _Clock()},
        events_kwargs={"readings": [TeleopReading.idle(7)] * 5 + [end]},
    )
    source = wrapper.source.motion
    hold = hold_action_fn_for(wrapper)
    assert isinstance(source, SpaceMouseTeleopSource)
    assert isinstance(hold, CartesianHold) and hold.tracker is source.cartesian_tracker

    episodes = []
    transitions = record_demos(
        wrapper,
        RecordDemosArgs(num_successes=1, control_hz=1e6, max_episodes=1),
        hold_action_fn=hold,
        on_episode=episodes.append,
    )
    assert len(episodes) == 1 and episodes[0].success
    actions = torch.cat([torch.as_tensor(row["action"]).reshape(1, -1) for row in transitions]).numpy()
    assert actions.shape == (5, 7)  # the 5 intervened steps; the idle first step is not recorded
    assert np.abs(actions).max() <= 1.0
    np.testing.assert_allclose(actions[:, 0], [0.0, 0.1, 0.1, 0.1, 0.0], atol=1e-6)
    np.testing.assert_allclose(actions[:, 6], [OPEN, OPEN, OPEN, CLOSED, CLOSED])
    np.testing.assert_allclose(env.tcp[:3], HOME_TCP[:3] + [3 * SPEED_M_S * STEP_S, 0.0, 0.0], atol=1e-6)


# -- the device layer (mocked hidapi) --------------------------------------------------


def test_hid_fake_queues_button_changes_as_separate_reports():
    sdk = make_hid()
    device = sdk.device()
    try:
        assert device.read(13)[0] == 1
        sdk.buttons = (1, 0)
        assert device.read(13) == [3, 1]
        assert device.read(13)[0] == 1  # a held button emits no new button report
        sdk.buttons = (0, 0)
        sdk.buttons = (1, 0)  # release and re-press before the next HID read
        assert device.read(13) == [3, 0]
        assert device.read(13) == [3, 1]
        sdk.buttons = (1, 0)  # assigning the same level emits nothing
        assert device.read(13)[0] == 1
    finally:
        device.close()


@pytest.fixture
def hid_sdk(monkeypatch):
    fake = make_hid()
    monkeypatch.setitem(sys.modules, "hid", fake)
    return fake


def _step_until(env, predicate, attempts=400):
    """Steps until the mocked device state has propagated through the device poll thread."""
    for _ in range(attempts):
        info = env.step(torch.zeros(1, 7))[4]
        if predicate(info):
            return info
    raise AssertionError("the mocked device state never reached a step")


def test_the_owned_device_is_opened_by_start_and_closed_by_close(hid_sdk):
    source = SpaceMouseTeleopSource(
        follower_pose_getter=_Follower(), last_gripper_action_getter=lambda: OPEN, device_index=3
    )
    assert hid_sdk.devices == []  # building opens nothing
    source.start()
    try:
        assert hid_sdk.devices[0].kwargs == {"device_index": 3}
    finally:
        source.close()
    assert hid_sdk.devices[0].closed
    source.close()  # safe twice


def test_missing_sdk_only_fails_at_start(monkeypatch):
    monkeypatch.setitem(sys.modules, "hid", None)  # makes `import hid` fail
    source = SpaceMouseTeleopSource(follower_pose_getter=_Follower(), last_gripper_action_getter=lambda: OPEN)
    source.reset()  # not connected: nothing to reset
    with pytest.raises(SdkMissing) as error:
        source.start()
    assert error.value.hint == "pip install -e '.[spacemouse]'"


def test_the_wrapper_takes_over_and_hands_back_on_button_0_of_the_device(hid_sdk):
    env = CartesianPoseEnv()
    wrapped = TeleopInterventionWrapper(env, device="spacemouse", device_index=1)
    try:
        assert hid_sdk.devices[0].kwargs == {"device_index": 1}
        wrapped.reset()
        hid_sdk.twist = (0.5, 0.0, 0.0, 0.0, 0.0, 0.0)  # HID reader x: output axis 1 (base y)
        for _ in range(20):
            assert wrapped.step(torch.zeros(1, 7))[4]["intervened"] is False  # the puck alone
        hid_sdk.buttons = (1, 0)
        _step_until(wrapped, lambda info: info["intervened"])
        hid_sdk.buttons = (0, 0)
        info = _step_until(wrapped, lambda info: info["intervened"] and info["intervene_action"][0, 1] > 0)
        assert info["intervene_action"].abs().max() <= 1.0
        assert wrapped.source.cartesian_tracker.following
        hid_sdk.twist = (0.0,) * 6
        hid_sdk.buttons = (1, 0)
        _step_until(wrapped, lambda info: not info["intervened"])
        assert not wrapped.source.cartesian_tracker.following
    finally:
        wrapped.close()
    assert hid_sdk.devices[0].closed and env.close_calls == 1


# -- spacemouse_twist: the old raw-twist semantics, sim only ---------------------------------


def _poll_until(source, predicate, attempts=300):
    """Polls until the mocked device state has propagated through the device poll thread."""
    for _ in range(attempts):
        reading = source.poll()
        if predicate(reading):
            return reading
    raise AssertionError("the mocked device state never reached a reading")


def test_spacemouse_twist_keeps_the_raw_twist_motion_intervention_and_old_buttons(hid_sdk):
    source = make_teleop_source("spacemouse_twist", FakeEnv(), device_index=2, hold_s=0.0)
    assert isinstance(source, SpaceMouseTwistTeleopSource)
    assert (source.action_mode, source.arm_action_dim, source.has_gripper) == ("cartesian_delta", 6, True)
    source.start()
    try:
        assert hid_sdk.devices[0].kwargs == {"device_index": 2}
        assert not source.poll().intervened
        hid_sdk.twist = (0.1, 0.2, 0.3, 0.4, 0.5, 0.6)  # x, y, z, roll, pitch, yaw
        reading = _poll_until(source, lambda r: r.intervened and r.action[0] != 0)  # motion intervenes
        np.testing.assert_allclose(reading.action, [-0.2, 0.1, 0.3, -0.4, -0.5, -0.6, OPEN], atol=1e-6)
        assert not reading.episode_end

        hid_sdk.twist = (0.0,) * 6
        hid_sdk.buttons = (1, 0)
        _poll_until(source, lambda r: r.action[6] == CLOSED)  # button 0: the sticky gripper
        hid_sdk.buttons = (0, 1)
        reading = _poll_until(source, lambda r: r.episode_end)  # button 1: episode end
        assert reading.intervened  # a held button counts as operating the device
        assert not source.poll().episode_end  # one press, one episode end
    finally:
        source.close()
    assert hid_sdk.devices[0].closed


def test_task_workspace_hands_back_and_button_can_retake():
    session = _Session(workspace={'frame': 'base', 'low': [0, -1, 0], 'high': [.512, 1, 1]})
    session.take_over()
    session.part.twist = FORWARD_X
    assert [session.poll().intervened for _ in range(3)] == [True, True, False]
    np.testing.assert_allclose(session.tracker.hold_pose, session.follower.pose)
    session.part.twist = np.zeros(6)
    assert not session.poll().intervened
    session.part.press(0)
    assert session.poll().intervened
