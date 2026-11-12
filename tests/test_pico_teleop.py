"""The Pico as a first-class device: the device-layer ``PicoTeleop`` part over a mocked ZeroMQ
(polling, latched presses, the button baseline, health, the SDK import), and ``device="pico"``,
the anchored pose source over the cartesian command tracker (known-answer mapping, bind take-over
and hand-back, the workspace hold, the gripper, episode end, failures, ownership of the part)."""
from __future__ import annotations

import logging
import subprocess
import sys
import time

import numpy as np
import pytest
from _teleop_fakes import HOME_TCP, CartesianPoseEnv, make_zmq, pico_record
from scipy.spatial.transform import Rotation

from real_world_rl.devices import DeviceFault, SdkMissing, Teleop, TeleopConfig
from real_world_rl.devices.teleop import EVENT_KEYS, TeleopSample
from real_world_rl.teleop import make_teleop_source
from real_world_rl.teleop.core.registry import source_class
from real_world_rl.teleop.pico import backend as pico_backend
from real_world_rl.teleop.pico.backend import PicoTeleop, hand_pose_of
from real_world_rl.teleop.pico.source import TARGET_MAPPING, PicoTeleopSource

SCALE_M, SCALE_RAD = 0.05, 0.3  # CartesianPoseEnv's env scales
STEP_S = 0.1  # seconds between two polls of the scripted clock
IDENTITY = (0.0, 0.0, 0.0, 1.0)


@pytest.fixture
def zmq_sdk(monkeypatch):
    fake = make_zmq()
    monkeypatch.setitem(sys.modules, "zmq", fake)
    monkeypatch.setattr(pico_backend, "_sleep", lambda seconds: None)
    return fake


def _part(**options) -> PicoTeleop:
    return PicoTeleop(TeleopConfig(name="pico", backend="pico", options=options))


def _wait(predicate, timeout_s: float = 2.0) -> None:
    deadline = time.monotonic() + timeout_s
    while not predicate():
        if time.monotonic() > deadline:
            raise AssertionError("the mocked Pico state never reached the part")
        time.sleep(0.002)


def _read_until(part, predicate, attempts: int = 200):
    for _ in range(attempts):
        sample = part.read()
        if predicate(sample):
            return sample
    raise AssertionError("the mocked Pico state never reached a sample")


# -- the device-layer part ------------------------------------------------------------------


def test_registered_with_its_host_declarations():
    assert Teleop.backend("pico") is PicoTeleop
    assert (PicoTeleop.ACTION_SPACE, PicoTeleop.HAS_GRIPPER) == ("cartesian_delta", True)
    assert PicoTeleop.SDK == "zmq" and PicoTeleop.PIP_REQUIREMENTS == ("pyzmq",)
    # A network publisher: no USB ids, no udev rule, no serial group, nothing to discover.
    assert not PicoTeleop.UDEV_SUBSYSTEMS and PicoTeleop.SERIAL_GROUP is None
    assert PicoTeleop.usb_ids() == frozenset() and PicoTeleop.discover() == set()


def test_the_host_doctor_checks_the_pico_extra():
    from real_world_rl.doctor import host as doctor_host

    assert doctor_host.sdk_checks()["pico"] == ("zmq", "print('import OK')", ["pyzmq"])


def test_a_robot_yaml_teleop_section_builds_the_part_for_the_daemon():
    import real_world_rl.devices.backends  # noqa: F401  (registers backends; SDK imports are lazy)
    import real_world_rl.devices.robots  # noqa: F401  (registers robot types)
    from real_world_rl.devices import Robot

    robot = Robot.from_dict({
        "type": "franka", "arm_backend": "fake", "gripper": None,
        "teleop": {"backend": "pico", "hold_s": 0.2, "options": {"address": "tcp://pico:7777", "hand": "left"}},
    })
    assert isinstance(robot.teleop, PicoTeleop) and robot.teleop.address == "tcp://pico:7777"
    assert robot.teleop.config.hold_s == 0.2


def test_a_missing_sdk_raises_sdk_missing_with_the_extra(monkeypatch):
    monkeypatch.setitem(sys.modules, "zmq", None)  # makes `import zmq` fail
    part = _part()
    with pytest.raises(SdkMissing) as info:
        part.connect()
    assert "pyzmq" in info.value.hint and ".[pico]" in info.value.hint
    assert not part.is_connected


@pytest.mark.parametrize(
    "options,match",
    [
        ({"port": 1}, "unknown pico option"),
        ({"hand": "both"}, "hand"),
        ({"bind_button": "trigger"}, "bind_button"),
        ({"poll_interval_s": -1.0}, "poll_interval_s"),
    ],
)
def test_invalid_options_are_rejected(options, match):
    with pytest.raises(ValueError, match=match):
        _part(**options)


def test_subscribes_to_the_address_and_topic_and_releases_the_socket(zmq_sdk):
    part = _part(address="tcp://pico:5555", topic="right")
    handle = part._open_device()
    (socket,) = zmq_sdk.sockets
    assert (socket.address, socket.subscription) == ("tcp://pico:5555", "right")
    part._close_device(handle)
    assert socket.closed and zmq_sdk.terminated == 1


def test_a_poll_reads_the_pose_trigger_and_displacement(zmq_sdk):
    part = _part()
    handle = part._open_device()
    assert part._poll_device(handle) is None  # nothing published yet
    zmq_sdk.publish(pico_record((0.1, 0.2, 0.3), trigger=1.0))
    reading = part._poll_device(handle)
    assert reading.diagnostics["hand_pose"] == [0.1, 0.2, 0.3, *IDENTITY]
    assert reading.gripper == 0.0  # trigger pulled: closed
    np.testing.assert_array_equal(reading.action, np.zeros(6))  # no previous pose yet
    assert reading.events_now == {"bind": False, "episode_end": False} and not reading.active

    turned = Rotation.from_rotvec([0.0, 0.0, 0.05]).as_quat()
    zmq_sdk.publish(pico_record((0.11, 0.2, 0.3), turned))
    reading = part._poll_device(handle)
    np.testing.assert_allclose(reading.action, [0.01, 0.0, 0.0, 0.0, 0.0, 0.05], atol=1e-9)
    assert reading.gripper == 1.0  # trigger released: open


def test_presses_between_two_polls_are_latched_and_the_newest_pose_wins(zmq_sdk):
    part = _part()
    handle = part._open_device()
    zmq_sdk.publish(pico_record())
    part._poll_device(handle)  # the baseline
    zmq_sdk.publish(pico_record((0.0, 0.0, 0.1), bind=True))
    zmq_sdk.publish(pico_record((0.0, 0.0, 0.2), bind=False, episode_end=True))
    reading = part._poll_device(handle)
    assert reading.events_now == {"bind": True, "episode_end": True}
    assert reading.buttons["bind"] is False and reading.buttons["episode_end"] is True
    assert reading.diagnostics["hand_pose"][:3] == [0.0, 0.0, 0.2] and reading.diagnostics["messages"] == 2


def test_buttons_held_at_connect_or_through_a_reset_are_not_presses(zmq_sdk):
    part = _part()
    handle = part._open_device()
    zmq_sdk.publish(pico_record(bind=True, episode_end=True))
    reading = part._poll_device(handle)
    assert reading.events_now == {"bind": False, "episode_end": False}
    assert reading.active and reading.buttons["bind"]
    part._reset_device(handle)
    zmq_sdk.publish(pico_record(bind=True, episode_end=True))
    assert not any(part._poll_device(handle).events_now.values())


def test_hand_and_bind_button_select_the_record_and_its_buttons(zmq_sdk):
    part = _part(hand="left", bind_button="episode_end")
    handle = part._open_device()
    right = pico_record((2.0, 0.0, 0.0))
    zmq_sdk.publish(pico_record((1.0, 0.0, 0.0)) + right)
    assert part._poll_device(handle).diagnostics["hand_pose"][0] == 1.0
    zmq_sdk.publish(pico_record((1.0, 0.0, 0.0), episode_end=True) + right)
    assert part._poll_device(handle).events_now == {"bind": True, "episode_end": False}
    zmq_sdk.publish(pico_record((1.0, 0.0, 0.0), bind=True) + right)
    assert part._poll_device(handle).events_now == {"bind": False, "episode_end": True}


def test_a_topic_message_carries_the_payload_in_its_last_frame(zmq_sdk):
    part = _part(topic="right")
    handle = part._open_device()
    zmq_sdk.publish(pico_record((0.5, 0.0, 0.0)), topic="right")
    assert part._poll_device(handle).diagnostics["hand_pose"][0] == 0.5


@pytest.mark.parametrize("payload", ["not json", "[1, 2, 3]", "[" + ", ".join(["0"] * 13) + "]"])
def test_a_malformed_message_fails_the_poll(zmq_sdk, payload):
    part = _part()
    handle = part._open_device()
    zmq_sdk.publish(payload)  # not JSON / wrong size / zero quaternion
    with pytest.raises(ValueError):
        part._poll_device(handle)


def test_the_poll_thread_feeds_read_latches_bind_and_reports_health(zmq_sdk):
    part = _part()
    part.connect()
    try:
        zmq_sdk.publish(pico_record())
        _read_until(part, lambda sample: True)
        zmq_sdk.publish(pico_record((0.0, 0.1, 0.0), bind=True))
        sample = _read_until(part, lambda sample: sample.events["bind"])
        assert sample.action_space == "cartesian_delta" and sample.intervening
        np.testing.assert_allclose(hand_pose_of(sample), [0.0, 0.1, 0.0, *IDENTITY])
        assert part.health().ok

        zmq_sdk.publish("not json")
        _wait(lambda: part.health().consecutive_failures == 1)
        health = part.health()
        assert not health.ok and health.thread_alive and health.error_count == 1
        zmq_sdk.publish(pico_record())
        _wait(lambda: part.health().ok)
    finally:
        part.disconnect()
    assert zmq_sdk.sockets[0].closed


def test_hand_pose_of_rejects_a_sample_from_another_part():
    sample = TeleopSample(action=np.zeros(6, np.float32), action_space="cartesian_delta", gripper=1.0,
                          buttons={}, events={}, intervening=False, timestamp=0.0, seq=1)
    with pytest.raises(ValueError, match="hand_pose"):
        hand_pose_of(sample)


# -- the anchored source ----------------------------------------------------------------------


class _PicoPart:
    """A Pico part read without a thread: the next ``read()`` returns ``pose`` (Pico frame) and the
    trigger ``width``; ``press(event)`` latches ``bind``/``episode_end`` until that read."""

    action_space, action_dim, has_gripper, name = "cartesian_delta", 6, True, "pico"

    def __init__(self):
        self.pose = np.array([0.0, 0.0, 0.0, *IDENTITY])
        self.width = 1.0
        self.pending: set = set()
        self.reset_calls = 0
        self.failure = None
        self.carries_pose = True

    def press(self, *events):
        self.pending.update(events)

    def read(self, timeout_s=None):
        if self.failure is not None:
            raise self.failure
        events = {key: False for key in EVENT_KEYS}
        events.update({event: True for event in self.pending})
        self.pending = set()
        return TeleopSample(
            action=np.zeros(6, np.float32), action_space=self.action_space, gripper=self.width,
            buttons={}, events=events, intervening=False, timestamp=0.0, seq=0,
            diagnostics={"hand_pose": self.pose.tolist()} if self.carries_pose else {},
        )

    def reset(self):
        self.reset_calls += 1


class _Clock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now


def _source(env=None, **kwargs):
    env = CartesianPoseEnv() if env is None else env
    part, clock = _PicoPart(), _Clock()
    source = make_teleop_source("pico", env, teleop=part, clock=clock, **kwargs)
    env.reset()
    source.reset()
    return source, part, clock


def _poll(source, clock):
    clock.now += STEP_S
    return source.poll()


def _take_over(source, part, clock):
    part.press("bind")
    reading = _poll(source, clock)
    assert reading.intervened
    np.testing.assert_allclose(reading.action[:6], 0.0, atol=1e-9)  # dt = 0 at the take-over
    return reading


def test_the_source_declares_the_tracker_capabilities():
    cls = source_class("pico")
    assert cls is PicoTeleopSource
    assert cls.USES_LAST_GRIPPER_ACTION and cls.DRIVES_ONE_ARM and cls.TRACKS_CARTESIAN_TARGET
    assert not (cls.REQUIRES_FULL_ACTION or cls.RECORDS_TELEOP_INPUT or cls.SUPPORTS_INPUT_DIAGNOSTICS)
    source, _, _ = _source()
    assert (source.action_mode, source.arm_action_dim, source.has_gripper) == ("cartesian_delta", 6, True)
    assert source.cartesian_tracker.initialized and not source.cartesian_tracker.following


def test_moving_the_hand_alone_never_intervenes():
    source, part, clock = _source()
    part.pose[:3] = [0.004, 0.0, 0.0]
    for _ in range(3):
        assert not _poll(source, clock).intervened


def test_a_hand_translation_moves_the_follower_by_the_same_displacement():
    source, part, clock = _source()
    part.pose = np.array([0.3, -0.2, 1.1, *Rotation.from_rotvec([0.4, 0.1, -0.2]).as_quat()])
    _take_over(source, part, clock)
    part.pose[:3] += [0.004, 0.0, -0.002]  # within one step's target speed (0.005 m)
    reading = _poll(source, clock)
    assert reading.intervened
    np.testing.assert_allclose(reading.action[:6], [0.004 / SCALE_M, 0.0, -0.002 / SCALE_M, 0.0, 0.0, 0.0],
                               atol=1e-6)
    np.testing.assert_allclose(source.cartesian_tracker.command[:3], HOME_TCP[:3] + [0.004, 0.0, -0.002], atol=1e-9)
    assert reading.diagnostics["target_mapping"] == TARGET_MAPPING == "pico_anchored_v1"


def test_a_hand_rotation_turns_the_follower_in_place_about_the_same_base_axis():
    source, part, clock = _source()
    start = Rotation.from_rotvec([0.4, 0.1, -0.2])
    part.pose = np.array([0.0, 0.0, 0.0, *start.as_quat()])
    _take_over(source, part, clock)
    part.pose[3:] = (Rotation.from_rotvec([0.0, 0.0, 0.02]) * start).as_quat()  # base z, within 0.03 rad
    reading = _poll(source, clock)
    np.testing.assert_allclose(reading.action[:6], [0.0, 0.0, 0.0, 0.0, 0.0, 0.02 / SCALE_RAD], atol=1e-6)
    np.testing.assert_allclose(source.cartesian_tracker.command[:3], HOME_TCP[:3], atol=1e-9)


def test_input_rotation_puts_the_hand_in_the_base_frame():
    base_R_input = [[0, -1, 0], [1, 0, 0], [0, 0, 1]]  # Pico +x is base +y
    source, part, clock = _source(input_rotation=base_R_input)
    _take_over(source, part, clock)
    part.pose[:3] = [0.004, 0.0, 0.0]
    np.testing.assert_allclose(_poll(source, clock).action[:6], [0.0, 0.004 / SCALE_M, 0, 0, 0, 0], atol=1e-6)
    with pytest.raises(ValueError, match="input_rotation"):
        _source(input_rotation=[[2, 0, 0], [0, 1, 0], [0, 0, 1]])


def test_the_tracker_limits_a_hand_jump_to_the_target_speed():
    source, part, clock = _source()
    _take_over(source, part, clock)
    part.pose[:3] = [0.1, 0.0, 0.0]
    reading = _poll(source, clock)
    np.testing.assert_allclose(reading.action[0], 0.05 * STEP_S / SCALE_M, atol=1e-6)  # 0.005 m per step


def test_a_second_bind_press_hands_back_and_holds():
    source, part, clock = _source()
    _take_over(source, part, clock)
    part.pose[:3] = [0.004, 0.0, 0.0]
    assert _poll(source, clock).intervened
    part.press("bind")
    reading = _poll(source, clock)
    assert not reading.intervened and not source.cartesian_tracker.following
    part.pose[:3] = [0.02, 0.0, 0.0]
    assert not _poll(source, clock).intervened  # the hand moved while handed back
    _take_over(source, part, clock)  # re-anchored where the hand is now: no jump
    np.testing.assert_allclose(_poll(source, clock).action[:6], 0.0, atol=1e-9)


def test_a_request_outside_the_workspace_holds_until_the_next_bind_press(caplog):
    env = CartesianPoseEnv(safety_box=[0.4, -0.1, 0.3, 0.502, 0.1, 0.5])
    source, part, clock = _source(env)
    _take_over(source, part, clock)
    part.pose[:3] = [0.004, 0.0, 0.0]  # requests x = 0.504
    with caplog.at_level(logging.WARNING):
        reading = _poll(source, clock)
    assert not reading.intervened and not source.cartesian_tracker.following
    assert "WORKSPACE HOLD" in caplog.text
    part.pose[:3] = [0.0, 0.0, 0.0]  # back inside: still handed back
    assert not _poll(source, clock).intervened
    _take_over(source, part, clock)
    part.pose[:3] = [0.001, 0.0, 0.0]
    assert _poll(source, clock).intervened


def test_a_stalled_loop_holds_and_re_anchors(caplog):
    source, part, clock = _source()
    _take_over(source, part, clock)
    clock.now += 1.0  # more than max_dt_s
    part.pose[:3] = [0.004, 0.0, 0.0]
    with caplog.at_level(logging.WARNING):
        _poll(source, clock)
    assert "control loop stalled" in caplog.text and not source.cartesian_tracker.following
    reading = _poll(source, clock)  # re-anchored at the current hand pose
    assert reading.intervened
    np.testing.assert_allclose(reading.action[:6], 0.0, atol=1e-9)


def test_the_episode_end_press_ends_the_episode_and_hands_back():
    source, part, clock = _source()
    _take_over(source, part, clock)
    part.press("episode_end")
    reading = _poll(source, clock)
    assert reading.episode_end and not source.cartesian_tracker.following
    assert not _poll(source, clock).intervened


def test_outcome_events_are_forwarded():
    source, part, clock = _source()
    part.press("success")
    assert _poll(source, clock).events == {"success": True}


def test_the_trigger_is_binarized():
    source, part, clock = _source()
    _take_over(source, part, clock)
    part.width = 0.3  # pulled past half travel: close
    assert _poll(source, clock).action[6] == -1.0
    part.width = 0.8
    assert _poll(source, clock).action[6] == 1.0
    continuous, part, clock = _source(gripper_binarize_threshold=None)
    part.width = 0.75
    assert continuous.poll().action[6] == pytest.approx(0.5)


def test_a_take_over_keeps_the_gripper_until_the_trigger_agrees():
    held = [-1.0]  # the gripper is closed on an object
    source, part, clock = _source(last_gripper_action_getter=lambda: held[0])
    part.width = 1.0  # trigger released (open) at the take-over
    assert _take_over(source, part, clock).action[6] == -1.0
    assert _poll(source, clock).action[6] == -1.0
    part.width = 0.0  # the operator closes the trigger: now it agrees
    assert _poll(source, clock).action[6] == -1.0
    part.width = 1.0  # and opens it: followed
    assert _poll(source, clock).action[6] == 1.0


def test_the_event_device_toggle_is_ignored_with_a_warning(caplog):
    source, part, clock = _source()
    with caplog.at_level(logging.WARNING):
        reading = source.poll_with_toggle(True)
    assert not reading.intervened and "bind" in caplog.text
    part.press("bind")
    assert source.poll_with_toggle(True).intervened


def test_a_read_failure_propagates_and_hands_back():
    source, part, clock = _source()
    _take_over(source, part, clock)
    part.failure = DeviceFault("teleop 'pico' produced no sample within 1.00s", device="pico")
    with pytest.raises(DeviceFault):
        _poll(source, clock)
    assert not source.cartesian_tracker.following
    part.failure = None
    assert not _poll(source, clock).intervened


def test_a_sample_without_a_hand_pose_is_a_device_fault():
    source, part, clock = _source()
    part.carries_pose = False
    with pytest.raises(DeviceFault, match="hand_pose"):
        _poll(source, clock)


def test_reset_hands_back_and_resets_the_part():
    source, part, clock = _source()
    _take_over(source, part, clock)
    source.reset()
    assert part.reset_calls == 2 and not source.cartesian_tracker.following
    assert not _poll(source, clock).intervened


def test_the_source_builds_and_owns_a_local_pico_part(zmq_sdk):
    source = make_teleop_source("pico", CartesianPoseEnv(), address="tcp://pico:7777", hand="left", hold_s=0.2)
    assert not zmq_sdk.sockets  # nothing opened at construction
    source.start()
    try:
        assert zmq_sdk.sockets[0].address == "tcp://pico:7777"
    finally:
        source.close()
    assert zmq_sdk.sockets[0].closed


def test_the_source_owns_a_robot_serving_the_pico():
    part = _PicoPart()

    class _Robot:
        teleop = None
        connected = False

        def connect(self):
            self.connected, self.teleop = True, part

        def disconnect(self):
            self.connected, self.teleop = False, None

    robot = _Robot()
    source = PicoTeleopSource(teleop_robot=robot, follower_pose_getter=lambda: HOME_TCP.copy())
    assert source.teleop_robot is robot and not robot.connected
    source.start()
    assert robot.connected
    part.press("bind")
    assert source.poll().intervened
    source.close()
    assert not robot.connected


def test_robot_teleop_reads_the_pico_served_as_the_env_robots_teleop_part():
    part = _PicoPart()

    class _RobotEnv(CartesianPoseEnv):
        def __init__(self):
            super().__init__()
            self.robot = type("_Robot", (), {"teleop": None})()  # connected later, by the env

    env = _RobotEnv()
    source = make_teleop_source("pico", env, robot_teleop=True)
    env.reset()
    source.reset()  # the robot is not connected yet: nothing to reset
    env.robot.teleop = part
    source.start()  # the robot owns the part: nothing to connect
    part.press("bind")
    assert source.poll().intervened
    with pytest.raises(ValueError, match="robot_teleop"):
        make_teleop_source("pico", CartesianPoseEnv(), robot_teleop=True)
    with pytest.raises(ValueError, match="not both"):
        make_teleop_source("pico", env, robot_teleop=True, remote_address="robot:50051")


def test_a_part_of_another_shape_is_rejected():
    class _JointPart(_PicoPart):
        action_space, action_dim = "joint_abs", 7

    with pytest.raises(ValueError, match="does not match"):
        PicoTeleopSource(_JointPart(), follower_pose_getter=lambda: HOME_TCP.copy())


def test_building_the_source_imports_no_zmq():
    code = (
        "import sys\n"
        "from real_world_rl.teleop import make_teleop_source\n"
        "from real_world_rl.teleop.pico.source import PicoTeleopSource\n"
        "PicoTeleopSource(follower_pose_getter=lambda: None)\n"
        "assert 'zmq' not in sys.modules, 'zmq imported'\n"
    )
    result = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
