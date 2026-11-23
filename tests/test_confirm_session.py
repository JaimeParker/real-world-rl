"""``episode_end_behavior=confirm``: the operator-confirmed episode boundary (``ConfirmSession``)
in the recorder, the actor loop and on its own, over fake devices and scripted operator keys
(JSONL event files, one event per poll)."""
from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
import torch

import real_world_rl.devices.backends  # noqa: F401  (registers the fake backends)
import real_world_rl.devices.claims as claims
from real_world_rl.contracts.action import LEGACY_ACTION_CONTRACT
from real_world_rl.dataset import Dataset
from real_world_rl.demos.record import RecordDemosArgs, open_dataset, record_demos, hold_action_fn_for
from real_world_rl.devices import CameraConfig, GripperConfig
from real_world_rl.devices.robots.franka import FrankaConfig, FrankaRobot
from real_world_rl.envs.real_robot import RealRobotEnv, RealRobotEnvConfig
from real_world_rl.envs.real_robot.env import RealRobotVectorEnv
from real_world_rl.envs.wrappers.intervention import InterventionWrapper
from real_world_rl.envs.wrappers.teleop_event_reward import TeleopEventRewardWrapper
from real_world_rl.supervise import capture_heartbeats
from real_world_rl.teleop import TeleopReading
from real_world_rl.teleop.core import session as session_module
from real_world_rl.teleop.core.composite import CompositeTeleopSource, ToggleableTeleopSource
from real_world_rl.teleop.core.fake import ScriptedTeleopSource
from real_world_rl.teleop.core.operator_keys import OperatorKeySource
from real_world_rl.teleop.core.session import ConfirmSession, require_confirm_source
from real_world_rl.teleop.keyboard.jsonl import JsonlEventSource
from real_world_rl.teleop.keyboard.stdin import keys_from_bytes

HOME = [0.0] * 7
LIMITS = [[-2.0, 2.0]] * 7
OPEN, CLOSED = 1.0, -1.0
MAX_TICKS = 400


@pytest.fixture(autouse=True)
def _claims(tmp_path, monkeypatch):
    monkeypatch.setattr(claims, "_DEFAULT_LOCK_DIR", tmp_path / "claims")


@pytest.fixture
def phases(monkeypatch):
    """Every wait tick (no sleeping; a stuck wait fails), and every status line in order."""
    log = SimpleNamespace(ticks=0, lines=[], phase=None)

    def pace(self, start):
        log.ticks += 1
        if log.ticks > MAX_TICKS:
            raise AssertionError(f"the confirm session kept waiting; status lines: {log.lines}")

    original_enter = ConfirmSession._enter

    def enter(self, phase, reason, text):
        log.phase = phase
        original_enter(self, phase, reason, text)

    monkeypatch.setattr(ConfirmSession, "_pace", pace)
    monkeypatch.setattr(ConfirmSession, "_enter", enter)
    monkeypatch.setattr(session_module, "operator_status", log.lines.append)
    # The session's own lines (the intervention wrapper's INTERVENTION/EPISODE lines left out).
    log.names = lambda: [name for name in (line.split(":", 1)[0] for line in log.lines) if name in _SESSION_LINES]
    log.session_lines = lambda: [line for line in log.lines if line.split(":", 1)[0] in _SESSION_LINES]
    return log


_SESSION_LINES = {"POST_EPISODE", "GRIPPER_CHECK", "GRIPPER", "RESETTING", "READY", "ACTIVE", "STOPPING"}


def _events(tmp_path: Path, *names: str) -> Path:
    path = tmp_path / f"events{len(list(tmp_path.glob('events*.jsonl')))}.jsonl"
    path.write_text("".join(f'{{"event":"{name}"}}\n' for name in names))
    return path


def _robot(**overrides) -> FrankaRobot:
    return FrankaRobot(FrankaConfig(
        arm_backend="fake", gripper=GripperConfig(backend="fake"),
        cameras=[CameraConfig(name="wrist", backend="fake", width=32, height=24, fps=60)],
        max_delta_per_step={"joint_rad": 10.0}, **overrides))


class Rig:
    """A real ``RealRobotEnv`` over fake devices, wrapped like the recorder's HIL env:
    ``TeleopEventRewardWrapper(InterventionWrapper(vector env, composite(motion, jsonl)))``."""

    def __init__(self, tmp_path, keys, *, mode="cartesian_delta", motion=None, gripper=CLOSED, **config):
        self.robot = _robot()
        values = dict(robot={"type": "franka", "arm_backend": "fake"}, cameras=[], home_joints=HOME,
                      reset_settle_s=0.0, control_hz=1000.0, action_mode=mode)
        if mode == "joint_abs":
            values["joint_limits"] = LIMITS
        values.update(config)
        self.env = RealRobotEnv(RealRobotEnvConfig(**values), robot=self.robot)
        self.vector = RealRobotVectorEnv(self.env)
        arm_dim = 6 if mode == "cartesian_delta" else 7
        if motion is None:
            motion = ScriptedTeleopSource(
                [TeleopReading(action=np.append(np.zeros(arm_dim), gripper).astype(np.float32), intervened=True)],
                action_mode=mode, arm_action_dim=arm_dim)
        self.events = JsonlEventSource(path=str(_events(tmp_path, *keys)))
        self.source = CompositeTeleopSource(motion, self.events)
        self.wrapped = TeleopEventRewardWrapper(InterventionWrapper(self.vector, self.source))
        self.resets = 0
        self.steps_by_phase = []
        self.order = []
        reset, step = self.vector.reset, self.env.step

        def counted_reset(**kwargs):
            self.resets += 1
            return reset(**kwargs)

        self.vector.reset = counted_reset
        self.env.step = lambda action: (self.steps_by_phase.append(self.phase()), step(action))[1]
        self.phase = lambda: None
        arm, grip = self.robot.arm, self.robot.gripper
        move, open_ = arm.move_to_joints, grip.open
        arm.move_to_joints = lambda q, blocking=True: (self.order.append("home"), move(q, blocking=blocking))[1]
        grip.open = lambda: (self.order.append("open"), open_())[1]

    def record(self, tmp_path, log=None, **overrides):
        if log is not None:
            self.phase = lambda: log.phase
        args = RecordDemosArgs(**{"num_successes": 1, "control_hz": 1e6, **overrides})
        assert args.episode_end_behavior == "confirm"  # the recorder's default
        writer = open_dataset(self.wrapped, args, tmp_path / "demos", obs_spec={
            "state": {"shape": list(self.vector.single_observation_space["state"].shape), "dtype": "float32"}})
        episodes = record_demos(self.wrapped, args, hold_action_fn=hold_action_fn_for(self.wrapped),
                                writer=writer, on_episode=self.on_episode)
        return episodes, Dataset(tmp_path / "demos")

    on_episode = None

    def close(self):
        self.wrapped.close()


@pytest.fixture
def rigs():
    built = []
    yield built
    for rig in built:
        rig.close()


def _rig(rigs, *args, **kwargs) -> Rig:
    rig = Rig(*args, **kwargs)
    rigs.append(rig)
    return rig


# -- the recorder ----------------------------------------------------------------------------


def test_two_episode_cycle_records_only_active_steps(tmp_path, phases, rigs):
    rig = _rig(rigs, tmp_path, ["start", "takeover", "success", "next", "start", "takeover", "success", "next"])
    with capture_heartbeats() as received:
        episodes, dataset = rig.record(tmp_path, phases, num_successes=2)
        beats = received()
    assert [(episode.steps, episode.kept) for episode in episodes] == [(2, True), (2, True)]
    # Fully closed gripper (nothing inside): n resets at once, no gripper check.
    assert phases.names() == ["RESETTING", "READY", "ACTIVE", "POST_EPISODE", "RESETTING", "READY",
                              "ACTIVE", "POST_EPISODE", "STOPPING"]
    lines = phases.session_lines()
    assert "budget reached" in lines[7] and "budget reached" in lines[8]
    assert rig.resets == 2  # the budget-ending n does not reset
    assert rig.steps_by_phase == ["ACTIVE"] * 4  # no env.step in any wait
    infos = dataset.episodes()
    assert [info.status for info in infos] == ["kept", "kept"]
    assert [info.steps for info in infos] == [2, 2]
    assert beats.count(b".") >= 4  # the waits feed the supervisor heartbeat


def test_row_zero_is_the_fresh_observation_at_the_start_key(tmp_path, phases, rigs, monkeypatch):
    rig = _rig(rigs, tmp_path, ["start", "takeover", "success", "next"])
    frames = []
    observe = rig.env.session_observe

    def counted_observe():
        frames.append(rig.phase())
        return observe()

    monkeypatch.setattr(rig.env, "session_observe", counted_observe)
    rig.record(tmp_path, phases)
    assert frames == ["ACTIVE"]  # taken once, at the start key, after READY ended


def test_partly_closed_gripper_asks_and_enter_resets_home_before_open(tmp_path, phases, rigs):
    rig = _rig(rigs, tmp_path, ["start", "takeover", "success", "next", "enter",
                                "start", "takeover", "success", "next"])
    rig.on_episode = lambda episode: setattr(rig.robot.gripper, "_width", 0.4) if episode.index == 0 else None
    rig.order.clear()
    episodes, _ = rig.record(tmp_path, phases, num_successes=2)
    assert len(episodes) == 2
    assert phases.names()[3:6] == ["POST_EPISODE", "GRIPPER_CHECK", "RESETTING"]
    assert phases.session_lines()[4] == (
        "GRIPPER_CHECK: the gripper is partly closed (32.0 mm) and may hold an object. Enter = reset "
        "now (the gripper opens after the arm reaches home); o = open it here first. Any other key = "
        "back to POST_EPISODE.")
    # Both resets home first, then open (the first one opens a gripper the reset found open: cached).
    second_reset = rig.order[rig.order.index("home", 1):]
    assert second_reset[:2] == ["home", "open"]


def test_partly_closed_gripper_opens_in_place_then_n_resets(tmp_path, phases, rigs):
    rig = _rig(rigs, tmp_path, ["start", "takeover", "success", "next", "open", "next",
                                "start", "takeover", "success", "next"])
    rig.on_episode = lambda episode: setattr(rig.robot.gripper, "_width", 0.4) if episode.index == 0 else None
    episodes, _ = rig.record(tmp_path, phases, num_successes=2)
    assert len(episodes) == 2
    names = phases.names()
    assert names[3:7] == ["POST_EPISODE", "GRIPPER_CHECK", "GRIPPER", "POST_EPISODE"]
    assert names[7] == "RESETTING"  # now fully open: no second check
    # Reset 1 homes then opens; o opens in place; reset 2 homes (the gripper is already open).
    assert rig.order == ["home", "open", "open", "home"]
    assert rig.robot.gripper.width == 0.0  # closed again by the second episode


def test_another_key_cancels_the_gripper_check(tmp_path, phases, rigs):
    rig = _rig(rigs, tmp_path, ["start", "takeover", "success", "next", "success", "next", "quit"])
    rig.on_episode = lambda episode: setattr(rig.robot.gripper, "_width", 0.4)
    episodes, _ = rig.record(tmp_path, phases, num_successes=2)
    assert len(episodes) == 1 and rig.resets == 1
    assert phases.names()[3:] == ["POST_EPISODE", "GRIPPER_CHECK", "POST_EPISODE", "GRIPPER_CHECK", "STOPPING"]
    assert rig.robot.gripper.width == 0.4  # nothing commanded the gripper


def test_budget_reached_post_episode_then_n_ends_without_reset(tmp_path, phases, rigs):
    rig = _rig(rigs, tmp_path, ["start", "takeover", "success", "open", "close", "next"])
    episodes, _ = rig.record(tmp_path, phases, num_successes=1)
    assert len(episodes) == 1 and rig.resets == 1
    assert phases.names()[3:] == ["POST_EPISODE", "GRIPPER", "GRIPPER", "STOPPING"]
    assert phases.session_lines()[3].startswith("POST_EPISODE: episode budget reached")
    assert phases.session_lines()[-1] == "STOPPING: episode budget reached; the session ends without a reset."
    assert rig.robot.gripper.call_log[-2:] == ["open", "close"]  # o/c work in the budget POST_EPISODE


@pytest.mark.parametrize("keys,episodes_recorded,last_phase_before_stop", [
    (["quit"], 0, "READY"),
    (["start", "quit"], 1, "ACTIVE"),
    (["start", "takeover", "success", "quit"], 1, "POST_EPISODE"),
])
def test_q_stops_in_every_phase(tmp_path, phases, rigs, keys, episodes_recorded, last_phase_before_stop):
    rig = _rig(rigs, tmp_path, keys)
    episodes, dataset = rig.record(tmp_path, phases, num_successes=2)
    assert len(episodes) == episodes_recorded and rig.resets == 1
    assert phases.names()[-2:] == [last_phase_before_stop, "STOPPING"]
    assert phases.session_lines()[-1].startswith("STOPPING: operator quit")
    assert len(dataset.episodes()) == episodes_recorded  # a finished episode stays on disk
    if episodes_recorded and last_phase_before_stop == "ACTIVE":
        assert episodes[0].operator_quit


def test_q_in_the_gripper_check_stops(tmp_path, phases, rigs):
    rig = _rig(rigs, tmp_path, ["start", "takeover", "success", "next", "quit"])
    rig.on_episode = lambda episode: setattr(rig.robot.gripper, "_width", 0.4)
    episodes, _ = rig.record(tmp_path, phases, num_successes=2)
    assert len(episodes) == 1 and rig.resets == 1
    assert phases.names()[-2:] == ["GRIPPER_CHECK", "STOPPING"]


def test_post_episode_holds_the_last_accepted_cartesian_target(tmp_path, phases, rigs, monkeypatch):
    rig = _rig(rigs, tmp_path, ["start", "takeover", "success", "handback", "handback", "next"])
    held = []
    tick = rig.env.session_hold_tick
    monkeypatch.setattr(rig.env, "session_hold_tick", lambda target: (held.append((rig.phase(), target.copy())), tick(target))[1])
    accepted = []
    step = rig.env.step

    def spy_step(action):
        result = step(action)
        accepted.append(rig.robot.arm.last_cartesian_target)
        return result

    rig.env.step = spy_step
    rig.record(tmp_path, phases)
    post = [target for phase, target in held if phase == "POST_EPISODE"]
    assert len(post) == 2  # one tick per poll until n (two fillers)
    for target in post:
        np.testing.assert_array_equal(target, accepted[-1])


def test_no_event_device_is_refused_before_any_reset(tmp_path, rigs):
    robot = _robot()
    config = RealRobotEnvConfig(robot={"type": "franka", "arm_backend": "fake"}, cameras=[], home_joints=HOME,
                                reset_settle_s=0.0, control_hz=1000.0)
    wrapped = TeleopEventRewardWrapper(InterventionWrapper(
        RealRobotVectorEnv(RealRobotEnv(config, robot=robot)), ScriptedTeleopSource([TeleopReading.idle(7)])))
    rigs.append(SimpleNamespace(close=wrapped.close))
    args = RecordDemosArgs(num_successes=1, control_hz=1e6)
    writer = open_dataset(wrapped, args, tmp_path / "none", obs_spec={"state": {"shape": [1], "dtype": "float32"}})
    with pytest.raises(ValueError, match="--teleop-event-device keyboard\\|stdin\\|jsonl"):
        record_demos(wrapped, args, hold_action_fn=lambda _: torch.zeros(7), writer=writer)
    assert not robot.is_connected and robot.arm.call_log == []  # refused before the reset


class AligningLeader(ToggleableTeleopSource):
    """A joint_abs leader that starts 1.0 rad off the follower in joint 2 and aligns after
    ``align_after`` reads with the toggle released."""

    def __init__(self, align_after=3):
        self.released_reads = 0
        self.align_after = align_after

    action_mode = "joint_abs"
    arm_action_dim = 7
    has_gripper = True

    def _reading(self, intervened):
        q = np.zeros(7)
        if self.released_reads < self.align_after:
            q[1] = 1.0
        return TeleopReading(action=np.append(q / 2.0, OPEN).astype(np.float32), intervened=intervened)

    def poll(self):
        return self._reading(True)

    def poll_with_toggle(self, toggle):
        if not toggle:
            self.released_reads += 1
        return self._reading(bool(toggle))


def test_joint_leader_far_from_follower_locks_the_start_key_without_raising(tmp_path, phases, rigs, monkeypatch):
    monkeypatch.setattr(session_module, "_LOCK_STATUS_PERIOD_S", 0.0)  # every change is shown
    leader = AligningLeader(align_after=3)
    rig = _rig(rigs, tmp_path, ["start", "handback", "handback", "handback", "start", "takeover", "success", "next"],
               mode="joint_abs", motion=leader)
    episodes, _ = rig.record(tmp_path, phases)
    assert len(episodes) == 1
    lines = phases.session_lines()
    locked = [index for index, line in enumerate(lines) if "start locked" in line]
    assert locked and "arm j2 1.00 rad" in lines[locked[0]]
    aligned = [index for index, line in enumerate(lines) if line.startswith("READY: leader aligned")]
    active = [index for index, line in enumerate(lines) if line.startswith("ACTIVE:")]
    # The first SPACE was ignored: ACTIVE follows the alignment, not the first key.
    assert locked[0] < aligned[0] < active[0]


def test_joint_post_episode_holds_the_measured_joints(tmp_path, phases, rigs, monkeypatch):
    rig = _rig(rigs, tmp_path, ["start", "takeover", "success", "handback", "next"], mode="joint_abs", gripper=OPEN)
    held = []
    tick = rig.env.session_hold_tick
    monkeypatch.setattr(rig.env, "session_hold_tick", lambda target: (held.append(target.copy()), tick(target))[1])
    rig.record(tmp_path, phases)
    assert held
    log = rig.robot.arm.call_log
    first_servo = log.index("servo_joints")
    assert "stop" not in log[first_servo:]  # nothing stops the arm between the episode and its hold
    for target in held:
        assert target.shape == (7,) and np.all(np.isfinite(target))


# -- the session on its own --------------------------------------------------------------------


class Backend:
    """A hold backend with scripted gripper widths; records ticks and gripper commands."""

    def __init__(self, grippers):
        self.grippers = list(grippers)
        self.ticks = []
        self.commands = []
        self.events = []

    def require_session_hold(self, mode="hold"):
        assert mode == "confirm"

    def session_hold_target(self):
        return np.ones(7)

    def session_start_target(self):
        return np.zeros(7)

    def session_hold_tick(self, target):
        self.ticks.append(target.copy())

    def session_grippers(self):
        return list(self.grippers)

    def session_gripper(self, open, arms=None):
        selected = [arm for arm, _, _ in self.grippers if arms is None or arm in arms]
        self.commands.append(("open" if open else "close", selected))
        if open:
            self.grippers = [(arm, 1.0 if arm in selected else width, m) for arm, width, m in self.grippers]
        return selected

    def session_event(self, phase, reason):
        self.events.append((phase, reason))


class Keys:
    """An event device returning one scripted list of key names per poll."""

    def __init__(self, *polls):
        self.polls = iter(polls)

    def provides_operator_keys(self):
        return True

    def poll_keys(self):
        return list(next(self.polls, []))


def _session(backend, *polls, lines=None):
    source = CompositeTeleopSource(SimpleNamespace(), Keys(*polls))
    return ConfirmSession(backend, source, control_hz=10., contract=dict(LEGACY_ACTION_CONTRACT),
                          sleep=lambda _: None, status=(lines if lines is not None else []).append)


@pytest.mark.parametrize("width", [1.0, 0.95, 0.0, 0.05])
def test_fully_open_or_fully_closed_resets_directly(width):
    backend = Backend([("arm", width, 0.08)])
    assert _session(backend, ["n"]).post_episode(allow_next=True) == "next"
    assert ("GRIPPER_CHECK", "partly_closed") not in backend.events


def test_no_width_feedback_asks():
    backend, lines = Backend([("arm", None, 0.08)]), []
    session = _session(backend, ["n"], ["enter"], lines=lines)
    assert session.post_episode(allow_next=True) == "next"
    assert "the gripper is partly closed (no width feedback)" in lines[1]


def test_bimanual_prompt_lists_every_partly_closed_arm_and_o_opens_only_them():
    backend, lines = Backend([("left", 0.4, 0.08), ("right", 1.0, 0.08), ("right2", None, 0.08)]), []
    session = _session(backend, ["n"], ["o"], ["n"], lines=lines)
    assert session.post_episode(allow_next=True) == "next"
    assert lines[1] == ("GRIPPER_CHECK: grippers left (32.0 mm), right2 (no width feedback) are partly closed "
                        "and may hold an object. Enter = reset now (the grippers open after the arms reach "
                        "home); o = open them here first. Any other key = back to POST_EPISODE.")
    assert backend.commands == [("open", ["left", "right2"])]


def test_o_and_c_are_ignored_outside_post_episode_and_gripper_check():
    backend = Backend([("arm", 0.4, 0.08)])
    assert _session(backend, ["o", "c"], ["space"]).ready() == "start"
    assert backend.commands == []


def test_ready_holds_the_post_reset_pose_and_post_episode_the_last_target():
    backend = Backend([])
    session = _session(backend, [], [], ["space"], [], ["n"])
    assert session.ready() == "start"
    assert session.post_episode(allow_next=True) == "next"
    ready_ticks, post_ticks = backend.ticks[:3], backend.ticks[3:]
    assert all(np.array_equal(t, np.zeros(7)) for t in ready_ticks)
    assert all(np.array_equal(t, np.ones(7)) for t in post_ticks)
    assert [phase for phase, _ in backend.events] == ["READY", "ACTIVE", "POST_EPISODE"]


def test_a_hold_fault_propagates():
    backend = Backend([])

    def unhealthy(target):
        raise RuntimeError("cannot hold an unhealthy arm")

    backend.session_hold_tick = unhealthy
    with pytest.raises(RuntimeError, match="unhealthy"):
        _session(backend, []).post_episode(allow_next=True)


def test_confirm_needs_an_event_device():
    with pytest.raises(ValueError, match="--teleop-event-device"):
        require_confirm_source(ScriptedTeleopSource([TeleopReading.idle(7)]))
    with pytest.raises(ValueError, match="--teleop-event-device"):
        require_confirm_source(CompositeTeleopSource(ScriptedTeleopSource([TeleopReading.idle(7)]),
                                                     ScriptedTeleopSource([TeleopReading.idle(7)])))
    keys = OperatorKeySource()
    require_confirm_source(CompositeTeleopSource(ScriptedTeleopSource([TeleopReading.idle(7)]), keys))
    assert keys._keys.confirm_keys


@pytest.mark.parametrize("options,name", [
    ({"gripper_open_fraction": 1.5}, "gripper_open_fraction"),
    ({"gripper_closed_fraction": -.1}, "gripper_closed_fraction"),
    ({"gripper_open_fraction": .3, "gripper_closed_fraction": .3}, "gripper_closed_fraction"),
])
def test_gripper_fraction_errors(options, name):
    from real_world_rl.demos.record import _validate_episode_limit
    from real_world_rl.loops.actor import ActorLoop
    with pytest.raises(ValueError, match=name):
        _validate_episode_limit(RecordDemosArgs(**options))
    with pytest.raises(ValueError, match=name):
        object.__new__(ActorLoop).configure_episode_session("confirm", **options)


# -- event devices ------------------------------------------------------------------------------


def test_keys_are_ordered_and_presses_taken_by_a_step_poll_are_not_repeated():
    source = OperatorKeySource()
    source.enable_confirm_keys()
    for key in ("o", "n", "space"):
        source._keys.press(key)
    assert source.poll_keys() == ["o", "n", "space"]
    source._keys.press("n")
    source.poll()  # an env step consumed it (ACTIVE)
    assert source.poll_keys() == []


def test_confirm_keys_do_not_change_the_ordinary_events():
    source = OperatorKeySource()
    source.enable_confirm_keys()
    for key in ("space", "o", "c", "enter"):
        source._keys.press(key)
    reading = source.poll()
    assert dict(reading.events) == {} and not reading.episode_end and not reading.intervened


def test_terminal_space_and_return_are_named():
    assert keys_from_bytes(b" \n\r\r\nx") == ["space", "enter", "enter", "enter", "x"]


def test_jsonl_names_the_confirm_keys(tmp_path):
    source = JsonlEventSource(path=str(_events(tmp_path, "start", "enter", "open", "close")))
    source.start()
    source.enable_confirm_keys()
    try:
        assert [source.poll_keys() for _ in range(5)] == [["space"], ["enter"], ["o"], ["c"], []]
    finally:
        source.close()


def test_pynput_space_and_enter(monkeypatch):
    import sys
    from _leader_teleop_fakes import make_pynput
    from real_world_rl.teleop.keyboard.pynput_source import KeyboardTeleopSource

    fake = make_pynput()
    monkeypatch.setitem(sys.modules, "pynput", fake)
    monkeypatch.setitem(sys.modules, "pynput.keyboard", fake.keyboard)
    source = KeyboardTeleopSource()
    source.start()
    source.enable_confirm_keys()
    try:
        listener = fake.keyboard.listeners[-1]
        for key in (fake.keyboard.Key.space, fake.keyboard.Key.enter, fake.keyboard.KeyCode("O")):
            listener.tap(key)
        assert source.poll_keys() == ["space", "enter", "o"]
    finally:
        source.close()


def test_gello_event_device_binds_the_confirm_keys_where_free():
    from real_world_rl.teleop.gello import backend

    assert backend.CONFIRM_EVENT_KEYS["start_episode"] == "KEY_SPACE"
    assert set(backend.CONFIRM_EVENT_KEYS) <= set(backend._KEY_ACTIONS)
    events = session_module._keys_from_events({"start_episode": True, "gripper_open": True, "success": True})
    assert events == ["space", "o", "other"]


# -- defaults and the actor loop --------------------------------------------------------------


def test_recording_defaults_to_confirm_and_training_to_auto():
    from real_world_rl.training.args import HilEnvArgs
    assert RecordDemosArgs().episode_end_behavior == "confirm"
    assert HilEnvArgs().episode_end_behavior == "auto"


def test_real_robot_env_reports_gripper_width_and_none_without_feedback(tmp_path, rigs, monkeypatch):
    rig = _rig(rigs, tmp_path, [])
    rig.vector.reset()
    rig.robot.gripper._width = 0.25
    assert rig.env.session_grippers() == [("arm", 0.25, 0.08)]
    from real_world_rl.devices.backends.grippers.fake import FakeGripper

    def no_feedback(self):
        raise RuntimeError("no feedback")

    monkeypatch.setattr(FakeGripper, "width", property(no_feedback))
    assert rig.env.session_grippers() == [("arm", None, 0.08)]


def test_bimanual_env_holds_and_commands_each_arm(tmp_path):
    from _bimanual_fakes import bimanual_config, fake_franka
    from real_world_rl.envs.bimanual import BimanualRealRobotEnv

    robots = {arm: fake_franka(arm) for arm in ("left", "right")}
    env = BimanualRealRobotEnv(bimanual_config(), robots=robots)
    try:
        env.require_session_hold(mode="confirm")
        env.reset()
        assert env.session_gripper(False, arms=["left"]) == ["left"]  # the operator closed it
        robots["left"].gripper._width = 0.4  # on an object
        assert [(arm, width) for arm, width, _ in env.session_grippers()] == [("left", 0.4), ("right", 1.0)]
        assert env.session_gripper(True, arms=["left"]) == ["left"]
        assert robots["left"].gripper.width == 1.0
        target = env.session_start_target()
        env.session_hold_tick(target)
        assert "servo_cartesian" in robots["right"].arm.call_log
        assert set(env.session_observe()["state"]) >= {"left_tcp_pose", "right_tcp_pose"}
    finally:
        env.close()


class _ZeroPolicy:
    def __init__(self, dim):
        self.dim = dim

    def eval(self):
        pass

    def predict(self, obs, deterministic=False):
        return torch.zeros(1, self.dim)


class _SyncClient:
    def __init__(self):
        self.pushed = []
        self.started = self.stopped = False

    def start(self):
        self.started = True

    def stop(self):
        self.stopped = True

    def push_transition(self, transition):
        self.pushed.append(transition)

    def latest_policy_params(self):
        return None


def test_actor_loop_confirm_cycle(tmp_path, phases, rigs):
    from real_world_rl.loops.actor import ActorLoop

    rig = _rig(rigs, tmp_path, ["start", "handback", "end", "next", "start", "handback", "quit"], max_episode_steps=50)
    rig.phase = lambda: phases.phase
    sync = _SyncClient()
    loop = ActorLoop(rig.wrapped, _ZeroPolicy(7), sync, control_hz=1e6)
    loop.configure_episode_session("confirm")
    loop.run()
    assert phases.names() == ["RESETTING", "READY", "ACTIVE", "POST_EPISODE", "RESETTING", "READY",
                              "ACTIVE", "STOPPING"]
    assert rig.resets == 2
    assert rig.steps_by_phase == ["ACTIVE"] * 4  # two steps per episode, none in the waits
    assert len(sync.pushed) == 4 and sync.started and sync.stopped


def test_evaluation_confirm_cycle_ends_in_post_episode_without_a_reset(tmp_path, phases, rigs):
    from real_world_rl.training.evaluation import evaluate_policy

    rig = _rig(rigs, tmp_path, ["start", "end", "next", "start", "end", "open", "next"], max_episode_steps=50)
    rig.phase = lambda: phases.phase
    summary = evaluate_policy(rig.wrapped, _ZeroPolicy(7), episodes=2, seed=1, control_hz=1e6,
                              episode_end_behavior="confirm")
    rigs.remove(rig)  # evaluate_policy closed the env
    assert summary["completed_episodes"] == 2 and summary["interrupted_episodes"] == 0
    assert phases.names() == ["RESETTING", "READY", "ACTIVE", "POST_EPISODE", "RESETTING", "READY", "ACTIVE",
                              "POST_EPISODE", "GRIPPER", "STOPPING"]
    assert rig.resets == 2 and rig.steps_by_phase == ["ACTIVE"] * 2
