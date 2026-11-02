"""GELLO teleop backend against a fake ``dynamixel_sdk`` and a fake ``evdev`` (no hardware)."""
from __future__ import annotations

import collections
import sys
import threading
import time
import types
from pathlib import Path

import numpy as np
import pytest
import yaml

import robot_infra.devices.claims as claims
from robot_infra.devices import (
    DeviceFault,
    DeviceNotFound,
    Robot,
    SdkMissing,
    Teleop,
    TeleopConfig,
    TeleopSample,
)
from robot_infra.devices.backends.teleops import gello as gello_backend
from robot_infra.devices.backends.teleops.gello import (
    GelloTeleop,
    gripper_width_from_raw,
    joint_positions_from_raw,
    signed_int32,
    ticks_to_rad,
    wrap_to_center,
)
from robot_infra.devices.robots.franka import FrankaConfig, FrankaRobot
from robot_infra.devices.robots.piper import PiperConfig
from robot_infra.doctor import checks
from robot_infra.server import RemoteRobot
from robot_infra.server.remote import RemoteTeleop
from robot_infra.server.service import RobotService
from robot_infra.server.transport import LocalClient

DOCS = Path(__file__).resolve().parents[1] / "docs"
COMM_SUCCESS = 0
COMM_RX_TIMEOUT = -3001
TORQUE_ENABLE_ADDRESS = 64
PANDA_J4_CENTER = -np.pi / 2
PANDA_J6_CENTER = 1.8675
FRANKA_OFFSETS = [0.0, 0.0, np.pi, np.pi, np.pi, 1.5 * np.pi, 0.0]
FRANKA_SIGNS = [1, -1, 1, -1, 1, 1, 1]
FRANKA_CENTERS = [0.0, 0.0, 0.0, PANDA_J4_CENTER, 0.0, PANDA_J6_CENTER, 0.0]
GRIPPER_OPEN_RAD, GRIPPER_CLOSED_RAD = 3.22, 2.0
# Panda j4 and j6 outside [-pi, pi) of a zero centre; multi-turn and negative raw readings.
LEADER_Q = [0.3, -1.2, 2.5, -2.8, -0.4, 3.5, -2.9]
LEADER_TURNS = [0, -1, 2, -3, 1, 0, -2]
TICK_TOLERANCE_RAD = 2e-3  # one tick is pi / 2048 = 1.5e-3 rad


def franka_options(**overrides) -> dict:
    options = {
        "port": "/dev/serial/by-id/usb-FTDI_fake-if00-port0",
        "joint_ids": [1, 2, 3, 4, 5, 6, 7],
        "joint_offsets_rad": FRANKA_OFFSETS,
        "joint_signs": FRANKA_SIGNS,
        "wrap_centers_rad": FRANKA_CENTERS,
        "gripper_id": 8,
        "gripper_open_rad": GRIPPER_OPEN_RAD,
        "gripper_closed_rad": GRIPPER_CLOSED_RAD,
    }
    options.update(overrides)
    return {key: value for key, value in options.items() if value is not None}


NO_GRIPPER = {"gripper_id": None, "gripper_open_rad": None, "gripper_closed_rad": None}


def make_gello(hold_s: float = 0.0, **overrides) -> GelloTeleop:
    return GelloTeleop(TeleopConfig(name="gello", backend="gello", hold_s=hold_s, options=franka_options(**overrides)))


def raw_ticks(q: float, offset: float, sign: int, turns: int = 0) -> int:
    """Inverse of ``q = (raw - offset) * sign`` plus whole turns."""
    raw_rad = offset + sign * q + 2 * np.pi * turns
    return int(round(raw_rad / np.pi * 2048))


def gripper_ticks(raw_rad: float) -> int:
    return int(round(raw_rad / np.pi * 2048))


# -- fake dynamixel_sdk ----------------------------------------------------------


class FakeServoBus:
    """Shared state behind the fake SDK: servo ticks, scripted failures, recorded writes."""

    def __init__(self) -> None:
        self.ticks: dict[int, int] = {}
        self.silent_ids: set[int] = set()  # do not answer a ping
        self.unavailable_ids: set[int] = set()  # missing from the sync-read reply
        self.comm_result = COMM_SUCCESS
        self.open_result = True
        self.open_error: Exception | None = None
        self.baud_result = True
        self.writes: list[tuple[int, int, int]] = []
        self.ports: list = []
        self.protocol_versions: list[float] = []
        self.sync_reads: list = []

    def set_leader(self, q, offsets=FRANKA_OFFSETS, signs=FRANKA_SIGNS, turns=LEADER_TURNS, gripper_rad=None):
        for index, (value, offset, sign, turn) in enumerate(zip(q, offsets, signs, turns)):
            self.ticks[index + 1] = raw_ticks(value, offset, sign, turn)
        if gripper_rad is not None:
            self.ticks[len(q) + 1] = gripper_ticks(gripper_rad)

    def torque_writes(self) -> list[tuple[int, int]]:
        return [(servo_id, value) for servo_id, address, value in self.writes if address == TORQUE_ENABLE_ADDRESS]


def make_dynamixel_sdk(bus: FakeServoBus) -> dict[str, types.ModuleType]:
    robotis_def = types.ModuleType("dynamixel_sdk.robotis_def")
    robotis_def.COMM_SUCCESS = COMM_SUCCESS
    robotis_def.COMM_RX_TIMEOUT = COMM_RX_TIMEOUT

    class PortHandler:
        def __init__(self, port_name):
            self.port_name = port_name
            self.is_open = False
            self.baudrate = None
            bus.ports.append(self)

        def openPort(self):
            if bus.open_error is not None:
                raise bus.open_error
            self.is_open = bus.open_result
            return bus.open_result

        def setBaudRate(self, baudrate):
            self.baudrate = baudrate
            return bus.baud_result

        def closePort(self):
            self.is_open = False

    class PacketHandler:
        def __init__(self, protocol_version):
            bus.protocol_versions.append(protocol_version)

        def ping(self, port, servo_id):
            answers = port.is_open and servo_id in bus.ticks and servo_id not in bus.silent_ids
            return 1200, COMM_SUCCESS if answers else COMM_RX_TIMEOUT, 0

        def write1ByteTxRx(self, port, servo_id, address, value):
            assert port.is_open, "write on a closed port"
            bus.writes.append((servo_id, address, value))
            return COMM_SUCCESS, 0

        def getTxRxResult(self, result):
            return "[TxRxResult] There is no status packet!" if result == COMM_RX_TIMEOUT else f"result {result}"

    class GroupSyncRead:
        def __init__(self, port, packet_handler, start_address, data_length):
            self.port = port
            self.start_address = start_address
            self.data_length = data_length
            self.ids: list[int] = []
            bus.sync_reads.append(self)

        def addParam(self, servo_id):
            if servo_id in self.ids:
                return False
            self.ids.append(servo_id)
            return True

        def txRxPacket(self):
            time.sleep(0.001)
            return bus.comm_result

        def isAvailable(self, servo_id, address, length):
            inside = self.start_address <= address and address + length <= self.start_address + self.data_length
            return inside and servo_id in self.ids and servo_id not in bus.unavailable_ids

        def getData(self, servo_id, address, length):
            assert (address, length) == (132, 4)
            return bus.ticks[servo_id] & 0xFFFFFFFF  # the register as the SDK returns it: unsigned

    class GroupSyncWrite:
        def __init__(self, *args, **kwargs):
            pass

    submodules = {
        "port_handler": ("PortHandler", PortHandler),
        "packet_handler": ("PacketHandler", PacketHandler),
        "group_sync_read": ("GroupSyncRead", GroupSyncRead),
        "group_sync_write": ("GroupSyncWrite", GroupSyncWrite),
    }
    package = types.ModuleType("dynamixel_sdk")
    modules = {"dynamixel_sdk": package, "dynamixel_sdk.robotis_def": robotis_def}
    package.robotis_def = robotis_def
    package.COMM_SUCCESS = COMM_SUCCESS
    package.COMM_RX_TIMEOUT = COMM_RX_TIMEOUT
    for submodule_name, (attribute, value) in submodules.items():
        module = types.ModuleType(f"dynamixel_sdk.{submodule_name}")
        setattr(module, attribute, value)
        setattr(package, submodule_name, module)
        setattr(package, attribute, value)  # the real package star-imports its submodules
        modules[module.__name__] = module
    return modules


# -- fake evdev --------------------------------------------------------------------

EV_SYN, EV_KEY = 0, 1
KEY_CODES = {"KEY_ESC": 1, "KEY_R": 19, "KEY_S": 31, "KEY_F": 33, "KEY_A": 30, "KEY_SPACE": 57, "KEY_B": 48}
InputEvent = collections.namedtuple("InputEvent", "type code value")


def make_evdev(paths: set[str]) -> types.ModuleType:
    module = types.ModuleType("evdev")
    ecodes = types.ModuleType("evdev.ecodes")
    ecodes.EV_SYN, ecodes.EV_KEY = EV_SYN, EV_KEY
    for name, code in KEY_CODES.items():
        setattr(ecodes, name, code)
    module.ecodes = ecodes
    module.queue = collections.deque()
    module.devices = []

    class InputDevice:
        def __init__(self, path):
            if path not in paths:
                raise FileNotFoundError(2, "No such file or directory", path)
            self.path = path
            self.closed = False
            self.grabbed = False
            module.devices.append(self)

        def read_one(self):
            if self.closed:
                raise OSError("read from a closed input device")
            try:
                return module.queue.popleft()
            except IndexError:
                return None

        def grab(self):
            self.grabbed = True

        def close(self):
            self.closed = True

    def tap(key_name, hold=False):
        code = KEY_CODES[key_name]
        events = [InputEvent(EV_KEY, code, 1), InputEvent(EV_SYN, 0, 0)]
        if hold:
            events += [InputEvent(EV_KEY, code, 2)] * 3  # autorepeat is not a new press
        events += [InputEvent(EV_KEY, code, 0), InputEvent(EV_SYN, 0, 0)]
        module.queue.extend(events)

    module.InputDevice = InputDevice
    module.tap = tap
    return module


EVENT_DEVICE = "/dev/input/by-id/usb-fake-event-kbd"


@pytest.fixture
def servo_bus(monkeypatch):
    bus = FakeServoBus()
    bus.set_leader(LEADER_Q, gripper_rad=GRIPPER_OPEN_RAD)
    for name, module in make_dynamixel_sdk(bus).items():
        monkeypatch.setitem(sys.modules, name, module)
    return bus


@pytest.fixture
def evdev(monkeypatch):
    module = make_evdev({EVENT_DEVICE})
    monkeypatch.setitem(sys.modules, "evdev", module)
    return module


@pytest.fixture
def gello(servo_bus):
    created = []

    def build(hold_s=0.0, **overrides):
        teleop = make_gello(hold_s, **overrides)
        teleop.connect()
        created.append(teleop)
        return teleop

    yield build
    for teleop in created:
        teleop.disconnect()


def settled_read(teleop, predicate, attempts=300):
    """Read until the fake state has propagated through the poll thread."""
    for _ in range(attempts):
        sample = teleop.read()
        if predicate(sample):
            return sample
    raise AssertionError("fake state never reached the sample")


# -- contract and pure decoding ---------------------------------------------------


def test_registered_and_contract():
    assert Teleop.backend("gello") is GelloTeleop
    teleop = make_gello()
    assert teleop.action_space == "joint_abs" and teleop.action_dim == 7 and teleop.has_gripper
    assert teleop.describe()["action_dim"] == 7 and teleop.describe()["has_gripper"] is True
    assert teleop.servo_ids == [1, 2, 3, 4, 5, 6, 7, 8]
    without_gripper = make_gello(
        joint_ids=[1, 2, 3], joint_offsets_rad=[0, 0, 0], joint_signs=[1, 1, 1], wrap_centers_rad=None, **NO_GRIPPER
    )
    assert without_gripper.action_dim == 3 and without_gripper.has_gripper is False
    assert without_gripper.observation_features["action"]["shape"] == (3,)
    assert "gripper" not in without_gripper.observation_features
    assert GelloTeleop.HAS_GRIPPER is False and GelloTeleop.NUM_JOINTS is None  # class attributes untouched


def test_signed_int32_and_ticks_to_rad():
    assert signed_int32(5) == 5
    assert signed_int32(0xFFFFFFFF) == -1
    assert signed_int32(0x80000000) == -(2**31)
    assert signed_int32(0x7FFFFFFF) == 2**31 - 1
    assert signed_int32((-4096 * 3) & 0xFFFFFFFF) == -4096 * 3
    np.testing.assert_allclose(ticks_to_rad([2048, -1024, 4096 * 2]), [np.pi, -np.pi / 2, 4 * np.pi])


def test_wrap_to_center_half_open_interval():
    np.testing.assert_allclose(wrap_to_center(np.pi, 0.0), -np.pi)  # upper bound excluded
    np.testing.assert_allclose(wrap_to_center(-np.pi, 0.0), -np.pi)
    np.testing.assert_allclose(wrap_to_center(3.5, 0.0), 3.5 - 2 * np.pi)
    np.testing.assert_allclose(wrap_to_center(3.5, PANDA_J6_CENTER), 3.5)
    np.testing.assert_allclose(wrap_to_center(3.5 + 6 * np.pi, PANDA_J6_CENTER), 3.5)
    np.testing.assert_allclose(wrap_to_center(-2.8 - 4 * np.pi, PANDA_J4_CENTER), -2.8)


def test_joint_positions_apply_offset_then_sign_then_wrap():
    raw = [0.5 + 2 * np.pi, np.pi - 0.25, -3 * np.pi]
    q = joint_positions_from_raw(raw, [0.0, np.pi, 0.0], [1, -1, 1], [0.0, 0.0, 0.0])
    np.testing.assert_allclose(q, [0.5, 0.25, -np.pi])


@pytest.mark.parametrize(
    "open_rad, closed_rad", [(GRIPPER_OPEN_RAD, GRIPPER_CLOSED_RAD), (GRIPPER_CLOSED_RAD, GRIPPER_OPEN_RAD)]
)
def test_gripper_width_both_trigger_directions(open_rad, closed_rad):
    middle = (open_rad + closed_rad) / 2
    assert gripper_width_from_raw(open_rad, open_rad, closed_rad) == pytest.approx(1.0)
    assert gripper_width_from_raw(closed_rad, open_rad, closed_rad) == pytest.approx(0.0)
    assert gripper_width_from_raw(middle, open_rad, closed_rad) == pytest.approx(0.5)
    beyond_open = open_rad + (open_rad - closed_rad)
    beyond_closed = closed_rad - (open_rad - closed_rad)
    assert gripper_width_from_raw(beyond_open, open_rad, closed_rad) == 1.0
    assert gripper_width_from_raw(beyond_closed, open_rad, closed_rad) == 0.0
    quarter = closed_rad + 0.25 * (open_rad - closed_rad)
    assert gripper_width_from_raw(quarter, open_rad, closed_rad, binarize_threshold=0.5) == 0.0
    assert gripper_width_from_raw(middle + 0.01 * (open_rad - closed_rad), open_rad, closed_rad, 0.5) == 1.0


# -- options -------------------------------------------------------------------------


@pytest.mark.parametrize(
    "overrides, match",
    [
        ({"bogus": 1}, r"unknown gello option 'bogus'; known: \["),
        ({"port": None}, "'port' is required"),
        ({"joint_ids": []}, "at least one servo id"),
        ({"joint_ids": [1, 1, 2, 3, 4, 5, 6]}, "duplicate ids"),
        ({"joint_ids": [1, 2, 3, 4, 5, 6, 300]}, r"\[0, 252\]"),
        ({"joint_offsets_rad": [0.0] * 6}, "joint_offsets_rad needs 7 entries"),
        ({"joint_signs": [1, 1, 1, 1, 1, 1, 2]}, "1 or -1"),
        ({"wrap_centers_rad": [0.0]}, "wrap_centers_rad needs 7 entries"),
        ({"gripper_id": 3}, "also in joint_ids"),
        ({"gripper_open_rad": None}, r"\['gripper_open_rad'\] are required with gripper_id"),
        ({"gripper_closed_rad": GRIPPER_OPEN_RAD}, "must differ"),
        ({**NO_GRIPPER, "gripper_binarize_threshold": 0.5}, "need gripper_id"),
        ({"gripper_binarize_threshold": 1.5}, r"\[0, 1\]"),
        ({"intervention": "pedal"}, r"choose from \['always', 'toggle'\]"),
        ({"intervention": "toggle"}, "needs event_device"),
        ({"intervention": "toggle", "event_device": EVENT_DEVICE, "event_keys": {"toggle": None}}, "event_keys.toggle"),
        ({"event_keys": {"pause": "KEY_P"}}, r"unknown gello event_keys action 'pause'; known: \['toggle', "),
        ({"baudrate": 0}, "baudrate must be positive"),
        ({"poll_interval_s": -0.1}, "poll_interval_s must be >= 0"),
    ],
)
def test_invalid_options_rejected(overrides, match):
    with pytest.raises(ValueError, match=match):
        make_gello(**overrides)


def test_defaults():
    teleop = make_gello(wrap_centers_rad=None, **NO_GRIPPER)
    assert teleop._baudrate == 57600 and teleop._poll_interval_s == 0.0
    assert teleop._intervention == "always" and teleop._event_device_path is None
    np.testing.assert_array_equal(teleop._wrap_centers_rad, np.zeros(7))
    assert teleop._gripper_binarize_threshold is None
    assert teleop._event_keys == gello_backend.DEFAULT_EVENT_KEYS


# -- open / close ----------------------------------------------------------------------


def test_open_pings_and_turns_torque_off_close_turns_it_off_again(gello, servo_bus):
    teleop = gello(baudrate=1_000_000)
    port = servo_bus.ports[0]
    assert port.port_name == franka_options()["port"] and port.baudrate == 1_000_000 and port.is_open
    assert servo_bus.protocol_versions == [2.0]
    assert servo_bus.torque_writes() == [(servo_id, 0) for servo_id in range(1, 9)]
    sync_read = servo_bus.sync_reads[0]
    assert (sync_read.start_address, sync_read.data_length) == (128, 8) and sync_read.ids == list(range(1, 9))
    teleop.read()
    teleop.disconnect()
    assert servo_bus.torque_writes() == [(servo_id, 0) for servo_id in range(1, 9)] * 2
    assert not port.is_open
    assert not any(t.name == "teleop-gello" for t in threading.enumerate())


def test_port_open_failure_is_device_not_found(servo_bus):
    servo_bus.open_result = False
    with pytest.raises(DeviceNotFound) as info:
        make_gello().connect()
    assert "dialout" in info.value.hint and "/dev/serial/by-id" in info.value.hint
    servo_bus.open_result, servo_bus.open_error = True, OSError("[Errno 16] Device or resource busy")
    teleop = make_gello()
    with pytest.raises(DeviceNotFound, match="resource busy"):
        teleop.connect()
    assert not teleop.is_connected and servo_bus.writes == []


def test_baud_rate_failure_is_device_not_found_and_closes_the_port(servo_bus):
    servo_bus.baud_result = False
    with pytest.raises(DeviceNotFound, match="baud rate 57600") as info:
        make_gello().connect()
    assert "Dynamixel Wizard" in info.value.hint
    assert not servo_bus.ports[0].is_open


def test_ping_failure_names_missing_ids_and_closes_the_port(servo_bus):
    servo_bus.silent_ids = {3, 8}
    with pytest.raises(DeviceNotFound, match=r"ids \[3, 8\]"):
        make_gello().connect()
    assert not servo_bus.ports[0].is_open and servo_bus.writes == []


def test_reconnect_reopens_the_port(gello, servo_bus):
    teleop = gello()
    teleop.disconnect()
    teleop.connect()
    assert len(servo_bus.ports) == 2 and not servo_bus.ports[0].is_open and servo_bus.ports[1].is_open
    assert teleop.read().action.shape == (7,)


# -- decoding through the poll thread ---------------------------------------------------


def test_sample_decodes_offsets_signs_multi_turn_and_panda_centres(gello, servo_bus):
    assert any(ticks < 0 for ticks in servo_bus.ticks.values())  # two's complement exercised
    sample = gello().read()
    assert isinstance(sample, TeleopSample) and sample.action_space == "joint_abs"
    assert sample.action.dtype == np.float32 and sample.action.shape == (7,)
    np.testing.assert_allclose(sample.action, LEADER_Q, atol=TICK_TOLERANCE_RAD)
    assert sample.gripper == pytest.approx(1.0, abs=1e-3)
    assert sample.intervening is True and sample.buttons == {"toggle": False}


def test_zero_wrap_centre_folds_panda_j6(gello):
    sample = gello(wrap_centers_rad=[0.0] * 7).read()
    expected = list(LEADER_Q)
    expected[5] -= 2 * np.pi  # 3.5 -> -2.78: j6 reaches past pi, so it needs a centre > 0
    np.testing.assert_allclose(sample.action, expected, atol=TICK_TOLERANCE_RAD)


def test_leader_motion_is_tracked(gello, servo_bus):
    teleop = gello()
    moved = list(LEADER_Q)
    moved[0] = -0.7
    servo_bus.set_leader(moved, gripper_rad=(GRIPPER_OPEN_RAD + GRIPPER_CLOSED_RAD) / 2)
    sample = settled_read(teleop, lambda s: abs(s.action[0] + 0.7) < TICK_TOLERANCE_RAD)
    assert sample.gripper == pytest.approx(0.5, abs=1e-3)


def test_gripper_binarization_and_reversed_trigger(gello, servo_bus):
    servo_bus.set_leader(LEADER_Q, gripper_rad=GRIPPER_CLOSED_RAD + 0.3 * (GRIPPER_OPEN_RAD - GRIPPER_CLOSED_RAD))
    assert gello(gripper_binarize_threshold=0.5).read().gripper == 0.0
    reversed_trigger = gello(gripper_open_rad=GRIPPER_CLOSED_RAD, gripper_closed_rad=GRIPPER_OPEN_RAD)
    assert reversed_trigger.read().gripper == pytest.approx(0.7, abs=1e-3)


def test_no_gripper_channel(gello):
    sample = gello(gripper_id=None, gripper_open_rad=None, gripper_closed_rad=None).read()
    assert sample.gripper is None and sample.action.shape == (7,)


def test_poll_interval_sleeps_before_each_bus_read(gello, monkeypatch):
    sleeps = []
    monkeypatch.setattr(gello_backend, "_sleep", lambda seconds: sleeps.append(seconds))
    teleop = gello(poll_interval_s=0.02)
    for _ in range(3):
        teleop.read()
    teleop.disconnect()
    assert len(sleeps) >= 3 and set(sleeps) == {0.02}


# -- failures ---------------------------------------------------------------------------


def test_comm_failure_raises_recoverable_device_fault_never_stale_data(gello, servo_bus):
    teleop = gello()
    teleop.read()
    servo_bus.comm_result = COMM_RX_TIMEOUT
    with pytest.raises(DeviceFault) as info:
        for _ in range(100):
            teleop.read()
    assert info.value.recoverable and "no status packet" in info.value.message
    assert teleop.health().consecutive_failures >= GelloTeleop.MAX_CONSECUTIVE_FAILURES
    with pytest.raises(DeviceFault):
        teleop.read()
    servo_bus.comm_result = COMM_SUCCESS
    for _ in range(300):  # the failure count clears with the next successful poll
        try:
            sample = teleop.read()
            break
        except DeviceFault:
            time.sleep(0.005)
    else:
        raise AssertionError("the teleop never recovered")
    np.testing.assert_allclose(sample.action, LEADER_Q, atol=TICK_TOLERANCE_RAD)


def test_servo_missing_from_the_reply_fails_the_poll(gello, servo_bus):
    teleop = gello()
    teleop.read()
    servo_bus.unavailable_ids = {6}
    with pytest.raises(DeviceFault) as info:
        for _ in range(100):
            teleop.read()
    assert "ids [6] missing" in info.value.message


# -- intervention and evdev events -------------------------------------------------------


def test_always_mode_intervenes_on_every_poll(gello):
    teleop = gello()
    assert all(teleop.read().intervening for _ in range(5))


def test_toggle_mode_follows_the_toggle_key(gello, evdev):
    teleop = gello(intervention="toggle", event_device=EVENT_DEVICE)
    assert evdev.devices[0].path == EVENT_DEVICE and not evdev.devices[0].grabbed
    assert teleop.read().intervening is False
    evdev.tap("KEY_SPACE", hold=True)
    sample = settled_read(teleop, lambda s: s.intervening)
    assert sample.buttons == {"toggle": True}
    time.sleep(0.02)
    assert teleop.read().intervening is True  # sticky after release; autorepeat ignored
    evdev.tap("KEY_SPACE")
    assert settled_read(teleop, lambda s: not s.intervening).buttons == {"toggle": False}


def fired_events(teleop, expected, attempts=300):
    """Union of the events seen over reads until every expected one fired (each fires once)."""
    seen: list[str] = []
    for _ in range(attempts):
        sample = teleop.read()
        seen += [key for key, fired in sample.events.items() if fired]
        if set(expected) <= set(seen):
            time.sleep(0.02)  # let any stray event reach the next read
            sample = teleop.read()
            return seen + [key for key, fired in sample.events.items() if fired]
    raise AssertionError(f"events {expected} never fired; saw {seen}")


def test_event_keys_latch_once(gello, evdev):
    teleop = gello(event_device=EVENT_DEVICE)
    evdev.tap("KEY_S")
    evdev.tap("KEY_ESC")
    evdev.tap("KEY_A")  # unbound
    assert sorted(fired_events(teleop, ["success", "episode_end"])) == ["episode_end", "success"]
    sample = teleop.read()
    assert not any(sample.events.values())
    assert sample.intervening is True  # always mode: the toggle does not gate intervention


def test_event_keys_override_and_unbind(gello, evdev):
    teleop = gello(event_device=EVENT_DEVICE, event_keys={"success": "KEY_B", "failure": None})
    evdev.tap("KEY_S")  # no longer bound
    evdev.tap("KEY_F")  # unbound
    evdev.tap("KEY_B")
    assert fired_events(teleop, ["success"]) == ["success"]


def test_reset_clears_the_toggle(gello, evdev):
    teleop = gello(intervention="toggle", event_device=EVENT_DEVICE)
    evdev.tap("KEY_SPACE")
    settled_read(teleop, lambda s: s.intervening)
    teleop.reset()
    assert settled_read(teleop, lambda s: not s.intervening).buttons == {"toggle": False}


def test_disconnect_closes_the_input_device(gello, evdev, servo_bus):
    teleop = gello(event_device=EVENT_DEVICE)
    teleop.read()
    teleop.disconnect()
    assert evdev.devices[0].closed and not servo_bus.ports[0].is_open


def test_missing_evdev_is_sdk_missing_with_hint(servo_bus, monkeypatch):
    monkeypatch.setitem(sys.modules, "evdev", None)
    teleop = make_gello(event_device=EVENT_DEVICE)
    with pytest.raises(SdkMissing) as info:
        teleop.connect()
    assert "pip install evdev" in info.value.hint and "gello-events" in info.value.hint
    assert not teleop.is_connected and servo_bus.ports == []


def test_unopenable_input_device_is_device_not_found_and_closes_the_port(servo_bus, evdev):
    teleop = make_gello(event_device="/dev/input/event99")
    with pytest.raises(DeviceNotFound) as info:
        teleop.connect()
    assert "'input' group" in info.value.hint
    assert not servo_bus.ports[0].is_open
    assert servo_bus.torque_writes() == [(servo_id, 0) for servo_id in range(1, 9)] * 2


def test_unknown_evdev_key_name_is_rejected_at_connect(servo_bus, evdev):
    teleop = make_gello(event_device=EVENT_DEVICE, event_keys={"success": "KEY_NOPE"})
    with pytest.raises(ValueError, match="unknown evdev key name 'KEY_NOPE'"):
        teleop.connect()
    assert not servo_bus.ports[0].is_open and not teleop.is_connected


def test_one_key_bound_to_two_actions_is_rejected(servo_bus, evdev):
    teleop = make_gello(event_device=EVENT_DEVICE, event_keys={"success": "KEY_ESC"})
    with pytest.raises(ValueError, match="KEY_ESC"):
        teleop.connect()


# -- discovery and doctor -----------------------------------------------------------------


def test_discover_globs_ftdi_and_robotis_by_id(tmp_path, monkeypatch):
    for name in ("usb-FTDI_USB__-__Serial_Converter_A-if00-port0", "usb-ROBOTIS_OpenRB-150_B-if00", "usb-Other_C"):
        (tmp_path / name).touch()
    monkeypatch.setattr(
        gello_backend, "SERIAL_BY_ID_GLOBS", (str(tmp_path / "usb-FTDI_*"), str(tmp_path / "usb-ROBOTIS_*"))
    )
    assert GelloTeleop.discover() == {
        str(tmp_path / "usb-FTDI_USB__-__Serial_Converter_A-if00-port0"),
        str(tmp_path / "usb-ROBOTIS_OpenRB-150_B-if00"),
    }


def test_default_discovery_globs():
    assert set(gello_backend.SERIAL_BY_ID_GLOBS) == {
        "/dev/serial/by-id/usb-FTDI_*",
        "/dev/serial/by-id/usb-ROBOTIS_*",
    }


def run_serial(port):
    return checks._make_check(2, "os:serial", checks._serial_outcome(port))()


def test_serial_check_ok_missing_denied_and_unset(tmp_path, monkeypatch):
    port = tmp_path / "usb-FTDI_fake"
    missing = run_serial(str(port))
    assert not missing.ok and "not found" in missing.message and "--device" in missing.hint
    port.touch()
    assert run_serial(str(port)).ok
    monkeypatch.setattr(checks.os, "access", lambda path, mode: False)
    denied = run_serial(str(port))
    assert not denied.ok and "dialout" in denied.hint
    unset = run_serial(None)
    assert not unset.ok and "port" in unset.message


def test_os_serial_check_only_for_gello_backend():
    def names(backend, options):
        config = FrankaConfig(arm_backend="fake", teleop=TeleopConfig(backend=backend, options=options))
        return {name for name, _ in checks._os_checks(config)}

    assert "os:serial" in names("gello", franka_options())
    assert "os:serial" not in names("fake", {})
    assert "os:hid" not in names("gello", franka_options())


# -- daemon round trip ------------------------------------------------------------------


def test_daemon_round_trip_of_a_7d_sample(tmp_path, monkeypatch, servo_bus):
    monkeypatch.setattr(claims, "_DEFAULT_LOCK_DIR", tmp_path / "claims")
    robot = FrankaRobot(
        FrankaConfig(
            name="gello_station",
            arm_backend="fake",
            gripper=None,
            teleop=TeleopConfig(name="gello", backend="gello", options=franka_options()),
        )
    )
    service = RobotService(robot, watchdog_s=30.0)
    remote = RemoteRobot.from_client(LocalClient(service))
    try:
        remote.connect()
        teleop = remote.teleop
        assert isinstance(teleop, RemoteTeleop)
        assert teleop.action_space == "joint_abs" and teleop.action_dim == 7 and teleop.has_gripper is True
        described = robot.describe()["children"]["teleop"]
        assert described["action_dim"] == 7 and described["has_gripper"] is True
        sample = teleop.read(timeout_s=1.0)
        assert isinstance(sample.action, np.ndarray) and sample.action.dtype == np.float32
        assert sample.action.shape == (7,)
        np.testing.assert_allclose(sample.action, LEADER_Q, atol=TICK_TOLERANCE_RAD)
        assert sample.gripper == pytest.approx(1.0, abs=1e-3) and sample.intervening is True
        assert remote.get_observation()["teleop"]["action"].shape == (7,)
    finally:
        remote.disconnect()
        service.close()
    assert not servo_bus.ports[0].is_open
    assert servo_bus.torque_writes()[-8:] == [(servo_id, 0) for servo_id in range(1, 9)]


# -- documented example blocks ----------------------------------------------------------


def commented_gello_block(path: Path) -> dict:
    lines = path.read_text().splitlines()
    start = next(index for index, line in enumerate(lines) if line.startswith("# GELLO"))
    uncommented = [line[2:] for line in lines[start:] if line.startswith("# ")]
    teleop_start = next(index for index, line in enumerate(uncommented) if line.startswith("teleop:"))
    return yaml.safe_load("\n".join(uncommented[teleop_start:]))["teleop"]


@pytest.mark.parametrize("name, config_cls", [("franka", FrankaConfig), ("piper", PiperConfig)])
def test_example_yaml_gello_block_lists_every_option_and_builds(name, config_cls):
    block = commented_gello_block(DOCS / f"robot.{name}.example.yaml")
    assert block["backend"] == "gello"
    assert set(block["options"]) == gello_backend._OPTION_KEYS
    robot = Robot.from_dict({"type": name, "arm_backend": "fake", "gripper": None, "teleop": block})
    assert isinstance(robot.teleop, GelloTeleop)
    assert robot.teleop.action_dim == config_cls.NUM_JOINTS and robot.teleop.has_gripper
