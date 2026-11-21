"""A station Franka Hand behaves like HIL-SERL's franka_gripper_server: SERL grasp/open values and
non-blocking commands (latest wins, errors raised on the next call); fake ``franky``, no hardware."""
from __future__ import annotations

import sys
import threading
import time

import pytest

from real_world_rl.devices.backends.grippers.franky import (
    SERL_CLOSE_WIDTH_M,
    SERL_GRASP_FORCE_N,
    SERL_SPEED_M_S,
    FrankyGripper,
)
from real_world_rl.devices.errors import DeviceFault
from real_world_rl.devices.robot import GripperConfig
from real_world_rl.devices.robots.franka import FrankaConfig
from test_franky_backend import make_fake_franky


@pytest.fixture
def franky(monkeypatch):
    fake = make_fake_franky([])
    monkeypatch.setitem(sys.modules, "franky", fake)
    return fake


def _station_gripper(**fields) -> FrankyGripper:
    config = FrankaConfig(robot_ip="10.0.0.2", gripper=GripperConfig(backend="franky", **fields))
    gripper = FrankyGripper.from_config(config)
    gripper.connect()
    return gripper


@pytest.fixture
def gripper(franky):
    gripper = _station_gripper()
    yield gripper
    gripper.disconnect()


def _block_grasps(franky):
    """Make the fake hand's grasp block until the returned event is set."""
    release, started = threading.Event(), threading.Event()
    grasp = franky.Gripper.grasp

    def blocking_grasp(self, *args, **kwargs):
        started.set()
        assert release.wait(5.0)
        return grasp(self, *args, **kwargs)

    franky.Gripper.grasp = blocking_grasp
    return release, started


def test_a_station_gripper_without_the_fields_uses_serl_values(gripper):
    assert (gripper.speed, gripper.force, gripper.close_width_m, gripper.open_width_m) == (0.3, 130.0, 0.01, None)
    assert gripper.asynchronous
    gripper.close()
    assert gripper.wait_idle(5.0)
    gripper.open()
    assert gripper.wait_idle(5.0)
    assert gripper._handle.calls == [("grasp", 0.01, 0.3, 130.0, 1.0, 1.0), ("open", 0.3)]


def test_station_gripper_fields_override_the_serl_values(franky):
    gripper = _station_gripper(speed_m_s=0.1, grasp_force_n=40, close_width_m=0.0, open_width_m=0.07)
    try:
        gripper.close()
        assert gripper.wait_idle(5.0)
        gripper.open()
        assert gripper.wait_idle(5.0)
        assert gripper._handle.calls == [("grasp", 0.0, 0.1, 40.0, 1.0, 1.0), ("move", 0.07, 0.1)]
    finally:
        gripper.disconnect()


def test_commands_do_not_block_and_the_latest_queued_one_wins(gripper, franky):
    release, started = _block_grasps(franky)
    began = time.monotonic()
    gripper.close()
    assert started.wait(5.0)
    gripper.open()   # queued behind the running grasp...
    gripper.move(0.5)  # ...and replaced by this newer command
    assert time.monotonic() - began < 1.0
    assert gripper._handle.calls == []
    release.set()
    assert gripper.wait_idle(5.0)
    assert gripper._handle.calls == [("grasp", SERL_CLOSE_WIDTH_M, SERL_SPEED_M_S, SERL_GRASP_FORCE_N, 1.0, 1.0),
                                     ("move", pytest.approx(0.04), SERL_SPEED_M_S)]


def test_binary_send_action_toggles_without_blocking(gripper, franky):
    import numpy as np
    release, started = _block_grasps(franky)
    gripper.send_action({"gripper": np.array([0.0])})
    assert started.wait(5.0)
    gripper.send_action({"gripper": np.array([1.0])})
    release.set()
    assert gripper.wait_idle(5.0)
    assert [call[0] for call in gripper._handle.calls] == ["grasp", "open"]


def test_a_failed_command_raises_from_the_next_call_once(gripper, franky):
    franky.Gripper.grasp_behavior = "command_exception"
    gripper.close()  # returns; the failure happens on the worker
    assert gripper.wait_idle(5.0)
    with pytest.raises(DeviceFault, match="grasp command rejected"):
        gripper.open()
    gripper.open()  # reported once; the hand is usable again
    assert gripper.wait_idle(5.0)
    assert gripper._handle.calls[-1] == ("open", SERL_SPEED_M_S)


def test_a_failed_open_is_a_fault_on_the_next_width_read(gripper, franky):
    def fail(self, speed):
        raise franky.CommandException("open rejected")

    franky.Gripper.open = fail
    gripper.open()
    assert gripper.wait_idle(5.0)
    with pytest.raises(DeviceFault, match="open rejected"):
        gripper.width
    assert gripper.width == pytest.approx(0.5)


@pytest.mark.parametrize("behavior", ["fail", "gripper_exception"])
def test_closing_on_nothing_is_not_an_error(gripper, franky, behavior):
    franky.Gripper.grasp_behavior = behavior
    gripper.close()
    assert gripper.wait_idle(5.0)
    gripper.open()


def test_disconnect_finishes_the_running_command(franky):
    gripper = _station_gripper()
    release, started = _block_grasps(franky)
    gripper.close()
    assert started.wait(5.0)
    hand = gripper._handle
    threading.Timer(0.05, release.set).start()
    gripper.disconnect()
    assert [call[0] for call in hand.calls] == ["grasp"]


@pytest.mark.parametrize("fields, match", [
    ({"grasp_force_n": 131.0}, "grasp_force_n"),
    ({"grasp_force_n": 0.0}, "grasp_force_n"),
    ({"speed_m_s": 0.0}, "speed_m_s"),
    ({"speed_m_s": True}, "speed_m_s"),
    ({"close_width_m": 0.09}, "close_width_m"),
    ({"open_width_m": -0.01}, "open_width_m"),
    ({"open_width_m": float("nan")}, "open_width_m"),
])
def test_gripper_command_fields_are_validated(fields, match):
    with pytest.raises(ValueError, match=match):
        GripperConfig.from_dict({"backend": "franky", **fields})


def test_130_newtons_is_accepted():
    assert GripperConfig.from_dict({"backend": "franky", "grasp_force_n": 130}).grasp_force_n == 130


def test_the_setup_wizard_writes_a_new_franka_hand_with_serl_values(tmp_path, monkeypatch):
    import real_world_rl.devices.claims as claims
    from real_world_rl.discovery.autoconfig import STATION_CONFIG_ENV, load_station
    from test_setup_wizard import Operator, run

    monkeypatch.setattr(claims, "_DEFAULT_LOCK_DIR", tmp_path / "claims")
    monkeypatch.delenv(STATION_CONFIG_ENV, raising=False)
    monkeypatch.setenv("XDG_CONFIG_HOME", str(tmp_path / "xdg"))
    monkeypatch.chdir(tmp_path)
    out = tmp_path / "station.yaml"
    code, output = run(["--mock", "--output", out], Operator())
    assert code == 0, output
    gripper = load_station(out).select().config.gripper
    assert gripper.backend == "franky"
    assert (gripper.speed_m_s, gripper.grasp_force_n, gripper.close_width_m, gripper.open_width_m) == (
        SERL_SPEED_M_S, SERL_GRASP_FORCE_N, SERL_CLOSE_WIDTH_M, None)
    assert "grasp_force_n: 130" in out.read_text()


def test_rwrl_gripper_reports_the_width_after_an_asynchronous_command(gripper, franky):
    from types import SimpleNamespace

    from real_world_rl.cli.cmd_gripper import gripper_command

    release, started = _block_grasps(franky)
    threading.Timer(0.05, release.set).start()
    ctx = SimpleNamespace(confirm=lambda *args, **kwargs: None)
    gripper_command(SimpleNamespace(action="close"), SimpleNamespace(gripper=gripper), ctx)
    assert [call[0] for call in gripper._handle.calls] == ["grasp"]  # finished before the report
