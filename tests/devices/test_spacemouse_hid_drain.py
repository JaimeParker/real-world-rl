"""The HID reader drains every queued report per poll (as HIL-SERL's busy-looping
``SpaceMouseExpert`` stays current): latest axes win, presses are never lost, no lag builds up."""
from __future__ import annotations

import pytest

from real_world_rl.devices.teleop import TeleopConfig
from real_world_rl.teleop.spacemouse import backend as spacemouse_backend
from real_world_rl.teleop.spacemouse import hid as reader
from test_spacemouse_hid import Handle, isolated_site_tables, words  # noqa: F401 (fixture)

SPEC = reader.BUILTIN_DEVICE_SPECS["SpaceMouse Compact"]


def test_read_pending_keeps_the_latest_axes_and_consumes_the_whole_queue():
    handle = Handle([words(1, 35, 0, 0), words(2, 0, 70, 0), words(1, 350, -175, 70)])
    device = reader._HidReader(handle, SPEC)
    state, pressed = device.read_pending()
    assert state[:6] == pytest.approx((1.0, 0.5, -0.2, -0.2, 0.0, 0.0))  # x, y, z, roll, pitch, yaw
    assert pressed == [False, False]
    assert not handle.reports and device.report_count == 3 and device.last_drain_count == 3
    # Nothing queued: the state is kept, not reset, and nothing is consumed.
    again, pressed = device.read_pending()
    assert again is state and pressed == [False, False] and device.last_drain_count == 0


def test_read_pending_reports_a_press_released_within_the_same_poll():
    handle = Handle([[3, 1], [3, 0], [3, 2]])
    device = reader._HidReader(handle, SPEC)
    state, pressed = device.read_pending()
    assert state.buttons == [0, 1]  # the latest level
    assert pressed == [True, True]  # button 0 was pressed and released, button 1 is held
    handle.reports.extend([[3, 2]])
    assert device.read_pending()[1] == [False, False]  # a held button is not pressed again


def test_read_pending_is_bounded():
    handle = Handle([words(1, 35 * i, 0, 0) for i in range(1, 11)])
    device = reader._HidReader(handle, SPEC)
    device.read_pending(max_reports=4)
    assert device.last_drain_count == 4 and len(handle.reports) == 6


def test_backend_poll_is_not_behind_a_queue_and_latches_short_presses(monkeypatch):
    monkeypatch.setattr(spacemouse_backend, "_sleep", lambda seconds: None)
    teleop = spacemouse_backend.SpaceMouseTeleop(TeleopConfig(name="spacemouse", backend="spacemouse"))
    # A backlog of five reports, a button 0 tap among them, then the puck at rest.
    handle = Handle([words(1, 350, 0, 0), [3, 1], words(1, 175, 0, 0), [3, 0], words(1, 0, 0, 0)])
    device = reader._HidReader(handle, SPEC)
    reading = teleop._poll_device(device)
    assert list(reading.action) == pytest.approx([0.0] * 6)  # the latest report, not the oldest
    assert reading.events_now == {"button_0": True, "button_1": False}
    assert reading.buttons == {"button_0": False, "button_1": False}
    assert reading.gripper == 0.0  # the tap toggled the robot-yaml gripper target
    assert reading.diagnostics["hid_reports_this_poll"] == 5
    assert not handle.reports
    later = teleop._poll_device(device)
    assert later.events_now == {"button_0": False, "button_1": False}
