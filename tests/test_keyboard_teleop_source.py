"""KeyboardTeleopSource against a mocked ``pynput`` module (no X session)."""
from __future__ import annotations

import sys

import numpy as np
import pytest
import torch
from _leader_teleop_fakes import make_pynput
from _teleop_fakes import ContractEnv, FakeEnv
from real_world_rl.devices import DeviceFault, SdkMissing

from real_world_rl.envs.wrappers.intervention import TeleopInterventionWrapper
from real_world_rl.teleop import make_teleop_source
from real_world_rl.teleop.keyboard import KeyboardTeleopSource


@pytest.fixture
def pynput(monkeypatch):
    fake = make_pynput()
    monkeypatch.setitem(sys.modules, "pynput", fake)
    return fake


@pytest.fixture
def started(pynput):
    source = KeyboardTeleopSource()
    source.start()
    yield source, pynput.keyboard.listeners[-1], pynput.keyboard
    source.close()


def test_declares_the_envs_action_space_and_emits_zeros():
    source = make_teleop_source("keyboard", ContractEnv("joint_delta", 7))
    assert isinstance(source, KeyboardTeleopSource)
    assert (source.action_mode, source.arm_action_dim, source.has_gripper) == ("joint_delta", 7, True)
    reading = source.poll()
    assert not reading.intervened and reading.action.shape == (8,) and not reading.action.any()

    legacy = make_teleop_source("keyboard", FakeEnv())  # no contract: the legacy cartesian convention
    assert (legacy.action_mode, legacy.arm_action_dim, legacy.has_gripper) == ("cartesian_delta", 6, True)


def test_building_does_not_import_pynput(monkeypatch):
    monkeypatch.delitem(sys.modules, "pynput", raising=False)
    make_teleop_source("keyboard", ContractEnv())
    assert "pynput" not in sys.modules


def test_t_takes_over_and_h_hands_back(started):
    source, listener, keyboard = started
    listener.press(keyboard.KeyCode("t"))
    assert source.poll().intervened
    listener.press(keyboard.KeyCode("t"))  # auto-repeat while held
    listener.release(keyboard.KeyCode("t"))
    assert source.poll().intervened  # a state, not a deadman
    listener.tap(keyboard.KeyCode("t"))
    assert source.poll().intervened  # pressing t again keeps it: no toggle
    listener.tap(keyboard.KeyCode("H"))
    assert not source.poll().intervened
    listener.tap(keyboard.KeyCode("h"))
    assert not source.poll().intervened


def test_space_no_longer_toggles(started):
    source, listener, keyboard = started
    listener.tap(keyboard.Key.space)
    reading = source.poll()
    assert not reading.intervened and not reading.episode_end and dict(reading.events) == {}


def test_q_ends_the_episode_and_asks_to_quit(started):
    source, listener, keyboard = started
    listener.tap(keyboard.KeyCode("q"))
    reading = source.poll()
    assert reading.episode_end and dict(reading.events) == {"operator_quit": True}
    reading = source.poll()
    assert not reading.episode_end and dict(reading.events) == {}


def test_esc_ends_the_episode_once(started):
    source, listener, keyboard = started
    listener.tap(keyboard.Key.esc)
    assert source.poll().episode_end
    assert not source.poll().episode_end


def test_s_f_r_raise_events_and_other_keys_are_ignored(started):
    source, listener, keyboard = started
    for key in (keyboard.KeyCode("s"), keyboard.KeyCode("F"), keyboard.KeyCode("r")):
        listener.tap(key)
    for key in (keyboard.KeyCode("x"), keyboard.KeyCode(None), keyboard.Key.shift):
        listener.tap(key)
    reading = source.poll()
    assert dict(reading.events) == {"success": True, "failure": True, "rerecord": True}
    assert not reading.intervened and not reading.episode_end
    assert dict(source.poll().events) == {}


def test_reset_clears_the_toggle_and_latched_presses(started):
    source, listener, keyboard = started
    for key in (keyboard.KeyCode("t"), keyboard.Key.esc, keyboard.KeyCode("s")):
        listener.tap(key)
    source.reset()
    reading = source.poll()
    assert not reading.intervened and not reading.episode_end and dict(reading.events) == {}


def test_start_is_idempotent_and_close_stops_the_listener(pynput):
    source = KeyboardTeleopSource()
    source.start()
    source.start()
    assert len(pynput.keyboard.listeners) == 1
    listener = pynput.keyboard.listeners[0]
    source.close()
    source.close()  # safe twice
    assert listener.stop_calls == 1 and not listener.is_alive()


def test_dead_listener_raises_instead_of_reading_idle(started):
    source, listener, _ = started
    listener.alive = False
    with pytest.raises(DeviceFault, match="listener"):
        source.poll()


def test_missing_pynput_only_fails_at_start(monkeypatch):
    monkeypatch.setitem(sys.modules, "pynput", None)  # makes `from pynput import keyboard` fail
    source = KeyboardTeleopSource()
    source.poll()  # not started: nothing to read yet, no SDK needed
    with pytest.raises(SdkMissing) as error:
        source.start()
    assert "pynput" in error.value.hint


def test_wrapper_starts_the_listener_and_ends_the_episode_on_esc(pynput):
    env = ContractEnv()
    wrapped = TeleopInterventionWrapper(env, device="keyboard")
    listener = pynput.keyboard.listeners[0]
    assert listener.is_alive()
    listener.tap(pynput.keyboard.KeyCode("t"))
    listener.tap(pynput.keyboard.Key.esc)
    _, _, _, truncated, info = wrapped.step(torch.full((1, 7), 0.3))
    assert info["intervened"] and info["human_episode_end"] and bool(truncated)
    np.testing.assert_array_equal(info["intervene_action"].numpy(), np.zeros((1, 7), dtype=np.float32))
    wrapped.close()
    assert not listener.is_alive() and env.close_calls == 1
