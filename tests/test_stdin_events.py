"""StdinTeleopSource: the operator keys read from a file descriptor (a pipe here, no terminal)."""
from __future__ import annotations

import os
import threading

import numpy as np
import pytest
import torch
from _teleop_fakes import ContractEnv
from real_world_rl.devices import DeviceFault

from real_world_rl.envs.wrappers.intervention import TeleopInterventionWrapper
from real_world_rl.teleop import make_teleop_source
from real_world_rl.teleop.keyboard.stdin import StdinTeleopSource, keys_from_bytes


class _Pipe:
    """A pipe whose read end feeds the source; ``type(data)`` returns once the reader thread
    applied every key of ``data``."""

    def __init__(self):
        self.read_fd, self.write_fd = os.pipe()

    def type(self, source, data: bytes) -> None:
        expected = len(keys_from_bytes(data))
        applied = []
        done = threading.Event()
        original = source._keys.press

        def press(name):
            original(name)
            applied.append(name)
            if len(applied) == expected:
                done.set()

        source._keys.press = press
        try:
            os.write(self.write_fd, data)
            assert done.wait(timeout=2.0), f"the reader thread applied {applied} of {data!r}"
        finally:
            source._keys.press = original

    def close_writer(self) -> None:
        if self.write_fd is not None:
            os.close(self.write_fd)
            self.write_fd = None

    def close(self) -> None:
        self.close_writer()
        os.close(self.read_fd)


@pytest.fixture
def piped():
    pipe = _Pipe()
    source = StdinTeleopSource(fd=pipe.read_fd, poll_interval_s=0.005)
    source.start()
    yield source, pipe
    source.close()
    pipe.close()


def test_key_edges_latch_until_the_next_poll(piped):
    source, pipe = piped
    pipe.type(source, b"t")
    reading = source.poll()
    assert reading.intervened and not reading.episode_end and dict(reading.events) == {}
    assert source.poll().intervened  # a state: t keeps it until h
    pipe.type(source, b"sFr")
    reading = source.poll()
    assert dict(reading.events) == {"success": True, "failure": True, "rerecord": True}
    assert dict(source.poll().events) == {}
    pipe.type(source, b"h")
    assert not source.poll().intervened


def test_esc_ends_the_episode_once(piped):
    source, pipe = piped
    pipe.type(source, b"\x1b")
    assert source.poll().episode_end
    assert not source.poll().episode_end


def test_q_ends_the_episode_and_asks_to_quit(piped):
    source, pipe = piped
    pipe.type(source, b"q")
    reading = source.poll()
    assert reading.episode_end and dict(reading.events) == {"operator_quit": True}


def test_reset_drops_the_latched_presses_and_hands_back(piped):
    source, pipe = piped
    pipe.type(source, b"ts\x1b")
    source.reset()
    reading = source.poll()
    assert not reading.intervened and not reading.episode_end and dict(reading.events) == {}


def test_end_of_input_raises_instead_of_reading_idle(piped):
    source, pipe = piped
    pipe.close_writer()
    source._reader.join(timeout=2.0)
    with pytest.raises(DeviceFault, match="end of input"):
        source.poll()


def test_arrow_keys_are_not_read_as_esc():
    assert keys_from_bytes(b"\x1b[A\x1b[1;5Cq\x1bOPt") == ["q", "t"]
    assert keys_from_bytes(b"\x1b") == ["esc"]
    assert keys_from_bytes(b"T\x1bH") == ["t", "esc", "h"]


def test_declares_the_envs_action_space_and_builds_without_a_terminal():
    source = make_teleop_source("stdin", ContractEnv("joint_delta", 7))
    assert isinstance(source, StdinTeleopSource)
    assert (source.action_mode, source.arm_action_dim, source.has_gripper) == ("joint_delta", 7, True)
    reading = source.poll()  # not started: nothing typed yet
    assert not reading.intervened and reading.action.shape == (8,) and not reading.action.any()
    source.close()  # never started: a no-op


def test_close_stops_the_reader_and_is_safe_twice(piped):
    source, pipe = piped
    reader = source._reader
    source.close()
    source.close()
    assert not reader.is_alive()


def test_wrapper_takes_over_and_ends_the_episode_from_the_pipe():
    pipe = _Pipe()
    env = ContractEnv()
    wrapped = TeleopInterventionWrapper(env, device="stdin", fd=pipe.read_fd, poll_interval_s=0.005)
    try:
        pipe.type(wrapped.source, b"t\x1b")
        _, _, _, truncated, info = wrapped.step(torch.full((1, 7), 0.3))
        assert info["intervened"] and info["human_episode_end"] and bool(truncated)
        np.testing.assert_array_equal(info["intervene_action"].numpy(), np.zeros((1, 7), dtype=np.float32))
    finally:
        wrapped.close()
        pipe.close()
    assert env.close_calls == 1
