"""SpaceMouseTeleopSource against a mocked ``pyspacemouse`` module (no hardware)."""
from __future__ import annotations

import sys

import numpy as np
import pytest
import torch
from _teleop_fakes import ContractEnv, FakeEnv, make_pyspacemouse
from robot_infra.devices import SdkMissing

from rl_garden_real_world.envs.wrappers.intervention import (
    InterventionWrapper,
    TeleopInterventionWrapper,
)
from rl_garden_real_world.teleop import make_teleop_source
from rl_garden_real_world.teleop.spacemouse import SpaceMouseTeleopSource

TWIST = (0.1, 0.2, 0.3, 0.4, 0.5, 0.6)  # x, y, z, roll, pitch, yaw
MAPPED_TWIST = [-0.2, 0.1, 0.3, -0.4, -0.5, -0.6]  # [-y, x, z, -roll, -pitch, -yaw]


@pytest.fixture
def pyspacemouse(monkeypatch):
    fake = make_pyspacemouse()
    monkeypatch.setitem(sys.modules, "pyspacemouse", fake)
    return fake


@pytest.fixture
def build_source():
    created = []

    def build(**kwargs):
        source = SpaceMouseTeleopSource(**kwargs)
        created.append(source)
        return source

    yield build
    for source in created:
        source.close()


def poll_until(source, predicate, attempts=300):
    """Polls until the mocked state has propagated through the device poll thread."""
    for _ in range(attempts):
        reading = source.poll()
        if predicate(reading):
            return reading
    raise AssertionError("the mocked device state never reached a reading")


def test_declares_the_cartesian_twist_plus_gripper_space(build_source):
    source = build_source()
    assert (source.action_mode, source.arm_action_dim, source.has_gripper) == ("cartesian_delta", 6, True)


def test_registered_as_spacemouse_and_built_from_the_env(pyspacemouse, build_source):
    source = make_teleop_source("spacemouse", ContractEnv(), device_index=3)
    try:
        assert isinstance(source, SpaceMouseTeleopSource)
        source.start()
        assert pyspacemouse.devices[0].kwargs == {"device_index": 3}
    finally:
        source.close()


def test_motion_is_an_intervention_with_the_mapped_twist_and_open_gripper(pyspacemouse, build_source):
    source = build_source()
    source.start()
    pyspacemouse.twist = TWIST

    reading = poll_until(source, lambda r: r.intervened and r.action[0] != 0)

    assert reading.action.dtype == np.float32 and reading.action.shape == (7,)
    np.testing.assert_allclose(reading.action, MAPPED_TWIST + [1.0], atol=1e-6)
    assert not reading.episode_end and dict(reading.events) == {}


def test_idle_puck_is_not_an_intervention(pyspacemouse, build_source):
    source = build_source()
    source.start()
    reading = source.poll()
    assert not reading.intervened and not reading.action[:6].any()


def test_invert_axes_and_deadzone_are_forwarded(pyspacemouse, build_source):
    source = build_source(invert_axes=[0, 5], deadzone=0.05)
    source.start()
    pyspacemouse.twist = (0.0, 0.0, 0.01, 0.0, 0.0, 0.0)
    below = poll_until(source, lambda r: r.action[2] != 0)
    assert not below.intervened  # under the deadzone: idle, action stays raw

    pyspacemouse.twist = TWIST
    reading = poll_until(source, lambda r: r.action[0] != 0)
    np.testing.assert_allclose(reading.action[:6], [0.2, 0.1, 0.3, -0.4, -0.5, 0.6], atol=1e-6)


def test_button_1_raises_episode_end(pyspacemouse, build_source):
    source = build_source()
    source.start()
    pyspacemouse.buttons = (0, 1)

    reading = poll_until(source, lambda r: r.episode_end)

    assert reading.intervened  # a held button counts as operating the device


def test_button_0_toggles_the_sticky_gripper_and_reset_reopens_it(pyspacemouse, build_source):
    source = build_source()
    source.start()
    pyspacemouse.buttons = (1, 0)
    poll_until(source, lambda r: r.action[-1] == -1.0)  # closed: -1
    pyspacemouse.buttons = (0, 0)
    assert poll_until(source, lambda r: not r.intervened).action[-1] == -1.0  # sticky after release

    source.reset()

    poll_until(source, lambda r: r.action[-1] == 1.0)


def test_close_disconnects_the_device(pyspacemouse, build_source):
    source = build_source()
    source.start()
    assert not pyspacemouse.devices[0].closed
    source.close()
    assert pyspacemouse.devices[0].closed
    source.close()  # safe twice


def test_missing_sdk_only_fails_at_start(monkeypatch, build_source):
    monkeypatch.setitem(sys.modules, "pyspacemouse", None)  # makes `import pyspacemouse` fail
    source = build_source()  # building needs no SDK
    source.reset()  # not connected: nothing to reset
    with pytest.raises(SdkMissing) as error:
        source.start()
    assert "pyspacemouse" in error.value.hint


def test_wrapper_starts_and_closes_the_device(pyspacemouse):
    env = FakeEnv()
    wrapped = TeleopInterventionWrapper(env, device="spacemouse", device_index=1)
    assert pyspacemouse.devices[0].kwargs == {"device_index": 1}
    pyspacemouse.twist = TWIST

    infos = []
    for _ in range(300):
        infos.append(wrapped.step(torch.zeros(1, 7))[4])
        if infos[-1]["intervened"]:
            break
    assert infos[-1]["intervened"] and infos[-1]["intervention_onset"]
    np.testing.assert_allclose(infos[-1]["intervene_action"].numpy()[0], MAPPED_TWIST + [1.0], atol=1e-6)

    wrapped.close()
    assert pyspacemouse.devices[0].closed and env.close_calls == 1


def test_env_with_a_joint_action_space_is_rejected_without_opening_the_device(pyspacemouse, build_source):
    source = build_source()
    with pytest.raises(ValueError, match="action_mode"):
        InterventionWrapper(ContractEnv("joint_delta", 6), source)
    assert pyspacemouse.devices == []
