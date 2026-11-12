"""TeleopSource contract: readings, the legacy adapter, the registry, the built-in sources."""
from __future__ import annotations

import subprocess
import sys
import types

import numpy as np
import pytest
from _teleop_fakes import ContractEnv, FakeEnv
from real_world_rl.teleop.pico.bridge import TeleOpSample

from real_world_rl.teleop import (
    LegacyTeleopAdapter,
    TeleopReading,
    TeleopSource,
    as_teleop_source,
    make_teleop_source,
    register_teleop_source,
    teleop_source_names,
)
from real_world_rl.teleop.core import registry
from real_world_rl.teleop.core.fake import ScriptedTeleopSource
from real_world_rl.teleop.core.part import TeleopPartSource


def _reading(value: float = 0.5, intervened: bool = True, dim: int = 7) -> TeleopReading:
    return TeleopReading(action=np.full(dim, value), intervened=intervened)


# -- TeleopReading -----------------------------------------------------------------------


def test_reading_coerces_action_to_float32_and_defaults_are_empty():
    reading = TeleopReading(action=[1, 2, 3], intervened=True)
    assert reading.action.dtype == np.float32 and reading.action.shape == (3,)
    assert reading.episode_end is False and dict(reading.events) == {} and reading.timestamp == 0.0


def test_reading_rejects_non_vector_action():
    with pytest.raises(ValueError, match="1-D"):
        TeleopReading(action=np.zeros((1, 7)), intervened=False)


def test_idle_reading_is_zero_and_not_intervening():
    reading = TeleopReading.idle(7)
    assert not reading.intervened and not reading.action.any() and reading.action.shape == (7,)


# -- legacy adapter -----------------------------------------------------------------------


class _LegacySource:
    def __init__(self, samples):
        self.samples = list(samples)
        self.reset_calls = 0
        self.close_calls = 0

    def poll(self):
        return self.samples.pop(0)

    def reset(self):
        self.reset_calls += 1

    def close(self):
        self.close_calls += 1


def test_duck_typed_source_is_adapted_with_legacy_defaults():
    legacy = _LegacySource([types.SimpleNamespace(intervened=True, action=np.arange(7), episode_end=True)])
    source = as_teleop_source(legacy)

    assert isinstance(source, LegacyTeleopAdapter) and isinstance(source, TeleopSource)
    assert (source.action_mode, source.arm_action_dim, source.has_gripper) == ("cartesian_delta", 6, True)
    reading = source.poll()
    assert reading.intervened and reading.episode_end
    np.testing.assert_array_equal(reading.action, np.arange(7, dtype=np.float32))
    source.reset()
    source.close()
    assert (legacy.reset_calls, legacy.close_calls) == (1, 1)


def test_adapter_reads_declared_action_space_and_tolerates_missing_methods():
    class _Bare:
        action_mode, arm_action_dim, has_gripper = "joint_delta", 6, False

        def poll(self):
            return types.SimpleNamespace(intervened=False, episode_end=False)

    source = as_teleop_source(_Bare())
    assert (source.action_mode, source.arm_action_dim, source.has_gripper) == ("joint_delta", 6, False)
    source.reset()
    source.close()  # neither exists on the legacy object
    reading = source.poll()
    assert not reading.intervened and reading.action.shape == (6,)  # idle sample without .action


def test_adapter_forwards_fired_events_only():
    sample = types.SimpleNamespace(
        intervened=True, action=np.zeros(7), episode_end=False, events={"success": True, "failure": False}
    )
    reading = as_teleop_source(_LegacySource([sample])).poll()
    assert dict(reading.events) == {"success": True}


def test_teleop_source_passes_through_as_teleop_source():
    source = ScriptedTeleopSource([_reading()])
    assert as_teleop_source(source) is source


def test_teleop_source_is_abstract():
    with pytest.raises(TypeError):
        TeleopSource()

    class _NoPoll(TeleopSource):
        action_mode, arm_action_dim, has_gripper = "cartesian_delta", 6, True

    with pytest.raises(TypeError):
        _NoPoll()


# -- registry -----------------------------------------------------------------------------


@pytest.fixture
def scratch_registry(monkeypatch):
    monkeypatch.setattr(registry, "_REGISTRY", dict(registry._REGISTRY))


def test_builtin_names_are_listed():
    assert {"pico", "spacemouse", "robot", "fake"} <= set(teleop_source_names())


def test_register_and_make_passes_env_and_kwargs_to_from_env(scratch_registry):
    @register_teleop_source("test_dummy")
    class _Dummy(ScriptedTeleopSource):
        @classmethod
        def from_env(cls, env, **kwargs):
            source = cls([_reading()])
            source.built_from = (env, kwargs)
            return source

    env = FakeEnv()
    source = make_teleop_source("test_dummy", env, gain=2)
    assert isinstance(source, _Dummy) and source.built_from == (env, {"gain": 2})
    assert "test_dummy" in teleop_source_names()


def test_default_from_env_calls_the_constructor_with_kwargs(scratch_registry):
    @register_teleop_source("test_ctor")
    class _Ctor(TeleopSource):
        action_mode, arm_action_dim, has_gripper = "cartesian_delta", 6, True

        def __init__(self, scale):
            self.scale = scale

        def poll(self):
            return _reading()

    assert make_teleop_source("test_ctor", FakeEnv(), scale=3).scale == 3


def test_duplicate_name_with_another_class_is_rejected(scratch_registry):
    register_teleop_source("test_dup")(ScriptedTeleopSource)
    register_teleop_source("test_dup")(ScriptedTeleopSource)  # same class: idempotent
    with pytest.raises(ValueError, match="already registered"):
        register_teleop_source("test_dup")(LegacyTeleopAdapter)


def test_only_teleop_sources_can_be_registered(scratch_registry):
    with pytest.raises(TypeError, match="TeleopSource"):
        register_teleop_source("test_bad")(dict)


def test_unknown_name_lists_the_available_ones():
    with pytest.raises(ValueError, match="nope") as error:
        make_teleop_source("nope", FakeEnv())
    for name in ("pico", "spacemouse", "robot", "fake"):
        assert name in str(error.value)


def test_importing_the_package_and_building_sources_keeps_sdks_lazy():
    code = (
        "import sys\n"
        "import real_world_rl.teleop as teleop\n"
        "teleop.teleop_source_names()\n"
        "import real_world_rl.teleop.pico.source\n"
        "source = teleop.make_teleop_source('spacemouse', None, follower_pose_getter=lambda: None,\n"
        "                                   last_gripper_action_getter=lambda: 1.0)\n"
        "assert 'pyspacemouse' not in sys.modules, 'pyspacemouse imported'\n"
        "assert 'zmq' not in sys.modules, 'zmq imported'\n"
    )
    result = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True)
    assert result.returncode == 0, result.stderr


# -- ScriptedTeleopSource -----------------------------------------------------------------


def test_scripted_source_returns_readings_in_order_and_repeats_the_last():
    readings = [_reading(0.1), _reading(0.2), _reading(0.3, intervened=False)]
    source = ScriptedTeleopSource(readings)
    polled = [source.poll() for _ in range(5)]
    assert all(a is b for a, b in zip(polled[:3], readings))
    assert polled[3] is readings[-1] and polled[4] is readings[-1]


def test_scripted_source_records_lifecycle_calls_and_declares_its_space():
    source = ScriptedTeleopSource(
        [_reading(dim=7)], action_mode="joint_abs", arm_action_dim=6, has_gripper=False
    )
    source.start()
    source.reset()
    source.reset()
    source.close()
    assert (source.start_calls, source.reset_calls, source.close_calls) == (1, 2, 1)
    assert (source.action_mode, source.arm_action_dim, source.has_gripper) == ("joint_abs", 6, False)


def test_scripted_source_needs_a_reading():
    with pytest.raises(ValueError, match="at least one"):
        ScriptedTeleopSource([])


def test_fake_device_follows_the_env_contract_and_never_intervenes():
    source = make_teleop_source("fake", ContractEnv("joint_delta", 6, gripper_in_action=True))
    assert (source.action_mode, source.arm_action_dim, source.has_gripper) == ("joint_delta", 6, True)
    reading = source.poll()
    assert not reading.intervened and reading.action.shape == (7,)

    legacy = make_teleop_source("fake", FakeEnv())  # no contract: the legacy cartesian convention
    assert (legacy.action_mode, legacy.arm_action_dim) == ("cartesian_delta", 6)


def test_fake_device_accepts_scripted_readings():
    readings = [_reading(0.7)]
    assert make_teleop_source("fake", FakeEnv(), readings=readings).poll() is readings[0]


# -- PicoTeleopSource ---------------------------------------------------------------------


class _FakePicoWrapper:
    instances = []

    def __init__(self, **kwargs):
        self.kwargs = kwargs
        self.reset_kwargs = []
        self.closed = False
        type(self).instances.append(self)

    def poll(self):
        return TeleOpSample(
            action=np.arange(7, dtype=np.float32),
            twist=np.zeros(6, dtype=np.float32),
            gripper=-1.0,
            bind_pressed=False,
            episode_end=True,
            intervened=True,
        )

    def reset(self, **kwargs):
        self.reset_kwargs.append(kwargs)

    def close(self):
        self.closed = True


def test_pico_source_wraps_the_twist_bridge(monkeypatch):
    monkeypatch.setattr("real_world_rl.teleop.pico.source.EETwistTeleOpWrapper", _FakePicoWrapper)
    _FakePicoWrapper.instances = []
    source = make_teleop_source("pico", FakeEnv(), zmq_url="tcp://h:1", hand="left")
    wrapper = _FakePicoWrapper.instances[0]

    assert wrapper.kwargs == {"device": "pico", "zmq_url": "tcp://h:1", "hand": "left"}
    assert (source.action_mode, source.arm_action_dim, source.has_gripper) == ("cartesian_delta", 6, True)
    reading = source.poll()
    assert reading.intervened and reading.episode_end
    np.testing.assert_array_equal(reading.action, np.arange(7, dtype=np.float32))

    source.reset()
    source.reset(episode_end_pressed=True)
    assert wrapper.reset_kwargs == [{}, {"episode_end_pressed": True}]
    source.close()
    assert wrapper.closed


def test_pico_source_tolerates_a_wrapper_without_reset_or_close(monkeypatch):
    class _PollOnlyWrapper:
        def __init__(self, **kwargs):
            pass

        def poll(self):
            return types.SimpleNamespace(intervened=False, episode_end=False)

    monkeypatch.setattr("real_world_rl.teleop.pico.source.EETwistTeleOpWrapper", _PollOnlyWrapper)
    source = make_teleop_source("pico", FakeEnv())
    source.reset()
    source.reset(episode_end_pressed=True)
    source.close()
    assert not source.poll().intervened


# -- TeleopPartSource ---------------------------------------------------------------------


class _FakePart:
    action_space, has_gripper, action_dim = "cartesian_delta", True, 6
    name = "part"

    def __init__(self, events):
        self.events = events
        self.disconnect_calls = 0

    def read(self, timeout_s=None):
        return types.SimpleNamespace(
            action=np.full(6, 0.25, dtype=np.float32),
            action_space="cartesian_delta",
            gripper=0.5,
            intervening=True,
            events={"episode_end": False, "success": False, "failure": False, "rerecord": False, **self.events},
            timestamp=3.0,
        )

    def reset(self):
        pass

    def disconnect(self):
        self.disconnect_calls += 1


def test_part_source_emits_readings_and_forwards_device_events():
    part = _FakePart({"success": True, "episode_end": True})
    source = TeleopPartSource(part, env_contract={"gripper_in_action": True})
    reading = source.poll()

    assert isinstance(source, TeleopSource) and isinstance(reading, TeleopReading)
    np.testing.assert_allclose(reading.action, [0.25] * 6 + [0.0])
    assert reading.intervened and reading.episode_end and reading.timestamp == 3.0
    assert dict(reading.events) == {"success": True}

    assert dict(TeleopPartSource(_FakePart({}), env_contract={"gripper_in_action": True}).poll().events) == {}


def test_part_source_close_leaves_the_robot_owned_part_alone():
    part = _FakePart({})
    source = TeleopPartSource(part, env_contract={"gripper_in_action": True})
    source.start()
    source.close()
    assert part.disconnect_calls == 0


def test_robot_device_needs_a_real_robot_env():
    with pytest.raises(ValueError, match="real_robot"):
        make_teleop_source("robot", FakeEnv())


# -- held gripper action (one entry per arm) -------------------------------------------------


@pytest.mark.parametrize("held", [-0.5, [-0.5], np.array([-0.5], dtype=np.float32)])
def test_held_gripper_action_reads_a_number_or_a_one_entry_vector(held):
    from real_world_rl.teleop.core.contract import held_gripper_action

    assert held_gripper_action(lambda: held) == -0.5


def test_held_gripper_action_reads_the_entry_of_its_arm():
    import torch

    from real_world_rl.teleop.core.contract import held_gripper_action

    getter = lambda: torch.tensor([1.0, -1.0])  # noqa: E731
    assert held_gripper_action(getter) == 1.0
    assert held_gripper_action(getter, arm_index=1) == -1.0
