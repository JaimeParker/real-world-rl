"""GelloTeleopSource over a stand-in ``gello`` backend (a FakeTeleop subclass; no Dynamixel SDK):
device options, the env side filled by ``from_env``, the onset policy and the device lifecycle."""
from __future__ import annotations

import numpy as np
import pytest
import torch
from _leader_teleop_fakes import (
    JOINT_SCALE_RAD,
    FakeGelloTeleop,
    JointEnv,
    executed_joints,
)
from _teleop_fakes import ContractEnv
from real_world_rl.devices.teleop import Teleop

from real_world_rl.envs.wrappers.intervention import (
    InterventionWrapper,
    TeleopInterventionWrapper,
)
from real_world_rl.teleop import make_teleop_source
from real_world_rl.teleop.gello import GelloTeleopSource
from real_world_rl.teleop.leader_arm import TELEOP_SYNCING_EVENT

DEVICE = dict(
    port="/dev/serial/by-id/usb-FTDI_gello",
    joint_ids=[1, 2, 3, 4, 5, 6, 7],
    joint_offsets_rad=[0.0, 1.5708, 0.0, 3.1416, 0.0, 0.0, 0.0],
    joint_signs=[1, 1, -1, 1, 1, 1, 1],
    gripper_id=8,
    gripper_open_rad=3.0,
    gripper_closed_rad=2.2,
)
FAR = [0.3, -0.02, 0.0, 0.0, 0.0, 0.0, 0.12]


@pytest.fixture
def fake_gello(monkeypatch):
    class _Gello(FakeGelloTeleop):
        script = [{"action": FAR, "gripper": 1.0, "active": True}]
        instances = []

    monkeypatch.setitem(Teleop._BACKENDS, "gello", _Gello)
    yield _Gello
    for part in _Gello.instances:
        part.disconnect()


def test_device_options_reach_the_backend_and_unset_optionals_stay_out(fake_gello):
    env = JointEnv("joint_abs")
    GelloTeleopSource.from_env(env, hold_s=0.2, baudrate=1_000_000, intervention="toggle", **DEVICE)
    config = fake_gello.instances[0].gello_config
    assert (config.name, config.backend, config.hold_s) == ("gello", "gello", 0.2)
    assert config.options == {**DEVICE, "baudrate": 1_000_000, "intervention": "toggle"}


def test_registered_as_gello_and_built_from_the_env_contract(fake_gello):
    source = make_teleop_source("gello", JointEnv("joint_abs"), **DEVICE)
    assert isinstance(source, GelloTeleopSource)
    assert (source.action_mode, source.arm_action_dim, source.has_gripper) == ("joint_abs", 7, True)
    assert not fake_gello.instances[0].is_connected  # building opens nothing


def test_joint_abs_takeover_ramps_from_the_envs_follower(fake_gello):
    env = JointEnv("joint_abs")
    env.follower = np.full(7, 0.01)
    source = make_teleop_source("gello", env, ramp_step_rad=0.05, **DEVICE)
    source.start()
    reading = source.poll()
    np.testing.assert_allclose(
        executed_joints(reading.action[:7]), [0.06, -0.02, 0.0, 0.0, 0.0, 0.0, 0.06], atol=1e-6
    )
    assert reading.intervened and reading.events == {TELEOP_SYNCING_EVENT: True}
    assert reading.action[7] == pytest.approx(1.0)


def test_joint_delta_env_gets_leader_minus_follower_over_its_scale(fake_gello):
    env = JointEnv("joint_delta")
    env.follower = np.full(7, 0.1)
    source = make_teleop_source("gello", env, **DEVICE)
    source.start()
    reading = source.poll()
    expected = np.clip((np.array(FAR) - env.follower) / JOINT_SCALE_RAD, -1.0, 1.0)
    np.testing.assert_allclose(reading.action[:7], expected, atol=1e-6)


def test_arm_onset_is_forwarded(fake_gello):
    source = make_teleop_source("gello", JointEnv("joint_abs"), onset="arm", arm_tolerance_rad=0.1, **DEVICE)
    source.start()
    assert not source.poll().intervened  # the leader is 0.3 rad from the follower
    assert source.poll_with_toggle(False).intervened is False


def test_gripperless_gello_on_a_gripper_env_holds_the_last_sent_gripper_action(fake_gello):
    fake_gello.script = [{"action": FAR, "active": True}]  # no gripper channel
    device = {k: v for k, v in DEVICE.items() if not k.startswith("gripper")}
    with pytest.raises(ValueError, match="last_gripper_action_getter"):
        make_teleop_source("gello", JointEnv("joint_abs"), **device)
    source = make_teleop_source(
        "gello", JointEnv("joint_abs"), last_gripper_action_getter=lambda: -1.0, **device
    )
    source.start()
    assert source.poll().action[-1] == pytest.approx(-1.0)


def test_cartesian_env_is_rejected_before_the_device_opens(fake_gello):
    env = ContractEnv()
    source = make_teleop_source("gello", env, **DEVICE)
    with pytest.raises(ValueError, match="action_mode"):
        InterventionWrapper(env, source)
    assert not fake_gello.instances[0].is_connected


def test_wrapper_starts_resets_and_closes_the_device(fake_gello):
    env = JointEnv("joint_abs")
    wrapped = TeleopInterventionWrapper(env, device="gello", **DEVICE)
    part = fake_gello.instances[0]
    assert part.is_connected
    wrapped.reset()
    _, _, _, _, info = wrapped.step(torch.zeros(1, 8))
    assert info["intervened"] and info["teleop_events"] == {TELEOP_SYNCING_EVENT: True}

    wrapped.close()
    assert not part.is_connected and env.close_calls == 1
    wrapped.source.close()  # safe twice


def test_options_yaml_supplies_the_device_options_from_the_cli(fake_gello, tmp_path):
    import yaml

    from real_world_rl.training import env_building
    from real_world_rl.training.entrypoints.hil_serl import HilSerlArgs

    options = tmp_path / "gello.yaml"
    options.write_text(yaml.safe_dump({**DEVICE, "hold_s": 0.1, "intervention": "toggle"}), encoding="utf-8")
    args = HilSerlArgs(teleop_device="gello", teleop_options_yaml=str(options))
    wrapped = TeleopInterventionWrapper(JointEnv("joint_abs"), **env_building.teleop_wrapper_kwargs(args))
    try:
        config = fake_gello.instances[0].gello_config
        assert config.hold_s == 0.1
        assert config.options == {**DEVICE, "intervention": "toggle"}
        assert fake_gello.instances[0].is_connected
    finally:
        wrapped.close()
