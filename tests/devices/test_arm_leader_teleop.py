"""Second-arm leaders against the fake arm/gripper; no hardware SDK is needed."""
from pathlib import Path

import numpy as np
import pytest
import yaml

import real_world_rl.devices.backends  # noqa: F401
from real_world_rl.devices import DeviceFault, DeviceNotConnected, Teleop, TeleopConfig
from real_world_rl.devices.backends.arms.fake import FakeArm
from real_world_rl.devices.backends.teleops.arm_leader import ArmLeaderTeleop
from real_world_rl.devices.robots.franka import FrankaConfig
from real_world_rl.kinematics import PandaKinematics
from real_world_rl.teleop.core.composite import CompositeTeleopSource
from real_world_rl.teleop.core.contract import TeleopReading
from real_world_rl.teleop.core.fake import ScriptedTeleopSource
from real_world_rl.teleop.core.part import TeleopContract, teleop_contract_of
from real_world_rl.teleop.leader.cartesian import LeaderArmCartesianSource


def _leader(**options):
    return Teleop.backend("arm_leader")(
        TeleopConfig(name="leader", backend="arm_leader",
                     options={"arm": {"type": "fake", "num_joints": 7, "alpha": 1.0}, **options})
    )


def test_registered_construction_and_default_contract():
    leader = _leader()
    assert isinstance(leader, ArmLeaderTeleop)
    assert isinstance(leader.arm, FakeArm)
    assert not leader.arm.is_connected
    assert leader.gripper is None
    assert teleop_contract_of(leader) == TeleopContract("joint_abs", 7, False)
    assert leader.discover() == set()
    assert leader.SDK is None  # the class-level doctor declaration cannot select an arm


@pytest.mark.parametrize("mode,active", [("always", True), ("event", False)])
def test_reading_maps_joint_positions_gripper_and_intervention(mode, active):
    leader = _leader(gripper=True, intervention=mode)
    q = np.array([0.1, -0.7, 0.2, -2.3, 0.0, 1.5, 0.7])
    try:
        leader.connect()
        leader.arm.move_to_joints(q)
        leader.gripper.move(0.25)
        # Direct raw poll checks the mapping before the consumer's event defaults are added.
        raw = leader._poll_device(leader.arm)
        np.testing.assert_allclose(raw.action, q)
        assert raw.gripper == pytest.approx(0.25)
        assert raw.buttons == {} and raw.events_now == {}
        assert raw.active is active
        for _ in range(100):
            sample = leader.read(1.0)
            if np.allclose(sample.action, q) and sample.gripper == pytest.approx(0.25):
                break
        np.testing.assert_allclose(sample.action, q)
        assert sample.gripper == pytest.approx(0.25)
        assert sample.intervening is active
    finally:
        leader.disconnect()
    assert not leader.arm.is_connected and not leader.gripper.is_connected
    assert not any(call.startswith(("servo", "enter:")) for call in leader.arm.call_log)


def test_default_intervention_is_active_without_a_gripper():
    leader = _leader()
    try:
        leader.connect()
        sample = leader.read(1.0)
        assert sample.intervening and sample.gripper is None
    finally:
        leader.disconnect()


def test_disconnected_arm_raises_typed_errors(monkeypatch):
    leader = _leader()
    poll = leader._poll_device

    def disconnected_poll(handle):
        handle.disconnect()
        return poll(handle)

    monkeypatch.setattr(leader, "_poll_device", disconnected_poll)
    try:
        leader.connect()
        leader.arm.disconnect()
        with pytest.raises(DeviceNotConnected):
            leader._poll_device(leader.arm)
        # The consumer converts repeated polling failures into a DeviceFault.
        with pytest.raises(DeviceFault, match="not connected"):
            leader.read(2.0)
    finally:
        leader.disconnect()


def test_gripper_connect_failure_releases_the_arm(monkeypatch):
    leader = _leader(gripper=True)

    def fail():
        raise DeviceFault("gripper unavailable")

    monkeypatch.setattr(leader.gripper, "connect", fail)
    with pytest.raises(DeviceFault, match="gripper unavailable"):
        leader.connect()
    assert not leader.is_connected and not leader.arm.is_connected


@pytest.mark.parametrize("options", [
    {"arm": {}}, {"arm": {"type": "missing"}}, {"gripper": "yes"},
    {"intervention": "toggle"}, {"unknown": True},
])
def test_invalid_options_are_rejected(options):
    with pytest.raises(ValueError):
        _leader(**options)


def test_event_composite_takes_over_with_panda_kinematics():
    leader = _leader(intervention="event")
    follower = np.concatenate(PandaKinematics().ee_pose(np.zeros(7)))
    source = LeaderArmCartesianSource(
        leader, follower_pose_getter=lambda: follower,
        teleop_contract=("joint_abs", 7, False),
        last_gripper_action_getter=lambda: 1.0,
        kinematics="panda",
    )
    events = ScriptedTeleopSource([
        TeleopReading(action=np.zeros(7), intervened=False),
        TeleopReading(action=np.zeros(7), intervened=True),
    ])
    composite = CompositeTeleopSource(source, events)
    try:
        leader.connect()
        composite.reset()
        assert not composite.poll().intervened
        reading = composite.poll()
        assert reading.intervened and source.cartesian_tracker.following
        np.testing.assert_allclose(reading.action[:6], 0.0, atol=1e-7)
        assert reading.action[6] == 1.0
    finally:
        composite.close()
        leader.disconnect()


def test_franka_example_keeps_the_real_arm_only_in_the_leader_part():
    path = Path(__file__).resolve().parents[2] / "examples/stations/leader.franka-as-leader.example.yaml"
    config = FrankaConfig.from_dict(yaml.safe_load(path.read_text()))
    assert config.arm_backend == "fake" and config.gripper is None and config.cameras == []
    assert config.teleop.backend == "arm_leader"
    leader = Teleop.backend(config.teleop.backend)(config.teleop)
    assert leader.arm.robot_ip == "172.16.0.2"
    assert not leader.arm.is_connected and leader.action_dim == 7
