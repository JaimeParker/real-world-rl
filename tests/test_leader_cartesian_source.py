"""LeaderArmCartesianSource: a joint_abs leader arm mapped through Panda FK to the cartesian twist
of franka_real / real_robot cartesian envs. Covers both mappings, saturation, the gripper, the
action-contract checks, and the leader served by a stub-robot daemon (RemoteRobot + LocalClient)."""
from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import pytest
import torch
import yaml
from _teleop_fakes import ContractEnv
from scipy.spatial.transform import Rotation

import robot_infra.devices.backends  # noqa: F401  (registers the fake backends)
import robot_infra.devices.claims as claims
from robot_infra.devices import DeviceFault
from robot_infra.devices.robots.franka import FrankaConfig, FrankaRobot
from robot_infra.devices.teleop import EVENT_KEYS, TeleopConfig, TeleopSample
from robot_infra.server import RemoteRobot
from robot_infra.server.service import RobotService
from robot_infra.server.transport import LocalClient

from rl_garden_real_world.envs.franka_real.bridge_client import FrankaBridgeClient
from rl_garden_real_world.envs.franka_real.config import FrankaRealEnvConfig
from rl_garden_real_world.envs.franka_real.env import make_franka_real_env
from rl_garden_real_world.envs.wrappers.intervention import (
    InterventionWrapper,
    TeleopInterventionWrapper,
)
from rl_garden_real_world.kinematics import PandaKinematics
from rl_garden_real_world.teleop.leader_cartesian import LeaderArmCartesianSource

READY_Q = [0.0, -math.pi / 4, 0.0, -3 * math.pi / 4, 0.0, math.pi / 2, math.pi / 4]
SMALL_DQ = np.array([0.006, -0.004, 0.006, 0.004, -0.006, 0.004, 0.006])
KINEMATICS = PandaKinematics()
GELLO_OPTION_KEYS = {
    "port",
    "baudrate",
    "joint_ids",
    "joint_offsets_rad",
    "joint_signs",
    "wrap_centers_rad",
    "gripper_id",
    "gripper_open_rad",
    "gripper_closed_rad",
    "gripper_binarize_threshold",
    "intervention",
    "event_device",
    "event_keys",
    "poll_interval_s",
}


@pytest.fixture(autouse=True)
def _claim_dir(tmp_path, monkeypatch):
    monkeypatch.setattr(claims, "_DEFAULT_LOCK_DIR", tmp_path / "claims")


class _Leader:
    """A joint_abs leader part: each ``read()`` returns the current ``q``/``gripper``/``intervening``
    (joint angles float32, as a device sample carries them)."""

    action_space = "joint_abs"
    action_dim = 7
    name = "leader"

    def __init__(self, q=READY_Q, *, gripper=1.0, intervening=True, has_gripper=True):
        self.q = np.asarray(q, dtype=np.float32)
        self.has_gripper = has_gripper
        self.gripper = gripper if has_gripper else None
        self.intervening = intervening
        self.events = {}
        self.reset_calls = 0
        self._seq = 0

    def read(self, timeout_s=None):
        self._seq += 1
        events = {key: bool(self.events.get(key, False)) for key in EVENT_KEYS}
        self.events = {}
        return TeleopSample(
            action=self.q.copy(),
            action_space=self.action_space,
            gripper=self.gripper,
            buttons={},
            events=events,
            intervening=self.intervening,
            timestamp=0.0,
            seq=self._seq,
        )

    def reset(self):
        self.reset_calls += 1


def franka_real_update(pose, action, pos_scale=0.02, rot_scale=0.1):
    """``FrankaRealEnv.step``'s target pose (without the safety box): base-frame translation and
    a left-multiplied rotation vector, quaternions xyzw."""
    action = np.asarray(action, dtype=np.float64)
    position = pose[:3] + action[:3] * pos_scale
    rotation = Rotation.from_rotvec(action[3:6] * rot_scale) * Rotation.from_quat(pose[3:7])
    return np.concatenate([position, rotation.as_quat()])


class _Follower:
    """Follower TCP pose, moved by ``franka_real_update``."""

    def __init__(self, pose):
        self.pose = np.asarray(pose, dtype=np.float64)

    def __call__(self):
        return self.pose.copy()

    def apply(self, action):
        self.pose = franka_real_update(self.pose, action)


def _pose(position, rotation: Rotation) -> np.ndarray:
    return np.concatenate([position, rotation.as_quat()])


def _ee_pose(q) -> np.ndarray:
    transform = KINEMATICS.ee_transform(q)
    return _pose(transform[:3, 3], Rotation.from_matrix(transform[:3, :3]))


def _offset_pose(q, offset_m, rotvec) -> np.ndarray:
    """The leader's FK pose of ``q`` moved by ``offset_m`` and rotated by ``rotvec`` (base frame)."""
    pose = _ee_pose(q)
    return _pose(pose[:3] + offset_m, Rotation.from_rotvec(rotvec) * Rotation.from_quat(pose[3:]))


def _source(leader, follower, **kwargs) -> LeaderArmCartesianSource:
    return LeaderArmCartesianSource(leader, follower_pose_getter=follower, **kwargs)


def _expected_twist(target: np.ndarray, follower_pose: np.ndarray, pos_scale=0.02, rot_scale=0.1):
    position = (target[:3, 3] - follower_pose[:3]) / pos_scale
    rotation = Rotation.from_matrix(target[:3, :3]) * Rotation.from_quat(follower_pose[3:]).inv()
    return np.concatenate([position, rotation.as_rotvec() / rot_scale])


def _rotation_error(pose, target_quat) -> float:
    return (Rotation.from_quat(pose[3:7]) * Rotation.from_quat(target_quat).inv()).magnitude()


def _transform(pose) -> np.ndarray:
    transform = np.eye(4)
    transform[:3, :3] = Rotation.from_quat(pose[3:]).as_matrix()
    transform[:3, 3] = pose[:3]
    return transform


# -- anchored mapping -------------------------------------------------------------------


def test_anchored_onset_emits_zero_twist_then_tracks_the_leader_displacement():
    leader = _Leader(intervening=False)
    follower_pose = _offset_pose(READY_Q, [0.05, 0.03, -0.04], [0.1, -0.05, 0.2])
    source = _source(leader, _Follower(follower_pose))

    reading = source.poll()
    assert reading.intervened is False
    np.testing.assert_array_equal(reading.action[:6], 0.0)

    leader.intervening = True
    reading = source.poll()  # onset: anchors the follower to the leader here
    assert reading.intervened is True
    np.testing.assert_allclose(reading.action[:6], 0.0, atol=1e-9)

    q_onset = leader.q.astype(np.float64)
    leader.q = (q_onset + SMALL_DQ).astype(np.float32)
    reading = source.poll()
    leader_onset, leader_now = KINEMATICS.ee_transform(q_onset), KINEMATICS.ee_transform(leader.q)
    target = np.eye(4)  # leader displacement applied to the follower, translation and rotation decoupled
    target[:3, 3] = follower_pose[:3] + (leader_now[:3, 3] - leader_onset[:3, 3])
    target[:3, :3] = (
        leader_now[:3, :3] @ leader_onset[:3, :3].T @ _transform(follower_pose)[:3, :3]
    )
    expected = _expected_twist(target, follower_pose)
    assert np.all(np.abs(expected) < 1.0)  # small enough not to saturate
    assert np.abs(expected).max() > 0.05
    np.testing.assert_allclose(reading.action[:6], expected, atol=1e-6)


def test_anchored_leader_rotation_turns_the_follower_in_place():
    """A pure leader rotation (joint 7 spins the end effector about its own axis) must not
    translate a follower whose pose differs from the leader's at onset."""
    leader = _Leader()
    follower = _Follower(_offset_pose(READY_Q, [0.1, 0.05, -0.05], [0.2, -0.1, 0.3]))
    source = _source(leader, follower)
    source.poll()  # onset
    leader_onset = KINEMATICS.ee_transform(leader.q)
    leader.q = leader.q + np.array([0, 0, 0, 0, 0, 0, 0.05], dtype=np.float32)
    leader_now = KINEMATICS.ee_transform(leader.q)
    np.testing.assert_allclose(leader_now[:3, 3], leader_onset[:3, 3], atol=1e-12)

    action = source.poll().action
    np.testing.assert_allclose(action[:3], 0.0, atol=1e-6)
    leader_rotation = Rotation.from_matrix(leader_now[:3, :3] @ leader_onset[:3, :3].T).as_rotvec()
    assert np.linalg.norm(leader_rotation) == pytest.approx(0.05, abs=1e-6)
    np.testing.assert_allclose(action[3:6], leader_rotation / 0.1, atol=1e-6)


def test_anchored_equals_absolute_when_the_follower_starts_at_the_leader_pose():
    follower = _Follower(_ee_pose(np.asarray(READY_Q, dtype=np.float32)))
    anchored_leader, absolute_leader = _Leader(), _Leader()
    anchored = _source(anchored_leader, follower)
    absolute = _source(absolute_leader, follower, mapping="absolute")
    anchored.poll()
    for leader in (anchored_leader, absolute_leader):
        leader.q = (np.asarray(READY_Q) + SMALL_DQ).astype(np.float32)
    np.testing.assert_allclose(anchored.poll().action, absolute.poll().action, atol=1e-6)


def test_anchor_is_renewed_at_each_onset_and_after_reset():
    leader = _Leader()
    source = _source(leader, _Follower(_offset_pose(READY_Q, [0.02, 0.0, 0.0], [0.0, 0.0, 0.1])))
    source.poll()  # onset
    leader.q = leader.q + SMALL_DQ.astype(np.float32)
    assert np.abs(source.poll().action[:6]).max() > 0.05

    leader.intervening = False
    source.poll()
    leader.intervening = True
    np.testing.assert_allclose(source.poll().action[:6], 0.0, atol=1e-9)  # re-anchored at the moved leader

    leader.q = leader.q + SMALL_DQ.astype(np.float32)
    assert np.abs(source.poll().action[:6]).max() > 0.05
    source.reset()
    assert leader.reset_calls == 1
    np.testing.assert_allclose(source.poll().action[:6], 0.0, atol=1e-9)


# -- absolute mapping, frames and saturation ---------------------------------------------


def test_absolute_mapping_converges_through_franka_real_update_math():
    q = np.asarray([0.3, -0.5, 0.2, -2.0, 0.1, 1.8, -0.4], dtype=np.float32)
    target = _ee_pose(q)
    follower = _Follower(_offset_pose(q, [0.1, -0.06, 0.08], [0.25, -0.15, 0.2]))
    source = _source(_Leader(q), follower, mapping="absolute")

    first = source.poll().action
    assert np.abs(first[:6]).max() == pytest.approx(1.0)
    follower.apply(first)
    for _ in range(40):
        follower.apply(source.poll().action)
    np.testing.assert_allclose(follower.pose[:3], target[:3], atol=1e-8)
    assert _rotation_error(follower.pose, target[3:]) < 1e-7


def test_unsaturated_absolute_step_lands_on_the_target():
    q = np.asarray(READY_Q, dtype=np.float32)
    target = _ee_pose(q)
    follower = _Follower(_offset_pose(q, [0.01, -0.005, 0.012], [0.03, 0.05, -0.04]))
    source = _source(_Leader(q), follower, mapping="absolute")
    action = source.poll().action
    assert np.abs(action[:6]).max() < 1.0
    follower.apply(action)
    np.testing.assert_allclose(follower.pose[:3], target[:3], atol=1e-8)
    assert _rotation_error(follower.pose, target[3:]) < 1e-6


def test_far_targets_saturate_at_exactly_one():
    q = np.asarray(READY_Q, dtype=np.float32)
    follower = _Follower(_offset_pose(q, [0.5, 0.0, 0.0], [0.0, 0.0, math.pi / 2]))
    action = _source(_Leader(q), follower, mapping="absolute").poll().action
    np.testing.assert_allclose(action[:6], [-1.0, 0.0, 0.0, 0.0, 0.0, -1.0], atol=1e-6)


# -- gripper --------------------------------------------------------------------------------


@pytest.mark.parametrize(
    "width, expected", [(1.0, 1.0), (0.8, 1.0), (0.51, 1.0), (0.5, -1.0), (0.3, -1.0), (0.0, -1.0)]
)
def test_gripper_width_is_binarized_at_half_travel(width, expected):
    follower = _Follower(_ee_pose(np.asarray(READY_Q, dtype=np.float32)))
    reading = _source(_Leader(gripper=width), follower).poll()
    assert reading.action.shape == (7,)
    assert reading.action[6] == expected


def test_gripper_threshold_none_keeps_the_continuous_value():
    follower = _Follower(_ee_pose(np.asarray(READY_Q, dtype=np.float32)))
    reading = _source(_Leader(gripper=0.7), follower, gripper_binarize_threshold=None).poll()
    assert reading.action[6] == pytest.approx(0.4)


def test_leader_without_gripper_holds_the_last_sent_gripper_action():
    follower = _Follower(_ee_pose(np.asarray(READY_Q, dtype=np.float32)))
    leader = _Leader(has_gripper=False)
    reading = _source(leader, follower, last_gripper_action_getter=lambda: -0.3).poll()
    assert reading.action[6] == pytest.approx(-0.3)
    with pytest.raises(ValueError, match="last_gripper_action_getter"):
        _source(_Leader(has_gripper=False), follower)


def test_events_are_forwarded():
    leader = _Leader()
    source = _source(leader, _Follower(_ee_pose(leader.q)))
    leader.events = {"episode_end": True, "success": True}
    reading = source.poll()
    assert reading.episode_end is True and dict(reading.events) == {"success": True}
    reading = source.poll()
    assert reading.episode_end is False and not reading.events


# -- contracts and failures -------------------------------------------------------------------


def test_intervention_wrapper_accepts_it_on_a_cartesian_env_and_rejects_joint_envs():
    leader = _Leader()
    wrapper = InterventionWrapper(ContractEnv(), _source(leader, _Follower(_ee_pose(leader.q))))
    wrapper.reset()
    _, _, _, _, info = wrapper.step(torch.zeros(1, 7))
    assert info["intervened"] is True and info["intervene_action"].shape == (1, 7)

    with pytest.raises(ValueError, match="cartesian_delta.*joint_abs"):
        InterventionWrapper(
            ContractEnv("joint_abs", 7), _source(_Leader(), _Follower(_ee_pose(leader.q)))
        )


def test_registered_as_leader_cartesian():
    leader = _Leader()
    wrapper = TeleopInterventionWrapper(
        ContractEnv(),
        device="leader_cartesian",
        leader=leader,
        follower_pose_getter=_Follower(_ee_pose(leader.q)),
    )
    assert isinstance(wrapper.source, LeaderArmCartesianSource)
    assert wrapper.source.action_mode == "cartesian_delta" and wrapper.source.arm_action_dim == 6


@pytest.mark.parametrize(
    "attributes, match",
    [({"action_space": "cartesian_delta", "action_dim": 6}, "joint_abs"), ({"action_dim": 6}, "7 joints")],
)
def test_rejects_leaders_that_are_not_7_joint_abs(attributes, match):
    leader = _Leader()
    for key, value in attributes.items():
        setattr(leader, key, value)
    with pytest.raises(ValueError, match=match):
        _source(leader, _Follower(_ee_pose(READY_Q)))


def test_declared_contract_must_match_the_part():
    with pytest.raises(ValueError, match="declares"):
        _source(
            _Leader(has_gripper=False),
            _Follower(_ee_pose(READY_Q)),
            teleop_contract=("joint_abs", 7, True),
            last_gripper_action_getter=lambda: 1.0,
        )
    with pytest.raises(ValueError, match="joint_abs"):
        _source(lambda: None, _Follower(_ee_pose(READY_Q)), teleop_contract=("joint_delta", 7, True))


def test_resolved_part_is_checked_when_it_appears():
    parts = [None]
    source = _source(lambda: parts[0], _Follower(_ee_pose(READY_Q)))  # nothing to check yet
    with pytest.raises(DeviceFault, match="no teleop part"):
        source.poll()
    parts[0] = _Leader()
    parts[0].action_dim = 6
    with pytest.raises(DeviceFault, match="7 joints"):
        source.poll()


def test_failures_propagate_instead_of_idling():
    leader = _Leader()
    source = _source(leader, lambda: None)
    with pytest.raises(RuntimeError, match="before the env was reset"):
        source.poll()

    def failing_read(timeout_s=None):
        raise DeviceFault("bus error", device="leader")

    leader.read = failing_read
    with pytest.raises(DeviceFault, match="bus error"):
        source.poll()


def test_exactly_one_leader():
    with pytest.raises(ValueError, match="exactly one"):
        LeaderArmCartesianSource(follower_pose_getter=lambda: None)
    with pytest.raises(ValueError, match="exactly one leader"):
        LeaderArmCartesianSource.from_env(ContractEnv(), follower_pose_getter=lambda: None)
    with pytest.raises(ValueError, match="exactly one leader"):
        LeaderArmCartesianSource.from_env(
            ContractEnv(),
            leader=_Leader(),
            remote_address="10.0.0.2:50051",
            follower_pose_getter=lambda: None,
        )


# -- follower discovery from the env --------------------------------------------------------


class _FakeBridge(FrankaBridgeClient):
    """In-memory bridge: the robot reaches every sent pose."""

    def __init__(self, pose):
        self.pose = np.asarray(pose, dtype=np.float64)
        self.gripper_calls = []

    def reset_joints(self):
        pass

    def get_state(self):
        return {
            "pose": self.pose.tolist(),
            "vel": [0.0] * 6,
            "gripper_pos": 0.0,
            "force": [0.0] * 3,
            "torque": [0.0] * 3,
        }

    def send_pose(self, pose):
        self.pose = np.asarray(pose, dtype=np.float64)

    def send_gripper(self, open_gripper):
        self.gripper_calls.append(bool(open_gripper))


def test_franka_real_pose_and_scales_found_through_the_wrapper_chain_and_followed():
    q = np.asarray(READY_Q, dtype=np.float32)
    target = _ee_pose(q)
    bridge = _FakeBridge(_offset_pose(q, [0.06, -0.004, 0.05], [0.0, 0.0, 0.2]))
    cfg = FrankaRealEnvConfig(
        bridge_url="http://unused",
        action_scale=(0.01, 0.05),
        reward_scale=2.0,  # adds a RewardScaleBiasWrapper above FrankaRealEnv
        max_episode_steps=1000,
        safety_box_low=(-10.0, -10.0, -10.0),
        safety_box_high=(10.0, 10.0, 10.0),
    )
    env = TeleopInterventionWrapper(
        make_franka_real_env(cfg, bridge_client=bridge),
        device="leader_cartesian",
        leader=_Leader(q, gripper=1.0),
        mapping="absolute",
    )
    env.reset()
    _, _, _, _, info = env.step(torch.zeros(1, 7))
    first = info["intervene_action"][0].numpy()
    np.testing.assert_allclose(first[:6], [-1.0, 0.4, -1.0, 0.0, 0.0, -1.0], atol=1e-5)  # 0.004 m / 0.01
    for _ in range(30):
        env.step(torch.zeros(1, 7))
    np.testing.assert_allclose(bridge.pose[:3], target[:3], atol=1e-8)
    assert _rotation_error(bridge.pose, target[3:]) < 1e-7
    assert bridge.gripper_calls[-1] is True


class _CartesianRealEnv(ContractEnv):
    """``RealRobotEnv``-shaped: a ``last_observation`` property and a cartesian contract with scales."""

    def __init__(self, tcp_pose):
        super().__init__()
        self.action_contract.update(xyz_scale_m=0.01, rot_scale_rad=0.05)
        self._tcp_pose = np.asarray(tcp_pose, dtype=np.float32)
        self._observation = None

    @property
    def last_observation(self):
        return self._observation

    def reset(self, **kwargs):
        self._observation = {"state": {"tcp_pose": self._tcp_pose.copy()}}
        return super().reset(**kwargs)


def test_real_robot_cartesian_env_pose_comes_from_the_last_observation():
    q = np.asarray(READY_Q, dtype=np.float32)
    env = _CartesianRealEnv(_offset_pose(q, [0.004, 0.0, 0.0], [0.0, 0.0, 0.0]))
    source = LeaderArmCartesianSource.from_env(env, leader=_Leader(q), mapping="absolute")
    with pytest.raises(RuntimeError, match="before the env was reset"):
        source.poll()
    env.reset()
    np.testing.assert_allclose(source.poll().action[:6], [-0.4, 0.0, 0.0, 0.0, 0.0, 0.0], atol=1e-4)


def test_env_without_a_follower_pose_is_rejected():
    with pytest.raises(ValueError, match="follower's TCP pose"):
        LeaderArmCartesianSource.from_env(ContractEnv(), leader=_Leader())


# -- leader on a stub robot (daemon on the robot host, or a local yaml) ------------------------


def _leader_teleop_config(script) -> TeleopConfig:
    return TeleopConfig(
        name="gello",
        backend="fake",
        options={"action_space": "joint_abs", "num_joints": 7, "script": script, "on_end": "hold_last"},
    )


def _gello_station(script) -> FrankaRobot:
    return FrankaRobot(
        FrankaConfig(
            name="gello_station",
            arm_backend="fake",
            gripper=None,
            cameras=[],
            teleop=_leader_teleop_config(script),
        )
    )


def test_remote_stub_robot_leader_is_connected_by_start_and_released_by_close():
    moved = (np.asarray(READY_Q) + SMALL_DQ).tolist()
    script = [
        {"action": READY_Q, "gripper": 1.0, "active": False},
        {"action": READY_Q, "gripper": 0.0, "active": True},
        {"action": moved, "active": True},
    ]
    service = RobotService(_gello_station(script), watchdog_s=30.0)
    robot = RemoteRobot.from_client(LocalClient(service))
    follower = _Follower(_offset_pose(READY_Q, [0.02, 0.01, -0.01], [0.0, 0.1, 0.0]))
    try:
        source = LeaderArmCartesianSource(leader_robot=robot, follower_pose_getter=follower)
        assert not robot.is_connected and source.leader_robot is robot
        wrapper = InterventionWrapper(ContractEnv(), source)  # validates, then start() connects
        assert robot.is_connected
        wrapper.reset()

        _, _, _, _, info = wrapper.step(torch.zeros(1, 7))
        assert info["intervened"] is False

        _, _, _, _, info = wrapper.step(torch.zeros(1, 7))
        assert info["intervention_onset"] is True
        np.testing.assert_allclose(info["intervene_action"][0, :6].numpy(), 0.0, atol=1e-6)
        assert info["intervene_action"][0, 6].item() == -1.0

        _, _, _, _, info = wrapper.step(torch.zeros(1, 7))
        assert np.abs(info["intervene_action"][0, :6].numpy()).max() > 0.05

        wrapper.close()
        assert not robot.is_connected
    finally:
        if robot.is_connected:
            robot.disconnect()
        service.close()


def test_start_releases_the_leader_robot_when_its_teleop_part_is_not_a_leader_arm():
    station = FrankaRobot(
        FrankaConfig(
            name="gello_station",
            arm_backend="fake",
            gripper=None,
            cameras=[],
            teleop=TeleopConfig(name="spacemouse", backend="fake"),  # cartesian_delta
        )
    )
    service = RobotService(station, watchdog_s=30.0)
    robot = RemoteRobot.from_client(LocalClient(service))
    try:
        source = LeaderArmCartesianSource(leader_robot=robot, follower_pose_getter=lambda: None)
        with pytest.raises(DeviceFault, match="joint_abs"):
            source.start()
        assert not robot.is_connected
    finally:
        if robot.is_connected:
            robot.disconnect()
        service.close()


def test_from_env_with_remote_address_builds_an_unconnected_remote_robot():
    source = LeaderArmCartesianSource.from_env(
        ContractEnv(), remote_address="10.0.0.2:50051", follower_pose_getter=lambda: None
    )
    robot = source.leader_robot
    assert isinstance(robot, RemoteRobot)
    assert robot.config.address == "10.0.0.2:50051" and not robot.is_connected
    source.close()  # never connected: a no-op


def test_from_env_with_robot_yaml_builds_and_owns_a_local_robot(tmp_path):
    robot_yaml = tmp_path / "gello_station.yaml"
    teleop = _leader_teleop_config([{"action": READY_Q, "gripper": 1.0, "active": True}])
    robot_yaml.write_text(
        yaml.safe_dump(
            {
                "type": "franka",
                "name": "gello_station",
                "arm_backend": "fake",
                "gripper": None,
                "cameras": [],
                "teleop": {"name": teleop.name, "backend": teleop.backend, "options": teleop.options},
            }
        )
    )
    source = LeaderArmCartesianSource.from_env(
        ContractEnv(), robot_yaml=str(robot_yaml), follower_pose_getter=_Follower(_ee_pose(READY_Q))
    )
    try:
        source.start()
        reading = source.poll()
        assert reading.intervened is True and reading.action[6] == 1.0
    finally:
        source.close()
    assert not source.leader_robot.is_connected


def test_gello_station_example_yaml_is_a_stub_robot_with_every_gello_option():
    path = Path(__file__).resolve().parents[1] / "robot_infra" / "docs" / "robot.gello_station.example.yaml"
    data = yaml.safe_load(path.read_text())
    assert data["type"] == "franka"
    config = FrankaConfig.from_dict(data)
    assert config.name == "gello_station" and config.arm_backend == "fake"
    assert config.gripper is None and config.cameras == []
    assert config.teleop.backend == "gello"
    assert set(config.teleop.options) == GELLO_OPTION_KEYS
    assert len(config.teleop.options["joint_ids"]) == 7


def test_leader_without_gripper_reads_its_arm_from_a_one_entry_gripper_vector():
    follower = _Follower(_ee_pose(np.asarray(READY_Q, dtype=np.float32)))
    leader = _Leader(has_gripper=False)
    reading = _source(leader, follower, last_gripper_action_getter=lambda: torch.tensor([-0.25])).poll()
    assert reading.action[6] == pytest.approx(-0.25)
