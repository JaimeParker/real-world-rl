"""The two-arm workflow end to end, hardware-free, the way a station runs it.

A station yaml lists two fake Frankas (``robots: {left, right}``), each with a fake
arm, gripper, two fake cameras and a scripted fake ``joint_abs`` leader part; both are served by
real daemons (``real_world_rl.server.daemon.build_daemon`` over gRPC on loopback). The actor builds
the ``real_robot_bimanual`` env through the backend registry (``env_building.build_hil_env``)
with ``remote`` per arm and ``--teleop-device bimanual --teleop-arm-device leader_arm``, then
runs the hil_serl actor loop, records demos with their contract sidecar, and runs the residual
and SiLRI actor loops. The learners load the recorded pkl as demos, ``real_world_bc`` trains on
it and ``deploy`` runs the BC checkpoint on both daemons.

The left leader's script loops over four reads: two idle reads (the policy acts), then two
intervening reads towards ``LEFT_LEADER`` with the trigger squeezed, the first of them with a
``success`` event. The right leader never intervenes, so its arm is held while the left one is.
"""
from __future__ import annotations

import dataclasses
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Optional

import numpy as np
import pytest
import real_world_rl.devices.claims as claims
import torch
import yaml
from _residual_fakes import FakeSyncClient
from rl_garden.encoders.config import EncoderConfig
from rl_garden.observations import ObservationConfig
from real_world_rl.devices.robots.franka import FRANKA_HOME_JOINTS
from real_world_rl.discovery.autoconfig import load_station

import real_world_rl.envs.backends.real_robot_bimanual  # noqa: F401  (registers the backend)
from real_world_rl.action_contract import (
    demo_contract_of,
    get_env_action_contract,
    read_contract_sidecar,
)
from real_world_rl.algorithms.residual import ResidualRLPDHybrid
from real_world_rl.demos.record import (
    RecordDemosArgs,
    hold_action_fn_for,
    record_demos,
    save_demos,
)
from real_world_rl.deploy import DeployArgs, restore_training_args, run_deploy
from real_world_rl.envs.bimanual import BimanualRealRobotEnvConfig
from real_world_rl.envs.real_robot import RealRobotEnvConfig
from real_world_rl.loops.hil_serl import HilSerlActorLoop, HilSerlLearnerLoop
from real_world_rl.loops.residual import ResidualActorLoop, ResidualLearnerLoop
from real_world_rl.loops.silri import SilriActorLoop, SilriLearnerLoop
from real_world_rl.policies.base_provider import ZeroBaseProvider
from real_world_rl.teleop.core.bimanual import BimanualTeleopSource
from real_world_rl.teleop.leader.joint import LeaderArmPartSource
from real_world_rl.training import env_building
from real_world_rl.training.entrypoints import hil_serl
from real_world_rl.training.entrypoints.bc import (
    RealWorldBCArgs,
    run_real_world_bc,
)
from real_world_rl.training.entrypoints.hil_serl import HilSerlArgs
from real_world_rl.training.entrypoints.residual_hil_serl import (
    ResidualHilSerlArgs,
    build_residual_hil_serl,
)
from real_world_rl.training.entrypoints.silri import SilriArgs, build_silri

BIMANUAL = "real_robot_bimanual"
ARMS = ("left", "right")
CAMERAS = {"left": ("left_wrist", "left_front"), "right": ("right_wrist", "right_front")}
ALL_CAMERAS = CAMERAS["left"] + CAMERAS["right"]
ROBOT_IPS = {"left": "172.16.0.2", "right": "172.16.1.2"}
TOKENS = {"left": None, "right": "station-token"}
FRAME = 64  # the plain_conv encoder's native input size
NUM_JOINTS = 7
ACTION_DIM = 2 * NUM_JOINTS + 2
# Panda joint limits (rad); the home pose and LEFT_LEADER lie inside.
PANDA_JOINT_LIMITS = [
    [-2.8973, 2.8973],
    [-1.7628, 1.7628],
    [-2.8973, 2.8973],
    [-3.0718, -0.0698],
    [-2.8973, 2.8973],
    [-0.0175, 3.7525],
    [-2.8973, 2.8973],
]
HOME = [float(q) for q in FRANKA_HOME_JOINTS]
# 0.1 rad off home on joints 0 and 1: every intervened joint_delta step saturates there.
LEFT_LEADER = [HOME[0] + 0.1, HOME[1] - 0.1, *HOME[2:]]
SATURATED_LEFT_BLOCK = [1.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0]
IDLE_READS = 2  # per script cycle, before the left operator takes over for two reads
EPISODE_STEPS = 4
FAST_HZ = 1000.0
CLOSED, OPEN = -1.0, 1.0


@dataclass
class StationDaemon:
    service: Any
    transport: Any
    address: str
    token: Optional[str]


# -- station yaml and daemons -------------------------------------------------------------


def _left_script() -> list[dict]:
    return [
        {"action": LEFT_LEADER, "gripper": 1.0, "active": False, "hold": IDLE_READS},
        {"action": LEFT_LEADER, "gripper": 0.0, "active": True, "events": {"success": True}},
        {"action": LEFT_LEADER, "gripper": 0.0, "active": True},
    ]


def _right_script() -> list[dict]:
    # Never intervenes; its trigger is squeezed, which must not reach the held right gripper.
    return [{"action": HOME, "gripper": 0.0, "active": False}]


def _station_robot(arm: str, index: int, script: list[dict], on_end: str) -> dict:
    return {
        "type": "franka",
        "name": arm,
        "arm_backend": "fake",
        "robot_ip": ROBOT_IPS[arm],
        "home_joints": HOME,
        "gripper": {"backend": "fake"},
        "cameras": [
            {"name": cam, "backend": "fake", "width": FRAME, "height": FRAME, "fps": 60} for cam in CAMERAS[arm]
        ],
        "teleop": {
            "name": "leader",
            "backend": "fake",
            "options": {
                "action_space": "joint_abs",
                "num_joints": NUM_JOINTS,
                "has_gripper": True,
                "script": script,
                "on_end": on_end,
            },
        },
        "daemon": {"bind": f"0.0.0.0:{50051 + index}", "token": TOKENS[arm]},
    }


def _station() -> dict:
    return {
        "robots": {
            "left": _station_robot("left", 0, _left_script(), on_end="loop"),
            "right": _station_robot("right", 1, _right_script(), on_end="hold_last"),
        }
    }


@pytest.fixture
def station_daemons(tmp_path, monkeypatch):
    """Both robots of the station yaml, each served by its own daemon on a loopback port (the
    yaml's ``daemon.bind`` names the robot PC's port; the test binds an ephemeral one)."""
    pytest.importorskip("grpc")
    from real_world_rl.server import daemon

    monkeypatch.setattr(claims, "_DEFAULT_LOCK_DIR", tmp_path / "claims")
    path = tmp_path / "station.yaml"
    path.write_text(yaml.safe_dump(_station()), encoding="utf-8")
    station = load_station(path)
    assert station.multi_robot and list(station.robots) == list(ARMS)
    daemon_args = daemon.parse_args(["--station", str(path), "--no-preflight", "--watchdog-s", "30"])

    served: dict[str, StationDaemon] = {}
    try:
        for arm, robot in station.robots.items():
            service, transport = daemon.build_daemon(daemon_args, config=robot.config)
            served[arm] = StationDaemon(service, transport, "", robot.token)
            transport.start(service, "127.0.0.1:0", token=robot.token)
            served[arm].address = f"127.0.0.1:{transport.port}"
        yield served
    finally:
        for station_daemon in served.values():
            station_daemon.transport.stop(grace_s=0.1)
            station_daemon.service.close()


# -- args -------------------------------------------------------------------------------


def _bimanual_config(daemons: dict[str, StationDaemon]) -> BimanualRealRobotEnvConfig:
    arms = {
        arm: RealRobotEnvConfig(
            remote=daemons[arm].address,
            token=daemons[arm].token,
            cameras=list(CAMERAS[arm]),
            image_size=(FRAME, FRAME),
            action_mode="joint_delta",
            joint_scale_rad=0.1,
            state_fields=("joint_pos", "gripper_pos"),
            control_hz=FAST_HZ,
            joint_limits=PANDA_JOINT_LIMITS,
        )
        for arm in ARMS
    }
    return BimanualRealRobotEnvConfig(**arms, max_episode_steps=EPISODE_STEPS, reset_settle_s=0.0)


def _small_encoder() -> EncoderConfig:
    return EncoderConfig(features_dim=16, proprio_latent_dim=8, image_augmentation="random_shift")


def _env_fields(daemons: dict[str, StationDaemon]) -> dict:
    """The env and teleop flags every two-arm command of the station shares."""
    return dict(
        env_backend=BIMANUAL,
        real_robot_bimanual=_bimanual_config(daemons),
        obs=ObservationConfig(state=True, rgb=ALL_CAMERAS),
        teleop_device="bimanual",
        teleop_arm_device="leader_arm",
        teleop_onset="ramp",
        robot_teleop_action_mode="joint_abs",
        control_hz=FAST_HZ,
    )


def _rlpd_fields(daemons) -> dict:
    return dict(
        _env_fields(daemons),
        hidden_dim=16,
        discrete_hidden_dim=8,
        n_critics=2,
        buffer_size=16,
        buffer_device="cpu",
        encoder=_small_encoder(),
    )


def _hil_serl_args(daemons) -> HilSerlArgs:
    return HilSerlArgs(**_rlpd_fields(daemons))


def _residual_args(daemons) -> ResidualHilSerlArgs:
    return ResidualHilSerlArgs(**_rlpd_fields(daemons), residual_variant="hybrid", base_provider="zero")


def _silri_args(daemons) -> SilriArgs:
    return SilriArgs(
        **_env_fields(daemons),
        buffer_size=16,
        buffer_device="cpu",
        feature_dim=8,
        hidden_dim=8,
        behavior_hidden_dims=(8,),
        lambda_hidden_dims=(8,),
        discrete_hidden_dim=8,
        encoder=_small_encoder(),
    )


# -- helpers ------------------------------------------------------------------------------


class ZeroPolicy:
    def eval(self):
        pass

    def load_state_dict(self, state_dict):
        del state_dict

    def predict(self, obs, deterministic=False):
        del obs, deterministic
        return torch.zeros(1, ACTION_DIM)


def _as_array(action) -> np.ndarray:
    return torch.as_tensor(action).detach().cpu().double().reshape(-1).numpy()


def _assert_left_drives_and_right_is_held(action, right_gripper: float, observation) -> None:
    """An intervened two-arm action: the left leader's saturated delta, the right arm held (zero
    delta), the left trigger squeezed (closed) and the right gripper at ``right_gripper``."""
    action = _as_array(action)
    assert action.shape == (ACTION_DIM,)
    follower = _as_array(observation["state"])[:NUM_JOINTS]
    expected = np.clip((np.asarray(LEFT_LEADER) - follower) / 0.1, -1.0, 1.0)
    np.testing.assert_allclose(action[:NUM_JOINTS], expected, atol=1e-4)
    np.testing.assert_array_equal(action[NUM_JOINTS : 2 * NUM_JOINTS], np.zeros(NUM_JOINTS))
    np.testing.assert_allclose(action[-2:], [CLOSED, right_gripper], atol=1e-6)


def _assert_rows_of_both_arms(rows: list[dict]) -> None:
    """Every pushed row is 16-D; the intervened ones (any three consecutive reads hold one) carry
    the left leader's squeezed trigger with the right arm held. The left block depends on where
    the policy moved the follower, so it is not checked here."""
    assert any(row["intervened"] for row in rows)
    for row in rows:
        assert torch.as_tensor(row["action"]).shape == (1, ACTION_DIM)
        if row["intervened"]:
            action = _as_array(row["action"])
            np.testing.assert_array_equal(action[NUM_JOINTS : 2 * NUM_JOINTS], np.zeros(NUM_JOINTS))
            assert action[-2] == CLOSED


def _robots_connected(daemons: dict[str, StationDaemon]) -> list[bool]:
    return [daemons[arm].service.robot.is_connected for arm in ARMS]


# -- stages -------------------------------------------------------------------------------


def _check_actor_env(env) -> None:
    keys = set(env.single_observation_space.spaces)
    assert keys == {"state"} | {f"rgb_{cam}" for cam in ALL_CAMERAS}
    for cam in ALL_CAMERAS:
        assert env.single_observation_space[f"rgb_{cam}"].shape == (FRAME, FRAME, 3)
    assert env.single_action_space.shape == (ACTION_DIM,)
    contract = get_env_action_contract(env)
    assert contract["num_arms"] == 2 and contract["arm_names"] == list(ARMS)
    assert contract["action_mode"] == "joint_delta"
    source = env.source
    assert isinstance(source, BimanualTeleopSource) and source.arm_names == list(ARMS)
    assert all(isinstance(child, LeaderArmPartSource) for child in source.sources.values())


def _run_hil_serl_actor(env) -> list[dict]:
    sync_client = FakeSyncClient()
    HilSerlActorLoop(env, ZeroPolicy(), sync_client, control_hz=FAST_HZ).run(total_steps=6)
    rows = sync_client.pushed
    # Reads 1-4 are the first script cycle; the episode truncates after step 4 and reads 5-6
    # start the next cycle (idle).
    assert [row["intervened"] for row in rows] == [False, False, True, True, False, False]
    for row in rows:
        if row["intervened"]:
            # The right gripper holds the zero policy's last action, not its squeezed trigger.
            _assert_left_drives_and_right_is_held(row["action"], right_gripper=0.0, observation=row["obs"])
        else:
            np.testing.assert_array_equal(_as_array(row["action"]), np.zeros(ACTION_DIM))
    return rows


def _record(env, daemons, demo_path: Path) -> list[dict]:
    args = RecordDemosArgs(**_env_fields(daemons), output_path=str(demo_path), num_successes=1)
    episodes = []

    def stop_unless_kept(episode) -> None:
        episodes.append(episode)
        if not episode.kept:
            raise AssertionError(f"recorded episode {episode.index} was not kept: {episode.discarded_reason}")

    transitions = record_demos(env, args, hold_action_fn=hold_action_fn_for(env), on_episode=stop_unless_kept)
    save_demos(transitions, demo_path, demo_contract_of(env))

    # A fresh connection starts the leader script: its first two active reads intervene.
    assert [(episode.steps, episode.success) for episode in episodes] == [(EPISODE_STEPS, True)]
    assert len(transitions) == 2
    for transition in transitions:
        # The right gripper holds the open gripper of the reset.
        _assert_left_drives_and_right_is_held(transition["action"], right_gripper=OPEN, observation=transition["obs"])
        assert set(transition["obs"]) == {"state"} | {f"rgb_{cam}" for cam in ALL_CAMERAS}
    assert demo_path.exists()
    sidecar = read_contract_sidecar(demo_path)
    assert sidecar["num_arms"] == 2 and sidecar["arm_names"] == list(ARMS)
    assert sidecar["joint_limits"] == [PANDA_JOINT_LIMITS, PANDA_JOINT_LIMITS]
    assert sidecar["action_mode"] == "joint_delta" and sidecar["arm_action_dim"] == NUM_JOINTS
    return transitions


def _run_residual_actor(env, daemons) -> list[dict]:
    args = dataclasses.replace(_residual_args(daemons), buffer_size=8)
    agent = build_residual_hil_serl(args, env, None, None, None)
    assert isinstance(agent, ResidualRLPDHybrid) and agent.num_grippers == 2
    assert agent.policy.residual_dim == 2 * NUM_JOINTS
    sync_client = FakeSyncClient()
    loop = ResidualActorLoop(
        env,
        agent.policy,
        sync_client,
        ZeroBaseProvider(ACTION_DIM),
        control_hz=FAST_HZ,
        device=agent.device,
        deterministic=True,
    )
    loop.run(total_steps=3)
    rows = sync_client.pushed
    assert len(rows) == 3
    _assert_rows_of_both_arms(rows)
    for row in rows:
        assert row["base_actions"].shape == (1, ACTION_DIM)
    return rows


def _run_silri_actor(env, daemons) -> list[dict]:
    agent = build_silri(dataclasses.replace(_silri_args(daemons), buffer_size=8), env, None, None, None)
    assert agent.num_grippers == 2
    sync_client = FakeSyncClient()
    loop = SilriActorLoop(env, agent.policy, sync_client, control_hz=FAST_HZ, device=agent.device)
    loop.run(total_steps=4)
    rows = sync_client.pushed
    assert len(rows) == 4 and loop._held_transition is None
    _assert_rows_of_both_arms(rows)
    for row in rows:
        assert torch.as_tensor(row["policy_action"]).shape == (1, ACTION_DIM)
        assert "intervention_onset" in row
    return rows


def _check_learners(daemons, demo_path: Path, transitions, actor_rows: dict, tmp_path: Path) -> None:
    """Each learner, built on its own (never connected) env, loads the recorded pkl as demos
    against the env's contract and takes its actor's rows."""
    learner_env = env_building.build_hil_env(_hil_serl_args(daemons), role="learner", enable_teleop=False)
    try:
        contract = demo_contract_of(learner_env)
        assert contract["num_arms"] == 2
        demo_actions = torch.cat([torch.as_tensor(t["action"]) for t in transitions])
        demos = [str(demo_path)]

        from rl_garden.training.online.entrypoints.rlpd_hybrid import build_rlpd_hybrid

        assert hil_serl._gripper_kwargs(learner_env) == {"num_grippers": 2}
        agent = build_rlpd_hybrid(
            _hil_serl_args(daemons), learner_env, None, None, None, **hil_serl._gripper_kwargs(learner_env)
        )
        agent.init_demo_buffer(16, 0.5)
        learner = HilSerlLearnerLoop(
            agent, "127.0.0.1", 0, checkpoint_dir=str(tmp_path / "hil_serl"),
            demo_dataset_paths=demos, action_contract=contract,
        )
        assert len(agent.offline_replay_buffer) == 2
        torch.testing.assert_close(agent.offline_replay_buffer.actions[:2, 0].cpu(), demo_actions)
        for row in actor_rows["hil_serl"]:
            learner._on_transition(row)
        assert len(agent.replay_buffer) == 6 and len(agent.offline_replay_buffer) == 4

        residual = build_residual_hil_serl(_residual_args(daemons), learner_env, None, None, None)
        residual.init_demo_buffer(16, 0.5)
        with pytest.warns(UserWarning, match="zero base actions"):
            residual_learner = ResidualLearnerLoop(
                residual, "127.0.0.1", 0, checkpoint_dir=str(tmp_path / "residual"),
                allow_base_less_demos=True, demo_dataset_paths=demos, action_contract=contract,
            )
        assert len(residual.offline_replay_buffer) == 2
        for row in actor_rows["residual"]:
            residual_learner._on_transition(row)
        assert len(residual.replay_buffer) == len(actor_rows["residual"])

        silri = build_silri(_silri_args(daemons), learner_env, None, None, None)
        silri.init_demo_buffer(16, 0.5)
        silri_learner = SilriLearnerLoop(
            silri, "127.0.0.1", 0, checkpoint_dir=str(tmp_path / "silri"),
            demo_dataset_paths=demos, action_contract=contract,
        )
        assert len(silri.offline_replay_buffer) == 2
        for row in actor_rows["silri"]:
            silri_learner._on_transition(row)
        assert len(silri.replay_buffer) == len(actor_rows["silri"])
    finally:
        learner_env.close()


def _train_bc(daemons, demo_path: Path, tmp_path: Path) -> tuple[RealWorldBCArgs, Path]:
    log_dir = tmp_path / "runs"
    args = RealWorldBCArgs(
        env_backend=BIMANUAL,
        real_robot_bimanual=_bimanual_config(daemons),
        obs=ObservationConfig(state=True, rgb=ALL_CAMERAS),
        encoder=_small_encoder(),
        # The recorded actions sit on the action bounds (saturated deltas, binary grippers).
        tanh_squash=False,
        demo_dataset_paths=(str(demo_path),),
        num_offline_steps=3,
        batch_size=2,
        buffer_size=16,
        buffer_device="cpu",
        device="cpu",
        log_dir=str(log_dir),
        log_type="none",
        exp_name="bc",
        checkpoint_freq=0,
        save_final_checkpoint=True,
    )
    run_real_world_bc(args)
    checkpoint = log_dir / "bc" / "checkpoints" / "final.pt"
    assert checkpoint.exists() and (log_dir / "bc" / "config.json").exists()
    return args, checkpoint


def _deploy_bc(daemons, bc_args: RealWorldBCArgs, checkpoint: Path) -> None:
    # The deploy command line gives the connection only; the run's config.json the contract.
    connection = BimanualRealRobotEnvConfig(
        **{
            arm: RealRobotEnvConfig(
                remote=daemons[arm].address,
                token=daemons[arm].token,
                cameras=list(CAMERAS[arm]),
                image_size=(FRAME, FRAME),
            )
            for arm in ARMS
        },
        reset_settle_s=0.0,
    )
    args = DeployArgs(
        checkpoint=str(checkpoint),
        algo="real_world_bc",
        env_backend=BIMANUAL,
        real_robot_bimanual=connection,
        episodes=1,
        max_steps_per_episode=2,
        device="cpu",
    )
    restored = restore_training_args(args)
    assert isinstance(restored, RealWorldBCArgs) and restored.device == "cpu"
    trained = restored.real_robot_bimanual
    assert trained.max_episode_steps == EPISODE_STEPS
    assert trained.left.action_mode == trained.right.action_mode == "joint_delta"
    assert trained.left.joint_limits == bc_args.real_robot_bimanual.left.joint_limits

    (result,) = run_deploy(args)
    assert result.steps == 2 and result.truncated and result.health_ok is True
    assert _robots_connected(daemons) == [False, False]


# -- the workflow -------------------------------------------------------------------------


def test_two_arm_station_end_to_end(station_daemons, tmp_path):
    daemons = station_daemons
    demo_path = tmp_path / "demos" / "bimanual.pkl"

    env = env_building.build_hil_env(_hil_serl_args(daemons), role="actor")
    try:
        _check_actor_env(env)
        actor_rows = {"hil_serl": _run_hil_serl_actor(env)}
        assert _robots_connected(daemons) == [False, False]
    finally:
        env.close()

    # Each command owns a fresh connection and releases it when its run ends.
    env = env_building.build_hil_env(_hil_serl_args(daemons), role="actor")
    try:
        transitions = _record(env, daemons, demo_path)
        assert _robots_connected(daemons) == [True, True]
    finally:
        env.close()
    assert _robots_connected(daemons) == [False, False]

    for name, args, run_actor in (
        ("residual", _residual_args(daemons), _run_residual_actor),
        ("silri", _silri_args(daemons), _run_silri_actor),
    ):
        env = env_building.build_hil_env(args, role="actor")
        try:
            actor_rows[name] = run_actor(env, daemons)
            assert _robots_connected(daemons) == [False, False]
        finally:
            env.close()

    _check_learners(daemons, demo_path, transitions, actor_rows, tmp_path)
    bc_args, checkpoint = _train_bc(daemons, demo_path, tmp_path)
    _deploy_bc(daemons, bc_args, checkpoint)
