"""The single-arm workflow end to end, hardware-free, the way a newcomer runs it.

A station yaml lists ONE fake Franka (fake arm, fake gripper, two fake cameras and
a scripted fake ``joint_abs`` leader part standing in for the GELLO the daemon serves), served by
a real daemon (``real_world_rl.server.daemon.build_daemon`` over gRPC on a loopback port). Every
command parses the tutorial's flags over a ``hil.yaml`` preset that points at that daemon
(``--config hil.yaml --teleop-device leader_arm --robot-teleop-action-mode joint_delta``); the
operator's buttons are the leader part's own scripted events (no ``--teleop-event-device``, so no
pynput). Only network sizes are shrunk after parsing.

Covered, each on its own connection to the daemon: demo recording (``run_recorder``), the
hil_serl actor and learner (metrics through a recording logger), ``real_world_bc`` training,
the residual actor over the BC checkpoint and over a tiny ACT checkpoint, the SiLRI actor, the
BC deploy and the classifier-data collector.

The leader part restarts its script on every connection and loops over six reads: two idle
reads (the policy acts), two intervening reads towards ``LEADER`` with the trigger squeezed, the
second firing ``success``, an idle read, and an idle read firing ``failure``. The episode cap is
six steps.
"""
from __future__ import annotations

import dataclasses
import pickle
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterator, Optional

import numpy as np
import pytest
import real_world_rl.devices.claims as claims
import torch
import yaml
from _residual_fakes import FakeSyncClient
from rl_garden.common.logger import Logger
from rl_garden.encoders.config import EncoderConfig
from real_world_rl.devices.robots.franka import FRANKA_HOME_JOINTS
from real_world_rl.discovery.autoconfig import load_station

import real_world_rl.envs.backends.real_robot  # noqa: F401  (registers the backend)
from real_world_rl.action_contract import (
    demo_contract_of,
    get_env_action_contract,
    read_contract_sidecar,
)
from real_world_rl.algorithms.residual import ResidualRLPDHybrid
from real_world_rl.demos.record import parse_record_args, run_recorder
from real_world_rl.deploy import DeployArgs, restore_training_args, run_deploy
from real_world_rl.envs.real_robot import RealRobotEnvConfig
from real_world_rl.loops.hil_serl import HilSerlActorLoop, HilSerlLearnerLoop
from real_world_rl.loops.residual import ResidualActorLoop
from real_world_rl.loops.silri import SilriActorLoop
from real_world_rl.reward import collect_data
from real_world_rl.teleop.leader.joint import LeaderArmPartSource
from real_world_rl.training import env_building
from real_world_rl.training.entrypoints import hil_serl
from real_world_rl.training.entrypoints import (
    residual_hil_serl as residual_entrypoint,
)
from real_world_rl.training.entrypoints import silri as silri_entrypoint
from real_world_rl.training.entrypoints.bc import (
    RealWorldBCArgs,
    run_real_world_bc,
)
from real_world_rl.training.entrypoints.hil_serl import HilSerlArgs
from real_world_rl.training.registry import registry as training_registry

ROBOT_NAME = "franka"
TOKEN = "station-token"
CAMERAS = ("wrist", "front")
OBS_KEYS = {"state"} | {f"rgb_{cam}" for cam in CAMERAS}
FRAME = 64  # the plain_conv encoder's native input size, also the fake cameras' size
NUM_JOINTS = 7
ACTION_DIM = NUM_JOINTS + 1
STATE_DIM = NUM_JOINTS + 1  # state_fields [joint_pos, gripper_pos]
JOINT_SCALE_RAD = 0.05
HOME = [float(q) for q in FRANKA_HOME_JOINTS]
# Inside --teleop-arm-tolerance-rad (0.1) of the home pose, so --teleop-onset arm arms at once;
# above joint_scale_rad, so the first intervened joint_delta step saturates joints 0 and 1.
LEADER_OFFSET_RAD = 0.08
LEADER = [HOME[0] + LEADER_OFFSET_RAD, HOME[1] - LEADER_OFFSET_RAD, *HOME[2:]]
SATURATED_ARM = [1.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0]
EPISODE_STEPS = 6
# Reads up to the success that ends the first episode of every connection.
STEPS_TO_SUCCESS = 4
FIRST_EPISODE_INTERVENED = [False, False, True, True]
FAST_HZ = 1000.0
CLOSED, OPEN = -1.0, 1.0


@dataclass
class StationDaemon:
    service: Any
    address: str
    token: Optional[str]

    def robot_connected(self) -> bool:
        return bool(self.service.robot.is_connected)


# -- station yaml, daemon and preset ------------------------------------------------------


def _leader_script() -> list[dict]:
    return [
        {"action": LEADER, "gripper": 1.0, "active": False, "hold": 2},
        {"action": LEADER, "gripper": 0.0, "active": True},
        {"action": LEADER, "gripper": 0.0, "active": True, "events": {"success": True}},
        {"action": LEADER, "gripper": 1.0, "active": False},
        {"action": LEADER, "gripper": 1.0, "active": False, "events": {"failure": True}},
    ]


def _station() -> dict:
    """A single-robot station yaml, as the setup wizard writes one."""
    return {
        "type": "franka",
        "name": ROBOT_NAME,
        "arm_backend": "fake",
        "robot_ip": "172.16.0.2",
        "home_joints": HOME,
        "gripper": {"backend": "fake"},
        "cameras": [
            {"name": cam, "backend": "fake", "width": FRAME, "height": FRAME, "fps": 60} for cam in CAMERAS
        ],
        "teleop": {
            "name": "leader",
            "backend": "fake",
            "options": {
                "action_space": "joint_abs",
                "num_joints": NUM_JOINTS,
                "has_gripper": True,
                "script": _leader_script(),
                "on_end": "loop",
            },
        },
        "daemon": {"bind": "0.0.0.0:50051", "token": TOKEN},
    }


def _hil_preset(station: StationDaemon) -> dict:
    """The tutorial's one-arm ``hil.yaml``, pointed at the test daemon and paced fast."""
    return {
        "env_backend": "real_robot",
        "env_id": "single_arm_smoke",
        "control_hz": FAST_HZ,
        "obs": {"rgb": list(CAMERAS), "image_size": [FRAME, FRAME]},
        "real_robot": {
            "remote": station.address,
            "token": station.token,
            "num_joints": NUM_JOINTS,
            "action_mode": "joint_delta",
            "joint_scale_rad": JOINT_SCALE_RAD,
            "state_fields": ["joint_pos", "gripper_pos"],
            "control_hz": FAST_HZ,
            "max_episode_steps": EPISODE_STEPS,
            "reset_settle_s": 0.0,
        },
    }


@pytest.fixture(scope="module")
def workdir(tmp_path_factory) -> Path:
    return tmp_path_factory.mktemp("single_arm_station")


@pytest.fixture(scope="module")
def station(workdir) -> Iterator[StationDaemon]:
    """The station's robot served by its daemon on a loopback port (the yaml's ``daemon.bind``
    names the robot PC's port; the test binds an ephemeral one)."""
    pytest.importorskip("grpc")
    from real_world_rl.server import daemon

    with pytest.MonkeyPatch.context() as patch:
        patch.setattr(claims, "_DEFAULT_LOCK_DIR", workdir / "claims")
        path = workdir / "station.yaml"
        path.write_text(yaml.safe_dump(_station()), encoding="utf-8")
        loaded = load_station(path)
        assert not loaded.multi_robot and list(loaded.robots) == [ROBOT_NAME]
        robot = loaded.select()
        assert robot.token == TOKEN
        daemon_args = daemon.parse_args(["--station", str(path), "--no-preflight", "--watchdog-s", "30"])
        service, transport = daemon.build_daemon(daemon_args, config=robot.config)
        try:
            transport.start(service, "127.0.0.1:0", token=robot.token)
            yield StationDaemon(service, f"127.0.0.1:{transport.port}", robot.token)
        finally:
            transport.stop(grace_s=0.1)
            service.close()


@pytest.fixture(scope="module")
def preset(station, workdir) -> Path:
    path = workdir / "hil.yaml"
    path.write_text(yaml.safe_dump(_hil_preset(station)), encoding="utf-8")
    return path


# -- command lines ------------------------------------------------------------------------


def _env_flags(preset: Path) -> list[str]:
    """The tutorial's ``ENV_FLAGS`` without ``--teleop-event-device`` (the leader part's own
    scripted buttons stand in for the keyboard)."""
    return [
        "--config", str(preset),
        "--teleop-device", "leader_arm",
        "--robot-teleop-action-mode", "joint_delta",
    ]


def _hil_flags(preset: Path, role: str) -> list[str]:
    return [*_env_flags(preset), "--role", role, "--reward-from-teleop-events", "--buffer-device", "cpu"]


def _training_args(*argv: str) -> Any:
    """What ``real-world-rl-train <argv>`` would run with."""
    training_registry.discover()
    return training_registry.parse_command(list(argv)).args


def _small_encoder() -> EncoderConfig:
    return EncoderConfig(features_dim=16, proprio_latent_dim=8, image_augmentation="random_shift")


def _tiny_rlpd(args):
    return dataclasses.replace(
        args,
        hidden_dim=16,
        discrete_hidden_dim=8,
        n_critics=2,
        buffer_size=16,
        demo_buffer_size=16,
        batch_size=4,
        utd=1,
        encoder=_small_encoder(),
    )


def _tiny_silri(args):
    return dataclasses.replace(
        args,
        feature_dim=8,
        hidden_dim=8,
        behavior_hidden_dims=(8,),
        lambda_hidden_dims=(8,),
        discrete_hidden_dim=8,
        buffer_size=16,
        demo_buffer_size=16,
        batch_size=4,
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


class _StatsSyncClient(FakeSyncClient):
    """``FakeSyncClient`` that also keeps the per-episode stats the actor posts."""

    def __init__(self) -> None:
        super().__init__()
        self.posted: list[dict] = []

    def post_stats(self, stats: dict) -> None:
        self.posted.append(stats)


class _RecordingLogger(Logger):
    """An rl-garden ``Logger`` that keeps every scalar instead of writing it."""

    def __init__(self) -> None:
        super().__init__(log_type="none")
        self.scalars: list[tuple[str, float, int]] = []

    def add_scalar(self, tag, value, step) -> None:
        self.scalars.append((tag, float(value), step))

    def tags(self) -> set[str]:
        return {tag for tag, _, _ in self.scalars}

    def value(self, tag: str) -> tuple[float, int]:
        """The latest ``(value, step)`` logged under ``tag``."""
        matches = [(value, step) for name, value, step in self.scalars if name == tag]
        assert matches, tag
        return matches[-1]


class _ResetCountingProvider:
    """A base action provider that records, at every query, how many resets preceded it."""

    def __init__(self, provider: Any) -> None:
        self.provider = provider
        self.action_dim = provider.action_dim
        self.reset_calls = 0
        self.resets_before_each_query: list[int] = []

    def select_action(self, obs) -> torch.Tensor:
        self.resets_before_each_query.append(self.reset_calls)
        return self.provider.select_action(obs)

    def reset(self) -> None:
        self.reset_calls += 1
        self.provider.reset()

    def to(self, device) -> "_ResetCountingProvider":
        self.provider.to(device)
        return self

    def spec(self) -> dict:
        return self.provider.spec()


def _as_array(action) -> np.ndarray:
    return torch.as_tensor(action).detach().cpu().double().reshape(-1).numpy()


def _assert_operator_action(action, *, saturated: bool) -> None:
    """An intervened action: joint 0 towards the leader (+), joint 1 towards it (-), the other
    joints still (the leader sits on them) and the squeezed trigger closing the gripper."""
    action = _as_array(action)
    assert action.shape == (ACTION_DIM,)
    if saturated:
        np.testing.assert_allclose(action[:NUM_JOINTS], SATURATED_ARM, atol=1e-4)
    assert action[0] > 0.0 and action[1] < 0.0
    np.testing.assert_allclose(action[2:NUM_JOINTS], np.zeros(NUM_JOINTS - 2), atol=1e-4)
    assert action[-1] == CLOSED


def _assert_first_episode_rows(rows: list[dict]) -> None:
    """Rows of one connection's first episode: two policy steps, then the operator's two (the
    trigger squeezed), the second a terminal success. The operator's arm action depends on
    where the policy moved the follower, so it is not checked here."""
    assert [row["intervened"] for row in rows] == FIRST_EPISODE_INTERVENED
    for row in rows:
        assert torch.as_tensor(row["action"]).shape == (1, ACTION_DIM)
        assert set(row["obs"]) == OBS_KEYS
    for row in rows[2:]:
        assert _as_array(row["action"])[-1] == CLOSED
    assert [float(torch.as_tensor(row["reward"]).sum()) for row in rows] == [0.0, 0.0, 0.0, 1.0]
    assert [bool(torch.as_tensor(row["done"]).any()) for row in rows] == [False, False, False, True]


def _check_actor_env(env) -> None:
    assert set(env.single_observation_space.spaces) == OBS_KEYS
    assert env.single_observation_space["state"].shape == (STATE_DIM,)
    for cam in CAMERAS:
        assert env.single_observation_space[f"rgb_{cam}"].shape == (FRAME, FRAME, 3)
    assert env.single_action_space.shape == (ACTION_DIM,)
    contract = get_env_action_contract(env)
    assert contract["action_mode"] == "joint_delta" and contract["num_arms"] == 1
    assert contract["arm_action_dim"] == NUM_JOINTS and contract["gripper_in_action"]
    assert isinstance(env.source, LeaderArmPartSource)


# -- shared artifacts ---------------------------------------------------------------------


@dataclass
class RecordedDemos:
    path: Path
    transitions: list


@pytest.fixture(scope="module")
def recorded_demos(station, preset, workdir) -> RecordedDemos:
    """``real-world-rl-record $ENV_FLAGS --teleop-onset arm --output-path demos/task.pkl
    --num-successes 1``."""
    demo_path = workdir / "demos" / "task.pkl"
    args = parse_record_args(
        [*_env_flags(preset), "--teleop-onset", "arm", "--output-path", str(demo_path), "--num-successes", "1"]
    )
    # The recorder's default: the success key pays the reward and ends the episode.
    assert args.reward_from_teleop_events
    assert run_recorder(args) == demo_path
    assert not station.robot_connected()
    with open(demo_path, "rb") as stream:
        transitions = pickle.load(stream)
    return RecordedDemos(demo_path, transitions)


@pytest.fixture(scope="module")
def bc_checkpoint(recorded_demos, preset, workdir) -> Path:
    """``real-world-rl-train real_world_bc --config hil.yaml --demo-dataset-paths ... --log-dir runs
    --exp-name bc --buffer-size ... --num-offline-steps ... --no-tanh-squash``."""
    log_dir = workdir / "runs"
    args = _training_args(
        "real_world_bc",
        "--config", str(preset),
        "--demo-dataset-paths", str(recorded_demos.path),
        "--log-dir", str(log_dir),
        "--exp-name", "bc",
        "--buffer-size", "16",
        "--num-offline-steps", "3",
        "--no-tanh-squash",
        "--buffer-device", "cpu",
    )
    assert isinstance(args, RealWorldBCArgs) and not args.tanh_squash
    args = dataclasses.replace(args, batch_size=2, device="cpu", log_type="none", encoder=_small_encoder())
    run_real_world_bc(args)
    checkpoint = log_dir / "bc" / "checkpoints" / "final.pt"
    assert checkpoint.exists() and (log_dir / "bc" / "config.json").exists()
    return checkpoint


# -- 1. demo recording --------------------------------------------------------------------


def test_recorded_demos_hold_the_operator_steps_and_a_terminal_success(recorded_demos):
    # Reads 3-4 intervene (the first one arms at once, the leader is 0.08 rad off), read 4 fires
    # success: reward 1, done, and the episode ends.
    transitions = recorded_demos.transitions
    assert len(transitions) == 2
    first, success = transitions
    _assert_operator_action(first["action"], saturated=True)
    _assert_operator_action(success["action"], saturated=False)
    for transition in transitions:
        assert torch.as_tensor(transition["action"]).shape == (1, ACTION_DIM)
        assert set(transition["obs"]) == OBS_KEYS and set(transition["next_obs"]) == OBS_KEYS
        assert transition["obs"]["state"].shape == (1, STATE_DIM)
        assert transition["obs"]["rgb_wrist"].shape == (1, FRAME, FRAME, 3)
    assert float(torch.as_tensor(first["reward"]).sum()) == 0.0
    assert not bool(torch.as_tensor(first["done"]).any())
    assert float(torch.as_tensor(success["reward"]).sum()) == 1.0
    assert bool(torch.as_tensor(success["done"]).all())

    sidecar = read_contract_sidecar(recorded_demos.path)
    assert recorded_demos.path.with_suffix(".contract.json").exists()
    assert sidecar["action_mode"] == "joint_delta" and sidecar["arm_action_dim"] == NUM_JOINTS
    assert sidecar["num_arms"] == 1 and sidecar["gripper_in_action"]
    assert sidecar["joint_scale_rad"] == JOINT_SCALE_RAD
    assert list(sidecar["state_fields"]) == ["joint_pos", "gripper_pos"]


# -- 2 and 3. hil_serl actor and learner --------------------------------------------------


def _run_hil_serl_actor(args, station) -> _StatsSyncClient:
    env = env_building.build_hil_env(args, role="actor")
    sync_client = _StatsSyncClient()
    try:
        _check_actor_env(env)
        HilSerlActorLoop(env, ZeroPolicy(), sync_client, control_hz=args.control_hz).run(total_steps=8)
        assert not station.robot_connected()
    finally:
        env.close()
    return sync_client


def test_hil_serl_actor_rows_and_stats_reach_the_learner_and_its_metrics(station, preset, recorded_demos, tmp_path):
    actor_args = _training_args("hil_serl", *_hil_flags(preset, "actor"))
    assert isinstance(actor_args, HilSerlArgs) and actor_args.reward_from_teleop_events
    sync_client = _run_hil_serl_actor(actor_args, station)
    assert not station.robot_connected()

    rows = sync_client.pushed
    # Reads 1-4: the first episode, ended by the success. Read 6 fires failure: the second
    # episode is truncated after two policy steps. Reads 7-8 start the script again.
    assert len(rows) == 8
    _assert_first_episode_rows(rows[:STEPS_TO_SUCCESS])
    # The zero policy left the follower at home, 0.08 rad from the leader.
    _assert_operator_action(rows[2]["action"], saturated=True)
    _assert_operator_action(rows[3]["action"], saturated=False)
    assert [row["intervened"] for row in rows[STEPS_TO_SUCCESS:]] == [False] * 4
    for row in rows:
        if not row["intervened"]:
            np.testing.assert_array_equal(_as_array(row["action"]), np.zeros(ACTION_DIM))

    first, second = sync_client.posted
    expected_first = {
        "episode": 1, "return": 1.0, "length": 4, "intervened_steps": 2, "intervention_count": 1,
        "intervention_rate": 0.5, "task_success": True, "autonomous_success_rate": 0.0, "human_episode_end": 0.0,
    }
    expected_second = {
        "episode": 2, "return": 0.0, "length": 2, "intervened_steps": 0, "intervention_count": 0,
        "intervention_rate": 0.0, "task_success": False, "autonomous_success_rate": 0.0, "human_episode_end": 0.0,
    }
    assert {key: first[key] for key in expected_first} == expected_first
    assert {key: second[key] for key in expected_second} == expected_second
    for stats in (first, second):
        # The daemon-served robot reports both the round trip and the device clip counter.
        assert stats["rtt_ms_mean"] > 0.0 and stats["rtt_ms_max"] >= stats["rtt_ms_mean"]
        assert stats["safety_clips"] == 0 and stats["safety_clips_total"] == 0
        assert {"step_time_ms_mean", "param_refresh_ms_mean"} <= set(stats)

    learner_args = _tiny_rlpd(
        _training_args(
            "hil_serl",
            *_hil_flags(preset, "learner"),
            "--demo-dataset-paths", str(recorded_demos.path),
            "--log-freq", "1",
        )
    )
    learner_env = env_building.build_hil_env(learner_args, role="learner", enable_teleop=False)
    try:
        from rl_garden.training.online.entrypoints.rlpd_hybrid import build_rlpd_hybrid

        logger = _RecordingLogger()
        assert hil_serl._gripper_kwargs(learner_env) == {}
        agent = build_rlpd_hybrid(
            learner_args, learner_env, None, logger, None, **hil_serl._gripper_kwargs(learner_env)
        )
        agent.init_demo_buffer(learner_args.demo_buffer_size, learner_args.demo_data_ratio)
        learner = HilSerlLearnerLoop(
            agent,
            "127.0.0.1",
            0,
            checkpoint_dir=str(tmp_path / "hil_serl"),
            demo_dataset_paths=learner_args.demo_dataset_paths,
            action_contract=demo_contract_of(learner_env),
        )
        assert len(agent.offline_replay_buffer) == 2
        for row in rows:
            learner._on_transition(row)
        for stats in sync_client.posted:
            learner._server._record_stats(stats)
        for _ in range(2):
            learner._train_iteration(learner.received_transitions)
        learner._log_actor_stats(learner.received_transitions)
    finally:
        learner_env.close()

    assert len(agent.replay_buffer) == 8
    # Both actor interventions are catch-up: they stay online without new expert labels.
    assert all(row.get("syncing", False) for row in rows if row["intervened"])
    assert len(agent.offline_replay_buffer) == len(recorded_demos.transitions)
    tags = logger.tags()
    assert {
        "learner/received",
        "learner/update",
        "learner/buffer_size",
        "learner/train_time_s",
        "learner/demo_buffer_size",
        "learner/intervened_received",
    } <= tags
    assert logger.value("learner/received") == (8.0, 8)
    assert logger.value("learner/demo_buffer_size") == (float(len(recorded_demos.transitions)), 8)
    assert logger.value("learner/intervened_received") == (0.0, 8)
    assert "losses/critic_loss" in tags
    assert [step for tag, _, step in logger.scalars if tag == "learner/train_time_s"] == [8]
    assert ("actor/task_success", 1.0, 8) in logger.scalars and ("actor/task_success", 0.0, 8) in logger.scalars
    assert ("actor/autonomous_success_rate", 0.0, 8) in logger.scalars
    assert ("actor/return", 1.0, 8) in logger.scalars


# -- 4 and 5. residual actor over a BC or an ACT base -------------------------------------


def _run_residual_actor(preset: Path, base_provider: str, station) -> tuple[list[dict], _ResetCountingProvider]:
    """The residual actor's first episode over ``--base-provider base_provider``."""
    args = _tiny_rlpd(
        _training_args(
            "residual_hil_serl",
            *_hil_flags(preset, "actor"),
            "--residual-variant", "hybrid",
            "--base-provider", base_provider,
        )
    )
    residual_entrypoint.validate_residual_args(args)
    env = env_building.build_hil_env(args, role="actor")
    sync_client = FakeSyncClient()
    try:
        agent = residual_entrypoint._scratch_agent(args, env)
        assert isinstance(agent, ResidualRLPDHybrid) and agent.num_grippers == 1
        assert agent.policy.residual_dim == NUM_JOINTS
        provider = _ResetCountingProvider(residual_entrypoint._make_provider(args, env, agent.device))
        ResidualActorLoop(
            env,
            agent.policy,
            sync_client,
            provider,
            warmup_unit_residual=args.residual_warmup_unit,
            base_resync_on_intervention=args.residual_base_resync_on_intervention,
            control_hz=args.control_hz,
            device=agent.device,
            deterministic=args.deterministic_actor,
            seed=args.seed,
        ).run(total_steps=STEPS_TO_SUCCESS)
        assert not station.robot_connected()
    finally:
        env.close()

    rows = sync_client.pushed
    _assert_first_episode_rows(rows)
    for row in rows:
        assert row["base_actions"].shape == (1, ACTION_DIM)
        assert row["next_base_actions"].shape == (1, ACTION_DIM)
        if not row["intervened"]:
            # No learner weights yet: the executed action is the base's.
            torch.testing.assert_close(row["action"].cpu(), row["base_actions"].cpu())
    # Queries: obs of step 1, then next_obs of steps 1-4. Resets: after each intervened step
    # (3 and 4) and at the episode end.
    assert provider.resets_before_each_query == [0, 0, 0, 1, 2]
    assert provider.reset_calls == 3
    return rows, provider


def test_residual_actor_runs_on_the_bc_checkpoint(station, preset, bc_checkpoint):
    _run_residual_actor(preset, f"checkpoint:{bc_checkpoint}?algo=real_world_bc", station)
    assert not station.robot_connected()


def test_residual_actor_runs_on_an_act_checkpoint_trained_from_the_demos(station, preset, recorded_demos, workdir):
    # The vendored ACT model code needs torchvision and IPython.
    pytest.importorskip("torchvision")
    pytest.importorskip("IPython")
    from rl_garden.models.act.train_trajectories import Args as ACTArgs
    from rl_garden.models.act.train_trajectories import train_act_from_trajectories

    act_args = ACTArgs(
        demo_paths=(str(recorded_demos.path),),
        exp_name="act",
        log_dir=str(workdir / "runs"),
        log_type="none",
        cuda=False,
        torch_deterministic=False,
        total_iters=2,
        batch_size=4,
        log_freq=1,
        backbone_weights="none",
        enc_layers=1,
        dec_layers=1,
        dim_feedforward=32,
        hidden_dim=16,
        nheads=2,
        num_queries=4,
        image_size=32,
    )
    checkpoint = train_act_from_trajectories(act_args)
    assert checkpoint == workdir / "runs" / "act" / "checkpoints" / "final.pt"
    assert torch.load(checkpoint, map_location="cpu")["camera_keys"] == sorted(f"rgb_{cam}" for cam in CAMERAS)

    _rows, provider = _run_residual_actor(preset, f"act:{checkpoint}?temporal_agg=1", station)
    assert provider.spec()["kind"] == "act"
    assert not station.robot_connected()


# -- 6. SiLRI actor ------------------------------------------------------------------------


def test_silri_actor_tags_the_step_before_the_takeover(station, preset):
    args = _tiny_silri(_training_args("silri", *_hil_flags(preset, "actor")))
    env = env_building.build_hil_env(args, role="actor")
    sync_client = FakeSyncClient()
    try:
        agent = silri_entrypoint._scratch_agent(args, env)
        assert agent.num_grippers == 1
        loop = SilriActorLoop(
            env,
            agent.policy,
            sync_client,
            control_hz=args.control_hz,
            device=agent.device,
            deterministic=args.deterministic_actor,
            seed=args.seed,
        )
        loop.run(total_steps=STEPS_TO_SUCCESS)
        assert not station.robot_connected()
    finally:
        env.close()
    assert not station.robot_connected()

    rows = sync_client.pushed
    assert loop._held_transition is None
    _assert_first_episode_rows(rows)
    assert [row["intervention_onset"] for row in rows] == [False, True, False, False]
    for row in rows:
        assert torch.as_tensor(row["policy_action"]).shape == (1, ACTION_DIM)


# -- 7. deploy -----------------------------------------------------------------------------


def test_deploy_runs_the_bc_checkpoint_on_the_daemon(station, bc_checkpoint):
    # The deploy command line gives the connection only; the run's config.json the contract.
    args = DeployArgs(
        checkpoint=str(bc_checkpoint),
        algo="real_world_bc",
        real_robot=RealRobotEnvConfig(remote=station.address, token=station.token, reset_settle_s=0.0),
        episodes=1,
        max_steps_per_episode=2,
        device="cpu",
    )
    restored = restore_training_args(args)
    assert isinstance(restored, RealWorldBCArgs) and restored.device == "cpu"
    trained = restored.real_robot
    assert trained.action_mode == "joint_delta" and trained.max_episode_steps == EPISODE_STEPS
    assert tuple(trained.state_fields) == ("joint_pos", "gripper_pos")

    (result,) = run_deploy(args)
    assert result.steps == 2 and result.truncated and result.health_ok is True
    assert result.rtt_ms_mean is not None
    assert not station.robot_connected()


# -- 8. classifier data collection --------------------------------------------------------


def _saved_frames(directory: Path, label: str) -> list:
    (path,) = directory.glob(f"real_robot_*{label}_images_*.pkl")
    with open(path, "rb") as stream:
        return pickle.load(stream)


def test_classifier_collector_latches_success_until_failure_or_episode_end(station, preset, workdir):
    output_dir = workdir / "classifier"
    collect_data.main(
        argv=[
            "--env-backend", "real_robot",
            *_env_flags(preset),
            "--output-dir", str(output_dir),
            "--successes-needed", "3",
        ]
    )
    assert not station.robot_connected()

    # Episode 1 (six reads, capped): F F F S(s) S(latched) F(f). Episode 2: F F F S(s) -> the
    # third success frame stops the collection.
    successes = _saved_frames(output_dir, "success")
    failures = _saved_frames(output_dir, "failure")
    assert (len(successes), len(failures)) == (3, 7)
    for entry in successes + failures:
        assert set(entry["obs"]) == {f"rgb_{cam}" for cam in CAMERAS}
        assert tuple(entry["obs"]["rgb_wrist"].shape) == (FRAME, FRAME, 3)
