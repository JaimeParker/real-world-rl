"""The training entrypoints on the two-arm backend: the actor of hil_serl, residual_hil_serl and
silri builds the bimanual teleop source and a two-gripper agent (a 9-way gripper head), the
learner records the two-arm contract and the gripper count, single-arm teleop devices are
refused, real_world_bc trains on a two-arm demo and deploy restores both a real_world_bc and a
two-gripper rlpd_hybrid checkpoint. The loops are monkeypatched; the backend returns a
``BimanualRealRobotVectorEnv`` over two local fake Frankas with fake leader parts."""
from __future__ import annotations

import json

import pytest
import real_world_rl.devices.claims as claims
import torch
from _bimanual_fakes import ARM_STATE_DIM, ARMS, bimanual_config, fake_franka
from rl_garden.common.checkpoint import load_checkpoint_file
from rl_garden.observations import ObservationConfig
from real_world_rl.devices.teleop import TeleopConfig

import real_world_rl.envs.backends.real_robot_bimanual  # noqa: F401  (registers the backend)
from real_world_rl.action_contract import demo_contract_of
from real_world_rl.demos.record import save_demos
from real_world_rl.deploy import (
    DeployArgs,
    _gripper_count_kwargs,
    _resolve_build_agent,
    build_deploy_env,
    restore_policy,
    restore_training_args,
    run_deploy,
)
from real_world_rl.envs.bimanual import (
    BimanualRealRobotEnv,
    BimanualRealRobotVectorEnv,
)
from real_world_rl.policies.base_provider import ZeroBaseProvider
from real_world_rl.teleop.core.bimanual import BimanualTeleopSource
from real_world_rl.teleop.leader.joint import LeaderArmPartSource
from real_world_rl.training.entrypoints.bc import (
    RealWorldBCArgs,
    build_real_world_bc,
    run_real_world_bc,
)
from real_world_rl.training.entrypoints.hil_serl import HilSerlArgs, run_hil_serl
from real_world_rl.training.entrypoints.residual_hil_serl import (
    ResidualHilSerlArgs,
    run_residual_hil_serl,
)
from real_world_rl.training.entrypoints.silri import SilriArgs, run_silri

BIMANUAL = "real_robot_bimanual"
NUM_JOINTS = 7
ACTION_DIM = 2 * NUM_JOINTS + 2
STATE_DIM = 2 * ARM_STATE_DIM
LEADER_OPTIONS = {"action_space": "joint_abs", "num_joints": NUM_JOINTS, "script": [{"action": [0.0] * NUM_JOINTS}]}

_RLPD_SIZES = dict(
    hidden_dim=8, discrete_hidden_dim=8, n_critics=2, buffer_size=64, buffer_device="cpu", demo_buffer_size=16
)
_SILRI_SIZES = dict(
    buffer_size=64,
    buffer_device="cpu",
    demo_buffer_size=16,
    feature_dim=8,
    hidden_dim=8,
    behavior_hidden_dims=(8,),
    lambda_hidden_dims=(8,),
    discrete_hidden_dim=8,
)
# method -> (run function, Args factory, module holding its loops)
_METHODS = {
    "hil_serl": (run_hil_serl, lambda **kw: HilSerlArgs(**_RLPD_SIZES, **kw), "real_world_rl.loops.hil_serl"),
    "residual_hil_serl": (
        run_residual_hil_serl,
        lambda **kw: ResidualHilSerlArgs(**_RLPD_SIZES, residual_variant="hybrid", base_provider="zero", **kw),
        "real_world_rl.loops.residual",
    ),
    "silri": (run_silri, lambda **kw: SilriArgs(**_SILRI_SIZES, **kw), "real_world_rl.loops.silri"),
}
_ACTOR_LOOPS = {"hil_serl": "HilSerlActorLoop", "residual_hil_serl": "ResidualActorLoop", "silri": "SilriActorLoop"}
_LEARNER_LOOPS = {
    "hil_serl": "HilSerlLearnerLoop",
    "residual_hil_serl": "ResidualLearnerLoop",
    "silri": "SilriLearnerLoop",
}
METHODS = list(_METHODS)


def _config():
    return bimanual_config(shared={"cameras": [], "action_mode": "joint_delta", "control_hz": 1000.0})


def _fields(**overrides) -> dict:
    fields = dict(
        env_backend=BIMANUAL,
        real_robot_bimanual=_config(),
        obs=ObservationConfig(state=True),
        teleop_device="bimanual",
        teleop_arm_device="leader_arm",
        sync_host="10.0.0.1",
        sync_port=7000,
    )
    fields.update(overrides)
    return fields


@pytest.fixture
def bimanual_backend(tmp_path, monkeypatch):
    """``make_training_envs`` for the two-arm backend: a fresh vector env over two fake Frankas,
    each with a fake ``joint_abs`` leader part, per call (closed at teardown)."""
    monkeypatch.setattr(claims, "_DEFAULT_LOCK_DIR", tmp_path / "claims")
    built = []

    def make_training_envs(backend, request):
        assert backend == BIMANUAL
        robots = {
            arm: fake_franka(arm, teleop=TeleopConfig(name="leader", backend="fake", options=dict(LEADER_OPTIONS)))
            for arm in ARMS
        }
        env = BimanualRealRobotVectorEnv(
            BimanualRealRobotEnv(request.backend_config, robots=robots), include_state=request.observation.state
        )
        built.append(env)
        return env, None

    monkeypatch.setattr("rl_garden.envs.backend_registry.make_training_envs", make_training_envs)
    yield built
    for env in built:
        env.close()


def _gripper_head_classes(policy) -> int:
    """Classes of the joint gripper head: the discrete critic of the hybrid policies, the
    classification head of SiLRI's."""
    head = policy.discrete_head if hasattr(policy, "discrete_head") else policy.discrete_critic
    return head.n_actions


# -- actor --------------------------------------------------------------------------------


@pytest.mark.parametrize("method", METHODS)
def test_actor_builds_the_bimanual_source_and_a_two_gripper_agent(method, bimanual_backend, monkeypatch):
    run, make_args, package = _METHODS[method]
    captured = {}

    class _FakeActorLoop:
        def __init__(self, env, policy, sync_client, *args, **kwargs):
            captured.update(env=env, policy=policy, sync_client=sync_client, args=args)

        def run(self):
            captured["ran"] = True

    monkeypatch.setattr(f"{package}.{_ACTOR_LOOPS[method]}", _FakeActorLoop)
    run(make_args(role="actor", **_fields()))

    env = captured["env"]
    try:
        assert captured["ran"] is True
        assert captured["sync_client"]._base_url == "http://10.0.0.1:7000"
        source = env.source
        assert isinstance(source, BimanualTeleopSource) and source.arm_names == list(ARMS)
        assert all(isinstance(child, LeaderArmPartSource) for child in source.sources.values())
        assert env.single_action_space.shape == (ACTION_DIM,)
        assert _gripper_head_classes(captured["policy"]) == 9
        if method == "residual_hil_serl":
            (provider,) = captured["args"]
            assert isinstance(provider, ZeroBaseProvider) and provider.action_dim == ACTION_DIM
            assert captured["policy"].residual_dim == 2 * NUM_JOINTS
    finally:
        env.close()


@pytest.mark.parametrize("method", METHODS)
def test_a_single_arm_teleop_device_is_refused_on_the_bimanual_backend(method, bimanual_backend):
    run, make_args, _ = _METHODS[method]
    with pytest.raises(ValueError, match="needs a bimanual teleop source: use --teleop-device bimanual"):
        run(make_args(role="actor", **_fields(teleop_device="leader_arm")))


# -- learner ------------------------------------------------------------------------------


@pytest.mark.parametrize("method", METHODS)
def test_learner_records_the_two_arm_contract_and_the_gripper_count(method, bimanual_backend, monkeypatch, tmp_path):
    run, make_args, package = _METHODS[method]
    captured = {}

    class _FakeLearnerLoop:
        def __init__(self, agent, host, port, **kwargs):
            captured.update(agent=agent, kwargs=kwargs)

        def run(self):
            captured["ran"] = True

    monkeypatch.setattr(f"{package}.{_LEARNER_LOOPS[method]}", _FakeLearnerLoop)
    run(make_args(role="learner", **_fields(), log_dir=str(tmp_path), log_type="none", exp_name="run"))

    assert captured["ran"] is True
    agent = captured["agent"]
    assert agent.num_grippers == 2
    assert agent.offline_replay_buffer is not None
    contract = captured["kwargs"]["action_contract"]
    assert contract["num_arms"] == 2 and contract["arm_names"] == list(ARMS)

    config = json.loads((tmp_path / "run" / "config.json").read_text(encoding="utf-8"))
    assert config["args"]["env_backend"] == BIMANUAL
    assert config["args"]["real_robot_bimanual"]["arm_names"] == list(ARMS)
    checkpoint = agent.save(tmp_path / "run" / "checkpoints" / "checkpoint_0.pt")
    metadata = load_checkpoint_file(checkpoint, map_location="cpu")["metadata"]
    assert metadata["hyperparameters"]["num_grippers"] == 2


# -- real_world_bc and deploy --------------------------------------------------------------


def _write_two_arm_demo(path) -> None:
    contract_env = BimanualRealRobotEnv(_config(), robots={arm: fake_franka(arm) for arm in ARMS})
    transitions = [
        {
            "obs": {"state": torch.randn(1, STATE_DIM)},
            "next_obs": {"state": torch.randn(1, STATE_DIM)},
            "action": torch.rand(1, ACTION_DIM) * 2 - 1,
            "reward": torch.ones(1),
            "done": torch.zeros(1),
        }
        for _ in range(6)
    ]
    save_demos(transitions, path, demo_contract_of(contract_env))


def test_real_world_bc_trains_on_a_two_arm_demo_and_deploy_restores_it(bimanual_backend, tmp_path):
    demo_path = tmp_path / "demos.pkl"
    _write_two_arm_demo(demo_path)
    log_dir = tmp_path / "runs"
    run_real_world_bc(
        RealWorldBCArgs(
            env_backend=BIMANUAL,
            real_robot_bimanual=_config(),
            obs=ObservationConfig(state=True),
            demo_dataset_paths=(str(demo_path),),
            num_offline_steps=2,
            batch_size=4,
            buffer_size=16,
            buffer_device="cpu",
            device="cpu",
            tanh_squash=False,
            log_dir=str(log_dir),
            log_type="none",
            exp_name="bc",
            checkpoint_freq=0,
            save_final_checkpoint=True,
        )
    )
    checkpoint = log_dir / "bc" / "checkpoints" / "final.pt"
    assert checkpoint.exists()

    args = DeployArgs(
        checkpoint=str(checkpoint),
        algo="real_world_bc",
        env_backend=BIMANUAL,
        real_robot_bimanual=_config(),
        episodes=1,
        max_steps_per_episode=2,
    )
    assert _resolve_build_agent("real_world_bc") is build_real_world_bc
    restored = restore_training_args(args)
    assert type(restored) is RealWorldBCArgs and restored.device == "cpu"
    assert restored.real_robot_bimanual.left.action_mode == "joint_delta"
    (result,) = run_deploy(args)
    assert result.steps == 2 and result.truncated


def test_deploy_restores_a_two_gripper_rlpd_hybrid_checkpoint(bimanual_backend, tmp_path):
    from rl_garden.training.online.entrypoints.rlpd_hybrid import (
        RLPDHybridArgs,
        build_rlpd_hybrid,
    )

    checkpoint = tmp_path / "run" / "checkpoints" / "checkpoint_0.pt"
    args = DeployArgs(
        checkpoint=str(checkpoint),
        algo="rlpd_hybrid",
        env_backend=BIMANUAL,
        real_robot_bimanual=_config(),
        episodes=1,
        max_steps_per_episode=2,
    )
    training_args = RLPDHybridArgs(hidden_dim=8, discrete_hidden_dim=8, n_critics=2, buffer_size=8, buffer_device="cpu")
    training_args.real_robot_bimanual = _config()
    env = build_deploy_env(args, training_args)
    saved = build_rlpd_hybrid(training_args, env, None, None, None, num_grippers=2)
    saved.save(checkpoint)

    policy = restore_policy(args, training_args, env)
    assert policy.num_grippers == 2 and policy.discrete_critic.n_actions == 9
    expected = saved.policy.state_dict()
    for key, value in policy.state_dict().items():
        torch.testing.assert_close(value.cpu(), expected[key].cpu())
    (result,) = run_deploy(args, env=env, policy=policy)
    assert result.steps == 2


def test_only_rlpd_hybrid_takes_the_gripper_count_and_only_past_one_gripper():
    from _teleop_fakes import ContractEnv

    two_arms = BimanualRealRobotEnv(_config(), robots={arm: fake_franka(arm) for arm in ARMS})
    assert _gripper_count_kwargs("rlpd_hybrid", two_arms) == {"num_grippers": 2}
    assert _gripper_count_kwargs("rlpd_hybrid", ContractEnv()) == {}
    assert _gripper_count_kwargs("rlpd", two_arms) == {}
    assert _gripper_count_kwargs("residual_hil_serl_hybrid", two_arms) == {}


# -- train_real_world.py command line ------------------------------------------------------


def test_the_training_cli_reaches_the_two_arm_backend_and_teleop_flags():
    from real_world_rl.training import env_building
    from real_world_rl.training.registry import registry

    command = registry.parse_command(
        [
            "hil_serl",
            "--role", "actor",
            "--env_backend", BIMANUAL,
            "--real_robot_bimanual.left.remote", "10.0.0.2:50051",
            "--real_robot_bimanual.right.remote", "10.0.0.3:50052",
            "--real_robot_bimanual.left.action_mode", "joint_delta",
            "--real_robot_bimanual.right.action_mode", "joint_delta",
            "--teleop-device", "bimanual",
            "--teleop-arm-device", "leader_arm",
            "--robot-teleop-action-mode", "joint_abs",
        ]
    )
    args = command.args
    assert isinstance(args, HilSerlArgs) and command.algorithm == "hil_serl"
    assert args.env_backend == BIMANUAL and args.resolve_backend_config() is args.real_robot_bimanual
    assert args.real_robot_bimanual.right.remote == "10.0.0.3:50052"
    assert args.real_robot_bimanual.left.action_mode == "joint_delta"
    kwargs = env_building.teleop_wrapper_kwargs(args)
    assert kwargs["device"] == "bimanual" and kwargs["device_per_arm"] == "leader_arm"
    assert kwargs["common_kwargs"]["teleop_contract"] == ("joint_abs", NUM_JOINTS, True)


@pytest.fixture(autouse=True)
def _isolate_actor_outbox_directory(monkeypatch, tmp_path):
    monkeypatch.chdir(tmp_path)
