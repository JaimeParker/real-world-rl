"""Hardware-free task/reset, source arbitration and observation contract regressions."""
from __future__ import annotations
import dataclasses
import types
import numpy as np
import pytest
import torch
import gymnasium as gym
from gymnasium import spaces
from _bimanual_fakes import bimanual_config, fake_franka
import robot_infra.devices.claims as claims
from real_world_rl.tasks import TaskConfig, ResetConfig
from real_world_rl.tasks.outcome import TaskOutcomeWrapper
from real_world_rl.envs.real_robot import RealRobotEnv, RealRobotVectorEnv, RealRobotEnvConfig
from real_world_rl.envs.bimanual import BimanualRealRobotEnv
from real_world_rl.envs.real_robot.preprocess import CameraPreprocessConfig, preprocess_frame
from real_world_rl.action_contract import (demo_contract_of, validate_transitions_against_contract,
    experiment_contract_of, experiment_hash, validate_experiment)
from real_world_rl.envs.wrappers.experiment import apply_experiment_contract


@pytest.fixture(autouse=True)
def isolated_claims(tmp_path, monkeypatch):
    monkeypatch.setattr(claims, "_DEFAULT_LOCK_DIR", tmp_path / "claims")


def single(task=None, **kwargs):
    robot = fake_franka("single")
    config = RealRobotEnvConfig(robot={"type": "franka", "arm_backend": "fake"},
        cameras=[], home_joints=[0.0]*7, reset_settle_s=0, control_hz=10000,
        task=task or TaskConfig(), **kwargs)
    return RealRobotEnv(config, robot=robot)


@pytest.mark.parametrize("field,value", [
    ("control_hz", float("nan")), ("control_hz", float("inf")),
    ("xyz_scale_m", float("nan")), ("rot_scale_rad", float("inf")),
    ("joint_scale_rad", -0.1), ("reset_settle_s", float("nan")),
    ("safety_box", [0, 0, 0, float("nan"), 1, 1]),
    ("safety_box", [1, 0, 0, 0, 1, 1]),
    ("joint_limits", [[-1, float("inf")]] * 7),
    ("home_joints", [float("nan")] * 7),
    ("max_episode_steps", float("inf")), ("image_size", (8.5, 8)),
])
def test_invalid_control_and_geometry_rejected_before_construction(field, value):
    with pytest.raises(ValueError):
        RealRobotEnvConfig(**{field: value})


@pytest.mark.parametrize("invalid", [float("nan"), float("inf"), -float("inf")])
def test_nonfinite_single_action_rejected_before_backend_command(monkeypatch, invalid):
    env = single()
    try:
        env.reset()
        commands = []
        monkeypatch.setattr(env, "_send_and_observe", lambda action: commands.append(action))
        action = np.zeros(env.action_space.shape)
        action[-1] = invalid
        with pytest.raises(ValueError, match="finite"):
            env.step(action)
        assert commands == []
    finally:
        env.close()


def test_invalid_right_arm_action_rejected_before_either_arm_command(monkeypatch):
    config = bimanual_config(shared={"control_hz": 10000})
    env = BimanualRealRobotEnv(config, robots={name: fake_franka(name) for name in config.arm_names})
    try:
        env.reset()
        commands = []
        for name, arm in env.arms.items():
            monkeypatch.setattr(arm, "step", lambda action, name=name: commands.append(name))
        action = np.zeros(env.action_space.shape)
        action[-1] = float("nan")
        with pytest.raises(ValueError, match="finite"):
            env.step(action)
        assert commands == []
    finally:
        env.close()


def test_invalid_right_station_boundaries_prevent_both_connections(monkeypatch):
    from robot_infra.devices import Robot
    config = bimanual_config()
    config.right.robot["safety_box"] = [0, 0, 0, float("nan"), 1, 1]
    connections = []
    monkeypatch.setattr(Robot, "connect", lambda self: connections.append(self))
    with pytest.raises(ValueError, match="finite"):
        BimanualRealRobotEnv(config)
    assert connections == []


def test_wizard_profile_reaches_real_fake_backend_reset():
    import yaml
    from robot_infra.devices.robots.franka import FrankaConfig
    from robot_infra.devices import GripperConfig
    from robot_infra.setup.command import build_parser, answers_from_args
    from robot_infra.setup.probes import mock_station_probe
    from robot_infra.setup.wizard import ArmDraft, StationDraft, run_wizard
    from robot_infra.setup.render import render_station_yaml
    from robot_infra.discovery.autoconfig import parse_station
    prior = StationDraft([ArmDraft("arm", FrankaConfig(name="arm", arm_backend="fake",
                         gripper=GripperConfig(backend="fake"), claim=False, cameras=[], teleop=None))])
    parsed = build_parser().parse_args(["--non-interactive", "--only", "daemon",
                                       "--control", "{max_delta_tau: 0.4}"])
    def no_prompt(text):
        pytest.fail(text)
    draft = run_wizard(mock_station_probe(), answers_from_args(parsed, prior), no_prompt, lambda _: None)
    station = parse_station(yaml.safe_load(render_station_yaml(draft)))
    env = RealRobotEnv(RealRobotEnvConfig(robot=station.select().config.to_dict(), reset_settle_s=0,
                       control={"max_delta_tau": .2}, task=TaskConfig(name="pick_and_place")))
    try:
        _, info = env.reset(seed=5)
        assert info["control_metadata"]["effective"]["max_delta_tau"] == .2
        assert env.robot.is_connected
        assert env.robot.arm.control_config["max_delta_tau"] == .2
    finally:
        env.close()
    assert not env.robot.is_connected


def test_registered_task_and_version_rejected_before_connection():
    for config in (TaskConfig(name="missing"),):
        with pytest.raises(ValueError, match="unknown task"):
            single(config)
    with pytest.raises(ValueError, match="version"):
        TaskConfig(version=2)


def test_reset_random_sequence_reproducible_without_reseeding(monkeypatch):
    task = TaskConfig(name="pick_and_place", reset=ResetConfig(
        waypoints=[[0.4, 0, 0.3, 0, 0, 0, 1]], xyz_jitter_m=[0.02, 0.02, 0]))
    sequences = []
    for _ in range(2):
        env = single(task)
        positions = []
        original = env.robot.arm.move_to_pose
        def move(pose, **kwargs):
            positions.append(pose.copy())
            return original(pose, **kwargs)
        monkeypatch.setattr(env.robot.arm, "move_to_pose", move)
        try:
            env.reset(seed=7); env.reset(); env.reset()
            sequences.append(positions)
        finally: env.close()
    np.testing.assert_allclose(sequences[0], sequences[1])
    assert not np.array_equal(sequences[0][0], sequences[0][1])


def test_invalid_later_waypoint_prevents_all_reset_motion(monkeypatch):
    task = TaskConfig(name="pick_and_place", reset=ResetConfig(
        waypoints=[[0.4, 0, 0.3, 0, 0, 0, 1], [5, 0, 0.3, 0, 0, 0, 1]]))
    env = single(task, safety_box=[0,-1,0,1,1,1])
    moves = []
    monkeypatch.setattr(env.robot.arm, "move_to_joints", lambda *a, **k: moves.append("home"))
    monkeypatch.setattr(env.robot.arm, "move_to_pose", lambda *a, **k: moves.append("pose"))
    try:
        with pytest.raises(ValueError, match="outside safety_box"): env.reset(seed=1)
        assert moves == []
    finally: env.close()


def test_bimanual_prepare_failure_stops_both_before_home(monkeypatch):
    config = bimanual_config(task=TaskConfig(name="parallel_pick_and_place"))
    env = BimanualRealRobotEnv(config, robots={name: fake_franka(name) for name in config.arm_names})
    stops, moves = [], []
    for name, arm in env.arms.items():
        monkeypatch.setattr(arm.robot.arm, "stop", lambda name=name: stops.append(name))
        monkeypatch.setattr(arm.robot.arm, "move_to_joints", lambda *a, **k: moves.append(True))
    def fail(): raise RuntimeError("prepare failed")
    monkeypatch.setattr(env.arms["right"], "_reset_begin", fail)
    try:
        with pytest.raises(RuntimeError, match="prepare failed"): env.reset()
        assert set(stops) == {"left", "right"}
        assert moves == []
    finally: env.close()


class LabelEnv(gym.Env):
    def __init__(self, infos):
        self.infos = iter(infos)
        self.observation_space = spaces.Dict({"state": spaces.Box(-1, 1, (1,), dtype=np.float32)})
        self.action_space = spaces.Box(-1, 1, (1,), dtype=np.float32)
    def reset(self, **kwargs): return {"state": torch.zeros(1,1)}, {}
    def step(self, action):
        return {"state": torch.zeros(1,1)}, torch.zeros(1), torch.zeros(1,dtype=torch.bool), torch.zeros(1,dtype=torch.bool), next(self.infos)


def test_parallel_manual_labels_require_both_arms_and_failure_veto():
    env = TaskOutcomeWrapper(LabelEnv([
        {"teleop_arm_events": {"left": {"success": True}}, "teleop_events": {"success": True}},
        {"teleop_arm_events": {"right": {"failure": True}}, "teleop_events": {"success": True}},
    ]), TaskConfig(name="parallel_pick_and_place"), ("left", "right"))
    env.reset()
    _, _, terminated, _, info = env.step(None)
    assert info["task_success"] is None and not bool(terminated)
    from real_world_rl.metrics import EpisodeOutcomeAccumulator
    accumulated = EpisodeOutcomeAccumulator()
    accumulated.observe(info)
    assert accumulated.finish(truncated=True).task_success is None
    _, _, _, truncated, info = env.step(None)
    assert info["task_success"] is False and bool(truncated)


def test_classifier_hold_two_real_wrapper_stack_and_dense_before_label():
    from real_world_rl.reward.outcome import ExplicitRewardClassifierWrapper
    task = TaskConfig(name="pick_and_place", success_source="classifier", success_hold_steps=2,
        targets={"arm": [0,0,0]}, dense_distance_weight=2)
    base = LabelEnv([{"task_tcp_positions": {"arm": [1,0,0]}}]*2)
    classified = ExplicitRewardClassifierWrapper(base, lambda obs: torch.ones(1), terminate_on_success=False)
    env = TaskOutcomeWrapper(classified, task, ("arm",))
    env.reset()
    _, reward, terminated, _, info = env.step(None)
    assert not bool(terminated) and float(reward) == -2
    from real_world_rl.metrics import EpisodeOutcomeAccumulator
    accumulated = EpisodeOutcomeAccumulator()
    accumulated.observe({**info, "teleop_events": {"success": True}})
    assert accumulated.finish(truncated=True).task_success is False
    _, reward, terminated, _, info = env.step(None)
    assert bool(terminated) and float(reward) == 1 and info["task_success_source"] == "classifier"
    manual = TaskOutcomeWrapper(LabelEnv([{"task_tcp_positions":{"arm":[1,0,0]}}]),
        dataclasses.replace(task, success_source="manual"), ("arm",))
    manual.reset()
    assert float(manual.step(None)[1]) == -2


def test_rgb_depth_crop_geometry_interpolation_units_and_invalid_zero():
    rgb = np.arange(4*6*3,dtype=np.uint8).reshape(4,6,3)
    depth = np.arange(24,dtype=np.float32).reshape(4,6)/10
    depth[1,2] = np.nan
    config = CameraPreprocessConfig(roi=[1,2,2,3], source_resolution=[4,6], depth_clip_m=[0.2,1.5])
    actual_rgb = preprocess_frame(rgb,config,(4,6))
    actual_depth = preprocess_frame(depth,config,(4,6),depth=True)
    from PIL import Image
    expected_rgb = np.asarray(Image.fromarray(rgb[1:3,2:5]).resize((6,4),Image.Resampling.BILINEAR))
    np.testing.assert_array_equal(actual_rgb,expected_rgb)
    assert actual_depth.shape == (4,6,1) and actual_depth.dtype == np.float32
    assert actual_depth[0,0,0] == 0 and actual_depth.max() <= 1.5
    np.testing.assert_allclose(np.unique(actual_depth), [0,0.9,1.0,1.4,1.5])
    with pytest.raises(ValueError,match="resolution"):
        preprocess_frame(rgb[:2],config,(4,6))


def test_contract_hash_roundtrip_and_legacy_novel_features_rejected():
    env = apply_experiment_contract(RealRobotVectorEnv(single(TaskConfig(name="pick_and_place"))))
    try:
        contract = experiment_contract_of(env)
        validate_experiment(env, contract)
        assert len(experiment_hash(env)) == 64
        changed = {**contract,"experiment":{**contract["experiment"],"control":{"damping_ratio":0.7}}}
        with pytest.raises(ValueError,match="differs"): validate_experiment(env, changed)
        expected = demo_contract_of(env)
        legacy = {key:value for key,value in expected.items() if key not in ("experiment","effective_control","contract_version")}
        with pytest.raises(ValueError,match="legacy metadata"):
            validate_transitions_against_contract([],expected,source="legacy.pkl",saved=legacy)
    finally: env.close()


def test_frequency_and_classifier_content_change_hash(tmp_path):
    env = apply_experiment_contract(RealRobotVectorEnv(single()))
    try:
        before = experiment_hash(env)
        env.env.env.config.control_hz = 5
        assert experiment_hash(env) != before
    finally: env.close()
    from real_world_rl.training.env_building import reward_sources_spec
    from real_world_rl.training.entrypoints.hil_serl import HilSerlArgs
    file = tmp_path / "classifier.pt"
    file.write_bytes(b"first")
    args = HilSerlArgs(classifier_checkpoint=str(file))
    before = reward_sources_spec(args)["classifier"]["content_sha256"]
    file.write_bytes(b"second")
    assert reward_sources_spec(args)["classifier"]["content_sha256"] != before


def test_train_record_deploy_rgb_depth_contract_roundtrip_and_early_rejection(tmp_path):
    from rl_garden.observations import ObservationConfig
    from real_world_rl.training.entrypoints.hil_serl import HilSerlArgs
    from real_world_rl.training.env_building import build_hil_env
    from real_world_rl.deploy import DeployArgs, build_deploy_env
    from real_world_rl.action_contract import save_experiment_metadata, read_contract_sidecar
    from real_world_rl.demos.record import save_demos
    robot = {"type":"franka", "arm_backend":"fake", "claim":False,
             "gripper":{"backend":"fake"}, "cameras":[{"name":"wrist","backend":"fake",
             "height":24,"width":32,"fps":120,"enable_depth":True}]}
    config = RealRobotEnvConfig(robot=robot,cameras=["wrist"],depth_cameras=["wrist"],
        image_size=(8,8),home_joints=[0.0]*7,reset_settle_s=0,
        preprocess={"wrist":CameraPreprocessConfig(roi=[2,2,16,16],source_resolution=[24,32],depth_clip_m=[0.1,2])},
        task=TaskConfig(name="pick_and_place",success_source="tcp_proxy",targets={"arm":[0,0,0]}))
    args = HilSerlArgs(env_backend="real_robot",real_robot=config,teleop_device="fake",
        obs=ObservationConfig(state=True,rgb=("wrist",),depth=("wrist",),image_size=(8,8),frame_stack=2))
    trained = build_hil_env(args,role="eval")
    try:
        obs,_ = trained.reset(seed=3)
        contract = demo_contract_of(trained)
        save_demos([],tmp_path/"demos.pkl",contract)
        assert read_contract_sidecar(tmp_path/"demos.pkl")["experiment"] == contract["experiment"]
        save_experiment_metadata(trained,tmp_path/"experiment.json")
        first = {key:value.clone() for key,value in obs.items()}
    finally: trained.close()
    deployed = build_deploy_env(DeployArgs(checkpoint=str(tmp_path/"agent.pt"),real_robot=config),args)
    try:
        second,_ = deployed.reset(seed=3)
        assert experiment_contract_of(deployed) == experiment_contract_of(trained)
        for key in first: torch.testing.assert_close(first[key],second[key])
        assert second["depth_wrist"].dtype == torch.float32
        assert second["depth_wrist"].shape == (1,2,8,8,1)
    finally: deployed.close()
    incompatible = dataclasses.replace(config,control_hz=20)
    with pytest.raises(ValueError,match="contradicts|differs"):
        build_deploy_env(DeployArgs(checkpoint=str(tmp_path/"agent.pt"),real_robot=incompatible),args)


def test_effective_control_mismatch_rejected_before_home(monkeypatch):
    from real_world_rl.action_contract import effective_experiment_of
    env = apply_experiment_contract(RealRobotVectorEnv(single()))
    try:
        env.reset()
        saved = effective_experiment_of(env)
        saved["runtime_control"]["arm"]["sdk_version"] = "different-sdk"
        validate_experiment(env,saved)
        raw = env.env.env
        moves = []
        monkeypatch.setattr(raw.robot.arm,"move_to_joints",lambda *a,**k:moves.append(True))
        with pytest.raises(ValueError,match="control/SDK"): env.reset()
        assert moves == []
    finally: env.close()


def test_tcp_proxy_is_not_object_task_success_in_real_metric_chain():
    from real_world_rl.metrics import EpisodeOutcomeAccumulator, EpisodeMetrics
    env = TaskOutcomeWrapper(LabelEnv([{"task_tcp_positions":{"arm":[0,0,0]}}]),
        TaskConfig(name="pick_and_place",success_source="tcp_proxy",targets={"arm":[0,0,0]}),("arm",))
    env.reset()
    _,_,terminated,_,info = env.step(None)
    assert bool(terminated) and info["tcp_proxy_success"] and info["task_success"] is None
    episode = EpisodeOutcomeAccumulator()
    episode.observe({**info,"success":True,"teleop_events":{"success":True}})
    metrics = EpisodeMetrics(); metrics.add(episode.finish(terminated=terminated))
    assert metrics.summary()["tcp_proxy_success_rate"] == 1
    assert metrics.summary()["task_success_rate"] == metrics.summary()["autonomous_success_rate"] == 0


def test_bimanual_depth_only_camera_name_collisions_are_rejected():
    from real_world_rl.envs.bimanual.config import BimanualRealRobotEnvConfig
    left = RealRobotEnvConfig(robot={"type":"franka"},cameras=[],depth_cameras=["depth_only"])
    right = RealRobotEnvConfig(robot={"type":"franka"},cameras=["depth_only"])
    with pytest.raises(ValueError,match="unique"):
        BimanualRealRobotEnvConfig(left=left,right=right)


@pytest.mark.parametrize("task_active",[False,True])
def test_reward_components_reconstruct_manual_and_classifier_stack(task_active):
    from real_world_rl.reward.outcome import ExplicitRewardClassifierWrapper
    from real_world_rl.envs.wrappers.teleop_event_reward import TeleopEventRewardWrapper
    base = LabelEnv([{"teleop_events":{"success":True},"grasp_penalty":torch.tensor([-0.2])}])
    env = ExplicitRewardClassifierWrapper(base,lambda obs:torch.ones(1),terminate_on_success=not task_active)
    env = TeleopEventRewardWrapper(env,success_reward=2,terminate_on_success=not task_active)
    if task_active:
        env = TaskOutcomeWrapper(env,TaskConfig(name="pick_and_place",success_source="classifier"),("arm",))
    env = apply_experiment_contract(env)
    env.reset()
    _,reward,_,_,info = env.step(None)
    assert set(info["reward_components"]) == {"base","task","manual","intervention"}
    torch.testing.assert_close(sum(info["reward_components"].values()),reward)
    assert float(info["reward_components"]["base"]) == 1
    assert float(info["reward_components"]["manual"]) == 1
    assert info["reward_side_channel_semantics"]["grasp_penalty"] == "critic_target_only"
    assert float(info["reward_side_channels"]["grasp_penalty"]) == pytest.approx(-0.2)


def test_task_limits_termination_and_jitter_validation():
    for kwargs in ({"max_episode_steps":0},{"eval_episodes":-1},{"eval_seed":-1},{"success_hold_steps":True}):
        with pytest.raises(ValueError): TaskConfig(**kwargs)
    with pytest.raises(ValueError,match="waypoints"):
        ResetConfig(xyz_jitter_m=[0.01,0,0])
    env = TaskOutcomeWrapper(LabelEnv([{"teleop_events":{"success":True}},{}]),
        TaskConfig(name="pick_and_place",max_episode_steps=2,terminate_on_success=False),("arm",))
    env.reset()
    _,_,terminated,truncated,_ = env.step(None)
    assert not bool(terminated) and not bool(truncated)
    _,_,terminated,truncated,_ = env.step(None)
    assert not bool(terminated) and bool(truncated)


def test_classifier_location_and_contents_are_saved_provenance_not_compared():
    env = apply_experiment_contract(RealRobotVectorEnv(single()),source_config={
        "classifier":{"path":"/actor/model.pt","content_sha256":"same-content"}})
    try:
        saved = experiment_contract_of(env)
        first = experiment_hash(env)
        env._source_config["classifier"]["path"] = "/learner/copy.pt"
        assert experiment_hash(env) == first
        validate_experiment(env,saved)
        env._source_config["classifier"]["content_sha256"] = "changed-content"
        assert experiment_hash(env) == first
        validate_experiment(env,saved)
        # Still saved with the experiment, for a human to compare.
        assert experiment_contract_of(env)["experiment"]["sources"]["classifier"]["content_sha256"] == "changed-content"
    finally: env.close()


_RECORDER_SOURCES = {"classifier": None, "classifier_threshold": 0.5, "arm_classifiers": {},
                     "classifier_image_keys": [], "reward_from_teleop_events": True,
                     "teleop_success_reward": 1.0, "use_grasp_penalty": False, "grasp_penalty_value": -0.02,
                     "teleop_selected_parts": [], "teleop_idle_behavior": "hold"}
# A training run with other reward sources: a classifier, another threshold, no teleop events,
# the grasp penalty.
_TRAINING_SOURCES = {**_RECORDER_SOURCES, "classifier": {"path": "/c.pt", "content_sha256": "c" * 64},
                     "classifier_threshold": 0.8, "reward_from_teleop_events": False, "use_grasp_penalty": True}


def test_demos_recorded_with_other_reward_sources_validate_against_the_training_env():
    recorded = apply_experiment_contract(RealRobotVectorEnv(single()), source_config=_RECORDER_SOURCES)
    trained = apply_experiment_contract(RealRobotVectorEnv(single()), source_config=_TRAINING_SOURCES)
    try:
        validate_transitions_against_contract([], demo_contract_of(trained), source="demos.pkl",
                                              saved=demo_contract_of(recorded))
        assert experiment_hash(recorded) == experiment_hash(trained)
        # A sidecar from before experiment metadata, into a classifier run: a warning, not an error.
        legacy = {key: value for key, value in demo_contract_of(recorded).items()
                  if key not in ("experiment", "effective_control", "contract_version")}
        with pytest.warns(UserWarning, match="predates"):
            validate_transitions_against_contract([], demo_contract_of(trained), source="legacy.pkl", saved=legacy)
    finally:
        recorded.close(); trained.close()


def test_station_facts_are_provenance_but_rate_and_image_size_are_compared():
    from real_world_rl.action_contract import compared_experiment, experiment_provenance
    trained = apply_experiment_contract(RealRobotVectorEnv(single()), source_config=_TRAINING_SOURCES)
    try:
        saved = experiment_contract_of(trained)
        recorded_contract = demo_contract_of(trained)
        first = experiment_hash(trained)
        config = trained.env.env.config
        # Deploy/eval CLI values: the station's, not the policy's.
        config.home_joints = [0.1] * 7
        config.safety_box = [0.0, 0.0, 0.0, 1.0, 1.0, 1.0]
        config.reset_settle_s = 2.0
        config.max_episode_steps = 50
        validate_experiment(trained, saved)
        validate_transitions_against_contract([], demo_contract_of(trained), source="demos.pkl", saved=recorded_contract)
        assert experiment_hash(trained) == first
        assert experiment_contract_of(trained)["experiment"]["home_joints"] == [0.1] * 7
        spec = {"cameras": ["wrist"], "image_size": [8, 8], "sources": {"teleop_idle_behavior": "policy"}}
        assert compared_experiment(spec) == {"image_size": [8, 8], "teleop": {"teleop_idle_behavior": "policy"}}
        assert experiment_provenance(spec) == {"cameras": ["wrist"], "sources": spec["sources"]}
        for field, value in (("control_hz", 5.0), ("image_size", (16, 16))):
            original = getattr(config, field)
            setattr(config, field, value)
            assert experiment_hash(trained) != first
            with pytest.raises(ValueError, match="differs"):
                validate_experiment(trained, saved)
            with pytest.raises(ValueError, match="experiment"):
                validate_transitions_against_contract([], demo_contract_of(trained), source="demos.pkl",
                                                      saved=recorded_contract)
            setattr(config, field, original)
    finally:
        trained.close()


@pytest.mark.parametrize("bimanual", [False, True])
def test_deploy_accepts_a_relocated_or_retrained_arm_classifier_before_connection(tmp_path, monkeypatch, bimanual):
    from rl_garden.observations import ObservationConfig
    from real_world_rl.training.entrypoints.hil_serl import HilSerlArgs
    from real_world_rl.training.env_building import build_hil_env
    from real_world_rl.deploy import DeployArgs, build_deploy_env
    from real_world_rl.action_contract import save_experiment_metadata
    from real_world_rl.envs.bimanual import BimanualRealRobotEnvConfig
    import rl_garden.models.reward.success.model as classifier_module
    from robot_infra.devices import Robot
    monkeypatch.setattr(classifier_module, "load_classifier_fn", lambda *a, **k: lambda obs: torch.ones(1))
    original, moved = tmp_path / "original.pt", tmp_path / "moved.pt"
    original.write_bytes(b"same classifier")
    moved.write_bytes(original.read_bytes())
    names = ("left", "right") if bimanual else ("arm",)
    robot = {"type": "franka", "arm_backend": "fake", "claim": False,
             "gripper": {"backend": "fake"}}
    arm = RealRobotEnvConfig(robot=robot, home_joints=[0.] * 7, reset_settle_s=0)
    task = TaskConfig(name="parallel_pick_and_place" if bimanual else "pick_and_place",
                      success_source="classifier", classifier_checkpoints={name: str(original) for name in names})
    config = (BimanualRealRobotEnvConfig(left=arm, right=dataclasses.replace(arm), task=task)
              if bimanual else dataclasses.replace(arm, task=task))
    field = "real_robot_bimanual" if bimanual else "real_robot"
    args = HilSerlArgs(env_backend=field, obs=ObservationConfig(state=True, rgb=()), **{field: config})
    trained = build_hil_env(args, role="learner", enable_teleop=False)
    try:
        save_experiment_metadata(trained, tmp_path / "experiment.json")
    finally:
        trained.close()
    paths_only = TaskConfig(classifier_checkpoints={name: str(moved) for name in names})
    cli = dataclasses.replace(config, task=paths_only)
    deploy_args = DeployArgs(checkpoint=str(tmp_path / "agent.pt"), env_backend=field,
                             enable_teleop=False, **{field: cli})
    connections = []
    monkeypatch.setattr(Robot, "connect", lambda self: connections.append(self))
    deployed = build_deploy_env(deploy_args, args)
    try:
        assert experiment_hash(deployed) == experiment_hash(trained)
        assert connections == []
    finally:
        deployed.close()
    # The classifier only scores success; its contents are provenance, not the policy's contract.
    moved.write_bytes(b"changed classifier")
    deployed = build_deploy_env(deploy_args, args)
    try:
        assert experiment_hash(deployed) == experiment_hash(trained)
    finally:
        deployed.close()
    assert connections == []


@pytest.mark.parametrize("global_hz,env_hz,expected",[(20,10,20),(10,20,20),(20,20,20)])
def test_actor_and_env_share_resolved_frequency(global_hz,env_hz,expected):
    from real_world_rl.training.runtime_paths import actor_control_hz,resolve_control_frequency
    args = types.SimpleNamespace(env_backend="real_robot",control_hz=global_hz,
        real_robot=RealRobotEnvConfig(control_hz=env_hz))
    resolve_control_frequency(args)
    assert actor_control_hz(args) == args.real_robot.control_hz == expected
    args.control_hz=30
    with pytest.raises(ValueError,match="conflicts"): resolve_control_frequency(args)


def test_manual_failure_remains_final_when_episode_continues():
    env = TaskOutcomeWrapper(LabelEnv([
        {"teleop_events":{"failure":True}},
        {"task_success":True,"task_success_source":"classifier"},
    ]),TaskConfig(name="pick_and_place",terminate_on_failure=False),("arm",))
    env.reset()
    first=env.step(None)
    second=env.step(None)
    assert not bool(first[3])
    assert first[4]["task_success"] is second[4]["task_success"] is False
    assert second[4]["manual_failure"] and float(second[1]) == 0
    assert not bool(second[2])


@pytest.mark.parametrize("configured,labels,expected_source",[
    ("classifier",{"left":{"success":True,"source":"manual"},
                   "right":{"success":True,"source":"classifier"}},"mixed"),
    ("tcp_proxy",{"left":{"success":True,"source":"manual"},
                  "right":{"success":True,"source":"manual"}},"manual"),
])
def test_parallel_outcome_source_reflects_actual_arm_labels(configured,labels,expected_source):
    env = TaskOutcomeWrapper(LabelEnv([{"arm_task_outcomes":labels}]),
        TaskConfig(name="parallel_pick_and_place",success_source=configured,
                   targets={"left":[0,0,0],"right":[0,0,0]}),("left","right"))
    env.reset()
    _,_,terminated,_,info = env.step(None)
    assert bool(terminated) and info["task_success"] is True
    assert info["task_success_source"] == expected_source
    assert info["arm_task_outcomes"] == labels
