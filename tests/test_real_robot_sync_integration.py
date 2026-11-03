"""Production env composition, semantic provenance, HTTP and bundle integration."""
import copy
import dataclasses
import threading
import urllib.error
import urllib.request

import pytest
import torch

from rl_garden.buffers.replay_buffer import ReplayBuffer
from rl_garden.observations.config import ObservationConfig
import real_world_rl.envs.backends.real_robot  # noqa: F401
from real_world_rl.envs.real_robot import RealRobotEnvConfig
from real_world_rl.loops.actor import ActorLoop
from real_world_rl.loops.learner import LearnerLoop
from real_world_rl.loops.sync import ActorSyncClient
from real_world_rl.loops.durable import DurableStore, encode, decode
from real_world_rl.training.entrypoints.hil_serl import HilSerlArgs
from real_world_rl.training.env_building import build_hil_env
from real_world_rl.training.runtime_paths import learner_experiment_kwargs
from real_world_rl.trajectory import begin_episode


class SpacePolicy:
    def __init__(self, action_shape): self.action_shape = action_shape
    def eval(self): return self
    def state_dict(self): return {"weight": torch.zeros(1)}
    def load_state_dict(self, state): pass
    def predict(self, obs, deterministic=False): return torch.zeros(1, *self.action_shape)


class SpaceAgent:
    def __init__(self, env, directory):
        self.env = env
        self.policy = SpacePolicy(env.single_action_space.shape)
        self.replay_buffer = ReplayBuffer(env.single_observation_space, env.single_action_space,
                                         num_envs=1, buffer_size=8, storage_device="cpu", sample_device="cpu")
        self.offline_replay_buffer = None
        self.buffer_device = "cpu"
        self.global_update = 0
        self.learning_starts = 100
        self.utd = 1
        self.checkpoint_dir = str(directory)
        self.checkpoint_freq = self.log_freq = 0
        self.save_final_checkpoint = True
        self.save_replay_buffer = False
        self.logger = None
    def save(self, path, include_replay_buffer=False):
        torch.save({"global_update": self.global_update}, path)
    def load(self, path, load_replay_buffer=False):
        self.global_update = torch.load(path, weights_only=True)["global_update"]


def args_for(model_path):
    return HilSerlArgs(env_backend="real_robot", classifier_checkpoint=str(model_path),
        classifier_image_keys=(), obs=ObservationConfig(state=True, rgb=()),
        real_robot=RealRobotEnvConfig(robot={"type": "franka", "arm_backend": "fake",
            "gripper": {"backend": "fake"}, "claim": False}, cameras=[], gripper_in_action=False,
            home_joints=[0.] * 7, reset_settle_s=0, max_episode_steps=2, control_hz=1000,
            control={"max_delta_tau": .5}))


def post_records(learner, records):
    request = urllib.request.Request("http://%s:%s/records" % learner._server.server_address,
                                     data=encode(records), method="POST")
    with urllib.request.urlopen(request, timeout=2) as response: return response.status


def test_real_env_http_pipeline_accepts_path_provenance_and_rejects_changed_controller(tmp_path, monkeypatch):
    import rl_garden.models.reward.success.model as classifier_module
    monkeypatch.setattr(classifier_module, "load_classifier_fn", lambda *a, **k: lambda obs: torch.zeros(1))
    source = tmp_path / "learner_model.pt"
    moved = tmp_path / "actor_model.pt"
    source.write_bytes(b"same model contents")
    moved.write_bytes(source.read_bytes())
    learner_env = build_hil_env(args_for(source), role="learner", enable_teleop=False)
    actor_env = build_hil_env(args_for(moved), role="actor", enable_teleop=False)
    directory = tmp_path / "checkpoints"
    learner = LearnerLoop(SpaceAgent(learner_env, directory), "127.0.0.1", 0,
                          **learner_experiment_kwargs(learner_env, directory))
    client = ActorSyncClient("http://%s:%s" % learner._server.server_address,
                             outbox_path=str(tmp_path / "actor" / "outbox.sqlite3"),
                             startup_timeout=3, drain_timeout=3, preflight=False)
    errors = []
    def run_learner():
        try: learner.run()
        except BaseException as exc: errors.append(exc)
    thread = threading.Thread(target=run_learner, daemon=True)
    thread.start()
    try:
        ActorLoop(actor_env, SpacePolicy(actor_env.single_action_space.shape), client,
                  control_hz=1000000).run(total_steps=2)
    finally:
        learner.stop(); thread.join(8)
    assert not thread.is_alive() and not errors
    assert learner.received_transitions == len(learner.agent.replay_buffer) == 2
    assert learner.episode_metrics.summary()["completed_episodes"] == 1
    journal = DurableStore(directory / "journal.sqlite3")
    records = [decode(payload) for _, _, payload in journal.read()]
    journal.close()
    assert [record["record_type"] for record in records] == ["episode_start", "transition", "transition", "episode_end"]
    assert str(moved) in str(learner.effective_experiment["provenance"])
    assert learner.experiment_hash == records[0]["experiment_hash"]
    assert (directory / "bundle_manifest.json").is_file()

    restored_env = build_hil_env(args_for(source), role="learner", enable_teleop=False)
    restored = LearnerLoop(SpaceAgent(restored_env, directory), "127.0.0.1", 0,
                          **learner_experiment_kwargs(restored_env, directory))
    try:
        assert restored.restore_checkpoint_bundle()
        restored._server.start()
        same = copy.deepcopy(restored.effective_experiment)
        same["provenance"]["sources"]["classifier"]["path"] = str(source)
        legitimate = begin_episode("new-episode", experiment_hash=restored.experiment_hash,
                                    effective_experiment=same)
        assert post_records(restored, [legitimate]) == 204
        for field in ("sdk_version", "control"):
            changed = copy.deepcopy(same)
            if field == "sdk_version": changed["runtime_control"]["arm"]["sdk_version"] = "different-sdk"
            else: changed["runtime_control"]["arm"]["effective"]["max_delta_tau"] = .25
            rejected = begin_episode("bad-" + field, experiment_hash=restored.experiment_hash,
                                     effective_experiment=changed)
            with pytest.raises(urllib.error.HTTPError) as error: post_records(restored, [rejected])
            assert error.value.code == 400
        assert len(restored._journal) == 5
    finally:
        restored._server.stop(); restored._journal.close(); restored_env.close()
