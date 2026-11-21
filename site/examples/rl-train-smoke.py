"""CPU rehearsal: real loops/storage/updates, test env, in-process transport only."""
from __future__ import annotations

import argparse
import copy
from pathlib import Path
import sys
from unittest.mock import patch

import gymnasium as gym
import torch

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tests"))
from test_real_world_hil_serl_loops import _DummyVecEnv, _agent
from real_world_rl.envs.wrappers.teleop_event_reward import TeleopEventRewardWrapper
from real_world_rl.loops.durable import DurableStore, decode
from real_world_rl.loops.hil_serl import HilSerlActorLoop, HilSerlLearnerLoop
from real_world_rl.loops.sync import ActorSyncClient, PolicySnapshot


class LabelEnv(_DummyVecEnv, gym.Env):
    """Two three-step episodes; second step is a scripted human correction."""

    def __init__(self):
        super().__init__()
        self.episode = -1
        self.step_index = 0

    def reset(self, seed=None, options=None):
        self.episode += 1
        self.step_index = 0
        obs, info = super().reset(seed)
        return {"state": obs}, info

    def step(self, action):
        self.step_index += 1
        human = self.step_index == 2
        mask = torch.full_like(action, human, dtype=torch.bool)
        info = {"human_action_mask": mask, "override_mask": mask}
        if human:
            info["intervene_action"] = torch.tensor([[0.25, -0.25, 1.0]])
        if self.step_index == 3:
            info["teleop_events"] = {"success" if self.episode == 0 else "failure": True}
        return ({"state": torch.zeros(1, 4)}, torch.zeros(1), torch.tensor([False]),
                torch.tensor([False]), info)


class InProcessServer:
    """Replace HTTP only. Every publication is copied, as a wire payload would be."""

    def __init__(self, *args, **kwargs):
        self.version = 0
        self.snapshot = None

    def publish_params(self, state_dict, **metadata):
        self.version += 1
        self.snapshot = PolicySnapshot(
            state_dict=copy.deepcopy(state_dict), session_id="cpu-rehearsal",
            version=self.version, **metadata)

    def drain_stats(self):
        return []

    def stop(self):
        pass


class InProcessClient(ActorSyncClient):
    """Use the real outbox, journal admission and ACK removal without sockets."""

    def __init__(self, learner, directory):
        super().__init__("http://127.0.0.1:6000", actor_id="actor",
                         run_id="cpu-rehearsal", outbox_path=directory / "outbox.sqlite3")
        self.learner = learner
        self.audit = DurableStore(directory / "actor-audit.sqlite3")

    def start(self):
        pass

    def wait_for_policy(self, timeout=None):
        return self.latest_policy_snapshot()

    def latest_policy_snapshot(self):
        return self.learner._server.snapshot

    def push_transition(self, transition):
        super().push_transition(transition)
        for _, identity, payload in self._outbox.read():
            envelope = decode(payload)
            if envelope["kind"] == "transition":
                record = envelope["data"]
                # Extra audit copy is tutorial instrumentation, not normal outbox retention.
                self.audit.append(identity, payload)
                self.learner._admit_records([record])
            self._outbox.remove(identity)
        self.learner._drain_journal()
        if transition.get("record_type") == "transition":
            print(f"received={self.learner.received_transitions} "
                  f"demo_buffer_size={len(self.learner.agent.offline_replay_buffer)}")
            if self.learner._ready_to_train():
                self.learner._train_step()
                self.learner._publish_policy()

    def stop(self, *args, **kwargs):
        result = super().stop(*args, **kwargs)
        self.audit.close()
        return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    args = parser.parse_args()
    args.directory.mkdir(parents=True, exist_ok=False)
    torch.set_num_threads(1)
    torch.manual_seed(7)
    agent = _agent()
    agent.init_demo_buffer(32, 0.5)
    agent.checkpoint_dir = str(args.directory / "checkpoints")
    agent.checkpoint_freq = 0
    actor_agent = _agent()
    with patch("real_world_rl.loops.learner.LearnerSyncServer", InProcessServer):
        learner = HilSerlLearnerLoop(agent, "127.0.0.1", 0, agent.checkpoint_dir,
                                     buffer_period=100, publish_freq=1)
        try:
            learner._publish_policy()
            client = InProcessClient(learner, args.directory)
            # Wrapper accepts the test vector-env surface; no hardware or teleop driver.
            env = TeleopEventRewardWrapper(LabelEnv())
            actor = HilSerlActorLoop(env, actor_agent.policy, client, control_hz=1000)
            actor.run(total_steps=6)
            learner._save_bundle()
            for key in ("received", "update", "buffer_size", "demo_buffer_size", "intervened_received"):
                print(f"learner/{key}={learner._learner_scalars()[key]}")
            print(f"global_update={agent.global_update} actor_params_loaded={actor.params_version}")
            print(f"actor_last_global_update={actor._snapshot_metadata['global_update']}")
            print("outcomes=" + str(learner.episode_metrics.summary()))
            assert learner.received_transitions == 6
            assert len(agent.offline_replay_buffer) == 2
            assert actor._snapshot_metadata["global_update"] > 0
        finally:
            learner._server.stop()
            learner._journal.close()
        restored_agent = _agent()
        restored_agent.init_demo_buffer(32, 0.5)
        restored_agent.checkpoint_dir = agent.checkpoint_dir
        restored = HilSerlLearnerLoop(restored_agent, "127.0.0.1", 0, agent.checkpoint_dir)
        try:
            assert restored.restore_checkpoint_bundle()
            assert restored._drain_journal() == 0
            assert restored.received_transitions == 6
            print(f"restored_received={restored.received_transitions} "
                  f"restored_update={restored_agent.global_update} "
                  f"restored_demo={len(restored_agent.offline_replay_buffer)}")
        finally:
            restored._server.stop()
            restored._journal.close()
        from real_world_rl.training.evaluation import evaluate_policy
        evaluate_policy(TeleopEventRewardWrapper(LabelEnv()), restored_agent.policy,
                        episodes=2, seed=7)


if __name__ == "__main__":
    main()
