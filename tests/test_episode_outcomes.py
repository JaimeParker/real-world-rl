from real_world_rl.metrics import EpisodeMetrics, EpisodeOutcomeAccumulator


def test_explicit_labels_and_manual_failure_precedence():
    episode = EpisodeOutcomeAccumulator()
    episode.observe({"task_success": True, "human_action_mask": [False, False],
                     "teleop_events": {"success": True}})
    assert episode.finish().autonomous_success
    episode.observe({"task_success": True, "teleop_events": {"failure": True, "success": True}})
    episode.observe({"task_success": True})
    assert episode.finish().task_success is False


def test_completed_denominator_and_unknown_interrupted_discarded():
    metrics = EpisodeMetrics()
    for info, options in [({"task_success": True}, {}),
                          ({"task_success": True, "human_action_mask": [True, False]}, {}),
                          ({"task_success": False}, {}), ({}, {}),
                          ({"task_success": True}, {"interrupted": True}),
                          ({}, {"discarded": True})]:
        episode = EpisodeOutcomeAccumulator()
        episode.observe(info)
        metrics.add(episode.finish(**options))
    result = metrics.summary()
    assert result["success_rate"] == result["autonomous_success_rate"] == 0.25
    assert result["task_success_rate"] == 0.5
    assert result["episode_intervention_rate"] == result["step_intervention_rate"] == 0.25
    assert result["unknown_episodes"] == 1
    assert result["completed_episodes"] == 4
    assert result["interrupted_episodes"] == result["discarded_episodes"] == 1


def test_no_success_inferred_from_terminal_or_positive_reward():
    episode = EpisodeOutcomeAccumulator()
    episode.observe({"terminated": True, "reward": 1})
    assert episode.finish().task_success is None


def test_authoritative_empty_mask_ignores_label_only_intervention_flag():
    episode = EpisodeOutcomeAccumulator()
    episode.observe({"intervened": True, "human_action_mask": [False], "succeed": True})
    assert episode.finish().autonomous_success


def test_evaluation_preserves_rollout_error_and_closes():
    import pytest
    import torch
    from real_world_rl.training.evaluation import evaluate_policy

    class Env:
        closed = False
        def reset(self, **kwargs):
            return {"state": torch.zeros(1, 1)}, {}
        def step(self, action):
            raise ValueError("rollout failed")
        def close(self):
            self.closed = True
            raise RuntimeError("close failed")

    env = Env()
    with pytest.raises(ValueError, match="rollout failed"):
        evaluate_policy(env, object(), episodes=1, seed=0,
                        predict=lambda obs: torch.zeros(1, 1))
    assert env.closed


def test_source_checkout_backend_request_in_fresh_process():
    import os
    from pathlib import Path
    import subprocess
    import sys

    root = Path(__file__).resolve().parents[1]
    environment = dict(os.environ, PYTHONDONTWRITEBYTECODE="1",
                       PYTHONPATH=os.pathsep.join(str(root / name) for name in (".", "rl-garden", "robot_infra")))
    subprocess.run([sys.executable, "-c", """
from real_world_rl.training.entrypoints.hil_serl import HilSerlArgs
from real_world_rl.training.env_building import hil_env_request
from real_world_rl.envs.real_robot.config import RealRobotEnvConfig
args = HilSerlArgs(env_backend='real_robot', real_robot=RealRobotEnvConfig(remote='unused:1'))
request = hil_env_request(args, 'source-checkout')
assert request.backend_config is args.real_robot
"""], cwd=root, env=environment, check=True, capture_output=True, text=True)


def test_finalized_unknown_clears_old_label_and_ignores_raw_success_aliases():
    episode = EpisodeOutcomeAccumulator()
    episode.observe({"task_success":True})
    episode.observe({"task_outcome_finalized":True,"task_success":None,"success":True,"succeed":True,
                     "teleop_events":{"success":True}})
    assert episode.finish().task_success is None
    episode.observe({"task_outcome_finalized":True,"task_success":True,"manual_failure":True})
    assert episode.finish().task_success is False
