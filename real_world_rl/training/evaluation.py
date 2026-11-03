"""Shared real-world evaluation with explicit outcomes and intervention metrics."""
from __future__ import annotations

import logging
import sys

import torch

from real_world_rl.metrics import EpisodeMetrics, EpisodeOutcomeAccumulator


def evaluate_policy(env, policy, *, episodes, seed, predict=None):
    task = getattr(env, "task_config", None)
    if task is not None:
        episodes = task.eval_episodes if task.eval_episodes is not None else episodes
        seed = task.eval_seed if task.eval_seed is not None else seed
    metrics = EpisodeMetrics()
    returns = []
    try:
        for episode in range(episodes):
            accumulator = EpisodeOutcomeAccumulator()
            episode_return = 0.0
            try:
                if hasattr(policy, "reset_episode"):
                    policy.reset_episode()
                obs, _ = env.reset(seed=seed if episode == 0 else None)
                while True:
                    with torch.no_grad():
                        action = predict(obs) if predict is not None else policy.predict(obs, deterministic=True)
                    sample = next(iter(obs.values())) if isinstance(obs, dict) else obs
                    obs, reward, terminated, truncated, info = env.step(action.to(sample.device))
                    accumulator.observe(info)
                    episode_return += float(torch.as_tensor(reward).sum())
                    if bool(torch.as_tensor(terminated).any()) or bool(torch.as_tensor(truncated).any()):
                        outcome = accumulator.finish(terminated=terminated, truncated=truncated)
                        metrics.add(outcome)
                        returns.append(episode_return)
                        print(f"[eval] episode={episode} task_success={outcome.task_success} "
                              f"autonomous_success={outcome.autonomous_success} return={episode_return:.3f}", flush=True)
                        break
            except BaseException:
                metrics.add(accumulator.finish(interrupted=True))
                raise
    finally:
        summary = metrics.summary()
        mean_return = sum(returns) / len(returns) if returns else 0.0
        print("[eval] " + " ".join(f"{key}={value:.3f}" if isinstance(value, float) else f"{key}={value}"
                                      for key, value in summary.items())
              + f" mean_return={mean_return:.3f} n_trajs={episodes}", flush=True)
        close_resources(env, getattr(policy, "provider", getattr(policy, "base_provider", None)))
    return summary


def close_resources(*resources):
    """Attempt every cleanup while preserving any active rollout exception."""
    handling_error = sys.exc_info()[0] is not None
    first_error = None
    seen = set()
    for resource in resources:
        if resource is None or id(resource) in seen:
            continue
        seen.add(id(resource))
        close = getattr(resource, "close", None)
        if close is None:
            continue
        try:
            close()
        except BaseException as error:
            logging.getLogger(__name__).exception("Evaluation resource cleanup failed")
            if first_error is None:
                first_error = error
    if first_error is not None and not handling_error:
        raise first_error


def close_env(env):
    close_resources(env)


def resolve_eval_checkpoint(args, env):
    """Resolve bundle weights and validate requested/effective identity before reset."""
    from types import SimpleNamespace
    from real_world_rl.deploy import _checkpoint_file, _saved_experiment
    from real_world_rl.action_contract import validate_experiment
    request = SimpleNamespace(checkpoint=args.load_checkpoint, config_path=None)
    path = _checkpoint_file(request)
    validate_experiment(env, _saved_experiment(request))
    return str(path)
