"""Shared real-world evaluation with explicit outcomes and intervention metrics."""
from __future__ import annotations

import logging
import math
import sys
import time

import torch

from real_world_rl.contracts.outcomes import EpisodeMetrics, EpisodeOutcomeAccumulator


def evaluate_policy(env, policy, *, episodes, seed, predict=None, control_hz=10.,
                    episode_end_behavior="auto", neutral_threshold=.05, neutral_stable_s=.2,
                    gripper_open_fraction=.95, gripper_closed_fraction=.05, start_tolerance_rad=.8):
    """Evaluate at the actor's control rate, honoring its episode-boundary setting (``hold``, or
    ``confirm``: ``ConfirmSession`` resets and starts every episode at the operator's keys)."""
    from real_world_rl.teleop.core import session
    from real_world_rl.teleop.core.operator_keys import operator_quit_requested
    task = getattr(env, "task_config", None)
    if task is not None:
        episodes = task.eval_episodes if task.eval_episodes is not None else episodes
        seed = task.eval_seed if task.eval_seed is not None else seed
    metrics = EpisodeMetrics()
    returns = []
    try:
        if not math.isfinite(control_hz) or control_hz <= 0:
            raise ValueError("control_hz must be positive")
        session.validate_episode_end_behavior(episode_end_behavior)
        source = None
        confirm = None
        if episode_end_behavior == "confirm":
            confirm = session.ConfirmSession(
                env, session.confirm_source(env), control_hz=control_hz,
                start_tolerance_rad=start_tolerance_rad, gripper_open_fraction=gripper_open_fraction,
                gripper_closed_fraction=gripper_closed_fraction)
        session_options = dict(control_hz=control_hz, neutral_threshold=neutral_threshold,
                               neutral_stable_s=neutral_stable_s)
        if episode_end_behavior == "hold":
            session.validate_neutral_options(neutral_threshold, neutral_stable_s)
            session.session_backend(env)
            source = session.session_source(env)
        for episode in range(episodes):
            accumulator = EpisodeOutcomeAccumulator()
            episode_return = 0.0
            try:
                if hasattr(policy, "reset_episode"):
                    policy.reset_episode()
                if confirm is not None:
                    reset = lambda: env.reset(seed=seed if episode == 0 else None)  # noqa: E731
                    started = (confirm.first_episode(reset) if episode == 0 else
                               confirm.between_episodes(reset, allow_next=True))
                    if started != "start":
                        break
                    obs = session.session_observe(env)
                else:
                    obs, _ = env.reset(seed=seed if episode == 0 else None)
                if source is not None:
                    if session.wait_before_episode(env, source, **session_options) == "quit":
                        break
                    obs = session.session_observe(env)
                while True:
                    start = time.perf_counter()
                    with torch.no_grad():
                        action = predict(obs) if predict is not None else policy.predict(obs, deterministic=True)
                    sample = next(iter(obs.values())) if isinstance(obs, dict) else obs
                    obs, reward, terminated, truncated, info = env.step(action.to(sample.device))
                    accumulator.observe(info)
                    episode_return += float(torch.as_tensor(reward).sum())
                    if not operator_quit_requested(info):
                        remaining = 1. / control_hz - (time.perf_counter() - start)
                        if remaining > 0:
                            time.sleep(remaining)
                    if (bool(torch.as_tensor(terminated).any()) or bool(torch.as_tensor(truncated).any())
                            or operator_quit_requested(info)):
                        outcome = accumulator.finish(terminated=terminated, truncated=truncated,
                                                     interrupted=operator_quit_requested(info))
                        metrics.add(outcome)
                        returns.append(episode_return)
                        print(f"[eval] episode={episode} task_success={outcome.task_success} "
                              f"autonomous_success={outcome.autonomous_success} return={episode_return:.3f}", flush=True)
                        break
            except BaseException:
                metrics.add(accumulator.finish(interrupted=True))
                raise
            if operator_quit_requested(info):
                if source is not None:
                    session.notify_session_event(env, "STOPPING", "operator_quit")
                if confirm is not None:
                    confirm.stopping("operator_quit")
                break
            if confirm is not None and episode + 1 >= episodes:
                # The last episode: POST_EPISODE so the operator can open the gripper; no reset.
                confirm.post_episode(allow_next=False)
            if source is not None:
                if session.wait_after_episode(env, source, allow_next=episode + 1 < episodes,
                                              **session_options) == "quit":
                    break
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
    from real_world_rl.deploy.checkpoint import _checkpoint_file, _saved_experiment
    from real_world_rl.contracts.action import validate_experiment
    request = SimpleNamespace(checkpoint=args.load_checkpoint, config_path=None)
    path = _checkpoint_file(request)
    validate_experiment(env, _saved_experiment(request))
    return str(path)
