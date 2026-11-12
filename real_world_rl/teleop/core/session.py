"""Episode-boundary holding, separate from policy/human intervention ownership."""
from __future__ import annotations

import math
import sys
import time

import numpy as np

from real_world_rl.supervise import Heartbeat
from real_world_rl.teleop.core.composite import CompositeTeleopSource


def session_backend(env):
    seen = set()
    while id(env) not in seen:
        seen.add(id(env))
        if callable(getattr(type(env), "session_hold_tick", None)):
            prepare = getattr(type(env), "require_session_hold", None)
            if callable(prepare):
                prepare(env)
            return env
        env = getattr(env, "env", None)
        if env is None:
            break
    raise ValueError("episode_end_behavior=hold requires a real_robot Cartesian env")


def validate_neutral_options(neutral_threshold=.05, neutral_stable_s=.2):
    if not math.isfinite(neutral_threshold) or not 0 < neutral_threshold < 1:
        raise ValueError("neutral_threshold must be in (0, 1)")
    if not math.isfinite(neutral_stable_s) or neutral_stable_s < 0:
        raise ValueError("neutral_stable_s must be nonnegative")


def session_observe(env):
    """Refresh through the observation chain without resetting the task or controller."""
    import gymnasium as gym
    method = getattr(type(env), "session_observe", None)
    if callable(method):
        return method(env)
    child = getattr(env, "env", None)
    if child is None:
        raise ValueError("session hold backend must provide session_observe()")
    obs = session_observe(child)
    return env.observation(obs) if isinstance(env, gym.ObservationWrapper) else obs


def wait_after_episode(env, source, *, allow_next, control_hz,
                       neutral_threshold=.05, neutral_stable_s=.2,
                       clock=time.monotonic, sleep=time.sleep, starting=False):
    """Keep the accepted target until explicit quit or neutral-confirmed next.

    No env.step/reset or archive writes happen here. Faults propagate to the
    owner's cleanup. Only this loop feeds its heartbeat, after a successful tick.
    """
    if not math.isfinite(control_hz) or control_hz <= 0:
        raise ValueError("control_hz must be positive")
    validate_neutral_options(neutral_threshold, neutral_stable_s)
    if not isinstance(source, CompositeTeleopSource) or not callable(getattr(source.motion, "session_input", None)):
        raise ValueError("session hold requires SpaceMouse plus an event device")
    enable = getattr(source.events, "enable_session_events", None)
    if callable(enable):
        enable()
    backend = session_backend(env)
    def event(phase, reason):
        emit = getattr(backend, "session_event", None)
        if callable(emit):
            emit(phase, reason)
    event("READY" if starting else "EPISODE_HOLD",
          "awaiting_neutral" if starting else "awaiting_next_or_quit" if allow_next else "budget_exhausted")
    target = backend.session_start_target() if starting else backend.session_hold_target()
    heartbeat = Heartbeat.from_env()
    neutral_since = None
    next_requested = starting
    print("START_NEUTRAL: release the puck; q exits control." if starting else
          "EPISODE_HOLD: motion input ignored; q exits control. " +
          ("n starts the next episode (task reset may move/home the arm)." if allow_next else
           "Episode budget reached; next is disabled."), file=sys.stderr, flush=True)
    while True:
        start = clock()
        events = source.events.poll().events
        if events.get("operator_quit"):
            event("STOPPING", "operator_quit")
            return "quit"
        next_requested = next_requested or (allow_next and bool(events.get("next_episode")))
        axes, buttons = source.motion.session_input()
        axes = np.asarray(axes, dtype=float)
        if axes.shape != (6,) or not np.isfinite(axes).all():
            raise ValueError("invalid session input")
        neutral = np.max(np.abs(axes)) <= neutral_threshold and not any(buttons.values())
        if not neutral:
            neutral_since = None
        elif neutral_since is None:
            neutral_since = start
        backend.session_hold_tick(target)
        heartbeat.beat()
        if next_requested and neutral_since is not None and start - neutral_since >= neutral_stable_s:
            event("ACTIVE" if starting else "READY", "neutral_confirmed" if starting else "next_episode_requested")
            return "next"
        sleep(max(0., 1. / control_hz - (clock() - start)))


def wait_before_episode(env, source, **kwargs):
    """After reset, hold the new measured pose until the puck is stably neutral.

    Never reuse the previous trajectory's accepted target after homing. This gate
    does not run on intervention changes inside an episode.
    """
    return wait_after_episode(env, source, allow_next=True, starting=True, **kwargs)


def notify_session_event(env, phase, reason):
    """Record owner events that happen outside the episode-boundary wait loop."""
    backend = session_backend(env)
    emit = getattr(backend, "session_event", None)
    if callable(emit):
        emit(phase, reason)


def session_source(env):
    """Find a SpaceMouse composite before reset or controller acquisition."""
    seen = set()
    while env is not None and id(env) not in seen:
        seen.add(id(env))
        source = vars(env).get("source") if hasattr(env, "__dict__") else None
        if isinstance(source, CompositeTeleopSource) and callable(getattr(source.motion, "session_input", None)):
            return source
        env = vars(env).get("env") if hasattr(env, "__dict__") else None
    raise ValueError("session hold requires SpaceMouse plus an event device")
