"""Episode-boundary holding, separate from policy/human intervention ownership.

The gate needs a composite teleop source: an event device for the operator's next/quit keys and
a motion device that is a ``SessionInputSource`` (its ``session_input()`` reading tells when the
operator's input is neutral).
"""
from __future__ import annotations

import math
import sys
import time

import numpy as np

from real_world_rl.supervise import Heartbeat
from real_world_rl.teleop.core.composite import CompositeTeleopSource
from real_world_rl.teleop.core.contract import provides_session_input

SESSION_SOURCE_REQUIRED = (
    "session hold requires an event device (--teleop-event-device) plus a motion device that "
    "provides a neutral reading (a SessionInputSource with session_input())"
)




def operator_status(text: str) -> None:
    """One ``PHASE: ...`` line for the operator, on stderr, flushed at once."""
    print(text, file=sys.stderr, flush=True)

def is_session_source(source) -> bool:
    """Whether the session gate can wait on ``source``: a composite whose motion half is a
    ``SessionInputSource``."""
    return isinstance(source, CompositeTeleopSource) and provides_session_input(source.motion)


def require_session_source(source) -> None:
    if not is_session_source(source):
        raise ValueError(SESSION_SOURCE_REQUIRED)


def session_backend(env, mode="hold"):
    """The env in ``env``'s chain that holds the arm between episodes (``session_hold_tick``),
    after its ``require_session_hold`` accepted ``mode`` (``hold``: Cartesian only; ``confirm``:
    Cartesian or joint)."""
    seen = set()
    while id(env) not in seen:
        seen.add(id(env))
        if callable(getattr(type(env), "session_hold_tick", None)):
            prepare = getattr(type(env), "require_session_hold", None)
            if callable(prepare):
                if mode == "hold":
                    prepare(env)
                else:
                    prepare(env, mode=mode)
            return env
        env = getattr(env, "env", None)
        if env is None:
            break
    if mode == "hold":
        raise ValueError("episode_end_behavior=hold requires a real_robot Cartesian env")
    raise ValueError(f"episode_end_behavior={mode} requires a real_robot or real_robot_bimanual env")


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
    require_session_source(source)
    enable =getattr(source.events, "enable_session_events", None)
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
    operator_status("START_NEUTRAL: release motion controls or align an always-on leader with the follower; q exits control." if starting else
                    "EPISODE_HOLD: motion input ignored; q exits control. " +
                    ("n starts the next episode (task reset may move/home the arm)." if allow_next else
                     "Episode budget reached; next is disabled."))
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
    """Find the session gate's teleop source (``is_session_source``) in ``env``'s chain before
    reset or controller acquisition."""
    seen = set()
    while env is not None and id(env) not in seen:
        seen.add(id(env))
        source = vars(env).get("source") if hasattr(env, "__dict__") else None
        if is_session_source(source):
            return source
        env = vars(env).get("env") if hasattr(env, "__dict__") else None
    raise ValueError(SESSION_SOURCE_REQUIRED)


# -- episode_end_behavior=confirm ------------------------------------------------------------

EPISODE_END_BEHAVIORS = ("auto", "hold", "confirm")
CONFIRM_SOURCE_REQUIRED = (
    "episode_end_behavior=confirm needs an event device for the operator keys (SPACE start, "
    "n reset, o/c gripper, Enter confirm, q quit): pass --teleop-event-device keyboard|stdin|jsonl, "
    "or bind the keys on a GELLO event_device"
)
# Rate limit of the READY start-lock line (it is printed only when the misaligned joints change).
_LOCK_STATUS_PERIOD_S = .5


def validate_episode_end_behavior(behavior) -> None:
    if behavior not in EPISODE_END_BEHAVIORS:
        raise ValueError("episode_end_behavior must be auto, hold or confirm")


def confirm_session_options(args) -> dict:
    """The confirm-session keyword arguments of an args object (``HilEnvArgs``; the start
    tolerance of a recorder's ``start_tolerance_rad``, else 0.8 rad)."""
    return {"gripper_open_fraction": args.gripper_open_fraction,
            "gripper_closed_fraction": args.gripper_closed_fraction,
            "start_tolerance_rad": getattr(args, "start_tolerance_rad", .8)}


def validate_gripper_fractions(gripper_open_fraction=.95, gripper_closed_fraction=.05) -> None:
    """``0 <= gripper_closed_fraction < gripper_open_fraction <= 1`` (fractions of the gripper's
    maximum width)."""
    for name, value in (("gripper_open_fraction", gripper_open_fraction),
                        ("gripper_closed_fraction", gripper_closed_fraction)):
        if not math.isfinite(value) or not 0 <= value <= 1:
            raise ValueError(f"{name} must be in [0, 1], got {value}")
    if not gripper_closed_fraction < gripper_open_fraction:
        raise ValueError("gripper_closed_fraction must be below gripper_open_fraction, got "
                         f"{gripper_closed_fraction} >= {gripper_open_fraction}")


def operator_keys_available(source):
    """Whether ``source`` delivers the confirm-mode operator keys: True (an event device of
    ``operator_keys``, or a GELLO with an ``event_device``), False, or None when it cannot tell
    before the robot connects (a leader arm read through the robot's teleop part)."""
    children = []
    if isinstance(source, CompositeTeleopSource):
        children = [source.events, source.motion]
    elif isinstance(getattr(source, "sources", None), dict):  # BimanualTeleopSource
        children = list(source.sources.values())
    if children:
        answers = [operator_keys_available(child) for child in children]
        return True if True in answers else None if None in answers else False
    method = getattr(source, "provides_operator_keys", None)
    return method() if callable(method) else False


def require_confirm_source(source) -> None:
    """Refuses a source without the operator keys and switches its keyboard-like event devices
    to ordered presses (``OperatorKeySource.enable_confirm_keys``)."""
    available = operator_keys_available(source)
    if available is False:
        raise ValueError(CONFIRM_SOURCE_REQUIRED)
    if available is None:
        operator_status("CONFIRM: the operator keys come from the robot's teleop event_device, which "
                        "cannot be checked before it connects; if SPACE/n do nothing, pass "
                        "--teleop-event-device stdin.")
    pending = [source]
    while pending:
        node = pending.pop()
        enable = getattr(node, "enable_confirm_keys", None)
        if callable(enable):
            enable()
        if isinstance(node, CompositeTeleopSource):
            pending += [node.events, node.motion]
        elif isinstance(getattr(node, "sources", None), dict):
            pending += list(node.sources.values())


def confirm_source(env):
    """The teleop source of the intervention wrapper in ``env``'s chain (the confirm session's
    key source), found before reset or controller acquisition."""
    seen = set()
    while env is not None and id(env) not in seen:
        seen.add(id(env))
        source = vars(env).get("source") if hasattr(env, "__dict__") else None
        if source is not None and callable(getattr(source, "poll", None)):
            return source
        env = vars(env).get("env") if hasattr(env, "__dict__") else None
    raise ValueError(CONFIRM_SOURCE_REQUIRED)


def leader_follower_gaps(env, reading, contract) -> np.ndarray:
    """``|q_leader - q_follower|`` per arm and joint (rad), ``(num_arms, num_joints)``.

    ``joint_abs`` only: the leader position is ``reading``'s arm action decoded from the joint
    limits (``contract["joint_limits"]``, per arm on a two-arm env); the follower is the latest
    observation of each arm's ``RealRobotEnv``."""
    from real_world_rl.contracts.action import arm_block_slices, num_arms_of
    from real_world_rl.teleop.core.hold import _arm_envs_of, follower_joint_pos

    action = np.asarray(reading.action, dtype=np.float64)
    if num_arms_of(contract) == 1:
        low, high = np.asarray(contract["joint_limits"], dtype=np.float64).T
        leader = low + (action[: len(low)] + 1.0) / 2.0 * (high - low)
        return np.abs(leader - follower_joint_pos(env))[None]
    errors = []
    limits = np.asarray(contract["joint_limits"], dtype=np.float64)
    for block, arm_env, arm_limits in zip(arm_block_slices(contract), _arm_envs_of(env, contract), limits):
        low, high = arm_limits.T
        leader = low + (action[block] + 1.0) / 2.0 * (high - low)
        errors.append(np.abs(leader - follower_joint_pos(arm_env)))
    return np.stack(errors)


def _keys_from_events(events) -> list:
    from real_world_rl.teleop.core.operator_keys import CONFIRM_EVENT_KEYS
    keys = [key for name, key in CONFIRM_EVENT_KEYS.items() if events.get(name)]
    # Any other key an event device reports (s/f/r/Esc) counts as "another key".
    if any(events.get(name) for name in ("success", "failure", "rerecord")):
        keys.append("other")
    return keys


class ConfirmSession:
    """The operator-confirmed episode boundary (``episode_end_behavior=confirm``).

    Phases, each announced by one ``<PHASE>: ...`` line (``operator_status``) and a session
    event: ``POST_EPISODE`` after an episode ended (the arm holds where it is, the gripper stays
    as it is, teleop motion is ignored; ``o``/``c`` open/close the gripper, ``n`` asks for the
    reset, ``q`` quits), ``GRIPPER_CHECK`` when ``n`` finds a partly closed gripper (``Enter``
    resets, ``o`` opens it in place, any other key goes back), ``RESETTING`` (the caller's reset:
    the arm homes, then the gripper opens), ``READY`` (the arm holds at home, ``SPACE`` starts;
    in ``joint_abs`` the start is locked while a leader joint is further than
    ``start_tolerance_rad`` from its follower), ``ACTIVE`` and ``STOPPING``.

    No ``env.step``/``reset`` happens in the waits; every wait tick re-sends the hold target
    through ``session_hold_tick`` (which checks the arm's health and the step watchdog) and then
    feeds the supervisor heartbeat. A fault propagates to the caller's cleanup. The backend and the
    source are checked at construction, before any reset.
    """

    def __init__(self, env, source, *, control_hz, contract=None, start_tolerance_rad=.8,
                 gripper_open_fraction=.95, gripper_closed_fraction=.05,
                 clock=time.monotonic, sleep=time.sleep, status=None):
        from real_world_rl.contracts.action import LEGACY_ACTION_CONTRACT, get_env_action_contract, with_arm_layout

        if not math.isfinite(control_hz) or control_hz <= 0:
            raise ValueError("control_hz must be positive")
        if not math.isfinite(start_tolerance_rad) or start_tolerance_rad <= 0:
            raise ValueError("start_tolerance_rad must be positive")
        validate_gripper_fractions(gripper_open_fraction, gripper_closed_fraction)
        self.backend = session_backend(env, mode="confirm")
        require_confirm_source(source)
        self.env = env
        self.source = source
        self.contract = with_arm_layout(contract or get_env_action_contract(env) or LEGACY_ACTION_CONTRACT)
        self.joint_abs = self.contract["action_mode"] == "joint_abs"
        self.start_tolerance_rad = float(start_tolerance_rad)
        self.open_fraction = float(gripper_open_fraction)
        self.closed_fraction = float(gripper_closed_fraction)
        self.period_s = 1. / control_hz
        self.clock, self.sleep = clock, sleep
        self.status = status if status is not None else (lambda text: operator_status(text))
        self.heartbeat = Heartbeat.from_env()
        self.phase = None
        self.episodes_started = 0

    # -- the session --------------------------------------------------------------

    def first_episode(self, reset) -> str:
        """``RESETTING`` (``reset()``), then ``READY``: ``"start"`` or ``"quit"``."""
        self._reset(reset)
        return self.ready()

    def between_episodes(self, reset, *, allow_next) -> str:
        """``POST_EPISODE`` (and ``GRIPPER_CHECK``), then ``RESETTING`` and ``READY``:
        ``"start"``, ``"quit"`` or ``"done"`` (budget reached, ended without a reset)."""
        result = self.post_episode(allow_next=allow_next)
        if result != "next":
            return result
        self._reset(reset)
        return self.ready()

    def stopping(self, reason) -> None:
        texts = {"operator_quit": "operator quit (q); finished episodes are on disk.",
                 "budget_exhausted": "episode budget reached; the session ends without a reset."}
        self._enter("STOPPING", reason, texts.get(reason, reason))

    # -- phases -------------------------------------------------------------------

    def post_episode(self, *, allow_next) -> str:
        """Hold the episode's last target until ``n`` (``"next"``, after the gripper check;
        ``"done"`` when ``allow_next`` is false) or ``q`` (``"quit"``)."""
        target = self.backend.session_hold_target()
        self._enter_post_episode(allow_next, "episode_ended")
        while True:
            start = self.clock()
            keys, _ = self._read_keys(read_leader=False)
            for key in keys:
                if key == "q":
                    self.stopping("operator_quit")
                    return "quit"
                if key in ("o", "c"):
                    self._command_gripper(key == "o")
                elif key == "n":
                    if not allow_next:
                        self.stopping("budget_exhausted")
                        return "done"
                    partly = self._partly_closed_grippers()
                    if not partly:
                        return "next"
                    result = self._gripper_check(target, partly)
                    if result != "back":
                        return result
                    self._enter_post_episode(allow_next, "gripper_check_cancelled")
                    break  # keys after the check's answer belong to the next poll
            self._tick(target)
            self._pace(start)

    def ready(self) -> str:
        """Hold the measured pose after the reset until ``SPACE`` (``"start"``) or ``q``."""
        target = self.backend.session_start_target()
        self._enter("READY", "awaiting_start",
                    "arm holds at home, teleop motion ignored; rearrange the scene. "
                    "SPACE = start the episode, q = quit.")
        shown, shown_at = frozenset(), -math.inf
        while True:
            start = self.clock()
            keys, reading = self._read_keys(read_leader=self.joint_abs)
            if "q" in keys:
                self.stopping("operator_quit")
                return "quit"
            gaps = self._misaligned(reading) if self.joint_abs else []
            self._tick(target)
            current = frozenset(arm_joint for arm_joint, _ in gaps)
            pressed_start = "space" in keys
            if (current != shown or (pressed_start and gaps)) and start - shown_at >= _LOCK_STATUS_PERIOD_S:
                self.status(self._lock_text(gaps, pressed_start))
                shown, shown_at = current, start
            if pressed_start and not gaps:
                self.episodes_started += 1
                self._enter("ACTIVE", "start_key",
                            f"episode {self.episodes_started} started; teleoperate now. s = success, "
                            "f = failure, r = rerecord, Esc = end the episode, q = quit after it.")
                return "start"
            self._pace(start)

    # -- helpers ------------------------------------------------------------------

    def _reset(self, reset) -> None:
        self._enter("RESETTING", "next_episode",
                    "the arm moves home, then the gripper opens; keys pressed now are discarded, wait for READY.")
        self.heartbeat.begin_long_operation()  # the reset homes the arm, far longer than a tick
        reset()
        self.heartbeat.beat()

    def _enter_post_episode(self, allow_next, reason) -> None:
        if allow_next:
            text = ("arm holds here, gripper unchanged, teleop motion ignored. o = open gripper, "
                    "c = close gripper, n = reset for the next episode, q = quit.")
        else:
            text = ("episode budget reached; arm holds here, gripper unchanged, teleop motion "
                    "ignored. o = open gripper, c = close gripper, n or q = end the session (no reset).")
        self._enter("POST_EPISODE", reason if allow_next else "budget_exhausted", text)

    def _gripper_check(self, target, partly) -> str:
        """``"next"`` (Enter), ``"quit"`` (q) or ``"back"`` (o opens in place; any other key)."""
        arms = [arm for arm, _, _ in partly]
        self._enter("GRIPPER_CHECK", "partly_closed", self._gripper_check_text(partly))
        while True:
            start = self.clock()
            keys, _ = self._read_keys(read_leader=False)
            for key in keys:
                if key == "q":
                    self.stopping("operator_quit")
                    return "quit"
                if key == "enter":
                    return "next"
                if key == "o":
                    self._command_gripper(True, arms=arms)
                return "back"
            self._tick(target)
            self._pace(start)

    def _gripper_check_text(self, partly) -> str:
        def width(entry):
            _, fraction, max_width_m = entry
            if fraction is None:
                return "no width feedback"
            if max_width_m is None:
                return f"{fraction:.0%} of its maximum width"
            return f"{fraction * max_width_m * 1000.:.1f} mm"

        if len(partly) == 1:
            arm = partly[0][0]
            # One arm is named "arm" (real_robot); a named arm of a two-arm station keeps its name.
            subject = "the gripper" if arm == "arm" else f"gripper {arm}"
            return (f"{subject} is partly closed ({width(partly[0])}) and may hold an object. "
                    "Enter = reset now (the gripper opens after the arm reaches home); "
                    "o = open it here first. Any other key = back to POST_EPISODE.")
        listed = ", ".join(f"{entry[0]} ({width(entry)})" for entry in partly)
        return (f"grippers {listed} are partly closed and may hold an object. Enter = reset now "
                "(the grippers open after the arms reach home); o = open them here first. "
                "Any other key = back to POST_EPISODE.")

    def _partly_closed_grippers(self) -> list:
        """``(arm, width fraction or None, max width m or None)`` of every gripper that is neither
        fully open nor fully closed; a gripper without width feedback counts as partly closed."""
        partly = []
        for arm, fraction, max_width_m in self.backend.session_grippers():
            if fraction is None or self.closed_fraction < fraction < self.open_fraction:
                partly.append((arm, fraction, max_width_m))
        return partly

    def _command_gripper(self, open_, arms=None) -> None:
        commanded = self.backend.session_gripper(open_, arms=arms)
        word = "open" if open_ else "close"
        if commanded:
            self.status(f"GRIPPER: {word} sent to {', '.join(commanded)}.")
        else:
            self.status(f"GRIPPER: no gripper to {word}; key ignored.")

    def _misaligned(self, reading) -> list:
        gaps = leader_follower_gaps(self.env, reading, self.contract)
        arms = list(self.contract["arm_names"])
        return [((arms[arm], joint), float(gaps[arm, joint]))
                for arm, joint in zip(*np.nonzero(gaps > self.start_tolerance_rad))]

    def _lock_text(self, gaps, pressed_start) -> str:
        if not gaps:
            return "READY: leader aligned with the follower; SPACE = start the episode, q = quit."
        joints = ", ".join(f"{arm} j{joint + 1} {gap:.2f} rad" for (arm, joint), gap in gaps)
        prefix = "SPACE ignored, start locked" if pressed_start else "start locked"
        return (f"READY: {prefix}: leader joints further than {self.start_tolerance_rad:g} rad from "
                f"the follower: {joints}; move the leader onto the follower.")

    def _read_keys(self, *, read_leader):
        """The keys pressed since the previous read, in order, and the leader reading when
        ``read_leader`` (or when the motion device is the only event device)."""
        from real_world_rl.teleop.core.composite import ToggleableTeleopSource

        keys, reading = [], None
        if isinstance(self.source, CompositeTeleopSource):
            events, motion = self.source.events, self.source.motion
        else:
            events, motion = None, self.source
        if events is not None:
            poll_keys = getattr(events, "poll_keys", None)
            keys.extend(poll_keys() if callable(poll_keys) else _keys_from_events(events.poll().events))
        if read_leader or events is None:
            # The toggle released: neither the onset policy nor a take-over runs on this read.
            reading = (motion.poll_with_toggle(False) if isinstance(motion, ToggleableTeleopSource)
                       else motion.poll())
            keys.extend(_keys_from_events(reading.events))
        return keys, reading

    def _tick(self, target) -> None:
        self.backend.session_hold_tick(target)
        self.heartbeat.beat()

    def _pace(self, start) -> None:
        self.sleep(max(0., self.period_s - (self.clock() - start)))

    def _enter(self, phase, reason, text) -> None:
        self.phase = phase
        emit = getattr(self.backend, "session_event", None)
        if callable(emit):
            emit(phase, reason)
        self.status(f"{phase}: {text}")
