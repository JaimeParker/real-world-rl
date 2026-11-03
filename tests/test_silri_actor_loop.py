"""SilriActorLoop: pushed fields, the one-step delayed ``intervention_onset`` tag and the
flush at the end of an episode."""
from __future__ import annotations

from typing import Optional, Sequence

import torch
from rl_garden_real_world.loops.hil_serl import HilSerlActorLoop
from rl_garden_real_world.loops.silri import SilriActorLoop

_POLICY_ACTION = torch.full((1, 3), 0.25)
_HUMAN_ACTION = torch.full((1, 3), -0.5)


class _ScriptedInterventionEnv:
    """Intervenes on step ``i`` (0-based) when ``intervened[i]``, with the info keys the
    intervention wrapper writes. Terminates every ``terminate_every`` steps."""

    num_envs = 1

    def __init__(self, intervened: Sequence[bool], terminate_every: Optional[int] = None):
        self.intervened = list(intervened)
        self.terminate_every = terminate_every
        self.steps = 0

    def reset(self, *, seed=None):
        del seed
        return torch.zeros(1, 4), {}

    def step(self, action):
        intervened = self.intervened[self.steps]
        self.steps += 1
        info = {"policy_action": action, "intervened": intervened}
        if intervened:
            info["intervene_action"] = _HUMAN_ACTION
        terminated = self.terminate_every is not None and self.steps % self.terminate_every == 0
        return (
            torch.full((1, 4), float(self.steps)),
            torch.ones(1),
            torch.tensor([terminated]),
            torch.tensor([False]),
            info,
        )


class _Policy:
    def eval(self):
        pass

    def load_state_dict(self, state_dict):
        del state_dict

    def predict(self, obs, deterministic=False):
        del obs, deterministic
        return _POLICY_ACTION.clone()


class _SyncClient:
    def __init__(self):
        self.pushed: list[dict] = []

    def start(self):
        pass

    def stop(self):
        pass

    def push_transition(self, transition):
        self.pushed.append(transition)

    def latest_policy_params(self):
        return None


def _run(intervened, terminate_every=None, total_steps=None):
    sync_client = _SyncClient()
    env = _ScriptedInterventionEnv(intervened, terminate_every)
    loop = SilriActorLoop(env, _Policy(), sync_client, control_hz=1_000_000.0)
    loop.run(total_steps=len(intervened) if total_steps is None else total_steps)
    return loop, sync_client.pushed


def test_silri_actor_loop_is_a_hil_serl_actor_loop():
    assert issubclass(SilriActorLoop, HilSerlActorLoop)


def test_onset_tags_the_transition_before_each_takeover():
    flags = [False, False, True, True, False, True]
    _, pushed = _run(flags, terminate_every=len(flags))

    assert [t["intervened"] for t in pushed] == flags
    assert [t["intervention_onset"] for t in pushed] == [False, True, False, False, True, False]


def test_pushed_fields_carry_the_executed_and_the_proposed_action():
    _, pushed = _run([False, True], terminate_every=2)

    assert set(pushed[0]) == {
        "obs",
        "next_obs",
        "action",
        "reward",
        "done",
        "intervened",
        "policy_action",
        "intervention_onset",
    }
    torch.testing.assert_close(pushed[0]["action"], _POLICY_ACTION)
    torch.testing.assert_close(pushed[1]["action"], _HUMAN_ACTION)
    for transition in pushed:
        torch.testing.assert_close(transition["policy_action"], _POLICY_ACTION)
    torch.testing.assert_close(pushed[1]["done"], torch.tensor([True]))


def test_the_latest_transition_is_held_until_the_next_one_is_pushed():
    sync_client = _SyncClient()
    loop = SilriActorLoop(
        _ScriptedInterventionEnv([]), _Policy(), sync_client, control_hz=1_000_000.0
    )
    first, second = {"intervened": False, "id": 1}, {"intervened": True, "id": 2}

    loop._push(first)
    assert sync_client.pushed == [] and loop._held_transition is first
    loop._push(second)
    assert sync_client.pushed == [first] and loop._held_transition is second


def test_stopping_after_a_non_terminal_step_still_pushes_the_held_row():
    loop, pushed = _run([False, False, True], total_steps=3)

    assert len(pushed) == 3
    assert loop._held_transition is None
    torch.testing.assert_close(pushed[-1]["next_obs"], torch.full((1, 4), 3.0))
    assert [t["intervention_onset"] for t in pushed] == [False, True, False]


def test_run_end_hook_runs_before_the_sync_client_stops():
    events = []

    class _OrderedSyncClient(_SyncClient):
        def push_transition(self, transition):
            events.append("push")

        def stop(self):
            events.append("stop")

    loop = SilriActorLoop(
        _ScriptedInterventionEnv([False]), _Policy(), _OrderedSyncClient(), control_hz=1_000_000.0
    )
    loop.run(total_steps=1)

    assert events == ["push", "stop"]


def test_episode_end_flushes_and_onset_never_crosses_episodes():
    # Episode 1: [F, F]; episode 2 starts with a takeover.
    loop, pushed = _run([False, False, True, False], terminate_every=2, total_steps=4)

    assert len(pushed) == 4
    assert loop._held_transition is None
    assert [t["intervention_onset"] for t in pushed] == [False, False, False, False]


def test_actor_loop_calls_the_run_end_hook_before_stopping_even_on_errors():
    from rl_garden_real_world.loops.actor import ActorLoop

    events = []

    class _OrderedSyncClient(_SyncClient):
        def stop(self):
            events.append("stop")

    class _Hooked(ActorLoop):
        def _on_run_end(self):
            events.append("run_end")

    class _FailingEnv(_ScriptedInterventionEnv):
        def step(self, action):
            raise RuntimeError("robot fault")

    loop = _Hooked(_FailingEnv([False]), _Policy(), _OrderedSyncClient(), control_hz=1_000_000.0)
    try:
        loop.run(total_steps=1)
    except RuntimeError:
        pass
    assert events == ["run_end", "stop"]
