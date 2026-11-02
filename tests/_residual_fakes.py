"""Shared fakes for the residual tests: a dict-observation env, a recording base
provider, a fake sync client, agent factories and replay fillers."""
from __future__ import annotations

from collections.abc import Iterable
from typing import Any, Optional

import gymnasium as gym
import torch
from gymnasium import spaces
from gymnasium.vector.utils import batch_space
from rl_garden_real_world.algorithms.residual import ResidualRLPD, ResidualRLPDHybrid

STATE_DIM = 4
ACTION_DIM = 3  # two arm dims + the gripper


class DictEnv(gym.Env):
    """One-env torch env with ``{"state"}`` observations. Reset observations are filled
    with ``100 + reset count`` and step observations with the step count, so a recorded
    observation tells which call produced it. Terminates every ``terminate_every`` steps.
    The steps numbered in ``intervene_steps`` (1-based, counted across episodes) report a
    human takeover: ``info["intervene_action"]`` is ``HUMAN_ACTION_VALUE`` on every dim."""

    num_envs = 1
    HUMAN_ACTION_VALUE = 0.5

    def __init__(
        self,
        action_dim: int = ACTION_DIM,
        terminate_every: Optional[int] = None,
        info: Optional[dict] = None,
        intervene_steps: Iterable[int] = (),
    ) -> None:
        self.single_observation_space = spaces.Dict(
            {"state": spaces.Box(-1.0, 1.0, (STATE_DIM,), dtype="float32")}
        )
        self.single_action_space = spaces.Box(-1.0, 1.0, (action_dim,), dtype="float32")
        self.observation_space = batch_space(self.single_observation_space, 1)
        self.action_space = batch_space(self.single_action_space, 1)
        self.terminate_every = terminate_every
        self.info = dict(info or {})
        self.intervene_steps = frozenset(intervene_steps)
        self.resets = 0
        self.steps = 0
        self.actions: list[torch.Tensor] = []
        self.closed = False

    def reset(self, *, seed=None, options=None):
        del seed, options
        self.resets += 1
        return {"state": torch.full((1, STATE_DIM), 100.0 + self.resets)}, {}

    def step(self, action):
        self.steps += 1
        self.actions.append(action)
        terminated = self.terminate_every is not None and self.steps % self.terminate_every == 0
        info = dict(self.info)
        if self.steps in self.intervene_steps:
            info["intervene_action"] = torch.full(action.shape, self.HUMAN_ACTION_VALUE)
        return (
            {"state": torch.full((1, STATE_DIM), float(self.steps))},
            torch.ones(1),
            torch.tensor([terminated]),
            torch.tensor([False]),
            info,
        )

    def close(self):
        self.closed = True


class RecordingProvider:
    """Base action ``state[0, 0] / 1000`` on every dim; records the first state entry of
    every observation it is asked about, counts resets and records the reset count at
    every query (``reset_calls_at_select``), so a test can tell which resets preceded it."""

    def __init__(self, action_dim: int = ACTION_DIM) -> None:
        self.action_dim = action_dim
        self.seen: list[float] = []
        self.reset_calls = 0
        self.reset_calls_at_select: list[int] = []

    @staticmethod
    def value_for(marker: float) -> float:
        return marker / 1000.0

    def select_action(self, obs) -> torch.Tensor:
        marker = float(obs["state"][0, 0])
        self.seen.append(marker)
        self.reset_calls_at_select.append(self.reset_calls)
        return torch.full((1, self.action_dim), self.value_for(marker))

    def reset(self) -> None:
        self.reset_calls += 1

    def to(self, device):
        return self

    def spec(self) -> dict[str, Any]:
        return {"kind": "recording", "path": None, "options": {}}


class FakeSyncClient:
    def __init__(self, params=None) -> None:
        self.params = params
        self.pushed: list[dict] = []

    def start(self):
        pass

    def stop(self):
        pass

    def push_transition(self, transition):
        self.pushed.append(transition)

    def latest_policy_params(self):
        return self.params


def residual_agent(cls=ResidualRLPD, env=None, **overrides):
    kwargs: dict[str, Any] = dict(
        device="cpu",
        buffer_device="cpu",
        buffer_size=64,
        batch_size=8,
        learning_starts=1,
        utd=4,
        eval_freq=0,
        log_freq=0,
        net_arch=[16],
        seed=0,
    )
    if cls is ResidualRLPDHybrid:
        kwargs["discrete_hidden_dim"] = 8
    kwargs.update(overrides)
    return cls(env=env or DictEnv(), **kwargs)


def residual_row(
    *,
    intervened: bool = False,
    base: float = 0.0,
    human_offset: float = 0.0,
    gripper: float = 1.0,
    grasp_penalty: Optional[float] = None,
) -> dict[str, Any]:
    """One transition with the residual extras: base action ``base`` on the arm dims, the
    stored action ``base + human_offset`` on the arm dims and ``gripper`` last."""
    base_actions = torch.full((1, ACTION_DIM), base)
    action = base_actions.clone()
    action[:, :-1] += human_offset
    action[:, -1] = gripper
    row: dict[str, Any] = {
        "obs": {"state": torch.randn(1, STATE_DIM)},
        "next_obs": {"state": torch.randn(1, STATE_DIM)},
        "action": action,
        "reward": torch.ones(1),
        "done": torch.zeros(1),
        "intervened": intervened,
        "policy_action": base_actions.clone(),
        "base_actions": base_actions,
        "next_base_actions": base_actions.clone(),
    }
    if grasp_penalty is not None:
        row["grasp_penalty"] = torch.full((1,), grasp_penalty)
    return row


def add_rows(adder, n: int, **row_kwargs) -> None:
    for _ in range(n):
        row = residual_row(**row_kwargs)
        adder(
            row.pop("obs"),
            row.pop("next_obs"),
            row.pop("action"),
            row.pop("reward"),
            row.pop("done"),
            intervened=row.pop("intervened"),
            policy_action=row.pop("policy_action"),
            **row,
        )
