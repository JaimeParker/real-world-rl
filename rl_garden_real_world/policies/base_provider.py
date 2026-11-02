"""Base action providers for residual RL.

A provider maps an observation to the base policy's normalized env action, a
``(B, A)`` tensor in ``[-1, 1]`` (``B`` is the observation's batch size, 1 on a
robot). Providers live only in the processes that act (the actor, eval and
deploy) and are never attributes of a policy or an algorithm: the learner trains
on the base actions the actor stored in replay, so it never loads the base.

The CLI, ``config.json`` and checkpoint metadata name a provider with a spec
string ``kind[:path][?key=value&key=value]``::

    zero
    checkpoint:/runs/bc/checkpoints/final.pt?algo=real_world_bc
    checkpoint:/runs/sim/checkpoints/final.pt?algo=rlpd_hybrid
    act:/data/act/policy_best.ckpt?temporal_agg=1&temporal_agg_k=0.01
"""
from __future__ import annotations

from collections.abc import Mapping
from pathlib import Path
from typing import Any, Optional, Protocol, runtime_checkable

import torch
from gymnasium import spaces
from gymnasium.vector.utils import batch_space

from rl_garden.common.types import Obs

PROVIDER_KINDS = ("zero", "checkpoint", "act")
_SPEC_USAGE = (
    "expected 'zero', 'checkpoint:<path>?algo=<registered algorithm>' or "
    "'act:<path>[?temporal_agg=1&temporal_agg_k=0.01&state_dict_key=ema_agent]'"
)


@runtime_checkable
class BaseActionProvider(Protocol):
    """What the residual actor loop, eval and deploy need from a base policy."""

    action_dim: int

    def select_action(self, obs: Obs) -> torch.Tensor:
        """``(B, action_dim)`` normalized env action in ``[-1, 1]`` for ``obs``,
        computed without gradients. Stateful providers advance by one step."""
        ...

    def reset(self) -> None:
        """Clear per-episode state; called when an episode ends and, by the residual
        actor loop, after every intervened step (so a stateful base replans)."""
        ...

    def to(self, device: torch.device | str) -> "BaseActionProvider":
        ...

    def spec(self) -> dict[str, Any]:
        """JSON-serialisable identity: ``{"kind", "path", "options"}``."""
        ...


def _obs_to_device(obs: Obs, device: torch.device) -> Obs:
    if isinstance(obs, Mapping):
        return {key: value.to(device) for key, value in obs.items()}
    return obs.to(device)


def _batch_size(obs: Obs) -> int:
    sample = next(iter(obs.values())) if isinstance(obs, Mapping) else obs
    return int(sample.shape[0])


class ZeroBaseProvider:
    """Base action ``0`` everywhere: residual RL degenerates to plain RL on
    ``scale``-bounded actions. For tests and ablations."""

    def __init__(self, action_dim: int, device: torch.device | str = "cpu") -> None:
        self.action_dim = int(action_dim)
        self.device = torch.device(device)

    def select_action(self, obs: Obs) -> torch.Tensor:
        return torch.zeros(_batch_size(obs), self.action_dim, device=self.device)

    def reset(self) -> None:
        return None

    def to(self, device: torch.device | str) -> "ZeroBaseProvider":
        self.device = torch.device(device)
        return self

    def spec(self) -> dict[str, Any]:
        return {"kind": "zero", "path": None, "options": {}}


class _SpacesOnlyEnv:
    """The surface an rl-garden agent reads from its env at construction."""

    num_envs = 1

    def __init__(self, observation_space: spaces.Space, action_space: spaces.Box) -> None:
        self.single_observation_space = observation_space
        self.single_action_space = action_space
        self.observation_space = batch_space(observation_space, 1)
        self.action_space = batch_space(action_space, 1)


# Applied after the run's config.json and checkpoint hyperparameters: the provider only
# needs the policy, so the agent is built with a token replay buffer and no checkpointing.
_INFERENCE_ONLY_ARGS = {
    "buffer_size": 8,
    "buffer_device": "cpu",
    "offline_dataset": None,
    "load_checkpoint": None,
    "checkpoint_dir": None,
    "checkpoint_freq": 0,
    "save_replay_buffer": False,
    "save_final_checkpoint": False,
    "log_type": "none",
}


def _restore_real_world_bc_agent(path: str, env: _SpacesOnlyEnv) -> Any:
    """A ``real_world_bc`` agent (the real-world registry's BC, built with rl-garden's
    offline ``build_bc``) with the Args of the run that wrote ``path``."""
    import json

    from rl_garden.common.checkpoint import load_checkpoint_file
    from rl_garden.training.offline.entrypoints.bc import build_bc
    from rl_garden_real_world.deploy import _apply_mapping
    from rl_garden_real_world.training.registry import registry

    registry.discover()
    training_args = registry.entries()["real_world_bc"].args_cls()
    config_path = Path(path).parent.parent / "config.json"
    if config_path.exists():
        config = json.loads(config_path.read_text(encoding="utf-8"))
        _apply_mapping(training_args, config.get("inputs", config.get("args", config)))
    hyperparameters = load_checkpoint_file(path, map_location="cpu").get("metadata", {}).get(
        "hyperparameters", {}
    )
    if isinstance(hyperparameters, Mapping):
        _apply_mapping(training_args, hyperparameters)
    _apply_mapping(training_args, _INFERENCE_ONLY_ARGS)
    return build_bc(training_args, env, None, eval_env=None)


def _restore_online_agent(path: str, algo: str, env: _SpacesOnlyEnv) -> Any:
    """An rl-garden online-registry agent (``rlpd``, ``rlpd_hybrid``, ...), rebuilt the way
    ``deploy`` rebuilds one: the entry's Args from the run's ``config.json`` and the
    checkpoint's hyperparameters, then the entry's ``build_agent``."""
    from rl_garden_real_world.deploy import (
        DeployArgs,
        _resolve_build_agent,
        restore_training_args,
    )

    training_args = restore_training_args(DeployArgs(checkpoint=path, algo=algo))
    return _resolve_build_agent(algo)(training_args, env, None, logger=None, checkpoint_dir=None)


class CheckpointBaseProvider:
    """The deterministic action of an rl-garden checkpoint's policy.

    ``algo`` names the algorithm that wrote the checkpoint: ``real_world_bc``, or any
    online-registry name with an agent builder (``rlpd``, ``rlpd_hybrid``, ...). The
    checkpoint must have been trained on the same observation and action spaces (the load
    checks them). The run's ``config.json`` next to ``checkpoints/`` is read when present.
    """

    def __init__(
        self,
        path: str,
        algo: str,
        *,
        observation_space: spaces.Space,
        action_space: spaces.Box,
        device: torch.device | str = "cpu",
    ) -> None:
        self.path = str(path)
        self.algo = algo
        self.action_dim = int(action_space.shape[0])
        self.device = torch.device(device)
        env = _SpacesOnlyEnv(observation_space, action_space)
        if algo == "real_world_bc":
            agent = _restore_real_world_bc_agent(self.path, env)
        else:
            agent = _restore_online_agent(self.path, algo, env)
        agent.load(self.path, load_replay_buffer=False, load_optimizers=False)
        self._policy = agent.policy.to(self.device).eval()

    @torch.no_grad()
    def select_action(self, obs: Obs) -> torch.Tensor:
        action = self._policy.predict(_obs_to_device(obs, self.device), deterministic=True)
        return action.clamp(-1.0, 1.0)

    def reset(self) -> None:
        return None

    def to(self, device: torch.device | str) -> "CheckpointBaseProvider":
        self.device = torch.device(device)
        self._policy.to(self.device)
        return self

    def spec(self) -> dict[str, Any]:
        return {"kind": "checkpoint", "path": self.path, "options": {"algo": self.algo}}


class ACTBaseProvider:
    """An ACT checkpoint (``rl_garden.models.act.provider.ACTBaseActionProvider``):
    action chunks with optional temporal ensembling, de-normalised by the checkpoint's
    statistics, then clipped to ``[-1, 1]``. Single env; ``reset`` clears the chunk
    and ensemble state. State comes from ``obs["state"]``."""

    def __init__(
        self,
        path: str,
        *,
        observation_space: spaces.Space,
        action_space: spaces.Box,
        temporal_agg: bool = True,
        temporal_agg_k: float = 0.01,
        state_dict_key: str = "ema_agent",
        device: torch.device | str = "cpu",
    ) -> None:
        from rl_garden.models.act.provider import ACTBaseActionProvider

        self.path = str(path)
        self.action_dim = int(action_space.shape[0])
        self._options = {
            "temporal_agg": bool(temporal_agg),
            "temporal_agg_k": float(temporal_agg_k),
            "state_dict_key": state_dict_key,
        }
        self._act = ACTBaseActionProvider.from_checkpoint(
            observation_space=observation_space,
            action_space=action_space,
            ckpt_path=self.path,
            state_dict_key=state_dict_key,
            temporal_agg=bool(temporal_agg),
            temporal_agg_k=float(temporal_agg_k),
            device=device,
        )

    def select_action(self, obs: Obs) -> torch.Tensor:
        action = self._act.select_action(_obs_to_device(obs, self._act.device))
        return action.clamp(-1.0, 1.0)

    def reset(self) -> None:
        self._act.reset()

    def to(self, device: torch.device | str) -> "ACTBaseProvider":
        self._act.to(device)
        return self

    def spec(self) -> dict[str, Any]:
        return {"kind": "act", "path": self.path, "options": dict(self._options)}


def parse_base_provider_spec(spec: str) -> dict[str, Any]:
    """``"kind[:path][?k=v&k2=v2]"`` -> ``{"kind": str, "path": str | None, "options": {k: v}}``
    (option values stay strings). Raises ``ValueError`` on an empty spec, an unknown kind or
    a malformed option."""
    text = spec.strip()
    if not text:
        raise ValueError(f"empty base provider spec; {_SPEC_USAGE}")
    head, _, query = text.partition("?")
    kind, _, path = head.partition(":")
    kind = kind.strip()
    if kind not in PROVIDER_KINDS:
        raise ValueError(f"unknown base provider kind {kind!r} in {spec!r}; {_SPEC_USAGE}")
    options: dict[str, str] = {}
    if query:
        for item in query.split("&"):
            key, separator, value = item.partition("=")
            if not separator or not key:
                raise ValueError(f"malformed option {item!r} in base provider spec {spec!r}")
            options[key] = value
    return {"kind": kind, "path": path or None, "options": options}


def _parse_bool(value: str, *, name: str) -> bool:
    lowered = value.strip().lower()
    if lowered in ("1", "true", "yes"):
        return True
    if lowered in ("0", "false", "no"):
        return False
    raise ValueError(f"base provider option {name}={value!r} is not a boolean")


def _reject_unknown_options(kind: str, options: Mapping[str, str]) -> None:
    if options:
        raise ValueError(f"unknown option(s) {sorted(options)} for a {kind!r} base provider; {_SPEC_USAGE}")


def make_base_provider(
    spec: str | Mapping[str, Any],
    *,
    device: torch.device | str,
    observation_space: spaces.Space,
    action_space: spaces.Box,
) -> BaseActionProvider:
    """Build the provider a spec string (or a parsed spec dict) names, for an env with
    ``observation_space`` / ``action_space`` (single-env spaces), on ``device``."""
    parsed = parse_base_provider_spec(spec) if isinstance(spec, str) else dict(spec)
    kind: str = parsed["kind"]
    path: Optional[str] = parsed.get("path")
    options = dict(parsed.get("options") or {})
    if kind == "zero":
        if path is not None:
            raise ValueError(f"the 'zero' base provider takes no path, got {path!r}")
        _reject_unknown_options(kind, options)
        return ZeroBaseProvider(int(action_space.shape[0]), device=device)
    if path is None:
        raise ValueError(f"a {kind!r} base provider needs a path: '{kind}:<path>'; {_SPEC_USAGE}")
    if kind == "checkpoint":
        algo = options.pop("algo", None)
        if not algo:
            raise ValueError(f"a 'checkpoint' base provider needs ?algo=<name>; {_SPEC_USAGE}")
        _reject_unknown_options(kind, options)
        return CheckpointBaseProvider(
            path,
            algo,
            observation_space=observation_space,
            action_space=action_space,
            device=device,
        )
    temporal_agg = _parse_bool(options.pop("temporal_agg", "1"), name="temporal_agg")
    temporal_agg_k = float(options.pop("temporal_agg_k", "0.01"))
    state_dict_key = options.pop("state_dict_key", "ema_agent")
    _reject_unknown_options(kind, options)
    return ACTBaseProvider(
        path,
        observation_space=observation_space,
        action_space=action_space,
        temporal_agg=temporal_agg,
        temporal_agg_k=temporal_agg_k,
        state_dict_key=state_dict_key,
        device=device,
    )


class ResidualPolicyWithBase:
    """A residual policy composed with its base provider, for the processes that act
    without a learner (``residual_hil_serl --role eval`` and ``deploy``).

    ``predict(obs, deterministic)`` has the plain ``BasePolicy`` signature: it queries
    the provider and passes its action as ``base_actions``. Call ``reset_episode`` at the
    start of every episode so a stateful provider (ACT) starts a fresh chunk.
    """

    def __init__(self, policy: Any, provider: BaseActionProvider) -> None:
        self.policy = policy
        self.provider = provider

    def _policy_device(self) -> torch.device:
        return next(self.policy.parameters()).device

    def predict(self, obs: Obs, deterministic: bool = False) -> torch.Tensor:
        device = self._policy_device()
        with torch.no_grad():
            base_actions = self.provider.select_action(obs).to(device)
            return self.policy.predict(
                _obs_to_device(obs, device), deterministic, base_actions=base_actions
            )

    def reset_episode(self) -> None:
        self.provider.reset()

    def eval(self) -> "ResidualPolicyWithBase":
        self.policy.eval()
        return self

    def to(self, device: torch.device | str) -> "ResidualPolicyWithBase":
        self.policy.to(device)
        self.provider.to(device)
        return self

    def state_dict(self) -> dict[str, Any]:
        return self.policy.state_dict()
