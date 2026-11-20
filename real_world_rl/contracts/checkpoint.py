"""Keep the environment's action interpretation across policy/checkpoint loads."""
from __future__ import annotations

import torch
import dataclasses

_ACTION_BUFFERS = frozenset({"action_scale", "action_bias", "action_low", "action_high",
                             "residual_scale", "_discrete_action_values"})


def _action_buffers(policy):
    state_dict = getattr(policy, "state_dict", None)
    if state_dict is None:
        return {}
    return {name: value.detach().cpu().clone() for name, value in state_dict().items()
            if name.rsplit(".", 1)[-1] in _ACTION_BUFFERS and isinstance(value, torch.Tensor)}


def _validate_action_buffers(policy, expected):
    actual = _action_buffers(policy)
    changed = [name for name, value in expected.items()
               if name not in actual or value.shape != actual[name].shape
               or not torch.equal(value, actual[name])]
    if changed:
        raise ValueError(f"loaded policy action bounds/scales differ from the constructed environment: {changed}")


def load_with_action_contract(agent, path, **kwargs):
    """Load once, preserving policy-specific continuous/gripper/residual action buffers.

    Legacy checkpoints use the original loader and remain supported. The comparison
    uses the freshly constructed policy's own buffer shapes, including partial
    continuous action heads, rather than assuming the full environment action width.
    """
    expected = _action_buffers(getattr(agent, "policy", None))
    result = agent.load(path, **kwargs)
    _validate_action_buffers(getattr(agent, "policy", None), expected)
    return result


def load_policy_state_with_action_contract(policy, state, **kwargs):
    """Apply actor synchronization parameters with the same action-buffer guard."""
    expected = _action_buffers(policy)
    result = policy.load_state_dict(state, **kwargs)
    _validate_action_buffers(policy, expected)
    return result


def build_agent_with_action_contract(builder, args, *positional, **kwargs):
    """Keep upstream builders' checkpoint restore behind the shared load guard."""
    path = getattr(args, "load_checkpoint", None)
    if path is None:
        return builder(args, *positional, **kwargs)
    agent = builder(dataclasses.replace(args, load_checkpoint=None), *positional, **kwargs)
    load_with_action_contract(agent, path, load_replay_buffer=args.load_replay_buffer)
    return agent
