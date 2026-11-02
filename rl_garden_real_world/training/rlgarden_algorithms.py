"""Registers the real-world methods that have an agent builder on rl-garden's
online registry, so ``rl-garden-eval`` (and ``deploy``) can rebuild their
agents from a checkpoint. Loaded through the ``rlgarden.algorithms``
entry-point group (see ``docs/guides/extending.md`` in rl-garden).

The real-world registry (``training/registry.py``) stays the CLI owner of
these methods: the ``run_fn`` registered here is the same function, kept so a
checkpoint of one of them resolves to the right Args class and builder.

To expose a method, add a row to ``_ALGORITHMS``::

    "my_method": (MyMethodArgs, run_my_method, _my_method_algorithm_cls, build_my_method),

``algorithm_cls`` is a thunk (import the class inside it) and ``build_agent``
is ``build_agent(args, env, eval_env, logger, checkpoint_dir)``.
"""
from __future__ import annotations

from collections.abc import Callable, Mapping
from typing import Any

AlgorithmRow = tuple[type, Callable[..., Any], Callable[[], type], Callable[..., Any]]

# name -> (args_cls, run_fn, algorithm_cls thunk, build_agent)
_ALGORITHMS: dict[str, AlgorithmRow] = {}


def register_real_world_algorithms(registry, algorithms: Mapping[str, AlgorithmRow] | None = None) -> None:
    """Register ``algorithms`` (default: ``_ALGORITHMS``) on ``registry``.

    Idempotent: a name already registered with the same Args class is skipped
    (the entry-point module may be imported more than once); one registered
    with a different Args class is left to ``registry.register`` to reject.
    """
    table = _ALGORITHMS if algorithms is None else algorithms
    for name, (args_cls, run_fn, algorithm_cls, build_agent) in table.items():
        existing = registry.entries().get(name)
        if existing is not None and existing.args_cls is args_cls:
            continue
        registry.register(
            name, args_cls, run_fn, algorithm_cls=algorithm_cls, build_agent=build_agent
        )


def _register_on_online_registry() -> None:
    from rl_garden.training.online.registry import registry

    register_real_world_algorithms(registry)


_register_on_online_registry()
