"""Registers the real-world methods that have an agent builder on rl-garden's
online registry (``residual_hil_serl``, ``residual_hil_serl_hybrid``, ``silri``), so
``rl_garden_real_world.deploy`` can rebuild their agents from a checkpoint. Loaded
through the ``rlgarden.algorithms`` entry-point group (see ``docs/guides/extending.md``
in rl-garden). The residual classes set ``checkpoint_eval_unsupported_reason`` so
``rl-garden-eval`` refuses them: they need a base provider that only the real-world eval
role and deploy build.

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

from rl_garden_real_world.training.entrypoints.residual_hil_serl import (
    ResidualHilSerlArgs,
    ResidualHilSerlHybridArgs,
    build_residual_hil_serl,
    run_residual_hil_serl,
)
from rl_garden_real_world.training.entrypoints.silri import (
    SilriArgs,
    build_silri,
    run_silri,
)

AlgorithmRow = tuple[type, Callable[..., Any], Callable[[], type], Callable[..., Any]]


def _residual_rlpd_algorithm_cls() -> type:
    from rl_garden_real_world.algorithms.residual import ResidualRLPD

    return ResidualRLPD


def _residual_rlpd_hybrid_algorithm_cls() -> type:
    from rl_garden_real_world.algorithms.residual import ResidualRLPDHybrid

    return ResidualRLPDHybrid


def _silri_algorithm_cls() -> type:
    from rl_garden_real_world.algorithms.silri import SiLRI

    return SiLRI


# name -> (args_cls, run_fn, algorithm_cls thunk, build_agent)
_ALGORITHMS: dict[str, AlgorithmRow] = {
    # One name per algorithm class (rl-garden matches checkpoints by class name and takes
    # one Args class per name); both build with build_residual_hil_serl, which reads
    # residual_variant. The real-world CLI only has residual_hil_serl.
    "residual_hil_serl": (
        ResidualHilSerlArgs,
        run_residual_hil_serl,
        _residual_rlpd_algorithm_cls,
        build_residual_hil_serl,
    ),
    "residual_hil_serl_hybrid": (
        ResidualHilSerlHybridArgs,
        run_residual_hil_serl,
        _residual_rlpd_hybrid_algorithm_cls,
        build_residual_hil_serl,
    ),
    # HIL-RL (SiLRI).
    "silri": (SilriArgs, run_silri, _silri_algorithm_cls, build_silri),
}


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
