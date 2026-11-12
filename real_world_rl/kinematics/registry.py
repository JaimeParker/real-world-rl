"""Named, numpy-only forward kinematics for leader arms."""
from __future__ import annotations

from collections.abc import Callable, Sequence
from typing import Protocol, TypeVar

import numpy as np

from real_world_rl.kinematics.panda import PandaKinematics


class Kinematics(Protocol):
    NUM_JOINTS: int

    def ee_transform(self, q: Sequence[float]) -> np.ndarray:
        """Return the base-frame end-effector transform as a (4, 4) array."""
        ...


_T = TypeVar("_T", bound=type[Kinematics])
_REGISTRY: dict[str, type[Kinematics]] = {}


def register(name: str) -> Callable[[_T], _T]:
    """Register a zero-argument kinematics class under ``name``."""
    key = name.lower()

    def decorator(cls: _T) -> _T:
        if key in _REGISTRY:
            raise ValueError(f"kinematics {key!r} is already registered")
        _REGISTRY[key] = cls
        return cls

    return decorator


def kinematics_for(name: str) -> Kinematics:
    """Construct registered kinematics; report the available names on a typo."""
    try:
        cls = _REGISTRY[name.lower()]
    except KeyError:
        raise ValueError(
            f"unknown kinematics {name!r}; registered: {kinematics_names()}"
        ) from None
    return cls()


def kinematics_names() -> list[str]:
    return sorted(_REGISTRY)


register("panda")(PandaKinematics)
