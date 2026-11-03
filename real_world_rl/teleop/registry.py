"""Name -> ``TeleopSource`` registry behind ``device="<name>"``.

Built-in sources are imported on first use of their name, so importing this package pulls in
neither the Pico ZMQ client nor the SpaceMouse, Dynamixel or pynput SDKs (nor the device layer).
"""
from __future__ import annotations

import importlib
from typing import Any, Callable, Dict, List, Type

from real_world_rl.teleop.base import TeleopSource

_REGISTRY: Dict[str, Type[TeleopSource]] = {}
_BUILTIN_MODULES: Dict[str, str] = {
    "pico": "real_world_rl.teleop.pico",
    "robot": "real_world_rl.teleop.part",
    "spacemouse": "real_world_rl.teleop.spacemouse",
    "fake": "real_world_rl.teleop.fake",
    "gello": "real_world_rl.teleop.gello",
    "leader_arm": "real_world_rl.teleop.leader_arm",
    "keyboard": "real_world_rl.teleop.keyboard",
    "composite": "real_world_rl.teleop.composite",
    "leader_cartesian": "real_world_rl.teleop.leader_cartesian",
    "bimanual": "real_world_rl.teleop.bimanual",
}


def register_teleop_source(name: str) -> Callable[[Type[TeleopSource]], Type[TeleopSource]]:
    """Class decorator: makes the ``TeleopSource`` subclass available as ``device=name``."""

    def decorator(source_cls: Type[TeleopSource]) -> Type[TeleopSource]:
        if not (isinstance(source_cls, type) and issubclass(source_cls, TeleopSource)):
            raise TypeError(f"{source_cls!r} is not a TeleopSource subclass")
        registered = _REGISTRY.get(name)
        if registered is not None and registered is not source_cls:
            raise ValueError(f"teleop source {name!r} is already registered to {registered.__name__}")
        _REGISTRY[name] = source_cls
        return source_cls

    return decorator


def teleop_source_names() -> List[str]:
    return sorted(set(_REGISTRY) | set(_BUILTIN_MODULES))


def make_teleop_source(name: str, env: Any, **kwargs: Any) -> TeleopSource:
    """Builds the source registered as ``name`` through its ``from_env(env, **kwargs)``."""
    if name not in _REGISTRY and name in _BUILTIN_MODULES:
        importlib.import_module(_BUILTIN_MODULES[name])  # registers itself
    if name not in _REGISTRY:
        raise ValueError(f"unknown teleop source {name!r}; available: {teleop_source_names()}")
    return _REGISTRY[name].from_env(env, **kwargs)
