"""Name -> ``TeleopSource`` registry behind ``device="<name>"``.

Built-in sources are imported on first use of their name, so importing this package pulls in
neither the Pico ZMQ client nor the SpaceMouse, Dynamixel or pynput SDKs (nor the device layer,
nor termios).
"""
from __future__ import annotations

import importlib
from typing import Any, Callable, Dict, List, Mapping, Optional, Type

from real_world_rl.teleop.core.contract import TeleopSource


class UnknownTeleopSourceError(ValueError):
    """``device=<name>`` names no registered teleop source."""

_REGISTRY: Dict[str, Type[TeleopSource]] = {}
_BUILTIN_MODULES: Dict[str, str] = {
    "pico": "real_world_rl.teleop.pico.source",
    "pico_twist": "real_world_rl.teleop.pico.twist_sim",
    "robot": "real_world_rl.teleop.core.part",
    "spacemouse": "real_world_rl.teleop.spacemouse.source",
    "spacemouse_twist": "real_world_rl.teleop.spacemouse.twist_sim",
    "fake": "real_world_rl.teleop.core.fake",
    "gello": "real_world_rl.teleop.gello.source",
    "leader_arm": "real_world_rl.teleop.leader.joint",
    "keyboard": "real_world_rl.teleop.keyboard.pynput_source",
    "jsonl": "real_world_rl.teleop.keyboard.jsonl",
    "stdin": "real_world_rl.teleop.keyboard.stdin",
    "composite": "real_world_rl.teleop.core.composite",
    "leader_cartesian": "real_world_rl.teleop.leader.cartesian",
    "bimanual": "real_world_rl.teleop.core.bimanual",
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


def _registered_class(name: str) -> Type[TeleopSource]:
    if name not in _REGISTRY and name in _BUILTIN_MODULES:
        importlib.import_module(_BUILTIN_MODULES[name])  # registers itself
    if name not in _REGISTRY:
        raise UnknownTeleopSourceError(f"unknown teleop source {name!r}; available: {teleop_source_names()}")
    return _REGISTRY[name]


def source_class(name: str, options: Optional[Mapping[str, Any]] = None) -> Type[TeleopSource]:
    """The class ``make_teleop_source(name, env, **options)`` builds, without building it (a
    built-in source's module is imported): its class attributes declare what the source can do.
    Raises ``UnknownTeleopSourceError`` (a ``ValueError``) for a name nothing registers."""
    return _registered_class(name).resolved_class(dict(options or {}))


def make_teleop_source(name: str, env: Any, **kwargs: Any) -> TeleopSource:
    """Builds the source registered as ``name`` through its ``from_env(env, **kwargs)``."""
    return _registered_class(name).from_env(env, **kwargs)
