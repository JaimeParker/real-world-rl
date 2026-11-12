"""Importing this package registers every robot type (no vendor SDK is imported)."""
from real_world_rl.devices.robots import franka as _franka  # noqa: F401
from real_world_rl.devices.robots import piper as _piper  # noqa: F401
