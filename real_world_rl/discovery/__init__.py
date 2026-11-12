"""Hardware discovery and config auto-fill; probes never connect a robot."""
from real_world_rl.discovery.autoconfig import fill_from_env, load_config
from real_world_rl.discovery.enumerate import (
    CanInfo,
    DiscoveryReport,
    HostInfo,
    discover_all,
    hosts_from_config,
)

__all__ = [
    "CanInfo",
    "DiscoveryReport",
    "HostInfo",
    "discover_all",
    "fill_from_env",
    "hosts_from_config",
    "load_config",
]
