"""Station loading with explicit real_world_rl validators, independent of metadata.

Every command that reads a station file, including the device commands (daemon, setup,
discover, the device doctor), loads it here, so the STATION_SECTIONS are always validated.
"""
from pathlib import Path

from robot_infra.discovery import autoconfig
from real_world_rl.teleop.profiles import validate_profiles_reference

STATION_SECTIONS = {"teleop_profiles": validate_profiles_reference}


def load_station(path: str | Path) -> autoconfig.Station:
    """Load device configuration and real_world_rl station sections."""
    return autoconfig.load_station(path, sections=STATION_SECTIONS)


def load_station_with_env(path: str | Path) -> autoconfig.Station:
    """Load sections and fill missing device fields from the environment."""
    return autoconfig.load_station_with_env(path, sections=STATION_SECTIONS)
