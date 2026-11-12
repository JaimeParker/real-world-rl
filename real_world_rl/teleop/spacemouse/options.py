"""What both SpaceMouse sources (the tracker mode, ``source``, and the serl mode, ``serl``)
declare about their options: they drive one arm; their options come from a flat mapping or a
station robot's ``spacemouse`` teleop section with its operator profile; they are frozen into
the run's ``teleop_input``; they take the input diagnostics filter, and the input frame they
compare is the diagnostics filter alone (the operator's input rotation is provenance).
"""
from __future__ import annotations

from typing import Any, Mapping, Optional

__all__ = ["SpaceMouseOptions"]

# Operator input rotation is source provenance: recorded actions are already in base
# coordinates, so a recalibration does not change their meaning.
_PROVENANCE_INPUT_FRAME_KEYS = ("input_rotation", "input_rotation_semantics")


class SpaceMouseOptions:
    """Mixin of the SpaceMouse ``TeleopSource`` classes (listed before ``TeleopSource``)."""

    DRIVES_ONE_ARM = True
    RECORDS_TELEOP_INPUT = True
    SUPPORTS_INPUT_DIAGNOSTICS = True

    @classmethod
    def options_from_station(
        cls,
        station_path: str,
        station: Mapping[str, Any],
        *,
        robot: Optional[str] = None,
        profile: Optional[str] = None,
        provenance: Optional[dict] = None,
    ) -> dict:
        """The selected station robot's ``spacemouse`` teleop options, with the input rotation of
        its operator profile (``profiles.source_options``); records the station file, and the
        profile, in ``provenance``."""
        from real_world_rl.teleop.spacemouse.profiles import source_options
        from real_world_rl.common.provenance import portable_path

        if provenance is not None:
            provenance["options_file"] = portable_path(station_path)
        return source_options(station_path, robot, profile, provenance=provenance)

    @staticmethod
    def compared_input_frame(options: Mapping[str, Any]) -> dict:
        """The part of the input frame contract (``twist_input.input_frame_contract``) that
        ``teleop_control`` compares: the diagnostics filter, when one is set."""
        from real_world_rl.teleop.core.targets.twist_input import input_frame_contract

        return {key: value for key, value in input_frame_contract(options).items()
                if key not in _PROVENANCE_INPUT_FRAME_KEYS}
