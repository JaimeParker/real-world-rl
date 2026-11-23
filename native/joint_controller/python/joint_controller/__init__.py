"""Trajectory joint impedance controller for franky (quintic reference + GELLO's joint impedance).

``TrajectoryImpedanceMotion`` is a ``franky.BaseTorqueMotion``: a 1 kHz joint impedance
controller (the law, velocity filter and default gains of the public GELLO
``joint_impedance_controller``, gello_software @ 9fc8c26) that tracks an online reference
generated from sparse joint targets (10 - 30 Hz) by one quintic per joint within velocity and
acceleration limits. Start it with ``robot.move(motion, asynchronous=True)``, then call
``set_target(q)`` for every leader-arm sample. This wheel must come from the same build as the
franky wheel it is installed next to; ``build_info()`` says which one (readable without importing
the extension).
"""

from __future__ import annotations

import json
from pathlib import Path

from ._joint_controller import (  # noqa: F401
    BUILD_INFO,
    GELLO_COMMIT,
    PARAM_FIELDS,
    WITH_FRANKY,
    TrajectoryImpedanceLaw,
    __version__,
    default_params,
    validate_params,
)

BUILD_INFO_FILE = Path(__file__).with_name("build_info.json")


def build_info() -> dict:
    """The build identity written next to the extension (franky wheel version, franky commit,
    sha256 of the libfranky.so linked against, bundled libfranka file name, build id)."""
    return json.loads(BUILD_INFO_FILE.read_text())


__all__ = [
    "BUILD_INFO",
    "BUILD_INFO_FILE",
    "GELLO_COMMIT",
    "PARAM_FIELDS",
    "WITH_FRANKY",
    "TrajectoryImpedanceLaw",
    "__version__",
    "build_info",
    "default_params",
    "validate_params",
]

if WITH_FRANKY:
    from ._joint_controller import TrajectoryImpedanceMotion  # noqa: F401

    __all__ += ["TrajectoryImpedanceMotion"]
