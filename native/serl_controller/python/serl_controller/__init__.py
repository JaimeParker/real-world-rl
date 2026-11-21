"""SERL's Franka controllers (serl_franka_controllers @ 1f140ef) for franky.

``SerlCartesianImpedanceMotion`` is a ``franky.BaseTorqueMotion`` (SERL's
CartesianImpedanceController); ``SerlJointPositionMotion`` is a
``franky.BaseJointPositionMotion`` (SERL's JointPositionController, the joint
reset). Run either with ``robot.move(motion, asynchronous=True, limit_rate=True,
cutoff_frequency=100.0)`` to match franka_control's defaults (see README). This
wheel must come from the same build as the franky wheel it is installed next to;
``build_info()`` says which one (readable without importing the extension).
"""

from __future__ import annotations

import json
from pathlib import Path

from ._serl_controller import (  # noqa: F401
    BUILD_INFO,
    DELTA_TAU_MAX,
    FILTER_PARAMS,
    FRANKA_JOINT_LIMITS,
    JOINT_BLEND_DURATION_S,
    PARAM_FIELDS,
    PINV_LAMBDA,
    SERL_COMMIT,
    WITH_FRANKY,
    PositionJointSoftLimits,
    SerlCartesianImpedanceLaw,
    SerlJointPositionLaw,
    SerlParams,
    __version__,
    law_step,
)

BUILD_INFO_FILE = Path(__file__).with_name("build_info.json")


def build_info() -> dict:
    """The build identity written next to the extension (franky wheel version, franky commit,
    sha256 of the libfranky.so linked against, bundled libfranka file name, build id)."""
    return json.loads(BUILD_INFO_FILE.read_text())


__all__ = [
    "BUILD_INFO",
    "BUILD_INFO_FILE",
    "DELTA_TAU_MAX",
    "FILTER_PARAMS",
    "FRANKA_JOINT_LIMITS",
    "JOINT_BLEND_DURATION_S",
    "PARAM_FIELDS",
    "PINV_LAMBDA",
    "SERL_COMMIT",
    "WITH_FRANKY",
    "PositionJointSoftLimits",
    "SerlCartesianImpedanceLaw",
    "SerlJointPositionLaw",
    "SerlParams",
    "__version__",
    "build_info",
    "law_step",
]

if WITH_FRANKY:
    from ._serl_controller import SerlCartesianImpedanceMotion, SerlJointPositionMotion  # noqa: F401

    __all__ += ["SerlCartesianImpedanceMotion", "SerlJointPositionMotion"]

# franka_control's (franka_ros) defaults (franka_control_node.yaml: rate_limiting, cutoff_frequency),
# which franka_hw applies to every control interface, torque and joint position alike.
FRANKA_ROS_LIMIT_RATE = True
FRANKA_ROS_CUTOFF_FREQUENCY = 100.0
__all__ += ["FRANKA_ROS_LIMIT_RATE", "FRANKA_ROS_CUTOFF_FREQUENCY"]
