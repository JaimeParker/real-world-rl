"""Type stubs for the compiled extension serl_controller._serl_controller."""

from typing import Any, Mapping

import numpy as np
import numpy.typing as npt

_Array = npt.ArrayLike
_F64 = npt.NDArray[np.float64]

__version__: str
SERL_COMMIT: str
FILTER_PARAMS: float
DELTA_TAU_MAX: float
PINV_LAMBDA: float
JOINT_BLEND_DURATION_S: float
# compiled-in copy of serl_controller/build_info.json
BUILD_INFO: dict[str, Any]
# {"panda": [7 x {min_position, max_position, max_velocity, soft_min_position, soft_max_position, k_position}], "fr3": [...]}
FRANKA_JOINT_LIMITS: dict[str, list[dict[str, float]]]

class PositionJointSoftLimits:
    def __init__(self, robot_model: str = "panda") -> None: ...
    def reset(self) -> None: ...
    def enforce(self, command: _Array, q_measured: _Array, period_ns: int) -> _F64: ...
WITH_FRANKY: bool
# (name, default, min, max) for each compliance_param.cfg field, in .cfg order
PARAM_FIELDS: tuple[tuple[str, float, float, float], ...]

class SerlParams:
    translational_stiffness: float
    translational_damping: float
    rotational_stiffness: float
    rotational_damping: float
    nullspace_stiffness: float
    joint1_nullspace_stiffness: float
    translational_clip_neg_x: float
    translational_clip_neg_y: float
    translational_clip_neg_z: float
    translational_clip_x: float
    translational_clip_y: float
    translational_clip_z: float
    rotational_clip_neg_x: float
    rotational_clip_neg_y: float
    rotational_clip_neg_z: float
    rotational_clip_x: float
    rotational_clip_y: float
    rotational_clip_z: float
    translational_Ki: float
    rotational_Ki: float
    def __init__(self, params: Mapping[str, float] = ...) -> None: ...
    def to_dict(self) -> dict[str, float]: ...
    def merged(self, updates: Mapping[str, float]) -> SerlParams: ...
    def __eq__(self, other: object) -> bool: ...

class SerlCartesianImpedanceLaw:
    def __init__(self, params: SerlParams | Mapping[str, float] = ...) -> None: ...
    def apply_params(self, params: SerlParams) -> None: ...
    def set_target(self, position: _Array, quat_xyzw: _Array) -> None: ...
    def starting(self, O_T_EE: _Array, q: _Array) -> None: ...
    def step(
        self,
        O_T_EE: _Array,
        q: _Array,
        dq: _Array,
        tau_J_d: _Array,
        zero_jacobian: _Array,
        coriolis: _Array,
    ) -> _F64: ...
    @property
    def params(self) -> SerlParams: ...
    def state(self) -> dict[str, Any]: ...

def law_step(
    law: SerlCartesianImpedanceLaw,
    O_T_EE: _Array,
    q: _Array,
    dq: _Array,
    tau_J_d: _Array,
    zero_jacobian: _Array,
    coriolis: _Array,
) -> _F64: ...

# Only present when built with franky (WITH_FRANKY is True); subclasses franky.BaseTorqueMotion.
class SerlCartesianImpedanceMotion:
    def __init__(self, params: SerlParams | Mapping[str, float] = ...) -> None: ...
    def set_target(self, position: _Array, quat_xyzw: _Array) -> int: ...
    def update_params(self, updates: Mapping[str, float]) -> dict[str, float]: ...
    @property
    def params(self) -> dict[str, float]: ...
    def snapshot(self) -> dict[str, Any]: ...
    def _offline_step(
        self,
        O_T_EE: _Array,
        q: _Array,
        dq: _Array,
        tau_J_d: _Array,
        zero_jacobian: _Array,
        coriolis: _Array,
    ) -> _F64: ...

class SerlJointPositionLaw:
    def __init__(self, target: _Array) -> None: ...
    def starting(self, q: _Array) -> None: ...
    def update(self, period_ns: int) -> _F64: ...
    @staticmethod
    def period_ns_from_milliseconds(milliseconds: int) -> int: ...
    @property
    def elapsed_s(self) -> float: ...
    @property
    def elapsed_ns(self) -> int: ...
    @property
    def target(self) -> _F64: ...
    @property
    def initial(self) -> _F64: ...
    @property
    def command(self) -> _F64: ...

# Only present when built with franky (WITH_FRANKY is True); subclasses franky.BaseJointPositionMotion.
class SerlJointPositionMotion:
    def __init__(self, target: _Array, robot_model: str = "panda") -> None: ...
    @property
    def target(self) -> _F64: ...
    def finish(self) -> None: ...
    @property
    def finish_requested(self) -> bool: ...
    def snapshot(self) -> dict[str, Any]: ...
    def _offline_step(self, q: _Array, time_step_ms: int) -> tuple[_F64, bool]: ...
