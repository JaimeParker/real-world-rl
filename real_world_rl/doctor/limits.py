"""Static limits and same-snapshot Franka TCP calibration checks."""
import numpy as np

from real_world_rl.devices.limits import ArmLimits
from real_world_rl.devices.robots.franka_kinematics import FLANGE_T_HAND
from .checks import CheckResult


def static_limits(config):
    try:
        ArmLimits.from_config(config)
    except (ValueError, TypeError) as error:
        return CheckResult(1, 'config:limits', False, str(error),
                           'review limits; run `rwrl limits rebase <yaml>` after changing home or task_frame')
    return CheckResult(1, 'config:limits', True, 'station limits and frozen origin/axes validate')


def _matrix(value):
    if hasattr(value, 'matrix'):
        value = value.matrix() if callable(value.matrix) else value.matrix
    matrix = np.asarray(value, dtype=float)
    if matrix.shape == (16,):
        matrix = matrix.reshape(4, 4, order='F')
    if matrix.shape != (4, 4) or not np.isfinite(matrix).all():
        raise ValueError('expected a finite 4x4 SDK transform')
    return matrix


def franka_kinematics(robot):
    name = 'feedback:kinematics'
    config = robot.config
    if config.TYPE != 'franka' or config.arm_backend != 'franky':
        return CheckResult(4, name, True, 'requires a connected Franka SDK arm', skipped=True)
    try:
        # One SDK snapshot keeps q and O_T_EE synchronous; no second connection.
        state = robot.arm._handle.robot.state
        expected = config.kinematics()(state.q)
        pose = state.O_T_EE
        measured = np.asarray(pose.translation, dtype=float) if hasattr(pose, 'translation') else _matrix(pose)[:3, 3]
        if measured.shape != (3,) or not np.isfinite(measured).all():
            raise ValueError("O_T_EE position must contain three finite numbers")
        error = float(np.linalg.norm(expected - measured))
        tool = FLANGE_T_HAND if config.flange_T_ee is None else np.asarray(config.flange_T_ee)
        tool_ok = np.allclose(_matrix(state.F_T_EE), tool, atol=1e-4, rtol=0)
        ok = error < .001 and tool_ok
        return CheckResult(4, name, bool(ok),
                           f'FK vs O_T_EE error {error * 1000:.6f} mm; flange_T_ee matches F_T_EE: {tool_ok}',
                           '' if ok else 'check robot_model and flange_T_ee, then run `rwrl limits rebase <yaml>`',
                           details={'position_error_m': error, 'tool_transform_matches': bool(tool_ok)})
    except (AttributeError, TypeError, ValueError) as error:
        return CheckResult(4, name, False, f'cannot verify Franka kinematics: {error}')
