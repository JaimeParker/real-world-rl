"""Station limits helpers for `rwrl limits` (numpy; imported lazily by cmd_limits)."""
from __future__ import annotations

from itertools import product

import numpy as np

from real_world_rl.devices.limits import ArmLimits, TcpBox
from real_world_rl.devices.workspace import task_transform


LEGACY_KEYS = ('safety_box', 'workspace', 'joint_limits', 'max_delta_per_step')


def arm_nodes(document):
    if not isinstance(document, dict):
        raise ValueError('station must be a mapping')
    if 'robots' in document:
        return document['robots'].items()
    node = document.get('robot', document)
    return [(node.get('name', node.get('type', 'robot')), node)]


def home_position(node):
    from real_world_rl.devices.robots.franka import FRANKA_HOME_JOINTS
    from real_world_rl.devices.robots.franka_kinematics import tcp_position
    if node.get('type', 'franka') != 'franka':
        raise ValueError('no kinematics: replace old position keys manually with limits.tcp using an explicit origin XYZ')
    home = node.get('home_joints', FRANKA_HOME_JOINTS)
    if home is None:
        raise ValueError('home_joints required to freeze limits; configure home_joints first')
    return tcp_position(home, flange_T_ee=node.get('flange_T_ee'), model=node.get('robot_model', 'panda'))


def corners(box):
    vertices = np.array(list(product(*zip(box.low, box.high))))
    return (vertices @ box.axes_R.T + box.origin_xyz).tolist()


def resolved_report(config):
    limits = ArmLimits.from_config(config)
    data = {} if limits is None else limits.to_dict()
    if limits is not None and limits.tcp is not None:
        data['tcp']['corners_base_m'] = corners(limits.tcp)
    return data


def _bounds(low, high):
    for value in (low, high):
        raw = np.asarray(value, dtype=object)
        if any(isinstance(x, (bool, str)) or not isinstance(x, (int, float, np.number)) for x in raw.flat):
            raise ValueError('old position bounds must contain finite numbers')
    low, high = np.asarray(low, dtype=float), np.asarray(high, dtype=float)
    if low.shape != (3,) or high.shape != (3,) or not np.isfinite([low, high]).all() or np.any(low >= high):
        raise ValueError('old position bounds must be finite XYZ with low < high')
    return low, high


def _max_volume_halfwidths(a, b):
    """Maximise sum(log(h)) subject to A h <= b using a numpy log barrier."""
    if np.any(b <= 0):
        raise ValueError('home must be strictly inside every old position box')
    h = np.full(3, np.min(b / np.maximum(a.sum(axis=1), 1e-30)) / 4)
    for weight in (1., .1, .01, .001, .0001, .00001, .000001, .0000001, .00000001):
        for _ in range(100):
            slack = b - a @ h
            gradient = -1 / h + weight * (a.T @ (1 / slack))
            hessian = np.diag(1 / h**2) + weight * (a.T @ (a / slack[:, None]**2))
            step = np.linalg.solve(hessian, -gradient)
            if -gradient @ step < 1e-13:
                break
            rate = 1.
            objective = -np.log(h).sum() - weight * np.log(slack).sum()
            while rate > 1e-15:
                candidate = h + rate * step
                remaining = b - a @ candidate
                if np.all(candidate > 0) and np.all(remaining > 0):
                    value = -np.log(candidate).sum() - weight * np.log(remaining).sum()
                    if value <= objective + .01 * rate * gradient @ step:
                        break
                rate *= .5
            h += rate * step
    # Keep corners on the safe side of floating-point roundoff.
    return h * (1 - 1e-12)


def migrate_node(node):
    warnings = []
    has_old_keys = any(key in node for key in LEGACY_KEYS)
    if not has_old_keys and node.get('limits') is not None:
        return warnings
    if has_old_keys and node.get('limits') is not None:
        raise ValueError('both limits and old keys are present; resolve the conflict before migration')
    limits = {}
    for old, new in (('joint_limits', 'joints'), ('max_delta_per_step', 'max_delta_per_step')):
        if node.get(old) is not None:
            limits[new] = node[old]
    if node.get('type', 'franka') == 'franka' and 'joints' not in limits:
        from real_world_rl.devices.robots.franka import DATASHEET_JOINT_LIMITS
        model = node.get('robot_model', 'panda')
        if model not in DATASHEET_JOINT_LIMITS:
            raise ValueError(f'unknown robot_model {model!r}')
        limits['joints'] = [list(pair) for pair in DATASHEET_JOINT_LIMITS[model]]
        warnings.append(f'WARNING: limits.joints was unset; writing the {model} datasheet bounds (the same default the robot applies).')
    boxes = []
    frame = task_transform(node.get('task_frame'))
    if node.get('safety_box') is not None:
        values = np.asarray(node['safety_box'], dtype=object)
        if values.shape != (6,):
            raise ValueError('old safety_box must contain six numbers')
        low, high = _bounds(values[:3], values[3:])
        boxes.append((np.eye(4), low, high))
    workspace = node.get('workspace')
    margin = .003
    if workspace is not None:
        unknown = set(workspace) - {'frame', 'low', 'high', 'orientation_xyzw', 'max_angle_rad', 'measured_margin_m', 'measured_margin_rad'}
        if unknown:
            raise ValueError(f'unknown old workspace keys: {sorted(unknown)}')
        name = workspace.get('frame', 'base')
        if name != 'base' and name != (node.get('task_frame') or {}).get('name'):
            raise ValueError(f'unknown workspace frame {name!r}')
        low, high = _bounds(workspace.get('low'), workspace.get('high'))
        boxes.append((np.eye(4) if name == 'base' else frame, low, high))
        margin = workspace.get('measured_margin_m', .003)
        if any(key in workspace for key in ('orientation_xyzw', 'max_angle_rad', 'measured_margin_rad')):
            warnings.append('WARNING: workspace orientation constraints dropped; station limits constrain position only.')
    if boxes:
        origin = home_position(node)
        axes = frame[:3, :3]
        lows, highs, constraints, distances = [], [], [], []
        rotated = False
        for transform, low, high in boxes:
            relative = transform[:3, :3].T @ axes
            center = transform[:3, :3].T @ (origin - transform[:3, 3])
            lo, hi = low - center, high - center
            if np.any(lo >= 0) or np.any(hi <= 0):
                raise ValueError('home must be strictly inside every old position box')
            if not np.allclose(relative, np.eye(3), atol=1e-12, rtol=0):
                rotated = True
            lows.append(lo)
            highs.append(hi)
            constraints.extend(abs(relative))
            distances.extend(np.minimum(-lo, hi))
        if rotated:
            half = _max_volume_halfwidths(np.array(constraints), np.array(distances))
            low, high = -half, half
            warnings.append('WARNING: rotated old box converted to a conservative maximum-volume table-axis box centred on home; confirm the new bounds.')
        else:
            low, high = np.max(lows, axis=0), np.min(highs, axis=0)
        box = TcpBox(origin, axes, low, high, margin)
        limits['tcp'] = {'origin': 'home', **box.to_dict()}
        if len(boxes) > 1:
            warnings.append('Position constraints intersected; new box is contained in both old boxes.')
    for key in LEGACY_KEYS:
        node.pop(key, None)
    node['limits'] = limits
    return warnings


def rebase_node(node):
    spec = (node.get('limits') or {}).get('tcp')
    if spec is None:
        raise ValueError('no limits.tcp configured')
    axes = task_transform(node.get('task_frame'))[:3, :3]
    origin = home_position(node) if spec.get('origin') == 'home' else np.asarray(spec['origin'], dtype=float)
    box = TcpBox(origin, axes, spec['low'], spec['high'], spec.get('measured_margin_m', .003))
    if node.get('type', 'franka') == 'franka' and not box.contains(home_position(node)):
        raise ValueError('home is outside the rebased TCP box')
    spec['axes_R'] = axes.tolist()
    if spec.get('origin') == 'home' or 'origin_xyz' in spec:
        spec['origin_xyz'] = origin.tolist()


