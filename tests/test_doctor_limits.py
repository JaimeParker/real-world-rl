"""Frozen values and independent SDK snapshot comparisons."""
from types import SimpleNamespace as NS

import numpy as np
import pytest

from real_world_rl.devices.robots.franka import FrankaConfig, DATASHEET_JOINT_LIMITS, FRANKA_HOME_JOINTS
from real_world_rl.devices.robots.franka_kinematics import FLANGE_T_HAND, tcp_position
from real_world_rl.doctor.limits import static_limits, franka_kinematics


def config():
    return FrankaConfig(limits={'joints': [list(p) for p in DATASHEET_JOINT_LIMITS['panda']], 'tcp': {
        'origin': 'home', 'origin_xyz': tcp_position(FRANKA_HOME_JOINTS).tolist(),
        'axes_R': np.eye(3).tolist(), 'low': [-.1] * 3, 'high': [.1] * 3}})


def test_static_valid_and_stale_home_and_axes():
    cfg = config()
    assert static_limits(cfg).ok
    cfg.home_joints[0] += .3
    result = static_limits(cfg)
    assert not result.ok and 'rebase' in result.hint
    cfg = config()
    cfg.task_frame = {'name': 'table', 'base_T_frame': [[0, -1, 0, 0], [1, 0, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]}
    assert not static_limits(cfg).ok


@pytest.mark.parametrize('delta,tool_delta,ok', [(0, 0, True), (.0009, 0, True), (.0011, 0, False), (0, .001, False)])
def test_connected_fk_and_tool_transform(delta, tool_delta, ok):
    cfg = config()
    tool = FLANGE_T_HAND.copy()
    tool[0, 3] += tool_delta
    xyz = tcp_position(FRANKA_HOME_JOINTS) + [delta, 0, 0]
    state = NS(q=FRANKA_HOME_JOINTS, O_T_EE=NS(translation=xyz), F_T_EE=tool.flatten(order='F'))
    robot = NS(config=cfg, arm=NS(_handle=NS(robot=NS(state=state))))
    assert franka_kinematics(robot).ok == ok


def test_missing_sdk_snapshot_fails_and_fake_skips():
    cfg = config()
    assert not franka_kinematics(NS(config=cfg, arm=NS())).ok
    cfg.arm_backend = 'fake'
    assert franka_kinematics(NS(config=cfg)).skipped


def test_nonfinite_sdk_position_is_a_serializable_failure():
    import dataclasses
    import json
    state = NS(q=FRANKA_HOME_JOINTS, O_T_EE=NS(translation=[float('nan'), 0, 0]), F_T_EE=FLANGE_T_HAND)
    result = franka_kinematics(NS(config=config(), arm=NS(_handle=NS(robot=NS(state=state)))))
    assert not result.ok and 'finite' in result.message
    json.dumps(dataclasses.asdict(result), allow_nan=False)
