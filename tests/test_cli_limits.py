"""Migration geometry, reviewed YAML changes, rebasing and resolved reports."""
from copy import deepcopy
from itertools import product

import numpy as np
import pytest
import yaml

from real_world_rl.cli import main
from real_world_rl.cli.limits_ops import corners, migrate_node, rebase_node
from real_world_rl.devices.limits import TcpBox
from real_world_rl.devices.robots.franka import DATASHEET_JOINT_LIMITS, FRANKA_HOME_JOINTS
from real_world_rl.devices.robots.franka_kinematics import tcp_position
from real_world_rl.station import load_station

HOME = tcp_position(FRANKA_HOME_JOINTS)
JOINTS = [list(p) for p in DATASHEET_JOINT_LIMITS['panda']]


def old_node(**extra):
    return {'type': 'franka', 'name': 'arm', 'joint_limits': deepcopy(JOINTS),
            'safety_box': np.r_[HOME - [.3, .4, .2], HOME + [.5, .6, .7]].tolist(), **extra}


def resolved(node):
    data = dict(node['limits']['tcp'])
    data.pop('origin')
    return TcpBox(**data)


def frame(angle=.6, translation=(.1, .2, .3)):
    c, s = float(np.cos(angle)), float(np.sin(angle))
    return {'name': 'table', 'base_T_frame': [[c, -s, 0, translation[0]], [s, c, 0, translation[1]],
                                            [0, 0, 1, translation[2]], [0, 0, 0, 1]]}


def test_identity_exact_and_step_caps():
    node = old_node(max_delta_per_step={'xyz_m': .03, 'joint_rad': .1})
    before = deepcopy(node)
    assert migrate_node(node) == []
    np.testing.assert_allclose(resolved(node).absolute_aabb(), before['safety_box'])
    np.testing.assert_allclose(node['limits']['tcp']['low'], [-.3, -.4, -.2])
    assert node['limits']['joints'] == JOINTS
    assert node['limits']['max_delta_per_step'] == before['max_delta_per_step']
    assert not set(node) & {'joint_limits', 'safety_box', 'workspace', 'max_delta_per_step'}


def test_rotated_is_conservative_and_warns():
    node = old_node(task_frame=frame())
    old = np.array(node['safety_box'])
    warnings = migrate_node(node)
    assert any('conservative' in w for w in warnings)
    vertices = np.array(corners(resolved(node)))
    assert np.all(vertices >= old[:3] - 1e-12)
    assert np.all(vertices <= old[3:] + 1e-12)
    np.testing.assert_allclose(resolved(node).low, -resolved(node).high)


def test_rotated_square_has_maximum_volume_halfwidths():
    node = old_node(task_frame=frame(np.pi / 4), safety_box=[*(HOME - 1), *(HOME + 1)])
    migrate_node(node)
    np.testing.assert_allclose(resolved(node).high, [1 / np.sqrt(2), 1 / np.sqrt(2), 1], atol=1e-6)


def test_workspace_own_frame_origin_and_dropped_orientation():
    transform = frame()
    matrix = np.array(transform['base_T_frame'])
    local_home = matrix[:3, :3].T @ (HOME - matrix[:3, 3])
    node = old_node(task_frame=transform, safety_box=None, workspace={
        'frame': 'table', 'low': (local_home - .1).tolist(), 'high': (local_home + .2).tolist(),
        'orientation_xyzw': [0, 0, 0, 1], 'max_angle_rad': .1, 'measured_margin_m': .002})
    warnings = migrate_node(node)
    assert any('orientation constraints dropped' in w for w in warnings)
    assert not any('conservative' in w for w in warnings)
    np.testing.assert_allclose(resolved(node).low, [-.1] * 3)
    np.testing.assert_allclose(resolved(node).high, [.2] * 3)
    assert resolved(node).measured_margin_m == .002


def test_intersection_of_base_boxes():
    node = old_node(workspace={'low': (HOME - .1).tolist(), 'high': (HOME + .1).tolist()})
    assert any('intersected' in w for w in migrate_node(node))
    np.testing.assert_allclose(resolved(node).absolute_aabb(), [*(HOME - .1), *(HOME + .1)])


def test_rotated_intersection_corners_inside_both_boxes():
    transform = frame()
    matrix = np.array(transform['base_T_frame'])
    local_home = matrix[:3, :3].T @ (HOME - matrix[:3, 3])
    node = old_node(task_frame=transform, workspace={'frame': 'table', 'low': (local_home - .12).tolist(),
                                                  'high': (local_home + .17).tolist()})
    before = deepcopy(node)
    migrate_node(node)
    vertices = np.array(corners(resolved(node)))
    assert np.all(vertices >= before['safety_box'][:3]) and np.all(vertices <= before['safety_box'][3:])
    local = (vertices - matrix[:3, 3]) @ matrix[:3, :3]
    assert np.all(local >= before['workspace']['low']) and np.all(local <= before['workspace']['high'])


def test_home_outside_refused_without_writing(tmp_path, capsys):
    path = tmp_path / 'old.yaml'
    path.write_text(yaml.safe_dump(old_node(safety_box=[1, 1, 1, 2, 2, 2])))
    raw = path.read_bytes()
    assert main(['limits', 'migrate', str(path), '--write']) == 1
    assert 'home must be strictly inside' in capsys.readouterr().err
    assert path.read_bytes() == raw


def test_write_preserves_comments_and_unrelated_bytes(tmp_path, capsys):
    path = tmp_path / 'old.yaml'
    raw = '# station notes\nrobot_ip:  "172.16.0.2" # IP\n\n' + yaml.safe_dump(old_node())
    path.write_text(raw)
    assert main(['limits', 'migrate', str(path)]) == 0
    output = capsys.readouterr().out
    assert 'limits:' in output and '--- ' in output and '-safety_box:' in output
    assert path.read_text() == raw
    assert main(['limits', 'migrate', str(path), '--write']) == 0
    assert path.read_text().startswith('# station notes\nrobot_ip:  "172.16.0.2" # IP\n\n')
    load_station(path)
    capsys.readouterr()


def test_rebase_repairs_stale_home_and_axes_and_keeps_bounds(tmp_path, capsys):
    node = old_node()
    migrate_node(node)
    low, high = deepcopy(node['limits']['tcp']['low']), deepcopy(node['limits']['tcp']['high'])
    node['home_joints'] = [*FRANKA_HOME_JOINTS]
    node['home_joints'][0] += .2
    node['task_frame'] = frame()
    path = tmp_path / 'station.yaml'
    path.write_text(yaml.safe_dump(node))
    with pytest.raises(ValueError, match='changed since'):
        load_station(path)
    raw = path.read_bytes()
    assert main(['limits', 'rebase', str(path)]) == 0
    assert path.read_bytes() == raw
    assert main(['limits', 'rebase', str(path), '--write']) == 0
    config = load_station(path).select().config
    np.testing.assert_allclose(config.limits['tcp']['origin_xyz'], tcp_position(node['home_joints']))
    assert config.limits['tcp']['low'] == low and config.limits['tcp']['high'] == high
    capsys.readouterr()


def test_rebase_refuses_home_outside_explicit_origin():
    node = old_node()
    migrate_node(node)
    node['limits']['tcp']['origin'] = [3., 3., 3.]
    with pytest.raises(ValueError, match='home is outside'):
        rebase_node(node)


@pytest.mark.parametrize('selector', ['positional', 'flag', 'global'])
def test_show_multiple_arms_and_eight_corners(tmp_path, capsys, selector):
    import json
    node = old_node()
    migrate_node(node)
    path = tmp_path / 'station.yaml'
    path.write_text(yaml.safe_dump({'robots': {'left': node, 'right': {**node, 'name': 'right'}}}))
    args = {'positional': ['limits', 'show', str(path)], 'flag': ['limits', 'show', '--station', str(path)],
            'global': ['--station', str(path), 'limits', 'show']}[selector]
    assert main(['--json', *args]) == 0
    result = json.loads(capsys.readouterr().out)
    assert set(result['arms']) == {'left', 'right'}
    for data in result['arms'].values():
        assert data['joints'] == JOINTS
        assert len(data['tcp']['corners_base_m']) == 8
        assert data['tcp']['measured_margin_m'] == .003


def test_missing_old_joint_limits_get_datasheet_with_warning():
    node = {'type': 'franka', 'robot_model': 'fr3'}
    warnings = migrate_node(node)
    assert node['limits']['joints'] == [list(p) for p in DATASHEET_JOINT_LIMITS['fr3']]
    assert any('limits.joints was unset' in warning and 'datasheet' in warning for warning in warnings)


def test_migrate_uses_configured_tool_transform():
    from real_world_rl.devices.robots.franka_kinematics import FLANGE_T_HAND
    tool = FLANGE_T_HAND.copy()
    tool[0, 3] += .04
    node = old_node(flange_T_ee=tool.tolist())
    migrate_node(node)
    np.testing.assert_allclose(resolved(node).origin_xyz, tcp_position(FRANKA_HOME_JOINTS, tool))


@pytest.mark.parametrize('invalid', [True, '0.1', float('nan'), float('inf')])
def test_migrate_rejects_invalid_old_bounds(invalid):
    node = old_node()
    node['safety_box'][0] = invalid
    with pytest.raises(ValueError, match='finite'):
        migrate_node(node)


def test_nonfranka_position_migration_refuses_to_guess_origin():
    node = {'type': 'piper', 'safety_box': [-1, -1, -1, 1, 1, 1]}
    with pytest.raises(ValueError, match='explicit origin XYZ'):
        migrate_node(node)


def test_nested_robot_migration_and_multirobot_rebase(tmp_path, capsys):
    path = tmp_path / 'nested.yaml'
    path.write_text(yaml.safe_dump({'robot': old_node()}))
    assert main(['limits', 'migrate', str(path), '--write']) == 0
    load_station(path)
    node = yaml.safe_load(path.read_text())['robot']
    second = deepcopy(node)
    second['home_joints'] = [*FRANKA_HOME_JOINTS]
    second['home_joints'][0] += .2
    multi = tmp_path / 'multi.yaml'
    multi.write_text(yaml.safe_dump({'robots': {'left': node, 'right': {**second, 'name': 'right'}}}))
    assert main(['limits', 'rebase', str(multi), '--write']) == 0
    assert len(load_station(multi).robots) == 2
    capsys.readouterr()


def test_conflicting_limits_and_legacy_keys_refused():
    node = old_node(limits={'joints': JOINTS})
    with pytest.raises(ValueError, match='both limits and old keys'):
        migrate_node(node)


def test_migration_preserves_registered_station_sections(tmp_path, capsys):
    path = tmp_path / 'station.yaml'
    link = {'address': 'robot-pc:50051'}
    path.write_text(yaml.safe_dump(old_node(link=link)))
    assert main(['limits', 'migrate', str(path), '--write']) == 0
    assert yaml.safe_load(path.read_text())['link'] == link
    load_station(path)
    capsys.readouterr()


def test_rebase_skips_arms_without_tcp_box(tmp_path, capsys):
    node = old_node()
    migrate_node(node)
    path = tmp_path / 'station.yaml'
    path.write_text(yaml.safe_dump({'robots': {'left': node, 'right': {
        'type': 'franka', 'name': 'right', 'limits': {'joints': JOINTS}}}}))
    assert main(['limits', 'rebase', str(path), '--write']) == 0
    assert 'no TCP box' in capsys.readouterr().out
    load_station(path)


def test_station_examples_validate():
    from pathlib import Path
    root = Path(__file__).resolve().parents[1] / 'examples' / 'stations'
    for path in root.glob('*.yaml'):
        load_station(path)
