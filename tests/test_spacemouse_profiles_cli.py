import json
from types import SimpleNamespace

import numpy as np
import pytest
import yaml

from real_world_rl.cli import main
from real_world_rl.teleop.profiles import AXIS_MAPPING_VERSION, resolve_profile
from real_world_rl.training.env_building import _single_arm_yaml_options
from robot_infra.discovery.autoconfig import load_station


def station(tmp_path):
    profile = {"frame": "table", "yaw_rad": .4, "axis_mapping_version": AXIS_MAPPING_VERSION}
    (tmp_path / 'profiles.yaml').write_text(yaml.safe_dump({"schema_version": 1, "profiles": {"rear": profile}}))
    config = {"type": "franka", "name": "left", "robot_ip": "172.16.0.2",
              "task_frame": {"name": "table", "base_T_frame": np.eye(4).tolist()},
              "teleop": {"backend": "spacemouse"},
              "teleop_profiles": {"file": "profiles.yaml", "default": "rear"}}
    path = tmp_path / 'station.yaml'
    path.write_text(yaml.safe_dump(config))
    return path


def test_profile_relative_path_and_training_resolution(tmp_path):
    path = station(tmp_path)
    selected = load_station(path).select().config
    resolved = resolve_profile(path, selected)
    args = SimpleNamespace(teleop_device="spacemouse", teleop_options_yaml=str(path))
    assert _single_arm_yaml_options(args)["input_rotation"] == resolved["input_rotation"]
    with pytest.raises(ValueError, match="not found"):
        resolve_profile(path, selected, "unknown")


def test_cli_profile_and_workspace_roundtrip(tmp_path, capsys):
    path = station(tmp_path)
    prefix = ["--station", str(path), "--json", "--yes"]
    assert main(prefix + ["teleop", "profiles", "show", "--profile", "rear"]) == 0
    assert json.loads(capsys.readouterr().out)["name"] == "rear"
    assert main(prefix + ["workspace", "set", "--frame", "table", "--low", "0", "0", "0",
                          "--high", "1", "1", "1", "--save"]) == 0
    saved = json.loads(capsys.readouterr().out)
    assert saved["saved"]["backup"]
    assert load_station(path).select().config.workspace["high"] == [1, 1, 1]
    assert main(prefix + ["workspace", "validate", "--pose", ".5", ".5", ".5", "0", "0", "0", "1"]) == 0
    assert json.loads(capsys.readouterr().out)["valid"]


def test_legacy_rotation_conflict(tmp_path):
    path = station(tmp_path)
    data = yaml.safe_load(path.read_text())
    data['teleop']['options'] = {'input_rotation': np.eye(3).tolist()}
    path.write_text(yaml.safe_dump(data))
    args = SimpleNamespace(teleop_device="spacemouse", teleop_options_yaml=str(path))
    with pytest.raises(ValueError, match="conflicts"):
        _single_arm_yaml_options(args)


def test_run_builds_supervised_child_with_hold(tmp_path, capsys, monkeypatch):
    path = station(tmp_path)
    preset = tmp_path / 'preset.yaml'
    preset.write_text(yaml.safe_dump({'real_robot': {'home_joints': [0]*7, 'gripper_in_action': False}}))
    calls = []
    def run(command, **kwargs):
        assert kwargs['env']['RWRL_DIAGNOSTICS_PATH'].endswith('diagnostics.jsonl')
        assert kwargs['env']['RWRL_STOP_TRACE_PATH'].endswith('stop-trace.json')
        assert kwargs['env']['RWRL_SESSION_EVENTS_PATH'].endswith('session-events.jsonl')
        calls.append(command)
        return SimpleNamespace(returncode=0)
    monkeypatch.setattr('subprocess.run', run)
    monkeypatch.delenv('ROBOT_INFRA_HEARTBEAT_FD', raising=False)
    result = main(['--station', str(path), '--preset', str(preset), '--json', '--yes',
                   'teleop', 'run', '--record', str(tmp_path / 'demo.pkl')])
    assert result == 0
    payload = json.loads(capsys.readouterr().out)
    from pathlib import Path
    resolved = yaml.safe_load((Path(payload['report_directory']) / 'resolved-config.yaml').read_text())
    assert resolved['episode_end_behavior'] == 'hold'
    assert resolved['teleop_profile'] == 'rear'
    assert resolved['teleop_input']['yaw_rad'] == .4
    assert not {'stop_trace_path', 'diagnostics_path', 'session_events_path'} & resolved['real_robot'].keys()
    assert 'robot_infra.supervise' in calls[0]
    provenance = json.loads((Path(payload['report_directory']) / 'provenance.json').read_text())
    assert provenance['input_files']['profiles']['sha256']
    assert provenance['packages']['real_world_rl'][0]['files']['cli/teleop_config.py']
    assert provenance['config_sha256']


def test_profile_identity_validation_and_source_propagation(tmp_path):
    from real_world_rl.teleop.profiles import validate_device_identity, verify_device_identity, source_options
    identity = {'vendor_id': 0x256f, 'product_id': 0xc63a, 'serial_number': 'unit-a'}
    assert validate_device_identity(identity) == identity
    with pytest.raises(ValueError, match='serial_number'):
        verify_device_identity(identity, {**identity, 'serial_number': 'unit-b'})
    with pytest.raises(ValueError, match='unavailable'):
        verify_device_identity(identity, None)
    with pytest.raises(ValueError, match='vendor_id'):
        validate_device_identity({'vendor_id': True, 'product_id': 1})
    path = station(tmp_path)
    config = load_station(path).select().config
    from pathlib import Path
    file = Path(path).parent / config.teleop_profiles['file']
    data = yaml.safe_load(file.read_text())
    data['profiles']['rear']['device_identity'] = identity
    file.write_text(yaml.safe_dump(data))
    assert source_options(path)['expected_device_identity'] == identity


def test_profile_without_serial_matches_model_but_not_other_product():
    from real_world_rl.teleop.profiles import verify_device_identity
    expected = {'vendor_id': 1, 'product_id': 2, 'serial_number': ''}
    verify_device_identity(expected, {**expected, 'serial_number': 'new-serial'})
    with pytest.raises(ValueError, match='product_id'):
        verify_device_identity(expected, {**expected, 'product_id': 3})


@pytest.mark.parametrize('owned', [True, False])
def test_wrong_device_fails_without_disconnecting_borrowed_input(owned):
    from real_world_rl.teleop.profiles import start_profiled_source
    from robot_infra.devices.errors import DeviceFault
    calls = []
    part = SimpleNamespace(connect=lambda: calls.append('connect'),
                           disconnect=lambda: calls.append('disconnect'),
                           device_identity={'vendor_id': 1, 'product_id': 3})
    source = SimpleNamespace(_part=part, _owns_part=owned,
                             _expected_device_identity={'vendor_id': 1, 'product_id': 2})
    with pytest.raises(DeviceFault, match='does not match'):
        start_profiled_source(source)
    assert calls == (['connect', 'disconnect'] if owned else [])


@pytest.mark.parametrize('axes', ['translation', 'rotation', 'all'])
def test_motion_diagnostic_launches_supervised_limited_session(tmp_path, capsys, monkeypatch, axes):
    path = station(tmp_path)
    data = yaml.safe_load(path.read_text())
    data['workspace'] = {'frame': 'table', 'low': [-1]*3, 'high': [1]*3}
    path.write_text(yaml.safe_dump(data))
    preset = tmp_path / 'preset.yaml'
    preset.write_text(yaml.safe_dump({'real_robot': {'home_joints': [0]*7, 'gripper_in_action': False}}))
    calls = []
    def run(command, **kwargs):
        assert kwargs['env']['RWRL_DIAGNOSTICS_PATH'].endswith('diagnostics.jsonl')
        assert kwargs['env']['RWRL_STOP_TRACE_PATH'].endswith('stop-trace.json')
        assert kwargs['env']['RWRL_SESSION_EVENTS_PATH'].endswith('session-events.jsonl')
        calls.append(command)
        return SimpleNamespace(returncode=0)
    monkeypatch.setattr('subprocess.run', run)
    monkeypatch.delenv('ROBOT_INFRA_HEARTBEAT_FD', raising=False)
    assert main(['--station', str(path), '--preset', str(preset), '--json', '--yes',
                 'teleop', 'diagnose', '--motion', axes, '--duration-s', '2',
                 '--record', str(tmp_path / 'diagnostic.pkl')]) == 0
    payload = json.loads(capsys.readouterr().out)
    from pathlib import Path
    resolved = yaml.safe_load((Path(payload['report_directory']) / 'resolved-config.yaml').read_text())
    assert resolved['teleop_diagnostic_axes'] == axes
    assert resolved['teleop_diagnostic_gain'] == .2
    assert resolved['max_episode_steps'] == 20
    assert resolved['episode_end_behavior'] == 'hold'
    assert not {'stop_trace_path', 'diagnostics_path', 'session_events_path'} & resolved['real_robot'].keys()
    assert 'robot_infra.supervise' in calls[0]


def test_workspace_saved_points_candidate_and_stale_frame(tmp_path, capsys, monkeypatch):
    from contextlib import contextmanager
    path = station(tmp_path)
    poses = iter([[0, 0, 0, 0, 0, 0, 1], [1, 2, 3, 0, 0, 0, 1]])
    @contextmanager
    def connect(args):
        yield SimpleNamespace(arm=SimpleNamespace(read_state=lambda: {'tcp_pose': np.array(next(poses))}))
    monkeypatch.setattr('real_world_rl.cli.connect.open_robot', connect)
    prefix = ['--station', str(path), '--json', '--yes']
    for name in ('near', 'far'):
        assert main(prefix + ['workspace', 'capture', '--name', name, '--output', 'points.yaml', '--save']) == 0
        assert json.loads(capsys.readouterr().out)['saved']
    original = path.read_bytes()
    assert main(prefix + ['workspace', 'set', '--frame', 'table', '--from-points', 'points.yaml']) == 0
    result = json.loads(capsys.readouterr().out)
    assert result['candidate']['high'] == [1, 2, 3]
    assert path.read_bytes() == original
    data = yaml.safe_load(path.read_text())
    data['task_frame']['base_T_frame'][0][3] = .1
    path.write_text(yaml.safe_dump(data))
    assert main(prefix + ['workspace', 'set', '--frame', 'table', '--from-points', 'points.yaml']) == 1
    assert 'frame changed' in capsys.readouterr().out


def test_workspace_points_reject_degenerate_and_invalid_measurements():
    from real_world_rl.teleop.workspace_points import point_set, candidate_bounds
    document, _ = point_set(None, None)
    document['points'] = {str(i): {'base_pose': [i, 0, 0, 0, 0, 0, 1]} for i in range(2)}
    with pytest.raises(ValueError, match='span all three'):
        candidate_bounds(document, None)
    document['points']['0']['base_pose'][0] = float('nan')
    with pytest.raises(ValueError, match='finite'):
        point_set(document, None)


def test_single_label_capture_preserves_merge_and_reports_missing(tmp_path, monkeypatch, capsys):
    path = station(tmp_path)
    report = tmp_path / 'report'
    report.mkdir()
    def collect(config, **kwargs):
        assert kwargs['labels'] == ['+x']
        return {'+x': [[.5, 0, 0, 0, 0, 0]]}, {'report_directory': str(report)}
    monkeypatch.setattr('real_world_rl.teleop.input_diagnostics.collect', collect)
    assert main(['--station', str(path), '--yes', '--json', 'teleop', 'calibrate',
                 '--profile', 'rear', '--capture-label', '+x']) == 0
    result = json.loads(capsys.readouterr().out)
    assert result['status'] == 'NEEDS_SAMPLES'
    assert result['required_fields'] == ['+y', '-x', '-y']
    assert json.loads((report / 'merged-samples.json').read_text())['+x']['trials']


@pytest.mark.parametrize('append,expected_trials', [(False, 1), (True, 2)])
def test_recapture_one_group_preserves_others(tmp_path, monkeypatch, capsys, append, expected_trials):
    from test_spacemouse_calibration import samples
    path = station(tmp_path)
    data = samples(0)
    original = tmp_path / 'input.json'
    original.write_text(json.dumps(data))
    report = tmp_path / 'report'
    report.mkdir()
    def collect(config, **kwargs):
        return {'+x': data['+x']}, {'report_directory': str(report),
                                  'device_identity': {'vendor_id': 1, 'product_id': 2}}
    monkeypatch.setattr('real_world_rl.teleop.input_diagnostics.collect', collect)
    argv = ['--station', str(path), '--yes', '--json', 'teleop', 'calibrate', '--profile', 'rear',
            '--samples', str(original), '--capture-label', '+x']
    assert main(argv + (['--append-trial'] if append else [])) == 0
    result = json.loads(capsys.readouterr().out)
    merged = json.loads((report / 'merged-samples.json').read_text())
    assert len(merged['+x']['trials']) == expected_trials
    assert merged['-x'] == data['-x']
    assert json.loads(original.read_text()) == data
    assert result['candidate']['device_identity'] == {'vendor_id': 1, 'product_id': 2}


def test_yaml_review_shows_exact_diff_and_backup(tmp_path, capsys):
    import stat
    from real_world_rl.cli.teleop_config import confirm_write, atomic_save
    path = tmp_path / 'profile.yaml'
    previous = b'# retained in backup\nyaw: 0\n'
    path.write_bytes(previous)
    confirm_write(SimpleNamespace(yes=True), path, {'yaw': 1}, previous)
    diff = capsys.readouterr().err
    assert '-yaw: 0' in diff and '+yaw: 1' in diff
    result = atomic_save(path, {'yaw': 1}, previous)
    from pathlib import Path
    backup = Path(result['backup'])
    assert backup.read_bytes() == previous
    assert stat.S_IMODE(backup.stat().st_mode) == 0o600


def test_yaml_save_rejects_change_during_review(tmp_path):
    from real_world_rl.cli.teleop_config import atomic_save
    from real_world_rl.cli.errors import CliError
    path = tmp_path / 'station.yaml'
    expected = b'value: original\n'
    path.write_bytes(b'value: external edit\n')
    with pytest.raises(CliError, match='changed during review'):
        atomic_save(path, {'value': 'candidate'}, expected)
    assert path.read_bytes() == b'value: external edit\n'
    assert not list(tmp_path.glob('*.bak'))


def test_yaml_save_rejects_change_during_staging(tmp_path, monkeypatch):
    from real_world_rl.cli.teleop_config import atomic_save
    from real_world_rl.cli.errors import CliError
    import os
    path = tmp_path / 'station.yaml'
    expected = b'value: original\n'
    path.write_bytes(expected)
    original_fsync = os.fsync
    def concurrent_edit(fd):
        original_fsync(fd)
        path.write_bytes(b'value: concurrent edit\n')
    monkeypatch.setattr(os, 'fsync', concurrent_edit)
    with pytest.raises(CliError, match='changed before replacement'):
        atomic_save(path, {'value': 'candidate'}, expected)
    assert path.read_bytes() == b'value: concurrent edit\n'
    assert list(tmp_path.glob('*.bak'))[0].read_bytes() == expected
    assert not list(tmp_path.glob('.station.yaml.*'))


def test_legacy_rotation_with_task_frame_without_profile_is_rejected(tmp_path):
    from real_world_rl.teleop.profiles import source_options
    path = station(tmp_path)
    data = yaml.safe_load(path.read_text())
    del data['teleop_profiles']
    data['teleop']['options'] = {'input_rotation': np.eye(3).tolist()}
    path.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError, match='conflicts with station task_frame'):
        source_options(path)
    # A legacy installation with no new task frame keeps its former behavior.
    del data['task_frame']
    path.write_text(yaml.safe_dump(data))
    assert source_options(path)['input_rotation'] == np.eye(3).tolist()


@pytest.mark.parametrize('command,missing', [
    (['teleop', 'run'], ['--record']),
    (['teleop', 'calibrate'], ['--profile']),
    (['teleop', 'inspect-stop'], ['--trace']),
    (['workspace', 'validate'], ['--pose']),
])
def test_agent_missing_arguments_are_structured(command, missing, capsys):
    assert main(['--json'] + command) == 2
    result = json.loads(capsys.readouterr().out)
    assert result['required_fields'] == missing


def test_agent_capture_reports_only_missing_save_fields(tmp_path, capsys):
    path = station(tmp_path)
    assert main(['--json', '--station', str(path), 'workspace', 'capture',
                 '--save', '--name', 'point']) == 2
    assert json.loads(capsys.readouterr().out)['required_fields'] == ['--output']


def test_workspace_show_reports_units_source_and_oriented_corners(tmp_path, capsys):
    path = station(tmp_path)
    data = yaml.safe_load(path.read_text())
    transform = np.eye(4)
    angle = np.pi / 4
    transform[:2, :2] = [[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]]
    transform[:3, 3] = [.2, .3, .4]
    data['task_frame']['base_T_frame'] = transform.tolist()
    data['task_frame']['source'] = 'measured fixture'
    data['workspace'] = {'frame': 'table', 'low': [0, 0, 0], 'high': [1, 2, 3]}
    path.write_text(yaml.safe_dump(data))
    assert main(['--station', str(path), '--json', 'workspace', 'show']) == 0
    result = json.loads(capsys.readouterr().out)
    assert result['units']['position'] == 'm'
    assert result['source']['frame_calibration'] == 'measured fixture'
    corners = np.array(result['workspace_corners_base_m'])
    recovered = (corners - transform[:3, 3]) @ transform[:3, :3]
    np.testing.assert_allclose(recovered, result['workspace_corners_frame_m'], atol=1e-12)
    assert corners.shape == (8, 3)
    assert 'no live daemon' in result['scope']


@pytest.mark.parametrize('mode', ['tracker', 'serl'])
def test_profile_contract_uses_numbers_not_names_and_shared_record_train_resolution(tmp_path, monkeypatch, mode):
    from real_world_rl.demos.record import RecordDemosArgs
    from real_world_rl.training.entrypoints.hil_serl import HilSerlArgs
    from real_world_rl.training.env_building import teleop_control_spec, resolve_teleop_input, reward_sources_spec
    from real_world_rl.teleop.spacemouse import SpaceMouseTeleopSource
    from real_world_rl.teleop.spacemouse_serl import SpaceMouseSerlTeleopSource
    def forbidden(self):
        raise AssertionError('contract resolution must not open hardware')
    monkeypatch.setattr(SpaceMouseTeleopSource, 'start', forbidden)
    monkeypatch.setattr(SpaceMouseSerlTeleopSource, 'start', forbidden)
    path = station(tmp_path)
    options = dict(env_backend='real_robot', teleop_device='spacemouse', spacemouse_mode=mode,
                   teleop_options_yaml=str(path))
    recorded = teleop_control_spec(RecordDemosArgs(**options))
    trained = teleop_control_spec(HilSerlArgs(**options))
    assert recorded == trained
    profile_path = tmp_path / 'profiles.yaml'
    profiles = yaml.safe_load(profile_path.read_text())
    profiles['profiles']['renamed'] = profiles['profiles'].pop('rear')
    renamed = tmp_path / 'relocated.yaml'
    renamed.write_text(yaml.safe_dump(profiles))
    config = yaml.safe_load(path.read_text())
    config['teleop_profiles'] = {'file': renamed.name, 'default': 'renamed'}
    path.write_text(yaml.safe_dump(config))
    assert teleop_control_spec(HilSerlArgs(**options)) == trained
    before = HilSerlArgs(**options)
    resolve_teleop_input(before)
    before_input = reward_sources_spec(before)['teleop_input']
    profiles['profiles']['renamed']['yaw_rad'] += .2
    renamed.write_text(yaml.safe_dump(profiles))
    after = HilSerlArgs(**options)
    resolve_teleop_input(after)
    assert teleop_control_spec(after) == trained
    after_input = reward_sources_spec(after)['teleop_input']
    assert after_input['yaw_rad'] == pytest.approx(before_input['yaw_rad'] + .2)
    assert after_input['input_rotation'] != before_input['input_rotation']


@pytest.mark.parametrize('mode', ['tracker', 'serl'])
def test_record_and_training_normalization_freeze_profile_before_serialization(tmp_path, mode):
    import dataclasses
    from real_world_rl.demos.record import parse_record_args
    from real_world_rl.envs.real_robot import RealRobotEnvConfig
    from real_world_rl.training.entrypoints.hil_serl import HilSerlArgs
    from real_world_rl.training.registry import RealWorldAlgorithmRegistry

    path = station(tmp_path)
    # The preset dumps the whole config, so its cartesian_semantics is explicit and must match the mode.
    config = RealRobotEnvConfig(remote='robot:50051', safety_box=[-1]*3+[1]*3,
                                safety_box_rpy_low=[0, -1, -1], safety_box_rpy_high=[3, 1, 1],
                                cartesian_semantics='serl' if mode == 'serl' else 'current')
    options = dict(env_backend='real_robot', teleop_device='spacemouse', spacemouse_mode=mode,
                   teleop_options_yaml=str(path), real_robot=dataclasses.asdict(config))
    preset = tmp_path / 'record.yaml'
    preset.write_text(yaml.safe_dump(options))
    recorded = parse_record_args(['--config', str(preset)])
    original = HilSerlArgs(**{**options, 'real_robot': config})
    trained, _ = RealWorldAlgorithmRegistry()._normalize_runtime(original, {})
    assert dataclasses.asdict(recorded)['teleop_input'] == dataclasses.asdict(trained)['teleop_input']
    assert trained.teleop_input['yaw_rad'] == .4
    assert trained.teleop_input['source_options']['input_rotation'] == resolve_profile(
        path, load_station(path).select().config)['input_rotation']


def test_recalibration_is_provenance_and_saved_rotation_is_not_reloaded(tmp_path):
    from dataclasses import replace
    from rl_garden.common.effective_config import json_value
    from real_world_rl.action_contract import compared_experiment
    from real_world_rl.training.entrypoints.hil_serl import HilSerlArgs
    from real_world_rl.training.env_building import resolve_teleop_input, reward_sources_spec
    path = station(tmp_path)
    args = HilSerlArgs(teleop_device='spacemouse', teleop_options_yaml=str(path))
    resolve_teleop_input(args)  # happens before persist_resolved_config
    saved = json_value(args)['teleop_input']
    assert saved['profile_name'] == 'rear'
    assert saved['yaw_rad'] == .4
    assert saved['input_rotation'] == _single_arm_yaml_options(args)['input_rotation']
    before = reward_sources_spec(args)
    assert 'input_rotation' not in before['teleop_control']
    assert before['teleop_input'] == saved
    profiles = tmp_path / 'profiles.yaml'
    data = yaml.safe_load(profiles.read_text())
    data['profiles']['rear']['yaw_rad'] += .04
    profiles.write_text(yaml.safe_dump(data))
    fresh = replace(args, teleop_input=None)
    resolve_teleop_input(fresh)
    after = reward_sources_spec(fresh)
    assert after['teleop_input']['input_rotation'] != saved['input_rotation']
    assert compared_experiment({'sources': before}) == compared_experiment({'sources': after})
    profiles.unlink()
    path.unlink()
    restored = replace(args, teleop_input=saved)
    assert _single_arm_yaml_options(restored)['input_rotation'] == saved['input_rotation']


def test_remote_teleop_identity_verifies_like_local(tmp_path, monkeypatch):
    from robot_infra.devices import TeleopConfig
    from robot_infra.devices.backends.teleops.fake import FakeTeleop
    from robot_infra.devices.robots.franka import FrankaConfig, FrankaRobot
    from robot_infra.server import RemoteRobot
    from robot_infra.server.service import RobotService
    from robot_infra.server.transport import LocalClient
    from real_world_rl.teleop.profiles import verify_device_identity
    identity = {'vendor_id': 0x256f, 'product_id': 0xc635, 'serial_number': 'unit-a'}
    monkeypatch.setattr('robot_infra.devices.claims._DEFAULT_LOCK_DIR', tmp_path / 'claims')
    monkeypatch.setattr(FakeTeleop, 'device_identity', property(lambda self: dict(identity)), raising=False)
    robot = FrankaRobot(FrankaConfig(arm_backend='fake', gripper=None,
                                    teleop=TeleopConfig(backend='fake')))
    service = RobotService(robot, watchdog_s=30.)
    remote = RemoteRobot.from_client(LocalClient(service))
    try:
        remote.connect()
        verify_device_identity(identity, robot.teleop.device_identity)
        verify_device_identity(identity, remote.teleop.device_identity)
        observed = remote.teleop.device_identity
        observed['product_id'] = 1
        assert remote.teleop.device_identity == identity
        assert service.dispatch('robot.describe')['teleop']['device_identity'] == identity
    finally:
        remote.disconnect()
        service.close()


def test_calibration_frame_rejected_before_input_collection(tmp_path, monkeypatch, capsys):
    path = station(tmp_path)
    def forbidden(*args, **kwargs):
        raise AssertionError('must reject the frame before opening input')
    monkeypatch.setattr('real_world_rl.teleop.input_diagnostics.collect', forbidden)
    assert main(['--station', str(path), '--yes', '--json', 'teleop', 'calibrate',
                 '--profile', 'rear', '--frame', 'wrong', '--capture-label', '+x']) == 2
    assert 'frame must match' in capsys.readouterr().out


def test_workspace_invalid_is_a_validation_result(tmp_path, capsys):
    from real_world_rl.cli.errors import WORKSPACE_INVALID_EXIT_CODE
    path = station(tmp_path)
    data = yaml.safe_load(path.read_text())
    data['workspace'] = {'low': [-1]*3, 'high': [1]*3}
    path.write_text(yaml.safe_dump(data))
    command = ['--station', str(path), 'workspace', 'validate', '--pose', '2', '0', '0', '0', '0', '0', '1']
    assert main(['--json', *command]) == 0
    result = json.loads(capsys.readouterr().out)
    assert result['valid'] is False
    assert result['violations']
    assert main(command) == WORKSPACE_INVALID_EXIT_CODE
    assert 'false' in capsys.readouterr().out


def test_legacy_sidecar_input_rotation_is_not_a_compared_semantic():
    from real_world_rl.action_contract import compared_experiment
    control = {'version': 'spacemouse_serl_v1', 'mode': 'serl', 'motion_threshold': .001,
               'gripper': 'serl_buttons'}
    legacy = {**control, 'input_rotation': np.eye(3).tolist(),
              'input_rotation_semantics': 'base_R_input_direction_preserving_cube_v1'}
    assert compared_experiment({'sources': {'teleop_control': legacy}}) == compared_experiment(
        {'sources': {'teleop_control': control, 'teleop_input': {'yaw_rad': .3}}})
    assert compared_experiment({'teleop_control': legacy}) == compared_experiment({'teleop_control': control})


@pytest.mark.parametrize('action', ['profiles', 'workspace', 'calibrate'])
def test_cli_edits_preserve_station_and_profile_text(tmp_path, capsys, monkeypatch, action):
    from test_spacemouse_calibration import samples
    from test_yaml_edit import assert_only_key_changed

    path = station(tmp_path)
    data = yaml.safe_load(path.read_bytes())
    data['workspace'] = {'frame': 'table', 'low': [-1, -1, -1], 'high': [2, 2, 2]}
    # Deliberately unsorted, quoted, and spaced like a hand-maintained station.
    before = '# Station notes\n' + yaml.safe_dump({'robot': data}, sort_keys=False)
    before = before.replace('  name: left\n', '  name:  "left"  # operator label\n\n')
    before = before.replace('  robot_ip: 172.16.0.2\n', "  robot_ip: '172.16.0.2' # FCI\n")
    before += '\n# End of station\n'
    path.write_text(before)
    profile_file = tmp_path / 'profiles.yaml'
    profile_before = ("# Direction notes\nprofiles:\n"
                      "  front: {frame: 'table', yaw_rad: 0.1, axis_mapping_version: "
                      + str(AXIS_MAPPING_VERSION) + "} # front\n\n"
                      "  rear: {frame: \"table\", yaw_rad: 0.4, axis_mapping_version: "
                      + str(AXIS_MAPPING_VERSION) + "} # keep rear\n"
                      "schema_version: 1  # schema\n")
    profile_file.write_text(profile_before)
    prefix = ['--station', str(path), '--yes', '--json']
    if action == 'profiles':
        command = ['teleop', 'profiles', 'use', '--profile', 'front']
    elif action == 'workspace':
        command = ['workspace', 'set', '--frame', 'table', '--low', '0', '0', '0',
                   '--high', '1', '1', '1', '--save']
    else:
        sample_file = tmp_path / 'samples.json'
        sample_data = samples(0)
        sample_file.write_text(json.dumps(sample_data))
        report = tmp_path / 'report'
        report.mkdir()
        def collect(config, **kwargs):
            return {'+x': sample_data['+x']}, {'report_directory': str(report),
                                             'device_identity': {'vendor_id': 1, 'product_id': 2}}
        monkeypatch.setattr('real_world_rl.teleop.input_diagnostics.collect', collect)
        command = ['teleop', 'calibrate', '--profile', 'front', '--samples', str(sample_file),
                   '--capture-label', '+x', '--use']
    assert main(prefix + command) == 0
    capsys.readouterr()
    key = 'workspace' if action == 'workspace' else 'teleop_profiles'
    assert_only_key_changed(before, path.read_text(), ('robot', key))
    assert '  name:  "left"  # operator label\n\n' in path.read_text()
    assert path.read_text().endswith('\n# End of station\n')
    if action == 'calibrate':
        assert_only_key_changed(profile_before, profile_file.read_text(), ('profiles', 'front'))
        assert '# keep rear' in profile_file.read_text()
    else:
        assert profile_file.read_text() == profile_before


def test_calibrate_creates_new_profiles_file(tmp_path, capsys):
    from test_spacemouse_calibration import samples
    path = station(tmp_path)
    sample_file = tmp_path / 'samples.json'
    sample_file.write_text(json.dumps(samples(0)))
    output = tmp_path / 'new-profiles.yaml'
    assert not output.exists()
    assert main(['--station', str(path), '--yes', '--json', 'teleop', 'calibrate',
                 '--samples', str(sample_file), '--profile', 'front', '--save',
                 '--output', str(output)]) == 0
    result = json.loads(capsys.readouterr().out)
    assert result['saved']['backup'] is None
    assert yaml.safe_load(output.read_bytes()) == {
        'schema_version': 1, 'profiles': {'front': result['candidate']}}
