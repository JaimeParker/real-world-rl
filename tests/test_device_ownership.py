import pytest

from real_world_rl.common.device_ownership import controlling_command, process_owners


def test_loaded_sdk_process_is_reported_without_argv_secrets(tmp_path):
    process = tmp_path / '124'
    process.mkdir()
    (process / 'cmdline').write_bytes(b'python\0--token\0secret\0')
    (process / 'maps').write_text('/env/site-packages/franky/_franky.so\n')
    assert process_owners(proc=tmp_path) == ['pid 124: loaded hardware SDK']


def test_empty_or_unreadable_process_metadata_is_tolerated(tmp_path):
    (tmp_path / '124').mkdir()
    assert process_owners(proc=tmp_path) == []


def test_in_process_rwrl_daemon_counts_as_owner(tmp_path):
    for pid, argv in (('201', b'/venv/bin/python\0/venv/bin/rwrl\0daemon\0--robot\0left\0'),
                      ('202', b'python\0-m\0real_world_rl.cli\0daemon\0'),
                      ('203', b'python\0-m\0robot_infra.server\0')):
        process = tmp_path / pid
        process.mkdir()
        (process / 'cmdline').write_bytes(argv)
    assert process_owners(proc=tmp_path) == [
        'pid 201: rwrl daemon', 'pid 202: real_world_rl.cli daemon', 'pid 203: robot_infra.server']


def test_offline_rwrl_process_is_not_an_owner(tmp_path):
    for pid, argv in (('301', b'/venv/bin/rwrl\0teleop\0inspect-stop\0--trace\0t.json\0'),
                      ('302', b'/venv/bin/rwrl\0--help\0')):
        process = tmp_path / pid
        process.mkdir()
        (process / 'cmdline').write_bytes(argv)
    assert process_owners(proc=tmp_path) == []


@pytest.mark.parametrize('argv, expected', [
    (['rwrl', 'daemon'], 'rwrl daemon'),
    (['/venv/bin/python', '/venv/bin/rwrl', '--station', 's.yaml', '--yes', 'home', 'go'], 'rwrl home'),
    (['rwrl', '--station=s.yaml', '--direct', 'gripper', 'open'], 'rwrl gripper'),
    (['rwrl', '--json', 'status'], 'rwrl status'),
    (['python', '-m', 'real_world_rl.cli', '--json', 'status'], 'real_world_rl.cli status'),
    # The per-robot child of a multi-robot `rwrl daemon` (robot_infra.server.daemon.station_daemon_argv).
    (['python', '-m', 'real_world_rl.cli', 'daemon', '--station', '/s.yaml', '--robot', 'left',
      '--watchdog-s', '0.5', '--transport', 'grpc'], 'real_world_rl.cli daemon'),
    # Process names of the previous release, still recognised while old daemons may run.
    (['python', '-m', 'robot_infra.cli', 'daemon'], 'robot_infra.cli daemon'),
    (['robot-infra', 'daemon'], 'robot-infra daemon'),
    (['robot-infra', '--station', 's.yaml', 'setup'], 'robot-infra setup'),
    (['rwrl', 'teleop', 'run', '--record', 'x.pkl'], 'rwrl teleop run'),
    (['rwrl', 'teleop', 'calibrate', '--profile', 'p'], 'rwrl teleop calibrate'),
    (['rwrl', 'teleop', 'diagnose'], 'rwrl teleop diagnose'),
    (['rwrl', 'teleop', 'read', '--duration-s', '5'], 'rwrl teleop read'),
    (['rwrl', 'camera', 'show'], 'rwrl camera'),
    (['robot-infra', 'gello-calibrate', '--port', '/dev/x', '--joint-ids', '1'], 'robot-infra gello-calibrate'),
    (['rwrl', 'workspace', 'capture'], 'rwrl workspace capture'),
    (['rwrl', '--preset', 'p.yaml', 'train'], 'rwrl train'),
    (['rwrl', 'record'], 'rwrl record'),
    (['rwrl', 'deploy'], 'rwrl deploy'),
    (['rwrl', 'collect-reward'], 'rwrl collect-reward'),
    (['python', 'examples/record_real_demos.py'], 'record_real_demos.py'),
    (['python', '-m', 'robot_infra.server'], 'robot_infra.server'),
])
def test_connecting_commands_are_controlling(argv, expected):
    assert controlling_command(argv) == expected


@pytest.mark.parametrize('argv', [
    ['rwrl'],
    ['rwrl', '--help'],
    ['rwrl', 'daemon', '--help'],
    ['rwrl', 'teleop', 'inspect-stop', '--trace', 't.json'],
    ['rwrl', 'teleop', 'profiles', 'list'],
    ['rwrl', 'teleop'],
    ['rwrl', 'workspace', 'show'],
    ['rwrl', '--station', 's.yaml', 'workspace', 'show'],
    ['rwrl', 'doctor', '--fix'],
    ['robot-infra', 'doctor'],
    ['robot-infra', '--json', 'discover'],
    ['python', '-m', 'real_world_rl.cli', '--help'],
    ['python', '-m', 'robot_infra.cli', 'doctor'],
    ['tail', '-f', 'rwrl.log'],
    ['python', 'unrelated.py'],
    [''],
])
def test_offline_commands_are_not_controlling(argv):
    assert controlling_command(argv) is None
