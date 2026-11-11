from real_world_rl.common.device_ownership import process_owners


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
        'pid 201: rwrl', 'pid 202: real_world_rl.cli', 'pid 203: robot_infra.server']
