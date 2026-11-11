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
