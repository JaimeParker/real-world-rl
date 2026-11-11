from real_world_rl.common.provenance import source_fingerprint, digest_document


def test_source_digest_tracks_content_and_names_not_location(tmp_path):
    first, second = tmp_path / 'one', tmp_path / 'two'
    first.mkdir()
    second.mkdir()
    for root in (first, second):
        (root / 'module.py').write_text('value = 1\n')
    assert source_fingerprint(first)['sha256'] == source_fingerprint(second)['sha256']
    (second / 'module.py').write_text('value = 2\n')
    assert source_fingerprint(first)['sha256'] != source_fingerprint(second)['sha256']
    (second / 'module.py').write_text('value = 1\n')
    (second / 'module.py').rename(second / 'renamed.py')
    assert source_fingerprint(first)['sha256'] != source_fingerprint(second)['sha256']


def test_config_digest_canonicalizes_key_order():
    assert digest_document({'a': 1, 'b': 2}) == digest_document({'b': 2, 'a': 1})
    assert digest_document({'a': 1}) != digest_document({'a': 2})


def test_provenance_paths_are_relative_or_home_anonymous(tmp_path, monkeypatch):
    import json
    from pathlib import Path
    from real_world_rl.common.provenance import portable_path, runtime_provenance
    repo = Path(__file__).resolve().parents[1]
    assert portable_path(repo / 'real_world_rl/common/provenance.py') == 'real_world_rl/common/provenance.py'
    assert portable_path(Path.home() / 'private/station.yaml') == '~/private/station.yaml'
    assert portable_path('/home/another-operator/profiles.yaml') == '~/profiles.yaml'
    file = tmp_path / 'station.yaml'
    file.write_text('test: true')
    monkeypatch.setattr('real_world_rl.common.provenance.importlib.util.find_spec', lambda _: None)
    data = runtime_provenance(files={'station': file, 'missing': Path.home() / 'private/missing.yaml'})
    for value in data['input_files'].values():
        assert not Path(value['path']).is_absolute()
    assert str(Path.home()) not in json.dumps(data)
