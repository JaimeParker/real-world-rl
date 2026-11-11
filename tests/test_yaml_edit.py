"""Source preservation and shared atomic YAML editing contracts."""
import difflib
from pathlib import Path

import pytest
import yaml

from real_world_rl.cli.errors import CliError
from real_world_rl.common.yaml_edit import atomic_replace, load_editable, render, render_changes


def assert_only_key_changed(before, after, path):
    """Compare every line outside a selected key and its safe-loaded semantics."""
    def bounds(text):
        node = yaml.compose(text, Loader=yaml.SafeLoader)
        for name in path:
            key, node = next((key, value) for key, value in node.value if key.value == name)
        def end(value):
            if isinstance(value, yaml.MappingNode) and not value.flow_style:
                return end(value.value[-1][1])
            if isinstance(value, yaml.SequenceNode) and not value.flow_style:
                return end(value.value[-1])
            return value.end_mark.line + (value.end_mark.column > 0)
        return key.start_mark.line, end(node)
    old_start, old_end = bounds(before)
    new_start, new_end = bounds(after)
    for tag, i, j, k, m in difflib.SequenceMatcher(
            a=before.splitlines(keepends=True), b=after.splitlines(keepends=True),
            autojunk=False).get_opcodes():
        if tag != 'equal':
            assert old_start <= i <= j <= old_end
            assert new_start <= k <= m <= new_end
    old, new = yaml.safe_load(before), yaml.safe_load(after)
    old_parent, new_parent = old, new
    for key in path[:-1]:
        old_parent, new_parent = old_parent[key], new_parent[key]
    del old_parent[path[-1]], new_parent[path[-1]]
    assert old == new


@pytest.mark.parametrize('newline', ['\n', '\r\n'])
def test_editable_mapping_preserves_untouched_bytes(tmp_path, newline):
    before = ('# station\nrobot:\n    name:  "left"  # retained\n\n'
              '    home_joints: [0.2, 0.2] # home\n'
              '    robot_ip: \'172.16.0.2\'\n# footer\n').replace('\n', newline)
    path = tmp_path / 'station.yaml'
    path.write_bytes(before.encode())
    document, raw = load_editable(path)
    assert raw == before.encode()
    assert render(document).encode() == raw
    document['robot']['home_joints'] = [0.0, 0.0]
    after = render(document)
    assert_only_key_changed(before, after, ('robot', 'home_joints'))
    assert '# retained' in after and '# footer' in after
    assert yaml.safe_load(after)['robot']['home_joints'] == [0., 0.]


def test_new_file_and_new_keys_append(tmp_path):
    path = tmp_path / 'profiles.yaml'
    document, raw = load_editable(path)
    assert raw is None
    document.update(schema_version=1, profiles={'front': {'frame': 'base', 'yaw_rad': 0.2}})
    saved = atomic_replace(path, render(document), raw)
    assert saved['backup'] is None
    assert yaml.safe_load(path.read_bytes()) == document
    original = b'# heading\nrobot:\n  name: "left"\n\n# footer\nother: 2\n'
    result = render_changes({'robot': {'name': 'left', 'workspace': {'frame': 'base'}},
                             'other': 2}, original)
    assert result == ('# heading\nrobot:\n  name: "left"\n  workspace:\n'
                      '    frame: base\n\n# footer\nother: 2\n')


def test_atomic_replace_failure_keeps_source_and_backup(tmp_path, monkeypatch):
    path = tmp_path / 'station.yaml'
    original = b'# exact\nvalue: 1\n'
    path.write_bytes(original)
    def fail(*args):
        raise OSError('replacement failed')
    monkeypatch.setattr('real_world_rl.common.yaml_edit.os.replace', fail)
    with pytest.raises(OSError, match='replacement failed'):
        atomic_replace(path, 'value: 2\n', original)
    assert path.read_bytes() == original
    assert list(tmp_path.glob('*.bak'))[0].read_bytes() == original
    assert not list(tmp_path.glob('.station.yaml.*'))


def test_home_rejects_stale_preset(tmp_path):
    from real_world_rl.cli.preset_edit import Preset
    path = tmp_path / 'preset.yaml'
    path.write_text('task: {reset: {home_joints: [0.2, 0.2]}}\n')
    preset = Preset.load(str(path))
    path.write_text('# external edit\ntask: {}\n')
    with pytest.raises(CliError, match='changed during review'):
        preset.save_home([0., 0.], [], num_joints=2)
    assert path.read_text() == '# external edit\ntask: {}\n'
    assert not list(tmp_path.glob('*.bak'))


def test_preview_matches_saved_text(tmp_path, capsys):
    from types import SimpleNamespace
    from real_world_rl.cli.station_edit import atomic_save, confirm_write
    path = tmp_path / 'station.yaml'
    before = '# retained\nrobot: {name: "left", home_joints: [0.2]}\n'
    path.write_text(before)
    desired = yaml.safe_load(before)
    desired['robot']['home_joints'] = [0.0]
    from real_world_rl.cli.command import Context
    confirm_write(Context(SimpleNamespace(yes=True)), path, desired, before.encode())
    preview = capsys.readouterr().err
    saved = atomic_save(path, desired, before.encode())
    after = path.read_text()
    assert preview == ''.join(difflib.unified_diff(
        before.splitlines(keepends=True), after.splitlines(keepends=True),
        fromfile=str(path), tofile=str(path) + ' (proposed)')) + '\n'
    assert Path(saved['backup']).read_text() == before
    assert 'warning' not in saved
