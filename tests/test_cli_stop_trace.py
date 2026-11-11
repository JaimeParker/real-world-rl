import json

from real_world_rl.cli import main


def test_stop_inspection_needs_no_station_and_preserves_blocked_report(tmp_path, capsys):
    path = tmp_path / 'trace.json'
    path.write_text(json.dumps({'schema_version': 1, 'stop_start_s': 1., 'stop_end_s': 2., 'samples': []}))
    assert main(['--json', 'teleop', 'inspect-stop', '--trace', str(path)]) == 4
    report = json.loads(capsys.readouterr().out)
    assert report['status'] == 'BLOCKED'
    assert 'no samples' in report['missing_evidence']
    assert len(report['trace_sha256']) == 64


def test_stop_inspection_rejects_invalid_thresholds(tmp_path, capsys):
    path = tmp_path / 'trace.json'
    path.write_text('{}')
    assert main(['--json', 'teleop', 'inspect-stop', '--trace', str(path),
                 '--thresholds', '{"stationary_s":0}']) == 2


def test_stop_fail_has_its_own_exit_code(tmp_path, capsys):
    from real_world_rl.cli.errors import STOP_TRACE_FAIL_EXIT_CODE, STOP_TRACE_BLOCKED_EXIT_CODE
    path = tmp_path / 'trace.json'
    path.write_text(json.dumps({'schema_version': 1, 'stop_start_s': 1., 'stop_end_s': 2.,
                               'samples': [], 'stop_error': 'DeviceFault: stop failed'}))
    assert main(['--json', 'teleop', 'inspect-stop', '--trace', str(path)]) == STOP_TRACE_FAIL_EXIT_CODE
    assert STOP_TRACE_FAIL_EXIT_CODE not in (0, 1, 2, STOP_TRACE_BLOCKED_EXIT_CODE)
    assert json.loads(capsys.readouterr().out)['status'] == 'FAIL'
