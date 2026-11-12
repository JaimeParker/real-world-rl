"""The doctor's JSON envelope and confirmation policy, with probes stubbed."""
import dataclasses
import json
from types import SimpleNamespace

import pytest

from real_world_rl.cli import main
from real_world_rl.doctor import report as doctor
from real_world_rl.doctor.repair import Action


@pytest.fixture
def isolated_reports(tmp_path, monkeypatch):
    monkeypatch.setenv('RWRL_DOCTOR_REPORT_DIR', str(tmp_path / 'reports'))


@pytest.mark.parametrize('failed', [False, True])
@pytest.mark.parametrize('install_sdk', [False, True])
def test_doctor_json_envelope(isolated_reports, monkeypatch, capsys, failed, install_sdk):
    rows = {'left': [{'layer': 1, 'name': 'sdk:arm', 'ok': not failed, 'message': 'checked'}]}
    def inspect(args, host):
        host.add('sample', 'FAIL' if failed else 'PASS', 'checked')
        return rows
    monkeypatch.setattr(doctor, 'inspect', inspect)
    extra = ['--install-sdk'] if install_sdk else []
    assert main(['doctor', '--json', *extra]) == int(failed)
    payload = json.loads(capsys.readouterr().out)
    assert payload['schema_version'] == 1
    assert payload['status'] == ('FAIL' if failed else 'PASS')
    assert isinstance(payload['checks'], list) and isinstance(payload['repairs'], list)
    # The device rows stay as the probe printed them, beside the findings made from them.
    assert payload['results'] == rows


@pytest.mark.parametrize('answer,confirmed', [('y', True), (' Y ', True), ('yes', False),
                                            ('true', False), ('1', False), ('', False), ('n', False)])
def test_host_doctor_confirms_only_y(isolated_reports, monkeypatch, capsys, answer, confirmed):
    executions = []
    action = Action('sample', 'sample', 'repair', ['true'])
    def inspect(args, host):
        host.add('sample', 'PASS' if executions else 'FAIL', 'checked')
        host.actions = [] if executions else [dataclasses.replace(action)]
        return {}
    monkeypatch.setattr(doctor, 'inspect', inspect)
    monkeypatch.setattr(doctor, 'execute', lambda *a, **kw: executions.append(kw))
    monkeypatch.setattr(doctor.sys, 'stdin', SimpleNamespace(isatty=lambda: True))
    monkeypatch.setattr('builtins.input', lambda *a: answer)
    assert main(['doctor', '--fix', '--json']) == (0 if confirmed else 1)
    payload = json.loads(capsys.readouterr().out)
    assert payload['confirmed'] is confirmed
    assert bool(executions) is confirmed


def test_yes_without_repairs_still_checks_and_preserves_failure(isolated_reports, monkeypatch, capsys):
    checked = []
    monkeypatch.setattr('builtins.input', lambda *a: pytest.fail('--yes must skip prompts'))
    def inspect(args, host):
        checked.append(args.yes)
        host.add('preflight', 'BLOCKED', 'device owned')
        return {}
    monkeypatch.setattr(doctor, 'inspect', inspect)
    assert main(['doctor', '--yes', '--json']) == 1
    assert checked == [True]
    assert json.loads(capsys.readouterr().out)['status'] == 'FAIL'


def test_rwrl_doctor_takes_only_the_station_flag():
    from real_world_rl.cli import build_parser

    parser = build_parser()
    args = parser.parse_args(['doctor', '--station', 'station.yaml'])
    assert args.station == 'station.yaml'
    assert not any('--config' in action.option_strings for action in args._parser._actions)
