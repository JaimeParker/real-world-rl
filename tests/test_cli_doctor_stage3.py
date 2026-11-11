"""Shared host/device doctor envelope and confirmation policy, with probes stubbed."""
import dataclasses
import json
from types import SimpleNamespace

import pytest

from real_world_rl.cli import doctor, main
from real_world_rl.cli.doctor_repair import Action
from robot_infra.cli import main as device_main
from robot_infra.doctor import cli as device_doctor
from robot_infra.doctor.checks import CheckResult


@pytest.fixture
def isolated_reports(tmp_path, monkeypatch):
    monkeypatch.setenv('RWRL_DOCTOR_REPORT_DIR', str(tmp_path / 'reports'))


@pytest.mark.parametrize('failed', [False, True])
@pytest.mark.parametrize('multi', [False, True])
@pytest.mark.parametrize('install_sdk', [False, True])
def test_both_doctors_share_envelope_without_changing_device_records(
    isolated_reports, monkeypatch, capsys, failed, multi, install_sdk,
):
    row = CheckResult(1, 'sdk:arm', not failed, 'checked', hint='install it' if failed else '')
    results = {'left': [row], **({'right': [row]} if multi else {})}
    monkeypatch.setattr(device_doctor, 'resolve_station_config_path', lambda path: 'station.yaml')
    def device_probe(*args, **kwargs):
        print('SDK chatter')
        return results
    monkeypatch.setattr(device_doctor, 'doctor_station', device_probe)
    def inspect(args, host):
        host.add('sample', 'FAIL' if failed else 'PASS', 'checked')
        return {}
    monkeypatch.setattr(doctor, 'inspect', inspect)
    extra = ['--install-sdk'] if install_sdk else []
    assert device_main(['doctor', '--json', *extra]) == int(failed)
    captured = capsys.readouterr()
    assert 'SDK chatter' in captured.err
    device_payload = json.loads(captured.out)
    assert main(['doctor', '--json', *extra]) == int(failed)
    host_payload = json.loads(capsys.readouterr().out)
    for payload in (device_payload, host_payload):
        assert payload['schema_version'] == 1
        assert payload['status'] == ('FAIL' if failed else 'PASS')
        assert isinstance(payload['checks'], list) and isinstance(payload['repairs'], list)
    record = {'layer': 1, 'name': 'sdk:arm', 'ok': not failed, 'message': 'checked',
              'hint': 'install it' if failed else '', 'skipped': False, 'warning': False}
    inner = {'left': [record], 'right': [record]} if multi else [record]
    expected = {'results': inner, 'install_sdk': []} if install_sdk else inner
    assert device_payload['results'] == expected


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


@pytest.mark.parametrize('binary', ['rwrl', 'robot-infra'])
def test_yes_without_repairs_still_checks_and_preserves_failure(
    isolated_reports, monkeypatch, capsys, binary,
):
    checked = []
    monkeypatch.setattr('builtins.input', lambda *a: pytest.fail('--yes must skip prompts'))
    if binary == 'rwrl':
        def inspect(args, host):
            checked.append(args.yes)
            host.add('preflight', 'BLOCKED', 'device owned')
            return {}
        monkeypatch.setattr(doctor, 'inspect', inspect)
        invoke = main
    else:
        monkeypatch.setattr(device_doctor, 'resolve_station_config_path', lambda path: 'station.yaml')
        def probe(*a, **kw):
            checked.append(True)
            return {'arm': [CheckResult(1, 'preflight', False, 'device owned')]}
        monkeypatch.setattr(device_doctor, 'doctor_station', probe)
        invoke = device_main
    assert invoke(['doctor', '--yes', '--json']) == 1
    assert checked == [True]
    assert json.loads(capsys.readouterr().out)['status'] == 'FAIL'


def test_rwrl_doctor_station_alias_and_help():
    from real_world_rl.cli import build_parser

    parser = build_parser()
    canonical = parser.parse_args(['doctor', '--station', 'station.yaml'])
    alias = parser.parse_args(['doctor', '--config', 'station.yaml'])
    assert vars(canonical) == vars(alias)
    action = next(action for action in alias._parser._actions if '--config' in action.option_strings)
    assert '(alias, will be removed)' in action.help
