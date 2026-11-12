"""Stage-3 CLI contracts; command execution is replaced with stubs."""
from importlib import import_module
from pathlib import Path

import pytest

from real_world_rl.cli.device_commands import COMMANDS
from real_world_rl.cli.errors import (
    ABORTED_EXIT_CODE, DEVICE_EXIT_CODE, DOCTOR_UNRESOLVED_EXIT_CODE,
    ESCALATED_EXIT_CODE, FAILURE_EXIT_CODE, LAUNCH_FAILURE_EXIT_CODE,
    STOP_TRACE_BLOCKED_EXIT_CODE, STOP_TRACE_FAIL_EXIT_CODE, SUCCESS_EXIT_CODE,
    USAGE_EXIT_CODE, WORKSPACE_INVALID_EXIT_CODE,
)
from real_world_rl.cli import main


STATION_COMMANDS = [('setup', '--from', 'from_path'), ('daemon', '--config', 'config'),
                    ('discover', '--config', 'config')]


@pytest.mark.parametrize('name,removed,dest', STATION_COMMANDS)
def test_station_flag_in_module_parser_and_rwrl(monkeypatch, name, removed, dest):
    registration = import_module('real_world_rl.cli.cmd_' + name)
    parser = registration.build_parser()
    assert getattr(parser.parse_args(['--station', 'station.yaml']), dest) == 'station.yaml'
    assert not any(removed in action.option_strings for action in parser._actions)
    calls = []
    monkeypatch.setattr(registration, 'run', lambda args, ctx: calls.append(getattr(args, dest)) or 0)
    for argv in ([name, '--station', 'station.yaml'],
                 ['--station', 'station.yaml', name],
                 ['--station', 'root.yaml', name, '--station', 'station.yaml']):
        assert main(argv) == SUCCESS_EXIT_CODE
    assert calls and set(calls) == {'station.yaml'}
    # The pre-0.1.0 spelling is gone.
    assert main([name, removed, 'station.yaml']) == USAGE_EXIT_CODE


def test_device_doctor_library_parser_has_no_config_alias():
    from real_world_rl.doctor.cli import build_parser

    parser = build_parser()
    assert parser.parse_args(['--station', 'station.yaml']).config == 'station.yaml'
    with pytest.raises(SystemExit):
        parser.parse_args(['--config', 'station.yaml'])


def test_rwrl_global_options_and_no_gello_calibrate_alias(capsys):
    assert main(['--help']) == SUCCESS_EXIT_CODE
    host_help = capsys.readouterr().out
    for flag in ('--station', '--robot', '--preset', '--direct', '--mock', '--json', '--yes'):
        assert flag in host_help
    assert 'gello-calibrate' not in host_help


@pytest.mark.parametrize('flag', ['--preset', '--direct'])
def test_device_command_rejects_unused_globals_before_execution(monkeypatch, capsys, flag):
    monkeypatch.setattr('real_world_rl.cli.cmd_discover.run', lambda *a: pytest.fail('must not run'))
    value = ['preset.yaml'] if flag == '--preset' else []
    assert main(['discover', flag, *value]) == USAGE_EXIT_CODE
    captured = capsys.readouterr()
    assert captured.out == '' and 'unrecognized arguments:' in captured.err


@pytest.mark.parametrize('entrypoint', ['rwrl', 'module'])
@pytest.mark.parametrize('name', ['setup', 'daemon'])
def test_setup_and_daemon_config_errors_use_stderr(monkeypatch, capsys, name, entrypoint):
    command = next(command for command in COMMANDS if command.name == name)
    module = import_module(command.module)
    if name == 'setup':
        def fail(*args):
            raise ValueError('invalid station')
        monkeypatch.setattr(module, 'load_station', fail)
        argv = ['--station', 'bad.yaml']
    else:
        def fail(*args):
            raise FileNotFoundError('missing station')
        monkeypatch.setattr(module, 'resolve_station_config_path', fail)
        argv = []
    code = main([name, *argv]) if entrypoint == 'rwrl' else module.main(argv)
    assert code == USAGE_EXIT_CODE
    captured = capsys.readouterr()
    assert captured.out == '' and 'station' in captured.err


def test_setup_interruption_uses_shared_code_and_stderr(monkeypatch, capsys):
    module = import_module('real_world_rl.setup.command')
    def interrupt(*args):
        raise KeyboardInterrupt
    monkeypatch.setattr(module, 'load_station', interrupt)
    assert module.main(['--station', 'station.yaml']) == ABORTED_EXIT_CODE
    captured = capsys.readouterr()
    assert captured.out == '' and 'aborted; nothing written' in captured.err


def test_exit_code_table_and_development_leftovers():
    assert (SUCCESS_EXIT_CODE, FAILURE_EXIT_CODE, USAGE_EXIT_CODE, DEVICE_EXIT_CODE,
            DOCTOR_UNRESOLVED_EXIT_CODE, STOP_TRACE_FAIL_EXIT_CODE, STOP_TRACE_BLOCKED_EXIT_CODE,
            WORKSPACE_INVALID_EXIT_CODE, ABORTED_EXIT_CODE, ESCALATED_EXIT_CODE,
            LAUNCH_FAILURE_EXIT_CODE) == (0, 1, 2, 1, 1, 3, 4, 5, 130, 1, 127)
    assert 'franka_bridge' not in {command.name for command in COMMANDS}
    root = Path(__file__).resolve().parents[2]
    inspector = root / 'examples/run_twist_input_inspector.py'
    assert inspector.is_file()
    assert 'sys.path' not in inspector.read_text() and 'REPO_ROOT' not in inspector.read_text()
    assert not (root / 'real_world_rl/legacy/teleop/examples/run_twist_input_inspector.py').exists()
    bridge = root / 'real_world_rl/legacy/controller/real/franka_bridge.py'
    assert 'rl_garden.envs.franka_real' not in bridge.read_text().split('from __future__')[0]


def test_missing_station_hint_uses_canonical_flag(tmp_path):
    from real_world_rl.discovery.autoconfig import resolve_station_config_path

    with pytest.raises(FileNotFoundError) as error:
        resolve_station_config_path(cwd=tmp_path, env={'XDG_CONFIG_HOME': str(tmp_path / 'xdg')})
    assert 'Pass --station' in str(error.value) and 'Pass --config' not in str(error.value)


@pytest.mark.parametrize('answer,expected', [('y', True), (' Y ', True), ('yes', False),
                                           ('true', False), ('1', False), ('', False), ('n', False)])
def test_device_doctor_confirms_only_y(monkeypatch, answer, expected):
    from real_world_rl.doctor.cli import _confirmed

    monkeypatch.setattr('builtins.input', lambda *a: answer)
    assert _confirmed('Install? [y/N] ') is expected
