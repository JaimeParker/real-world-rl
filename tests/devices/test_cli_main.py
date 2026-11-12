"""Native device command registration in rwrl and the module entry contracts; implementations are stubbed."""
import argparse
from importlib import import_module
import json
import subprocess
import sys

import pytest

from robot_infra.cli import COMMANDS
from robot_infra.cli.command import Context
from robot_infra.cli.main import build_parser
from real_world_rl.cli import main


ARGUMENTS = {
    'setup': ['--mock', '--non-interactive'],
    'daemon': ['--config', 'station.yaml', '--robot', 'left', '--no-preflight'],
    'supervise': ['--grace-s', '1', '--', 'python', '-c', 'pass', '--json'],
    'discover': ['--config', 'station.yaml'],
    'gello-calibrate': ['--port', '/dev/example', '--joint-ids', *map(str, range(1, 8))],
}


def test_rwrl_help_lists_the_five_native_commands(capsys):
    assert main(['--help']) == 0
    help_text = capsys.readouterr().out
    assert [command.name for command in COMMANDS] == list(ARGUMENTS)
    for command in COMMANDS:
        assert command.name in help_text


def flags(parser):
    return {flag for action in parser._actions for flag in action.option_strings}


@pytest.mark.parametrize('command', COMMANDS, ids=lambda command: command.name)
def test_native_help_preserves_module_flags(command, capsys):
    module = import_module(command.module)
    parser = build_parser(commands=COMMANDS, prog='rwrl', description='test')
    subcommands = next(action for action in parser._actions if isinstance(action, argparse._SubParsersAction))
    native = subcommands.choices[command.name]
    original = module.build_parser()
    assert flags(native) == flags(original)
    assert main([command.name, '--help']) == 0
    help_text = capsys.readouterr().out
    for flag in flags(original):
        assert flag in help_text


@pytest.mark.parametrize('command', COMMANDS, ids=lambda command: command.name)
def test_module_main_and_rwrl_reach_same_run(monkeypatch, command, capsys):
    module = import_module(command.module)
    calls = []

    def run(args, ctx, **kwargs):
        if command.name == 'gello-calibrate':
            module.validate_args(args, module.build_parser())
        calls.append((vars(args).copy(), ctx))
        print('native output')
        return 17

    monkeypatch.setattr(module, 'run', run)
    argv = ARGUMENTS[command.name]
    assert module.main(argv) == 17
    assert main([command.name, *argv]) == 17
    module_args, module_ctx = calls[0]
    native_args, native_ctx = calls[1]
    for key, value in module_args.items():
        assert native_args[key] == value
    assert module_ctx is None
    assert isinstance(native_ctx, Context)
    assert capsys.readouterr().out.splitlines() == ['native output', 'native output']


@pytest.mark.parametrize('name', ['setup', 'daemon', 'supervise', 'gello-calibrate'])
def test_unsupported_json_is_usage_error_before_run(monkeypatch, capsys, name):
    command = next(command for command in COMMANDS if command.name == name)
    module = import_module(command.module)
    monkeypatch.setattr(module, 'run', lambda *args: pytest.fail('unsupported JSON must not execute'))
    assert main(['--json', name, *ARGUMENTS[name]]) == 2
    captured = capsys.readouterr()
    assert f'{name} has no JSON output' in captured.err
    assert len(captured.out.splitlines()) == 1


DISCOVERY_KEYS = {'schema_version', 'cameras', 'teleops', 'can_interfaces', 'hosts', 'hid_devices', 'unknown_hid'}


def stub_discovery(monkeypatch, report):
    from robot_infra.discovery import report as discovery_cli

    monkeypatch.setattr(discovery_cli, '_station_path', lambda explicit: None)
    monkeypatch.setattr(discovery_cli, 'discover_all', lambda host_ips, spacemouse_table=None: report)


def found_hardware():
    from robot_infra.discovery.enumerate import CanInfo, DiscoveryReport, HostInfo

    return DiscoveryReport(
        cameras={'realsense': {'B2', 'A1'}}, teleops={'gello': {'/dev/ttyUSB0'}, 'spacemouse': set()},
        can_interfaces={'can0': CanInfo('can0', 'up', 1000000)},
        hosts={'172.16.0.2': HostInfo('172.16.0.2', True, [22, 80])}, hid_devices=['/dev/hidraw3'],
        unknown_hid=[UNKNOWN_BT])


UNKNOWN_BT = {'vid': '256f', 'pid': 'c63a', 'product': 'SpaceMouse BT', 'path': '/dev/hidraw6',
              'message': 'unknown 3Dconnexion-vendor HID device 256f:c63a; add it to spacemouse-devices.yaml '
                         '(see docs/spacemouse-commissioning.md#site-device-table)'}


def test_discover_json_lists_every_category(monkeypatch, capsys):
    stub_discovery(monkeypatch, found_hardware())
    assert main(['--json', 'discover']) == 0
    out = capsys.readouterr().out
    assert len(out.splitlines()) == 1
    payload = json.loads(out)
    assert set(payload) == DISCOVERY_KEYS
    assert payload['cameras'] == [{'backend': 'realsense', 'serial': 'A1'}, {'backend': 'realsense', 'serial': 'B2'}]
    assert payload['teleops'] == [{'backend': 'gello', 'device': '/dev/ttyUSB0'}]
    assert payload['can_interfaces'] == [{'name': 'can0', 'operstate': 'up', 'bitrate': 1000000}]
    assert payload['hosts'] == [{'ip': '172.16.0.2', 'ping_ok': True, 'open_ports': [22, 80]}]
    assert payload['hid_devices'] == ['/dev/hidraw3']
    assert payload['unknown_hid'] == [UNKNOWN_BT]


def test_discover_json_gives_empty_lists_when_nothing_is_found(monkeypatch, capsys):
    from robot_infra.discovery.enumerate import DiscoveryReport

    stub_discovery(monkeypatch, DiscoveryReport())
    assert main(['--json', 'discover']) == 0
    payload = json.loads(capsys.readouterr().out)
    assert set(payload) == DISCOVERY_KEYS
    assert all(payload[key] == [] for key in DISCOVERY_KEYS - {'schema_version'})


def test_discover_text_output_is_unchanged(monkeypatch, capsys):
    report = found_hardware()
    stub_discovery(monkeypatch, report)
    assert main(['discover']) == 0
    assert capsys.readouterr().out == report.format() + '\n'


def test_context_defaults_to_device_loader_and_accepts_connection_factory(monkeypatch):
    from robot_infra.discovery import autoconfig
    calls = []
    station = object()
    monkeypatch.setattr(autoconfig, 'resolve_station_config_path', lambda path: path)
    monkeypatch.setattr(autoconfig, 'load_station', lambda path: calls.append(path) or station)
    args = argparse.Namespace(station='station.yaml')
    ctx = Context(args, connection_factory=lambda value: value)
    assert calls == []
    assert ctx.station() is station
    assert ctx.station() is station
    assert calls == ['station.yaml']
    assert ctx.open_robot() is args


@pytest.mark.parametrize('name', [command.name for command in COMMANDS])
def test_native_command_help_is_light(name):
    code = '''
import importlib.abc
import runpy
import sys
class NoHeavyImports(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if (fullname.split('.')[0] in {'torch', 'rl_garden', 'numpy', 'yaml'}
                or (fullname.startswith('robot_infra.') and fullname != 'robot_infra.cli'
                    and not fullname.startswith('robot_infra.cli.'))):
            raise AssertionError('heavy import: ' + fullname)
sys.meta_path.insert(0, NoHeavyImports())
sys.argv = ['rwrl', *sys.argv[1:], '--help']
runpy.run_module('real_world_rl.cli', run_name='__main__')
'''
    result = subprocess.run([sys.executable, '-c', code, name],
                            text=True, capture_output=True, check=False)
    assert result.returncode == 0, result.stderr
    assert f'rwrl {name}' in result.stdout
