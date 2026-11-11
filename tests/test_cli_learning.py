"""Learning adapters preserve entrypoint ownership without opening devices."""
import argparse
from dataclasses import dataclass
import sys
from types import ModuleType

import pytest

from real_world_rl.cli import main
from real_world_rl.cli.cmd_learning import COMMANDS, LearningCommand
from real_world_rl.cli.command import Context
from real_world_rl.cli.errors import CliError, USAGE_EXIT_CODE


@pytest.fixture
def target(monkeypatch):
    def install(command, function):
        module_name, name = command.target.split(':')
        module = ModuleType(module_name)
        setattr(module, name, function)
        monkeypatch.setitem(sys.modules, module_name, module)
    return install


@pytest.mark.parametrize('command', COMMANDS, ids=lambda command: command.name)
def test_learning_argv_passthrough_and_exit_code(target, command, capsys):
    seen = []

    def run(argv):
        seen.append(argv)
        print('entrypoint output')
        return 7

    target(command, run)
    remaining = ['algorithm', '--config', 'preset.yaml', '--value=-1', '--json', 'literal']
    assert main([command.name, *remaining]) == 7
    assert seen == [remaining]
    assert capsys.readouterr().out.splitlines() == ['entrypoint output']


@pytest.mark.parametrize('command', COMMANDS, ids=lambda command: command.name)
def test_preset_maps_to_leading_config(target, command):
    seen = []
    target(command, lambda argv: seen.append(argv))
    assert main(['--preset', 'P.yaml', command.name, 'algo', '--value', '1']) == 0
    assert seen == [['--config', 'P.yaml', 'algo', '--value', '1']]


@pytest.mark.parametrize('config', [['--config', 'Q.yaml'], ['--config=Q.yaml']])
def test_preset_config_conflict_prevents_import(monkeypatch, capsys, config):
    monkeypatch.setattr('real_world_rl.cli.cmd_learning.import_module',
                        lambda name: pytest.fail('conflict must precede target import'))
    assert main(['--preset', 'P.yaml', 'train', 'algo', *config]) == 2
    assert '--preset conflicts' in capsys.readouterr().err


@pytest.mark.parametrize('command', COMMANDS, ids=lambda command: command.name)
@pytest.mark.parametrize('option', [
    ['--station', 'S'], ['--robot', 'R'], ['--station', ''], ['--robot', ''],
    ['--direct'], ['--mock'], ['--json'], ['--yes'],
])
def test_inapplicable_globals_fail_before_import(monkeypatch, capsys, command, option):
    monkeypatch.setattr('real_world_rl.cli.cmd_learning.import_module',
                        lambda name: pytest.fail('invalid globals must precede import'))
    assert main([*option, command.name]) == 2
    assert (f'{option[0]} not applicable to rwrl {command.name}; '
            "pass the entrypoint's own flags") in capsys.readouterr().err


def test_string_system_exit_is_usage_error(target, capsys):
    def run(argv):
        raise SystemExit('Configuration error')

    target(COMMANDS[0], run)
    assert main(['train']) == 2
    captured = capsys.readouterr()
    assert 'Configuration error' in captured.err
    assert captured.out == ''


def context():
    args = argparse.Namespace(station=None, robot=None, direct=False, mock=False,
                              json=False, yes=False, preset=None)
    return Context(args, ['unchanged'])


def test_config_error_becomes_cli_usage_error(target):
    from rl_garden.common.effective_config import ConfigError

    def run(argv):
        raise ConfigError('invalid preset')

    target(COMMANDS[0], run)
    ctx = context()
    with pytest.raises(CliError, match='invalid preset') as error:
        COMMANDS[0].run(ctx.args, ctx)
    assert error.value.code == USAGE_EXIT_CODE


def test_tyro_usage_error_becomes_cli_usage_error(target):
    import tyro

    @dataclass
    class Args:
        count: int

    def run(argv):
        tyro.cli(Args, args=argv)

    target(COMMANDS[0], run)
    ctx = context()
    ctx.remaining = ['--count', 'not-an-int']
    with pytest.raises(CliError) as error:
        COMMANDS[0].run(ctx.args, ctx)
    assert error.value.code == USAGE_EXIT_CODE


@pytest.mark.parametrize('failure', [RuntimeError('runtime'), ValueError('runtime'), SystemExit(0), SystemExit(7)])
def test_other_exceptions_propagate(target, failure):
    def run(argv):
        raise failure

    target(COMMANDS[0], run)
    ctx = context()
    with pytest.raises(type(failure)) as error:
        COMMANDS[0].run(ctx.args, ctx)
    assert error.value is failure


def test_argv_object_is_handed_to_target_unchanged(target):
    ctx = context()
    target(COMMANDS[0], lambda argv: None if argv is ctx.remaining else pytest.fail('argv copied'))
    assert COMMANDS[0].run(ctx.args, ctx).exit_code == 0


def test_train_help_is_the_entrypoints_tyro_help(capsys):
    assert main(['train', '--help']) == 0
    output = capsys.readouterr().out
    assert 'usage:' in output.lower()
    assert '--print-config' in output
    assert 'real_world' in output


def test_training_main_passes_argv_to_registry(monkeypatch):
    from real_world_rl.training import __main__ as entrypoint
    calls = []
    monkeypatch.setattr(entrypoint.registry, 'run_cli', lambda argv: calls.append(argv))
    argv = ['algo', '--print-config']
    entrypoint.main(argv)
    assert calls == [argv]
    assert calls[0] is argv


def test_deploy_main_cli_parses_then_calls_existing_runner(monkeypatch):
    import tyro
    from real_world_rl import deploy
    parsed = object()
    calls = []
    argv = ['--help']

    def parse(cls, *, args):
        assert cls is deploy.DeployArgs
        assert args is argv
        return parsed

    monkeypatch.setattr(tyro, 'cli', parse)
    monkeypatch.setattr(deploy, 'main', lambda args: calls.append(args) or 7)
    assert deploy.main_cli(argv) == 7
    assert calls == [parsed]


def test_collect_main_cli_preserves_backend_selection(monkeypatch):
    from real_world_rl.reward import collect_data
    seen = []
    monkeypatch.setattr(collect_data, 'main', lambda *, argv: seen.append(argv))
    argv = ['--env-backend', 'real_robot', '--help']
    collect_data.main_cli(argv)
    assert seen == [argv]
    assert seen[0] is argv


def test_registration_defers_help_to_entrypoint():
    command = LearningCommand('test', 'test', 'unused:main')
    assert command.parser_options['add_help'] is False
    assert command.accepts_remaining
