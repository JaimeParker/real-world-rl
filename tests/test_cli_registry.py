"""Registry and orchestration contracts; no device calls or subprocesses."""
import argparse
from contextlib import contextmanager
import io
import json
from pathlib import Path
import sys

import pytest

from real_world_rl.cli import COMMANDS, build_parser, main
from real_world_rl.cli.command import Context, Result
from real_world_rl.cli.errors import (
    CliError, SUCCESS_EXIT_CODE, FAILURE_EXIT_CODE, USAGE_EXIT_CODE,
    DOCTOR_UNRESOLVED_EXIT_CODE, STOP_TRACE_FAIL_EXIT_CODE,
    STOP_TRACE_BLOCKED_EXIT_CODE, WORKSPACE_INVALID_EXIT_CODE,
)


def options(**changes):
    return argparse.Namespace(**dict(dict(station=None, robot=None, preset=None,
                                         direct=False, mock=False, json=False, yes=False), **changes))


def test_registry_contract_and_parser_commands():
    names = [command.name for command in COMMANDS]
    assert names == ["status", "q", "tcp", "stop", "recover", "home", "gripper", "camera",
                     "doctor", "setup", "daemon", "supervise", "discover", "teleop", "workspace",
                     "archive", "train", "record", "deploy", "collect-reward"]
    assert len(names) == len(set(names))
    parser = build_parser()
    subparsers = next(action for action in parser._actions if isinstance(action, argparse._SubParsersAction))
    assert list(subparsers.choices) == names
    for command in COMMANDS:
        assert isinstance(command.help, str)
        assert callable(command.register)
        assert callable(command.run)
        assert subparsers.choices[command.name].get_default("_command") is command
    document = (Path(__file__).resolve().parents[1] / "docs/cli.md").read_text()
    for name in names:
        assert f"`{name}" in document


@pytest.mark.parametrize("mode,answer,allowed,message", [
    ("tty", " y ", True, ""), ("tty", "n", False, "cancelled"),
    ("tty", "yes", False, "cancelled"), ("tty", "", False, "cancelled"),
    ("eof", "", False, "cancelled"), ("yes", "", True, ""),
    ("non-tty", "", False, "without"),
])
@pytest.mark.parametrize("saving", [False, True])
@pytest.mark.parametrize("as_json", [False, True])
def test_shared_confirmation_policy(monkeypatch, capsys, mode, answer, allowed, message, saving, as_json):
    stdin = io.StringIO(answer + "\n")
    monkeypatch.setattr(stdin, "isatty", lambda: mode in ("tty", "eof"))
    monkeypatch.setattr(sys, "stdin", stdin)

    def read(*args):
        if mode == "eof":
            raise EOFError
        assert mode == "tty", "noninteractive confirmation must not read input"
        return answer

    monkeypatch.setattr("builtins.input", read)
    monkeypatch.setattr(stdin, "readline", read)
    ctx = Context(options(yes=mode == "yes", json=as_json))
    if allowed:
        ctx.confirm("Save? [y/N] " if saving else "proceed? [y/N] ", detail="preview", saving=saving)
    else:
        with pytest.raises(CliError, match=message) as error:
            ctx.confirm("Save? [y/N] " if saving else "proceed? [y/N] ", detail="preview", saving=saving)
        assert error.value.code == USAGE_EXIT_CODE
    captured = capsys.readouterr()
    assert "preview" in (captured.err if saving or as_json else captured.out)
    if saving or as_json:
        assert captured.out == ""


def test_generic_dispatch_json_and_result_exit_code(monkeypatch, capsys):
    class NewCommand:
        name = "new-command"
        help = "test registration without dispatcher edits"

        def register(self, parser):
            parser.add_argument("--value", required=True)

        def run(self, args, ctx):
            assert ctx.remaining == []
            print("SDK chatter")
            ctx.confirm("proceed? [y/N] ", detail="target preview")
            return Result({"value": args.value}, "human output", 19)

    monkeypatch.setattr("real_world_rl.cli.COMMANDS", [NewCommand()])
    assert main(["--json", "--yes", "new-command", "--value", "sample"]) == 19
    captured = capsys.readouterr()
    assert captured.out.splitlines() == ['{"value": "sample"}']
    assert "SDK chatter" in captured.err
    assert "target preview" in captured.err


@pytest.mark.parametrize("code,expected", [
    ("Configuration error: bad preset", USAGE_EXIT_CODE),
    (None, SUCCESS_EXIT_CODE),
    (0, SUCCESS_EXIT_CODE),
    (7, 7),
])
@pytest.mark.parametrize("as_json", [False, True])
def test_command_system_exit_codes(monkeypatch, capsys, code, expected, as_json):
    class ExitingCommand:
        name = "exiting"
        help = "raises SystemExit from run"

        def register(self, parser):
            pass

        def run(self, args, ctx):
            raise SystemExit(code)

    monkeypatch.setattr("real_world_rl.cli.COMMANDS", [ExitingCommand()])
    assert main((["--json"] if as_json else []) + ["exiting"]) == expected
    captured = capsys.readouterr()
    assert captured.out == ""
    if isinstance(code, str):
        assert captured.err.strip() == code
    else:
        assert captured.err == ""


@pytest.mark.parametrize("name,argv", [
    ("setup", []),
    ("daemon", []),
    ("supervise", ["--", "python", "child.py"]),
])
def test_native_commands_without_json_fail_before_run(monkeypatch, capsys, name, argv):
    from importlib import import_module
    from real_world_rl.cli.device_commands import COMMANDS as device_commands

    command = next(command for command in device_commands if command.name == name)
    calls = []
    monkeypatch.setattr(import_module(command.module), "run", lambda *args: calls.append(args))
    assert main(["--json", name, *argv]) == USAGE_EXIT_CODE
    assert calls == []
    captured = capsys.readouterr()
    payload = json.loads(captured.out)
    assert captured.out.splitlines() == [json.dumps(payload)]
    assert payload["error"] == f"{name} has no JSON output"
    assert f"{name} has no JSON output" in captured.err


@pytest.mark.parametrize("name,argv", [
    ("setup", ["--station", "S", "--mock"]),
    ("daemon", ["--station", "S", "--robot", "R"]),
    ("supervise", ["--grace-s", "1", "--", "python", "--json"]),
    ("discover", ["--station", "S"]),
])
def test_native_registry_dispatch_preserves_implementation_arguments(monkeypatch, capsys, name, argv):
    from importlib import import_module
    from real_world_rl.cli.device_commands import COMMANDS as device_commands

    command = next(command for command in device_commands if command.name == name)
    assert next(item for item in COMMANDS if item.name == name) is command
    implementation = import_module(command.module)
    expected = vars(implementation.build_parser().parse_args(argv))
    calls = []

    def run(args, ctx):
        calls.append((args, ctx))
        print("native output")
        return 17

    monkeypatch.setattr(implementation, "run", run)
    assert main([name, *argv]) == 17
    assert len(calls) == 1
    args, ctx = calls[0]
    assert ctx.args is args
    assert ctx.remaining == []
    for key, value in expected.items():
        assert getattr(args, key) == value
    captured = capsys.readouterr()
    assert captured.out.splitlines() == ["native output"]
    assert captured.err == ""


@pytest.mark.parametrize("command,status,as_json,expected", [
    ("teleop", "PASS", False, SUCCESS_EXIT_CODE),
    ("teleop", "FAIL", False, STOP_TRACE_FAIL_EXIT_CODE),
    ("teleop", "BLOCKED", True, STOP_TRACE_BLOCKED_EXIT_CODE),
    ("workspace", True, False, SUCCESS_EXIT_CODE),
    ("workspace", False, False, WORKSPACE_INVALID_EXIT_CODE),
    ("workspace", False, True, SUCCESS_EXIT_CODE),
])
def test_commands_own_exit_codes(monkeypatch, capsys, tmp_path, command, status, as_json, expected):
    if command == "teleop":
        trace = tmp_path / "trace.json"
        trace.write_text("{}")
        monkeypatch.setattr("real_world_rl.devices.stop_trace.evaluate_stop_trace",
                            lambda *args: {"status": status})
        argv = ["teleop", "inspect-stop", "--trace", str(trace)]
        key = "status"
    else:
        from types import SimpleNamespace
        monkeypatch.setattr("real_world_rl.cli.cmd_workspace.station_document",
                            lambda args: (Path("station.yaml"), b"", {}, {},
                                          SimpleNamespace(config=SimpleNamespace(name="robot"))))
        monkeypatch.setattr("real_world_rl.cli.cmd_workspace.workspace_command",
                            lambda *args: {"valid": status})
        argv = ["workspace", "validate", "--pose", "0", "0", "0", "0", "0", "0", "1"]
        key = "valid"
    assert main((["--json"] if as_json else []) + argv) == expected
    assert json.loads(capsys.readouterr().out)[key] == status


def test_exit_code_table():
    assert (SUCCESS_EXIT_CODE, FAILURE_EXIT_CODE, USAGE_EXIT_CODE,
            DOCTOR_UNRESOLVED_EXIT_CODE, STOP_TRACE_FAIL_EXIT_CODE,
            STOP_TRACE_BLOCKED_EXIT_CODE, WORKSPACE_INVALID_EXIT_CODE) == (0, 1, 2, 1, 3, 4, 5)
    assert Result({}, "").exit_code == SUCCESS_EXIT_CODE
    assert CliError("failure").code == FAILURE_EXIT_CODE


def test_context_station_is_lazy_cached_and_connection_factory_is_injectable(monkeypatch):
    calls = []
    station = object()
    monkeypatch.setattr("real_world_rl.discovery.autoconfig.resolve_station_config_path", lambda value: value)
    monkeypatch.setattr("real_world_rl.station.load_station", lambda path: calls.append(path) or station)
    from real_world_rl.station import load_station
    from real_world_rl.cli import connect as connection_module
    ctx = Context(options(station="station.yaml"), station_loader=load_station,
                  connection_factory=lambda args: connection_module.open_robot(args))
    assert calls == []
    assert ctx.station() is station
    assert ctx.station() is station
    assert calls == ["station.yaml"]

    @contextmanager
    def connect(args):
        assert args is ctx.args
        yield "connection"

    monkeypatch.setattr("real_world_rl.cli.connect.open_robot", connect)
    with ctx.open_robot() as connection:
        assert connection == "connection"


def test_save_confirmation_uses_context_without_rebinding(monkeypatch, capsys, tmp_path):
    from real_world_rl.cli import station_edit
    original = station_edit.confirm_write
    calls = []
    station = tmp_path / "station.yaml"
    station.write_text("type: franka\nworkspace: null\n")

    def confirm(self, question, *, detail, saving=False):
        calls.append((question, detail, saving))
        raise CliError("cancelled", code=USAGE_EXIT_CODE)

    monkeypatch.setattr(Context, "confirm", confirm)
    assert main(["--json", "--station", str(station), "workspace", "set", "--frame", "base",
                 "--low", "0", "0", "0", "--high", "1", "1", "1", "--save"]) == USAGE_EXIT_CODE
    assert station_edit.confirm_write is original
    assert calls[0][0] == "Save? [y/N] "
    assert "-workspace: null" in calls[0][1] and "+workspace:" in calls[0][1]
    assert calls[0][2]
    assert json.loads(capsys.readouterr().out)["error"] == "cancelled"
    assert station.read_text() == "type: franka\nworkspace: null\n"


def test_device_commands_are_shared_and_rwrl_owns_the_doctor():
    from real_world_rl.cli.device_commands import COMMANDS as device_commands
    from real_world_rl.cli.cmd_doctor import DOCTOR
    native = {command.name: command for command in device_commands}
    composed = {command.name: command for command in COMMANDS}
    assert "doctor" not in native and composed["doctor"] is DOCTOR
    for name, command in native.items():
        assert composed[name] is command


def test_rwrl_context_injects_upper_layer_station_loader(monkeypatch):
    import real_world_rl.station
    station = object()
    calls = []
    monkeypatch.setattr(real_world_rl.station, "load_station", lambda path: calls.append(path) or station)
    monkeypatch.setattr("real_world_rl.discovery.autoconfig.resolve_station_config_path", lambda value: value)

    class StationCommand:
        name = "station-test"
        help = "inspect station context"

        def register(self, parser):
            pass

        def run(self, args, ctx):
            assert ctx.station() is station
            assert ctx.station() is station
            return Result({}, "loaded")

    monkeypatch.setattr("real_world_rl.cli.COMMANDS", [StationCommand()])
    assert main(["--station", "station.yaml", "station-test"]) == 0
    assert calls == ["station.yaml"]


def test_protocol_and_errors_are_defined_in_rwrl():
    from real_world_rl.cli import command, errors
    for name in ("Context", "Result", "Command", "confirm", "render"):
        assert getattr(command, name).__module__ == "real_world_rl.cli.command", name
    assert errors.CliError.__module__ == "real_world_rl.cli.errors"
    assert (errors.ABORTED_EXIT_CODE, errors.ESCALATED_EXIT_CODE,
            errors.LAUNCH_FAILURE_EXIT_CODE) == (130, 1, 127)
