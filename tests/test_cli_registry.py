"""Registry and orchestration contracts; no device calls or subprocesses."""
import argparse
from contextlib import contextmanager
import io
import json
from pathlib import Path
from types import SimpleNamespace
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
    assert names == ["status", "q", "tcp", "stop", "recover", "home", "gripper",
                     "doctor", "setup", "daemon", "supervise", "teleop", "workspace"]
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


def test_forwarded_json_is_not_redirected_or_rendered_twice(monkeypatch, capsys):
    def child(argv):
        assert argv == ["--help", "--unknown"]
        print('{"child": true}')
        return 17

    monkeypatch.setattr("real_world_rl.cli.cmd_forward.import_module",
                        lambda name: SimpleNamespace(main=child))
    assert main(["--json", "setup", "--help", "--unknown"]) == 17
    captured = capsys.readouterr()
    assert captured.out.splitlines() == ['{"child": true}']
    assert captured.err == ""


@pytest.mark.parametrize("name,expected", [
    ("setup", ["--from", "S", "--mock"]),
    ("daemon", ["--config", "S", "--robot", "R"]),
    ("supervise", []),
])
def test_declarative_forward_option_mapping(monkeypatch, name, expected):
    calls = []
    monkeypatch.setattr("real_world_rl.cli.cmd_forward.import_module", lambda module: SimpleNamespace(
        main=lambda argv: calls.append(argv)))
    assert main(["--station", "S", "--robot", "R", "--preset", "P", "--mock", "--json", "--yes",
                 name, "--", "literal"]) == SUCCESS_EXIT_CODE
    assert calls == [[*expected, "--", "literal"]]


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
        monkeypatch.setattr("robot_infra.devices.stop_trace.evaluate_stop_trace",
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
    monkeypatch.setattr("robot_infra.discovery.autoconfig.resolve_station_config_path", lambda value: value)
    monkeypatch.setattr("real_world_rl.station.load_station", lambda path: calls.append(path) or station)
    ctx = Context(options(station="station.yaml"))
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
