"""Doctor text on a terminal: colour, the plain-text invariant, and the failure summary block."""
from __future__ import annotations

import io
import json
import re
import sys
from types import SimpleNamespace

import pytest

import real_world_rl.server.daemon as daemon
from real_world_rl.cli import doctor, main
from real_world_rl.cli.doctor_repair import Action
from real_world_rl.cli.render import strip_ansi
from real_world_rl.doctor import cli as device_doctor
from real_world_rl.doctor.checks import CheckResult


class Terminal(io.StringIO):
    def isatty(self):
        return True


@pytest.fixture
def colour_capable(monkeypatch):
    """A colour-capable TERM with NO_COLOR unset; stdout itself is replaced by ``attach_terminal``."""
    monkeypatch.delenv("NO_COLOR", raising=False)
    monkeypatch.setenv("TERM", "xterm-256color")


def attach_terminal(monkeypatch):
    """Make sys.stdout an interactive terminal. Call it in the test body: pytest's output capture
    reinstalls its own sys.stdout when the test call starts, replacing one set by a fixture."""
    stdout = Terminal()
    monkeypatch.setattr(sys, "stdout", stdout)
    return stdout


@pytest.fixture
def no_station(tmp_path, monkeypatch):
    """No station file anywhere the doctor looks, so --mock checks the host only."""
    monkeypatch.chdir(tmp_path)
    monkeypatch.setenv("XDG_CONFIG_HOME", str(tmp_path / "config"))
    monkeypatch.delenv("RWRL_STATION", raising=False)


def inject(monkeypatch, rows, actions=()):
    def inspect(args, host):
        for row in rows:
            host.add(*row)
        host.actions = [Action(*action) for action in actions]
        return {}

    monkeypatch.setattr(doctor, "inspect", inspect)


def run_plain(monkeypatch, argv):
    stdout = io.StringIO()
    monkeypatch.setattr(sys, "stdout", stdout)
    return main(argv), stdout.getvalue()


def without_report_directory(text):
    return re.sub(r"^Report: .*$", "Report: <dir>", text, flags=re.M)


FAILING_ROWS = [
    ("host.python", "PASS", "/venv/bin/python"),
    ("coverage.gello", "SKIPPED", "GELLO not configured", "run rwrl setup"),
    ("arm.preset:robot_ip", "WARN", 'station="a"; preset="b"', "regenerate the preset"),
    (
        "arm.device",
        "FAIL",
        "Traceback (most recent call last):\n  ...\nConnectionError: robot unreachable",
        "check the Ethernet cable\nand the robot IP in Desk",
    ),
    ("arm.os:claim", "BLOCKED", "device inspection requires an idle station", "finish the controlling program"),
    ("groups.dialout", "PENDING_ACTION", "log out and back in"),
]

FAILING_TEXT = """\
PASS           host.python: /venv/bin/python
SKIPPED        coverage.gello: GELLO not configured
    hint: run rwrl setup
WARN           arm.preset:robot_ip: station="a"; preset="b"
    hint: regenerate the preset
FAIL           arm.device: Traceback (most recent call last):
  ...
ConnectionError: robot unreachable
    hint: check the Ethernet cable
          and the robot IP in Desk
BLOCKED        arm.os:claim: device inspection requires an idle station
    hint: finish the controlling program
PENDING_ACTION groups.dialout: log out and back in
Report: <dir>
Needs attention: 1 FAIL, 1 BLOCKED, 1 PENDING_ACTION
  FAIL           arm.device: ConnectionError: robot unreachable
    hint: check the Ethernet cable
  BLOCKED        arm.os:claim: device inspection requires an idle station
    hint: finish the controlling program
  PENDING_ACTION groups.dialout: log out and back in
Result: unresolved failures / blocked checks / pending user actions
"""


def test_mock_doctor_on_a_terminal_is_coloured(doctor_report_dir, no_station, colour_capable, monkeypatch):
    terminal = attach_terminal(monkeypatch)
    assert main(["doctor", "--mock"]) == 0
    output = terminal.getvalue()
    assert "\x1b[" in output
    assert f"\x1b[90m{'SKIPPED':14}\x1b[0m station: no station configured" in output
    assert "    hint: run rwrl setup when ready" in strip_ansi(output)
    assert output.splitlines()[-1] == (
        "\x1b[1;32mResult: no blocking findings; review SKIPPED/WARN coverage\x1b[0m")
    assert "Needs attention" not in output


def test_no_color_keeps_the_terminal_plain(doctor_report_dir, no_station, colour_capable, monkeypatch):
    terminal = attach_terminal(monkeypatch)
    monkeypatch.setenv("NO_COLOR", "1")
    assert main(["doctor", "--mock"]) == 0
    assert "\x1b[" not in terminal.getvalue()


def test_piped_stdout_stays_plain(doctor_report_dir, no_station, colour_capable, monkeypatch):
    code, output = run_plain(monkeypatch, ["doctor", "--mock"])
    assert code == 0 and "Result: " in output and "\x1b[" not in output


@pytest.mark.parametrize("failing", [False, True])
def test_json_stdout_is_never_coloured(doctor_report_dir, no_station, colour_capable, monkeypatch, failing):
    terminal = attach_terminal(monkeypatch)
    if failing:
        inject(monkeypatch, FAILING_ROWS)
    assert main(["--json", "doctor", "--mock"]) == int(failing)
    output = terminal.getvalue()
    assert "\x1b" not in output
    assert json.loads(output)["status"] == ("FAIL" if failing else "PASS")


def test_coloured_text_strips_to_the_plain_text(doctor_report_dir, colour_capable, monkeypatch):
    terminal = attach_terminal(monkeypatch)
    inject(monkeypatch, FAILING_ROWS)
    assert main(["doctor"]) == 1
    coloured = terminal.getvalue()
    code, plain = run_plain(monkeypatch, ["doctor"])
    assert code == 1 and "\x1b[" not in plain
    assert without_report_directory(strip_ansi(coloured)) == without_report_directory(plain)
    for fragment in (
        f"\x1b[32m{'PASS':14}\x1b[0m host.python",
        f"\x1b[90m{'SKIPPED':14}\x1b[0m coverage.gello",
        f"\x1b[33m{'WARN':14}\x1b[0m arm.preset:robot_ip",
        f"\x1b[31m{'FAIL':14}\x1b[0m arm.device",
        f"\x1b[1;35m{'BLOCKED':14}\x1b[0m arm.os:claim",
        f"\x1b[33m{'PENDING_ACTION':14}\x1b[0m groups.dialout",
        "\x1b[1mReport:\x1b[0m ",
        "\x1b[1mNeeds attention:\x1b[0m 1 FAIL, 1 BLOCKED, 1 PENDING_ACTION",
        "\x1b[1;31mResult: unresolved failures / blocked checks / pending user actions\x1b[0m",
    ):
        assert fragment in coloured


def test_failure_path_ends_with_needs_attention(doctor_report_dir, monkeypatch):
    inject(monkeypatch, FAILING_ROWS)
    code, output = run_plain(monkeypatch, ["doctor"])
    assert code == 1
    assert without_report_directory(output) == FAILING_TEXT


def test_unconfirmed_repairs_are_listed_as_blocked(doctor_report_dir, monkeypatch):
    inject(monkeypatch, [("host.tools", "FAIL", "missing packages: usbutils")],
           [("apt:tools", "host.tools", "Install missing host diagnostic tools", ["true"])])
    monkeypatch.setattr(doctor.sys, "stdin", SimpleNamespace(isatty=lambda: False))
    code, output = run_plain(monkeypatch, ["doctor", "--fix"])
    assert code == 1
    assert output.splitlines()[-4:] == [
        "Needs attention: 1 FAIL, 1 BLOCKED",
        f"  {'FAIL':14} host.tools: missing packages: usbutils",
        f"  {'BLOCKED':14} apt:tools: plan not confirmed; no repairs executed",
        "Result: unresolved failures / blocked checks / pending user actions",
    ]


@pytest.mark.parametrize("statuses", [["PASS"], ["PASS", "WARN", "SKIPPED"]])
def test_success_path_has_no_needs_attention(doctor_report_dir, colour_capable, monkeypatch, statuses):
    terminal = attach_terminal(monkeypatch)
    inject(monkeypatch, [(f"sample.{status.lower()}", status, "ready") for status in statuses])
    assert main(["doctor"]) == 0
    output = terminal.getvalue()
    assert "Needs attention" not in output
    lines = output.splitlines()
    assert lines[0] == f"\x1b[32m{'PASS':14}\x1b[0m sample.pass: ready"
    assert strip_ansi(lines[-1]) == "Result: no blocking findings; review SKIPPED/WARN coverage"


DEVICE_RESULTS = [
    CheckResult(1, "sdk:arm", True, "franky importable"),
    CheckResult(2, "os:latency_timer", True, "16 ms", hint="set it to 1 ms", warning=True),
    CheckResult(3, "handshake", False, "robot unreachable", hint="check the cable"),
    CheckResult(4, "feedback:camera", True, "no camera configured", skipped=True),
]
DEVICE_TEXT = """\
[1/5] sdk:arm ... OK  franky importable
[2/5] os:latency_timer ... WARN  16 ms
    hint: set it to 1 ms
[3/5] handshake ... FAIL  robot unreachable
    hint: check the cable
[4/5] feedback:camera ... SKIP  no camera configured
summary: 2 ok (1 with warnings), 1 failed, 1 skipped"""


def test_device_doctor_text_is_plain_unless_colour_is_requested():
    results = {"arm": DEVICE_RESULTS}
    plain = device_doctor.format_station_text(results)
    assert plain == device_doctor.format_station_text(results, color=False) == DEVICE_TEXT
    coloured = device_doctor.format_station_text(results, color=True)
    assert strip_ansi(coloured) == plain
    for fragment in (
        "\x1b[1m[1/5]\x1b[0m sdk:arm ... \x1b[32mOK\x1b[0m  franky importable",
        "\x1b[33mWARN\x1b[0m  16 ms",
        "\x1b[31mFAIL\x1b[0m  robot unreachable",
        "\x1b[90mSKIP\x1b[0m  no camera configured",
        "\x1b[1;31msummary: 2 ok (1 with warnings), 1 failed, 1 skipped\x1b[0m",
    ):
        assert fragment in coloured


def test_device_doctor_station_text_colours_headers_and_verdicts():
    results = {"left": DEVICE_RESULTS, "right": [CheckResult(1, "sdk:arm", True, "franky importable")]}
    plain = device_doctor.format_station_text(results)
    assert "\x1b" not in plain
    coloured = device_doctor.format_station_text(results, color=True)
    assert strip_ansi(coloured) == plain
    assert "\x1b[1m== robot 'left' ==\x1b[0m" in coloured
    assert "\x1b[1;32msummary: 1 ok, 0 failed, 0 skipped\x1b[0m" in coloured
    assert coloured.splitlines()[-1] == "\x1b[1;31moverall: 2 robots, failures on ['left']\x1b[0m"


@pytest.mark.parametrize("no_color", [False, True])
def test_device_doctor_command_colours_only_a_terminal(colour_capable, monkeypatch, no_color):
    terminal = attach_terminal(monkeypatch)
    if no_color:
        monkeypatch.setenv("NO_COLOR", "1")
    monkeypatch.setattr(device_doctor, "resolve_station_config_path", lambda path: "station.yaml")
    monkeypatch.setattr(device_doctor, "doctor_station", lambda *args, **kwargs: {"arm": DEVICE_RESULTS})
    assert device_doctor.main([]) == 1
    output = terminal.getvalue()
    assert ("\x1b[" in output) is not no_color
    assert strip_ansi(output).rstrip("\n") == DEVICE_TEXT


def test_device_doctor_json_is_never_coloured(colour_capable, monkeypatch):
    terminal = attach_terminal(monkeypatch)
    monkeypatch.setattr(device_doctor, "resolve_station_config_path", lambda path: "station.yaml")
    monkeypatch.setattr(device_doctor, "doctor_station", lambda *args, **kwargs: {"arm": DEVICE_RESULTS})
    assert device_doctor.main(["--json"]) == 1
    output = terminal.getvalue()
    assert "\x1b" not in output and json.loads(output)["status"] == "FAIL"


def test_daemon_preflight_colours_the_stream_it_writes_to(colour_capable, monkeypatch):
    terminal = attach_terminal(monkeypatch)
    stderr = io.StringIO()
    monkeypatch.setattr(sys, "stderr", stderr)
    monkeypatch.setattr(daemon, "run_doctor", lambda config: [CheckResult(1, "sdk:arm", True, "fine")])
    assert daemon.preflight_passed(None)
    assert "\x1b[32mOK\x1b[0m  fine" in terminal.getvalue()
    # A failed preflight goes to stderr, here a pipe, so it stays plain although stdout is a terminal.
    monkeypatch.setattr(daemon, "run_doctor", lambda config: [CheckResult(1, "sdk:arm", False, "boom", "install it")])
    assert not daemon.preflight_passed(None)
    assert "sdk:arm ... FAIL  boom" in stderr.getvalue() and "\x1b" not in stderr.getvalue()
