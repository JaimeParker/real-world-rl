"""Doctor tests: repair boundaries, real metadata, and hardware-free CLI orchestration."""

from __future__ import annotations

import dataclasses
import json
import os
import shutil
import subprocess
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

from real_world_rl.cli import doctor, main
from real_world_rl.cli.doctor_host import Host, process_owners
from real_world_rl.cli.doctor_platform import Capabilities
from real_world_rl.cli.doctor_repair import (
    Action,
    Finding,
    Report,
    execute,
    exit_status,
    file_action,
)


@pytest.fixture
def report(tmp_path, monkeypatch):
    monkeypatch.setenv("RWRL_DOCTOR_REPORT_DIR", str(tmp_path / "reports"))
    return Report(tmp_path)


@pytest.fixture
def host(tmp_path, report):
    return Host(
        tmp_path,
        report,
        sysfs=tmp_path / "sys",
        dev=tmp_path / "dev",
        etc=tmp_path / "etc",
    )


@pytest.fixture
def native_linux_platform(tmp_path, monkeypatch):
    """A repair-capable Debian host, independent of the test runner's platform."""
    capabilities = Capabilities(
        system="Linux", distribution="debian", version="12", architecture="x86_64",
        container=False, wsl=False, debian_family=True,
        etc=tmp_path / "etc", sysfs=tmp_path / "sys", runtime=tmp_path / "run",
    )
    for directory in (capabilities.etc / "udev/rules.d", capabilities.etc / "security/limits.d"):
        directory.mkdir(parents=True)
    monkeypatch.setattr(Capabilities, "detect", classmethod(lambda cls, **kwargs: capabilities))
    original_which = shutil.which
    monkeypatch.setattr(
        "real_world_rl.cli.doctor_platform.shutil.which",
        lambda tool: f"/usr/bin/{tool}" if tool in {"apt-get", "udevadm"} else original_which(tool),
    )
    return capabilities


def test_report_is_private_and_redacts_nested_credentials(report):
    report.save(
        "report.json",
        {
            "password": "do not keep",
            "child": {"token": "hidden"},
            "url": "https://name:secret@example.org/simple",
            "text": "password=hidden",
        },
    )
    path = report.path / "report.json"
    assert path.stat().st_mode & 0o777 == 0o600
    assert report.path.stat().st_mode & 0o777 == 0o700
    data = json.loads(path.read_text())
    assert data["child"]["token"] == "<redacted>"
    assert "name:secret" not in data["url"]
    assert "hidden" not in data["text"]


def test_command_stderr_is_logged_without_corrupting_json(report):
    code, output = report.run(
        [
            sys.executable,
            "-c",
            "import sys; print('{}'); print('warning',file=sys.stderr)",
        ],
        stdout_only=True,
    )
    assert code == 0 and json.loads(output) == {}
    assert "warning" in report.log.read_text()


def test_doctor_never_executes_a_shell(report, monkeypatch):
    calls = []
    monkeypatch.setattr(
        subprocess,
        "run",
        lambda argv, **kw: (
            calls.append((argv, kw))
            or SimpleNamespace(returncode=0, stdout="", stderr="")
        ),
    )
    report.run(["printf", "literal;$(touch /tmp/unwanted)"])
    assert calls[0][0][1] == "literal;$(touch /tmp/unwanted)"
    assert "shell" not in calls[0][1]


@pytest.mark.parametrize(
    "status,code",
    [
        ("PASS", 0),
        ("WARN", 0),
        ("SKIPPED", 0),
        ("FAIL", 1),
        ("BLOCKED", 1),
        ("PENDING_ACTION", 1),
    ],
)
def test_exit_codes(status, code):
    assert exit_status([Finding("item", status, "evidence")], []) == code


def test_repairs_continue_independent_items_but_block_dependents(report, monkeypatch, native_linux_platform):
    actions = [
        Action("one", "one", "fail", ["one"]),
        Action("two", "two", "dependent", ["two"], dependencies=["one"]),
        Action("three", "three", "independent", ["three"]),
    ]
    calls = []
    monkeypatch.setattr(
        report,
        "run",
        lambda argv, **kw: calls.append(argv) or (int(argv == ["one"]), "result"),
    )
    execute(actions, report, allowed=True, busy=lambda: [], assume_yes=True)
    assert [a.status for a in actions] == ["FAIL", "BLOCKED", "PASS"]
    assert calls == [["one"], ["three"]]


@pytest.mark.parametrize(
    "allowed,busy,reason",
    [(False, [], "disabled"), (True, ["pid 12: controller"], "running")],
)
def test_unsupported_host_or_busy_device_blocks_mutation(
    report, monkeypatch, native_linux_platform, allowed, busy, reason
):
    monkeypatch.setattr(report, "run", lambda *a, **kw: pytest.fail("must not execute"))
    action = Action("repair", "sdk", "change", ["anything"], exclusive=True)
    execute([action], report, allowed=allowed, busy=lambda: busy, assume_yes=True)
    assert action.status == "BLOCKED" and reason in action.detail


@pytest.mark.parametrize("container,wsl", [(True, False), (False, True)])
def test_container_or_wsl_blocks_system_repair(report, monkeypatch, native_linux_platform, container, wsl):
    capabilities = dataclasses.replace(native_linux_platform, container=container, wsl=wsl)
    monkeypatch.setattr(report, "run", lambda *args, **kwargs: pytest.fail("must not repair this host"))
    action = Action("apt:tools", "tools", "install", ["apt-get", "install", "-y", "acl"],
                    privileged=True, capability="packages")
    execute([action], report, allowed=True, busy=lambda: [], assume_yes=True,
            capabilities=capabilities)
    assert action.status == "BLOCKED"
    assert "native Linux host outside containers and WSL" in action.detail


def test_system_python_repair_refused(report, monkeypatch):
    from real_world_rl.cli import doctor_repair

    monkeypatch.setattr(doctor_repair, "project_environment", lambda: False)
    monkeypatch.setattr(report, "run", lambda *a, **kw: pytest.fail("must not run pip"))
    action = Action(
        "pip:test",
        "sdk",
        "install",
        [sys.executable, "-m", "pip", "install", "test"],
        python=True,
    )
    execute([action], report, allowed=True, busy=lambda: [], assume_yes=True)
    assert action.status == "BLOCKED" and "sudo pip" in action.detail


def test_current_interpreter_is_required_for_pip(report, monkeypatch):
    monkeypatch.setattr(
        report, "run", lambda *a, **kw: pytest.fail("must not run other interpreter")
    )
    action = Action(
        "pip:test",
        "sdk",
        "install",
        ["/other/python", "-m", "pip", "install", "test"],
        python=True,
    )
    execute([action], report, allowed=True, busy=lambda: [], assume_yes=True)
    assert action.status == "BLOCKED"


def test_sudo_missing_is_blocked_without_password_prompt(report, monkeypatch, native_linux_platform):
    monkeypatch.setattr("real_world_rl.cli.doctor_repair.os.geteuid", lambda: 1000)
    monkeypatch.setattr(report, "run", lambda argv, **kw: (1, "sudo password required"))
    action = Action(
        "apt:tools",
        "tools",
        "install",
        ["apt-get", "install", "-y", "acl"],
        privileged=True,
    )
    execute([action], report, allowed=True, busy=lambda: [], assume_yes=True)
    assert action.status == "BLOCKED" and "sudo -v" in action.detail


def test_file_destination_allowlist(report, monkeypatch, tmp_path, native_linux_platform):
    monkeypatch.setattr(report, "run", lambda argv, **kw: (0, ""))
    target = tmp_path / "unowned.conf"
    action = file_action("file", "change unrelated file", str(target), "bad")
    execute([action], report, allowed=True, busy=lambda: [], assume_yes=True)
    assert action.status == "BLOCKED" and not target.exists()


def test_plan_detects_intervening_file_edits(report, monkeypatch, tmp_path, native_linux_platform):
    monkeypatch.setattr(report, "run", lambda argv, **kw: (0, ""))
    target = tmp_path / "concurrent"
    action = file_action("file", "change", str(target), "new")
    target.write_text("someone else's edit")
    execute([action], report, allowed=True, busy=lambda: [], assume_yes=True)
    assert action.status == "BLOCKED" and "changed" in action.detail
    assert target.read_text() == "someone else's edit"


def test_no_control_process_is_killed(tmp_path):
    proc = tmp_path / "proc"
    task = proc / "123"
    task.mkdir(parents=True)
    (task / "cmdline").write_bytes(
        b"python\0examples/record_real_demos.py\0--token\0private\0"
    )
    found = process_owners(proc=proc)
    assert found == ["pid 123: record_real_demos.py"]
    assert "private" not in str(found)


def test_spacemouse_rules_cover_libusb_and_hidraw_and_all_supported_products(host):
    from robot_infra.devices.backends.teleops._spacemouse_hid import device_specs

    host.rules("usb.spacemouse", "spacemouse", [])
    action = next(a for a in host.actions if a.target)
    assert (
        'SUBSYSTEM=="usb"' in action.content and 'SUBSYSTEM=="hidraw"' in action.content
    )
    for spec in device_specs.values():
        assert f'idProduct}}=="{spec.hid_id[1]:04x}"' in action.content
    assert action.pending and "replug" in action.pending
    assert all("trigger" not in a.argv for a in host.actions)


def test_managed_rules_are_idempotent(host):
    host.rules("usb.spacemouse", "spacemouse", [])
    action = next(a for a in host.actions if a.target)
    path = Path(action.target)
    path.parent.mkdir(parents=True)
    path.write_text(action.content)
    host.actions = []
    host.rules("usb.spacemouse", "spacemouse", [])
    assert not host.actions


def test_usb_discovery_is_independent_of_installed_sdks(tmp_path, report):
    sysfs = tmp_path / "sys"
    usb = sysfs / "bus/usb/devices/1-1"
    usb.mkdir(parents=True)
    for name, value in {
        "idVendor": "256f",
        "idProduct": "c635",
        "product": "SpaceMouse Compact",
        "busnum": "1",
        "devnum": "5",
    }.items():
        (usb / name).write_text(value)
    host = Host(
        tmp_path, report, sysfs=sysfs, dev=tmp_path / "dev", etc=tmp_path / "etc"
    )
    assert host.usb[0]["kind"] == "spacemouse"
    assert host.usb[0]["node"].endswith("bus/usb/001/005")


def test_unrelated_intel_usb_is_not_realsense(tmp_path, report):
    usb = tmp_path / "sys/bus/usb/devices/1-1"
    usb.mkdir(parents=True)
    for name, value in {
        "idVendor": "8086",
        "idProduct": "1234",
        "product": "Bluetooth",
    }.items():
        (usb / name).write_text(value)
    host = Host(tmp_path, report, sysfs=tmp_path / "sys")
    assert host.usb[0]["kind"] == ""


def test_unknown_brltty_log_does_not_disable_service(host, monkeypatch, tmp_path):
    port = tmp_path / "ttyUSB0"
    config = SimpleNamespace(
        teleop=SimpleNamespace(backend="gello", options={"port": str(port)})
    )
    monkeypatch.setattr(
        host, "command", lambda *a, **kw: (0, "brltty service started but unrelated")
    )
    host.inspect_gello([config])
    assert not any("systemctl" in a.argv for a in host.actions)


def test_kernel_is_manual_and_no_kernel_actions_are_generated(host, monkeypatch):
    host.expected.add("franky")
    host.inspect_realtime()
    assert any("DOUBLE CHECK" in f.hint for f in host.findings)
    assert all(
        not any(word in str(a.argv) for word in ["linux-image", "reboot", "grub"])
        for a in host.actions
    )


def test_configured_but_ineffective_limits_are_pending(host):
    target = host.etc / "security/limits.d/99-rwrl-realtime.conf"
    target.parent.mkdir(parents=True)
    target.write_text(f"{host.user} - rtprio 99\n{host.user} - memlock unlimited\n")
    host.expected.add("franky")
    host.inspect_realtime()
    if any(f.id == "realtime.limits" and f.status == "PASS" for f in host.findings):
        pytest.skip("test process already has full realtime permissions")
    assert any(
        f.id == "realtime.limits" and f.status == "PENDING_ACTION"
        for f in host.findings
    )
    assert not host.actions


def test_report_is_git_ignored(tmp_path):
    git = shutil.which("git")
    if git is None:
        pytest.skip("git is unavailable")
    root = Path(__file__).resolve().parents[1]
    repository = tmp_path / "checkout"
    repository.mkdir()
    (repository / ".gitignore").write_bytes((root / ".gitignore").read_bytes())
    environment = {key: value for key, value in os.environ.items() if not key.startswith("GIT_")}
    subprocess.run([git, "init"], cwd=repository, env=environment, check=True,
                   capture_output=True, text=True)
    result = subprocess.run(
        [git, "check-ignore", "local/doctor/example/report.json"],
        cwd=repository,
        env=environment,
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0


def test_declining_or_non_tty_does_not_execute_repairs(report, monkeypatch, capsys):
    action = Action(
        "pip:test",
        "deps.test",
        "install test",
        [sys.executable, "-m", "pip", "install", "test"],
        python=True,
    )

    def fake_inspect(args, host):
        host.allowed = True
        host.findings = [Finding("deps.test", "FAIL", "missing")]
        host.actions = [dataclasses.replace(action)]
        return {}

    monkeypatch.setattr(doctor, "inspect", fake_inspect)
    monkeypatch.setattr(doctor.sys, "stdin", SimpleNamespace(isatty=lambda: False))
    monkeypatch.setattr(
        doctor, "execute", lambda *a, **kw: pytest.fail("no confirmation")
    )
    assert main(["doctor", "--fix", "--json"]) == 1
    result = json.loads(capsys.readouterr().out)
    assert result["confirmed"] is False and result["repairs"][0]["status"] == "BLOCKED"


def test_yes_runs_repairs_and_rechecks(report, monkeypatch, capsys):
    calls = []

    def fake_inspect(args, host):
        host.allowed = True
        host.findings = [Finding("test", "PASS" if calls else "FAIL", "evidence")]
        host.actions = [] if calls else [Action("test", "test", "repair", ["true"])]
        return {}

    monkeypatch.setattr(doctor, "inspect", fake_inspect)
    monkeypatch.setattr(doctor, "execute", lambda *a, **kw: calls.append("repair"))
    assert main(["doctor", "--fix", "--yes", "--json"]) == 0
    result = json.loads(capsys.readouterr().out)
    assert result["confirmed"] is True and result["checks"][0]["status"] == "PASS"
    assert (Path(result["report_directory"]) / "before.json").exists()


def test_probe_never_exposes_fix_or_motion_entrypoints():
    path = Path(doctor.__file__).with_name("doctor_probe.py")
    text = path.read_text()
    assert "fix=False" in text and "fix=True" not in text
    for forbidden in [
        ".recover(",
        ".reset(",
        "move_to_",
        ".homing(",
        "command_binary(",
    ]:
        assert forbidden not in text


def test_missing_preset_is_usage_error_with_a_report(report, capsys):
    assert main(["doctor", "--preset", "/nonexistent/preset.yaml"]) == 2
    assert any(report.path.parent.glob("*/error.json"))


def test_extra_markers_are_resolved_before_pip_invocation(host, monkeypatch):
    from real_world_rl.cli import doctor_host

    host.expected.add("spacemouse")
    monkeypatch.setattr(
        doctor_host.metadata,
        "requires",
        lambda name: (
            ["hidapi==0.14.0.post4; extra == 'spacemouse'"]
            if name == "rlgarden-robot-infra"
            else []
        ),
    )

    def missing(name):
        raise doctor_host.metadata.PackageNotFoundError(name)

    monkeypatch.setattr(doctor_host.metadata, "version", missing)
    monkeypatch.setattr(host, "command", lambda *a, **kw: (0, "OK"))
    host.inspect_dependencies()
    action = next(a for a in host.actions if a.id == "pip:hidapi")
    assert action.argv[-1] == "hidapi==0.14.0.post4" and ";" not in action.argv[-1]


def test_franky_conflicting_server_versions_never_generate_install(host, monkeypatch):
    from robot_infra.devices import RobotConfig

    configs = [
        (
            name,
            RobotConfig.from_dict(
                {"type": "franka", "arm_backend": "fake", "name": name, "gripper": None}
            ),
            "unused",
            False,
            "",
        )
        for name in ("left", "right")
    ]
    monkeypatch.setattr(doctor, "load_targets", lambda args: (configs, set()))
    monkeypatch.setattr(host, "inspect_system", lambda **kw: None)
    rows = iter([9, 10])

    def run(argv, **kwargs):
        return 0, json.dumps(
            [
                {
                    "name": "handshake",
                    "ok": False,
                    "message": "mismatch",
                    "details": {"libfranka_server_version": next(rows)},
                }
            ]
        )

    monkeypatch.setattr(host.report, "run", run)
    doctor.inspect(SimpleNamespace(mock=True), host)
    assert any(
        f.id == "sdk.franky.conflict" and f.status == "BLOCKED" for f in host.findings
    )
    assert not any(a.id.startswith("sdk:") for a in host.actions)


@pytest.mark.parametrize(
    "hint, expected",
    [
        (
            "clear errors in Desk, then run with --fix (recover)",
            "clear errors in Desk, then run rwrl recover (or robot-infra doctor --fix)",
        ),
        (
            "try another port; run with --fix to reopen",
            "try another port; rerun rwrl status after reconnecting the camera (or robot-infra doctor --fix)",
        ),
        ("check the cable", "check the cable"),
        ("", ""),
    ],
)
def test_device_fix_hints_name_rwrl_commands(hint, expected):
    assert doctor.rwrl_device_hint(hint) == expected


def test_device_rows_report_rwrl_hints(host, monkeypatch):
    from robot_infra.devices import RobotConfig

    config = RobotConfig.from_dict(
        {"type": "franka", "arm_backend": "fake", "name": "left", "gripper": None}
    )
    monkeypatch.setattr(doctor, "load_targets", lambda args: ([("left", config, "unused", False, "")], set()))
    monkeypatch.setattr(host, "inspect_system", lambda **kw: None)
    rows = [{"name": "arm", "ok": False, "message": "reflex", "hint": "then run with --fix (recover)"}]
    monkeypatch.setattr(host.report, "run", lambda argv, **kwargs: (0, json.dumps(rows)))
    doctor.inspect(SimpleNamespace(mock=True), host)
    finding = next(f for f in host.findings if f.id == "left.arm")
    assert "rwrl recover" in finding.hint and "--fix (recover)" not in finding.hint


def test_serial_identity_is_stable_and_targets_only_one_adapter(host):
    usb = host.sysfs / "bus/usb/devices/1-2"
    port = usb / "1-2:1.0/ttyUSB0"
    port.mkdir(parents=True)
    tty = host.sysfs / "class/tty/ttyUSB0"
    tty.mkdir(parents=True)
    (tty / "device").symlink_to(port)
    host.usb = [dict(sysfs=str(usb), vid="0403", pid="6001", serial="FT123")]
    identity, selector = host.serial_identity(Path("/dev/ttyUSB0"))
    assert identity["serial"] == "FT123"
    assert 'ATTRS{serial}=="FT123"' in selector
    assert "ttyUSB0" not in selector


def test_serial_identity_rejects_rule_injection_and_uses_usb_topology(host):
    usb = host.sysfs / "bus/usb/devices/1-2"
    port = usb / "1-2:1.0/ttyUSB0"
    port.mkdir(parents=True)
    tty = host.sysfs / "class/tty/ttyUSB0"
    tty.mkdir(parents=True)
    (tty / "device").symlink_to(port)
    host.usb = [
        dict(sysfs=str(usb), vid="0403", pid="6001", serial='bad", RUN+="danger')
    ]
    _, selector = host.serial_identity(Path("/dev/ttyUSB0"))
    assert 'KERNELS=="1-2"' in selector and "RUN" not in selector


def test_unknown_serial_is_candidate_not_proven_gello(host):
    host.dev.mkdir(parents=True)
    (host.dev / "ttyUSB0").touch()
    host.inspect_gello([])
    assert any(
        f.id == "gello.candidates" and f.status == "SKIPPED" for f in host.findings
    )
    assert not host.actions and "gello" not in host.expected


def test_network_inspection_cannot_change_addresses(host, monkeypatch):
    calls = []
    monkeypatch.setattr(
        host,
        "command",
        lambda argv, **kw: (
            calls.append(argv) or (0, '[{"dev":"eno1","prefsrc":"192.0.2.2"}]')
        ),
    )
    host.inspect_network([SimpleNamespace(robot_ip="192.0.2.1")])
    assert calls == [["ip", "-j", "route", "get", "192.0.2.1"]]
    assert not host.actions


def test_probe_preserves_franka_fields_and_forces_read_only_claim(monkeypatch, capsys):
    import io

    from robot_infra.doctor import checks

    from real_world_rl.cli import doctor_probe

    captured = []

    def run(config, **kwargs):
        captured.append((config, kwargs))
        print("SDK chatter")
        return []

    monkeypatch.setattr(checks, "run_doctor", run)
    request = {
        "config": {
            "type": "franka",
            "name": "test",
            "arm_backend": "fake",
            "gripper": None,
            "claim": False,
            "home_joints": [0.0] * 7,
        }
    }
    monkeypatch.setattr(sys, "stdin", io.StringIO(json.dumps(request)))
    assert doctor_probe.main() == 0
    output = capsys.readouterr()
    assert json.loads(output.out) == [] and "SDK chatter" in output.err
    config, kwargs = captured[0]
    assert config.claim and config.home_joints == [0.0] * 7
    assert kwargs["fix"] is False


def test_brltty_repair_requires_correlated_usb_topology(host, monkeypatch):
    host.usb = [
        dict(vid="1a86", pid="7523", sysfs=str(host.sysfs / "bus/usb/devices/1-2"))
    ]
    config = SimpleNamespace(
        teleop=SimpleNamespace(backend="gello", options={"port": "/dev/ttyUSB9"})
    )
    logs = "usb 1-2: ch341 converter now attached to ttyUSB9\nusb 1-3: interface 0 claimed while 'brltty' sets config #1"
    monkeypatch.setattr(host, "command", lambda *a, **kw: (0, logs))
    host.inspect_gello([config])
    assert not any("systemctl" in a.argv for a in host.actions)
    host.actions = []
    logs = logs.replace("usb 1-3:", "usb 1-2:")
    host.inspect_gello([config])
    assert any("systemctl" in a.argv for a in host.actions)


def test_missing_libusb_has_system_repair_not_only_pip(host, monkeypatch):
    host.expected.add("spacemouse")
    monkeypatch.setattr(
        host,
        "command",
        lambda argv, **kw: (
            (1, "shared library not found") if "'usb-1.0'" in argv[-1] else (0, "OK")
        ),
    )
    host.inspect_dependencies()
    action = next(a for a in host.actions if a.id == "apt:libusb-1.0-0")
    assert action.privileged and action.exclusive
    assert action.argv == ["apt-get", "install", "--reinstall", "-y", "libusb-1.0-0"]
    assert not action.python


def test_registered_doctor_json_is_single_line(report, monkeypatch, capsys):
    def inspect(args, host):
        print("SDK diagnostic chatter")
        host.add("sample", "PASS", "设备可读")
        return {}

    monkeypatch.setattr(doctor, "inspect", inspect)
    assert main(["--json", "doctor"]) == 0
    captured = capsys.readouterr()
    assert len(captured.out.splitlines()) == 1
    result = json.loads(captured.out)
    assert result["schema_version"] == 1
    assert result["checks"][0]["message"] == "设备可读"
    assert "设备可读" in captured.out  # Preserve doctor's ensure_ascii=False output.
    assert "SDK diagnostic chatter" in captured.err


def test_global_station_and_legacy_config_are_equivalent(report, tmp_path, monkeypatch, capsys):
    path = tmp_path / "station.yaml"
    path.write_text("type: franka\nname: sample\narm_backend: fake\ngripper: null\n")
    targets = []

    def inspect(args, host):
        loaded, expected = doctor.load_targets(args)
        targets.append(([(name, config.to_dict(), source, daemon, bind)
                         for name, config, source, daemon, bind in loaded], expected))
        host.add("sample", "PASS", "configuration loaded")
        return {}

    monkeypatch.setattr(doctor, "inspect", inspect)
    assert main(["--station", str(path), "--json", "doctor"]) == 0
    first = json.loads(capsys.readouterr().out)
    assert main(["doctor", "--config", str(path), "--json"]) == 0
    second = json.loads(capsys.readouterr().out)
    assert targets[0] == targets[1]
    assert first["checks"] == second["checks"]
    assert first["results"] == second["results"]


def test_doctor_compatibility_entry_delegates(monkeypatch):
    calls = []
    monkeypatch.setattr("real_world_rl.cli.main", lambda argv: calls.append(argv) or 1)
    assert doctor.main(["--config", "station.yaml", "--json"]) == 1
    assert calls == [["doctor", "--config", "station.yaml", "--json"]]


def test_doctor_parser_owns_help_and_preserves_trailing_globals(capsys):
    from real_world_rl.cli import build_parser
    from real_world_rl.cli.cmd_doctor import DOCTOR

    parser = build_parser()
    args = parser.parse_args(["--station", "first", "--robot", "left", "--mock", "--json",
                              "doctor", "--config", "second", "--robot", "right", "--fix"])
    assert args._command is DOCTOR
    assert (args.station, args.robot, args.mock, args.json, args.fix) == ("second", "right", True, True, True)
    assert not getattr(DOCTOR, "accepts_remaining", False)
    from robot_infra.cli import DeviceCommand
    assert not isinstance(DOCTOR, DeviceCommand)
    script = """
import importlib.abc
import sys
class NoDeviceDoctor(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if fullname == 'robot_infra.doctor' or fullname.startswith('robot_infra.doctor.'):
            raise AssertionError('device doctor import: ' + fullname)
sys.meta_path.insert(0, NoDeviceDoctor())
from real_world_rl.cli import main
assert main(['doctor', '--help']) == 0
assert 'robot_infra.doctor.cli' not in sys.modules
"""
    result = subprocess.run([sys.executable, "-c", script], cwd=Path(__file__).resolve().parents[1],
                            env={**os.environ, "PYTHONDONTWRITEBYTECODE": "1"},
                            capture_output=True, text=True, check=False)
    assert result.returncode == 0, result.stderr
    assert main(["doctor", "--help"]) == 0
    output = capsys.readouterr().out
    assert "usage: rwrl doctor" in output
    assert "--fix" in output and "--report-dir" in output and "--install-sdk" in output


def test_doctor_unknown_option_preserves_versioned_error_json(capsys):
    assert main(["--json", "doctor", "--unknown"]) == 2
    captured = capsys.readouterr()
    assert len(captured.out.splitlines()) == 1
    assert json.loads(captured.out) == {
        "schema_version": 1, "error": "unrecognized arguments: --unknown", "report_directory": None,
    }
    assert "usage: rwrl doctor" in captured.err
    assert "rwrl doctor: error: unrecognized arguments: --unknown" in captured.err


@pytest.mark.parametrize("status,expected", [
    ("PASS", 0), ("WARN", 0), ("SKIPPED", 0),
    ("FAIL", 1), ("BLOCKED", 1), ("PENDING_ACTION", 1),
])
def test_registered_doctor_returns_status_code(report, monkeypatch, capsys, status, expected):
    def inspect(args, host):
        host.add("sample", status, "status evidence")
        return {}

    monkeypatch.setattr(doctor, "inspect", inspect)
    assert main(["--json", "doctor"]) == expected
    assert json.loads(capsys.readouterr().out)["checks"][0]["status"] == status


def test_registered_doctor_text_summary_is_unchanged(report, monkeypatch, capsys):
    def inspect(args, host):
        host.add("sample", "PASS", "ready")
        return {}

    monkeypatch.setattr(doctor, "inspect", inspect)
    assert main(["doctor"]) == 0
    captured = capsys.readouterr()
    lines = captured.out.splitlines()
    assert lines[0] == f"{'PASS':14} sample: ready"
    assert lines[1].startswith("Report: ")
    assert Path(lines[1].removeprefix("Report: "), "report.json").is_file()
    assert lines[2:] == ["Result: no blocking findings; review SKIPPED/WARN coverage"]
    assert captured.err == ""
