"""Release boundaries: platform capabilities, installed layouts and private reports."""

from __future__ import annotations

import dataclasses
import json
import subprocess
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

from real_world_rl.cli import doctor, doctor_host, doctor_repair
from real_world_rl.cli.doctor_host import Host
from real_world_rl.cli.doctor_platform import Capabilities
from real_world_rl.cli.doctor_repair import Action, Report, execute, redact


@pytest.fixture
def caps(tmp_path, monkeypatch):
    monkeypatch.setattr(
        "real_world_rl.cli.doctor_platform.shutil.which",
        lambda tool: "/usr/bin/" + tool,
    )
    etc, runtime = tmp_path / "etc", tmp_path / "run"
    for path in (
        etc / "udev/rules.d",
        etc / "security/limits.d",
        runtime / "systemd/system",
    ):
        path.mkdir(parents=True)
    return Capabilities(
        "Linux",
        "ubuntu",
        "24.04",
        "aarch64",
        False,
        False,
        True,
        etc,
        tmp_path / "sys",
        runtime,
    )


@pytest.fixture
def report(tmp_path, monkeypatch):
    monkeypatch.setenv("RWRL_DOCTOR_REPORT_DIR", str(tmp_path / "reports"))
    return Report()


@pytest.mark.parametrize(
    "distribution,version,architecture,family",
    [
        ("ubuntu", "20.04", "x86_64", ""),
        ("ubuntu", "24.04", "aarch64", ""),
        ("debian", "12", "aarch64", ""),
        ("linuxmint", "22", "x86_64", "ubuntu debian"),
        ("fedora", "41", "aarch64", "fedora"),
    ],
)
def test_release_and_architecture_do_not_gate_independent_repairs(
    caps, tmp_path, monkeypatch, distribution, version, architecture, family
):
    (caps.etc / "os-release").write_text(
        f'ID={distribution}\nVERSION_ID="{version}"\nID_LIKE="{family}"\n'
    )
    monkeypatch.delenv("container", raising=False)
    monkeypatch.setattr("platform.system", lambda: "Linux")
    monkeypatch.setattr("platform.release", lambda: "generic-kernel")
    monkeypatch.setattr("platform.machine", lambda: architecture)
    actual = Capabilities.detect(
        etc=caps.etc,
        sysfs=caps.sysfs,
        proc=tmp_path / "proc",
        runtime=caps.runtime,
        filesystem=tmp_path,
    )
    assert actual.device_probes and actual.architecture == architecture
    for capability in ("udev", "groups", "pam", "systemd", "python"):
        assert not actual.block_reason(capability)
    assert bool(actual.block_reason("packages")) == (distribution == "fedora")


@pytest.mark.parametrize(
    "change",
    [
        dict(container=True),
        dict(wsl=True),
        dict(system="Darwin"),
        dict(system="Windows"),
    ],
)
def test_restricted_hosts_block_system_but_allow_project_python(
    caps, report, monkeypatch, change
):
    caps = dataclasses.replace(caps, **change)
    monkeypatch.setattr(doctor_repair, "project_environment", lambda: True)
    calls = []
    monkeypatch.setattr(
        report, "run", lambda argv, **kwargs: calls.append(argv) or (0, "OK")
    )
    actions = [
        Action(
            "apt",
            "tools",
            "system repair",
            ["apt-get", "install", "acl"],
            privileged=True,
            capability="packages",
        ),
        Action(
            "pip",
            "deps",
            "Python repair",
            [sys.executable, "-m", "pip", "install", "example"],
            python=True,
        ),
    ]
    execute(
        actions,
        report,
        allowed=True,
        busy=lambda: [],
        assume_yes=True,
        capabilities=caps,
    )
    assert [action.status for action in actions] == ["BLOCKED", "PASS"]
    assert calls == [actions[1].argv]


@pytest.mark.parametrize("marker", [".dockerenv", "run/.containerenv"])
def test_container_markers_are_detected(caps, tmp_path, monkeypatch, marker):
    monkeypatch.delenv("container", raising=False)
    path = tmp_path / marker
    path.parent.mkdir(parents=True, exist_ok=True)
    path.touch()
    actual = Capabilities.detect(
        etc=caps.etc, proc=tmp_path / "proc", filesystem=tmp_path
    )
    assert actual.container and not actual.device_probes


def test_wsl_kernel_is_detected(caps, tmp_path, monkeypatch):
    monkeypatch.delenv("container", raising=False)
    monkeypatch.setattr("platform.release", lambda: "6.6-microsoft-standard-WSL2")
    actual = Capabilities.detect(
        etc=caps.etc, proc=tmp_path / "proc", filesystem=tmp_path
    )
    assert actual.wsl and actual.block_reason("udev")


def test_missing_apt_does_not_block_other_linux_repairs(caps, monkeypatch):
    monkeypatch.setattr(
        "real_world_rl.cli.doctor_platform.shutil.which",
        lambda tool: None if tool == "apt-get" else "/usr/bin/" + tool,
    )
    assert caps.block_reason("packages") and not caps.block_reason("udev")


def test_non_linux_diagnosis_returns_json_without_host_commands(
    caps, report, monkeypatch, capsys
):
    monkeypatch.setattr(
        Capabilities,
        "detect",
        lambda **kwargs: dataclasses.replace(caps, system="Windows"),
    )
    monkeypatch.setattr(doctor, "find_checkout", lambda: None)
    monkeypatch.setattr(doctor, "load_targets", lambda args: ([], set()))
    monkeypatch.setattr(
        Host, "command", lambda *args, **kwargs: pytest.fail("no native host commands")
    )
    assert doctor.main(["--json"]) == 0
    result = json.loads(capsys.readouterr().out)
    assert any(
        row["id"] == "host.platform" and row["status"] == "WARN"
        for row in result["checks"]
    )


def test_help_does_not_require_posix_modules(tmp_path):
    script = """
import builtins
original = builtins.__import__
def guarded(name, *args, **kwargs):
    if name in {'pwd', 'grp', 'resource'}:
        raise ImportError('not available on this platform')
    return original(name, *args, **kwargs)
builtins.__import__ = guarded
from real_world_rl.cli.doctor import main
main(['--help'])
"""
    result = subprocess.run(
        [sys.executable, "-c", script], capture_output=True, text=True
    )
    assert result.returncode == 0, result.stderr
    assert "--report-dir" in result.stdout


def test_installed_layout_uses_user_state_not_site_packages(tmp_path, monkeypatch):
    monkeypatch.delenv("RWRL_DOCTOR_REPORT_DIR", raising=False)
    monkeypatch.setenv("XDG_STATE_HOME", str(tmp_path / "state"))
    monkeypatch.setattr(doctor_repair.sys, "platform", "linux")
    report = Report()
    assert report.path.parent == tmp_path / "state/rwrl/doctor"


def test_read_only_checkout_falls_back_to_user_state(tmp_path, monkeypatch):
    monkeypatch.delenv("RWRL_DOCTOR_REPORT_DIR", raising=False)
    monkeypatch.setattr(doctor_repair, "state_directory", lambda: tmp_path / "state")
    root = tmp_path / "checkout"
    original = Path.mkdir

    def mkdir(path, *args, **kwargs):
        if path == root / "local/doctor":
            raise PermissionError("read-only checkout")
        return original(path, *args, **kwargs)

    monkeypatch.setattr(Path, "mkdir", mkdir)
    report = Report(root)
    assert report.path.parent == tmp_path / "state"


def test_explicit_report_directory_overrides_environment(tmp_path, monkeypatch):
    monkeypatch.setenv("RWRL_DOCTOR_REPORT_DIR", str(tmp_path / "environment"))
    report = Report(directory=tmp_path / "explicit")
    assert report.path.parent == tmp_path / "explicit"
    assert not (tmp_path / "environment").exists()


def test_unwritable_explicit_directory_returns_json_error(report, tmp_path, capsys):
    destination = tmp_path / "file-not-directory"
    destination.touch()
    with pytest.raises(SystemExit) as error:
        doctor.main(["--json", "--report-dir", str(destination)])
    assert error.value.code == 2
    result = json.loads(capsys.readouterr().out)
    assert "cannot create private doctor report" in result["error"]
    assert result["report_directory"] is None


def test_configuration_error_returns_one_json_object(report, capsys):
    with pytest.raises(SystemExit) as error:
        doctor.main(["--json", "--preset", "/does/not/exist.yaml"])
    assert error.value.code == 2
    result = json.loads(capsys.readouterr().out)
    assert result["schema_version"] == 1 and result["error"]
    assert (Path(result["report_directory"]) / "error.json").is_file()


def test_missing_sibling_distribution_never_uses_an_invented_source(
    report, monkeypatch
):
    host = Host(None, report)
    monkeypatch.setattr(
        doctor_host.metadata,
        "requires",
        lambda name: ["rlgarden-robot-infra==0.0.1"] if name == "real-world-rl" else [],
    )

    def missing(name):
        raise doctor_host.metadata.PackageNotFoundError(name)

    monkeypatch.setattr(doctor_host.metadata, "version", missing)
    host.inspect_dependencies()
    assert not host.actions
    assert any(
        row.id.endswith(".source") and row.status == "BLOCKED" for row in host.findings
    )


def test_verified_sibling_source_can_be_reinstalled(report, tmp_path, monkeypatch):
    source = tmp_path / "robot_infra"
    source.mkdir()
    (source / "pyproject.toml").write_text('[project]\nname="rlgarden-robot-infra"\n')
    host = Host(tmp_path, report)
    monkeypatch.setattr(
        doctor_host.metadata,
        "requires",
        lambda name: ["rlgarden-robot-infra==0.0.1"] if name == "real-world-rl" else [],
    )

    def missing(name):
        raise doctor_host.metadata.PackageNotFoundError(name)

    monkeypatch.setattr(doctor_host.metadata, "version", missing)
    host.inspect_dependencies()
    assert host.actions[0].argv[-2:] == ["-e", str(source)]


def test_configured_backend_is_delegated_without_a_second_allowlist(
    caps, report, monkeypatch
):
    config = SimpleNamespace(
        arm_backend="installed-plugin",
        gripper=None,
        cameras=[],
        teleop=None,
        robot_ip=None,
        to_dict=lambda: {},
    )
    host = Host(None, report)
    host.capabilities = caps
    monkeypatch.setattr(
        doctor,
        "load_targets",
        lambda args: ([("plugin", config, "unused", False, "")], set()),
    )
    for method in (
        "inspect_system",
        "inspect_gello",
        "inspect_usb",
        "inspect_network",
        "inspect_realtime",
        "inspect_dependencies",
    ):
        monkeypatch.setattr(host, method, lambda *args, **kwargs: None)
    monkeypatch.setattr(host, "busy", lambda: [])
    row = {"name": "handshake", "ok": True, "message": "plugin read"}
    monkeypatch.setattr(report, "run", lambda *args, **kwargs: (0, json.dumps([row])))
    results = doctor.inspect(SimpleNamespace(mock=False), host)
    assert results["plugin"] == [row]


def test_redaction_covers_authorization_headers_and_argument_values():
    assert "top-secret" not in redact("Authorization: Bearer top-secret")
    assert redact(["tool", "--api-key", "top-secret"])[-1] == "<redacted>"
    assert redact({"authorization": "top-secret"})["authorization"] == "<redacted>"


def test_standalone_usb_ownership_is_blocked_rather_than_silently_omitted(
    caps, report, monkeypatch
):
    host = Host(None, report)
    host.capabilities = caps
    monkeypatch.setattr(doctor, "load_targets", lambda args: ([], {"spacemouse"}))
    for method in (
        "inspect_system",
        "inspect_gello",
        "inspect_usb",
        "inspect_network",
        "inspect_realtime",
        "inspect_dependencies",
    ):
        monkeypatch.setattr(host, method, lambda *args, **kwargs: None)
    monkeypatch.setattr(host, "busy", lambda: ["pid 123: open target device"])
    monkeypatch.setattr(
        report,
        "run",
        lambda *args, **kwargs: pytest.fail("no SDK probe while occupied"),
    )
    doctor.inspect(SimpleNamespace(mock=False), host)
    assert any(
        row.id == "read.spacemouse" and row.status == "BLOCKED" for row in host.findings
    )


@pytest.mark.parametrize(
    "base_record,active_base,expected",
    [(False, False, True), (True, False, False), (False, True, False)],
)
def test_conda_prefix_environments_do_not_require_envs_directory(
    tmp_path, monkeypatch, base_record, active_base, expected
):
    prefix = tmp_path / "custom-project-environment"
    metadata = prefix / "conda-meta"
    metadata.mkdir(parents=True)
    if base_record:
        (metadata / "conda-26.0.0-0.json").touch()
    monkeypatch.setattr(sys, "prefix", str(prefix))
    monkeypatch.setattr(sys, "base_prefix", str(prefix))
    monkeypatch.delenv("CONDA_EXE", raising=False)
    monkeypatch.setenv("CONDA_DEFAULT_ENV", "base" if active_base else str(prefix))
    assert doctor_repair.project_environment() is expected
