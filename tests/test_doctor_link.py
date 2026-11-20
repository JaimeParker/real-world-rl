"""Doctor link checks use fake host commands and read-only client calls."""
from __future__ import annotations

import contextlib
import json
import shutil
from pathlib import Path
from types import SimpleNamespace

import pytest

from real_world_rl.cli import main
from real_world_rl.common.link_config import LinkConfig, link_overrides, validate_link
from real_world_rl.doctor import link, link_host, platform, repair
from real_world_rl.doctor.host import Host
from real_world_rl.doctor.platform import Capabilities
from real_world_rl.station import load_station


@pytest.fixture
def host(tmp_path, monkeypatch):
    etc = tmp_path / "etc"
    runtime = tmp_path / "run"
    (runtime / "systemd/system").mkdir(parents=True)
    monkeypatch.setenv("XDG_CONFIG_HOME", str(tmp_path / "config"))
    monkeypatch.setattr(repair, "project_environment", lambda: True)
    monkeypatch.setattr(repair.os, "geteuid", lambda: 0)
    # The systemd capability looks for systemctl on PATH; containers running these tests have none.
    which = platform.shutil.which
    monkeypatch.setattr(platform.shutil, "which",
                        lambda name, *a, **kw: f"/usr/bin/{name}" if name == "systemctl" else which(name, *a, **kw))
    report = repair.Report(directory=tmp_path / "reports")
    site = Host(None, report, etc=etc)
    site.capabilities = Capabilities("Linux", "ubuntu", "24", "x86_64", False, False,
                                     True, etc=etc, runtime=runtime)
    return site


def row(host, key):
    return next(item for item in host.findings if item.id == key)


def test_station_link_and_cli_override(tmp_path, monkeypatch, capsys):
    path = tmp_path / "station.yaml"
    path.write_text("""type: franka
name: follower
arm_backend: fake
gripper: null
link:
  address: 127.0.0.1:50061
  ssh: operator@nuc
  policy: localhost:8000
  token: original
  remote_bind: 127.0.0.1:50062
  control_hz: 10
""")
    config = load_station(path).select().sections["link"]
    flags = SimpleNamespace(link="localhost:50071", ssh="user@robot", policy="localhost:8001", token="override")
    assert link_overrides(flags, config) == LinkConfig(
        "localhost:50071", "user@robot", "127.0.0.1:50062", "localhost:8001", "override", 10,
    )
    monkeypatch.setattr(link, "inspect_client", lambda *args: pytest.fail("--mock probed the network"))
    args = ["doctor", "--station", str(path), "--mock", "--json", "--report-dir", str(tmp_path / "reports")]
    assert main([*args, "--link", "localhost:50071", "--ssh", "user@robot",
                 "--policy", "localhost:8001", "--token", "override"]) == 0
    output = capsys.readouterr().out
    assert "override" not in output and "original" not in output
    assert any(item["id"] == "link" and item["status"] == "SKIPPED" for item in json.loads(output)["checks"])
    assert main([*args, "--link", "localhost:70000"]) == 2


@pytest.mark.parametrize("fields", [
    {}, {"address": "localhost:0"}, {"address": "localhost:50061", "ssh": "-oProxyCommand=x"},
    {"address": "localhost:50061", "ssh": "user@"}, {"address": "localhost:50061", "ssh": "a b"},
    {"address": "localhost:50061", "remote_bind": "a b:30"},
    {"address": "localhost:50061", "token": 12}, {"address": "localhost:50061", "control_hz": float("inf")},
    {"address": "localhost:50061", "unknown": True},
])
def test_invalid_link_config(fields):
    with pytest.raises(ValueError):
        validate_link(fields)


@pytest.mark.parametrize("stderr,hint", [
    ("Host key verification failed.", "fingerprint"),
    ("Permission denied (publickey).", "ssh-add"),
    ("ssh: connect to host foo port 22: Connection timed out", "route"),
    ("ssh: Could not resolve hostname foo: Name or service not known", "DNS"),
    ("ssh: connect to host foo port 22: Connection refused", "sshd.service"),
])
def test_ssh_failure_hints(host, monkeypatch, stderr, hint):
    def command(argv, timeout):
        assert argv[-2:] == ["operator@nuc", "true"]
        assert "BatchMode=yes" in argv and "ConnectTimeout=10" in argv and timeout == 12
        return 255, stderr

    monkeypatch.setattr(host, "command", command)
    assert not link._ssh(host, "operator@nuc")
    result = row(host, "link.ssh")
    assert result.status == "FAIL"
    assert hint in result.hint


@pytest.mark.parametrize("elapsed,status", [(2, "PASS"), (2.01, "WARN")])
def test_ssh_speed(host, monkeypatch, elapsed, status):
    ticks = iter((0, elapsed))
    monkeypatch.setattr(link.time, "perf_counter", lambda: next(ticks))
    monkeypatch.setattr(host, "command", lambda *a, **kw: (0, ""))
    assert link._ssh(host, "operator@nuc")
    assert row(host, "link.ssh").status == status


@pytest.mark.parametrize("variables,resolved,status", [
    ({"HTTP_PROXY": "http://private:secret@proxy"}, "192.168.1.4", "WARN"),
    ({"ALL_PROXY": "socks5://proxy", "NO_PROXY": "127.0.0.1"}, "192.168.1.4", "PASS"),
    ({"grpc_proxy": "http://proxy", "no_proxy": "localhost", "no_grpc_proxy": "127.0.0.0/8"},
     "192.168.1.4", "PASS"),
    ({"HTTPS_PROXY": "http://proxy", "no_proxy": "127.0.0.1"}, "198.19.1.44", "WARN"),
])
def test_proxy_detection(host, monkeypatch, variables, resolved, status):
    monkeypatch.setattr(link.os, "environ", variables)
    monkeypatch.setattr(host, "command", lambda *a, **kw: (0, resolved))
    link._proxy(host, LinkConfig("127.0.0.1:50061", "operator@nuc"))
    result = row(host, "link.proxy")
    assert result.status == status
    assert "private" not in result.message and "secret" not in result.message
    if "HTTP_PROXY" in variables:
        assert 'export no_proxy="' in result.hint and "127.0.0.1,localhost" in result.hint
    if resolved.startswith("198."):
        assert "198.18.0.0/15" in result.message


@pytest.mark.parametrize("state,status,owner", [
    ("inactive", "FAIL", ""),
    ("failed", "FAIL", ""),
    ("active", "PASS", "ssh"),
    ("active", "WARN", "other"),
])
def test_tunnel_state_and_plan(host, monkeypatch, state, status, owner):
    def command(argv, timeout):
        if argv[0] == "systemctl":
            if "show" in argv:
                return 0, ""
            return (0 if state == "active" else 3), state
        assert argv[0] == "ss"
        return 0, f'LISTEN 0 128 127.0.0.1:50061 0.0.0.0:* users:(("{owner}",pid=98,fd=3))' if owner else ""

    monkeypatch.setattr(host, "command", command)
    monkeypatch.setattr(link_host.shutil, "which", lambda program: None if program == "autossh" else "/bin/" + program)
    config = LinkConfig("127.0.0.1:50061", "operator@nuc", "127.0.0.1:50052")
    link_host.inspect_tunnel(host, config, "left", ssh_ok=True)
    assert row(host, "link.tunnel").status == status
    if owner:
        assert not host.actions
        return
    apt, unit = host.actions
    assert apt.argv == ["apt-get", "install", "-y", "autossh"] and apt.privileged
    assert unit.dependencies == [apt.id] and not unit.privileged
    assert unit.target.endswith("systemd/user/rwrl-tunnel-left.service")
    assert "127.0.0.1:50061:127.0.0.1:50052 -- operator@nuc" in unit.content
    for text in ("-M 0 -N", "ServerAliveInterval=5", "ServerAliveCountMax=3", "ExitOnForwardFailure=yes",
                 "BatchMode=yes", "Restart=always", "RestartSec=5"):
        assert text in unit.content
    assert unit.after_argv[0] == ["systemctl", "--user", "daemon-reload"]
    assert unit.after_argv[-1][2] == ("restart" if state == "failed" else "enable")
    assert "loginctl enable-linger" in unit.advice


def test_tunnel_requires_ssh(host, monkeypatch):
    monkeypatch.setattr(host, "command", lambda *a, **kw: (0, ""))
    link_host.inspect_tunnel(host, LinkConfig("127.0.0.1:50061"), "left", ssh_ok=False)
    assert row(host, "link.tunnel").status == "BLOCKED"
    assert "--ssh" in row(host, "link.tunnel").hint
    assert not host.actions


@pytest.mark.parametrize("remote_error,status,hint", [
    ("", "PASS", ""),
    ("channel 2: open failed: administratively prohibited: open failed", "FAIL", "link.sshd"),
    ("channel 2: open failed: connect failed: Connection refused", "FAIL", "daemon is running"),
])
def test_temporary_forward_teardown(host, monkeypatch, remote_error, status, hint):
    closed = []

    @contextlib.contextmanager
    def command(argv, background):
        assert background and "ExitOnForwardFailure=yes" in argv and "-N" in argv
        assert argv[argv.index("-L") + 1].endswith(":127.0.0.1:50051")
        Path(argv[argv.index("-S") + 1]).touch()
        try:
            yield SimpleNamespace(poll=lambda: None), lambda: remote_error
        finally:
            closed.append("ssh")

    def call(method):
        assert method == "robot.describe"
        if remote_error:
            raise OSError("unreachable")

    def grpc(config, address, proxy):
        assert address.startswith("127.0.0.1:") and not proxy
        return contextlib.nullcontext(SimpleNamespace(open=lambda: None, call=call))

    reservation = SimpleNamespace(bind=lambda address: None, getsockname=lambda: ("127.0.0.1", 50123))
    monkeypatch.setattr(host, "command", command)
    monkeypatch.setattr(link.socket, "socket", lambda: contextlib.nullcontext(reservation))
    monkeypatch.setattr(link.socket, "create_connection", lambda *a, **kw: contextlib.nullcontext())
    monkeypatch.setattr(link, "_grpc", grpc)
    link._forward(host, LinkConfig("127.0.0.1:50061", "operator@nuc"), ssh_ok=True, grpc_ok=True)
    result = row(host, "link.forward")
    assert result.status == status and hint in result.hint
    assert closed == ["ssh"]


def _sshd_site(host, monkeypatch, *, main_text, config, before=None, validate=0, effective=True):
    main = host.etc / "ssh/sshd_config"
    target = main.parent / "sshd_config.d/99-rwrl.conf"
    target.parent.mkdir(parents=True)
    main.write_text(main_text)
    if before is not None:
        target.write_text(before)
    calls = []

    def command(argv, **kwargs):
        calls.append(argv)
        if argv[0] == "install":
            # Keep the test entirely in the fake etc tree.
            shutil.copyfile(argv[-2], argv[-1])
        if argv[0] == "mv":
            Path(argv[-2]).replace(argv[-1])
        if argv[0] == "rm":
            Path(argv[-1]).unlink(missing_ok=True)
        if "-T" in argv:
            if effective and target.exists() and target.read_text() == link_host._SSHD_CONTENT:
                return 0, "allowtcpforwarding local"
            return config
        if "-t" in argv:
            assert target.read_text() == link_host._SSHD_CONTENT
            return validate, "bad sshd config" if validate else ""
        return 0, ""

    monkeypatch.setattr(host.report, "run", command)
    link_host._sshd(host, "127.0.0.1:50051")
    return main, target, calls


_FORWARDING_OFF = (0, "allowtcpforwarding no\ndisableforwarding no\npermitopen any\n")


@pytest.mark.parametrize("status,config", [
    ("PASS", (0, "allowtcpforwarding local\ndisableforwarding no\npermitopen 127.0.0.1:50051")),
    ("BLOCKED", (0, "allowtcpforwarding yes\ndisableforwarding yes\npermitopen any")),
    ("BLOCKED", (0, "allowtcpforwarding yes\ndisableforwarding no\npermitopen none")),
    ("BLOCKED", (1, "sudo: a password is required")),
    ("BLOCKED", _FORWARDING_OFF),
])
def test_sshd_effective_settings_and_include(host, monkeypatch, status, config):
    monkeypatch.setattr(repair.os, "geteuid", lambda: 1234)
    _, _, calls = _sshd_site(host, monkeypatch, main_text="Match User someone\n  Include sshd_config.d/*.conf\n",
                             config=config)
    assert row(host, "link.sshd").status == status
    assert calls[-1][:2] == ["sudo", "-n"]
    assert not host.actions


@pytest.mark.parametrize("before,validate,edit", [(None, 0, False), (None, 1, False),
                                                 ("# old\n", 1, False), ("# old\n", 0, True)])
def test_sshd_drop_in_validation_rollback_and_concurrent_edit(host, monkeypatch, before, validate, edit):
    main, target, calls = _sshd_site(host, monkeypatch, main_text="Include sshd_config.d/*.conf\n",
                                     config=_FORWARDING_OFF, before=before, validate=validate)
    action, = host.actions
    assert row(host, "link.sshd").status == "FAIL"
    assert "MAJOR CHANGE / DOUBLE CHECK" in action.title
    if edit:
        main.write_text("# administrator edit\n")
    repair.execute([action], host.report, allowed=True, busy=lambda: [], assume_yes=True,
                   capabilities=host.capabilities, command=host.command)
    assert action.status == ("BLOCKED" if edit else "FAIL" if validate else "PASS")
    reloads = [i for i, argv in enumerate(calls) if argv[:2] == ["systemctl", "reload"]]
    if validate or edit:
        assert not reloads
        assert (target.read_text() if target.exists() else None) == before
    else:
        assert [argv[1] for argv in calls[reloads[0] - 2:reloads[0]]] == ["-t", "-T"]
        assert target.read_text() == link_host._SSHD_CONTENT
    if validate and before:
        assert (host.report.path / "99-rwrl.conf.before").read_text() == before


@pytest.mark.parametrize("address,token,process,allowed,expected", [
    ("127.0.0.1:50051", None, "rwrl", False, ("PASS", "PASS", "SKIPPED")),
    ("0.0.0.0:50051", None, "other", True, ("WARN", "FAIL", "PASS")),
    ("0.0.0.0:50051", "configured", "", False, ("PASS", "FAIL", "FAIL")),
])
def test_server_bind_listen_firewall(host, monkeypatch, address, token, process, allowed, expected):
    def command(argv, timeout):
        if argv[0] == "ss":
            return 0, f'LISTEN 0 4 127.0.0.1:50051 0.0.0.0:* users:(("{process}",pid=99,fd=3))' if process else ""
        if argv[0] == "ufw":
            return 0, "Status: active\nDefault: deny (incoming)\n" + ("50051/tcp ALLOW IN Anywhere\n" if allowed else "")
        if "-T" in argv:
            return 0, "allowtcpforwarding local\ndisableforwarding no\npermitopen any\n"
        return 0, ""

    monkeypatch.setattr(host, "command", command)
    monkeypatch.setattr(link_host.shutil, "which", lambda program: program)
    monkeypatch.setattr(link_host, "read_text", lambda path: "\0/bin/rwrl\0daemon\0" if process == "rwrl" else "")
    link_host.inspect_server(host, address, token)
    assert tuple(row(host, "link." + key).status for key in ("daemon.bind", "daemon.listen", "firewall")) == expected


def test_dependency_and_preprocess_repair(host, monkeypatch, tmp_path):
    host.root = tmp_path
    monkeypatch.setattr(host, "command", lambda argv, **kw: (1, "") if argv[-1] != "import msgpack" else (0, ""))
    missing = link._dependencies(host)
    assert missing == ["grpc", "openpi-client"]
    link._report_dependencies(host, missing, jpeg_missing=False)
    link.inspect_preprocess(host)
    assert row(host, "link.deps").status == "FAIL"
    assert row(host, "link.preprocess").status == "WARN"
    assert {a.id for a in host.actions} == {"pip:extra:jpeg", "pip:extra:grpc", "pip:extra:openpi-client"}
    assert all(a.python for a in host.actions)


def test_ssh_alias_and_remote_bind_flag():
    assert validate_link({"address": "localhost:50061", "ssh": "nuc"}).ssh == "nuc"
    flags = SimpleNamespace(link="localhost:50061", ssh="nuc", policy=None, token=None, remote_bind="127.0.0.1:50071")
    assert link_overrides(flags, None) == LinkConfig("localhost:50061", "nuc", "127.0.0.1:50071")


@pytest.mark.parametrize("connected", [False, True])
def test_quality_connects_only_an_idle_daemon_robot(host, monkeypatch, connected):
    calls = []

    def call(method, **kwargs):
        calls.append((method, client.timeout_s))
        if method == "robot.health":
            return {"connected": connected}

    client = SimpleNamespace(call=call, last_response_bytes=1000, timeout_s=2.0)
    link._observation_rate(client, {"cameras": {"wrist": {}}})
    methods = [method for method, _ in calls]
    assert ("robot.connect" in methods, "robot.disconnect" in methods) == (not connected, not connected)
    assert all(timeout == link._DEVICE_CALL_TIMEOUT_S for method, timeout in calls if method != "robot.health")
    assert client.timeout_s == 2.0


def test_quality_keeps_the_measurement_when_disconnect_fails(host):
    def call(method, **kwargs):
        if method == "robot.health":
            return {"connected": False}
        if method == "robot.disconnect":
            raise RuntimeError("deadline")

    text, per_observation = link._observation_rate(
        SimpleNamespace(call=call, last_response_bytes=1000, timeout_s=2.0), {"cameras": {"wrist": {}}})
    assert per_observation is not None and "robot.disconnect after sampling failed: deadline" in text


@pytest.mark.parametrize("rate,expected", [(10, "longer than the 10 Hz control period"),
                                           (None, "steps cannot run faster than 2.0 Hz")])
def test_quality_flags_slow_observations(host, monkeypatch, rate, expected):
    ticks = iter([0, 0.01] * 10 + [0, 1.5])
    monkeypatch.setattr(link.time, "perf_counter", lambda: next(ticks))
    client = SimpleNamespace(call=lambda method, **kw: {"connected": True} if method == "robot.health" else None,
                             last_response_bytes=100_000)
    link._quality(host, client, LinkConfig("localhost:50061", control_hz=rate),
                  {"watchdog_s": 5, "cameras": {"wrist": {}}})
    result = row(host, "link.quality")
    assert result.status == "WARN" and expected in result.hint and "0.500s per observation" in result.message


@pytest.mark.parametrize("watchdog,rate,status", [(0.5, 10, "WARN"), (5, 10, "WARN"),
                                                 (None, None, "PASS"), (5, 2, "PASS")])
def test_quality_watchdog_and_control_period(host, monkeypatch, watchdog, rate, status):
    ticks = iter([0, 0.2] * 10 + [0, 0.3])
    monkeypatch.setattr(link.time, "perf_counter", lambda: next(ticks))
    observations = []

    def call(method, **kwargs):
        if method == "robot.get_observation":
            observations.append(kwargs)
        elif method == "robot.health":
            return {"connected": True}
        else:
            assert method == "robot.describe"

    client = SimpleNamespace(call=call, last_response_bytes=300_000)
    describe = {"watchdog_s": watchdog, "frame_preprocess_cv2_v1": True, "cameras": {"wrist": {}}}
    link._quality(host, client, LinkConfig("localhost:50061", control_hz=rate), describe)
    result = row(host, "link.quality")
    assert result.status == status
    assert "p50 200.0 ms, p95 200.0 ms" in result.message
    assert "post-reset gap 0.600s" in result.message
    assert "900,000 response bytes" in result.message and "3.00 MB/s" in result.message
    assert ("--watchdog-s" in result.hint) == (watchdog == 0.5)
    assert (f"{rate} Hz control period" in result.hint) == (rate == 10)
    assert len(observations) == 3
    assert observations[0]["frame_preprocess"]["wrist"] == {
        "config": {"resize": "cv2_linear"}, "size": [128, 128], "color": True, "depth": False,
    }


@pytest.mark.parametrize("error,status", [("", "PASS"), ("token mismatch", "FAIL")])
def test_daemon_describe_and_authentication(host, monkeypatch, error, status):
    def call(method):
        assert method == "robot.describe"
        if error:
            raise RuntimeError(error)
        return {"daemon_instance_id": "identifier", "frame_preprocess_cv2_v1": True, "image_encoding": {"jpeg": 90}}

    monkeypatch.setattr(link, "_grpc", lambda *a: contextlib.nullcontext(SimpleNamespace(open=lambda: None, call=call)))
    monkeypatch.setattr(link, "_quality", lambda *a, **kw: None)
    link._daemon(host, LinkConfig("localhost:50061", token="private"), grpc_ok=True)
    result = row(host, "link.daemon")
    assert result.status == status
    assert ("UNAUTHENTICATED" if error else "daemon_instance_id present") in result.message


@pytest.mark.parametrize("working", [True, False])
def test_policy_only_reads_metadata(host, monkeypatch, working):
    @contextlib.contextmanager
    def client(address, port, timeout_s):
        assert (address, port, timeout_s) == ("localhost", 8000, 4)
        if not working:
            raise OSError("unreachable")
        yield SimpleNamespace(metadata={"optional": True})

    monkeypatch.setattr("real_world_rl.common.openpi_client.OpenPiClient", client)
    link._policy(host, "localhost:8000", policy_ok=True)
    assert row(host, "link.policy").status == ("PASS" if working else "FAIL")


def test_sshd_rolls_back_a_drop_in_shadowed_by_earlier_settings(host, monkeypatch):
    _, target, calls = _sshd_site(host, monkeypatch, main_text="Include sshd_config.d/*.conf\n",
                                  config=_FORWARDING_OFF, effective=False)
    action, = host.actions
    repair.execute(host.actions, host.report, allowed=True, busy=lambda: [], assume_yes=True,
                   capabilities=host.capabilities, command=host.command)
    assert action.status == "FAIL" and "Include order" in action.detail, (action.status, action.detail)
    assert not target.exists()
    assert all(argv[:2] != ["systemctl", "reload"] for argv in calls)


@pytest.mark.parametrize("yes", [False, True])
def test_tunnel_cli_confirmation_and_reinspection(tmp_path, monkeypatch, capsys, yes):
    from real_world_rl.doctor import report as doctor_report

    target_dir = tmp_path / "config/systemd/user"
    target_dir.mkdir(parents=True)
    target = target_dir / "rwrl-tunnel-50061.service"
    target.write_text("# previous unit\n")
    monkeypatch.setenv("XDG_CONFIG_HOME", str(tmp_path / "config"))
    monkeypatch.setattr(doctor_report.sys, "stdin", SimpleNamespace(isatty=lambda: False))
    monkeypatch.setattr(link_host.shutil, "which", lambda name: "/usr/bin/" + name)
    commands = []
    inspections = []

    def fake_run(report, argv, **kwargs):
        commands.append(argv)
        if argv[0] == "ss":
            if target.read_text().startswith("[Unit]"):
                return 0, 'LISTEN 0 128 127.0.0.1:50061 0.0.0.0:* users:(("ssh",pid=98,fd=3))'
            return 0, ""
        if argv[0] == "systemctl":
            if "is-active" in argv:
                return 3, "inactive"
            if "show" in argv:
                return 0, str(target)
        if argv[0] == "install":
            shutil.copyfile(argv[-2], argv[-1])
        if argv[0] == "mv":
            Path(argv[-2]).replace(argv[-1])
        return 0, ""

    def inspect(args, host):
        inspections.append(args.link)
        host.allowed = True
        host.capabilities = Capabilities("Linux", "ubuntu", "24", "x86_64", False, False, True)
        doctor_report.inspect_links(args, host, [])
        return {}

    def inspect_client(host, config, name):
        link_host.inspect_tunnel(host, config, name, ssh_ok=True)

    monkeypatch.setattr(repair.Report, "run", fake_run)
    monkeypatch.setattr(doctor_report, "inspect", inspect)
    monkeypatch.setattr(link, "inspect_client", inspect_client)
    argv = ["doctor", "--link", "127.0.0.1:50061", "--ssh", "operator@nuc", "--fix",
            "--json", "--report-dir", str(tmp_path / "reports")]
    assert main(argv + (["--yes"] if yes else [])) == (0 if yes else 1)
    captured = capsys.readouterr()
    result = json.loads(captured.out)
    assert result["confirmed"] is yes and len(inspections) == (2 if yes else 1)
    assert "--now" in captured.err and "Apply this plan?" not in captured.err
    assert result["repairs"][0]["status"] == ("PASS" if yes else "BLOCKED")
    if yes:
        assert target.read_text().startswith("[Unit]")
        assert (Path(result["report_directory"]) / (target.name + ".before")).read_text() == "# previous unit\n"
        assert any(item["id"] == "link.tunnel" and item["status"] == "PASS" for item in result["checks"])
        assert ["systemctl", "--user", "enable", "--now", target.name] in commands
    else:
        assert target.read_text() == "# previous unit\n"
        assert not any("enable" in argv or argv[0] == "install" for argv in commands)



def test_advertised_jpeg_requires_a_decoder(host, monkeypatch, tmp_path):
    host.root = tmp_path

    def missing_decoder():
        raise ImportError("missing")

    monkeypatch.setattr("real_world_rl.server.codec.require_jpeg_decoder", missing_decoder)
    assert link._missing_jpeg_decoder({"image_encoding": {"jpeg": 90}})
    assert not link._missing_jpeg_decoder({})
    link._report_dependencies(host, [], jpeg_missing=True)
    assert row(host, "link.deps").status == "FAIL"
    assert "JPEG" in row(host, "link.deps").message
    action, = host.actions
    assert action.id == "pip:extra:jpeg" and action.check == "link.deps"


def test_a_dual_stack_listener_matches_its_ipv4_bind(host, monkeypatch):
    def command(argv, timeout):
        if argv[0] == "ss":
            return 0, 'LISTEN 0 4096 [::ffff:127.0.0.1]:50071 *:* users:(("python",pid=99,fd=7))'
        return 0, ""

    monkeypatch.setattr(host, "command", command)
    monkeypatch.setattr(link_host, "read_text", lambda path: "python\0-m\0real_world_rl.cli\0daemon\0")
    link_host._daemon_listen(host, "127.0.0.1:50071")
    assert row(host, "link.daemon.listen").status == "PASS"


@pytest.mark.parametrize("sshd_running", [True, False])
def test_sshd_without_systemd_is_blocked_not_failed(host, monkeypatch, sshd_running):
    def command(argv, timeout):
        if argv[0] == "systemctl":
            return 127, "systemctl: command not found"
        if argv[0] == "pgrep":
            return (0, "42") if sshd_running else (1, "")
        return pytest.fail(f"unexpected {argv}")

    monkeypatch.setattr(host, "command", command)
    link_host._sshd(host, "127.0.0.1:50051")
    result = row(host, "link.sshd")
    assert result.status == "BLOCKED" and "outside containers" in result.hint
    assert ("an sshd process is running" in result.message) == sshd_running
