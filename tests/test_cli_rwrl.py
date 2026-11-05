"""Hardware-free rwrl coverage; daemon calls stay inside LocalClient."""
from __future__ import annotations

import builtins
import json
import os
from pathlib import Path
import socket
import subprocess
import sys

import numpy as np
import pytest
import yaml

from real_world_rl.cli import main
from real_world_rl.cli import connect

HOME = [0.1, 0.0, 0.0, -1.5, 0.0, 1.5, 0.7]


@pytest.fixture(autouse=True)
def offline(monkeypatch):
    def forbidden(*args, **kwargs):
        raise AssertionError("CLI tests must not open sockets")

    monkeypatch.setattr(socket, "create_connection", forbidden)
    monkeypatch.setattr(sys.stdin, "isatty", lambda: False)


@pytest.fixture
def station(tmp_path):
    path = tmp_path / "station.yaml"
    path.write_text(yaml.safe_dump({
        "type": "franka", "name": "cli_fake", "arm_backend": "fake", "claim": False,
        "gripper": {"backend": "fake"}, "home_joints": HOME,
    }), encoding="utf-8")
    return path


@pytest.fixture
def built_robots(monkeypatch):
    from robot_infra.devices import Robot

    original = Robot.from_config
    robots = []

    def create(cls, config):
        robot = original(config)
        robots.append(robot)
        return robot

    monkeypatch.setattr(Robot, "from_config", classmethod(create))
    return robots


def invoke(station, capsys, command, *, options=(), code=0):
    actual = main(["--station", str(station), "--mock", "--json", *options, *command])
    captured = capsys.readouterr()
    assert actual == code, captured.err
    payload = json.loads(captured.out)
    assert isinstance(payload, dict)
    if code:
        assert set(payload) == {"error", "hint"}
        assert "error: " in captured.err
    else:
        assert payload["command"] == " ".join(command[:2] if command[0] in ("home", "gripper") else command)
        assert payload["path"].startswith(("direct", "daemon"))
    return payload, captured


@pytest.mark.parametrize("command", [
    ["status"], ["q"], ["tcp"], ["home", "show"], ["home", "go"],
    ["gripper", "open"], ["gripper", "close"], ["gripper", "show"],
    ["gripper", "move", "0.3"], ["stop"], ["recover"],
])
def test_direct_json_commands(station, capsys, built_robots, command):
    moves = command == ["home", "go"] or (command[0] == "gripper" and command[1] != "show")
    payload, _ = invoke(station, capsys, command, options=["--yes"] if moves else [])
    robot = built_robots[-1]
    assert payload["path"] == "direct fake"
    assert not robot.is_connected
    if command == ["home", "show"]:
        assert payload["source"] == "station home_joints"
        assert payload["home_joints"] == HOME
        assert payload["delta"] == HOME
    elif command == ["home", "go"]:
        assert payload["max_delta"] < 1e-6
        np.testing.assert_allclose(robot.arm.read_state()["joint_pos"], HOME)
        assert "move_to_joints" in robot.arm.call_log
    elif command == ["q"]:
        assert payload["joint_pos"] == [0.0] * 7
    elif command == ["tcp"]:
        assert payload["tcp_pose"] == [0.0] * 6 + [1.0]
    elif command[0] == "gripper":
        expected = {"open": 1.0, "close": 0.0, "show": 1.0, "move": 0.3}[command[1]]
        assert payload["gripper"]["width"] == expected
    elif command[0] in ("stop", "recover"):
        assert command[0] in robot.arm.call_log
    else:
        assert payload["arm_health"]["ok"]
        assert payload["description"]["name"] == "cli_fake"


def test_text_status_and_state(station, capsys):
    for command, expected in [
        (["status"], "gripper width 0.0800 m (1.00 of max_width_m 0.080), grasped=unknown, source FakeGripper"),
        (["q"], "q (rad): [0.0000,"),
        (["tcp"], "quat (xyzw): [0.0000, 0.0000, 0.0000, 1.0000]"),
        (["home", "show"], "max delta: 1.500000 rad"),
    ]:
        assert main(["--station", str(station), "--mock", *command]) == 0
        assert expected in capsys.readouterr().out


def test_mock_replaces_sdk_backends(station, capsys, built_robots):
    data = yaml.safe_load(station.read_text())
    data["arm_backend"] = "franky"
    data["gripper"]["backend"] = "franky"
    station.write_text(yaml.safe_dump(data))
    payload, _ = invoke(station, capsys, ["status"])
    assert payload["path"] == "direct fake"
    assert type(built_robots[-1].arm).__name__ == "FakeArm"
    assert type(built_robots[-1].gripper).__name__ == "FakeGripper"


def test_float32_device_state(station, capsys, monkeypatch):
    from robot_infra.devices.backends.arms.fake import FakeArm

    original = FakeArm.read_state

    def state(self):
        return {name: values.astype(np.float32) for name, values in original(self).items()}

    monkeypatch.setattr(FakeArm, "read_state", state)
    for command, field in [(["q"], "joint_pos"), (["tcp"], "tcp_pose")]:
        payload, _ = invoke(station, capsys, command)
        assert len(payload[field]) == 7


@pytest.fixture
def daemon(station, monkeypatch):
    from robot_infra.devices import Robot
    from robot_infra.discovery.autoconfig import load_station
    from robot_infra.server import remote
    from robot_infra.server.service import RobotService
    from robot_infra.server.transport.local import LocalClient

    config = load_station(station).select().config
    service = RobotService(Robot.from_config(config), claim=False, watchdog_s=30.0)
    real_remote = remote.RemoteRobot
    data = yaml.safe_load(station.read_text())
    data["daemon"] = {"bind": "127.0.0.1:0", "token": "test-token"}
    station.write_text(yaml.safe_dump(data))
    created = []

    def factory(config):
        assert config.address == "127.0.0.1:0"
        assert config.token == "test-token"
        robot = real_remote.from_client(LocalClient(service))
        created.append(robot)
        return robot

    monkeypatch.setattr(connect, "probe_daemon", lambda address: True)
    monkeypatch.setattr(remote, "RemoteRobot", factory)
    yield service, created
    service.close()


def test_daemon_status_q_and_home(station, daemon, capsys):
    for command in (["status"], ["q"], ["home", "show"], ["home", "go"]):
        payload, _ = invoke(station, capsys, command, options=["--yes"])
        assert payload["path"] == "daemon 127.0.0.1:0"
        if command == ["status"]:
            assert payload["rtt_ms"] >= 0
        if command == ["home", "show"]:
            assert "daemon arm.home_joints" in payload["source"]
            assert payload["home_joints"] == HOME
        if command == ["home", "go"]:
            assert payload["max_delta"] < 1e-6
    assert all(not robot.is_connected and not robot.client.is_open for robot in daemon[1])


def test_daemon_home_comes_from_daemon_arm(station, daemon, capsys):
    daemon[0].robot.config.home_joints = [0.6] * 7
    payload, _ = invoke(station, capsys, ["home", "show"])
    assert payload["home_joints"] == [0.6] * 7
    assert "daemon arm.home_joints" in payload["source"]


def test_daemon_unreachable(station, daemon, capsys, monkeypatch):
    monkeypatch.setattr(connect, "probe_daemon", lambda address: False)
    payload, _ = invoke(station, capsys, ["q"], code=1)
    assert "127.0.0.1:0" in payload["error"]
    assert "start it (rwrl daemon) or pass --direct" in payload["hint"]
    payload, _ = invoke(station, capsys, ["q"], options=["--direct"])
    assert payload["path"] == "direct fake"


def test_direct_refuses_daemon_owner(station, daemon, capsys):
    payload, _ = invoke(station, capsys, ["q"], options=["--direct"], code=2)
    assert payload["error"] == "the daemon owns the arm"


def test_wildcard_bind_maps_to_loopback(station, capsys, monkeypatch):
    data = yaml.safe_load(station.read_text())
    data["daemon"] = {}  # Default bind is 0.0.0.0:50051.
    station.write_text(yaml.safe_dump(data))
    seen = []

    def probe(address):
        seen.append(address)
        return False

    monkeypatch.setattr(connect, "probe_daemon", probe)
    payload, _ = invoke(station, capsys, ["q"], code=1)
    assert "0.0.0.0:50051" in payload["error"]
    assert seen == ["127.0.0.1:50051"]


def test_tcp_probe_timeout_and_socket_cleanup(monkeypatch):
    seen = []

    class Socket:
        def __enter__(self):
            return self

        def __exit__(self, *args):
            seen.append("closed")

    def create(address, timeout):
        seen.append((address, timeout))
        return Socket()

    monkeypatch.setattr(socket, "create_connection", create)
    assert connect.probe_daemon("127.0.0.1:50051")
    assert seen == [(("127.0.0.1", 50051), 0.5), "closed"]


@pytest.mark.parametrize("command", [["home", "go"], ["gripper", "open"],
                                          ["gripper", "close"], ["gripper", "move", "0.4"]])
@pytest.mark.parametrize("mode", ["non-tty", "decline", "accept", "yes"])
def test_confirmation_matrix(station, capsys, built_robots, monkeypatch, command, mode):
    monkeypatch.setattr(sys.stdin, "isatty", lambda: mode in ("decline", "accept"))
    monkeypatch.setattr(builtins, "input", lambda prompt: "y" if mode == "accept" else "n")
    allowed = mode in ("accept", "yes")
    options = ["--yes"] if mode == "yes" else []
    payload, captured = invoke(station, capsys, command, options=options, code=0 if allowed else 2)
    robot = built_robots[-1]
    assert "target " in captured.err and "max delta " in captured.err
    if command[0] == "home":
        assert ("move_to_joints" in robot.arm.call_log) == allowed
        if not allowed:
            np.testing.assert_array_equal(robot.arm.read_state()["joint_pos"], np.zeros(7))
    else:
        assert (command[1] in robot.gripper.call_log) == allowed
    assert not robot.is_connected
    if not allowed:
        assert payload["error"] == ("cancelled" if mode == "decline" else "refusing to move without --yes")


@pytest.mark.parametrize("fraction", ["-0.1", "1.1", "nan", "inf"])
def test_invalid_gripper_fraction(station, capsys, built_robots, fraction):
    invoke(station, capsys, ["gripper", "move", fraction], options=["--yes"], code=2)
    assert not built_robots


def test_no_gripper_and_no_home(station, capsys):
    data = yaml.safe_load(station.read_text())
    data["gripper"] = None
    data["home_joints"] = None
    station.write_text(yaml.safe_dump(data))
    for command, expected in [(["gripper", "show"], "no gripper"), (["home", "show"], "no home")]:
        payload, _ = invoke(station, capsys, command, code=1)
        assert expected in payload["error"]


def test_device_fault_is_clean_and_disconnects(station, capsys, monkeypatch):
    from robot_infra.devices import DeviceFault
    from robot_infra.devices.backends.arms.fake import FakeArm

    closed = []
    original_release = FakeArm._release

    def fail(self):
        raise DeviceFault("test fault", hint="recover the arm")

    def release(self, handle):
        closed.append(True)
        original_release(self, handle)

    monkeypatch.setattr(FakeArm, "read_state", fail)
    monkeypatch.setattr(FakeArm, "_release", release)
    monkeypatch.delenv("RWRL_DEBUG", raising=False)
    payload, captured = invoke(station, capsys, ["q"], code=1)
    assert payload == {"error": "test fault", "hint": "recover the arm"}
    assert "Traceback" not in captured.err
    assert closed
    monkeypatch.setenv("RWRL_DEBUG", "1")
    _, captured = invoke(station, capsys, ["q"], code=1)
    assert "Traceback" in captured.err


def test_failed_connect_still_disconnects(station, capsys, monkeypatch, built_robots):
    from robot_infra.devices import Robot

    calls = []

    def fail(self):
        calls.append("connect")
        raise RuntimeError("connect failed")

    def disconnect(self):
        calls.append("disconnect")

    monkeypatch.setattr(Robot, "connect", fail)
    monkeypatch.setattr(Robot, "disconnect", disconnect)
    invoke(station, capsys, ["q"], code=1)
    assert calls == ["connect", "disconnect"]


def test_doctor_forwarding(station, capsys):
    assert main(["--station", str(station), "doctor", "--mock", "--json"]) == 0
    payload = json.loads(capsys.readouterr().out)
    assert payload["schema_version"] == 1
    assert any(item["name"] == "feedback:gripper" and item["ok"] for item in next(iter(payload["results"].values())))


@pytest.mark.parametrize("command,module", [
    ("doctor", "real_world_rl.cli.doctor"), ("setup", "robot_infra.setup.__main__"),
    ("daemon", "robot_infra.server.daemon"), ("supervise", "robot_infra.supervise.supervisor"),
])
def test_forward_argv_and_exit_code(monkeypatch, command, module):
    import types
    from real_world_rl.cli import forward

    seen = []
    monkeypatch.setattr(forward, "import_module", lambda name: (
        seen.append(name) or types.SimpleNamespace(main=lambda argv: seen.append(argv) or 7)
    ))
    remaining = ["--help", "--unknown=literal", "--", "python", "--json"]
    assert main([command, *remaining]) == 7
    assert seen == [module, remaining]


def test_global_doctor_options_are_translated(monkeypatch):
    import types
    from real_world_rl.cli import forward

    seen = []
    monkeypatch.setattr(forward, "import_module", lambda name: types.SimpleNamespace(
        main=lambda argv: seen.append(argv) or 0))
    assert main(["--station", "S", "--robot", "arm", "--mock", "--json", "doctor", "--fix"]) == 0
    assert seen == [["--config", "S", "--robot", "arm", "--mock", "--json", "--fix"]]


def test_missing_station_and_usage(capsys, monkeypatch, tmp_path):
    from robot_infra.discovery import autoconfig

    def missing(explicit):
        raise FileNotFoundError("no station found")

    monkeypatch.setattr(autoconfig, "resolve_station_config_path", missing)
    assert main(["--json", "status"]) == 2
    payload = json.loads(capsys.readouterr().out)
    assert "python -m robot_infra.setup" in payload["hint"]
    for argv in (["--json"], ["--json", "wat"], ["--json", "q", "--typo"],
                 ["--json", "gripper", "move", "bad"]):
        assert main(argv) == 2
        assert "error" in json.loads(capsys.readouterr().out)


def test_multi_robot_selection(station, capsys):
    data = yaml.safe_load(station.read_text())
    station.write_text(yaml.safe_dump({"robots": {"left": data, "right": data}}))
    payload, _ = invoke(station, capsys, ["q"], code=2)
    assert "--robot" in payload["error"]
    invoke(station, capsys, ["q"], options=["--robot", "right"])


def test_module_help_and_lazy_package_exports():
    root = Path(__file__).resolve().parents[1]
    script = '''
import importlib.abc
import runpy
import sys
class NoHeavyImports(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if fullname.split('.')[0] in {'torch', 'rl_garden', 'robot_infra', 'numpy', 'yaml'}:
            raise AssertionError('heavy import: ' + fullname)
sys.meta_path.insert(0, NoHeavyImports())
import real_world_rl
expected = {'ActorLoop', 'FWBWActorLoop', 'LearnerLoop', 'ActorSyncClient', 'LearnerSyncServer'}
assert set(real_world_rl.__all__) == expected
assert expected | {'loops'} <= set(dir(real_world_rl))
assert isinstance(real_world_rl.__version__, str)
assert 'real_world_rl.loops' not in sys.modules
sys.argv = ['rwrl', '--help']
runpy.run_module('real_world_rl.cli', run_name='__main__')
'''
    env = {**os.environ, "PYTHONDONTWRITEBYTECODE": "1"}
    result = subprocess.run([sys.executable, "-c", script], cwd=root, env=env,
                            capture_output=True, text=True, check=False)
    assert result.returncode == 0, result.stderr
    assert "Robot station hardware primitives" in result.stdout


def test_lazy_exports_resolve_on_first_access():
    # Substitute only the imported loops module: this checks PEP 562 without loading torch.
    root = Path(__file__).resolve().parents[1]
    script = '''
import sys
import types
import real_world_rl
fake = types.ModuleType('real_world_rl.loops')
for name in real_world_rl.__all__:
    setattr(fake, name, type(name, (), {}))
sys.modules['real_world_rl.loops'] = fake
from real_world_rl import ActorLoop, loops
assert loops is fake
assert ActorLoop is fake.ActorLoop
assert real_world_rl.ActorLoop is ActorLoop
assert dir(real_world_rl) == sorted(dir(real_world_rl))
'''
    result = subprocess.run([sys.executable, "-c", script], cwd=root, capture_output=True,
                            text=True, check=False)
    assert result.returncode == 0, result.stderr
