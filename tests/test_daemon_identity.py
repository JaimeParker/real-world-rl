"""Daemon identity checks never connect a mismatched robot or obstruct stop."""
from types import SimpleNamespace

import pytest

from test_cli_teleop_run import session, gello_station

from real_world_rl.common.robot_identity import expected_identity, identity_mismatches, robot_identity
from real_world_rl.cli import connect
from real_world_rl.cli.cmd_primitives import stop
from real_world_rl.cli.errors import CliError
from real_world_rl.devices.errors import DeviceError
from real_world_rl.devices.robots.franka import FrankaConfig, FrankaRobot
from real_world_rl.server.remote import RemoteRobot, RemoteRobotConfig
from real_world_rl.server.service import RobotService
from real_world_rl.server.transport import LocalClient


@pytest.mark.parametrize("attrs,env,address", [
    ({"robot_ip": "a"}, {"ROBOT_IP": "b"}, "a"),
    ({"robot_ip": ""}, {"ROBOT_IP": "b"}, "b"),
    ({"can_interface": None}, {"CAN_INTERFACE": "can0"}, "can0"),
    ({}, {"ROBOT_IP": "b", "CAN_INTERFACE": "can0"}, None),
    ({"robot_ip": "a", "can_interface": "can0"}, {}, "a"),
])
def test_expected_identity(attrs, env, address):
    config = SimpleNamespace(name="left", TYPE="franka", **attrs)
    assert expected_identity(config, env) == {"name": "left", "type": "franka", "address": address}
    assert robot_identity(config)["address"] == (attrs.get("robot_ip") or attrs.get("can_interface") or None)


def test_environment_default(monkeypatch):
    monkeypatch.setenv("ROBOT_IP", "environment")
    assert expected_identity(SimpleNamespace(name="left", TYPE="franka", robot_ip=""))["address"] == "environment"


def test_mismatch_rules():
    expected = {"name": "left", "type": "franka", "address": None}
    assert identity_mismatches(expected, {**expected, "address": "anything"}) == []
    assert identity_mismatches({**expected, "address": "a"}, expected) == ["address: station a, daemon None"]
    assert identity_mismatches(expected, {"name": "right", "type": "other"}) == [
        "name: station left, daemon right", "type: station franka, daemon other"]
    assert identity_mismatches(expected, None) == ["identity: not reported by the daemon (older version)"]


@pytest.fixture
def service():
    robot = FrankaRobot(FrankaConfig(name="left", arm_backend="fake", gripper=None,
                                    cameras=[], claim=False, limits={"joints": [[-4., 4.]] * 7}))
    service = RobotService(robot, watchdog_s=30.)
    yield service
    service.close()


class RecordingClient(LocalClient):
    identity = "original"

    def __init__(self, service):
        super().__init__(service)
        self.calls = []

    def call(self, method, **kwargs):
        self.calls.append(method)
        info = super().call(method, **kwargs)
        if method == "robot.describe" and self.identity != "original":
            info["identity"] = self.identity
        return info


def test_service_identity(service):
    assert service._describe()["identity"] == robot_identity(service.robot.config)
    service.robot.config.TYPE = "different"
    assert service._describe()["identity"]["type"] == service.robot.ROBOT_TYPE


@pytest.mark.parametrize("change", [{"name": "right"}, {"address": "wrong"}, None])
@pytest.mark.parametrize("refresh", [False, True])
def test_remote_refuses_before_connect(service, change, refresh):
    expected = robot_identity(service.robot.config)
    client = RecordingClient(service)
    remote = RemoteRobot(RemoteRobotConfig(expected_identity=expected), client=client)
    if refresh:
        remote.connect()
        arm = remote.arm
        client.calls.clear()
    client.identity = None if change is None else {**expected, **change}
    with pytest.raises(DeviceError) as error:
        remote.refresh_connection() if refresh else remote.connect()
    assert "127.0.0.1:50051" in str(error.value)
    assert error.value.hint == ("restart the daemon with the current version" if change is None else
        "the daemon at this address serves another robot: select the matching station or restart the daemon with this one")
    assert client.calls == ["robot.describe"]
    assert not client.is_open
    assert not remote.is_connected
    if refresh:
        with pytest.raises(DeviceError):
            arm.stop()
    assert remote.daemon_identity == client.identity


@pytest.mark.parametrize("identity", [None, {"name": "right", "type": "other", "address": "b"}])
def test_stop_allowed_and_default_unchanged(service, identity):
    client = RecordingClient(service)
    client.identity = identity
    remote = RemoteRobot(RemoteRobotConfig(), client=client)
    with connect._connected(remote):
        client.calls.clear()
        data, text = stop(connect.RobotConnection(remote, service.robot.config, "daemon local", True))
        assert data["ok"] is True
        assert data["identity_mismatches"]
        assert text.startswith("stop: ok\nwarning: stopped ")
        assert client.calls == ["arm.stop"]


def test_stop_diagnostic_error_is_ignored():
    calls = []
    class Connection:
        remote = True
        arm = SimpleNamespace(stop=lambda: calls.append("stop"))

        @property
        def daemon_identity(self):
            assert calls == ["stop"]
            raise RuntimeError("unavailable")
    assert stop(Connection()) == ({"ok": True}, "stop: ok")


@pytest.mark.parametrize("identity", ["original", None, {"name": "wrong"}])
def test_check_only_describes(service, monkeypatch, identity):
    from real_world_rl.server.transport import grpc_transport

    client = RecordingClient(service)
    client.identity = identity
    def factory(address, **kwargs):
        assert address == "local"
        assert kwargs == {"token": "secret", "reconnect_attempts": 0}
        return client
    monkeypatch.setattr(grpc_transport, "GrpcClient", factory)
    selected = SimpleNamespace(config=service.robot.config, token="secret")
    if identity == "original":
        connect.check_daemon_identity("local", selected)
    else:
        with pytest.raises(CliError):
            connect.check_daemon_identity("local", selected)
    assert client.calls == ["robot.describe"]
    assert not client.is_open
    assert not service.robot.is_connected


@pytest.mark.parametrize("command", ["stop", "status"])
def test_cli_passes_expected_identity(service, monkeypatch, command):
    selected = SimpleNamespace(config=service.robot.config, token=None)
    client = RecordingClient(service)
    client.identity = {"name": "other"}
    monkeypatch.setattr(connect, "_selected_robot", lambda args: selected)
    monkeypatch.setattr(connect, "_daemon_address", lambda args, selected: "local")
    from real_world_rl.server import remote as module
    original = module.RemoteRobot
    monkeypatch.setattr(module, "RemoteRobot", lambda config: original(config, client=client))
    if command == "stop":
        with connect.open_robot(SimpleNamespace(command=command)) as connection:
            assert stop(connection)[0]["ok"]
        assert "arm.stop" in client.calls
    else:
        with pytest.raises(DeviceError):
            with connect.open_robot(SimpleNamespace(command=command)):
                pytest.fail("must refuse")
        assert client.calls == ["robot.describe"]


def test_describe_transport_error_closes_client(service, monkeypatch):
    from real_world_rl.server.transport import grpc_transport

    client = RecordingClient(service)
    def fail(method):
        raise RuntimeError("transport failed")
    monkeypatch.setattr(client, "call", fail)
    monkeypatch.setattr(grpc_transport, "GrpcClient", lambda *args, **kwargs: client)
    with pytest.raises(RuntimeError, match="transport failed"):
        connect.describe_daemon_identity("local", None)
    assert not client.is_open


def test_supervised_refuses_before_report_or_child(session, tmp_path, monkeypatch, capsys):
    from real_world_rl.cli import main

    gello_station(session, daemon=True)
    calls = []
    def describe(address, token):
        calls.append(address)
        assert not list(tmp_path.glob("gello.session-*"))
        return {"name": "other", "type": "franka", "address": None}
    monkeypatch.setattr(connect, "describe_daemon_identity", describe)
    assert main([*session.prefix, "teleop", "run", "--record", str(tmp_path / "gello")]) == 1
    assert calls == ["127.0.0.1:50051"]
    assert session.calls == []
    assert not list(tmp_path.glob("gello.session-*"))
    assert "identity mismatch" in capsys.readouterr().out
