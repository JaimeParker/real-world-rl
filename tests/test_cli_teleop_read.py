"""`rwrl teleop read`: hardware-free; the direct route opens only the teleop part."""
from __future__ import annotations

import json
import socket
import sys

import numpy as np
import pytest
import yaml
from _leader_teleop_fakes import FakeGelloTeleop
from real_world_rl.devices.teleop import EVENT_KEYS, Teleop, TeleopSample

from real_world_rl.cli import connect, main
from real_world_rl.cli.teleop_read import sample_record, text_line

LEADER_Q = [0.3, -0.02, 0.0, -1.5, 0.0, 1.5, 0.12]
SCRIPT = [{"action": LEADER_Q, "gripper": 0.25, "active": True, "events": {"success": True}},
          {"action": LEADER_Q, "gripper": 0.25, "active": False}]


@pytest.fixture(autouse=True)
def offline(monkeypatch):
    def forbidden(*args, **kwargs):
        raise AssertionError("CLI tests must not open sockets")

    monkeypatch.setattr(socket, "create_connection", forbidden)
    monkeypatch.setattr(sys.stdin, "isatty", lambda: False)


@pytest.fixture
def leader(monkeypatch):
    class Leader(FakeGelloTeleop):
        script = SCRIPT
        instances = []

    monkeypatch.setitem(Teleop._BACKENDS, "gello", Leader)
    yield Leader
    for part in Leader.instances:
        part.disconnect()


def write_station(tmp_path, teleop):
    path = tmp_path / "station.yaml"
    data = {"type": "franka", "name": "cli_fake", "arm_backend": "fake", "claim": False,
            "gripper": {"backend": "fake"}}
    if teleop is not None:
        data["teleop"] = teleop
    path.write_text(yaml.safe_dump(data), encoding="utf-8")
    return path


GELLO_TELEOP = {"name": "leader", "backend": "gello",
                "options": {"port": "/dev/serial/by-id/fake", "joint_ids": [1, 2, 3, 4, 5, 6, 7], "gripper_id": 8}}


@pytest.fixture
def arm_never_opened(monkeypatch):
    from real_world_rl.devices.backends.arms.fake import FakeArm
    from real_world_rl.devices.backends.grippers.fake import FakeGripper

    def refuse(self):
        raise AssertionError("teleop read must not open the arm or the gripper on the direct route")

    monkeypatch.setattr(FakeArm, "_open", refuse)
    monkeypatch.setattr(FakeGripper, "_open", refuse)


def run(path, capsys, *command, options=()):
    code = main(["--station", str(path), *options, "teleop", "read", *command])
    return code, capsys.readouterr()


def test_direct_jsonl_opens_only_the_teleop(tmp_path, capsys, leader, arm_never_opened):
    path = write_station(tmp_path, GELLO_TELEOP)
    code, captured = run(path, capsys, "--duration-s", "0.3", "--rate-hz", "20", options=["--json"])
    assert code == 0, captured.err
    records = [json.loads(line) for line in captured.out.splitlines()]
    samples, summary = records[:-1], records[-1]["summary"]
    assert samples and set(records[-1]) == {"summary"}
    first = samples[0]
    assert set(first) == {"t", "seq", "q", "gripper", "intervening", "events"}
    assert first["q"] == pytest.approx(LEADER_Q)
    assert first["gripper"] == 0.25 and first["intervening"] and first["events"] == ["success"]
    assert all(sample["events"] == [] for sample in samples[1:])
    assert summary["samples"] == len(samples)
    assert summary["events_seen"] == {"success": 1}
    assert (summary["action_space"], summary["backend"], summary["path"]) == ("joint_abs", "gello", "direct gello")
    assert summary["duration_s"] >= 0.3
    assert not leader.instances[-1].is_connected


def test_text_lines_match_the_leader_heredoc_fields(tmp_path, capsys, leader, arm_never_opened):
    path = write_station(tmp_path, GELLO_TELEOP)
    code, captured = run(path, capsys, "--duration-s", "0.1", "--rate-hz", "50")
    assert code == 0, captured.err
    lines = captured.out.splitlines()
    assert lines[0].startswith("t=")
    assert "q=[0.3, -0.02, 0.0, -1.5, 0.0, 1.5, 0.12]" in lines[0]
    assert "gripper=0.250 intervening=True events=[success]" in lines[0]
    assert lines[-1].startswith("samples=")
    assert "backend=gello" in lines[-1]


def test_cartesian_teleop_names_the_action_twist(tmp_path, capsys):
    path = write_station(tmp_path, {"name": "mouse", "backend": "fake", "options": {"script": [{"active": True}]}})
    code, captured = run(path, capsys, "--duration-s", "0.1", "--rate-hz", "50", options=["--json"])
    assert code == 0, captured.err
    first = json.loads(captured.out.splitlines()[0])
    assert first["twist"] == [0.0] * 6 and "q" not in first


def test_ctrl_c_ends_normally(tmp_path, capsys, monkeypatch, leader):
    class Interrupted(leader):
        reads = 0

        def read(self, timeout_s=None):
            type(self).reads += 1
            if type(self).reads > 2:
                raise KeyboardInterrupt
            return super().read(timeout_s)

    monkeypatch.setitem(Teleop._BACKENDS, "gello", Interrupted)
    path = write_station(tmp_path, GELLO_TELEOP)
    code, captured = run(path, capsys, "--rate-hz", "100", options=["--json"])
    assert code == 0, captured.err
    assert json.loads(captured.out.splitlines()[-1])["summary"]["samples"] == 2


def test_read_failure_exits_1_with_summary(tmp_path, capsys):
    teleop = {"name": "mouse", "backend": "fake", "options": {"fail_after": 1}}
    path = write_station(tmp_path, teleop)
    code, captured = run(path, capsys, "--duration-s", "5", "--rate-hz", "100", options=["--json"])
    assert code == 1
    summary = json.loads(captured.out.splitlines()[-1])["summary"]
    assert summary["samples"] == 1 and summary["error"]
    assert "error: " in captured.err


def test_with_follower_is_refused_on_the_direct_route(tmp_path, capsys, leader, arm_never_opened):
    path = write_station(tmp_path, GELLO_TELEOP)
    code, captured = run(path, capsys, "--with-follower", options=["--json"])
    assert code == 2
    payload = json.loads(captured.out)
    assert "follower comparison needs the daemon route" in payload["hint"]
    assert leader.instances == []


def test_station_without_teleop_is_a_usage_error(tmp_path, capsys):
    path = write_station(tmp_path, None)
    code, captured = run(path, capsys, options=["--json"])
    assert code == 2
    assert "station has no teleop" in json.loads(captured.out)["error"]


@pytest.mark.parametrize("flag,value", [("--rate-hz", "0"), ("--duration-s", "-1")])
def test_non_positive_rates_are_usage_errors(tmp_path, capsys, leader, flag, value):
    path = write_station(tmp_path, GELLO_TELEOP)
    code, _ = run(path, capsys, flag, value, options=["--json"])
    assert code == 2


@pytest.fixture
def daemon(tmp_path, monkeypatch):
    from real_world_rl.devices import Robot
    from real_world_rl.discovery.autoconfig import load_station
    from real_world_rl.server import remote
    from real_world_rl.server.service import RobotService
    from real_world_rl.server.transport.local import LocalClient

    teleop = {"name": "leader", "backend": "fake",
              "options": {"action_space": "joint_abs", "num_joints": 7, "has_gripper": True, "script": SCRIPT}}
    station = write_station(tmp_path, teleop)
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
    yield station, created
    service.close()


def test_daemon_route_prints_follower_and_delta(capsys, daemon):
    station, created = daemon
    code, captured = run(station, capsys, "--with-follower", "--duration-s", "0.2", "--rate-hz", "20",
                         options=["--json"])
    assert code == 0, captured.err
    records = [json.loads(line) for line in captured.out.splitlines()]
    first = records[0]
    assert first["follower_q"] == [0.0] * 7
    assert first["delta"] == pytest.approx(LEADER_Q)
    assert first["events"] == ["success"]
    assert records[-1]["summary"]["path"] == "daemon 127.0.0.1:0"
    assert all(not robot.is_connected and not robot.client.is_open for robot in created)


@pytest.mark.parametrize("action_space,action", [("joint_abs", [0.5, -0.25]), ("joint_delta", [0.5, -0.25]),
                                                 ("cartesian_delta", [0.1] * 6)])
def test_delta_only_for_absolute_joint_targets(action_space, action):
    sample = TeleopSample(action=np.asarray(action, dtype=np.float32), action_space=action_space, gripper=None,
                          buttons={}, events={key: False for key in EVENT_KEYS}, intervening=False,
                          timestamp=0.0, seq=1)
    record = sample_record(sample, 0.0, follower_q=[0.25, 0.25])
    assert record["follower_q"] == [0.25, 0.25]
    if action_space == "joint_abs":
        assert record["delta"] == [0.25, -0.5]
        assert "delta=[0.25, -0.5]" in text_line(record)
    else:
        assert "delta" not in record
        assert "delta=" not in text_line(record)
