"""Daemon on a station file: one child process per robot (launcher monkeypatched), signal
forwarding, exit codes, --robot selection and the per-robot daemon block."""
from __future__ import annotations

import os
import signal
import sys

import pytest
import yaml

import robot_infra.devices.claims as claims
import robot_infra.server.daemon as daemon
from robot_infra.discovery.autoconfig import STATION_CONFIG_ENV
from robot_infra.server.transport import Transport

STATION = {
    "robots": {
        "left": {
            "type": "franka",
            "arm_backend": "fake",
            "gripper": {"backend": "fake"},
            "daemon": {"bind": "0.0.0.0:50051"},
        },
        "right": {
            "type": "franka",
            "arm_backend": "fake",
            "gripper": {"backend": "fake"},
            "daemon": {"bind": "0.0.0.0:50052", "token": "t"},
        },
    }
}


class FakeChild:
    """A daemon process stand-in: ``poll`` returns ``exit_code`` after ``polls_to_exit`` polls, or
    once a signal was forwarded; ``on_first_poll`` runs once."""

    def __init__(self, argv, kwargs, *, exit_code=0, polls_to_exit=1, on_first_poll=None):
        self.argv, self.kwargs = argv, kwargs
        self.exit_code, self.polls_to_exit = exit_code, polls_to_exit
        self.on_first_poll = on_first_poll
        self.polls = 0
        self.signals: list[int] = []

    def poll(self):
        self.polls += 1
        if self.on_first_poll is not None:
            hook, self.on_first_poll = self.on_first_poll, None
            hook()
        if self.signals:
            return 0
        if self.polls_to_exit is not None and self.polls >= self.polls_to_exit:
            return self.exit_code
        return None

    def send_signal(self, signum):
        self.signals.append(signum)

    def wait(self):
        return 0


@pytest.fixture
def station_path(tmp_path, monkeypatch):
    monkeypatch.setattr(claims, "_DEFAULT_LOCK_DIR", tmp_path / "claims")
    path = tmp_path / "station.yaml"
    path.write_text(yaml.safe_dump(STATION))
    return path


@pytest.fixture
def launcher(monkeypatch):
    """Records launches; ``launcher.behaviour[name]`` sets FakeChild keyword arguments per robot."""

    class Launcher:
        children: list[FakeChild] = []
        behaviour: dict[str, dict] = {}

        def __call__(self, argv, **kwargs):
            robot = argv[argv.index("--robot") + 1]
            child = FakeChild(argv, kwargs, **self.behaviour.get(robot, {}))
            self.children.append(child)
            return child

    launcher = Launcher()
    launcher.children, launcher.behaviour = [], {}
    monkeypatch.setattr(daemon, "_launch_daemon", launcher)
    monkeypatch.setattr(daemon, "_sleep", lambda seconds: None)
    return launcher


class StubTransport(Transport):
    def __init__(self) -> None:
        self.served = None

    def serve(self, service, bind, *, token=None) -> None:
        self.served = (service, bind, token)

    def start(self, service, bind, *, token=None) -> None:
        self.served = (service, bind, token)

    def stop(self, grace_s: float = 1.0) -> None:
        pass


@pytest.fixture
def stub_transport(monkeypatch):
    transport = StubTransport()
    monkeypatch.setattr(daemon, "_make_transport", lambda name: transport)
    return transport


def test_one_child_per_robot_with_the_shared_flags(station_path, launcher, capsys):
    argv = ["--config", str(station_path), "--no-preflight", "--watchdog-s", "0.25", "--jpeg-quality", "80"]
    assert daemon.main(argv) == 0
    assert [child.argv for child in launcher.children] == [
        [
            sys.executable, "-m", "robot_infra.server", "--config", os.path.abspath(station_path),
            "--robot", robot, "--watchdog-s", "0.25", "--transport", "grpc", "--no-preflight",
            "--jpeg-quality", "80",
        ]
        for robot in ("left", "right")
    ]
    assert all(child.kwargs == {"start_new_session": True} for child in launcher.children)
    out = capsys.readouterr().out
    assert "starting 2 daemons" in out and "'right' on 0.0.0.0:50052" in out
    assert "daemon 'left' exited with code 0" in out


def test_preflight_flag_is_not_passed_when_unset(station_path, launcher):
    assert daemon.main(["--config", str(station_path)]) == 0
    assert all("--no-preflight" not in child.argv and "--jpeg-quality" not in child.argv for child in launcher.children)


def test_exit_code_is_the_first_non_zero_in_robot_order(station_path, launcher):
    launcher.behaviour = {"left": {"exit_code": 0}, "right": {"exit_code": 3, "polls_to_exit": 3}}
    assert daemon.main(["--config", str(station_path)]) == 3
    launcher.children.clear()
    launcher.behaviour = {"left": {"exit_code": -signal.SIGTERM}, "right": {"exit_code": 3}}
    assert daemon.main(["--config", str(station_path)]) == 128 + signal.SIGTERM


def test_sigterm_is_forwarded_to_every_child_and_the_handler_restored(station_path, launcher):
    previous = signal.getsignal(signal.SIGTERM)
    launcher.behaviour = {
        "left": {"polls_to_exit": None, "on_first_poll": lambda: signal.raise_signal(signal.SIGTERM)},
        "right": {"polls_to_exit": None},
    }
    assert daemon.main(["--config", str(station_path)]) == 0
    assert [child.signals for child in launcher.children] == [[signal.SIGTERM], [signal.SIGTERM]]
    assert signal.getsignal(signal.SIGTERM) == previous


@pytest.mark.parametrize(
    "extra, message",
    [(["--bind", "0.0.0.0:1"], "--bind/--token apply to one robot"), (["--token", "x"], "--bind/--token apply to one robot")],
)
def test_bind_or_token_with_several_robots_is_an_error(station_path, launcher, capsys, extra, message):
    assert daemon.main(["--config", str(station_path), *extra]) == 2
    assert launcher.children == [] and message in capsys.readouterr().err


def test_two_robots_on_one_bind_is_an_error(tmp_path, launcher, capsys):
    data = yaml.safe_load(yaml.safe_dump(STATION))
    data["robots"]["right"]["daemon"]["bind"] = "0.0.0.0:50051"
    path = tmp_path / "station.yaml"
    path.write_text(yaml.safe_dump(data))
    assert daemon.main(["--config", str(path)]) == 2
    assert launcher.children == [] and "both bind 0.0.0.0:50051" in capsys.readouterr().err


def test_robot_flag_serves_one_robot_with_its_daemon_block(station_path, launcher, stub_transport, capsys):
    assert daemon.main(["--config", str(station_path), "--robot", "right", "--no-preflight"]) == 0
    service, bind, token = stub_transport.served
    assert (bind, token) == ("0.0.0.0:50052", "t") and service.robot.name == "right"
    assert launcher.children == []
    assert "serving franka 'right' on 0.0.0.0:50052 (token required)" in capsys.readouterr().out


def test_command_line_bind_and_token_override_the_block_for_one_robot(station_path, stub_transport):
    argv = ["--config", str(station_path), "--robot", "right", "--no-preflight", "--bind", "127.0.0.1:0", "--token", "u"]
    assert daemon.main(argv) == 0
    assert stub_transport.served[1:] == ("127.0.0.1:0", "u")


def test_build_daemon_selects_the_robot(station_path, stub_transport):
    service, _ = daemon.build_daemon(["--config", str(station_path), "--robot", "left"])
    try:
        assert service.robot.name == "left"
    finally:
        service.close()
    with pytest.raises(ValueError, match="pass --robot <name>"):
        daemon.build_daemon(["--config", str(station_path)])


def test_no_config_resolves_the_station(station_path, stub_transport, monkeypatch):
    monkeypatch.setenv(STATION_CONFIG_ENV, str(station_path))
    assert daemon.main(["--robot", "left", "--no-preflight"]) == 0
    assert stub_transport.served[1] == "0.0.0.0:50051"


def test_no_config_and_no_station_exits_2(tmp_path, monkeypatch, capsys):
    monkeypatch.delenv(STATION_CONFIG_ENV, raising=False)
    monkeypatch.setenv("XDG_CONFIG_HOME", str(tmp_path / "xdg"))
    monkeypatch.chdir(tmp_path)
    assert daemon.main(["--no-preflight"]) == 2
    assert "python -m robot_infra.setup" in capsys.readouterr().err
