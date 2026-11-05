"""Doctor layer 0 (extras), the libfranka mismatch parser and handshake report, and the
``--install-sdk`` flow; ``importlib.util.find_spec``, pip and the prompt are faked."""
from __future__ import annotations

import importlib.util
import json
import subprocess
import types

import pytest
import yaml

from robot_infra.devices import DeviceNotFound, GripperConfig, TeleopConfig, VersionMismatch
from robot_infra.devices.backends.arms.fake import FakeArm
from robot_infra.devices.backends.arms.franky import (
    franky_connect_error,
    parse_libfranka_mismatch,
)
from robot_infra.devices.backends.grippers.fake import FakeGripper
from robot_infra.devices.robots.franka import FrankaConfig
from robot_infra.discovery import enumerate as probes
from robot_infra.discovery.autoconfig import load_station
from robot_infra.doctor import cli, deps
from robot_infra.doctor.checks import LIBFRANKA_SERVER_VERSION, exit_code, run_doctor

MISMATCH_MESSAGE = (
    "libfranka/robot firmware mismatch: libfranka: Incompatible library version (server version: 10, "
    "library version: 9). Please check https://frankaemika.github.io for Robot system compatibility with libfranka."
)
INSTALL_ARGV_10 = deps.franky_wheel_install_argv(10)


@pytest.fixture(autouse=True)
def no_network(monkeypatch, tmp_path):
    import robot_infra.devices.claims as claims

    monkeypatch.setattr(probes, "ping", lambda ip, timeout_s=1.0: True)
    monkeypatch.setattr(probes, "tcp_open", lambda ip, port, timeout_s=0.5: False)
    monkeypatch.setattr(claims, "_DEFAULT_LOCK_DIR", tmp_path / "claims")


def gello_station(event_device: str | None = None) -> FrankaConfig:
    options = {"port": "/dev/serial/by-id/usb-FTDI_fake-if00-port0"}
    if event_device:
        options["event_device"] = event_device
    return FrankaConfig(
        arm_backend="franky",
        gripper=GripperConfig(backend="franky"),
        teleop=TeleopConfig(backend="gello", options=options),
    )


def fake_find_spec(monkeypatch, missing: set[str]):
    real_find_spec = importlib.util.find_spec
    known = {module for modules in deps.EXTRA_MODULES.values() for module in modules}

    def find_spec(name, *args, **kwargs):
        if name in missing:
            return None
        if name in known:
            return object()
        return real_find_spec(name, *args, **kwargs)

    monkeypatch.setattr(importlib.util, "find_spec", find_spec)


# -- layer 0: deps ------------------------------------------------------------------------------


def test_required_extras_for_a_franky_gello_station():
    assert deps.required_extras(gello_station()) == ["franky", "gello"]
    assert deps.required_extras(gello_station(), serves_daemon=True) == ["franky", "gello", "grpc"]
    assert deps.required_extras(gello_station("/dev/input/event3"), serves_daemon=True) == [
        "franky",
        "gello",
        "gello-events",
        "grpc",
    ]


def test_fake_backends_need_no_extras_and_no_transport():
    config = FrankaConfig(arm_backend="fake", gripper=GripperConfig(backend="fake"))
    assert deps.required_extras(config) == []
    assert deps.deps_outcome(config) == deps.DepsOutcome(True, "no extras needed (fake or SDK-free backends)")


def test_deps_pass_names_every_extra(monkeypatch):
    fake_find_spec(monkeypatch, missing=set())
    assert deps.deps_outcome(gello_station(), serves_daemon=True) == deps.DepsOutcome(
        True, "extras ok: franky, gello, grpc"
    )
    assert deps.deps_outcome(gello_station()) == deps.DepsOutcome(True, "extras ok: franky, gello")


def test_deps_fail_lists_missing_extras_and_the_exact_install_command(monkeypatch):
    fake_find_spec(monkeypatch, missing={"franky", "dynamixel_sdk"})
    outcome = deps.deps_outcome(gello_station())
    assert not outcome.ok and outcome.message == "missing extras for this station: franky, gello"
    assert outcome.hint.startswith("pip install -e 'robot_infra[franky,gello]'")
    assert "--install-sdk" in outcome.hint


def test_missing_grpc_fails_only_for_a_declared_daemon(monkeypatch):
    fake_find_spec(monkeypatch, missing={"grpc"})
    served = deps.deps_outcome(gello_station(), serves_daemon=True)
    assert not served.ok and served.message == "missing extras for this station: grpc"
    assert served.hint.startswith("pip install -e 'robot_infra[grpc]'")
    single_process = deps.deps_outcome(gello_station())
    assert single_process.ok and single_process.warning
    assert single_process.message == "extras ok: franky, gello; optional, not installed: grpc (daemon route)"
    assert "robot_infra[grpc]" in single_process.hint


def test_missing_optional_jpeg_is_reported_but_passes(monkeypatch):
    fake_find_spec(monkeypatch, missing={"PIL", "cv2"})
    outcome = deps.deps_outcome(gello_station(), serves_daemon=True)
    assert outcome.ok and not outcome.warning
    assert outcome.message == "extras ok: franky, gello, grpc; optional, not installed: jpeg (daemon --jpeg-quality)"


def test_jpeg_counts_as_installed_with_opencv_only(monkeypatch):
    fake_find_spec(monkeypatch, missing={"PIL"})
    assert deps.extra_installed("jpeg")


def test_deps_is_layer_zero_and_runs_on_the_mocked_config(monkeypatch):
    fake_find_spec(monkeypatch, missing={"franky", "dynamixel_sdk"})
    results = run_doctor(gello_station(), mock=True)
    assert (results[0].layer, results[0].name, results[0].ok) == (0, "deps", True)
    real = run_doctor(gello_station())
    assert (real[0].layer, real[0].name, real[0].ok) == (0, "deps", False)
    assert exit_code(real) == 1


def test_mock_never_requires_grpc(monkeypatch):
    fake_find_spec(monkeypatch, missing={"grpc"})
    deps_result = run_doctor(gello_station(), mock=True, serves_daemon=True)[0]
    assert deps_result.ok and not deps_result.warning


def write_station_yaml(tmp_path, robot: dict) -> str:
    path = tmp_path / "station.yaml"
    path.write_text(yaml.safe_dump(robot))
    return str(path)


def test_doctor_station_requires_grpc_only_with_a_daemon_block(tmp_path, monkeypatch, capsys):
    fake_find_spec(monkeypatch, missing={"grpc"})
    monkeypatch.setattr(deps, "ARM_BACKEND_EXTRAS", {"fake": "franky"})  # a fake arm that "needs" an extra
    robot = {"type": "franka", "arm_backend": "fake", "gripper": {"backend": "fake"}}
    single = cli.doctor_station(write_station_yaml(tmp_path, robot))
    (single_deps,) = [r for results in single.values() for r in results if r.name == "deps"]
    assert single_deps.ok and single_deps.warning and "grpc (daemon route)" in single_deps.message
    assert "WARN" in cli.format_station_text(single)
    served = cli.doctor_station(write_station_yaml(tmp_path, dict(robot, daemon={"bind": "0.0.0.0:50051"})))
    (served_deps,) = [r for results in served.values() for r in results if r.name == "deps"]
    assert not served_deps.ok and served_deps.message == "missing extras for this station: grpc"


# -- libfranka mismatch ------------------------------------------------------------------------


@pytest.mark.parametrize(
    ("message", "versions"),
    [
        ("libfranka: Incompatible library version (server version: 10, library version: 9).", (10, 9)),
        (MISMATCH_MESSAGE, (10, 9)),
        ("Incompatible library version. Server version: 5\nLibrary version: 4", (5, 4)),
        ("libfranka gripper: Incompatible library version (server version: 3, library version: 2)", (3, 2)),
        ("server version: 5, library version: 9", (5, 9)),
    ],
)
def test_parse_libfranka_mismatch(message, versions):
    assert parse_libfranka_mismatch(message) == versions


@pytest.mark.parametrize(
    "message",
    [
        # field wording from the 2026-11-04 commissioning doctor log (FCI not yet activated)
        "libfranka: Connection to FCI refused. Please install FCI feature or enable FCI mode in Desk.",
        "UDP receive: Timeout",
        "",
    ],
)
def test_parse_libfranka_mismatch_ignores_other_errors(message):
    assert parse_libfranka_mismatch(message) is None


def fake_franky_module() -> types.SimpleNamespace:
    return types.SimpleNamespace(
        IncompatibleVersionException=type("IncompatibleVersionException", (Exception,), {}),
        RealtimeException=type("RealtimeException", (Exception,), {}),
    )


def test_connect_error_keeps_the_libfranka_text_and_points_at_install_sdk():
    franky = fake_franky_module()
    error = franky_connect_error(
        franky,
        franky.IncompatibleVersionException(
            "libfranka: Incompatible library version (server version: 10, library version: 9)"
        ),
        "10.0.0.2",
        "arm",
    )
    assert isinstance(error, VersionMismatch) and parse_libfranka_mismatch(error.message) == (10, 9)
    assert "system version in Desk → Settings → System" in error.hint and "--install-sdk" in error.hint
    assert "https://frankarobotics.github.io/docs/compatibility.html" in error.hint


def test_unreachable_hint_lists_the_bring_up_steps():
    error = franky_connect_error(fake_franky_module(), RuntimeError("UDP receive: Timeout"), "10.0.0.2", "arm")
    assert isinstance(error, DeviceNotFound)
    assert error.hint == (
        "check: robot powered on and brakes released; Desk reachable at https://10.0.0.2/desk; "
        "you hold control in Desk; FCI activated (Desk → Activate FCI); cable on the Control port; "
        "firewall allows UDP"
    )


def test_franky_ping_failure_gets_the_bring_up_hint(monkeypatch):
    from robot_infra.doctor import checks

    monkeypatch.setattr(probes, "ping", lambda ip, timeout_s=1.0: False)
    named = dict(checks._os_checks(FrankaConfig(robot_ip="10.0.0.2", gripper=None)))
    result = named["os:ping"]()
    assert not result.ok and "https://10.0.0.2/desk" in result.hint and "Activate FCI" in result.hint


def fake_station_config(**kwargs) -> FrankaConfig:
    return FrankaConfig(arm_backend="fake", gripper=GripperConfig(backend="fake"), **kwargs)


@pytest.fixture
def arm_mismatch(monkeypatch):
    def open_fails(self):
        raise VersionMismatch(MISMATCH_MESSAGE, hint="backend hint", device=self.name)

    monkeypatch.setattr(FakeArm, "_open", open_fails)


def test_handshake_reports_both_versions(arm_mismatch):
    handshake = next(r for r in run_doctor(fake_station_config()) if r.name == "handshake")
    assert not handshake.ok
    assert handshake.message == "libfranka mismatch: robot server 10, installed library 9"
    assert handshake.hint == (
        "the robot server version 10 comes from libfranka's handshake error (authoritative); "
        "to cross-check, read the system version in Desk → Settings → System and look it up in the "
        "compatibility matrix https://frankarobotics.github.io/docs/compatibility.html; "
        "then: python -m robot_infra.doctor --config <cfg> --install-sdk"
    )
    assert handshake.details[LIBFRANKA_SERVER_VERSION] == 10


def test_handshake_hint_names_the_config_path(arm_mismatch):
    results = run_doctor(fake_station_config(), config_path="/etc/robot.yaml")
    handshake = next(r for r in results if r.name == "handshake")
    assert handshake.hint.endswith("python -m robot_infra.doctor --config /etc/robot.yaml --install-sdk")


def test_gripper_mismatch_keeps_the_backend_message(monkeypatch):
    def open_fails(self):
        raise VersionMismatch(MISMATCH_MESSAGE, hint="backend hint", device=self.name)

    monkeypatch.setattr(FakeGripper, "_open", open_fails)
    handshake = next(r for r in run_doctor(fake_station_config()) if r.name == "handshake")
    assert handshake.message == MISMATCH_MESSAGE and handshake.hint == "backend hint"
    assert not handshake.details


# -- --install-sdk ---------------------------------------------------------------------------


def write_config(tmp_path) -> str:
    path = tmp_path / "robot.yaml"
    path.write_text(yaml.safe_dump({"type": "franka", "arm_backend": "fake", "gripper": {"backend": "fake"}}))
    return str(path)


class FakeSubprocess:
    def __init__(self, monkeypatch, returncodes=()):
        self.calls: list[list[str]] = []
        self.returncodes = list(returncodes)
        monkeypatch.setattr(subprocess, "run", self)

    def __call__(self, argv, *args, **kwargs):
        self.calls.append(list(argv))
        code = self.returncodes.pop(0) if self.returncodes else 0
        return subprocess.CompletedProcess(argv, code)


class Prompt:
    def __init__(self, monkeypatch, answer: str | None):
        self.prompts: list[str] = []
        self.answer = answer
        monkeypatch.setattr("builtins.input", self)

    def __call__(self, prompt: str = "") -> str:
        self.prompts.append(prompt)
        if self.answer is None:
            raise AssertionError(f"prompted: {prompt}")
        return self.answer


def robot_name(path: str) -> str:
    return next(iter(load_station(path).robots))


def test_install_sdk_yes_installs_and_reruns_the_doctor(tmp_path, monkeypatch, capsys, arm_mismatch):
    path = write_config(tmp_path)
    pip = FakeSubprocess(monkeypatch)
    prompt = Prompt(monkeypatch, "y")
    assert cli.main(["--config", path, "--install-sdk"]) == 0
    out = capsys.readouterr().out
    assert prompt.prompts == ["Install now? [y/N] "]
    assert "robot server version 10 detected" in out
    assert "comes from libfranka's handshake error (authoritative)" in out
    assert "system version in Desk → Settings → System" in out and "compatibility.html" in out
    assert "by-robot-server-version/10/" in out and "--force-reinstall --no-deps" in out
    assert pip.calls[0] == INSTALL_ARGV_10
    assert pip.calls[0][1:5] == ["-m", "pip", "install", "franky-control"]
    assert pip.calls[1] == cli.doctor_rerun_argv(path, robot_name(path))
    assert "--install-sdk" not in pip.calls[1]


@pytest.mark.parametrize("answer", ["n", "N", "", "no"])
def test_install_sdk_declined_installs_nothing(tmp_path, monkeypatch, capsys, arm_mismatch, answer):
    path = write_config(tmp_path)
    pip = FakeSubprocess(monkeypatch)
    Prompt(monkeypatch, answer)
    assert cli.main(["--config", path, "--install-sdk"]) == 1
    assert pip.calls == [] and "not installed" in capsys.readouterr().out


def test_install_sdk_end_of_input_counts_as_no(tmp_path, monkeypatch, capsys, arm_mismatch):
    path = write_config(tmp_path)
    pip = FakeSubprocess(monkeypatch)

    def closed_stdin(prompt=""):
        raise EOFError

    monkeypatch.setattr("builtins.input", closed_stdin)
    assert cli.main(["--config", path, "--install-sdk"]) == 1
    assert pip.calls == []
    capsys.readouterr()


def test_install_sdk_with_yes_flag_skips_the_prompt(tmp_path, monkeypatch, capsys, arm_mismatch):
    path = write_config(tmp_path)
    pip = FakeSubprocess(monkeypatch, returncodes=[0, 1])
    Prompt(monkeypatch, None)
    assert cli.main(["--config", path, "--install-sdk", "--yes"]) == 1  # the fresh doctor run still fails
    assert pip.calls[0] == INSTALL_ARGV_10 and len(pip.calls) == 2
    capsys.readouterr()


def test_install_sdk_pip_failure_skips_the_rerun(tmp_path, monkeypatch, capsys, arm_mismatch):
    path = write_config(tmp_path)
    pip = FakeSubprocess(monkeypatch, returncodes=[2])
    assert cli.main(["--config", path, "--install-sdk", "--yes"]) == 1
    assert len(pip.calls) == 1 and "pip exited with code 2" in capsys.readouterr().out


def test_install_sdk_json_reports_the_command_without_prompting(tmp_path, monkeypatch, capsys, arm_mismatch):
    path = write_config(tmp_path)
    pip = FakeSubprocess(monkeypatch)
    Prompt(monkeypatch, None)
    assert cli.main(["--config", path, "--install-sdk", "--json"]) == 1
    payload = json.loads(capsys.readouterr().out)
    assert pip.calls == []
    (entry,) = payload["install_sdk"]
    assert (entry["robot"], entry["server_version"], entry["library_version"]) == (robot_name(path), 10, 9)
    assert entry["command"].split()[1:] == INSTALL_ARGV_10[1:]
    handshake = next(result for result in payload["results"] if result["name"] == "handshake")
    assert handshake["message"] == "libfranka mismatch: robot server 10, installed library 9"
    assert path in handshake["hint"]


def test_install_sdk_without_mismatch_installs_nothing(tmp_path, monkeypatch, capsys):
    path = write_config(tmp_path)
    pip = FakeSubprocess(monkeypatch)
    Prompt(monkeypatch, None)
    assert cli.main(["--config", path, "--mock", "--install-sdk"]) == 0
    assert pip.calls == [] and "nothing to install" in capsys.readouterr().out


def test_install_sdk_refuses_robots_with_different_server_versions(tmp_path, monkeypatch, capsys):
    path = tmp_path / "station.yaml"
    robot = {"type": "franka", "arm_backend": "fake", "gripper": {"backend": "fake"}}
    path.write_text(yaml.safe_dump({"robots": {"left": dict(robot), "right": dict(robot, robot_ip="10.0.0.3")}}))
    server_versions = {"left": 10, "right": 9}

    def open_fails(self):
        server = server_versions[self.owner_robot_name]
        raise VersionMismatch(
            f"Incompatible library version (server version: {server}, library version: 8)", device=self.name
        )

    original_run_doctor = cli.run_doctor

    def run_doctor_tagging_arm(config, **kwargs):
        monkeypatch.setattr(FakeArm, "owner_robot_name", config.name, raising=False)
        return original_run_doctor(config, **kwargs)

    monkeypatch.setattr(FakeArm, "_open", open_fails)
    monkeypatch.setattr(cli, "run_doctor", run_doctor_tagging_arm)
    pip = FakeSubprocess(monkeypatch)
    Prompt(monkeypatch, None)
    assert cli.main(["--config", str(path), "--install-sdk", "--yes"]) == 1
    out = capsys.readouterr().out
    assert pip.calls == [] and "different robot server versions [9, 10]" in out
    assert "by-robot-server-version/10/" in out and "by-robot-server-version/9/" in out


def test_yes_needs_install_sdk(tmp_path, capsys):
    with pytest.raises(SystemExit) as info:
        cli.main(["--config", write_config(tmp_path), "--yes"])
    assert info.value.code == 2 and "--install-sdk" in capsys.readouterr().err


def test_spacemouse_extra_checks_hid_module(monkeypatch):
    assert deps.EXTRA_MODULES["spacemouse"] == ("hid",)
    config = FrankaConfig(arm_backend="fake", gripper=None, teleop=TeleopConfig(backend="spacemouse"))
    fake_find_spec(monkeypatch, missing={"hid"})
    outcome = deps.deps_outcome(config)
    assert not outcome.ok and "spacemouse" in outcome.message
    assert "pip install" in outcome.hint
    fake_find_spec(monkeypatch, missing={"pyspacemouse", "easyhid"})
    assert deps.extra_installed("spacemouse")
