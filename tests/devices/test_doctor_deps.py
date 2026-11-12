"""Doctor layer 0 (extras), the libfranka mismatch parser and the handshake report that
``rwrl doctor --install-sdk`` acts on; ``importlib.util.find_spec`` is faked."""
from __future__ import annotations

import importlib.util
import types

import pytest
import yaml

from real_world_rl.devices import DeviceNotFound, GripperConfig, TeleopConfig, VersionMismatch
from real_world_rl.devices.backends.arms.fake import FakeArm
from real_world_rl.devices.backends.arms.franky import (
    franky_connect_error,
    parse_libfranka_mismatch,
)
from real_world_rl.devices.backends.grippers.fake import FakeGripper
from real_world_rl.devices.robots.franka import FrankaConfig
from real_world_rl.discovery import enumerate as probes
from real_world_rl.doctor import deps
from real_world_rl.doctor.report import station_device_findings
from real_world_rl.doctor.checks import LIBFRANKA_SERVER_VERSION, run_doctor

MISMATCH_MESSAGE = (
    "libfranka/robot firmware mismatch: libfranka: Incompatible library version (server version: 10, "
    "library version: 9). Please check https://frankaemika.github.io for Robot system compatibility with libfranka."
)


@pytest.fixture(autouse=True)
def no_network(monkeypatch, tmp_path):
    import real_world_rl.devices.claims as claims

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
    assert outcome.hint.startswith("pip install -e '.[franky,gello]' (from the repository root)")
    assert "--install-sdk" in outcome.hint


def test_missing_grpc_fails_only_for_a_declared_daemon(monkeypatch):
    fake_find_spec(monkeypatch, missing={"grpc"})
    served = deps.deps_outcome(gello_station(), serves_daemon=True)
    assert not served.ok and served.message == "missing extras for this station: grpc"
    assert served.hint.startswith("pip install -e '.[grpc]'")
    single_process = deps.deps_outcome(gello_station())
    assert single_process.ok and single_process.warning
    assert single_process.message == "extras ok: franky, gello; optional, not installed: grpc (daemon route)"
    assert "'.[grpc]'" in single_process.hint


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
    assert any(not result.ok and not result.skipped for result in real)


def test_mock_never_requires_grpc(monkeypatch):
    fake_find_spec(monkeypatch, missing={"grpc"})
    deps_result = run_doctor(gello_station(), mock=True, serves_daemon=True)[0]
    assert deps_result.ok and not deps_result.warning


def write_station_yaml(tmp_path, robot: dict) -> str:
    path = tmp_path / "station.yaml"
    path.write_text(yaml.safe_dump(robot))
    return str(path)


def test_station_findings_require_grpc_only_with_a_daemon_block(tmp_path, monkeypatch):
    fake_find_spec(monkeypatch, missing={"grpc"})
    monkeypatch.setattr(deps, "ARM_BACKEND_EXTRAS", {"fake": "franky"})  # a fake arm that "needs" an extra
    robot = {"type": "franka", "arm_backend": "fake", "gripper": {"backend": "fake"}}
    single = station_device_findings(write_station_yaml(tmp_path, robot))
    (single_deps,) = [finding for finding in single if finding.id.endswith(".deps")]
    assert single_deps.status == "WARN" and "grpc (daemon route)" in single_deps.message
    served = station_device_findings(write_station_yaml(tmp_path, dict(robot, daemon={"bind": "0.0.0.0:50051"})))
    (served_deps,) = [finding for finding in served if finding.id.endswith(".deps")]
    assert served_deps.status == "FAIL" and served_deps.message == "missing extras for this station: grpc"


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
    from real_world_rl.doctor import checks

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
        "then: rwrl doctor --station <cfg> --install-sdk"
    )
    assert handshake.details[LIBFRANKA_SERVER_VERSION] == 10


def test_handshake_hint_names_the_config_path(arm_mismatch):
    results = run_doctor(fake_station_config(), config_path="/etc/robot.yaml")
    handshake = next(r for r in results if r.name == "handshake")
    assert handshake.hint.endswith("rwrl doctor --station /etc/robot.yaml --install-sdk")


def test_gripper_mismatch_keeps_the_backend_message(monkeypatch):
    def open_fails(self):
        raise VersionMismatch(MISMATCH_MESSAGE, hint="backend hint", device=self.name)

    monkeypatch.setattr(FakeGripper, "_open", open_fails)
    handshake = next(r for r in run_doctor(fake_station_config()) if r.name == "handshake")
    assert handshake.message == MISMATCH_MESSAGE and handshake.hint == "backend hint"
    assert not handshake.details


def test_spacemouse_extra_checks_hid_module(monkeypatch):
    assert deps.EXTRA_MODULES["spacemouse"] == ("hid",)
    config = FrankaConfig(arm_backend="fake", gripper=None, teleop=TeleopConfig(backend="spacemouse"))
    fake_find_spec(monkeypatch, missing={"hid"})
    outcome = deps.deps_outcome(config)
    assert not outcome.ok and "spacemouse" in outcome.message
    assert "pip install" in outcome.hint
    fake_find_spec(monkeypatch, missing={"pyspacemouse", "easyhid"})
    assert deps.extra_installed("spacemouse")
