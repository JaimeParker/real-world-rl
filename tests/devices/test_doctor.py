"""Hardware-free tests for the layered doctor; OS probes are monkeypatched."""
from __future__ import annotations

import json

import pytest
import yaml

from real_world_rl.devices import CameraConfig, DeviceFault, Gripper, GripperConfig
from real_world_rl.devices.claims import DeviceClaim, claim_key
from real_world_rl.devices.robot import Robot
from real_world_rl.devices.backends.arms.fake import FakeArm
from real_world_rl.devices.backends.cameras.fake import FakeCamera
from real_world_rl.devices.backends.grippers.fake import FakeGripper
from real_world_rl.devices.robots.franka import FrankaConfig
from real_world_rl.devices.robots.piper import PiperConfig
from real_world_rl.discovery import enumerate as probes
from real_world_rl.cli import main
from real_world_rl.doctor import checks
from real_world_rl.doctor.checks import CheckResult, check_rows, run_doctor
from real_world_rl.doctor.repair import exit_status
from real_world_rl.doctor.report import device_findings, station_device_findings


def franka_config(**kwargs) -> FrankaConfig:
    return FrankaConfig(
        arm_backend="fake",
        gripper=GripperConfig(backend="fake"),
        cameras=[CameraConfig(name="wrist", backend="fake", fps=60)],
        **kwargs,
    )


@pytest.fixture(autouse=True)
def reachable(monkeypatch):
    monkeypatch.setattr(probes, "ping", lambda ip, timeout_s=1.0: True)
    monkeypatch.setattr(probes, "tcp_open", lambda ip, port, timeout_s=0.5: False)
    monkeypatch.setattr(FakeCamera, "discover", classmethod(lambda cls: {"S1"}))


@pytest.fixture
def sdk_backed_fakes(monkeypatch):
    """Make the fakes look like SDK-backed hardware so the OS probes run (os is always importable)."""
    monkeypatch.setattr(FakeArm, "SDK", "os")
    monkeypatch.setattr(FakeCamera, "SDK", "os")


def by_layer(results, layer):
    return [result for result in results if result.layer == layer]


def exit_code(results) -> int:
    """The doctor exit status of one robot's device checks."""
    return exit_status(device_findings("robot", check_rows(results)), [])


def test_all_green(sdk_backed_fakes):
    results = run_doctor(franka_config())
    assert exit_code(results) == 0
    assert {result.layer for result in results} == {0, 1, 2, 3, 4, 5}
    assert all(result.ok and not result.skipped for result in results)
    names = {result.name for result in results}
    assert {"os:ping", "handshake", "feedback:arm", "feedback:gripper", "feedback:camera:wrist"} <= names
    assert "mode:none" in names


def test_gripper_feedback_reports_width():
    results = run_doctor(franka_config())
    feedback = next(result for result in results if result.name == "feedback:gripper")
    assert feedback.ok and not feedback.warning
    assert " m (" in feedback.message and "max_width_m" in feedback.message
    assert feedback.message == (
        "gripper width 0.0800 m (1.00 of max_width_m 0.080), "
        "grasped=unknown, source FakeGripper"
    )


@pytest.mark.parametrize(
    ("width", "max_width_m"),
    [
        pytest.param(float("nan"), 0.08, id="nan-width"),
        pytest.param(float("inf"), 0.08, id="infinite-width"),
        pytest.param(float("-inf"), 0.08, id="negative-infinite-width"),
        pytest.param(-0.1, 0.08, id="negative-width"),
        pytest.param(1.1, 0.08, id="width-above-one"),
        pytest.param(1.0, 0.0, id="zero-max-width"),
        pytest.param(1.0, -0.08, id="negative-max-width"),
        pytest.param(1.0, float("nan"), id="nan-max-width"),
        pytest.param(1.0, float("inf"), id="infinite-max-width"),
        pytest.param(1.0, float("-inf"), id="negative-infinite-max-width"),
    ],
)
def test_gripper_feedback_warns_on_untrustworthy_width(monkeypatch, width, max_width_m):
    class UntrustworthyGripper(FakeGripper):
        @property
        def width(self) -> float:
            return width

    monkeypatch.setitem(Gripper._BACKENDS, "fake", UntrustworthyGripper)
    config = franka_config()
    config.gripper.max_width_m = max_width_m
    results = run_doctor(config)
    feedback = next(result for result in results if result.name == "feedback:gripper")
    assert feedback.ok and feedback.warning
    assert feedback.hint
    assert "max_width_m" in feedback.hint and "SDK" in feedback.hint
    assert "source UntrustworthyGripper" in feedback.message
    assert exit_code(results) == 0


def test_ping_failure_does_not_skip_later_layers(monkeypatch, sdk_backed_fakes):
    monkeypatch.setattr(probes, "ping", lambda ip, timeout_s=1.0: False)
    results = run_doctor(franka_config())
    ping_result = next(result for result in results if result.name == "os:ping")
    assert not ping_result.ok and "Ethernet cable" in ping_result.hint
    assert exit_code(results) == 1
    assert all(r.ok and not r.skipped for layer in (3, 4, 5) for r in by_layer(results, layer))


def test_unknown_backend_skips_later_layers():
    config = franka_config()
    config.arm_backend = "nonexistent"
    results = run_doctor(config)
    sdk_arm = next(result for result in results if result.name == "sdk:arm")
    assert not sdk_arm.ok and "nonexistent" in sdk_arm.message and sdk_arm.hint
    assert all(r.skipped for layer in (3, 4, 5) for r in by_layer(results, layer))
    assert exit_code(results) == 1


def test_handshake_failure_skips_feedback_and_mode(monkeypatch):
    def open_fails(self):
        raise DeviceFault("fci down", hint="enable FCI", device="arm", recoverable=True)

    monkeypatch.setattr(FakeArm, "_open", open_fails)
    results = run_doctor(franka_config())
    handshake = by_layer(results, 3)[0]
    assert not handshake.ok and handshake.hint == "enable FCI"
    assert all(r.skipped for layer in (4, 5) for r in by_layer(results, layer))


def test_mock_piper_skips_os_and_passes():
    results = run_doctor(PiperConfig(), mock=True)
    assert all(r.skipped for r in by_layer(results, 2)) and by_layer(results, 2)
    assert all(r.message == "no SDK needed" for r in by_layer(results, 1))
    assert exit_code(results) == 0


def test_camera_without_frames_fails_and_points_at_rwrl_status(monkeypatch):
    monkeypatch.setattr(FakeCamera, "is_ready", lambda self, timeout_s=2.0: False)
    camera = next(r for r in run_doctor(franka_config()) if r.name == "feedback:camera:wrist")
    assert not camera.ok and "rerun rwrl status after reconnecting the camera" in camera.hint
    assert "--fix" not in camera.hint


def test_no_feedback_arm_fails_with_hint(monkeypatch):
    original = FakeArm._open

    def open_and_inject(self):
        self.inject("no_feedback")
        return original(self)

    monkeypatch.setattr(FakeArm, "_open", open_and_inject)
    feedback = next(r for r in run_doctor(franka_config()) if r.name == "feedback:arm")
    assert not feedback.ok and "follower/slave" in feedback.hint


def test_mock_piper_findings_have_no_failure_and_three_skips():
    statuses = [finding.status for finding in device_findings("piper", check_rows(run_doctor(PiperConfig(), mock=True)))]
    assert "FAIL" not in statuses and statuses.count("SKIPPED") == 3


def write_config(tmp_path) -> str:
    path = tmp_path / "robot.yaml"
    path.write_text(
        yaml.safe_dump(
            {
                "type": "franka",
                "arm_backend": "fake",
                "gripper": {"backend": "fake"},
                "cameras": [{"name": "wrist", "backend": "fake", "fps": 60}],
            }
        )
    )
    return str(path)


def test_rwrl_doctor_mock_text_and_json(tmp_path, capsys, doctor_report_dir):
    path = write_config(tmp_path)
    assert main(["doctor", "--station", path, "--mock"]) == 0
    text = capsys.readouterr().out
    assert ".handshake: robot connected" in text and "summary:" not in text and "[3/5]" not in text
    assert main(["--json", "doctor", "--station", path, "--mock"]) == 0
    payload = json.loads(capsys.readouterr().out)
    assert payload["schema_version"] == 1 and payload["status"] == "PASS"
    assert any(check["id"].endswith(".handshake") and check["status"] == "PASS" for check in payload["checks"])
    (rows,) = payload["results"].values()
    assert {entry["layer"] for entry in rows} == {0, 1, 2, 3, 4, 5}
    assert all("autofix" not in entry for entry in rows)


def write_station(tmp_path) -> str:
    robot = {"type": "franka", "arm_backend": "fake", "gripper": {"backend": "fake"}}
    path = tmp_path / "station.yaml"
    path.write_text(yaml.safe_dump({"robots": {"left": dict(robot), "right": dict(robot, robot_ip="10.0.0.3")}}))
    return str(path)


def test_rwrl_doctor_without_station_flag_checks_the_station_file(tmp_path, monkeypatch, capsys, doctor_report_dir):
    monkeypatch.setenv("RWRL_STATION", write_config(tmp_path))
    assert main(["doctor", "--mock"]) == 0
    assert ".handshake: robot connected" in capsys.readouterr().out


def test_station_device_findings_cover_every_robot_or_one(tmp_path):
    path = write_station(tmp_path)
    findings = station_device_findings(path, mock=True)
    assert {finding.id.split(".")[0] for finding in findings} == {"left", "right"}
    assert exit_status(findings, []) == 0
    one = station_device_findings(path, mock=True, robot="right")
    assert one and all(finding.id.startswith("right.") for finding in one)


def test_station_exit_status_fails_when_one_robot_fails(tmp_path, monkeypatch):
    path = write_station(tmp_path)
    monkeypatch.setattr(
        checks, "run_doctor", lambda config, **_: [CheckResult(1, "sdk:arm", config.name == "left", "checked")]
    )
    findings = station_device_findings(path)
    assert exit_status(findings, []) == 1
    assert [finding.id for finding in findings if finding.status == "FAIL"] == ["right.sdk:arm"]


def test_fake_backends_skip_os_probes_and_pass(monkeypatch):
    monkeypatch.setattr(probes, "ping", lambda ip, timeout_s=1.0: False)
    results = run_doctor(franka_config(robot_ip="10.0.0.2"))
    assert exit_code(results) == 0
    os_results = {r.name: r for r in by_layer(results, 2)}
    assert {"os:ping", "os:tcp", "os:camera:wrist"} <= set(os_results)
    assert all(r.skipped and r.message == "backend 'fake' needs no OS device" for r in os_results.values())


def test_station_device_findings_fill_robot_ip_from_env(tmp_path, monkeypatch, sdk_backed_fakes):
    pinged = []
    monkeypatch.setattr(probes, "ping", lambda ip, timeout_s=1.0: pinged.append(ip) or True)
    monkeypatch.setenv("ROBOT_IP", "10.9.8.7")
    path = tmp_path / "robot.yaml"
    path.write_text(yaml.safe_dump({"type": "franka", "arm_backend": "fake", "robot_ip": "", "gripper": {"backend": "fake"}}))
    assert exit_status(station_device_findings(str(path)), []) == 0
    assert pinged == ["10.9.8.7"]


def test_can_hints_fill_both_halves(monkeypatch, sdk_backed_fakes):
    monkeypatch.setattr(probes, "list_can_interfaces", lambda: {})
    results = run_doctor(PiperConfig(arm_backend="fake", can_interface="can3", bitrate=500000, gripper=None))
    hint = next(r for r in results if r.name == "os:can_exists").hint
    assert "<iface>" not in hint and "<bitrate>" not in hint
    assert "bash can_activate.sh can3 500000" in hint and "ip link set can3 up type can bitrate 500000" in hint


def test_camera_check_messages_with_unset_serial(monkeypatch, sdk_backed_fakes):
    def camera_result():
        return next(r for r in run_doctor(franka_config()) if r.name == "os:camera:wrist")

    monkeypatch.setattr(FakeCamera, "discover", classmethod(lambda cls: set()))
    none_found = camera_result()
    assert not none_found.ok and none_found.message == "no RealSense device found (serial unset)"
    monkeypatch.setattr(FakeCamera, "discover", classmethod(lambda cls: {"A1", "B2"}))
    several = camera_result()
    assert several.message == "serial unset and 2 devices attached; set serial to one of ['A1', 'B2']"


@pytest.fixture
def claim_dir(tmp_path, monkeypatch):
    import real_world_rl.devices.claims as claims

    monkeypatch.setattr(claims, "_DEFAULT_LOCK_DIR", tmp_path / "claims")


def test_doctor_fails_handshake_when_robot_is_claimed(claim_dir):
    config = franka_config()
    with DeviceClaim(claim_key(Robot.from_config(config))):
        results = run_doctor(config)
    handshake = next(r for r in results if r.name == "handshake")
    assert not handshake.ok and "claimed by another process" in handshake.message
    assert "claim: false" in handshake.hint
    assert exit_code(results) == 1


def test_doctor_releases_claim_afterwards(claim_dir):
    config = franka_config()
    assert exit_code(run_doctor(config)) == 0
    with DeviceClaim(claim_key(Robot.from_config(config))):
        pass


def test_mock_doctor_covers_teleop_and_skips_its_os_check():
    from real_world_rl.devices import TeleopConfig

    config = PiperConfig(teleop=TeleopConfig(name="sm", backend="spacemouse", options={"device_index": 0}))
    results = run_doctor(config, mock=True)
    names = {r.name: r for r in results}
    assert names["sdk:teleop:fake"].ok and names["sdk:teleop:fake"].message == "no SDK needed"
    assert names["os:teleop:sm"].skipped
    assert names["feedback:teleop:sm"].ok and not names["feedback:teleop:sm"].skipped
    assert exit_code(results) == 0


def test_fake_teleop_needs_no_os_device():
    from real_world_rl.devices import TeleopConfig

    results = run_doctor(franka_config(teleop=TeleopConfig(backend="fake")))
    os_teleop = next(r for r in results if r.name == "os:teleop:teleop")
    assert os_teleop.skipped and os_teleop.message == "backend 'fake' needs no OS device"
    assert next(r for r in results if r.name == "feedback:teleop:teleop").ok
    assert exit_code(results) == 0


def test_unknown_teleop_backend_fails_sdk_layer():
    from real_world_rl.devices import TeleopConfig

    results = run_doctor(franka_config(teleop=TeleopConfig(backend="nonexistent")))
    sdk = next(r for r in results if r.name == "sdk:teleop:nonexistent")
    assert not sdk.ok and "nonexistent" in sdk.message
    assert exit_code(results) == 1


def test_spacemouse_sdk_check_reports_missing_hidapi_and_wrong_distribution(monkeypatch):
    import sys
    import types

    from real_world_rl.teleop.spacemouse.backend import SpaceMouseTeleop
    from real_world_rl.doctor import checks

    monkeypatch.setitem(sys.modules, "hid", None)
    outcome = checks._sdk_outcome(SpaceMouseTeleop)
    assert outcome[0] is False and "hidapi" in outcome[1]
    assert outcome[2] == "pip install -e '.[spacemouse]'"
    monkeypatch.setitem(sys.modules, "hid", types.SimpleNamespace(Device=lambda: None))
    outcome = checks._sdk_outcome(SpaceMouseTeleop)
    assert outcome[0] is False and "wrong distribution 'hid'" in outcome[1]


def test_spacemouse_sdk_check_distinguishes_native_load_failure(monkeypatch):
    from real_world_rl.teleop.spacemouse import hid as _spacemouse_hid
    from real_world_rl.teleop.spacemouse.backend import SpaceMouseTeleop
    from real_world_rl.doctor import checks

    def fail(name):
        raise OSError("native library unavailable")

    monkeypatch.setattr(_spacemouse_hid.importlib, "import_module", fail)
    outcome = checks._sdk_outcome(SpaceMouseTeleop)
    assert outcome[0] is False and "cannot load hidapi" in outcome[1]
    assert "wheel" in outcome[2] and "udev" not in outcome[2]
