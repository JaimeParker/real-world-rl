"""The ZED camera backends in the tooling around the device layer: doctor (deps, USB, SDK checks,
standalone USB read), setup (probe, wizard defaults, station rendering) and ownership."""
from __future__ import annotations

import os
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml

from real_world_rl.devices import Camera, CameraConfig
from real_world_rl.devices.backends.cameras.realsense import RealSenseCamera
from real_world_rl.devices.backends.cameras.zed import ZedCamera
from real_world_rl.devices.backends.cameras.zed_uvc import ZedUvcCamera
from real_world_rl.devices.robots.franka import FrankaConfig
from real_world_rl.doctor import deps, report as doctor_report
from real_world_rl.doctor import host as doctor_host
from real_world_rl.doctor.host import Host
from real_world_rl.doctor.repair import Report
from real_world_rl.setup import probes, wizard
from real_world_rl.setup.render import check_rendered_station, render_station_yaml
from real_world_rl.setup.wizard import ArmDraft, StationDraft


def zed_station(backend: str, **camera) -> FrankaConfig:
    return FrankaConfig(
        arm_backend="fake", gripper=None, cameras=[CameraConfig(name="wrist", backend=backend, **camera)]
    )


# -- doctor: deps ---------------------------------------------------------------------------------


def test_camera_extras_come_from_the_backend_declarations():
    assert deps.CAMERA_BACKEND_EXTRAS == {"realsense": "realsense", "zed": "zed-sdk", "zed_uvc": "zed-uvc"}
    assert deps.EXTRA_MODULES["zed-sdk"] == ("pyzed.sl",) and deps.EXTRA_MODULES["zed-uvc"] == ("cv2",)
    assert deps.required_extras(zed_station("zed")) == ["zed-sdk"]
    assert deps.required_extras(zed_station("zed_uvc")) == ["zed-uvc"]
    assert deps.required_extras(zed_station("fake")) == []


def test_a_missing_zed_sdk_points_to_the_zed_installer_not_pip(monkeypatch):
    monkeypatch.setattr(deps, "_importable", lambda module: module not in ("pyzed.sl", "pyrealsense2"))
    outcome = deps.deps_outcome(zed_station("zed"))
    assert not outcome.ok and outcome.message == "missing extras for this station: zed-sdk"
    assert "pip install" not in outcome.hint and "get_python_api.py" in outcome.hint
    mixed = deps.install_extras_hint(["realsense", "zed-sdk"])
    assert mixed.startswith("pip install -e '.[realsense]' (from the repository root); zed-sdk: install the ZED SDK")


# -- doctor: host -----------------------------------------------------------------------------------


@pytest.fixture
def report(tmp_path, monkeypatch):
    monkeypatch.setenv("RWRL_DOCTOR_REPORT_DIR", str(tmp_path / "reports"))
    return Report(tmp_path)


def usb_device(sysfs: Path, name: str, vid: str, pid: str, product: str, devnum: str = "5") -> None:
    usb = sysfs / "bus/usb/devices" / name
    usb.mkdir(parents=True)
    for key, value in {"idVendor": vid, "idProduct": pid, "product": product, "busnum": "1", "devnum": devnum}.items():
        (usb / key).write_text(value)


def test_usb_discovery_marks_zed_video_interfaces_only(tmp_path, report):
    sysfs = tmp_path / "sys"
    usb_device(sysfs, "1-1", "2b03", "f582", "ZED", "5")
    usb_device(sysfs, "1-2", "2b03", "f881", "ZED-2i HID INTERFACE", "6")
    usb_device(sysfs, "1-3", "8086", "0b07", "Intel(R) RealSense(TM) Depth Camera 435", "7")
    host = Host(tmp_path, report, sysfs=sysfs, dev=tmp_path / "dev", etc=tmp_path / "etc")
    assert [d["kind"] for d in host.usb] == ["zed", "", "realsense"]
    assert {kind: [name for name, _ in backends] for kind, backends in host.camera_usb.items()} == {
        "realsense": ["realsense"],
        "zed": ["zed", "zed_uvc"],
    }


def test_an_expected_zed_without_usb_device_is_blocked(tmp_path, report):
    host = Host(tmp_path, report, sysfs=tmp_path / "sys", dev=tmp_path / "dev", etc=tmp_path / "etc")
    host.expected.add("zed")
    host.inspect_usb()
    row = next(row for row in host.findings if row.id == "usb.zed")
    assert row.status == "BLOCKED" and row.message == "no supported zed USB device attached"


def test_a_denied_zed_node_plans_udev_rules_for_its_ids(tmp_path, report, monkeypatch):
    sysfs = tmp_path / "sys"
    usb_device(sysfs, "1-1", "2b03", "f880", "ZED 2i")
    host = Host(tmp_path, report, sysfs=sysfs, dev=tmp_path / "dev", etc=tmp_path / "etc")
    monkeypatch.setattr(host, "command", lambda *a, **kw: (1, ""))
    host.expected.add("zed")
    host.inspect_usb()
    assert next(row for row in host.findings if row.id == "usb.zed").status == "FAIL"
    rules = next(action for action in host.actions if action.target.endswith("99-rwrl-zed.rules"))
    assert 'SUBSYSTEM=="video4linux", ATTRS{idVendor}=="2b03", ATTRS{idProduct}=="f880"' in rules.content


def test_sdk_checks_and_install_hints_cover_the_zed_backends():
    checks = doctor_host.sdk_checks()
    assert checks["zed-sdk"] == ("pyzed.sl", "print('import OK')", [])
    assert checks["zed-uvc"] == ("cv2", "print('import OK')", ["opencv-python-headless"])
    assert set(doctor_host.manual_sdk_hints()) == {"zed-sdk"}


def test_a_failed_zed_sdk_import_carries_the_installer_hint(tmp_path, report, monkeypatch):
    host = Host(tmp_path, report, sysfs=tmp_path / "sys", dev=tmp_path / "dev", etc=tmp_path / "etc")
    host.expected.add("zed-sdk")
    monkeypatch.setattr(host, "command", lambda argv, timeout=15: (1, "ModuleNotFoundError: No module named 'pyzed'"))
    host.inspect_dependencies()
    row = next(row for row in host.findings if row.id == "sdk.zed-sdk")
    assert row.status == "FAIL" and "get_python_api.py" in row.hint
    assert not any(action.check == "sdk.zed-sdk" for action in host.actions)  # nothing pip can repair


def test_the_unconfigured_zed_read_uses_the_selected_or_installed_backend(tmp_path, report, monkeypatch):
    host = Host(tmp_path, report, sysfs=tmp_path / "sys", dev=tmp_path / "dev", etc=tmp_path / "etc")
    monkeypatch.setattr(deps, "extra_installed", lambda extra: extra == "zed-uvc")
    assert doctor_report._usb_read_camera_backend(host, "zed") == ("zed_uvc", ZedUvcCamera)
    host.expected.add("zed-sdk")
    assert doctor_report._usb_read_camera_backend(host, "zed") == ("zed", ZedCamera)
    monkeypatch.setattr(deps, "extra_installed", lambda extra: False)
    host.expected.clear()
    assert doctor_report._usb_read_camera_backend(host, "zed") is None


def test_backend_usb_declarations():
    assert ZedCamera.usb_matches(0x2B03, 0xF582, "") and not ZedCamera.usb_matches(0x2B03, 0xF881, "")
    assert RealSenseCamera.usb_matches(0x8086, 0x0B07, "Intel RealSense D435")
    assert not RealSenseCamera.usb_matches(0x8086, 0x1234, "Bluetooth")
    assert not ZedCamera.USB_SERIAL_IS_DEVICE_SERIAL and ZedUvcCamera.USB_SERIAL_IS_DEVICE_SERIAL
    assert Camera.backend("fake").doctor_extra("fake") is None


def test_a_process_with_pyzed_loaded_owns_the_device(tmp_path):
    from real_world_rl.common.device_ownership import process_owners

    proc = tmp_path / "proc" / "4242"
    proc.mkdir(parents=True)
    (proc / "cmdline").write_bytes(b"python\0my_viewer.py\0")
    (proc / "maps").write_text("7f00 r-xp /opt/venv/lib/python3.10/site-packages/pyzed/sl.so\n")
    assert process_owners(proc=tmp_path / "proc") == ["pid 4242: loaded hardware SDK"]


# -- setup -------------------------------------------------------------------------------------------


def test_probe_cameras_lists_every_hardware_backend(monkeypatch):
    found = {"realsense": {"RS1"}, "zed": {"31234567"}, "zed_uvc": {"OV0001"}}
    models = {"realsense": {"RS1": "D400"}, "zed": {"31234567": "ZED"}, "zed_uvc": {"OV0001": "ZED"}}
    for name, backend in (("realsense", RealSenseCamera), ("zed", ZedCamera), ("zed_uvc", ZedUvcCamera)):
        monkeypatch.setattr(backend, "discover", classmethod(lambda cls, name=name: found[name]))
        monkeypatch.setattr(backend, "describe", classmethod(lambda cls, name=name: models[name]))
    assert probes.probe_cameras() == [
        probes.CameraProbe("RS1", "D400", "realsense"),
        probes.CameraProbe("31234567", "ZED", "zed"),
        probes.CameraProbe("OV0001", "ZED", "zed_uvc"),
    ]
    assert probes.probe_realsense() == [probes.CameraProbe("RS1", "D400", "realsense")]
    text = probes.StationProbe(franka_hosts=[], cameras=probes.probe_cameras(), gello_buses=[], spacemice=[]).format()
    assert "  zed: 31234567 (ZED)" in text


def test_a_new_zed_camera_gets_a_16_9_stream():
    camera = wizard._camera_config("31234567", "wrist", [], {}, "zed")
    assert (camera.backend, camera.width, camera.height, camera.fps) == ("zed", 640, 360, 30)
    assert wizard._camera_config("RS1", "wrist", [], {}, "realsense") == CameraConfig(name="wrist", serial="RS1")
    hd = wizard._camera_config("OV0001", "front", [], {"width": 1280, "height": 720, "fps": 60}, "zed_uvc")
    assert (hd.backend, hd.width, hd.height, hd.fps) == ("zed_uvc", 1280, 720, 60)


@pytest.mark.parametrize(
    "settings, backend, message",
    [
        ({"width": 640, "height": 480}, "zed", "without stretching"),
        ({"fps": 100, "width": 1280, "height": 720}, "zed", "100 fps"),
        ({"enable_depth": True}, "zed_uvc", "no depth"),
    ],
)
def test_the_wizard_rejects_a_stream_a_zed_cannot_deliver(settings, backend, message):
    with pytest.raises(ValueError, match=message):
        wizard._camera_config("S", "wrist", [], settings, backend)


def test_a_zed_station_renders_and_reloads_with_its_depth_mode():
    config = FrankaConfig(
        name="franka",
        cameras=[
            CameraConfig(name="wrist", backend="zed", serial="31234567", width=640, height=360,
                         enable_depth=True, depth_mode="NEURAL"),
            CameraConfig(name="front", backend="zed_uvc", serial="OV0001", width=1280, height=720),
        ],
    )
    draft = StationDraft([ArmDraft("franka", config)])
    text = render_station_yaml(draft)
    check_rendered_station(text, draft)
    cameras = yaml.safe_load(text)["cameras"]
    assert [(c["backend"], c["depth_mode"]) for c in cameras] == [("zed", "NEURAL"), ("zed_uvc", "")]
