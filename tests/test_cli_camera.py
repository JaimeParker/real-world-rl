"""Hardware-free `rwrl camera show|grab` coverage with scripted cameras and the fake backend."""
from __future__ import annotations

from contextlib import contextmanager
import json
import socket
import sys
from types import SimpleNamespace

import numpy as np
import pytest
import yaml

from real_world_rl.cli import connect, main
from robot_infra.devices.base import Health
from robot_infra.devices.camera import CameraConfig, Frame
from robot_infra.devices.errors import DeviceFault

HEIGHT, WIDTH = 4, 6


@pytest.fixture(autouse=True)
def offline(monkeypatch):
    def forbidden(*args, **kwargs):
        raise AssertionError("CLI tests must not open sockets")

    monkeypatch.setattr(socket, "create_connection", forbidden)
    monkeypatch.setattr(sys.stdin, "isatty", lambda: False)


class ScriptedCamera:
    """Returns fixed frames stamped from ``timestamps``; reads past ``frames_available`` raise
    the no-frame ``DeviceFault`` a real camera raises."""

    def __init__(self, name, *, timestamps=(0.0, 0.1, 0.2, 0.3), depth=None, serial="1234",
                 frames_available=None, health=None, image_encoding=None):
        self.config = CameraConfig(name=name, serial=serial, width=WIDTH, height=HEIGHT, fps=30,
                                   enable_depth=depth is not None, backend="scripted")
        self.color = np.arange(HEIGHT * WIDTH * 3, dtype=np.uint8).reshape(HEIGHT, WIDTH, 3)
        self.depth = depth
        self.timestamps = list(timestamps)
        self.frames_available = frames_available
        self.reported_health = health or Health(ok=True, detail="", last_feedback_age_s=0.012)
        self.image_encoding = image_encoding
        self.read_timeouts = []

    def read(self, timeout_s=None):
        self.read_timeouts.append(timeout_s)
        reads = len(self.read_timeouts)
        if self.frames_available is not None and reads > self.frames_available:
            raise DeviceFault(f"camera '{self.config.name}' produced no frame within 2.0s",
                              hint="check the USB cable", device=self.config.name, recoverable=True)
        timestamp = self.timestamps[min(reads, len(self.timestamps)) - 1]
        return Frame(color=self.color, depth=self.depth, timestamp=timestamp)

    def health(self):
        return self.reported_health


@pytest.fixture
def cameras(monkeypatch):
    """Scripted cameras behind the rwrl connection factory; ``opened`` counts connections."""
    state = SimpleNamespace(cameras={}, remote=False, opened=0)

    @contextmanager
    def open_robot(args):
        state.opened += 1
        yield SimpleNamespace(cameras=state.cameras, remote=state.remote,
                              path="daemon 127.0.0.1:0" if state.remote else "direct scripted")

    monkeypatch.setattr(connect, "open_robot", open_robot)
    return state


def run(tmp_path, capsys, *command, as_json=True):
    argv = ["--station", str(tmp_path / "station.yaml"), *(["--json"] if as_json else []), "camera", *command]
    code = main(argv)
    captured = capsys.readouterr()
    return code, json.loads(captured.out) if as_json else captured.out, captured.err


def test_show_json_reports_configuration_measured_rate_and_health(tmp_path, capsys, cameras):
    wrist = ScriptedCamera("wrist")
    front = ScriptedCamera("front", serial="", depth=np.ones((HEIGHT, WIDTH), dtype=np.float32),
                           image_encoding={"jpeg": 90})
    cameras.cameras.update(wrist=wrist, front=front)
    code, payload, _ = run(tmp_path, capsys, "show", "--frames", "4")
    assert code == 0
    assert set(payload) == {"command", "path", "cameras"}
    assert payload["command"] == "camera show"
    assert payload["path"] == "direct scripted"
    first, second = payload["cameras"]
    assert first == {
        "name": "wrist", "serial": "1234", "width": WIDTH, "height": HEIGHT, "fps": 30, "enable_depth": False,
        "backend": "scripted", "source": "direct", "image_encoding": None, "frames_read": 4,
        "measured_fps": pytest.approx(10.0), "frame_age_s": 0.012, "color_shape": [HEIGHT, WIDTH, 3],
        "depth_shape": None, "dtype": "uint8", "ok": True, "detail": "",
    }
    assert second["name"] == "front"
    assert second["depth_shape"] == [HEIGHT, WIDTH]
    assert second["image_encoding"] == {"jpeg": 90}
    assert wrist.read_timeouts == [None] * 4


def test_show_text_is_one_line_per_camera(tmp_path, capsys, cameras):
    cameras.cameras["wrist"] = ScriptedCamera("wrist")
    code, out, _ = run(tmp_path, capsys, "show", "--frames", "4", as_json=False)
    assert code == 0
    assert out.splitlines() == [
        "connection: direct scripted",
        "wrist: 6x4@30 fps (measured 10.0), age 0.012 s, color (4, 6, 3) uint8, depth none, serial 1234, ok",
    ]


def test_show_rate_ignores_repeated_frames_and_defaults_to_thirty_reads(tmp_path, capsys, cameras):
    cameras.cameras["wrist"] = camera = ScriptedCamera("wrist", timestamps=(1.0, 1.0, 1.5, 1.5, 2.0))
    code, payload, _ = run(tmp_path, capsys, "show")
    assert code == 0
    assert len(camera.read_timeouts) == 30
    assert payload["cameras"][0]["measured_fps"] == pytest.approx(2.0)


def test_show_frames_zero_does_not_read(tmp_path, capsys, cameras):
    # Unhealthy on purpose: without reads, health is not judged and cannot fail the command.
    stale = Health(ok=False, detail="no frame yet", last_feedback_age_s=None)
    cameras.cameras["wrist"] = camera = ScriptedCamera("wrist", health=stale)
    code, payload, _ = run(tmp_path, capsys, "show", "--frames", "0")
    assert code == 0
    assert camera.read_timeouts == []
    entry = payload["cameras"][0]
    assert (entry["frames_read"], entry["measured_fps"], entry["color_shape"], entry["dtype"]) == (0, None, None, None)
    assert (entry["ok"], entry["detail"], entry["frame_age_s"]) == (None, "not measured", None)
    assert "error" not in entry
    code, out, _ = run(tmp_path, capsys, "show", "--frames", "0", as_json=False)
    assert code == 0
    assert out.splitlines()[1] == ("wrist: 6x4@30 fps (not measured), age n/a, color n/a, depth n/a, "
                                   "serial 1234, not measured")
    assert "failed cameras" not in out


def test_show_name_selects_one_camera_and_unknown_name_lists_available(tmp_path, capsys, cameras):
    cameras.cameras.update(wrist=ScriptedCamera("wrist"), front=ScriptedCamera("front"))
    code, payload, _ = run(tmp_path, capsys, "show", "--name", "front", "--frames", "2")
    assert code == 0
    assert [entry["name"] for entry in payload["cameras"]] == ["front"]
    code, payload, err = run(tmp_path, capsys, "show", "--name", "side")
    assert code == 2
    assert payload["error"] == "no camera named 'side'; available: wrist, front"
    assert "error: no camera named 'side'" in err


def test_show_without_frames_fails_but_reports_every_camera(tmp_path, capsys, cameras):
    cameras.cameras.update(wrist=ScriptedCamera("wrist", frames_available=0), front=ScriptedCamera("front"))
    code, payload, _ = run(tmp_path, capsys, "show", "--frames", "3")
    assert code == 1
    wrist, front = payload["cameras"]
    assert wrist["error"] == "camera 'wrist' produced no frame within 2.0s"
    assert wrist["hint"] == "check the USB cable"
    assert (wrist["frames_read"], wrist["color_shape"]) == (0, None)
    assert "error" not in front
    assert front["frames_read"] == 3
    code, out, _ = run(tmp_path, capsys, "show", "--frames", "3", as_json=False)
    assert code == 1
    lines = out.splitlines()
    assert "error: camera 'wrist' produced no frame within 2.0s (hint: check the USB cable)" in lines[1]
    assert lines[2].endswith(", ok")
    assert lines[-1] == "failed cameras: wrist"


def test_show_unhealthy_camera_exits_one(tmp_path, capsys, cameras):
    stale = Health(ok=False, detail="capture stalled", last_feedback_age_s=0.5)
    cameras.cameras["wrist"] = ScriptedCamera("wrist", health=stale)
    code, payload, _ = run(tmp_path, capsys, "show", "--frames", "2")
    assert code == 1
    assert (payload["cameras"][0]["ok"], payload["cameras"][0]["detail"]) == (False, "capture stalled")
    assert "error" not in payload["cameras"][0]
    code, out, _ = run(tmp_path, capsys, "show", "--frames", "2", as_json=False)
    assert out.splitlines()[1].endswith("serial 1234, not ok (capture stalled)")


def test_show_marks_daemon_frames(tmp_path, capsys, cameras):
    cameras.remote = True
    cameras.cameras["wrist"] = ScriptedCamera("wrist")
    code, payload, _ = run(tmp_path, capsys, "show", "--frames", "2")
    assert code == 0
    assert payload["path"] == "daemon 127.0.0.1:0"
    assert payload["cameras"][0]["source"] == "daemon"


@pytest.mark.parametrize("command", [["show", "--frames", "-1"], ["grab", "--name", "wrist", "--output", "out",
                                                                   "--timeout-s", "0"]])
def test_invalid_numbers_are_usage_errors_before_connecting(tmp_path, capsys, cameras, command):
    code, payload, _ = run(tmp_path, capsys, *command)
    assert code == 2
    assert "requires" in payload["error"]
    assert cameras.opened == 0


def test_grab_writes_png_files_and_sidecar_then_refuses_to_overwrite(tmp_path, capsys, cameras):
    image = pytest.importorskip("PIL.Image")
    depth = np.full((HEIGHT, WIDTH), 0.5, dtype=np.float32)
    depth[0, 0] = np.nan
    cameras.cameras["wrist"] = camera = ScriptedCamera("wrist", depth=depth, timestamps=(12.5,))
    output = tmp_path / "grabs" / "first"
    code, payload, _ = run(tmp_path, capsys, "grab", "--name", "wrist", "--output", str(output), "--timeout-s", "1.5")
    assert code == 0
    color_path, depth_path, metadata_path = (output / "wrist-color.png", output / "wrist-depth.png",
                                             output / "wrist.json")
    assert payload == {"command": "camera grab", "path": "direct scripted", "camera": "wrist", "output": str(output),
                       "files": [str(color_path), str(depth_path), str(metadata_path)], "ok": True, "detail": ""}
    assert camera.read_timeouts == [1.5]
    np.testing.assert_array_equal(np.asarray(image.open(color_path)), camera.color)
    written_depth = np.asarray(image.open(depth_path)).astype(np.int64)
    expected_depth = np.full((HEIGHT, WIDTH), 500)
    expected_depth[0, 0] = 0
    np.testing.assert_array_equal(written_depth, expected_depth)
    metadata = json.loads(metadata_path.read_text())
    assert metadata["camera"] == {"name": "wrist", "serial": "1234", "width": WIDTH, "height": HEIGHT, "fps": 30,
                                  "enable_depth": True, "backend": "scripted"}
    assert metadata["station"] == str((tmp_path / "station.yaml").absolute())
    assert (metadata["source"], metadata["path"], metadata["timestamp"]) == ("direct", "direct scripted", 12.5)
    assert (metadata["color_shape"], metadata["depth_shape"], metadata["depth_dtype"]) == (
        [HEIGHT, WIDTH, 3], [HEIGHT, WIDTH], "float32")
    assert (metadata["depth_png_unit"], metadata["depth_png_scale_m"]) == ("mm", 0.001)
    assert metadata["files"] == payload["files"]

    before = {path: path.read_bytes() for path in (color_path, depth_path, metadata_path)}
    code, payload, _ = run(tmp_path, capsys, "grab", "--name", "wrist", "--output", str(output))
    assert code == 2
    assert payload["error"].startswith("refusing to overwrite ")
    assert payload["hint"] == "choose another --output directory"
    assert cameras.opened == 1
    assert {path: path.read_bytes() for path in before} == before


def test_grab_text_without_depth_writes_color_and_sidecar_only(tmp_path, capsys, cameras):
    pytest.importorskip("PIL.Image")
    cameras.cameras["wrist"] = ScriptedCamera("wrist")
    output = tmp_path / "out"
    code, out, _ = run(tmp_path, capsys, "grab", "--name", "wrist", "--output", str(output), as_json=False)
    assert code == 0
    assert sorted(path.name for path in output.iterdir()) == ["wrist-color.png", "wrist.json"]
    assert out.splitlines()[1] == f"camera wrist: wrote {output / 'wrist-color.png'}, {output / 'wrist.json'}"
    metadata = json.loads((output / "wrist.json").read_text())
    assert (metadata["depth_shape"], metadata["depth_png_unit"]) == (None, None)


def test_grab_with_unhealthy_camera_still_succeeds_and_warns(tmp_path, capsys, cameras):
    pytest.importorskip("PIL.Image")
    stale = Health(ok=False, detail="capture stalled", last_feedback_age_s=0.5)
    cameras.cameras["wrist"] = ScriptedCamera("wrist", health=stale)
    code, payload, _ = run(tmp_path, capsys, "grab", "--name", "wrist", "--output", str(tmp_path / "json"))
    assert code == 0
    assert (payload["ok"], payload["detail"]) == (False, "capture stalled")
    metadata = json.loads((tmp_path / "json" / "wrist.json").read_text())
    assert (metadata["ok"], metadata["detail"], metadata["frame_age_s"]) == (False, "capture stalled", 0.5)
    code, out, _ = run(tmp_path, capsys, "grab", "--name", "wrist", "--output", str(tmp_path / "text"), as_json=False)
    assert code == 0
    assert out.splitlines()[-1] == "warning: camera wrist health not ok: capture stalled"


def test_grab_refuses_a_single_existing_target(tmp_path, capsys, cameras):
    pytest.importorskip("PIL.Image")
    cameras.cameras["wrist"] = ScriptedCamera("wrist")
    output = tmp_path / "out"
    output.mkdir()
    (output / "wrist-depth.png").write_bytes(b"kept")
    code, payload, _ = run(tmp_path, capsys, "grab", "--name", "wrist", "--output", str(output))
    assert code == 2
    assert str(output / "wrist-depth.png") in payload["error"]
    assert cameras.opened == 0
    assert sorted(path.name for path in output.iterdir()) == ["wrist-depth.png"]


@pytest.mark.parametrize("camera,expected_code,expected_error", [
    (ScriptedCamera("wrist", frames_available=0), 1, "camera 'wrist': camera 'wrist' produced no frame within 2.0s"),
    (ScriptedCamera("front"), 2, "no camera named 'wrist'; available: front"),
])
def test_grab_failures_write_nothing(tmp_path, capsys, cameras, camera, expected_code, expected_error):
    pytest.importorskip("PIL.Image")
    cameras.cameras[camera.config.name] = camera
    output = tmp_path / "out"
    code, payload, _ = run(tmp_path, capsys, "grab", "--name", "wrist", "--output", str(output))
    assert code == expected_code
    assert payload["error"] == expected_error
    assert not output.exists()


def test_grab_requires_name_and_output(tmp_path, capsys, cameras):
    code, payload, _ = run(tmp_path, capsys, "grab")
    assert code == 2
    assert set(payload["required_fields"]) == {"--name", "--output"}
    assert cameras.opened == 0


@pytest.fixture
def camera_station(tmp_path):
    path = tmp_path / "station.yaml"
    path.write_text(yaml.safe_dump({
        "type": "franka", "name": "camera_fake", "arm_backend": "fake", "claim": False,
        "gripper": {"backend": "fake"}, "home_joints": [0.0] * 7,
        "cameras": [{"name": "wrist", "backend": "fake", "width": WIDTH, "height": HEIGHT, "fps": 30}],
    }), encoding="utf-8")
    return path


def _consistent_fake_camera_report(code, payload, source):
    entry = payload["cameras"][0]
    assert (entry["name"], entry["backend"], entry["source"]) == ("wrist", "fake", source)
    assert (entry["frames_read"], entry["color_shape"], entry["dtype"]) == (3, [HEIGHT, WIDTH, 3], "uint8")
    assert entry["measured_fps"] > 0
    # Health freshness depends on scheduling; the exit code must agree with it either way.
    assert code == (0 if entry["ok"] else 1)


def test_show_direct_mock_reads_the_fake_backend(camera_station, capsys):
    code = main(["--station", str(camera_station), "--mock", "--json", "camera", "show", "--frames", "3"])
    payload = json.loads(capsys.readouterr().out)
    assert payload["path"] == "direct fake"
    _consistent_fake_camera_report(code, payload, "direct")


def test_show_daemon_reads_the_remote_camera(camera_station, capsys, monkeypatch):
    from robot_infra.devices import Robot
    from robot_infra.discovery.autoconfig import load_station
    from robot_infra.server import remote
    from robot_infra.server.service import RobotService
    from robot_infra.server.transport.local import LocalClient

    service = RobotService(Robot.from_config(load_station(camera_station).select().config),
                           claim=False, watchdog_s=30.0)
    data = yaml.safe_load(camera_station.read_text())
    data["daemon"] = {"bind": "127.0.0.1:0", "token": "test-token"}
    camera_station.write_text(yaml.safe_dump(data))
    real_remote = remote.RemoteRobot
    monkeypatch.setattr(connect, "probe_daemon", lambda address: True)
    monkeypatch.setattr(remote, "RemoteRobot", lambda config: real_remote.from_client(LocalClient(service)))
    try:
        code = main(["--station", str(camera_station), "--json", "camera", "show", "--frames", "3"])
    finally:
        service.close()
    payload = json.loads(capsys.readouterr().out)
    assert payload["path"] == "daemon 127.0.0.1:0"
    _consistent_fake_camera_report(code, payload, "daemon")
