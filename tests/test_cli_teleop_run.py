"""Supervised teleop configuration and evidence, with no recorder or hardware access."""
from __future__ import annotations

import hashlib
import json
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml

from real_world_rl.cli import main
from real_world_rl.common.provenance import portable_path


# Captured from the SpaceMouse runner before extracting the common session body.
# Only machine-local paths are substituted; whitespace and key order are compared as bytes.
SPACEMOUSE_CONFIG = b"""real_robot:
  home_joints:
  - 0
  - 0
  - 0
  - 0
  - 0
  - 0
  - 0
  gripper_in_action: false
  task_frame: null
  workspace: null
  robot:
    type: franka
    name: left
    arm_backend: fake
    gripper: null
    cameras: []
    home_joints:
    - 0.0
    - -0.785
    - 0.0
    - -2.356
    - 0.0
    - 1.571
    - 0.785
    safety_box: null
    joint_limits: null
    max_delta_per_step: null
    control: null
    claim: true
    teleop:
      name: teleop
      backend: fake
      hold_s: 0.0
      options: {}
    task_frame: null
    workspace: null
    robot_ip: 172.16.0.2
    bridge_url: http://127.0.0.1:5000
    relative_dynamics_factor: 0.2
    realtime: ignore
  remote: null
  token: null
env_backend: real_robot
teleop_device: spacemouse
teleop_event_device: stdin
teleop_events_file: '-'
spacemouse_mode: tracker
teleop_options_yaml: <ROOT>/station.yaml
teleop_robot: null
teleop_profile: front
episode_end_behavior: hold
neutral_threshold: 0.05
num_successes: 20
max_episodes: null
output_path: <ROOT>/demo.pkl
teleop_input:
  options_file: <STATION_FILE>
  profile_name: front
  profile_file: <PROFILE_FILE>
  yaw_rad: 0.0
  input_rotation:
  - - 1.0
    - 0.0
    - 0.0
  - - 0.0
    - 1.0
    - 0.0
  - - 0.0
    - 0.0
    - 1.0
  task_frame: null
  source_options:
    input_rotation:
    - - 1.0
      - 0.0
      - 0.0
    - - 0.0
      - 1.0
      - 0.0
    - - 0.0
      - 0.0
      - 1.0
"""


@pytest.fixture
def session(tmp_path, monkeypatch):
    """Real station/config parsing, but no child process and no connected devices."""
    path = tmp_path / "station.yaml"
    config = {"type": "franka", "name": "left", "arm_backend": "fake", "gripper": None,
              "cameras": {}, "teleop": {"backend": "spacemouse"},
              "teleop_profiles": {"file": "profiles.yaml", "default": "front"}}
    path.write_text(yaml.safe_dump(config))
    (tmp_path / "profiles.yaml").write_text(
        "schema_version: 1\nprofiles:\n"
        "  front: {frame: base, yaw_rad: 0.0, axis_mapping_version: serl_hid_axes_v1}\n")
    preset = tmp_path / "preset.yaml"
    preset.write_text("real_robot:\n  home_joints: [0, 0, 0, 0, 0, 0, 0]\n"
                      "  gripper_in_action: false\n  stop_trace_path: stale\n"
                      "  diagnostics_path: stale\n  session_events_path: stale\n")
    calls = []

    def run(command, **kwargs):
        calls.append((command, kwargs))
        return SimpleNamespace(returncode=0)

    def forbidden(*args, **kwargs):
        raise AssertionError("configuration preparation must not open a device")

    monkeypatch.setattr("subprocess.run", run)
    monkeypatch.setattr("real_world_rl.cli.connect.open_robot", forbidden)
    monkeypatch.setattr("real_world_rl.devices.Robot.from_yaml", forbidden)
    monkeypatch.delenv("RWRL_HEARTBEAT_FD", raising=False)
    return SimpleNamespace(path=path, preset=preset, calls=calls, prefix=[
        "--station", str(path), "--preset", str(preset), "--mock", "--yes", "--json"])


def test_spacemouse_resolved_config_bytes(session, tmp_path, capsys):
    assert main([*session.prefix, "teleop", "run", "--record", str(tmp_path / "demo.pkl")]) == 0
    payload = json.loads(capsys.readouterr().out)
    report = Path(payload["report_directory"])
    data = (report / "resolved-config.yaml").read_bytes()
    data = data.replace(b"  options_file: " + portable_path(session.path).encode() + b"\n",
                        b"  options_file: <STATION_FILE>\n")
    data = data.replace(b"  profile_file: " + portable_path(tmp_path / "profiles.yaml").encode() + b"\n",
                        b"  profile_file: <PROFILE_FILE>\n")
    data = data.replace(str(tmp_path).encode(), b"<ROOT>")
    assert data == SPACEMOUSE_CONFIG
    assert payload["profile"] == "front"
    assert len(session.calls) == 1


def gello_station(session, *, wrapped=False, daemon=False):
    # Real follower-only fields must never leak into the leader stub.
    config = {"type": "franka", "name": "left", "robot_ip": "192.0.2.9",
              "arm_backend": "franky", "gripper": {"backend": "franky"},
              "cameras": [{"name": "wrist", "backend": "realsense"}],
              "teleop": {"backend": "gello", "name": "leader", "hold_s": .1,
                         "options": {"port": "/dev/serial/by-id/test-leader", "joint_ids": list(range(1, 8)),
                                     "joint_signs": [1, -1, 1, -1, 1, -1, 1]}}}
    if daemon:
        config["daemon"] = {"bind": "127.0.0.1:50051"}
        document = {"robot": config}
    elif wrapped:
        document = {"robots": {"left": config, "right": {"type": "franka", "name": "right",
                                                             "teleop": {"backend": "spacemouse"}}}}
    else:
        document = config
    text = yaml.safe_dump(document, sort_keys=False)
    # Keep the original spelling and comments, as well as the values.
    text = text.replace("backend: gello", 'backend: "gello"  # leader only')
    session.path.write_text(text)
    return config["teleop"]


@pytest.mark.parametrize("wrapped", [False, True])
@pytest.mark.parametrize("events", [None, "-", "events.jsonl"])
def test_gello_resolved_config_and_provenance(session, tmp_path, capsys, wrapped, events):
    teleop = gello_station(session, wrapped=wrapped)
    original = session.path.read_bytes()
    event_flags = [] if events is None else ["--events-file", events]
    robot_flags = ["--robot", "left"] if wrapped else []
    assert main([*session.prefix, *robot_flags, "teleop", "run", "--backend", "gello",
                 "--record", str(tmp_path / "gello.pkl"), *event_flags]) == 0
    payload = json.loads(capsys.readouterr().out)
    report = Path(payload["report_directory"])
    config = yaml.safe_load((report / "resolved-config.yaml").read_bytes())
    assert config["teleop_device"] == "leader_cartesian"
    assert config["episode_end_behavior"] == "hold"
    assert config["neutral_threshold"] == .5
    assert config["neutral_stable_s"] == .2
    assert config["teleop_event_device"] == ("jsonl" if events else "stdin")
    assert config["teleop_events_file"] == (str(Path(events).resolve()) if events not in (None, "-") else "-")
    assert config["num_successes"] == 20 and config["max_episodes"] is None
    options_path = Path(config["teleop_options_yaml"])
    options = yaml.safe_load(options_path.read_bytes())
    stub_path = Path(options["robot_yaml"])
    assert options_path.parent == stub_path.parent == report
    assert set(options) == {"robot_yaml"}
    stub = yaml.safe_load(stub_path.read_bytes())
    assert stub == {"type": "franka", "name": "gello-leader", "arm_backend": "fake",
                    "gripper": None, "cameras": [], "teleop": teleop}
    assert 'backend: "gello"  # leader only' in stub_path.read_text()
    assert session.path.read_bytes() == original
    assert not {"stop_trace_path", "diagnostics_path", "session_events_path"} & config["real_robot"].keys()
    command, kwargs = session.calls[0]
    assert command[1:5] == ["-m", "real_world_rl.cli", "supervise", "--"]
    assert command[-1] == str(report / "resolved-config.yaml")
    for variable, filename in (("RWRL_STOP_TRACE_PATH", "stop-trace.json"),
                               ("RWRL_DIAGNOSTICS_PATH", "diagnostics.jsonl"),
                               ("RWRL_SESSION_EVENTS_PATH", "session-events.jsonl")):
        assert kwargs["env"][variable] == str(report / filename)
    evidence = json.loads((report / "provenance.json").read_text())["input_files"]
    for key, path in (("station", session.path), ("preset", session.preset),
                      ("resolved_config", report / "resolved-config.yaml"),
                      ("leader_robot", stub_path), ("leader_options", options_path)):
        assert evidence[key]["path"] == portable_path(path)
        assert evidence[key]["sha256"] == hashlib.sha256(path.read_bytes()).hexdigest()
    assert json.loads((report / "summary.json").read_text()) == payload
    assert payload["stop_assessment"]["status"] == "BLOCKED"


def test_gello_daemon_and_custom_limits(session, tmp_path, capsys):
    gello_station(session, daemon=True)
    assert main([*session.prefix, "teleop", "run", "--record", str(tmp_path / "gello.pkl"),
                 "--neutral-stable-s", ".4", "--num-successes", "2", "--max-episodes", "3"]) == 0
    payload = json.loads(capsys.readouterr().out)
    config = yaml.safe_load((Path(payload["report_directory"]) / "resolved-config.yaml").read_bytes())
    assert config["real_robot"]["robot"] is None
    assert config["real_robot"]["remote"] == "127.0.0.1:50051"
    assert config["neutral_stable_s"] == .4
    assert config["num_successes"] == 2 and config["max_episodes"] == 3


@pytest.mark.parametrize("backend", [None, "spacemouse", "fake"])
def test_gello_rejects_other_station_teleop(session, tmp_path, capsys, backend):
    document = yaml.safe_load(session.path.read_bytes())
    document["teleop"] = None if backend is None else {"backend": backend}
    session.path.write_text(yaml.safe_dump(document))
    assert main([*session.prefix, "teleop", "run", "--backend", "gello",
                 "--record", str(tmp_path / "demo.pkl")]) == 2
    assert "requires a station teleop backend of gello" in json.loads(capsys.readouterr().out)["error"]
    assert session.calls == []
    assert not list(tmp_path.glob("*.session-*"))


@pytest.mark.parametrize("flags,message", [
    (["--neutral-stable-s", "-1"], "neutral_stable_s"),
    (["--neutral-stable-s", "nan"], "neutral_stable_s"),
    (["--num-successes", "0"], "episode limits"),
    (["--max-episodes", "0"], "episode limits"),
    (["--mode", "serl"], "not applicable to --backend gello"),
    (["--gain", ".2"], "not applicable to --backend gello"),
])
def test_gello_invalid_options_do_not_launch(session, tmp_path, capsys, flags, message):
    gello_station(session)
    assert main([*session.prefix, "teleop", "run", "--backend", "gello",
                 "--record", str(tmp_path / "demo.pkl"), *flags]) == 2
    assert message in json.loads(capsys.readouterr().out)["error"]
    assert session.calls == []
    assert not list(tmp_path.glob("*.session-*"))


def test_spacemouse_refuses_gello_flags(session, tmp_path, capsys):
    assert main([*session.prefix, "teleop", "run", "--backend", "spacemouse",
                 "--record", str(tmp_path / "demo.pkl"), "--neutral-stable-s", ".2"]) == 2
    assert "--neutral-stable-s not applicable to --backend spacemouse" in json.loads(
        capsys.readouterr().out)["error"]
    assert session.calls == []


def test_run_help_groups_and_common_flags(capsys):
    assert main(["teleop", "run", "--help"]) == 0
    text = capsys.readouterr().out
    common, backends = text.split("\nspacemouse:\n")
    spacemouse, gello = backends.split("\ngello:\n")
    assert "--backend {spacemouse,gello}" in common
    assert "--mode" in spacemouse and "--diagnose" in spacemouse
    assert "--neutral-stable-s" in gello and "--profile" not in gello
    for flag in ("--record", "--events-file", "--num-successes", "--max-episodes"):
        assert sum(line.lstrip().startswith(flag + " ") for line in text.splitlines()) == 1
        assert flag not in spacemouse and flag not in gello
