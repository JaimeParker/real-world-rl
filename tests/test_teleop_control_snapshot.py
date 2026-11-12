"""Golden snapshots of the teleop experiment blocks: ``teleop_control`` (compared when demos or a
run are resumed) and the frozen ``teleop_input``. Their serialised form, key order included, must
not change when the teleop modules move."""
from __future__ import annotations

import json

import pytest

from real_world_rl.common.provenance import portable_path
from real_world_rl.training import env_building
from real_world_rl.training.entrypoints.hil_serl import HilSerlArgs

ROTATION = [[0, -1, 0], [1, 0, 0], [0, 0, 1]]


@pytest.fixture
def isolated_site(tmp_path, monkeypatch):
    """No local device table of this machine (./local or the user config directory) applies."""
    monkeypatch.chdir(tmp_path)
    monkeypatch.setenv("XDG_CONFIG_HOME", str(tmp_path / "xdg"))
    return tmp_path


def _options_yaml(tmp_path, text: str) -> str:
    path = tmp_path / "teleop.yaml"
    path.write_text(text, encoding="utf-8")
    return str(path)


@pytest.mark.parametrize("device", ["spacemouse", "leader_cartesian"])
def test_tracker_teleop_control(device):
    spec = env_building.teleop_control_spec(HilSerlArgs(teleop_device=device))
    assert json.dumps(spec) == (
        '{"version": "cartesian_tracker_v1", "lead_cap_m": 0.05, "lead_cap_rad": 0.3, '
        '"speed_m_s": 0.05, "speed_rad_s": 0.3}'
    )


def test_spacemouse_tracker_teleop_control_with_input_rotation_and_diagnostics(isolated_site):
    args = HilSerlArgs(
        teleop_device="spacemouse", teleop_diagnostic_axes="translation", teleop_diagnostic_gain=0.5,
        teleop_options_yaml=_options_yaml(isolated_site, f"input_rotation: {ROTATION}\n"),
    )
    assert json.dumps(env_building.teleop_control_spec(args)) == (
        '{"version": "cartesian_tracker_v1", "lead_cap_m": 0.05, "lead_cap_rad": 0.3, '
        '"speed_m_s": 0.05, "speed_rad_s": 0.3, "input_diagnostic": {"axes": "translation", "gain": 0.5}}'
    )


def test_spacemouse_serl_teleop_control(isolated_site):
    args = HilSerlArgs(
        teleop_device="spacemouse", spacemouse_mode="serl",
        teleop_options_yaml=_options_yaml(isolated_site, f"invert_axes: [0, 5]\ninput_rotation: {ROTATION}\n"),
    )
    assert json.dumps(env_building.teleop_control_spec(args)) == (
        '{"version": "spacemouse_serl_v1", "mode": "serl", "motion_threshold": 0.001, '
        '"gripper": "serl_buttons", "invert_axes": [0, 5]}'
    )


def test_non_tracker_devices_have_no_teleop_control():
    for device in ("pico", "gello", "leader_arm", "robot", "fake"):
        assert env_building.teleop_control_spec(HilSerlArgs(teleop_device=device)) is None


def test_spacemouse_teleop_input(isolated_site):
    path = _options_yaml(isolated_site, f"device_index: 2\ninput_rotation: {ROTATION}\n")
    args = HilSerlArgs(teleop_device="spacemouse", teleop_options_yaml=path)
    env_building.resolve_teleop_input(args)
    assert json.dumps(args.teleop_input) == (
        f'{{"options_file": {json.dumps(portable_path(path))}, '
        f'"source_options": {{"device_index": 2, "input_rotation": {json.dumps(ROTATION)}}}}}'
    )
