"""``rwrl workspace show``: the station's joint limits, servo step caps and the home pose check."""
import json

import pytest
import yaml

from real_world_rl.cli import main
from real_world_rl.cli.cmd_workspace import home_limit_report

HOME = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
LIMITS = [[-2.8, 2.8], [-1.7, 1.7], [-2.8, 2.8], [-3.0, -0.1], [-2.8, 2.8], [0.0, 3.7], [-2.8, 2.8]]
MAX_DELTA = {"xyz_m": 0.05, "joint_rad": 0.04}


def show(tmp_path, capsys, **fields):
    path = tmp_path / "station.yaml"
    path.write_text(yaml.safe_dump({"type": "franka", "name": "arm", "robot_ip": "172.16.0.2", **fields}))
    assert main(["--station", str(path), "--json", "workspace", "show"]) == 0
    return json.loads(capsys.readouterr().out)


def test_show_reports_limits_step_caps_and_a_home_inside_the_limits(tmp_path, capsys):
    result = show(tmp_path, capsys, home_joints=HOME, joint_limits=LIMITS, max_delta_per_step=MAX_DELTA)
    assert result["joint_limits"] == LIMITS
    assert result["max_delta_per_step"] == MAX_DELTA
    assert result["home_joints"] == HOME
    assert result["home_within_limits"] is True
    assert "home_violations" not in result


def test_show_without_joint_limits_cannot_judge_the_home(tmp_path, capsys):
    result = show(tmp_path, capsys, home_joints=HOME)
    assert result["joint_limits"] is None and result["max_delta_per_step"] is None
    assert result["home_joints"] == HOME
    assert result["home_within_limits"] is None
    assert "home_violations" not in result


def test_show_without_home_cannot_judge_it(tmp_path, capsys):
    # A Franka station defaults home_joints to the ready pose; an explicit null clears it.
    result = show(tmp_path, capsys, joint_limits=LIMITS, home_joints=None)
    assert result["home_joints"] is None and result["home_within_limits"] is None


def test_show_text_mode_prints_the_same_keys(tmp_path, capsys):
    path = tmp_path / "station.yaml"
    path.write_text(yaml.safe_dump({"type": "franka", "name": "arm", "robot_ip": "172.16.0.2",
                                    "home_joints": HOME, "joint_limits": LIMITS}))
    assert main(["--station", str(path), "workspace", "show"]) == 0
    text = capsys.readouterr().out
    assert '"home_within_limits": true' in text and '"joint_limits"' in text and '"max_delta_per_step"' in text


def test_home_outside_the_limits_names_each_violating_joint():
    home = [0.0, -0.785, 3.0, -2.356, 0.0, 1.571, -2.9]
    report = home_limit_report(home, LIMITS)
    assert report["home_within_limits"] is False
    assert report["home_violations"] == [
        {"joint": 2, "value": 3.0, "low": -2.8, "high": 2.8},
        {"joint": 6, "value": -2.9, "low": -2.8, "high": 2.8},
    ]


@pytest.mark.parametrize("home, limits", [(None, LIMITS), (HOME, None), (None, None)])
def test_home_limit_report_is_none_without_home_or_limits(home, limits):
    assert home_limit_report(home, limits) == {"home_within_limits": None}


def test_home_on_a_limit_is_inside():
    assert home_limit_report([2.8, 0.0], [[-2.8, 2.8], [0.0, 1.0]]) == {"home_within_limits": True}
