"""``rwrl workspace frame set/show``: task_frame from a matrix or three captured points."""
import json

import numpy as np
import pytest
import yaml

from real_world_rl.cli import main
from real_world_rl.station import load_station
from real_world_rl.teleop.core.workspace_points import (frame_axes, frame_from_points, frame_point_quality,
                                                    point_base_positions)

ROBOT = {"type": "franka", "name": "left", "robot_ip": "172.16.0.2"}
TRANSLATED = [1, 0, 0, .5, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]
# 90 degrees about base z, origin at (0.1, 0.2, 0.3).
ROTATED = [[0, -1, 0, .1], [1, 0, 0, .2], [0, 0, 1, .3], [0, 0, 0, 1]]
UNIT_POINTS = {"o": [1, 0, 0], "px": [2, 0, 0], "py": [1, 1, 0]}


def write_station(tmp_path, **fields):
    path = tmp_path / "station.yaml"
    path.write_text(yaml.safe_dump({**ROBOT, **fields}))
    return path


def write_points(tmp_path, points, *, frame="base", base_T_frame=None):
    document = {"schema_version": 1, "frame": frame, "units": "m",
                "base_T_frame": np.eye(4).tolist() if base_T_frame is None else base_T_frame,
                "points": {name: {"base_pose": [*xyz, 0, 0, 0, 1]} for name, xyz in points.items()}}
    path = tmp_path / "points.yaml"
    path.write_text(yaml.safe_dump(document))
    return path


def frame(path, capsys, *command, code=0):
    assert main(["--station", str(path), "--json", "--yes", "workspace", "frame", *command]) == code
    return json.loads(capsys.readouterr().out)


def matrix_args(values):
    return ["--base-T-frame", *(str(value) for value in values)]


def point_args(origin="o", x_axis="px", y_axis="py"):
    return ["--from-points", "points.yaml", "--origin", origin, "--x-axis", x_axis, "--y-axis", y_axis]


def test_frame_from_points_known_answer():
    expected = np.eye(4)
    expected[0, 3] = 1
    np.testing.assert_allclose(frame_from_points(*UNIT_POINTS.values()), expected, atol=1e-12)


def test_frame_from_points_orthogonalises_y_and_reports_quality():
    transform = frame_from_points([0, 0, 0], [1, 0, 0], [1, 1, 0])
    np.testing.assert_allclose(transform[:3, :3], np.eye(3), atol=1e-12)
    quality = frame_point_quality([0, 0, 0], [1, 0, 0], [1, 1, 0])
    assert quality["axis_lengths_m"] == pytest.approx({"x": 1.0, "y": 2 ** .5})
    assert quality["y_axis_angle_deg"] == pytest.approx(45.0)


@pytest.mark.parametrize("x_point, y_point, message", [
    ([3, 0, 0], [2, 0, 0], "collinear"),
    ([1.0005, 0, 0], [1, 1, 0], "at least 1 mm"),
])
def test_frame_from_points_rejects_degenerate_points(x_point, y_point, message):
    with pytest.raises(ValueError, match=message):
        frame_from_points([1, 0, 0], x_point, y_point)


def test_point_base_positions_use_base_poses_from_any_recorded_frame():
    document = {"schema_version": 1, "frame": "table", "units": "m", "base_T_frame": ROTATED,
                "points": {"o": {"base_pose": [1, 2, 3, 0, 0, 0, 1]}}}
    np.testing.assert_allclose(point_base_positions(document)["o"], [1, 2, 3])
    document["base_T_frame"] = np.eye(4).tolist()
    document["frame"] = "base"
    np.testing.assert_allclose(point_base_positions(document)["o"], [1, 2, 3])


def test_frame_axes_are_rotation_columns():
    axes = frame_axes({"name": "table", "base_T_frame": ROTATED})
    assert axes["origin_base_m"] == [.1, .2, .3]
    assert axes["axes_base"] == {"x": [0, 1, 0], "y": [-1, 0, 0], "z": [0, 0, 1]}
    assert frame_axes(None) == {"origin_base_m": None, "axes_base": None}


def test_set_from_matrix_is_a_candidate_without_source(tmp_path, capsys):
    path = write_station(tmp_path)
    original = path.read_bytes()
    result = frame(path, capsys, "set", "--name", "table", *matrix_args(TRANSLATED))
    assert result["candidate"] == {"name": "table", "base_T_frame": [TRANSLATED[i:i + 4] for i in range(0, 16, 4)]}
    assert result["saved"] is None and "workspace_frame_rebased" not in result
    assert path.read_bytes() == original


@pytest.mark.parametrize("values, name, message", [
    ([2, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1], "table", "proper rigid"),
    ([1, 0, 0, 0, 0, 1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 1], "table", "proper rigid"),
    (TRANSLATED, "base", "non-base frame"),
])
def test_set_rejects_invalid_frames_as_usage_errors(tmp_path, capsys, values, name, message):
    path = write_station(tmp_path)
    result = frame(path, capsys, "set", "--name", name, *matrix_args(values), code=2)
    assert message in result["error"]


def test_set_from_matrix_rejects_point_names(tmp_path, capsys):
    path = write_station(tmp_path)
    result = frame(path, capsys, "set", "--name", "table", *matrix_args(TRANSLATED), "--origin", "o", code=2)
    assert "--from-points" in result["error"]


def test_set_from_points_builds_the_frame_with_default_source(tmp_path, capsys):
    path = write_station(tmp_path)
    write_points(tmp_path, UNIT_POINTS)
    result = frame(path, capsys, "set", "--name", "table", *point_args())
    candidate = result["candidate"]
    np.testing.assert_allclose(candidate["base_T_frame"], [[1, 0, 0, 1], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]],
                               atol=1e-12)
    assert candidate["source"] == "points:points.yaml origin=o x=px y=py"
    assert result["axis_lengths_m"] == pytest.approx({"x": 1.0, "y": 1.0})
    assert result["y_axis_angle_deg"] == pytest.approx(0.0, abs=1e-9)
    explicit = frame(path, capsys, "set", "--name", "table", *point_args(), "--source", "survey 2026-11")
    assert explicit["candidate"]["source"] == "survey 2026-11"


def test_set_from_collinear_points_is_a_usage_error(tmp_path, capsys):
    path = write_station(tmp_path)
    write_points(tmp_path, {**UNIT_POINTS, "py": [3, 0, 0]})
    assert "collinear" in frame(path, capsys, "set", "--name", "table", *point_args(), code=2)["error"]


def test_set_from_points_requires_known_point_names(tmp_path, capsys):
    path = write_station(tmp_path)
    write_points(tmp_path, UNIT_POINTS)
    missing = frame(path, capsys, "set", "--name", "table", "--from-points", "points.yaml",
                    "--origin", "o", "--x-axis", "px", code=2)
    assert missing["required_fields"] == ["--y-axis"]
    unknown = frame(path, capsys, "set", "--name", "table", *point_args(y_axis="corner"), code=2)
    assert "corner" in unknown["error"]


def test_save_adds_task_frame_and_preserves_comments(tmp_path, capsys):
    path = tmp_path / "station.yaml"
    before = ("# Station notes\ntype: franka\nname:  \"left\"  # operator label\n\n"
              "robot_ip: '172.16.0.2' # FCI\n")
    path.write_text(before)
    write_points(tmp_path, UNIT_POINTS)
    result = frame(path, capsys, "set", "--name", "table", *point_args(), "--save")
    assert result["saved"]["backup"]
    text = path.read_text()
    assert text.startswith("# Station notes\n")
    assert "name:  \"left\"  # operator label\n" in text and "robot_ip: '172.16.0.2' # FCI\n" in text
    assert load_station(path).select().config.task_frame == result["candidate"]


def test_save_replaces_only_the_task_frame(tmp_path, capsys):
    from test_yaml_edit import assert_only_key_changed

    path = tmp_path / "station.yaml"
    before = ("# Station notes\ntype: franka\nname:  \"left\"  # operator label\n"
              + yaml.safe_dump({"task_frame": {"name": "table", "base_T_frame": np.eye(4).tolist()}})
              + "robot_ip: '172.16.0.2' # FCI\n\n# End of station\n")
    path.write_text(before)
    result = frame(path, capsys, "set", "--name", "table", *matrix_args(TRANSLATED), "--save")
    assert_only_key_changed(before, path.read_text(), ("task_frame",))
    assert path.read_text().endswith("# FCI\n\n# End of station\n")
    assert load_station(path).select().config.task_frame == result["candidate"]


def test_save_refuses_a_name_that_orphans_the_workspace(tmp_path, capsys):
    workspace = {"frame": "table", "low": [0, 0, 0], "high": [1, 1, 1]}
    path = write_station(tmp_path, task_frame={"name": "table", "base_T_frame": np.eye(4).tolist()},
                         workspace=workspace)
    original = path.read_bytes()
    result = frame(path, capsys, "set", "--name", "bench", *matrix_args(TRANSLATED), "--save", code=2)
    assert "'table'" in result["error"] and "--name table" in result["hint"]
    assert path.read_bytes() == original
    # The station could not load such a candidate, so a preview is refused the same way.
    preview = frame(path, capsys, "set", "--name", "bench", *matrix_args(TRANSLATED), code=2)
    assert preview["error"] == result["error"]


def test_save_with_a_base_frame_workspace_is_not_a_rebase(tmp_path, capsys):
    workspace = {"frame": "base", "low": [0, 0, 0], "high": [1, 1, 1]}
    path = write_station(tmp_path, workspace=workspace)
    result = frame(path, capsys, "set", "--name", "bench", *matrix_args(TRANSLATED), "--save")
    assert "workspace_frame_rebased" not in result
    config = load_station(path).select().config
    assert config.task_frame["name"] == "bench" and config.workspace == workspace


def test_save_with_the_workspace_frame_name_rebases_it(tmp_path, capsys):
    workspace = {"frame": "table", "low": [0, 0, 0], "high": [1, 1, 1]}
    path = write_station(tmp_path, task_frame={"name": "table", "base_T_frame": np.eye(4).tolist()},
                         workspace=workspace)
    result = frame(path, capsys, "set", "--name", "table", *matrix_args(TRANSLATED), "--save")
    assert result["workspace_frame_rebased"] is True
    assert "rwrl workspace validate" in result["workspace_note"]
    config = load_station(path).select().config
    assert config.task_frame["base_T_frame"][0][3] == .5 and config.workspace == workspace


def test_show_reports_the_frame_and_its_axes(tmp_path, capsys):
    task_frame = {"name": "table", "base_T_frame": ROTATED, "source": "survey"}
    path = write_station(tmp_path, task_frame=task_frame)
    result = frame(path, capsys, "show")
    assert result["task_frame"] == task_frame
    assert result["origin_base_m"] == [.1, .2, .3]
    assert result["axes_base"] == {"x": [0, 1, 0], "y": [-1, 0, 0], "z": [0, 0, 1]}


def test_show_without_a_task_frame_is_null(tmp_path, capsys):
    result = frame(write_station(tmp_path), capsys, "show")
    assert result["task_frame"] is None and result["origin_base_m"] is None and result["axes_base"] is None


def test_set_text_mode_prints_the_json_result(tmp_path, capsys):
    path = write_station(tmp_path)
    assert main(["--station", str(path), "workspace", "frame", "set", "--name", "table",
                 *matrix_args(TRANSLATED)]) == 0
    assert json.loads(capsys.readouterr().out)["candidate"]["name"] == "table"


def test_frame_set_help_lists_both_definitions(capsys):
    assert main(["workspace", "frame", "set", "--help"]) == 0
    output = capsys.readouterr().out
    assert "--base-T-frame" in output and "--from-points" in output and "--y-axis" in output
