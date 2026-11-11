"""Station config lookup order, single- and multi-robot loading, the ``daemon:`` block and errors."""
from __future__ import annotations

from pathlib import Path

import pytest
import yaml

from robot_infra.devices.robots.franka import FrankaConfig
from robot_infra.discovery.autoconfig import (
    DEFAULT_DAEMON_BIND,
    STATION_CONFIG_ENV,
    default_station_config_dir,
    load_config,
    load_config_with_env,
    load_station,
    load_station_with_env,
    parse_station,
    resolve_station_config_path,
)

DOCS = Path(__file__).resolve().parents[1] / "docs"
SINGLE = {"type": "franka", "name": "solo", "arm_backend": "fake", "gripper": {"backend": "fake"}}
MULTI = {
    "robots": {
        "left": {"type": "franka", "arm_backend": "fake", "robot_ip": "10.0.0.1", "daemon": {"bind": "0.0.0.0:50051"}},
        "right": {
            "type": "franka",
            "arm_backend": "fake",
            "robot_ip": "10.0.0.2",
            "daemon": {"bind": "0.0.0.0:50052", "token": "t"},
        },
    }
}


def write(path: Path, data) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.safe_dump(data))
    return path


# -- lookup ---------------------------------------------------------------------------------


def test_explicit_path_wins_over_everything(tmp_path):
    write(tmp_path / "local" / "station.yaml", SINGLE)
    env = {STATION_CONFIG_ENV: str(tmp_path / "from_env.yaml")}
    assert resolve_station_config_path("given.yaml", env=env, cwd=tmp_path) == Path("given.yaml")


def test_env_var_wins_over_the_files_even_when_missing(tmp_path):
    write(tmp_path / "local" / "station.yaml", SINGLE)
    env = {STATION_CONFIG_ENV: str(tmp_path / "from_env.yaml")}
    assert resolve_station_config_path(env=env, cwd=tmp_path) == tmp_path / "from_env.yaml"


def test_local_station_comes_before_the_user_station(tmp_path):
    env = {"XDG_CONFIG_HOME": str(tmp_path / "xdg"), STATION_CONFIG_ENV: ""}
    cwd = tmp_path / "work"
    user = write(tmp_path / "xdg" / "robot_infra" / "station.yaml", SINGLE)
    assert resolve_station_config_path(env=env, cwd=cwd) == user
    local = write(cwd / "local" / "station.yaml", SINGLE)
    assert resolve_station_config_path(env=env, cwd=cwd) == local


def test_station_dirs(tmp_path):
    assert default_station_config_dir("local", cwd=tmp_path) == tmp_path / "local"
    assert default_station_config_dir("user", env={"XDG_CONFIG_HOME": "/x"}) == Path("/x/robot_infra")
    assert default_station_config_dir("user", env={"HOME": str(tmp_path)}) == tmp_path / ".config" / "robot_infra"
    with pytest.raises(ValueError, match="scope"):
        default_station_config_dir("system", cwd=tmp_path)


def test_nothing_found_names_every_location_and_the_wizard(tmp_path):
    env = {"XDG_CONFIG_HOME": str(tmp_path / "xdg")}
    with pytest.raises(FileNotFoundError) as info:
        resolve_station_config_path(env=env, cwd=tmp_path)
    message = str(info.value)
    assert f"${STATION_CONFIG_ENV}" in message
    assert str(tmp_path / "local" / "station.yaml") in message
    assert str(tmp_path / "xdg" / "robot_infra" / "station.yaml") in message
    assert "python -m robot_infra.setup" in message


# -- loading --------------------------------------------------------------------------------


def test_single_robot_file_is_one_robot_named_by_its_name(tmp_path):
    station = load_station(write(tmp_path / "robot.yaml", SINGLE))
    assert list(station.robots) == ["solo"] and not station.multi_robot
    robot = station.select()
    assert isinstance(robot.config, FrankaConfig) and robot.config.arm_backend == "fake"
    assert (robot.bind, robot.token) == (DEFAULT_DAEMON_BIND, None)


def test_daemon_block_is_stripped_from_a_nested_single_robot(tmp_path):
    path = write(tmp_path / "robot.yaml", {"robot": {**SINGLE, "daemon": {"bind": "127.0.0.1:6000", "token": "s"}}})
    robot = load_station(path).select()
    assert (robot.bind, robot.token) == ("127.0.0.1:6000", "s")
    assert load_config(path) == robot.config == FrankaConfig.from_dict({**SINGLE})


def test_multi_robot_station(tmp_path):
    station = load_station(write(tmp_path / "station.yaml", MULTI))
    assert station.multi_robot and list(station.robots) == ["left", "right"]
    left, right = station.robots["left"], station.robots["right"]
    assert left.config.name == "left" and left.config.robot_ip == "10.0.0.1"
    assert (left.bind, left.token) == ("0.0.0.0:50051", None)
    assert (right.bind, right.token) == ("0.0.0.0:50052", "t")


def test_an_explicit_name_is_kept_over_the_key():
    station = parse_station({"robots": {"left": {"type": "franka", "name": "panda_l", "arm_backend": "fake"}}})
    assert station.robots["left"].config.name == "panda_l"


def test_load_config_needs_a_robot_name_only_for_several_robots(tmp_path, monkeypatch):
    monkeypatch.delenv("ROBOT_IP", raising=False)
    path = write(tmp_path / "station.yaml", MULTI)
    with pytest.raises(ValueError, match="pass --robot <name>"):
        load_config_with_env(path)
    assert load_config_with_env(path, robot="right").robot_ip == "10.0.0.2"
    with pytest.raises(ValueError, match="no robot 'middle'"):
        load_config(path, robot="middle")
    one = write(tmp_path / "one.yaml", {"robots": {"left": MULTI["robots"]["left"]}})
    assert load_config(one).name == "left"


def test_load_config_with_env_fills_a_single_robot_file_as_before(tmp_path, monkeypatch):
    monkeypatch.setenv("ROBOT_IP", "10.9.8.7")
    path = write(tmp_path / "robot.yaml", {**SINGLE, "robot_ip": ""})
    assert load_config_with_env(path).robot_ip == "10.9.8.7"
    assert load_config(path).robot_ip == ""


def test_load_station_with_env_fills_every_robot(tmp_path, monkeypatch):
    monkeypatch.setenv("ROBOT_IP", "10.9.8.7")
    data = {"robots": {name: {"type": "franka", "arm_backend": "fake", "robot_ip": ""} for name in ("a", "b")}}
    station = load_station_with_env(write(tmp_path / "station.yaml", data))
    assert [robot.config.robot_ip for robot in station.robots.values()] == ["10.9.8.7", "10.9.8.7"]


@pytest.mark.parametrize(
    "data, match",
    [
        ({**SINGLE, "daemon": {"port": 1}}, "unknown daemon keys"),
        ({**SINGLE, "daemon": "0.0.0.0:1"}, "'daemon' must be a mapping"),
        ({"robots": {}}, "at least one robot"),
        ({"robots": {"a": SINGLE}, "extra": 1}, "unknown station keys"),
        ({"robots": {"a": {"arm_backend": "fake"}}}, "has no 'type'"),
        ({**SINGLE, "colour": "red"}, "unknown key 'colour'"),
        ({"robots": {"a": ["not", "a", "mapping"]}}, "must be a mapping"),
    ],
    ids=["daemon-key", "daemon-scalar", "no-robots", "extra-top-level", "no-type", "robot-key", "robot-list"],
)
def test_invalid_stations(data, match):
    with pytest.raises(ValueError, match=match):
        parse_station(data)


def test_bimanual_example_loads():
    station = load_station(DOCS / "station.bimanual.example.yaml")
    assert list(station.robots) == ["left", "right"]
    assert [robot.bind for robot in station.robots.values()] == ["0.0.0.0:50051", "0.0.0.0:50052"]
    assert [camera.name for robot in station.robots.values() for camera in robot.config.cameras] == [
        "wrist_left", "wrist_right", "front",
    ]
    assert all(robot.config.teleop.backend == "gello" for robot in station.robots.values())


@pytest.mark.parametrize("nested", [False, True])
def test_registered_sections_are_validated_stored_and_not_device_fields(nested):
    data = {**SINGLE, "application": {"label": "  operator  "},
            "daemon": {"bind": "localhost:50052", "token": "secret"}}
    original = {**data}
    calls = []

    def validate(value):
        calls.append(value)
        return value["label"].strip()

    station = parse_station({"robot": data} if nested else data,
                            sections={"application": validate})
    robot = station.select()
    assert calls == [{"label": "  operator  "}]
    assert robot.sections == {"application": "operator"}
    assert not hasattr(robot.config, "application")
    assert robot.bind == "localhost:50052"
    assert robot.token == "secret"
    assert robot.daemon_declared
    assert data == original


def test_section_validation_error_identifies_robot_and_section():
    def reject(value):
        raise ValueError("invalid label")

    with pytest.raises(ValueError, match="robot 'left', section 'application': invalid label"):
        parse_station({"robots": {"left": {"type": "franka", "application": 42}}},
                      sections={"application": reject})


def test_unregistered_section_and_other_unknown_keys_still_fail():
    with pytest.raises(ValueError, match="unknown.*application"):
        parse_station({**SINGLE, "application": {}})
    with pytest.raises(ValueError, match="unknown.*typo"):
        parse_station({**SINGLE, "application": {}, "typo": True},
                      sections={"application": dict})


def test_sections_are_per_robot_and_absent_sections_do_not_run_validators():
    calls = []

    def validate(value):
        calls.append(value)
        return {"value": value}

    station = parse_station({"robots": {
        "left": {"type": "franka", "application": 1},
        "right": {"type": "franka", "application": 2},
        "unused": {"type": "franka"},
    }}, sections={"application": validate})
    assert calls == [1, 2]
    assert station.select("left").sections == {"application": {"value": 1}}
    assert station.select("right").sections == {"application": {"value": 2}}
    assert station.select("unused").sections == {}
    station.select("left").sections["extra"] = True
    assert "extra" not in station.select("right").sections
    assert parse_station(SINGLE).select().sections == {}


@pytest.mark.parametrize("loader", [load_station, load_station_with_env])
def test_station_loaders_forward_sections(tmp_path, monkeypatch, loader):
    monkeypatch.setenv("ROBOT_IP", "10.0.0.9")
    path = write(tmp_path / "station.yaml", {**SINGLE, "robot_ip": "", "application": "value"})
    robot = loader(path, sections={"application": str.upper}).select()
    assert robot.sections == {"application": "VALUE"}
    if loader is load_station_with_env:
        assert robot.config.robot_ip == "10.0.0.9"
    with pytest.raises(ValueError, match="unknown.*application"):
        loader(path)


def test_robot_config_has_no_teleop_profiles():
    from robot_infra.devices import RobotConfig

    assert not hasattr(RobotConfig(), "teleop_profiles")
