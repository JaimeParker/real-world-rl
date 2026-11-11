"""Installation metadata and installer CLI contracts; no downloads or hardware."""
from __future__ import annotations

import json
import os
from pathlib import Path
import subprocess
import sys

import pytest

try:
    import tomllib
except ModuleNotFoundError:  # Python 3.10: pytest supplies the TOML backport.
    import tomli as tomllib

ROOT = Path(__file__).resolve().parents[1]
INSTALLER = ROOT / "scripts" / "install.sh"


def load_project(path: Path) -> dict:
    with path.open("rb") as stream:
        return tomllib.load(stream)["project"]


def test_root_dependencies_and_extras():
    project = load_project(ROOT / "pyproject.toml")
    assert {"rl_garden==0.0.1", "rlgarden-robot-infra==0.0.1", "scipy>=1.10", "ruamel.yaml>=0.18"} <= set(
        project["dependencies"]
    )
    assert project["optional-dependencies"] == {
        "robot-infra": ["rlgarden-robot-infra==0.0.1"],
        "reward-collection": ["pynput"],
        "examples": ["mani_skill"],
        "dev": ["pytest", "scipy"],
        "franka-gello": [
            "rlgarden-robot-infra[franky,gello,gello-events,grpc]==0.0.1",
            "franky-control",
            "websockets>=13",
        ],
        "spacemouse": ["rlgarden-robot-infra[spacemouse]==0.0.1"],
        "test": ["pytest", "opencv-python-headless", "ipython", "torchvision", "grpcio>=1.60"],
    }
    requirements = list(project["dependencies"])
    for extra in project["optional-dependencies"].values():
        requirements.extend(extra)
    assert not any("git+" in requirement for requirement in requirements)


def test_device_jpeg_extra_and_local_versions():
    root = load_project(ROOT / "pyproject.toml")
    device = load_project(ROOT / "robot_infra" / "pyproject.toml")
    assert not any("ruamel" in requirement for requirement in device["dependencies"])
    garden_pyproject = ROOT / "rl-garden" / "pyproject.toml"
    if not garden_pyproject.exists():
        pytest.skip("rl-garden submodule not checked out at the repository root here")
    garden = load_project(garden_pyproject)
    assert root["version"] == device["version"] == garden["version"] == "0.0.1"
    assert device["optional-dependencies"]["jpeg"] == ["pillow", "opencv-python-headless"]
    # The unrelated pyagxarm extra intentionally retains its upstream Git URL.
    assert not any("git+" in requirement for requirement in device["dependencies"])


def test_spacemouse_extra_pins_trezor_hidapi_and_preserves_legacy_real():
    device = load_project(ROOT / "robot_infra" / "pyproject.toml")
    assert device["optional-dependencies"]["spacemouse"] == ["hidapi==0.14.0.post4"]
    assert device["optional-dependencies"]["real"] == ["flask", "pyzmq", "scipy", "pyspacemouse"]


def test_installer_shell_syntax():
    subprocess.run(["bash", "-n", str(INSTALLER)], check=True, capture_output=True, text=True)


def test_installer_help_works_outside_the_checkout(tmp_path):
    result = subprocess.run([str(INSTALLER), "--help"], cwd=tmp_path, capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    for flag in ("--extras", "--venv", "--python", "--torch-index", "--station", "--no-doctor"):
        assert flag in result.stdout


@pytest.mark.parametrize("args", [
    ["--unknown"], ["--venv"], ["--extras", "--no-doctor"],
    ["--station", ""], ["--torch-index", ""], ["--venv", ""], ["--python", ""],
])
def test_installer_rejects_bad_arguments(tmp_path, args):
    result = subprocess.run([str(INSTALLER), *args], cwd=tmp_path, capture_output=True, text=True)
    assert result.returncode != 0
    assert "argument parsing" in result.stderr and "retry" in result.stderr.lower()


def test_installer_requires_initialized_submodule(tmp_path):
    (tmp_path / "pyproject.toml").touch()
    (tmp_path / "robot_infra").mkdir()
    result = subprocess.run([str(INSTALLER)], cwd=tmp_path, capture_output=True, text=True)
    assert result.returncode != 0
    assert "git submodule update --init" in result.stderr
    assert not (tmp_path / ".venv").exists()


@pytest.fixture
def installer_checkout(tmp_path):
    """Stand-in interpreter records subprocess arguments; it never runs pip or doctor."""
    (tmp_path / "pyproject.toml").touch()
    (tmp_path / "robot_infra").mkdir()
    (tmp_path / "rl-garden").mkdir()
    (tmp_path / "rl-garden" / "pyproject.toml").touch()
    interpreter = tmp_path / "fake-python"
    interpreter.write_text(
        f"#!{sys.executable}\n" + '''import json
import os
from pathlib import Path
import sys

args = sys.argv[1:]
with open(os.environ["INSTALL_CALLS"], "a") as stream:
    stream.write(json.dumps({"args": args, "pythonpath": os.environ.get("PYTHONPATH")}) + "\\n")
if args[:2] == ["-m", "venv"]:
    target = Path(args[2])
    (target / "bin").mkdir(parents=True)
    (target / "pyvenv.cfg").touch()
    (target / "bin" / "python").symlink_to(Path(__file__).resolve())
if os.environ.get("FAIL_INSTALL") and args[:3] == ["-m", "pip", "install"]:
    sys.exit(9)
'''
    )
    interpreter.chmod(0o755)
    log = tmp_path / "calls.jsonl"
    env = {**os.environ, "INSTALL_CALLS": str(log), "PYTHONPATH": "/unwanted/sdk"}
    return tmp_path, interpreter, log, env


def run_installer(checkout, *args):
    directory, interpreter, log, env = checkout
    result = subprocess.run(
        [str(INSTALLER), "--python", str(interpreter), *args],
        cwd=directory, env=env, capture_output=True, text=True,
    )
    calls = [json.loads(line) for line in log.read_text().splitlines()]
    assert all(call["pythonpath"] is None for call in calls)
    return result, [call["args"] for call in calls]


def test_default_installer_runs_mock_and_prints_hardware_next_step(installer_checkout):
    result, calls = run_installer(installer_checkout)
    assert result.returncode == 0, result.stderr
    assert calls[-1] == ["-m", "robot_infra.doctor", "--mock"]
    assert ["-m", "pip", "install", "-e", "./rl-garden", "-e", "./robot_infra",
            "-e", ".[franka-gello,test]"] in calls
    assert "Next, configure local/station.yaml, then run from this repository root:" in result.stdout
    assert "rwrl doctor --station local/station.yaml" in result.stdout


def test_custom_options_and_existing_venv(installer_checkout):
    directory, _, _, _ = installer_checkout
    args = ("--venv", "custom venv", "--extras", "franka-gello,spacemouse",
            "--torch-index", "https://download.pytorch.org/whl/cpu",
            "--station", "local/my station.yaml")
    result, calls = run_installer(installer_checkout, *args)
    assert result.returncode == 0, result.stderr
    assert calls[-1] == ["-m", "robot_infra.doctor", "--config", "local/my station.yaml"]
    torch = ["-m", "pip", "install", "torch", "--index-url", "https://download.pytorch.org/whl/cpu"]
    assert calls.index(torch) < next(i for i, call in enumerate(calls) if "-e" in call)
    assert any(".[franka-gello,spacemouse,test]" in call for call in calls)
    initial_creates = sum(call[:2] == ["-m", "venv"] for call in calls)
    result, calls = run_installer(installer_checkout, "--venv", "custom venv", "--no-doctor")
    assert result.returncode == 0, result.stderr
    assert "Reusing venv" in result.stdout
    assert sum(call[:2] == ["-m", "venv"] for call in calls) == initial_creates
    assert (directory / "custom venv" / "pyvenv.cfg").exists()


def test_no_doctor_skips_hardware_checks(installer_checkout):
    result, calls = run_installer(installer_checkout, "--no-doctor", "--extras", "")
    assert result.returncode == 0, result.stderr
    assert not any("robot_infra.doctor" in call for call in calls)
    assert any(".[test]" in call for call in calls)


def test_failed_step_prints_retry_and_stops(installer_checkout):
    installer_checkout[3]["FAIL_INSTALL"] = "1"
    result, calls = run_installer(installer_checkout)
    assert result.returncode == 9
    assert "Upgrade pip failed" in result.stderr and "Retry" in result.stderr
    assert not any("robot_infra.doctor" in call for call in calls)


def test_station_sections_entry_point():
    project = load_project(ROOT / "pyproject.toml")
    assert project["entry-points"]["robot_infra.station_sections"] == {
        "teleop_profiles": "real_world_rl.teleop.profiles:validate_profiles_reference",
    }
