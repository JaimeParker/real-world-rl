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
CORE_DEPENDENCIES = [
    "numpy", "pyyaml", "msgpack", "filelock", "ruamel.yaml>=0.18", "packaging>=23", "scipy>=1.10",
]
# The only requirement installed from outside the package index: pyAgxArm comes from GitHub.
GIT_URL_EXTRAS = {"pyagxarm"}


def load_pyproject(path: Path) -> dict:
    with path.open("rb") as stream:
        return tomllib.load(stream)


def load_project(path: Path) -> dict:
    return load_pyproject(path)["project"]


def test_core_dependencies_are_torch_free():
    project = load_project(ROOT / "pyproject.toml")
    assert project["dependencies"] == CORE_DEPENDENCIES
    assert project["version"] == "0.1.0"


def test_extras():
    project = load_project(ROOT / "pyproject.toml")
    assert project["optional-dependencies"] == {
        "learning": ["rl_garden==0.0.1", "torch>=2.1", "gymnasium>=0.29", "tyro", "requests"],
        "sim": ["mani_skill"],
        "legacy-bridge": ["flask", "pyzmq", "scipy", "pyspacemouse"],
        "realsense": ["pyrealsense2"],
        "pyagxarm": ["pyAgxArm @ git+https://github.com/agilexrobotics/pyAgxArm.git"],
        "franky": ["franky-control"],
        "piper-sdk": ["piper_sdk"],
        "grpc": ["grpcio>=1.60"],
        "jpeg": ["pillow", "opencv-python-headless"],
        "spacemouse": ["hidapi==0.14.0.post4"],
        "gello": ["dynamixel-sdk>=3.7.31,<3.9"],
        "pico": ["pyzmq"],
        "gello-events": ["evdev"],
        "franka-gello": ["franky-control", "dynamixel-sdk>=3.7.31,<3.9", "evdev", "grpcio>=1.60", "websockets>=13"],
        "reward-collection": ["pynput"],
        "dev": ["pytest", "scipy"],
        "test": ["pytest", "opencv-python-headless", "ipython", "torchvision", "grpcio>=1.60"],
    }


def test_git_urls_only_in_allowlisted_extras():
    project = load_project(ROOT / "pyproject.toml")
    assert not any("git+" in requirement for requirement in project["dependencies"])
    with_git = {name for name, rows in project["optional-dependencies"].items()
                if any("git+" in row for row in rows)}
    assert with_git == GIT_URL_EXTRAS


def test_franka_gello_spells_out_its_device_extras_without_self_reference():
    extras = load_project(ROOT / "pyproject.toml")["optional-dependencies"]
    combined = set(extras["franka-gello"])
    assert set().union(*(extras[name] for name in ("franky", "gello", "gello-events", "grpc"))) <= combined
    assert not any(row.startswith("real-world-rl") for rows in extras.values() for row in rows)


def test_learning_extra_matches_the_submodule_version():
    garden_pyproject = ROOT / "rl-garden" / "pyproject.toml"
    if not garden_pyproject.exists():
        pytest.skip("rl-garden submodule not checked out at the repository root here")
    garden = load_project(garden_pyproject)
    learning = load_project(ROOT / "pyproject.toml")["optional-dependencies"]["learning"]
    assert f"rl_garden=={garden['version']}" in learning


def test_one_package_ships_the_device_layer_and_the_setup_data():
    pyproject = load_pyproject(ROOT / "pyproject.toml")
    setuptools = pyproject["tool"]["setuptools"]
    assert setuptools["packages"]["find"]["include"] == ["real_world_rl*"]
    assert setuptools["package-data"] == {"real_world_rl": ["py.typed", "setup/*.yaml", "teleop/spacemouse/udev/*.rules"]}
    assert list((ROOT / "real_world_rl" / "setup").glob("*.yaml"))
    assert list((ROOT / "real_world_rl" / "teleop" / "spacemouse" / "udev").glob("*.rules"))
    assert not (ROOT / "robot_infra").exists()


def test_no_robot_infra_command_or_station_section_entry_points():
    project = load_project(ROOT / "pyproject.toml")
    assert "robot-infra" not in project["scripts"]
    assert "robot_infra.station_sections" not in project.get("entry-points", {})
    assert "robot-infra" not in project["optional-dependencies"]


def test_installer_shell_syntax():
    subprocess.run(["bash", "-n", str(INSTALLER)], check=True, capture_output=True, text=True)


def test_installer_help_works_outside_the_checkout(tmp_path):
    result = subprocess.run([str(INSTALLER), "--help"], cwd=tmp_path, capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    for flag in ("--extras", "--daemon-only", "--venv", "--python", "--torch-index", "--station", "--no-doctor"):
        assert flag in result.stdout


@pytest.mark.parametrize("args", [
    ["--unknown"], ["--venv"], ["--extras", "--no-doctor"],
    ["--station", ""], ["--torch-index", ""], ["--venv", ""], ["--python", ""],
    ["--daemon-only", "--extras", "franky"],
    ["--extras", "grpc", "--torch-index", "https://download.pytorch.org/whl/cpu"],
])
def test_installer_rejects_bad_arguments(tmp_path, args):
    result = subprocess.run([str(INSTALLER), *args], cwd=tmp_path, capture_output=True, text=True)
    assert result.returncode != 0
    assert "argument parsing" in result.stderr and "retry" in result.stderr.lower()


def test_installer_requires_the_repository_root(tmp_path):
    (tmp_path / "pyproject.toml").touch()
    (tmp_path / "rl-garden").mkdir()  # the submodule alone does not make a checkout
    result = subprocess.run([str(INSTALLER)], cwd=tmp_path, capture_output=True, text=True)
    assert result.returncode != 0
    assert "repository root" in result.stderr
    assert not (tmp_path / ".venv").exists()


def test_installer_requires_initialized_submodule_for_learning(tmp_path):
    (tmp_path / "pyproject.toml").touch()
    (tmp_path / "real_world_rl").mkdir()
    result = subprocess.run([str(INSTALLER)], cwd=tmp_path, capture_output=True, text=True)
    assert result.returncode != 0
    assert "git submodule update --init" in result.stderr
    assert not (tmp_path / ".venv").exists()


@pytest.fixture
def installer_checkout(tmp_path):
    """Stand-in interpreter (and venv rwrl) records subprocess arguments; it never runs pip or doctor."""
    (tmp_path / "pyproject.toml").touch()
    (tmp_path / "real_world_rl").mkdir()
    (tmp_path / "rl-garden").mkdir()
    (tmp_path / "rl-garden" / "pyproject.toml").touch()
    interpreter = tmp_path / "fake-python"
    interpreter.write_text(
        f"#!{sys.executable}\n" + '''import json
import os
from pathlib import Path
import sys

args = sys.argv[1:]
# The installer deletes its throwaway mock station, so capture it while it exists.
station_text = None
if "--mock" in args and "--station" in args:
    station_text = Path(args[args.index("--station") + 1]).read_text()
with open(os.environ["INSTALL_CALLS"], "a") as stream:
    stream.write(json.dumps({"program": Path(sys.argv[0]).name, "args": args,
                             "pythonpath": os.environ.get("PYTHONPATH"),
                             "station_text": station_text}) + "\\n")
if args[:2] == ["-m", "venv"]:
    target = Path(args[2])
    (target / "bin").mkdir(parents=True)
    (target / "pyvenv.cfg").touch()
    (target / "bin" / "python").symlink_to(Path(__file__).resolve())
    (target / "bin" / "rwrl").symlink_to(Path(__file__).resolve())
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


def doctor_calls(checkout):
    return [call for call in map(json.loads, checkout[2].read_text().splitlines()) if call["program"] == "rwrl"]


def test_default_installer_runs_mock_and_prints_hardware_next_step(installer_checkout):
    result, calls = run_installer(installer_checkout)
    assert result.returncode == 0, result.stderr
    assert calls[-1][:3] == ["doctor", "--mock", "--station"] and len(calls[-1]) == 4
    assert [call["args"] for call in doctor_calls(installer_checkout)] == [calls[-1]]
    assert ["-m", "pip", "install", "-e", "./rl-garden", "-e", ".[learning,franka-gello,test]"] in calls
    assert not any("./robot_infra" in call for call in calls)
    assert any("import rl_garden, real_world_rl" in " ".join(call) for call in calls)
    assert "Next, configure local/station.yaml, then run from this repository root:" in result.stdout
    assert "rwrl doctor --station local/station.yaml" in result.stdout


def test_default_mock_doctor_gets_a_loadable_throwaway_station(installer_checkout, tmp_path, doctor_report_dir):
    """The installer hands ``rwrl doctor --mock`` a throwaway station; the file must load and
    pass under ``--mock`` and must not outlive the run."""
    from real_world_rl.cli import main

    result, calls = run_installer(installer_checkout)
    assert result.returncode == 0, result.stderr
    assert not Path(calls[-1][-1]).exists()
    station = tmp_path / "recorded-station.yaml"
    station.write_text(doctor_calls(installer_checkout)[-1]["station_text"])
    assert main(["doctor", "--mock", "--station", str(station)]) == 0


def test_custom_options_and_existing_venv(installer_checkout):
    directory, _, _, _ = installer_checkout
    args = ("--venv", "custom venv", "--extras", "learning,franka-gello,spacemouse",
            "--torch-index", "https://download.pytorch.org/whl/cpu",
            "--station", "local/my station.yaml")
    result, calls = run_installer(installer_checkout, *args)
    assert result.returncode == 0, result.stderr
    assert calls[-1] == ["doctor", "--station", "local/my station.yaml"]
    torch = ["-m", "pip", "install", "torch", "--index-url", "https://download.pytorch.org/whl/cpu"]
    assert calls.index(torch) < next(i for i, call in enumerate(calls) if "-e" in call)
    assert any(".[learning,franka-gello,spacemouse,test]" in call for call in calls)
    initial_creates = sum(call[:2] == ["-m", "venv"] for call in calls)
    result, calls = run_installer(installer_checkout, "--venv", "custom venv", "--no-doctor")
    assert result.returncode == 0, result.stderr
    assert "Reusing venv" in result.stdout
    assert sum(call[:2] == ["-m", "venv"] for call in calls) == initial_creates
    assert (directory / "custom venv" / "pyvenv.cfg").exists()


def test_daemon_only_installs_franky_and_grpc_without_the_submodule_or_torch(installer_checkout):
    directory = installer_checkout[0]
    (directory / "rl-garden" / "pyproject.toml").unlink()
    result, calls = run_installer(installer_checkout, "--daemon-only")
    assert result.returncode == 0, result.stderr
    installs = [call for call in calls if call[:3] == ["-m", "pip", "install"]]
    assert installs == [["-m", "pip", "install", "-U", "pip"], ["-m", "pip", "install", "-e", ".[franky,grpc]"]]
    imports = [call for call in calls if call[:1] == ["-c"]]
    assert len(imports) == 1 and "rl_garden" not in imports[0][1]
    assert "import real_world_rl.server, real_world_rl" in imports[0][1]
    assert calls[-1][:3] == ["doctor", "--mock", "--station"]


def test_core_only_installs_the_project_without_extras(installer_checkout):
    result, calls = run_installer(installer_checkout, "--no-doctor", "--extras", "")
    assert result.returncode == 0, result.stderr
    assert ["-m", "pip", "install", "-e", "."] in calls
    assert not any("./rl-garden" in call for call in calls)
    assert not doctor_calls(installer_checkout)


def test_failed_step_prints_retry_and_stops(installer_checkout):
    installer_checkout[3]["FAIL_INSTALL"] = "1"
    result, calls = run_installer(installer_checkout)
    assert result.returncode == 9
    assert "Upgrade pip failed" in result.stderr and "Retry" in result.stderr
    assert not doctor_calls(installer_checkout)
