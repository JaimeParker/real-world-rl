"""The device layer, the daemon and the rwrl command run without the learning extra
(torch, rl_garden, gymnasium, tyro); learning commands then fail with the install command."""
from __future__ import annotations

import ast
import json
import os
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace

import pytest

from real_world_rl.cli.cmd_learning import LEARNING_INSTALL_HINT, LEARNING_MODULES, learning_dependencies
from real_world_rl.cli.errors import CliError, USAGE_EXIT_CODE

ROOT = Path(__file__).resolve().parents[1]
# Learning-side subpackages of real_world_rl that the device layer must never import.
LEARNING_SUBPACKAGES = ("envs", "loops", "training", "policies", "algorithms", "buffers", "demos", "reward", "deploy")
DEVICE_ENTRY_MODULES = (
    "robot_infra.server.daemon",
    "robot_infra.doctor.cli",
    "robot_infra.setup.command",
    "robot_infra.discovery.report",
)
# The meta_path finder makes every learning module look uninstalled, as on a daemon-only host.
BLOCK_LEARNING = f'''
import importlib.abc
import sys
BLOCKED = {set(LEARNING_MODULES)!r}
class BlockLearning(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if fullname.split(".")[0] in BLOCKED:
            raise ModuleNotFoundError("No module named " + repr(fullname), name=fullname)
sys.meta_path.insert(0, BlockLearning())
'''


def run_without_learning(code: str, *args: str, env: dict | None = None) -> subprocess.CompletedProcess:
    return subprocess.run([sys.executable, "-c", BLOCK_LEARNING + code, *args], cwd=ROOT,
                          env={**os.environ, **(env or {})}, capture_output=True, text=True, timeout=120)


def test_device_entry_modules_import_without_learning_packages():
    code = f'''
import importlib
for name in {DEVICE_ENTRY_MODULES!r} + ("real_world_rl.station", "real_world_rl.cli"):
    importlib.import_module(name)
loaded = sorted(name for name in sys.modules if name.split(".")[0] in BLOCKED)
assert not loaded, loaded
'''
    result = run_without_learning(code)
    assert result.returncode == 0, result.stderr


@pytest.mark.parametrize("argv", [["--help"], ["daemon", "--help"], ["setup", "--help"],
                                  ["supervise", "--help"], ["discover", "--help"], ["doctor", "--help"]])
def test_rwrl_help_runs_without_learning_packages(argv):
    result = run_without_learning("from real_world_rl.cli import main\nraise SystemExit(main(sys.argv[1:]))", *argv)
    assert result.returncode == 0, result.stderr
    assert "usage:" in result.stdout


def test_mock_doctor_runs_without_learning_packages(tmp_path):
    station = tmp_path / "station.yaml"
    station.write_text("type: franka\narm_backend: fake\ngripper:\n  backend: fake\n")
    result = run_without_learning(
        "from real_world_rl.cli import main\nraise SystemExit(main(sys.argv[1:]))",
        "--json", "doctor", "--mock", "--station", str(station),
        env={"RWRL_DOCTOR_REPORT_DIR": str(tmp_path / "reports")},
    )
    assert result.returncode == 0, result.stderr
    assert json.loads(result.stdout)["status"] == "PASS"


@pytest.mark.parametrize("argv", [["train", "--help"], ["archive", "inspect", "missing.sqlite"]])
def test_learning_commands_report_the_learning_extra(argv):
    result = run_without_learning("from real_world_rl.cli import main\nraise SystemExit(main(sys.argv[1:]))", *argv)
    assert result.returncode == USAGE_EXIT_CODE, result.stderr
    assert "needs the learning extra" in result.stderr
    assert "pip install -e ./rl-garden -e '.[learning]'" in result.stderr
    assert "Traceback" not in result.stderr


def test_teleop_run_reports_the_learning_extra(monkeypatch):
    from real_world_rl.cli.cmd_teleop import TELEOP

    def missing_rl_garden(args, ctx):
        raise ModuleNotFoundError("No module named 'rl_garden'", name="rl_garden")

    monkeypatch.setattr("real_world_rl.cli.teleop_session.run_session", missing_rl_garden)
    with pytest.raises(CliError) as error:
        TELEOP.run(SimpleNamespace(teleop_command="run"), None)
    assert error.value.code == USAGE_EXIT_CODE and error.value.hint == LEARNING_INSTALL_HINT
    assert "rwrl teleop run needs the learning extra" in error.value.message


@pytest.mark.parametrize("name", ["torch", "torch.nn", "rl_garden.common", "gymnasium", "tyro"])
def test_learning_dependencies_converts_only_learning_modules(name):
    with pytest.raises(CliError) as error:
        with learning_dependencies("train"):
            raise ModuleNotFoundError(f"No module named {name!r}", name=name)
    assert error.value.code == USAGE_EXIT_CODE


def test_learning_dependencies_keeps_other_import_errors():
    with pytest.raises(ModuleNotFoundError):
        with learning_dependencies("train"):
            raise ModuleNotFoundError("No module named 'franky'", name="franky")


def test_device_layer_never_imports_the_learning_side():
    """robot_infra may load real_world_rl's station sections, never its learning subpackages."""
    offenders = []
    for path in sorted((ROOT / "robot_infra").rglob("*.py")):
        for node in ast.walk(ast.parse(path.read_text(encoding="utf-8"))):
            names = []
            if isinstance(node, ast.Import):
                names = [alias.name for alias in node.names]
            elif isinstance(node, ast.ImportFrom) and node.level == 0 and node.module:
                names = [node.module]
            for name in names:
                parts = name.split(".")
                if parts[0] in LEARNING_MODULES or (
                        parts[0] == "real_world_rl" and len(parts) > 1 and parts[1] in LEARNING_SUBPACKAGES):
                    offenders.append(f"{path.relative_to(ROOT)}: {name}")
    allowed_torch = {"controller"}  # legacy torch controllers, imported by nothing on the device path
    offenders = [line for line in offenders if line.split("/")[1] not in allowed_torch]
    assert not offenders, offenders
