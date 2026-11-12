"""Package layout: version, removed module paths, console-script targets, lazy loop exports."""
from __future__ import annotations

import importlib
import subprocess
import sys
from pathlib import Path

import pytest

import real_world_rl

PYPROJECT = Path(__file__).resolve().parents[1] / "pyproject.toml"

REMOVED_MODULES = [
    "rl_garden_real_world",
    "rl_garden_real_world.loops",
    "real_world_rl.actor_loop",
    "real_world_rl.learner_loop",
    "real_world_rl.sync",
    "real_world_rl.serl",
    "real_world_rl.serl.actor_loop",
    "real_world_rl.hil_serl",
    "real_world_rl.hil_serl.learner_loop",
    "real_world_rl.residual",
    "real_world_rl.residual.actor_loop",
    "real_world_rl.silri",
    "real_world_rl.silri.learner_loop",
    "real_world_rl.utils",
    "real_world_rl.utils.tree",
    "real_world_rl.rlinf",
    "real_world_rl.rlinf.register",
    "real_world_rl.envs.wrappers.teleop_intervention",
    "real_world_rl.envs.wrappers.teleop_sources",
    # The device layer has no command or module entry points of its own; rwrl runs them.
    "robot_infra.cli.__main__",
    "robot_infra.cli.cmd_doctor",
    "robot_infra.server.__main__",
    "robot_infra.doctor.__main__",
    "robot_infra.supervise.__main__",
    "robot_infra.setup.__main__",
    "robot_infra.discovery.__main__",
]


def test_version_is_a_non_empty_string():
    assert isinstance(real_world_rl.__version__, str)
    assert real_world_rl.__version__


@pytest.mark.parametrize("module_name", REMOVED_MODULES)
def test_removed_module_paths_do_not_import(module_name):
    with pytest.raises(ModuleNotFoundError):
        importlib.import_module(module_name)


def test_device_layer_is_a_top_level_package_of_this_checkout():
    import robot_infra

    root = PYPROJECT.parent
    assert Path(robot_infra.__file__).resolve().parent == root / "robot_infra"
    assert not (root / "robot_infra" / "tests").exists()
    assert not (root / "robot_infra" / "docs").exists()
    assert (root / "docs" / "devices" / "README.md").is_file()
    assert sorted(path.name for path in (root / "examples" / "stations").glob("*.example.yaml")) == [
        "robot.franka.example.yaml", "robot.gello_station.example.yaml",
        "robot.piper.example.yaml", "station.bimanual.example.yaml",
    ]


def test_device_cli_has_no_entry_point_or_section_discovery():
    import inspect
    import robot_infra.cli
    from robot_infra.discovery import autoconfig

    # `main` may exist as the imported submodule attribute; the entry-point function is gone.
    assert not inspect.isfunction(getattr(robot_infra.cli, "main", None))
    assert "doctor" not in {command.name for command in robot_infra.cli.COMMANDS}
    assert not hasattr(autoconfig, "discover_station_sections")


def _project_scripts(text: str) -> dict[str, str]:
    """``[project.scripts]`` table; falls back to reading the block as text on Python 3.10 without tomli."""
    try:
        import tomllib
    except ModuleNotFoundError:
        try:
            import tomli as tomllib
        except ModuleNotFoundError:
            tomllib = None
    if tomllib is not None:
        return tomllib.loads(text)["project"]["scripts"]
    return _project_scripts_from_text(text)


def _project_scripts_from_text(text: str) -> dict[str, str]:
    scripts: dict[str, str] = {}
    in_block = False
    for raw_line in text.splitlines():
        line = raw_line.split("#", 1)[0].strip()
        if line.startswith("["):
            in_block = line == "[project.scripts]"
        elif in_block and "=" in line:
            name, target = (part.strip() for part in line.split("=", 1))
            scripts[name.strip("\"'")] = target.strip("\"'")
    return scripts


def test_project_scripts_text_fallback_reads_only_the_scripts_block():
    text = ('[project]\nname = "x"\n\n[project.scripts]\n'
            'tool = "pkg.cli:main"  # comment\n"other-tool" = \'pkg.other:run\'\n\n'
            '[project.entry-points."group"]\nignored = "pkg:ignored"\n')
    assert _project_scripts_from_text(text) == {"tool": "pkg.cli:main", "other-tool": "pkg.other:run"}
    actual = PYPROJECT.read_text(encoding="utf-8")
    assert _project_scripts_from_text(actual) == _project_scripts(actual)


def test_every_console_script_target_is_callable():
    scripts = _project_scripts(PYPROJECT.read_text(encoding="utf-8"))
    assert set(scripts) == {
        "rwrl",
        "real-world-rl-train",
        "real-world-rl-record",
        "real-world-rl-deploy",
        "real-world-rl-collect-reward",
    }
    for name, target in scripts.items():
        module_name, attribute = target.split(":")
        assert callable(getattr(importlib.import_module(module_name), attribute)), name


def test_training_module_entry_exposes_main():
    training_main = importlib.import_module("real_world_rl.training.__main__")
    assert callable(training_main.main)


def test_loops_package_resolves_method_classes_to_their_modules():
    from real_world_rl import loops
    from real_world_rl.loops import hil_serl, residual, serl, silri

    assert loops.SerlLearnerLoop is serl.SerlLearnerLoop
    assert loops.HilSerlActorSyncClient is hil_serl.HilSerlActorSyncClient
    assert loops.ResidualLearnerLoop is residual.ResidualLearnerLoop
    assert loops.SilriActorLoop is silri.SilriActorLoop
    with pytest.raises(AttributeError):
        loops.NotALoop  # noqa: B018


def test_importing_the_package_keeps_training_and_policies_lazy():
    code = (
        "import sys\n"
        "import real_world_rl\n"
        "for name in ('real_world_rl.training', 'real_world_rl.policies',\n"
        "             'real_world_rl.algorithms', 'real_world_rl.loops.hil_serl'):\n"
        "    assert name not in sys.modules, name + ' imported'\n"
    )
    result = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
