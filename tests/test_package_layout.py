"""Package layout: version, removed module paths, console-script targets, lazy loop exports."""
from __future__ import annotations

import dataclasses
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
    # robot_infra is now real_world_rl's device layer; no import shim is kept.
    "robot_infra",
    "robot_infra.cli",
    "robot_infra.cli.main",
    "robot_infra.control",
    "robot_infra.controller",
    "robot_infra.devices",
    "robot_infra.discovery",
    "robot_infra.doctor",
    "robot_infra.server",
    "robot_infra.setup",
    "robot_infra.supervise",
    "robot_infra.teleop",
    # The device layer has no command or module entry points of its own; rwrl runs them.
    "real_world_rl.cli.main",
    "real_world_rl.cli.cmd_gello_calibrate",
    "real_world_rl.server.__main__",
    "real_world_rl.doctor.__main__",
    "real_world_rl.supervise.__main__",
    "real_world_rl.setup.__main__",
    "real_world_rl.discovery.__main__",
    # One doctor: everything lives in real_world_rl.doctor; the device doctor command and its
    # text format are gone.
    "real_world_rl.cli.doctor",
    *(f"real_world_rl.cli.doctor_{name}" for name in ("host", "platform", "preset", "probe", "repair")),
    "real_world_rl.doctor.cli",
    # The teleop package is laid out as core + per-device slices; no import shim is kept.
    *(f"real_world_rl.teleop.{name}" for name in (
        "base", "registry", "part", "composite", "bimanual", "fake", "action_parts", "hold", "session",
        "session_events", "cartesian_tracker", "input_frame", "stdin", "jsonl_events", "spacemouse_serl",
        "profiles", "calibration", "input_diagnostics", "leader_arm", "leader_cartesian", "workspace_points",
    )),
    "real_world_rl.devices.backends.teleops.spacemouse",
    "real_world_rl.devices.backends.teleops._spacemouse_hid",
    "real_world_rl.devices.backends.teleops.gello",
    "real_world_rl.devices.backends.teleops.gello_calibrate",
    "real_world_rl.legacy.teleop.utils.telo_op_control_twist",
    # The SpaceMouse and GELLO teleop commands live in their slices (teleop/<device>/cli.py).
    "real_world_rl.cli.teleop_calibrate",
    "real_world_rl.cli.teleop_session",
]
# The device doctor's own text/JSON format, exit code and autofix, removed when the doctors merged.
REMOVED_DOCTOR_NAMES = [
    ("real_world_rl.doctor.checks", name) for name in ("summarize", "exit_code", "_apply_fixes", "_preflight_rerun")
] + [
    ("real_world_rl.doctor.report", "rwrl_device_hint"),
    ("real_world_rl.doctor.probes", "grant_user_acl"),
]
# Old teleop module names that are now slice packages: the package imports, the old names do not.
RENAMED_TO_PACKAGES = [
    ("real_world_rl.teleop.spacemouse", "SpaceMouseTeleopSource"),
    ("real_world_rl.teleop.spacemouse", "SpaceMouseTwistTeleopSource"),
    ("real_world_rl.teleop.gello", "GelloTeleopSource"),
    ("real_world_rl.teleop.pico", "PicoTeleopSource"),
    ("real_world_rl.teleop.keyboard", "KeyboardTeleopSource"),
    ("real_world_rl.teleop.keyboard", "OperatorKeySource"),
]


def test_version_is_a_non_empty_string():
    assert isinstance(real_world_rl.__version__, str)
    assert real_world_rl.__version__


@pytest.mark.parametrize("module_name", REMOVED_MODULES)
def test_removed_module_paths_do_not_import(module_name):
    with pytest.raises(ModuleNotFoundError):
        importlib.import_module(module_name)


@pytest.mark.parametrize("module_name, name", REMOVED_DOCTOR_NAMES)
def test_removed_doctor_names_are_gone(module_name, name):
    assert not hasattr(importlib.import_module(module_name), name)


def test_device_check_results_carry_no_autofix():
    from real_world_rl.doctor.checks import CheckResult

    assert "autofix" not in {field.name for field in dataclasses.fields(CheckResult)}


@pytest.mark.parametrize("package_name, old_name", RENAMED_TO_PACKAGES)
def test_teleop_slice_packages_do_not_re_export_the_old_module_names(package_name, old_name):
    package = importlib.import_module(package_name)
    assert not hasattr(package, old_name)
    with pytest.raises(ImportError):
        exec(f"from {package_name} import {old_name}", {})


def test_device_layer_lives_in_real_world_rl():
    import real_world_rl.devices

    root = PYPROJECT.parent
    assert Path(real_world_rl.devices.__file__).resolve().parent == root / "real_world_rl" / "devices"
    assert not (root / "robot_infra").exists()
    assert (root / "docs" / "devices.md").is_file() and (root / "docs" / "daemon.md").is_file()
    assert sorted(path.name for path in (root / "examples" / "stations").glob("*.example.yaml")) == [
        "leader.franka-as-leader.example.yaml", "robot.franka.example.yaml", "robot.gello_station.example.yaml",
        "robot.piper.example.yaml", "station.bimanual.example.yaml",
    ]


def test_device_commands_have_no_doctor_alias_or_section_discovery():
    from real_world_rl.cli import device_commands
    from real_world_rl.discovery import autoconfig

    names = {command.name for command in device_commands.COMMANDS}
    assert names == {"setup", "daemon", "supervise", "discover"}
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
