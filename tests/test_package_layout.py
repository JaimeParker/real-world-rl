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
]


def test_version_is_a_non_empty_string():
    assert isinstance(real_world_rl.__version__, str)
    assert real_world_rl.__version__


@pytest.mark.parametrize("module_name", REMOVED_MODULES)
def test_removed_module_paths_do_not_import(module_name):
    with pytest.raises(ModuleNotFoundError):
        importlib.import_module(module_name)


def test_every_console_script_target_is_callable():
    tomllib = pytest.importorskip("tomllib")
    scripts = tomllib.loads(PYPROJECT.read_text(encoding="utf-8"))["project"]["scripts"]
    assert set(scripts) == {
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
