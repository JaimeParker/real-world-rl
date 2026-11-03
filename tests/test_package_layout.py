"""Package layout: version, removed module paths, console-script targets, lazy loop exports."""
from __future__ import annotations

import importlib
import subprocess
import sys
from pathlib import Path

import pytest

import rl_garden_real_world

PYPROJECT = Path(__file__).resolve().parents[1] / "pyproject.toml"

REMOVED_MODULES = [
    "rl_garden_real_world.actor_loop",
    "rl_garden_real_world.learner_loop",
    "rl_garden_real_world.sync",
    "rl_garden_real_world.serl",
    "rl_garden_real_world.serl.actor_loop",
    "rl_garden_real_world.hil_serl",
    "rl_garden_real_world.hil_serl.learner_loop",
    "rl_garden_real_world.residual",
    "rl_garden_real_world.residual.actor_loop",
    "rl_garden_real_world.silri",
    "rl_garden_real_world.silri.learner_loop",
    "rl_garden_real_world.utils",
    "rl_garden_real_world.utils.tree",
    "rl_garden_real_world.rlinf",
    "rl_garden_real_world.rlinf.register",
    "rl_garden_real_world.envs.wrappers.teleop_intervention",
    "rl_garden_real_world.envs.wrappers.teleop_sources",
]


def test_version_is_a_non_empty_string():
    assert isinstance(rl_garden_real_world.__version__, str)
    assert rl_garden_real_world.__version__


@pytest.mark.parametrize("module_name", REMOVED_MODULES)
def test_removed_module_paths_do_not_import(module_name):
    with pytest.raises(ModuleNotFoundError):
        importlib.import_module(module_name)


def test_every_console_script_target_is_callable():
    tomllib = pytest.importorskip("tomllib")
    scripts = tomllib.loads(PYPROJECT.read_text(encoding="utf-8"))["project"]["scripts"]
    assert set(scripts) == {
        "rlgarden-real-train",
        "rlgarden-real-record",
        "rlgarden-real-deploy",
        "rlgarden-real-collect-reward",
    }
    for name, target in scripts.items():
        module_name, attribute = target.split(":")
        assert callable(getattr(importlib.import_module(module_name), attribute)), name


def test_training_module_entry_exposes_main():
    training_main = importlib.import_module("rl_garden_real_world.training.__main__")
    assert callable(training_main.main)


def test_loops_package_resolves_method_classes_to_their_modules():
    from rl_garden_real_world import loops
    from rl_garden_real_world.loops import hil_serl, residual, serl, silri

    assert loops.SerlLearnerLoop is serl.SerlLearnerLoop
    assert loops.HilSerlActorSyncClient is hil_serl.HilSerlActorSyncClient
    assert loops.ResidualLearnerLoop is residual.ResidualLearnerLoop
    assert loops.SilriActorLoop is silri.SilriActorLoop
    with pytest.raises(AttributeError):
        loops.NotALoop  # noqa: B018


def test_importing_the_package_keeps_training_and_policies_lazy():
    code = (
        "import sys\n"
        "import rl_garden_real_world\n"
        "for name in ('rl_garden_real_world.training', 'rl_garden_real_world.policies',\n"
        "             'rl_garden_real_world.algorithms', 'rl_garden_real_world.loops.hil_serl'):\n"
        "    assert name not in sys.modules, name + ' imported'\n"
    )
    result = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
