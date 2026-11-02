"""The ``rlgarden.algorithms`` entry-point module and the deploy builder lookup."""
from __future__ import annotations

import importlib
from dataclasses import dataclass

import pytest
import torch

from rl_garden.training.algorithm_registry import BaseAlgorithmRegistry
from rl_garden_real_world.training import rlgarden_algorithms
from rl_garden_real_world.training.rlgarden_algorithms import register_real_world_algorithms


class _Registry(BaseAlgorithmRegistry):
    package_name = "unused"
    phase_name = "online"


@dataclass
class _FakeArgs:
    value: int = 1


@dataclass
class _OtherArgs:
    value: int = 2


def _run(args):  # pragma: no cover - never dispatched
    raise AssertionError


def _algorithm_cls():
    return object


def _build(args, env, eval_env, logger, checkpoint_dir):  # pragma: no cover
    raise AssertionError


def _table(args_cls=_FakeArgs):
    return {"fake_method": (args_cls, _run, _algorithm_cls, _build)}


def test_module_ships_an_empty_table_and_imports_cleanly():
    importlib.reload(rlgarden_algorithms)
    assert rlgarden_algorithms._ALGORITHMS == {}


def test_registering_the_shipped_empty_table_is_a_no_op():
    registry = _Registry()
    register_real_world_algorithms(registry)
    assert registry.entries() == {}


def test_online_registry_is_untouched_by_the_empty_table():
    from rl_garden.training.online.registry import registry

    before = set(registry.entries())
    importlib.reload(rlgarden_algorithms)
    assert set(registry.entries()) == before


def test_registering_a_fake_entry_records_its_builder_and_thunk():
    registry = _Registry()
    register_real_world_algorithms(registry, _table())

    entry = registry.entries()["fake_method"]
    assert entry.args_cls is _FakeArgs
    assert entry.run_fn is _run
    assert entry.algorithm_cls is _algorithm_cls
    assert entry.build_agent is _build


def test_registration_is_idempotent():
    registry = _Registry()
    register_real_world_algorithms(registry, _table())
    register_real_world_algorithms(registry, _table())
    assert list(registry.entries()) == ["fake_method"]


def test_a_name_taken_by_another_args_class_is_rejected():
    registry = _Registry()
    register_real_world_algorithms(registry, _table())
    with pytest.raises(ValueError, match="already registered"):
        register_real_world_algorithms(registry, _table(_OtherArgs))


def test_pyproject_declares_the_entry_point():
    import pathlib
    tomllib = pytest.importorskip("tomllib")

    pyproject = pathlib.Path(__file__).resolve().parents[1] / "pyproject.toml"
    declared = tomllib.loads(pyproject.read_text(encoding="utf-8"))["project"]["entry-points"]
    assert declared["rlgarden.algorithms"] == {
        "real_world": "rl_garden_real_world.training.rlgarden_algorithms"
    }


# -- deploy resolves the builder through the online registry entry ------------------------


class _FakeEntry:
    def __init__(self, build_agent):
        self.build_agent = build_agent


class _FakeRegistry:
    def __init__(self, entries):
        self._entries = entries
        self.discovered = False

    def discover(self):
        self.discovered = True

    def entries(self):
        return dict(self._entries)


def _patch_online_registry(monkeypatch, entries):
    fake = _FakeRegistry(entries)
    # sys.modules, not a dotted string: the package attribute of the same name is the registry object.
    registry_module = importlib.import_module("rl_garden.training.online.registry")
    monkeypatch.setattr(registry_module, "registry", fake)
    return fake


def test_deploy_uses_the_registry_entry_builder_when_present(monkeypatch):
    from rl_garden_real_world.deploy import _resolve_build_agent

    fake = _patch_online_registry(monkeypatch, {"custom": _FakeEntry(_build)})
    assert _resolve_build_agent("custom") is _build
    assert fake.discovered


def test_deploy_falls_back_to_the_in_tree_builder_without_one(monkeypatch):
    from rl_garden.training.online.entrypoints.rlpd import build_rlpd
    from rl_garden_real_world.deploy import _resolve_build_agent

    _patch_online_registry(monkeypatch, {"rlpd": _FakeEntry(None)})
    assert _resolve_build_agent("rlpd") is build_rlpd
    _patch_online_registry(monkeypatch, {})
    assert _resolve_build_agent("rlpd") is build_rlpd


@pytest.mark.parametrize("algo", ["rlpd", "rlpd_hybrid"])
def test_deploy_resolves_the_in_tree_builders_through_the_real_registry(algo):
    import importlib

    from rl_garden_real_world.deploy import _resolve_build_agent

    module = importlib.import_module(f"rl_garden.training.online.entrypoints.{algo}")
    assert _resolve_build_agent(algo) is getattr(module, f"build_{algo}")


def test_restore_policy_builds_and_loads_through_the_resolved_builder(monkeypatch):
    from rl_garden_real_world.deploy import DeployArgs, restore_policy

    calls = {}

    class _Agent:
        policy = torch.nn.Linear(1, 1)

        def load(self, path, **kwargs):
            calls["load"] = (path, kwargs)

    def _custom_build(args, env, eval_env, logger, checkpoint_dir):
        calls["build"] = (args, env, eval_env, logger, checkpoint_dir)
        return _Agent()

    _patch_online_registry(monkeypatch, {"custom": _FakeEntry(_custom_build)})
    args = DeployArgs(checkpoint="ckpt.pt", algo="custom")

    policy = restore_policy(args, "training-args", "env")

    assert policy is _Agent.policy
    assert calls["build"] == ("training-args", "env", None, None, None)
    assert calls["load"] == ("ckpt.pt", {"load_replay_buffer": False, "load_optimizers": False})
