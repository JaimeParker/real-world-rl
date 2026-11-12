"""real_world_rl/legacy/teleop/spacemouse must be importable without pyspacemouse
installed (this sandbox has neither) -- the HID device import must stay
inside functions/methods, never at module level."""
from __future__ import annotations

import sys

import pytest


def test_module_imports_without_pyspacemouse_installed():
    for name in list(sys.modules):
        if name == "pyspacemouse" or name.startswith("real_world_rl.legacy.teleop.spacemouse"):
            del sys.modules[name]

    import real_world_rl.legacy.teleop.spacemouse as m

    assert hasattr(m, "SpaceMouseExpert")
    assert hasattr(m, "SpaceMouseTeleOpWrapper")


def test_spacemouse_expert_raises_actionable_error_without_pyspacemouse(monkeypatch):
    from real_world_rl.legacy.teleop.spacemouse import SpaceMouseExpert

    # Simulate the missing optional dependency even on development machines
    # with HID support installed; the test must never enumerate real devices.
    monkeypatch.setitem(sys.modules, "pyspacemouse", None)
    with pytest.raises(ModuleNotFoundError, match="pyspacemouse"):
        SpaceMouseExpert()
