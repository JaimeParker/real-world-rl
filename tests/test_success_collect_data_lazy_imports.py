"""collect_data.py must be importable without pynput installed (this sandbox
has neither) -- the keyboard-listener import must stay inside main(), never
at module level."""
from __future__ import annotations


def test_module_imports_without_pynput_installed():
    import real_world_rl.reward.collect_data as m

    assert hasattr(m, "Args")
    assert hasattr(m, "main")
