"""The collector entrypoint and pickle writer need no keyboard SDK at import time."""
from __future__ import annotations


def test_module_imports_without_pynput_installed():
    import real_world_rl.reward.collect_data as m

    assert callable(m.save_classifier_data)
    assert hasattr(m, "main")
