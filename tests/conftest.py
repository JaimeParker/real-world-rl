"""Fixtures shared across test modules."""
from __future__ import annotations

from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture
def doctor_report_dir(tmp_path, monkeypatch):
    """Doctor reports go to a private temporary directory instead of the checkout or user state."""
    directory = tmp_path / "reports"
    monkeypatch.setenv("RWRL_DOCTOR_REPORT_DIR", str(directory))
    return directory


@pytest.fixture(autouse=True)
def no_station_from_environment(monkeypatch):
    """A station named by the host's environment (current or pre-0.1.0 variable) never leaks into a test."""
    for name in ("RWRL_STATION", "ROBOT_INFRA_CONFIG"):
        monkeypatch.delenv(name, raising=False)


@pytest.fixture(autouse=True)
def no_rwrl_use_selection(tmp_path_factory, monkeypatch):
    """A selection made with ``rwrl use`` in the checkout never leaks into a test; tests that
    exercise ``rwrl use`` chdir into their own directory."""
    from real_world_rl.common import station_paths

    isolated = tmp_path_factory.mktemp("rwrl-use")
    original = station_paths.selection_file

    def selection_file(cwd=None):
        return original(isolated if cwd is None and Path.cwd() == REPO_ROOT else cwd)

    monkeypatch.setattr(station_paths, "selection_file", selection_file)
    monkeypatch.setattr(station_paths, "_announced", {})
