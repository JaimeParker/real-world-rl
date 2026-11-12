"""Fixtures shared across test modules."""
from __future__ import annotations

import pytest


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
