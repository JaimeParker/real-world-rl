"""The robot-PC install script for franky + serl_controller: shell syntax and pins (no build)."""
from __future__ import annotations

import re
import shutil
import subprocess
from pathlib import Path

import pytest

NATIVE = Path(__file__).resolve().parents[1] / "native" / "serl_controller"
SCRIPT = NATIVE / "scripts" / "install-on-ubuntu.sh"
DOCKER = NATIVE / "docker" / "build-on-host.sh"


def _pin(path: Path, name: str) -> str:
    match = re.search(rf'^{name}="\$\{{{name}:-([^}}]+)\}}"', path.read_text(), re.MULTILINE)
    assert match, f"{path} has no {name} default"
    return match.group(1)


@pytest.mark.skipif(shutil.which("bash") is None, reason="needs bash")
def test_script_parses():
    subprocess.run(["bash", "-n", str(SCRIPT)], check=True)


def test_script_pins_the_same_unmodified_franky_as_the_docker_recipe():
    for name in ("FRANKY_TAG", "FRANKY_COMMIT", "FRANKY_REPO"):
        assert _pin(SCRIPT, name) == _pin(DOCKER, name)


def test_script_pins_franky_pybind11():
    text = SCRIPT.read_text()
    pyproject = (NATIVE / "pyproject.toml").read_text()
    pinned = re.search(r'"pybind11==([0-9.]+)"', pyproject).group(1)
    assert f'PYBIND11_VERSION_PINNED="{pinned}"' in text


def test_script_requires_a_libfranka_version_and_an_environment():
    text = SCRIPT.read_text()
    assert '[ -n "${LIBFRANKA_VERSION}" ] || die' in text
    assert "never build into the system Python" in text
    assert "REPLACE_LIBFRANKA" in text  # never silently replaces another installed libfranka
