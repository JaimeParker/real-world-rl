"""CLI compatibility for the RST pair checker at its historical script path."""
from pathlib import Path
import subprocess
import sys

import pytest

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / 'scripts/check_zh_manifest.py'
MANIFEST = Path('docs/translation-manifest.json')


def run_checker(root, *flags):
    return subprocess.run([sys.executable, str(SCRIPT), str(root), *flags],
                          capture_output=True, text=True, check=False)


@pytest.fixture
def tree(tmp_path):
    for name in ('README.md', 'README.zh-CN.md', 'docs/index.rst', 'docs/en/index.rst'):
        path = tmp_path / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text('original\n')
    result = run_checker(tmp_path, '--update', '--strict')
    assert result.returncode == 0, result.stderr
    return tmp_path


def test_warning_and_strict_modes_do_not_rewrite_manifest(tree):
    original = (tree / MANIFEST).read_bytes()
    (tree / 'README.md').write_text('changed\n')
    assert run_checker(tree).returncode == 0
    strict = run_checker(tree, '--strict')
    assert strict.returncode == 1
    assert 'stale:' in strict.stdout
    assert (tree / MANIFEST).read_bytes() == original


def test_update_is_idempotent(tree):
    original = (tree / MANIFEST).read_bytes()
    assert run_checker(tree, '--update', '--strict').returncode == 0
    assert (tree / MANIFEST).read_bytes() == original


def test_missing_counterpart_prevents_update(tree):
    original = (tree / MANIFEST).read_bytes()
    (tree / 'docs/en/index.rst').unlink()
    result = run_checker(tree, '--update', '--strict')
    assert result.returncode == 1
    assert 'missing counterpart: docs/en/index.rst' in result.stdout
    assert (tree / MANIFEST).read_bytes() == original


def test_repository_manifest():
    result = run_checker(ROOT, '--strict')
    assert result.returncode == 0, result.stdout + result.stderr
