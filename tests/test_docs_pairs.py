"""Pair discovery and hash tracking without learning dependencies."""
import importlib.util
from pathlib import Path

import pytest

SPEC = importlib.util.spec_from_file_location(
    'docs_pairs', Path(__file__).resolve().parents[1] / 'scripts/check_zh_manifest.py')
checker = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(checker)


@pytest.fixture
def tree(tmp_path):
    for name in ('README.md', 'README.zh-CN.md', 'docs/index.rst', 'docs/en/index.rst',
                 'docs/reference/a.rst', 'docs/en/reference/a.rst',
                 'docs/examples/a.sh', 'docs/examples/en/a.sh'):
        path = tmp_path / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text('original\n')
    assert checker.check_manifest(tmp_path, update=True) == []
    return tmp_path


@pytest.mark.parametrize('name', ['docs/reference/a.rst', 'docs/en/reference/a.rst',
                                  'docs/examples/a.sh', 'README.zh-CN.md'])
def test_missing_counterpart(tree, name):
    (tree / name).unlink()
    assert any(name in issue for issue in checker.check_manifest(tree))


@pytest.mark.parametrize('name', ['docs/index.rst', 'docs/en/index.rst', 'README.md'])
def test_one_sided_change_and_update(tree, name):
    (tree / name).write_text('changed\n')
    assert any('stale:' in issue for issue in checker.check_manifest(tree))
    assert checker.main([str(tree)]) == 0
    assert checker.main([str(tree), '--strict']) == 1
    assert checker.main([str(tree), '--update', '--strict']) == 0
    assert checker.check_manifest(tree) == []


def test_both_changed(tree):
    for name in ('docs/index.rst', 'docs/en/index.rst'):
        (tree / name).write_text('changed\n')
    assert checker.check_manifest(tree) == []
