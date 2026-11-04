"""Wiki generation contracts, independent of installed packages and network access."""

from __future__ import annotations

from pathlib import Path
import subprocess
import sys

import pytest


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts/sync_wiki.py"
BLOB = "https://github.com/JaimeParker/real-world-rl/blob/main/"


@pytest.fixture
def wiki_repo(tmp_path: Path) -> Path:
    sources = {
        "README.md": "# Project\n\n[Documentation](docs/README.md)\n",
        "docs/README.md": "# Documentation\n",
        "docs/teleop.md": "# Teleop\n",
        "docs/tutorial/01-install.md": "# Install \t\n\n# Later heading\n",
        "docs/testing/README.md": (
            "# Testing\n\n[Prepare](../tutorial/01-install.md#prepare)\n"
            "[Record](x-record.csv)\n[Missing](absent.md)\n"
            "[External](https://example.com/a.md#b) [Mail](mailto:a@example.com) "
            "[Anchor](#here)\n\n"
            "```markdown\n[a](b.md)\n```\n\n"
            "~~~~markdown\n[a](b.md)\n```\n~~~~\n\n"
            "    [Indented](b.md)\n\n`[Inline](b.md)`\n"
        ),
        "docs/testing/x.md": "## Test X\n",
        "docs/testing/x-record.csv": "result\n",
        "docs/zh-CN/README.md": "# 中文索引\n",
        "docs/zh-CN/teleop.md": "# 遥操作\n",
        "docs/zh-CN/tutorial/01-install.md": "# 安装 \t\n\n# 后续标题\n",
        "docs/zh-CN/testing/x.md": "## 测试\n",
        "docs/README.zh-CN.md": "# 中文文档\n",
        "docs/internal/x.md": "# Private\n",
        "docs/design/x.md": "# Design\n",
        "robot_infra/README.md": "# Device layer\n",
        "robot_infra/docs/control-profiles.md": "# Control profiles\n",
        "rl-garden/rl_garden/models/act/README.md": "# Excluded\n",
    }
    repo = tmp_path / "repo"
    for source, text in sources.items():
        path = repo / source
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(text, encoding="utf-8")
    return repo


def run_sync(root: Path, *args: str) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        [sys.executable, str(SCRIPT), str(root), *args],
        capture_output=True, text=True, encoding="utf-8", check=False,
    )


def test_mapping_and_generated_navigation(wiki_repo: Path, tmp_path: Path) -> None:
    out = tmp_path / "wiki"
    result = run_sync(wiki_repo, "--out", str(out), "--sha", "123456789abcdef")
    assert result.returncode == 0, result.stderr
    expected = {
        "README.md": "Home",
        "docs/README.md": "docs",
        "docs/teleop.md": "teleop",
        "docs/tutorial/01-install.md": "tutorial-01-install",
        "docs/testing/README.md": "testing",
        "docs/testing/x.md": "testing-x",
        "robot_infra/README.md": "robot-infra",
        "robot_infra/docs/control-profiles.md": "control-profiles",
        "docs/README.zh-CN.md": "zh-docs",
        "docs/zh-CN/README.md": "zh-README",
        "docs/zh-CN/teleop.md": "zh-teleop",
        "docs/zh-CN/tutorial/01-install.md": "zh-tutorial-01-install",
        "docs/zh-CN/testing/x.md": "zh-testing-x",
    }
    assert {page.name for page in out.iterdir()} == {
        f"{name}.md" for name in [*expected.values(), "_Sidebar"]
    }
    for source, name in expected.items():
        assert f"{source} -> {name}.md" in result.stdout.splitlines()
        page = (out / f"{name}.md").read_text(encoding="utf-8")
        assert page.startswith(
            f"> Generated from `{source}` at 1234567; "
            "edit the file in the repository, not this page.\n"
        )
    home = (out / "Home.md").read_text(encoding="utf-8")
    assert "[Documentation](docs)" in home
    assert "## Chinese translations" in home
    assert "(zh-docs)" in home
    sidebar = (out / "_Sidebar.md").read_text(encoding="utf-8")
    english, chinese = sidebar.split("## 中文 / Chinese")
    english_order = [
        "Home", "docs", "tutorial-01-install", "teleop", "testing", "testing-x",
        "robot-infra", "control-profiles",
    ]
    positions = [english.index(f"]({name})") for name in english_order]
    assert positions == sorted(positions)
    assert "(zh-" not in english
    assert all(f"]({name})" in chinese for name in expected.values() if name.startswith("zh-"))
    assert "- [Install](tutorial-01-install)" in english
    assert "- [testing-x](testing-x)" in english
    assert "- [安装](zh-tutorial-01-install)" in chinese
    assert "- [zh-testing-x](zh-testing-x)" in chinese


def test_link_rewriting_and_warnings(wiki_repo: Path, tmp_path: Path) -> None:
    out = tmp_path / "wiki"
    result = run_sync(wiki_repo, "--out", str(out))
    assert result.returncode == 0, result.stderr
    content = (out / "testing.md").read_text(encoding="utf-8")
    assert "[Prepare](tutorial-01-install#prepare)" in content
    assert f"[Record]({BLOB}docs/testing/x-record.csv)" in content
    assert "[Missing](absent.md)" in content
    assert result.stderr.splitlines() == [
        "warning: docs/testing/README.md: missing link target: absent.md"
    ]
    assert "```markdown\n[a](b.md)\n```" in content
    assert "~~~~markdown\n[a](b.md)\n```\n~~~~" in content
    assert "    [Indented](b.md)" in content
    assert "`[Inline](b.md)`" in content
    assert "[External](https://example.com/a.md#b)" in content
    assert "[Mail](mailto:a@example.com)" in content
    assert "[Anchor](#here)" in content
    assert (out / "Home.md").read_text(encoding="utf-8").startswith(
        "> Generated from `README.md` at local;"
    )


def test_link_labels_titles_directories_and_submodule(wiki_repo: Path, tmp_path: Path) -> None:
    (wiki_repo / ".gitmodules").write_text(
        '[submodule "uninitialized"]\n path = uninitialized\n', encoding="utf-8",
    )
    (wiki_repo / "docs/teleop.md").write_text(
        '[`Install`](tutorial/01-install.md#prepare "Read this")\n'
        "[Docs](./) [CSV](testing/x-record.csv?raw=1#record)\n"
        "[Excluded](internal/x.md)\n"
        "[Submodule](../uninitialized/models/README.md)\n"
        "[Spaces](<testing/with%20space.csv>)\n"
        "[Parentheses](testing/data(1).csv)\n"
        "![Image](testing/x-record.csv)\n",
        encoding="utf-8",
    )
    for name in ("with space.csv", "data(1).csv"):
        (wiki_repo / "docs/testing" / name).touch()
    out = tmp_path / "wiki"
    result = run_sync(wiki_repo, "--out", str(out))
    assert result.returncode == 0, result.stderr
    assert "docs/teleop.md" not in result.stderr
    content = (out / "teleop.md").read_text(encoding="utf-8")
    assert '[`Install`](tutorial-01-install#prepare "Read this")' in content
    assert f"[Docs]({BLOB}docs)" in content
    assert f"[CSV]({BLOB}docs/testing/x-record.csv?raw=1#record)" in content
    assert f"[Excluded]({BLOB}docs/internal/x.md)" in content
    assert f"[Submodule]({BLOB}uninitialized/models/README.md)" in content
    assert f"[Spaces](<{BLOB}docs/testing/with%20space.csv>)" in content
    assert f"[Parentheses]({BLOB}docs/testing/data%281%29.csv)" in content
    assert "![Image](testing/x-record.csv)" in content


def test_rebuild_removes_stale_markdown_only(wiki_repo: Path, tmp_path: Path) -> None:
    out = tmp_path / "wiki"
    first = run_sync(wiki_repo, "--out", str(out))
    assert first.returncode == 0, first.stderr
    (out / "stale.md").touch()
    (out / "old").mkdir()
    (out / "old/stale.md").touch()
    (out / "keep.txt").write_text("keep", encoding="utf-8")
    (wiki_repo / "docs/teleop.md").unlink()
    second = run_sync(wiki_repo, "--out", str(out))
    assert second.returncode == 0, second.stderr
    assert not (out / "stale.md").exists()
    assert not (out / "old/stale.md").exists()
    assert not (out / "teleop.md").exists()
    assert (out / "keep.txt").read_text(encoding="utf-8") == "keep"


def test_check_writes_nothing(wiki_repo: Path, tmp_path: Path) -> None:
    out = tmp_path / "wiki"
    result = run_sync(wiki_repo, "--check", "--out", str(out))
    assert result.returncode == 0, result.stderr
    assert not out.exists()
    assert "README.md -> Home.md" in result.stdout.splitlines()
    assert "warning:" in result.stderr
    out.mkdir()
    (out / "stale.md").write_text("unchanged", encoding="utf-8")
    result = run_sync(wiki_repo, "--check", "--out", str(out))
    assert result.returncode == 0, result.stderr
    assert (out / "stale.md").read_text(encoding="utf-8") == "unchanged"
    assert [path.name for path in out.iterdir()] == ["stale.md"]


def test_output_cannot_delete_sources(wiki_repo: Path) -> None:
    result = run_sync(wiki_repo, "--out", str(wiki_repo))
    assert result.returncode != 0
    assert "--out must not contain source documents" in result.stderr
    assert (wiki_repo / "README.md").is_file()


def test_real_repository_check_has_no_link_warnings() -> None:
    result = subprocess.run(
        [sys.executable, str(SCRIPT), "--check"], cwd=ROOT,
        capture_output=True, text=True, encoding="utf-8", check=False,
    )
    assert result.returncode == 0, result.stderr
    assert "README.md -> Home.md" in result.stdout.splitlines()
    assert "(generated sidebar) -> _Sidebar.md" in result.stdout.splitlines()
    assert result.stderr == "", result.stderr
