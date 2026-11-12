"""Wiki generation contracts, independent of installed packages and network access."""

from __future__ import annotations

import json
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
        "docs/tutorial/01-install.md": "# Install \t\n\n# Later heading\n\n## Prepare\n",
        "docs/testing/README.md": (
            "# Testing\n\n[Prepare](../tutorial/01-install.md#prepare)\n"
            "[Record](x-record.csv)\n[Missing](absent.md)\n"
            "[External](https://example.com/a.md#b) [Mail](mailto:a@example.com) "
            "[Anchor](#here)\n\n"
            "```markdown\n[a](b.md)\n```\n\n"
            "~~~~markdown\n[a](b.md)\n```\n~~~~\n\n"
            "    [Indented](b.md)\n\n`[Inline](b.md)`\n\n## Here\n"
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
        "docs/devices.md": "# Device layer\n",
        "docs/daemon.md": "# Daemon\n",
        "docs/control-profiles.md": "# Control profiles\n",
        "docs/devices-roadmap.md": "# Device roadmap\n",
        "real_world_rl/legacy/controller/README.md": "# Excluded package notes\n",
        "rl-garden/rl_garden/models/act/README.md": "# Excluded\n",
    }
    return build_repo(tmp_path / "repo", sources)


def build_repo(repo: Path, sources: dict[str, str]) -> Path:
    for source, text in sources.items():
        path = repo / source
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(text, encoding="utf-8")
    return repo


def sidebar_sections(sidebar: str) -> dict[tuple[str, str], list[str]]:
    """(language heading, section heading) -> page names, in sidebar order."""
    sections: dict[tuple[str, str], list[str]] = {}
    language = section = ""
    for line in sidebar.splitlines():
        if line.startswith("### "):
            section = line[4:]
            sections[(language, section)] = []
        elif line.startswith("## "):
            language = line[3:]
        elif line.startswith("- ["):
            sections[(language, section)].append(line.rsplit("](", 1)[1].rstrip(")"))
    return sections


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
        "docs/devices.md": "devices",
        "docs/daemon.md": "daemon",
        "docs/devices-roadmap.md": "devices-roadmap",
        "docs/control-profiles.md": "control-profiles",
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
        "Home", "docs", "tutorial-01-install", "devices", "daemon", "teleop", "control-profiles",
        "testing", "testing-x", "devices-roadmap",
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


REFERENCE_ORDER = [
    "cli", "doctor", "devices", "daemon", "teleop", "gello", "spacemouse-commissioning", "bimanual",
    "control-profiles", "recording-demos", "real-world-experiments", "residual", "silri",
    "hil-serl-roadmap", "known-issues",
]


def test_sidebar_is_grouped_in_reading_order(tmp_path: Path) -> None:
    sources = {
        "README.md": "# Project\n",
        "docs/README.md": "# Documentation\n",
        "docs/getting-started-hardware.md": "# Old entry\n",
        "docs/zz-unlisted.md": "# Unlisted\n",
        "docs/tutorial/02-setup.md": "# 2. Set up\n",
        "docs/tutorial/00-overview.md": "# 0. Overview\n",
        "docs/tutorial/01-install.md": "# 1. Install\n",
        "docs/testing/README.md": "# Testing\n",
        "docs/testing/a-session.md": "# Session\n",
        "docs/devices.md": "# Device layer\n",
        "docs/daemon.md": "# Daemon\n",
        "docs/control-profiles.md": "# Control profiles\n",
    }
    for name in REFERENCE_ORDER:
        if name not in ("devices", "control-profiles"):
            sources[f"docs/{name}.md"] = f"# {name}\n"
    repo = build_repo(tmp_path / "repo", sources)
    out = tmp_path / "wiki"
    result = run_sync(repo, "--out", str(out))
    assert result.returncode == 0, result.stderr
    sections = sidebar_sections((out / "_Sidebar.md").read_text(encoding="utf-8"))
    assert {key: names for key, names in sections.items() if key[0] == "English"} == {
        ("English", "Start here"): ["Home", "docs"],
        ("English", "Tutorial"): [
            "tutorial-00-overview", "tutorial-01-install", "tutorial-02-setup",
        ],
        ("English", "Reference"): REFERENCE_ORDER,
        ("English", "Testing"): ["testing", "testing-a-session"],
        ("English", "Other"): ["getting-started-hardware", "zz-unlisted"],
    }
    assert list(sections)[:5] == [
        ("English", "Start here"), ("English", "Tutorial"), ("English", "Reference"),
        ("English", "Testing"), ("English", "Other"),
    ]


def test_sidebar_labels_come_from_the_first_h1_outside_code(tmp_path: Path) -> None:
    repo = build_repo(tmp_path / "repo", {
        "docs/teleop.md": "```bash\n# a shell comment\n```\n\n## Section\n\n# Real title #\n\n# Second\n",
        "docs/gello.md": "Text before.\n\n#hashtag is not a heading\n\n## Only a section\n",
        "docs/tutorial/01-install.md": "# 1. Install the stack\n",
    })
    out = tmp_path / "wiki"
    result = run_sync(repo, "--out", str(out))
    assert result.returncode == 0, result.stderr
    sidebar = (out / "_Sidebar.md").read_text(encoding="utf-8")
    assert "- [Real title](teleop)" in sidebar
    assert "- [gello](gello)" in sidebar  # no H1: falls back to the wiki page name
    assert "- [1. Install the stack](tutorial-01-install)" in sidebar


def test_chinese_sidebar_follows_the_english_order_through_the_manifest(tmp_path: Path) -> None:
    manifest = {
        "language": "zh-CN",
        "documents": [
            {"source": "README.md", "translation": "docs/zh-CN/project-readme.md"},
            {"source": "docs/README.md", "translation": "docs/zh-CN/README.md"},
            {"source": "docs/tutorial/00-overview.md", "translation": "docs/zh-CN/tutorial/00-overview.md"},
            {"source": "docs/tutorial/01-install.md", "translation": "docs/zh-CN/tutorial/01-install.md"},
            {"source": "docs/cli.md", "translation": "docs/zh-CN/cli.md"},
            {"source": "docs/teleop.md", "translation": "docs/zh-CN/teleop.md"},
            {"source": "docs/testing/x.md", "translation": "docs/zh-CN/testing/x.md"},
            {"source": "docs/testing/x-record.csv", "translation": "docs/zh-CN/testing/x-record.csv"},
            {"source": "docs/getting-started-hardware.md", "translation": "docs/zh-CN/getting-started-hardware.md"},
        ],
    }
    repo = build_repo(tmp_path / "repo", {
        "README.md": "# Project\n",
        "docs/README.md": "# Documentation\n",
        "docs/README.zh-CN.md": "# 中文文档\n",
        "docs/cli.md": "# CLI\n",
        "docs/teleop.md": "# Teleop\n",
        "docs/getting-started-hardware.md": "# Old entry\n",
        "docs/tutorial/00-overview.md": "# 0. Overview\n",
        "docs/tutorial/01-install.md": "# 1. Install\n",
        "docs/testing/x.md": "# X\n",
        "docs/testing/x-record.csv": "result\n",
        "docs/zh-CN/project-readme.md": "# 项目\n",
        "docs/zh-CN/README.md": "# 文档索引\n",
        "docs/zh-CN/cli.md": "# 命令行\n",
        "docs/zh-CN/teleop.md": "# 遥操作\n",
        "docs/zh-CN/getting-started-hardware.md": "# 旧入口\n",
        "docs/zh-CN/tutorial/00-overview.md": "# 0. 总览\n",
        "docs/zh-CN/tutorial/01-install.md": "# 1. 安装\n",
        "docs/zh-CN/testing/x.md": "# 测试 X\n",
        "docs/zh-CN/act.md": "# ACT\n",
        "docs/zh-CN/translation-manifest.json": json.dumps(manifest),
    })
    out = tmp_path / "wiki"
    result = run_sync(repo, "--out", str(out))
    assert result.returncode == 0, result.stderr
    sidebar = (out / "_Sidebar.md").read_text(encoding="utf-8")
    sections = {key: names for key, names in sidebar_sections(sidebar).items() if key[0] != "English"}
    assert sections == {
        ("中文 / Chinese", "从这里开始"): ["zh-project-readme", "zh-README"],
        ("中文 / Chinese", "教程"): ["zh-tutorial-00-overview", "zh-tutorial-01-install"],
        ("中文 / Chinese", "参考"): ["zh-cli", "zh-teleop"],
        ("中文 / Chinese", "测试"): ["zh-testing-x"],
        # The translation of an "Other" page, then the Chinese pages that translate nothing.
        ("中文 / Chinese", "其他"): ["zh-getting-started-hardware", "zh-act", "zh-docs"],
    }
    assert list(sections) == [
        ("中文 / Chinese", title) for title in ("从这里开始", "教程", "参考", "测试", "其他")
    ]
    assert "- [1. 安装](zh-tutorial-01-install)" in sidebar


def test_chinese_sidebar_without_a_manifest_lists_every_page_under_other(wiki_repo: Path, tmp_path: Path) -> None:
    out = tmp_path / "wiki"
    result = run_sync(wiki_repo, "--out", str(out))
    assert result.returncode == 0, result.stderr
    sections = sidebar_sections((out / "_Sidebar.md").read_text(encoding="utf-8"))
    chinese = {key: names for key, names in sections.items() if key[0] != "English"}
    assert chinese == {("中文 / Chinese", "其他"): sorted(
        ["zh-README", "zh-docs", "zh-teleop", "zh-testing-x", "zh-tutorial-01-install"]
    )}


ANCHOR_HEADINGS = """# Anchors

## Hello, World!

## 4. Install the project environment

## The env preset (`hil.yaml`)

## Setup / Wizard

## Leader arm (`leader_cartesian`)

## [Link text](teleop.md) and *emphasis*

## 安装 与 配置

## 括号（全角）示例

## Dup

## Dup

## Dup

## A

## A

## A-1

## C# & C++

## Pre-flight check

## Closed ##

<a id="explicit-anchor"></a>

```markdown
## Not a heading
```
"""

# The GitHub anchor of each heading above, in order, then the explicit anchor.
GITHUB_ANCHORS = [
    "anchors", "hello-world", "4-install-the-project-environment", "the-env-preset-hilyaml",
    "setup--wizard", "leader-arm-leader_cartesian", "link-text-and-emphasis", "安装-与-配置",
    "括号全角示例", "dup", "dup-1", "dup-2", "a", "a-1", "a-1-1", "c--c", "pre-flight-check",
    "closed", "explicit-anchor",
]


def anchor_repo(tmp_path: Path, linking_page: str) -> Path:
    return build_repo(tmp_path / "repo", {
        "docs/anchors.md": ANCHOR_HEADINGS,
        "docs/teleop.md": "# Teleop\n",
        "docs/internal/x.md": "# Private\n",
        "docs/linking.md": linking_page,
        "docs/data.csv": "result\n",
    })


def test_anchor_check_accepts_github_anchors(tmp_path: Path) -> None:
    links = [f"[{anchor}](anchors.md#{anchor})" for anchor in GITHUB_ANCHORS]
    links += [
        "[percent-encoded](anchors.md#%E5%AE%89%E8%A3%85-%E4%B8%8E-%E9%85%8D%E7%BD%AE)",
        "[same page](#linking)",
        "[same page, later heading](#second-heading)",
        "[outside the wiki pages](internal/x.md#private)",
        "[not a Markdown file](data.csv#row-3)",
        "[titled](anchors.md#hello-world \"Title\")",
        "`[in code](anchors.md#absent)`",
    ]
    page = "# Linking\n\n" + "\n".join(links) + "\n\n## Second heading\n"
    repo = anchor_repo(tmp_path, page)
    result = run_sync(repo, "--check")
    assert result.returncode == 0, result.stderr
    assert result.stderr == ""
    out = tmp_path / "wiki"
    assert run_sync(repo, "--out", str(out)).returncode == 0
    assert "[hello-world](anchors#hello-world)" in (out / "linking.md").read_text(encoding="utf-8")


def test_anchor_check_reports_missing_anchors_and_fails_check_only(tmp_path: Path) -> None:
    page = (
        "# Linking\n\n"
        "[case matters](anchors.md#Hello-World)\n"
        "[heading in a code block](anchors.md#not-a-heading)\n"
        "[third duplicate is dup-2](anchors.md#dup-3)\n"
        "[same page](#nowhere)\n"
        "[outside the wiki pages](internal/x.md#missing)\n"
        "[fine](anchors.md#dup-2)\n"
    )
    repo = anchor_repo(tmp_path, page)
    expected = [
        "warning: docs/linking.md: missing anchor target: anchors.md#Hello-World",
        "warning: docs/linking.md: missing anchor target: anchors.md#not-a-heading",
        "warning: docs/linking.md: missing anchor target: anchors.md#dup-3",
        "warning: docs/linking.md: missing anchor target: #nowhere",
        "warning: docs/linking.md: missing anchor target: internal/x.md#missing",
    ]
    checked = run_sync(repo, "--check")
    assert checked.returncode == 1
    assert checked.stderr.splitlines() == expected
    assert "docs/linking.md -> linking.md" in checked.stdout.splitlines()
    out = tmp_path / "wiki"
    generated = run_sync(repo, "--out", str(out))
    assert generated.returncode == 0
    assert generated.stderr.splitlines() == expected
    assert (out / "linking.md").is_file()


def test_missing_link_target_alone_does_not_fail_check(tmp_path: Path) -> None:
    repo = anchor_repo(tmp_path, "# Linking\n\n[gone](absent.md#anchor)\n")
    result = run_sync(repo, "--check")
    assert result.returncode == 0
    assert result.stderr.splitlines() == [
        "warning: docs/linking.md: missing link target: absent.md#anchor"
    ]
