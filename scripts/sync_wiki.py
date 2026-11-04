#!/usr/bin/env python3
"""Build GitHub Wiki Markdown from repository docs using only the standard library."""

from __future__ import annotations

import argparse
import configparser
from pathlib import Path
import re
import sys
from urllib.parse import quote, unquote, urlsplit


BLOB_URL = "https://github.com/JaimeParker/real-world-rl/blob/main/"
# Inline destinations may contain balanced parentheses and an optional link title.
LINK = re.compile(
    r"(?<!!)\[(?:\\.|[^\[\]\\]|\[[^\[\]]*\])*\]"
    r"\(\s*(?P<target><[^>\n]+>|(?:\\.|[^\s()\\]|\([^()\n]*\))+)"
    r'''(?:\s+(?:"[^"\n]*"|'[^'\n]*'|\([^()\n]*\)))?\s*\)'''
)
FENCE = re.compile(r" {0,3}(?P<marker>`{3,}|~{3,})(?P<rest>[^\n]*)")
INLINE_CODE = re.compile(r"(`+)(?!`)(.+?)(?<!`)\1(?!`)", re.DOTALL)
PROSE_TOKEN = re.compile(f"(?:{INLINE_CODE.pattern})|(?:{LINK.pattern})", re.DOTALL)


def page_mapping(root: Path) -> dict[Path, str]:
    """Map the allowed source files, never walking the submodule or internal docs."""
    pages = {}

    def add(source: Path, name: str) -> None:
        if (root / source).is_file():
            if name in pages.values():
                raise ValueError(f"duplicate wiki page name: {name}")
            pages[source] = name

    add(Path("README.md"), "Home")
    for source in sorted((root / "docs").glob("*.md")):
        name = {"README": "docs", "README.zh-CN": "zh-docs"}.get(
            source.stem, source.stem
        )
        add(source.relative_to(root), name)
    for section in ("tutorial", "testing"):
        for source in sorted((root / "docs" / section).glob("*.md")):
            name = section if source.stem == "README" else f"{section}-{source.stem}"
            add(source.relative_to(root), name)
    add(Path("robot_infra/README.md"), "robot-infra")
    add(Path("robot_infra/docs/control-profiles.md"), "control-profiles")
    chinese_root = root / "docs/zh-CN"
    for source in sorted(chinese_root.rglob("*.md")):
        relative = source.relative_to(chinese_root)
        if {"internal", "design"}.intersection(relative.parts[:-1]):
            continue
        add(source.relative_to(root), "zh-" + "-".join(relative.with_suffix("").parts))
    return pages


def submodule_paths(root: Path) -> tuple[Path, ...]:
    """Submodule files belong to the repo even when checkout disables submodules."""
    config = configparser.ConfigParser(interpolation=None)
    config.read(root / ".gitmodules", encoding="utf-8")
    return tuple(
        (root / config[section]["path"]).resolve()
        for section in config.sections()
        if section.startswith("submodule ") and "path" in config[section]
    )


def rewrite_links(
    content: str, source: Path, root: Path, pages: dict[Path, str],
    submodules: tuple[Path, ...],
) -> str:
    """Rewrite inline Markdown links outside fenced/indented code and code spans."""
    def replace_link(match: re.Match[str]) -> str:
        original = match.group("target")
        target = original[1:-1] if original.startswith("<") else original
        parsed = urlsplit(target)
        if not target or target.startswith("#") or parsed.scheme or parsed.netloc:
            return match.group()
        destination = (root / source.parent / unquote(parsed.path)).resolve()
        try:
            relative = destination.relative_to(root)
        except ValueError:
            relative = None
        in_submodule = any(destination.is_relative_to(path) for path in submodules)
        if relative is None or (not destination.exists() and not in_submodule):
            print(f"warning: {source.as_posix()}: missing link target: {target}", file=sys.stderr)
            return match.group()
        suffix = ("?" + parsed.query if parsed.query else "")
        suffix += "#" + parsed.fragment if parsed.fragment else ""
        rewritten = pages.get(relative, BLOB_URL + quote(relative.as_posix(), safe="/"))
        if original.startswith("<"):
            rewritten = f"<{rewritten}{suffix}>"
        else:
            rewritten += suffix
        start, end = match.span("target")
        return match.group()[:start - match.start()] + rewritten + match.group()[end - match.start():]

    def rewrite_prose(prose: str) -> str:
        # Match whole links before any code spans inside their labels.
        return PROSE_TOKEN.sub(
            lambda match: replace_link(match) if match.group("target") else match.group(),
            prose,
        )

    result = []
    prose = []
    fence = ""
    for line in content.splitlines(keepends=True):
        marker = FENCE.fullmatch(line.rstrip("\r\n"))
        is_code = bool(fence or marker or line.startswith(("    ", "\t")))
        if is_code:
            result.append(rewrite_prose("".join(prose)))
            prose.clear()
            result.append(line)
        else:
            prose.append(line)
        if fence:
            if (marker and marker["marker"][0] == fence[0]
                    and len(marker["marker"]) >= len(fence) and not marker["rest"].strip()):
                fence = ""
        elif marker:
            fence = marker["marker"]
    result.append(rewrite_prose("".join(prose)))
    return "".join(result)


def generated_note(source: str, sha: str) -> str:
    return (
        f"> Generated from `{source}` at {sha[:7]}; "
        "edit the file in the repository, not this page.\n\n"
    )


def sidebar(pages: dict[Path, str], sha: str, labels: dict[str, str]) -> str:
    names = set(pages.values())
    ordered = ["Home", "docs", "getting-started-hardware"]
    ordered += sorted(name for name in names if name.startswith("tutorial-"))
    ordered += [
        "teleop", "gello", "recording-demos", "real-world-experiments",
        "residual", "silri", "known-issues", "hil-serl-roadmap",
    ]
    ordered += sorted(name for name in names if name == "testing" or name.startswith("testing-"))
    ordered += ["robot-infra", "control-profiles"]
    ordered += sorted(name for name in names if not name.startswith("zh-") and name not in ordered)
    lines = [generated_note("scripts/sync_wiki.py", sha).rstrip(), "", "## English", ""]
    for name in ordered:
        if name in names:
            lines.append(f"- [{labels.get(name, name)}]({name})")
    lines.extend(["", "## 中文 / Chinese", ""])
    lines.extend(
        f"- [{labels.get(name, name)}]({name})"
        for name in sorted(names) if name.startswith("zh-")
    )
    return "\n".join(lines) + "\n"


def render_pages(root: Path, sha: str) -> tuple[dict[Path, str], dict[str, str]]:
    pages = page_mapping(root)
    submodules = submodule_paths(root)
    rendered = {}
    labels = {}
    for source, name in pages.items():
        source_content = (root / source).read_text(encoding="utf-8")
        labels[name] = next(
            (line[2:].rstrip() for line in source_content.splitlines() if line.startswith("# ")),
            name,
        )
        content = rewrite_links(
            source_content, source, root, pages, submodules,
        )
        if name == "Home":
            content = content.rstrip() + "\n\n## Chinese translations\n\n[中文文档](zh-docs)\n"
        rendered[name] = generated_note(source.as_posix(), sha) + content
    rendered["_Sidebar"] = sidebar(pages, sha, labels)
    return pages, rendered


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "root", nargs="?", type=Path, default=Path(__file__).resolve().parents[1],
        help="repository root (default: this script's repository)",
    )
    parser.add_argument("--out", type=Path, help="output directory; replace all its Markdown files")
    parser.add_argument("--sha", default="local", help="source commit (default: local; shortened to 7 chars)")
    parser.add_argument("--check", action="store_true", help="print pages and warnings without writing files")
    args = parser.parse_args()
    if not args.check and args.out is None:
        parser.error("--out is required unless --check is used")
    root = args.root.resolve()
    pages, rendered = render_pages(root, args.sha)
    for source, name in sorted(pages.items(), key=lambda item: (item[1].startswith("zh-"), item[1])):
        print(f"{source.as_posix()} -> {name}.md")
    print("(generated sidebar) -> _Sidebar.md")
    if not args.check:
        out = args.out.resolve()
        if any((root / source).is_relative_to(out) for source in pages):
            parser.error("--out must not contain source documents")
        out.mkdir(parents=True, exist_ok=True)
        for old in out.rglob("*.md"):
            old.unlink()
        for name, content in rendered.items():
            (out / f"{name}.md").write_text(content, encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
