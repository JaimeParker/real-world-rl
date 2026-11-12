#!/usr/bin/env python3
"""Build GitHub Wiki Markdown from repository docs using only the standard library."""

from __future__ import annotations

import argparse
import configparser
import functools
import json
from pathlib import Path
import re
import sys
import unicodedata
from typing import Iterator
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
ATX_HEADING = re.compile(r" {0,3}(#{1,6})(?:[ \t]+(.*?))?(?:[ \t]+#+)?[ \t]*")
HEADING_LINK_OR_IMAGE = re.compile(r"!?\[([^\]]*)\]\([^)]*\)")
HTML_TAG = re.compile(r"</?[A-Za-z][^>]*>")
EXPLICIT_ANCHOR = re.compile(r"""<a\s[^>]*?\b(?:id|name)\s*=\s*["']([^"']+)["']""")
TRANSLATION_MANIFEST = Path("docs/zh-CN/translation-manifest.json")

# Sidebar sections. A page lands in the first section that lists it; English pages that no
# section lists go to "Other", so a new page can never silently vanish from navigation.
START_HERE_PAGES = ("Home", "docs")
REFERENCE_PAGES = (
    "cli", "doctor", "devices", "daemon", "teleop", "gello", "spacemouse-commissioning", "bimanual",
    "control-profiles", "recording-demos", "real-world-experiments",
    "residual", "silri", "hil-serl-roadmap", "known-issues",
)


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


def unfenced_lines(content: str) -> Iterator[str]:
    """Lines outside fenced code blocks, fence lines excluded."""
    fence = ""
    for line in content.splitlines():
        marker = FENCE.fullmatch(line)
        if fence:
            if (marker and marker["marker"][0] == fence[0]
                    and len(marker["marker"]) >= len(fence) and not marker["rest"].strip()):
                fence = ""
        elif marker:
            fence = marker["marker"]
        else:
            yield line


def headings(content: str) -> Iterator[tuple[int, str]]:
    """ATX headings outside fenced code as (level, raw text), in document order."""
    for line in unfenced_lines(content):
        match = ATX_HEADING.fullmatch(line)
        if match:
            yield len(match[1]), match[2] or ""


def page_title(content: str, fallback: str) -> str:
    """The first H1 outside code, or the fallback (the wiki page name) when there is none."""
    return next((text for level, text in headings(content) if level == 1 and text), fallback)


def github_slug(heading: str) -> str:
    """GitHub's heading anchor: markup dropped, lowercased, only letters, digits, combining
    marks, '_' and '-' kept (so Chinese stays), each space becoming '-'."""
    text = HEADING_LINK_OR_IMAGE.sub(r"\1", heading)
    text = HTML_TAG.sub("", text)
    text = re.sub(r"\\(.)", r"\1", text).lower()
    kept = (
        char for char in text
        if char in " _-" or unicodedata.category(char)[0] in "LNM"
    )
    return "".join(kept).replace(" ", "-")


@functools.lru_cache(maxsize=None)
def anchors_in_file(path: Path) -> frozenset[str]:
    """Anchors a link into this Markdown file can hit: heading slugs (a repeated slug gets
    -1, -2, ... in document order) and explicit <a id="..."> / <a name="..."> anchors."""
    content = path.read_text(encoding="utf-8")
    anchors = set()
    occurrences: dict[str, int] = {}
    for _, heading in headings(content):
        base = slug = github_slug(heading)
        while slug in occurrences:
            occurrences[base] += 1
            slug = f"{base}-{occurrences[base]}"
        occurrences[slug] = 0
        anchors.add(slug)
    for line in unfenced_lines(content):
        anchors.update(EXPLICIT_ANCHOR.findall(line))
    return frozenset(anchors)


def rewrite_links(
    content: str, source: Path, root: Path, pages: dict[Path, str],
    submodules: tuple[Path, ...], missing_anchors: list[str],
) -> str:
    """Rewrite inline Markdown links outside fenced/indented code and code spans.

    A link into an existing Markdown file (or "#anchor" within this one) whose anchor that file
    does not define is warned about and appended to ``missing_anchors``."""
    def check_anchor(destination: Path, path_as_written: str, fragment: str) -> None:
        anchor = unquote(fragment)
        if not anchor or destination.suffix != ".md" or not destination.is_file():
            return
        if anchor not in anchors_in_file(destination):
            message = (
                f"warning: {source.as_posix()}: missing anchor target: "
                f"{path_as_written}#{fragment}"
            )
            print(message, file=sys.stderr)
            missing_anchors.append(message)

    def replace_link(match: re.Match[str]) -> str:
        original = match.group("target")
        target = original[1:-1] if original.startswith("<") else original
        parsed = urlsplit(target)
        if not target or parsed.scheme or parsed.netloc:
            return match.group()
        if target.startswith("#"):
            check_anchor((root / source).resolve(), "", parsed.fragment)
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
        check_anchor(destination, parsed.path, parsed.fragment)
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


def translation_map(root: Path) -> dict[Path, Path]:
    """English source file -> Chinese translation file, from the translation manifest."""
    manifest = root / TRANSLATION_MANIFEST
    if not manifest.is_file():
        return {}
    documents = json.loads(manifest.read_text(encoding="utf-8"))["documents"]
    return {Path(item["source"]): Path(item["translation"]) for item in documents}


def english_sidebar_sections(names: set[str]) -> list[tuple[str, str, list[str]]]:
    """(English title, Chinese title, English page names) per non-empty section, in order."""
    english = {name for name in names if not name.startswith("zh-")}
    start_here = [name for name in START_HERE_PAGES if name in english]
    tutorial = sorted(name for name in english if name == "tutorial" or name.startswith("tutorial-"))
    reference = [name for name in REFERENCE_PAGES if name in english]
    testing = sorted(name for name in english if name == "testing" or name.startswith("testing-"))
    other = sorted(english - {*start_here, *tutorial, *reference, *testing})
    sections = [
        ("Start here", "从这里开始", start_here),
        ("Tutorial", "教程", tutorial),
        ("Reference", "参考", reference),
        ("Testing", "测试", testing),
        ("Other", "其他", other),
    ]
    return [section for section in sections if section[2]]


def chinese_sidebar_sections(
    english_sections: list[tuple[str, str, list[str]]], pages: dict[Path, str],
    translations: dict[Path, Path],
) -> list[tuple[str, list[str]]]:
    """The English sections with each page replaced by its translation (per the manifest);
    Chinese pages that are no page's translation are appended to the last section, "其他"."""
    source_of = {name: source for source, name in pages.items()}
    placed: set[str] = set()
    sections = []
    for _, title, members in english_sections:
        translated = []
        for name in members:
            chinese = pages.get(translations.get(source_of[name]))
            if chinese is not None and chinese not in placed:
                translated.append(chinese)
                placed.add(chinese)
        sections.append((title, translated))
    unmapped = sorted(
        name for name in pages.values() if name.startswith("zh-") and name not in placed
    )
    if unmapped:
        if sections and sections[-1][0] == "其他":
            sections[-1][1].extend(unmapped)
        else:
            sections.append(("其他", unmapped))
    return [section for section in sections if section[1]]


def sidebar(
    pages: dict[Path, str], sha: str, labels: dict[str, str], translations: dict[Path, Path],
) -> str:
    english = english_sidebar_sections(set(pages.values()))
    lines = [generated_note("scripts/sync_wiki.py", sha).rstrip(), "", "## English"]
    for title, _, members in english:
        lines.extend(["", f"### {title}", ""])
        lines.extend(f"- [{labels.get(name, name)}]({name})" for name in members)
    lines.extend(["", "## 中文 / Chinese"])
    for title, members in chinese_sidebar_sections(english, pages, translations):
        lines.extend(["", f"### {title}", ""])
        lines.extend(f"- [{labels.get(name, name)}]({name})" for name in members)
    return "\n".join(lines) + "\n"


def render_pages(root: Path, sha: str) -> tuple[dict[Path, str], dict[str, str], list[str]]:
    pages = page_mapping(root)
    submodules = submodule_paths(root)
    rendered = {}
    labels = {}
    missing_anchors: list[str] = []
    for source, name in pages.items():
        source_content = (root / source).read_text(encoding="utf-8")
        labels[name] = page_title(source_content, name)
        content = rewrite_links(
            source_content, source, root, pages, submodules, missing_anchors,
        )
        if name == "Home":
            content = content.rstrip() + "\n\n## Chinese translations\n\n[中文文档](zh-docs)\n"
        rendered[name] = generated_note(source.as_posix(), sha) + content
    rendered["_Sidebar"] = sidebar(pages, sha, labels, translation_map(root))
    return pages, rendered, missing_anchors


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "root", nargs="?", type=Path, default=Path(__file__).resolve().parents[1],
        help="repository root (default: this script's repository)",
    )
    parser.add_argument("--out", type=Path, help="output directory; replace all its Markdown files")
    parser.add_argument("--sha", default="local", help="source commit (default: local; shortened to 7 chars)")
    parser.add_argument(
        "--check", action="store_true",
        help="print pages and warnings without writing files; exit 1 if a link names a missing anchor",
    )
    args = parser.parse_args()
    if not args.check and args.out is None:
        parser.error("--out is required unless --check is used")
    root = args.root.resolve()
    pages, rendered, missing_anchors = render_pages(root, args.sha)
    for source, name in sorted(pages.items(), key=lambda item: (item[1].startswith("zh-"), item[1])):
        print(f"{source.as_posix()} -> {name}.md")
    print("(generated sidebar) -> _Sidebar.md")
    if args.check:
        return 1 if missing_anchors else 0
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
