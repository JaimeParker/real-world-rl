#!/usr/bin/env python3
"""Warn about stale Chinese translations; optionally update source hashes or fail CI."""

from __future__ import annotations

import argparse
import hashlib
import json
import sys
import re
from collections.abc import Iterator
from pathlib import Path

MANIFEST = Path("docs/zh-CN/translation-manifest.json")
DECODER = json.JSONDecoder()
FIELD_SEPARATOR = re.compile(r"[\s,]*")
COLON = re.compile(r"\s*:\s*")


def object_fields(text: str, start: int) -> Iterator[tuple[str, int, int]]:
    """Yield JSON object field value spans, preserving all original formatting."""
    cursor = start + 1  # opening brace
    while True:
        cursor = FIELD_SEPARATOR.match(text, cursor).end()
        if text[cursor] == "}":
            return
        key, cursor = DECODER.raw_decode(text, cursor)
        cursor = COLON.match(text, cursor).end()
        _, end = DECODER.raw_decode(text, cursor)
        yield key, cursor, end
        cursor = end


def hash_spans(text: str) -> list[tuple[int, int]]:
    """Find only documents[*].source_sha256 values, not similarly named metadata."""
    start = next(start for key, start, _ in object_fields(text, len(text) - len(text.lstrip()))
                 if key == "documents")
    cursor = start + 1  # opening bracket
    spans = []
    while True:
        cursor = FIELD_SEPARATOR.match(text, cursor).end()
        if text[cursor] == "]":
            return spans
        spans.append(next((start, end) for key, start, end in object_fields(text, cursor)
                          if key == "source_sha256"))
        _, cursor = DECODER.raw_decode(text, cursor)


def submodule_not_checked_out(root: Path, source: str) -> bool:
    """Whether ``source`` lies under a git submodule whose checkout is absent (an uninitialised
    submodule, or a synced copy of the repository without it); such a source is skipped, not
    reported missing."""
    first = source.split("/", 1)[0]
    gitmodules = root / ".gitmodules"
    if not gitmodules.is_file() or f"path = {first}" not in gitmodules.read_text(encoding="utf-8"):
        return False
    submodule = root / first
    return not submodule.is_dir() or not any(submodule.iterdir())


def check_manifest(root: Path, *, update: bool = False) -> list[str]:
    """Check sources/translations and update existing sources when requested.

    Updates replace only hash value tokens: indentation, newlines, Unicode escapes,
    key order, and unrelated metadata remain byte-for-byte unchanged.
    """
    manifest = root / MANIFEST
    original = manifest.read_bytes().decode("utf-8")
    documents = json.loads(original)["documents"]
    issues = []
    replacements = []
    spans = hash_spans(original) if update else []
    for index, document in enumerate(documents):
        source = root / document["source"]
        translation = root / document["translation"]
        if not translation.is_file():
            issues.append(f"missing translation: {document['translation']} (source: {document['source']})")
        if not source.is_file():
            if submodule_not_checked_out(root, document["source"]):
                print(f"note: {document['source']} is in a submodule that is not checked out; skipped",
                      file=sys.stderr)
                continue
            issues.append(f"missing source: {document['source']}")
            continue
        digest = hashlib.sha256(source.read_bytes()).hexdigest()
        if document["source_sha256"] != digest:
            if update:
                start, end = spans[index]
                replacements.append((start, end, json.dumps(digest)))
                print(f"updated: {document['source']}")
            else:
                issues.append(f"stale: {document['source']} -> {document['translation']}")
    if replacements:
        result = original
        for start, end, value in reversed(replacements):
            result = result[:start] + value + result[end:]
        manifest.write_bytes(result.encode("utf-8"))
    return issues


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("root", nargs="?", type=Path,
                        default=Path(__file__).resolve().parents[1],
                        help="repository root (default: this script's repository)")
    parser.add_argument("--strict", action="store_true",
                        help="exit 1 for stale sources or missing files (default: warn only)")
    parser.add_argument("--update", action="store_true",
                        help="rewrite source hashes, preserving JSON formatting and key order")
    args = parser.parse_args(argv)
    try:
        issues = check_manifest(args.root, update=args.update)
    except (OSError, ValueError, KeyError, TypeError, StopIteration, AttributeError) as error:
        parser.exit(2, f"manifest error: {error}\n")
    for issue in issues:
        print(f"warning: {issue}")
    print(f"Chinese manifest: {len(issues)} issue(s)")
    return 1 if args.strict and issues else 0


if __name__ == "__main__":
    raise SystemExit(main())
