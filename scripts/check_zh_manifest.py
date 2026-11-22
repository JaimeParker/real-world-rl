#!/usr/bin/env python3
"""Check bilingual RST, example and README pairs; record hashes with --update.

Exactly one changed side is stale. By default issues warn; --strict fails.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

MANIFEST = Path('docs/translation-manifest.json')


def discover_pairs(root: Path) -> set[tuple[str, str]]:
    """Discover from both RST trees so missing pages cannot escape checking."""
    pairs = {('README.zh-CN.md', 'README.md')}
    for prefix in ('docs', 'docs/en'):
        paths = {Path('index.rst')}
        for section in ('tutorials', 'reference'):
            paths.update(p.relative_to(root / prefix)
                         for p in (root / prefix / section).rglob('*.rst'))
        pairs.update((f'docs/{p}', f'docs/en/{p}') for p in paths)
    for p in (root / 'docs/examples/en').rglob('*'):
        if p.is_file():
            relative = p.relative_to(root / 'docs/examples/en')
            pairs.add((f'docs/examples/{relative}', f'docs/examples/en/{relative}'))
    return pairs


def check_manifest(root: Path, *, update: bool = False) -> list[str]:
    manifest = root / MANIFEST
    previous = json.loads(manifest.read_text())['pairs'] if manifest.exists() else []
    recorded = {(p['zh'], p['en']): p for p in previous}
    pairs = discover_pairs(root)
    issues = []
    current = []
    for zh, en in sorted(pairs):
        missing = [p for p in (zh, en) if not (root / p).is_file()]
        if missing:
            issues.extend(f'missing counterpart: {p}' for p in missing)
            continue
        entry = {'zh': zh, 'en': en}
        for language, path in [('zh', zh), ('en', en)]:
            entry[f'{language}_sha256'] = hashlib.sha256((root / path).read_bytes()).hexdigest()
        old = recorded.get((zh, en))
        if not update:
            if old is None:
                issues.append(f'untracked pair: {zh} <-> {en}')
            elif ((entry['zh_sha256'] != old['zh_sha256']) !=
                  (entry['en_sha256'] != old['en_sha256'])):
                issues.append(f'stale: {zh} <-> {en}')
        current.append(entry)
    if update and not issues:
        manifest.parent.mkdir(parents=True, exist_ok=True)
        manifest.write_text(json.dumps({'pairs': current}, indent=2) + '\n')
    return issues


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('root', nargs='?', type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument('--strict', action='store_true', help='fail on any pairing or stale issue')
    parser.add_argument('--update', action='store_true', help='record both hashes after translation review')
    args = parser.parse_args(argv)
    try:
        issues = check_manifest(args.root, update=args.update)
    except (OSError, ValueError, KeyError, TypeError) as error:
        parser.exit(2, f'manifest error: {error}\n')
    for issue in issues:
        print(f'warning: {issue}')
    print(f'Documentation pairs: {len(issues)} issue(s)')
    return int(args.strict and bool(issues))


if __name__ == '__main__':
    raise SystemExit(main())
