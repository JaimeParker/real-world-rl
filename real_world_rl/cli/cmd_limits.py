"""Inspect and explicitly migrate or rebase frozen station limits."""
from __future__ import annotations

import argparse
import difflib
import json
from pathlib import Path

from .command import Result


class LimitsCommand:
    name = 'limits'
    help = 'show, migrate or rebase station limits'

    def register(self, parser):
        actions = parser.add_subparsers(dest='action', required=True)
        show = actions.add_parser('show')
        show.add_argument('station_path', nargs='?')
        show.add_argument('--station', default=argparse.SUPPRESS)
        for name in ('migrate', 'rebase'):
            sub = actions.add_parser(name)
            sub.add_argument('yaml')
            sub.add_argument('--write', action='store_true')

    def run(self, args, ctx):
        import yaml
        from .limits_ops import arm_nodes, migrate_node, rebase_node, resolved_report
        from real_world_rl.station import load_station, STATION_SECTIONS
        from real_world_rl.discovery.autoconfig import resolve_station_config_path, parse_station
        from real_world_rl.common.yaml_edit import render_changes, atomic_replace
        if args.action == 'show':
            path = resolve_station_config_path(args.station_path or args.station)
            station = load_station(path)
            selected = station.robots if not args.robot else {args.robot: station.select(args.robot)}
            data = {'station': str(path), 'arms': {name: resolved_report(entry.config) for name, entry in selected.items()}}
            return Result(data, json.dumps(data, indent=2, allow_nan=False))
        path = Path(args.yaml)
        raw = path.read_bytes()
        document = yaml.safe_load(raw)
        warnings = []
        blocks = {}
        rebased = False
        for name, node in arm_nodes(document):
            if args.action == 'migrate':
                warnings.extend(migrate_node(node))
            elif (node.get('limits') or {}).get('tcp') is not None:
                rebase_node(node)
                rebased = True
            else:
                warnings.append(f'{name}: no TCP box; limits unchanged.')
            blocks[name] = {'limits': node.get('limits')}
        if args.action == 'rebase' and not rebased:
            raise ValueError('no limits.tcp configured')
        # Validate the entire candidate before rendering or writing anything.
        parse_station(document, sections=STATION_SECTIONS)
        text = render_changes(document, raw)
        diff = ''.join(difflib.unified_diff(raw.decode().splitlines(True), text.splitlines(True),
                                          fromfile=str(path), tofile=str(path) + ' (proposed)'))
        saved = atomic_replace(path, text, raw) if args.write else None
        data = {'arms': blocks, 'warnings': warnings, 'diff': diff, 'saved': saved}
        output = '\n'.join(warnings) + '\n' + yaml.safe_dump(blocks, sort_keys=False) + diff
        return Result(data, output)


LIMITS = LimitsCommand()
