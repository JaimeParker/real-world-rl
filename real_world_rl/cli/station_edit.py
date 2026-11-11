"""Shared station documents and reviewed YAML writes for CLI commands."""

def station_document(args):
    from real_world_rl.station import load_station
    import yaml
    from robot_infra.discovery.autoconfig import resolve_station_config_path
    path = resolve_station_config_path(args.station).resolve()
    raw = path.read_bytes()
    document = yaml.safe_load(raw)
    station = load_station(path)
    selected = station.select(args.robot)
    if "robots" in document:
        key = next(key for key, value in station.robots.items() if value is selected)
        node = document["robots"][key]
    else:
        node = document.get("robot", document)
    return path, raw, document, node, selected


def confirm_write(ctx, path, document, expected):
    import difflib
    from real_world_rl.common.yaml_edit import render_changes
    candidate = render_changes(document, expected)
    previous = expected.decode('utf-8') if expected is not None else ''
    difference = ''.join(difflib.unified_diff(previous.splitlines(keepends=True),
        candidate.splitlines(keepends=True), fromfile=str(path), tofile=str(path) + ' (proposed)'))
    ctx.confirm("Save? [y/N] ", detail=difference or f"No content changes for {path}", saving=True)


def atomic_save(path, document, expected):
    from real_world_rl.common.yaml_edit import atomic_replace, render_changes
    return atomic_replace(path, render_changes(document, expected), expected)


