"""Round-trip YAML edits with verbatim untouched text and one atomic writer.

Ruamel retains node styles/comments, but its emitter may normalize unrelated
whitespace. Source spans therefore limit its output to changed values. Safe-load
validation remains separate from the round-trip representation used for editing.
"""
from __future__ import annotations

from io import StringIO
import os
from pathlib import Path
import tempfile
import uuid

import yaml
from ruamel.yaml import YAML

from real_world_rl.cli.errors import CliError


def _yaml():
    codec = YAML(typ="rt")
    codec.preserve_quotes = True
    codec.width = 1_000_000
    codec.indent(mapping=2, sequence=4, offset=2)
    return codec


def _load(raw):
    document = _yaml().load(raw.decode("utf-8"))
    if not isinstance(document, dict):
        raise CliError("editable YAML must be a mapping", code=2)
    document._edit_source = raw
    return document


def load_editable(path):
    """Return a round-trip mapping and its exact source bytes (None if new)."""
    path = Path(path)
    raw = path.read_bytes() if path.exists() else None
    document = _load(raw if raw is not None else b"{}\n")
    document._edit_source = raw
    return document, raw


def _merge(target, previous, desired):
    """Apply only semantic changes; retain round-trip styles on existing nodes."""
    for key in list(target):
        if key not in desired:
            del target[key]
    for key, value in desired.items():
        if key in previous and previous[key] == value:
            continue
        if isinstance(value, dict) and isinstance(target.get(key), dict):
            _merge(target[key], previous.get(key, {}), value)
        else:
            target[key] = value


def render_changes(document, expected):
    """Render a safe-loaded candidate against the bytes captured before review."""
    if expected is None:
        return render(document)
    editable = _load(expected)
    _merge(editable, yaml.safe_load(expected), document)
    result = render(editable)
    if yaml.safe_load(result) != document:
        raise CliError("cannot edit shared YAML aliases independently; file was not changed", code=2)
    return result


def _end(node):
    # Collection end marks include trailing comments/whitespace and sometimes the
    # next key's indentation. Those belong to the original text, not this edit.
    if isinstance(node, yaml.MappingNode) and not node.flow_style and node.value:
        return _end(node.value[-1][1])
    if isinstance(node, yaml.SequenceNode) and not node.flow_style and node.value:
        return _end(node.value[-1])
    return node.end_mark.index


def _line_end(source, index):
    if index and source[index - 1] == '\n':
        return index
    end = source.find('\n', index)
    return len(source) if end < 0 else end + 1


def _block(node):
    return isinstance(node, (yaml.MappingNode, yaml.SequenceNode)) and not node.flow_style


def _splice(source, candidate, before, after):
    old_tree = yaml.compose(source, Loader=yaml.SafeLoader)
    new_tree = yaml.compose(candidate, Loader=yaml.SafeLoader)
    edits = []

    def visit(old, new, old_value, new_value, key_column=0):
        if old_value == new_value:
            return
        if (isinstance(old, yaml.MappingNode) and isinstance(new, yaml.MappingNode)
                and ((not old.flow_style and not new.flow_style)
                     or set(old_value) == set(new_value))):
            old_pairs = {key.value: (key, value) for key, value in old.value}
            new_pairs = {key.value: (key, value) for key, value in new.value}
            # YAML configuration keys are strings; refuse ambiguous key spellings.
            if set(old_pairs) != set(old_value) or set(new_pairs) != set(new_value):
                raise CliError("editable YAML keys must be strings", code=2)
            for key, (key_node, value_node) in old_pairs.items():
                if key in new_pairs:
                    visit(value_node, new_pairs[key][1], old_value[key], new_value[key],
                          key_node.start_mark.column)
                else:
                    start = key_node.start_mark.index - key_node.start_mark.column
                    edits.append((start, _line_end(source, _end(value_node)), ''))
            additions = [key for key in new_pairs if key not in old_pairs]
            if additions:
                position = _line_end(source, _end(old))
                column = old.value[0][0].start_mark.column
                chunks = []
                for key in additions:
                    key_node, value_node = new_pairs[key]
                    chunk = candidate[key_node.start_mark.index:_end(value_node)]
                    lines = chunk.splitlines()
                    shift = column - key_node.start_mark.column
                    chunks.append(' ' * column + lines[0] + '\n' + ''.join(
                        (' ' * shift + line if shift >= 0 else line[-shift:]) + '\n'
                        for line in lines[1:]))
                newline = '\r\n' if '\r\n' in source else '\n'
                addition = ''.join(chunks).replace('\n', newline)
                if position and source[position - 1] != '\n':
                    addition = newline + addition
                edits.append((position, position, addition))
            return
        replacement = candidate[new.start_mark.index:_end(new)]
        if _block(new):
            column = old.start_mark.column if _block(old) else key_column + 2
            if old is old_tree:
                column = 0
            shift = column - new.start_mark.column
            lines = replacement.splitlines()
            replacement = lines[0] + ''.join('\n' + (
                ' ' * shift + line if shift >= 0 else line[-shift:]) for line in lines[1:])
            if not _block(old) and old is not old_tree:
                replacement = '\n' + ' ' * column + replacement
        if '\r\n' in source:
            replacement = replacement.replace('\n', '\r\n')
        edits.append((old.start_mark.index, _end(old), replacement))

    visit(old_tree, new_tree, before, after)
    for start, end, replacement in reversed(sorted(edits, key=lambda edit: edit[0])):
        source = source[:start] + replacement + source[end:]
    return source


def render(document):
    """Render edited nodes, preserving all bytes outside their source spans."""
    stream = StringIO()
    _yaml().dump(document, stream)
    candidate = stream.getvalue()
    raw = getattr(document, '_edit_source', None)
    if raw is None:
        return candidate
    source = raw.decode('utf-8')
    before, after = yaml.safe_load(source), yaml.safe_load(candidate)
    result = _splice(source, candidate, before, after)
    if yaml.safe_load(result) != after:
        raise CliError("cannot preserve this YAML layout safely; file was not changed", code=2)
    return result


def atomic_replace(path, rendered, expected):
    """Refuse stale edits, preserve exact previous bytes, atomically replace YAML."""
    path = Path(path)
    rendered = rendered.encode("utf-8")
    current = path.read_bytes() if path.exists() else None
    if current != expected:
        raise CliError("configuration changed during review; reload and retry")
    backup = None
    if current is not None:
        backup = path.with_name(path.name + f".{uuid.uuid4().hex}.bak")
        with backup.open("xb") as stream:
            os.chmod(backup, 0o600)
            stream.write(current)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(dir=path.parent, prefix=f".{path.name}.", delete=False) as stream:
            temporary = Path(stream.name)
            stream.write(rendered)
            stream.flush()
            os.fsync(stream.fileno())
        if expected is not None:
            os.chmod(temporary, path.stat().st_mode & 0o777)
        if (path.read_bytes() if path.exists() else None) != expected:
            raise CliError("configuration changed before replacement; retry")
        os.replace(temporary, path)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)
    return {"path": str(path), "backup": str(backup) if backup else None}
