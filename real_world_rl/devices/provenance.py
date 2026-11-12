"""Read-only fingerprints for loaded SDK artifacts; no imports or device access."""
from __future__ import annotations

import hashlib
from pathlib import Path
import sys

# The repository root: real_world_rl/devices/provenance.py -> real_world_rl/devices -> real_world_rl -> root.
_SOURCE_ROOT = Path(__file__).resolve().parents[2]


def portable_path(filename):
    if filename is None:
        return None
    path = Path(filename).expanduser().resolve()
    root = _SOURCE_ROOT
    if path.is_relative_to(root):
        return str(path.relative_to(root))
    if path.is_relative_to(Path.home()):
        return "~/" + str(path.relative_to(Path.home()))
    if len(path.parts) > 2 and path.parts[1] in ("home", "Users"):
        return "~/" + str(Path(*path.parts[3:]))
    import os
    return os.path.relpath(path, root)


def resolve_portable_path(portable) -> Path:
    """This machine's path for a ``portable_path`` result: ``~/...`` under the home directory,
    a relative path under the source root, an absolute path unchanged."""
    text = str(portable)
    if text.startswith("~/"):
        return Path.home() / text[2:]
    path = Path(text)
    return path if path.is_absolute() else _SOURCE_ROOT / path


def sdk_artifacts(sdk, *, implementation_type=None):
    """Hash facade and implementation modules, including native superclass modules.

    Missing module files are explicit evidence gaps. A facade hash alone does not
    establish the identity of a native extension or its dynamically linked libraries.
    """
    modules = [('facade', sdk)]
    if implementation_type is not None:
        name = getattr(implementation_type, '__module__', None)
        modules.append(('implementation', sys.modules.get(name)))
        seen = {name}
        for base in getattr(implementation_type, '__mro__', ())[1:]:
            name = getattr(base, '__module__', None)
            if base is object or name in seen:
                continue
            seen.add(name)
            modules.append(('implementation_base', sys.modules.get(name)))
    artifacts = []
    for role, module in modules:
        filename = getattr(module, '__file__', None)
        entry = {'role': role, 'module': getattr(module, '__name__', None),
                 'path': portable_path(filename), 'sha256': None}
        if filename is None:
            entry['unavailable_reason'] = 'loaded module file unavailable'
        else:
            try:
                path = Path(filename)
                digest = hashlib.sha256()
                with path.open('rb') as stream:
                    for chunk in iter(lambda: stream.read(1024 * 1024), b''):
                        digest.update(chunk)
                entry.update(path=portable_path(path), sha256=digest.hexdigest())
            except OSError as error:
                entry['unavailable_reason'] = type(error).__name__
        artifacts.append(entry)
    return {'algorithm': 'sha256', 'artifacts': artifacts,
            'scope': 'loaded module files on disk; does not attest loaded memory or linked system libraries'}
