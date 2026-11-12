"""Software/configuration evidence captured outside control loops."""
from __future__ import annotations

import hashlib
import importlib.metadata
import importlib.util
import json
from pathlib import Path
import platform

from real_world_rl.devices.provenance import portable_path


def digest_document(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'),
                                     allow_nan=False).encode()).hexdigest()


def source_fingerprint(root):
    """Include relative paths and contents, so uncommitted edits change the digest."""
    root = Path(root)
    files, unavailable = {}, {}
    for path in sorted(root.rglob('*.py')):
        name = str(path.relative_to(root))
        try:
            files[name] = hashlib.sha256(path.read_bytes()).hexdigest()
        except OSError as error:
            unavailable[name] = type(error).__name__
    return {'root': portable_path(root), 'files': files, 'sha256': digest_document(files),
            'unavailable': unavailable, 'scope': 'Python source files; excludes native libraries and data'}


def runtime_provenance(*, config=None, files=None):
    packages = {}
    for name in ('real_world_rl', 'rl_garden'):
        spec = importlib.util.find_spec(name)
        roots = list(spec.submodule_search_locations or ()) if spec else []
        packages[name] = ([source_fingerprint(root) for root in roots] if roots else
                          {'unavailable_reason': 'package source location unavailable'})
    versions = sorted((dist.metadata.get('Name', 'unknown'), dist.version)
                      for dist in importlib.metadata.distributions())
    inputs = {}
    for name, filename in (files or {}).items():
        path = Path(filename)
        try:
            inputs[name] = {'path': portable_path(path), 'sha256': hashlib.sha256(path.read_bytes()).hexdigest()}
        except OSError as error:
            inputs[name] = {'path': portable_path(path), 'unavailable_reason': type(error).__name__}
    return {'schema_version': 1, 'python_version': platform.python_version(),
            'packages': packages, 'installed_versions': versions,
            'installed_versions_sha256': digest_document(versions),
            'config_sha256': digest_document(config) if config is not None else None,
            'input_files': inputs,
            'scope': 'pre-run on-disk evidence; not loaded-memory attestation or dependency binary hashes'}
