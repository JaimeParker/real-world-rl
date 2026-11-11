"""Sparse session lifecycle evidence, separate from trajectory transitions."""
from __future__ import annotations

import json
import os
from pathlib import Path
import time


class SessionEvents:
    def __init__(self, path):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
        descriptor = os.open(self.path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
        os.close(descriptor)
        self.sequence = 0

    def emit(self, phase, *, reason, episode_index=None):
        if phase not in {'READY', 'ACTIVE', 'EPISODE_HOLD', 'STOPPING', 'CLOSED', 'FAULT'}:
            raise ValueError('unknown session phase')
        row = {'schema_version': 1, 'sequence': self.sequence, 'monotonic_s': time.monotonic(),
               'phase': phase, 'reason': reason, 'episode_index': episode_index}
        with self.path.open('a', encoding='utf-8') as stream:
            stream.write(json.dumps(row, allow_nan=False) + '\n')
        self.sequence += 1
