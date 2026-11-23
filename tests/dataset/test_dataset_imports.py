"""``real_world_rl.dataset`` is numpy-only: importing it (and opening a dataset) loads no torch."""
from __future__ import annotations

import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def test_importing_the_dataset_package_does_not_import_torch(tmp_path):
    code = (
        "import sys\n"
        "import real_world_rl.dataset as dataset\n"
        f"writer = dataset.DatasetWriter.open({str(tmp_path / 'ds')!r}, control_hz=10.0, action_dim=2,\n"
        "    obs_spec={'state': {'shape': [3], 'dtype': 'float32'}})\n"
        "dataset.Dataset(writer.root).episodes()\n"
        "assert 'torch' not in sys.modules, 'torch was imported'\n"
    )
    result = subprocess.run([sys.executable, "-c", code], cwd=ROOT, capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
