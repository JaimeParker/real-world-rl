"""Private JSON evidence files, created without overwriting existing reports."""
import json
import os
from pathlib import Path


def write_json(path, data):
    descriptor = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
        json.dump(data, stream, indent=2, allow_nan=False)


def unique_output_path(path):
    """Put a unique timestamped evidence filename under the requested run directory."""
    import time
    import uuid
    target = Path(path)
    stamp = time.strftime("%Y%m%dT%H%M%S", time.gmtime()) + "-" + uuid.uuid4().hex[:10]
    return target.with_name(f"{target.stem}-{stamp}{target.suffix}")
