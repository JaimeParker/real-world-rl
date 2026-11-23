"""On-disk constants and helpers of the rwrl demo dataset (schema v1).

A dataset is a directory: ``dataset.json`` (the header) plus one ``episode_{index:06d}/``
directory per recorded episode (``meta.json``, ``data.npz``, ``frames/<key>.npz`` and
``video/<key>.mp4``). Episodes appear by an atomic rename of a complete ``.tmp-...`` directory;
JSON files are replaced atomically. numpy and the standard library only: no torch here.
"""
from __future__ import annotations

import dataclasses
import json
import math
import os
import re
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Mapping, Optional

import numpy as np

FORMAT = "rwrl-dataset"
SCHEMA_VERSION = 1

DATASET_FILE = "dataset.json"
META_FILE = "meta.json"
DATA_FILE = "data.npz"
FRAMES_DIR = "frames"
VIDEO_DIR = "video"
FRAMES_ARRAY = "frames"
TMP_PREFIX = ".tmp-"
EPISODE_PREFIX = "episode_"
_EPISODE_DIR = re.compile(r"^episode_(\d{6})$")

STATE_KEY = "state"
RGB_PREFIX = "rgb_"
NAMED_PREFIX = "named."
EVENT_PREFIX = "event."

STATUS_KEPT = "kept"
STATUS_DISCARDED = "discarded"
DISCARD_REASONS = (
    "rerecord", "failure event", "no success", "no intervened steps", "interrupted", "manual",
)
MANUAL_DISCARD = "manual"

# Header keys that must be equal (after JSON normalization) to append to a dataset.
COMPARED_HEADER_KEYS = ("action_contract", "obs_spec", "named_state_spec", "action_dim", "control_hz")


class DatasetMismatchError(ValueError):
    """Appending to a dataset whose header differs from the recording's. ``keys`` names the
    differing header keys."""

    def __init__(self, root: Path, keys: list[str]) -> None:
        self.keys = list(keys)
        super().__init__(
            f"dataset {root} was recorded with a different {', '.join(self.keys)}; record into a "
            "new dataset directory or match the recording settings"
        )


def episode_dir_name(index: int) -> str:
    return f"{EPISODE_PREFIX}{index:06d}"


def episode_index_of(name: str) -> Optional[int]:
    """The index of an ``episode_{index:06d}`` directory name, ``None`` for any other name."""
    match = _EPISODE_DIR.match(name)
    return int(match.group(1)) if match else None


def is_rgb_key(key: str) -> bool:
    return key.startswith(RGB_PREFIX)


def frame_keys(obs_spec: Mapping[str, Any]) -> list[str]:
    """The image keys of ``obs_spec``: every key but ``state``, in spec order."""
    return [key for key in obs_spec if key != STATE_KEY]


def utc_now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def json_normalize(value: Any) -> Any:
    """Canonical JSON form, as ``real_world_rl.contracts.action.json_normalize`` (sorted mapping
    keys, lists for tuples and arrays, non-finite floats as strings) without importing torch:
    a tensor is recognised by its ``detach`` method."""
    if dataclasses.is_dataclass(value) and not isinstance(value, type):
        value = dataclasses.asdict(value)
    if isinstance(value, Mapping):
        return {str(key): json_normalize(item) for key, item in sorted(value.items())}
    if isinstance(value, (tuple, list)):
        return [json_normalize(item) for item in value]
    if hasattr(value, "detach") and hasattr(value, "cpu"):
        value = value.detach().cpu().numpy()
    if isinstance(value, np.ndarray):
        return json_normalize(value.tolist())
    if isinstance(value, np.generic):
        value = value.item()
    if isinstance(value, float) and not math.isfinite(value):
        return str(value)
    return value


def fsync_dir(path: Path) -> None:
    descriptor = os.open(path, os.O_RDONLY)
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def fsync_file(path: Path) -> None:
    with open(path, "rb") as stream:
        os.fsync(stream.fileno())


def write_json_atomic(path: Path, value: Any) -> None:
    """Writes ``value`` as JSON to ``<path>.tmp``, fsyncs it and replaces ``path`` with it."""
    path = Path(path)
    tmp = path.with_name(path.name + ".tmp")
    with open(tmp, "w", encoding="utf-8") as stream:
        json.dump(value, stream, indent=2, sort_keys=True)
        stream.write("\n")
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(tmp, path)
    fsync_dir(path.parent)


def read_json(path: Path) -> Any:
    with open(path, encoding="utf-8") as stream:
        return json.load(stream)


def validate_header(header: Mapping[str, Any], root: Path) -> None:
    """Raises ``ValueError`` unless ``header`` is a schema-v1 rwrl dataset header."""
    if header.get("format") != FORMAT:
        raise ValueError(f"{root / DATASET_FILE} is not an {FORMAT} header (format={header.get('format')!r})")
    if header.get("schema_version") != SCHEMA_VERSION:
        raise ValueError(
            f"{root / DATASET_FILE} has schema_version {header.get('schema_version')!r}; this "
            f"version reads {SCHEMA_VERSION}"
        )


def differing_header_keys(saved: Mapping[str, Any], expected: Mapping[str, Any]) -> list[str]:
    """The ``COMPARED_HEADER_KEYS`` on which the two headers differ after JSON normalization."""
    return [
        key for key in COMPARED_HEADER_KEYS
        if json_normalize(saved.get(key)) != json_normalize(expected.get(key))
    ]
