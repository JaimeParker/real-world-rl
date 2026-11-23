"""Reading and maintaining a dataset: ``Dataset(root)``.

``episodes()`` lists the visible episodes (meta only); ``episode(index)`` loads one lazily
(``meta``, ``data`` and per-key ``frames``). The maintenance operations edit ``meta.json``
(``drop``/``undrop``) or delete discarded episode directories (``prune``); indices are never
renumbered or reused. Reading never touches ``.tmp-`` directories: a recording may be writing
one right now (``DatasetWriter.open`` cleans up stale ones). numpy only.
"""
from __future__ import annotations

import os
import secrets
import shutil
from dataclasses import dataclass
from functools import cached_property
from pathlib import Path
from typing import Any, Optional

import numpy as np

from real_world_rl.dataset import format as fmt

__all__ = ["Dataset", "Episode", "EpisodeInfo", "scan_episodes"]


@dataclass(frozen=True)
class EpisodeInfo:
    """One visible episode: its index, directory and ``meta.json`` contents."""

    index: int
    path: Path
    meta: dict

    @property
    def status(self) -> str:
        return self.meta["status"]

    @property
    def kept(self) -> bool:
        return self.meta["status"] == fmt.STATUS_KEPT

    @property
    def steps(self) -> int:
        return int(self.meta["steps"])


def scan_episodes(root: Path) -> list[EpisodeInfo]:
    """Every ``episode_{index:06d}`` directory under ``root`` with its meta, by index."""
    infos = []
    for path in Path(root).iterdir():
        index = fmt.episode_index_of(path.name)
        if index is not None and path.is_dir():
            infos.append(EpisodeInfo(index, path, fmt.read_json(path / fmt.META_FILE)))
    return sorted(infos, key=lambda info: info.index)


class Episode:
    """One episode on disk; ``meta``, ``data`` and each key's frames load on first use."""

    def __init__(self, index: int, path: Path) -> None:
        self.index = index
        self.path = path
        self._frames: dict[str, np.ndarray] = {}

    @cached_property
    def meta(self) -> dict:
        return fmt.read_json(self.path / fmt.META_FILE)

    @cached_property
    def data(self) -> dict[str, np.ndarray]:
        """The ``data.npz`` arrays (observation rows T+1, step rows T)."""
        with np.load(self.path / fmt.DATA_FILE) as archive:
            return {key: archive[key] for key in archive.files}

    def frames(self, key: str) -> np.ndarray:
        """The lossless ``(T+1, H, W, C)`` frames of image key ``key``."""
        if key not in self._frames:
            path = self.path / fmt.FRAMES_DIR / f"{key}.npz"
            if not path.is_file():
                raise KeyError(f"episode {self.index} has no frames for {key!r}")
            with np.load(path) as archive:
                self._frames[key] = archive[fmt.FRAMES_ARRAY]
        return self._frames[key]

    def video_path(self, key: str) -> Optional[Path]:
        """The review mp4 of RGB key ``key``, ``None`` when none was written."""
        relative = self.meta.get("videos", {}).get(key)
        return None if relative is None else self.path / relative


class Dataset:
    """A dataset directory (its ``dataset.json`` header must be schema v1)."""

    def __init__(self, root: str | Path) -> None:
        self.root = Path(root)
        header_path = self.root / fmt.DATASET_FILE
        if not header_path.is_file():
            raise FileNotFoundError(f"{self.root} is not a dataset: no {fmt.DATASET_FILE}")
        self.header: dict = fmt.read_json(header_path)
        fmt.validate_header(self.header, self.root)

    def episodes(self) -> list[EpisodeInfo]:
        return scan_episodes(self.root)

    def episode(self, index: int) -> Episode:
        path = self._episode_path(index)
        return Episode(index, path)

    # -- maintenance --------------------------------------------------------------------

    def drop(self, index: int, note: Optional[str] = None) -> None:
        """Marks kept episode ``index`` discarded (reason ``manual``); ``undrop`` reverses it."""
        meta = self.episode(index).meta
        if meta["status"] != fmt.STATUS_KEPT:
            raise ValueError(f"episode {index} is already discarded ({meta['discard_reason']})")
        meta.update(status=fmt.STATUS_DISCARDED, discard_reason=fmt.MANUAL_DISCARD)
        self._write_meta(index, meta, "drop", note)

    def undrop(self, index: int, note: Optional[str] = None) -> None:
        """Restores a dropped episode to kept. An episode discarded at record time cannot be."""
        meta = self.episode(index).meta
        if meta.get("recorded_discard_reason") is not None:
            raise ValueError(f"episode {index} was discarded when recorded "
                             f"({meta['recorded_discard_reason']}); it cannot be undropped")
        if meta["status"] != fmt.STATUS_DISCARDED:
            raise ValueError(f"episode {index} is not dropped")
        meta.update(status=fmt.STATUS_KEPT, discard_reason=None)
        self._write_meta(index, meta, "undrop", note)

    def prune(self) -> list[int]:
        """Deletes every discarded episode's directory; returns their indices. Each directory is
        first renamed to a ``.tmp-`` name, so a crash mid-delete never leaves a broken episode."""
        removed = []
        for info in self.episodes():
            if info.status != fmt.STATUS_DISCARDED:
                continue
            doomed = self.root / f"{fmt.TMP_PREFIX}pruned-{info.path.name}-{secrets.token_hex(4)}"
            os.rename(info.path, doomed)
            fmt.fsync_dir(self.root)
            shutil.rmtree(doomed)
            removed.append(info.index)
        return removed

    # -- internals ----------------------------------------------------------------------

    def _episode_path(self, index: int) -> Path:
        path = self.root / fmt.episode_dir_name(index)
        if not path.is_dir():
            raise KeyError(f"dataset {self.root} has no episode {index}")
        return path

    def _write_meta(self, index: int, meta: dict[str, Any], action: str, note: Optional[str]) -> None:
        meta.setdefault("history", []).append({"at": fmt.utc_now_iso(), "action": action, "note": note})
        fmt.write_json_atomic(self._episode_path(index) / fmt.META_FILE, meta)
