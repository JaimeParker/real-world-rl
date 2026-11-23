"""The rwrl demo dataset (schema v1): one directory per recorded episode.

``DatasetWriter`` records episodes into a dataset directory, ``Dataset`` reads, drops, undrops
and prunes them, and ``iter_transitions``/``export_pkl`` give the training view (the old
recorder's pkl rows). The layout and fields are those of ``format``. Importing this package
does not import torch.
"""
from __future__ import annotations

from real_world_rl.dataset.format import (
    DISCARD_REASONS,
    FORMAT,
    SCHEMA_VERSION,
    DatasetMismatchError,
)
from real_world_rl.dataset.reader import Dataset, Episode, EpisodeInfo
from real_world_rl.dataset.transitions import export_pkl, iter_transitions
from real_world_rl.dataset.writer import DatasetWriter, EpisodeWriter

__all__ = [
    "DISCARD_REASONS",
    "FORMAT",
    "SCHEMA_VERSION",
    "Dataset",
    "DatasetMismatchError",
    "DatasetWriter",
    "Episode",
    "EpisodeInfo",
    "EpisodeWriter",
    "export_pkl",
    "iter_transitions",
]
