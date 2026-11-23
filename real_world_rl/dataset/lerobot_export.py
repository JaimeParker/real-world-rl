"""Export the kept episodes of a dataset as a LeRobot v3.0 dataset (``lerobot==0.7.0``).

One LeRobot frame per step ``t``: observation row ``t`` plus the executed action of step ``t``
(the final observation row ``T`` has no action and is dropped). Features:

- ``observation.state``: the policy state vector (float32);
- ``observation.<field>`` for each ``named.<field>`` of the header's ``named_state_spec`` (float32);
- ``observation.images.<camera>`` (video) for each ``rgb_<camera>`` key; depth keys are skipped
  with a warning (LeRobot's depth video needs its own encoder settings and is not exported);
- ``action``: the executed action (float32);
- ``intervened`` (bool, shape 1) and ``human_action_mask`` (bool, action dim): extra features
  LeRobot accepts as plain columns.

The task is the header's ``task_description``; fps is the header's ``control_hz``. RGB frame
height and width must be even (yuv420p video). ``lerobot`` is imported inside the function, so
importing this module needs neither lerobot nor torch.
"""
from __future__ import annotations

import logging
from pathlib import Path
from typing import Any, Optional

import numpy as np

from real_world_rl.dataset import format as fmt
from real_world_rl.dataset.reader import Dataset

__all__ = ["export_lerobot", "lerobot_features"]

logger = logging.getLogger(__name__)
IMAGE_PREFIX = "observation.images."
DEFAULT_VCODEC = "h264"


def _fps(header: dict, override: Optional[int]) -> int:
    if override is not None:
        return int(override)
    hz = float(header["control_hz"])
    if abs(hz - round(hz)) > 1e-6 or round(hz) < 1:
        raise ValueError(f"control_hz {hz} is not a whole number of frames per second; pass an explicit fps")
    return int(round(hz))


def lerobot_features(header: dict) -> tuple[dict, list[str], list[str]]:
    """``(features, rgb_keys, skipped_depth_keys)`` for ``header``."""
    obs_spec = header["obs_spec"]
    features: dict[str, dict] = {}
    if fmt.STATE_KEY in obs_spec:
        features["observation.state"] = {"dtype": "float32", "shape": tuple(obs_spec[fmt.STATE_KEY]["shape"]),
                                         "names": None}
    for field, shape in (header.get("named_state_spec") or {}).items():
        size = int(np.prod(shape))
        features[f"observation.{field}"] = {"dtype": "float32", "shape": (size,), "names": None}
    rgb_keys, skipped = [], []
    for key in fmt.frame_keys(obs_spec):
        if not fmt.is_rgb_key(key):
            skipped.append(key)
            continue
        height, width, channels = (int(n) for n in obs_spec[key]["shape"])
        if height % 2 or width % 2:
            raise ValueError(f"{key} frames are {width}x{height}; LeRobot video needs even width and height")
        rgb_keys.append(key)
        features[IMAGE_PREFIX + key[len(fmt.RGB_PREFIX):]] = {
            "dtype": "video", "shape": (height, width, channels), "names": ["height", "width", "channels"]}
    action_dim = int(header["action_dim"])
    features["action"] = {"dtype": "float32", "shape": (action_dim,), "names": None}
    features["intervened"] = {"dtype": "bool", "shape": (1,), "names": None}
    features["human_action_mask"] = {"dtype": "bool", "shape": (action_dim,), "names": None}
    return features, rgb_keys, skipped


def export_lerobot(dataset: Dataset, out: str | Path, *, repo_id: Optional[str] = None,
                   fps: Optional[int] = None, vcodec: str = DEFAULT_VCODEC) -> dict[str, Any]:
    """Writes the kept episodes of ``dataset`` to the new directory ``out``; returns a summary.

    ``out`` must not exist. ``repo_id`` defaults to ``rwrl/<out directory name>``; ``fps`` to the
    header's ``control_hz``. ``vcodec`` is the video codec (``h264`` by default: LeRobot's own
    default, AV1 via SVT, crashes on frames smaller than about 64 pixels).
    """
    from lerobot.configs.video import RGBEncoderConfig
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    out = Path(out)
    header = dataset.header
    kept = [info for info in dataset.episodes() if info.kept and info.steps > 0]
    if not kept:
        raise ValueError("the dataset has no kept episode with steps to export")
    features, rgb_keys, skipped = lerobot_features(header)
    for key in skipped:
        logger.warning("skipping %s: depth keys are not exported to LeRobot", key)
    fps = _fps(header, fps)
    named_fields = list(header.get("named_state_spec") or {})
    task = header.get("task_description") or ""
    target = LeRobotDataset.create(
        repo_id or f"rwrl/{out.name}", fps, features, root=out, robot_type=None, use_videos=True,
        rgb_encoder=RGBEncoderConfig(vcodec=vcodec))
    frames = 0
    try:
        for info in kept:
            episode = dataset.episode(info.index)
            data = episode.data
            images = {key: episode.frames(key) for key in rgb_keys}
            for step in range(info.steps):
                frame: dict[str, Any] = {"task": task}
                if "observation.state" in features:
                    frame["observation.state"] = np.asarray(data[fmt.STATE_KEY][step], dtype=np.float32)
                for field in named_fields:
                    frame[f"observation.{field}"] = np.asarray(
                        data[fmt.NAMED_PREFIX + field][step], dtype=np.float32).reshape(-1)
                for key in rgb_keys:
                    frame[IMAGE_PREFIX + key[len(fmt.RGB_PREFIX):]] = np.ascontiguousarray(images[key][step])
                frame["action"] = np.asarray(data["action"][step], dtype=np.float32)
                frame["intervened"] = np.asarray([data["intervened"][step]], dtype=bool)
                frame["human_action_mask"] = np.asarray(data["human_action_mask"][step], dtype=bool)
                target.add_frame(frame)
            target.save_episode()
            frames += info.steps
    finally:
        target.finalize()
    return {"out": str(out), "repo_id": target.repo_id, "fps": fps, "episodes": len(kept), "frames": frames,
            "episode_indices": [info.index for info in kept], "skipped_depth_keys": skipped}
