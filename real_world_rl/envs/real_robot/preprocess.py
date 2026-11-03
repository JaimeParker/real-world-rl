"""Deterministic RGB/depth preprocessing with explicit source geometry."""
from dataclasses import dataclass
import math
import numpy as np


@dataclass
class CameraPreprocessConfig:
    roi: list[int] | None = None
    source_resolution: list[int] | None = None
    depth_clip_m: list[float] | None = None

    def __post_init__(self):
        if self.source_resolution is not None:
            if len(self.source_resolution) != 2 or any(type(x) is not int or x <= 0 for x in self.source_resolution):
                raise ValueError("source_resolution must be positive [height,width]")
        if self.roi is not None:
            if self.source_resolution is None or len(self.roi) != 4 or any(type(x) is not int for x in self.roi):
                raise ValueError("ROI requires [top,left,height,width] integers and source_resolution")
            top, left, height, width = self.roi
            if min(top, left) < 0 or min(height, width) <= 0 or top+height > self.source_resolution[0] or left+width > self.source_resolution[1]:
                raise ValueError("ROI lies outside source_resolution")
        if self.depth_clip_m is not None:
            if len(self.depth_clip_m) != 2 or not all(math.isfinite(x) for x in self.depth_clip_m) or not 0 <= self.depth_clip_m[0] < self.depth_clip_m[1]:
                raise ValueError("depth_clip_m must be finite [low,high] metres with 0<=low<high")


def preprocess_frame(frame, config, size, *, depth=False):
    frame = np.asarray(frame)
    if config.source_resolution is not None and list(frame.shape[:2]) != list(config.source_resolution):
        raise ValueError(f"camera source resolution {frame.shape[:2]} differs from {config.source_resolution}")
    if depth:
        if frame.ndim == 3 and frame.shape[-1] == 1: frame = frame[..., 0]
        if frame.ndim != 2: raise ValueError("depth frame must be HW or HW1")
        frame = frame.astype(np.float32, copy=True)
        invalid = ~np.isfinite(frame) | (frame <= 0)
        if config.depth_clip_m is not None:
            frame = np.clip(frame, *config.depth_clip_m)
        frame[invalid] = 0
    elif frame.ndim != 3 or frame.shape[-1] != 3:
        raise ValueError("RGB frame must be HWC with three channels")
    if config.roi is not None:
        top, left, height, width = config.roi
        frame = frame[top:top+height, left:left+width]
    if frame.shape[:2] != tuple(size):
        from PIL import Image
        frame = np.asarray(Image.fromarray(frame).resize((size[1], size[0]),
                           Image.Resampling.NEAREST if depth else Image.Resampling.BILINEAR))
    return np.require(frame[..., None] if depth else frame, dtype=np.float32 if depth else np.uint8, requirements=["C", "W"])
