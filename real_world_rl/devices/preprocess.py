"""Deterministic RGB/depth preprocessing with explicit source geometry."""
from dataclasses import asdict, dataclass, replace
import math
from typing import Literal

import numpy as np


@dataclass
class CameraPreprocessConfig:
    roi: list[int] | None = None
    source_resolution: list[int] | None = None
    depth_clip_m: list[float] | None = None
    resize: Literal["pil_bilinear", "cv2_linear"] = "pil_bilinear"
    # Resolved to an integer ROI from camera metadata before requesting frames.
    crop_region: tuple[float, float, float, float] | Literal["center_square"] | None = None

    def __post_init__(self):
        if self.resize not in ("pil_bilinear", "cv2_linear"):
            raise ValueError("resize must be 'pil_bilinear' or 'cv2_linear'")
        if self.crop_region is not None:
            if self.roi is not None:
                raise ValueError("set either roi or crop_region")
            if self.crop_region != "center_square":
                region = self.crop_region
                if isinstance(region, str) or len(region) != 4:
                    raise ValueError("crop_region must be (top, left, bottom, right) or 'center_square'")
                top, left, bottom, right = region
                if not (0 <= top < bottom <= 1 and 0 <= left < right <= 1):
                    raise ValueError("crop_region must give increasing coordinates in [0, 1]")
                self.crop_region = tuple(region)
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

    def for_source(self, height: int, width: int) -> "CameraPreprocessConfig":
        """Resolve an RLinf crop using the declared native camera resolution."""
        if self.crop_region is None:
            return self
        resolution = [height, width]
        if self.source_resolution is not None and list(self.source_resolution) != resolution:
            raise ValueError(f"camera source resolution {tuple(resolution)} differs from {self.source_resolution}")
        if self.crop_region == "center_square":
            side = min(height, width)
            roi = [(height - side) // 2, (width - side) // 2, side, side]
        else:
            top, left, bottom, right = self.crop_region
            y1, x1 = int(height * top), int(width * left)
            y2, x2 = int(height * bottom), int(width * right)
            roi = [y1, x1, y2 - y1, x2 - x1]
        return replace(self, roi=roi, source_resolution=resolution, crop_region=None)

    def as_options(self) -> dict:
        """Keep default preprocessing compatible with existing wire and saved contracts."""
        options = asdict(self)
        if self.resize == "pil_bilinear":
            options.pop("resize")
        if self.crop_region is None:
            options.pop("crop_region")
        return options


def preprocess_frame(frame, config, size, *, depth=False):
    frame = np.asarray(frame)
    if config.crop_region is not None:
        raise ValueError("resolve crop_region against the camera source resolution before processing")
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
        if config.resize == "cv2_linear":
            import cv2

            interpolation = cv2.INTER_NEAREST if depth else cv2.INTER_LINEAR
            frame = cv2.resize(frame, (size[1], size[0]), interpolation=interpolation)
        else:
            from PIL import Image

            frame = np.asarray(Image.fromarray(frame).resize((size[1], size[0]),
                               Image.Resampling.NEAREST if depth else Image.Resampling.BILINEAR))
    return np.require(frame[..., None] if depth else frame, dtype=np.float32 if depth else np.uint8, requirements=["C", "W"])
