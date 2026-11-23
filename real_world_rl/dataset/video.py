"""Review videos of an episode's RGB frames: libx264/yuv420p mp4 through an ffmpeg pipe.

The lossless ``frames/<key>.npz`` stay the training source; a video is a convenience. A missing
ffmpeg or a failed encode therefore never fails the episode: ``encode_video`` logs a warning and
returns ``False``.
"""
from __future__ import annotations

import logging
import shutil
import subprocess
from pathlib import Path

import numpy as np

_LOGGER = logging.getLogger(__name__)
# Odd frame sizes cannot be encoded as yuv420p; pad them to even.
_PAD_EVEN = "pad=ceil(iw/2)*2:ceil(ih/2)*2"


def ffmpeg_executable() -> str:
    """ffmpeg on PATH, else imageio-ffmpeg's bundled binary, else ``"ffmpeg"`` (the lookup of
    ``real_world_rl.envs.wrappers.video_recorder``)."""
    ffmpeg = shutil.which("ffmpeg")
    if ffmpeg is not None:
        return ffmpeg
    try:
        import imageio_ffmpeg
    except ImportError:
        return "ffmpeg"
    return imageio_ffmpeg.get_ffmpeg_exe()


def encode_video(frames: np.ndarray, path: Path, *, fps: float) -> bool:
    """Encodes ``frames`` ``(N, H, W, 3)`` uint8 to the mp4 ``path``; ``True`` on success. On any
    failure the partial file is removed, a warning logged and ``False`` returned."""
    if frames.ndim != 4 or frames.shape[-1] != 3 or frames.dtype != np.uint8 or len(frames) == 0:
        _LOGGER.warning("no review video %s: frames %s %s are not (N, H, W, 3) uint8",
                        path, frames.shape, frames.dtype)
        return False
    height, width = frames.shape[1:3]
    command = [
        ffmpeg_executable(), "-y", "-loglevel", "error",
        "-f", "rawvideo", "-pixel_format", "rgb24", "-video_size", f"{width}x{height}",
        "-framerate", str(fps), "-i", "-",
        "-vf", _PAD_EVEN, "-pix_fmt", "yuv420p", "-vcodec", "libx264", "-crf", "23", str(path),
    ]
    try:
        completed = subprocess.run(
            command, input=np.ascontiguousarray(frames).tobytes(), stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE, check=False,
        )
    except OSError as error:
        _LOGGER.warning("no review video %s: cannot run ffmpeg (%s)", path, error)
        path.unlink(missing_ok=True)
        return False
    if completed.returncode != 0 or not path.is_file():
        _LOGGER.warning("no review video %s: ffmpeg exited %s: %s", path, completed.returncode,
                        completed.stderr.decode(errors="replace").strip())
        path.unlink(missing_ok=True)
        return False
    return True
