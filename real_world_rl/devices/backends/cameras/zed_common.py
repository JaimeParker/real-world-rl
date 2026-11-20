"""Facts shared by the two Stereolabs ZED backends (``zed`` via the ZED SDK, ``zed_uvc`` via
OpenCV): USB ids, native stereo modes and the mode a camera config selects. No SDK import."""
from __future__ import annotations

from dataclasses import dataclass

from real_world_rl.devices.camera import CameraConfig

ZED_USB_VENDOR = 0x2B03
# The video interfaces of the supported models. Each camera also enumerates a separate HID
# interface for its IMU/MCU (e.g. ZED 2 0xf781, ZED 2i 0xf881) that carries no video.
ZED_VIDEO_PRODUCTS: dict[int, str] = {
    0xF582: "ZED",
    0xF780: "ZED 2",
    0xF880: "ZED 2i",
}
ZED_USB_IDS: frozenset[tuple[int, int]] = frozenset((ZED_USB_VENDOR, pid) for pid in ZED_VIDEO_PRODUCTS)
# Aspect ratios within this relative tolerance count as equal (672x376 is 1.787, 16:9 is 1.778).
_ASPECT_TOLERANCE = 0.02


@dataclass(frozen=True)
class ZedMode:
    """One native mode: the size of each eye (UVC delivers both side by side) and its frame rates."""

    resolution: str  # the ``sl.RESOLUTION`` member name
    width: int
    height: int
    fps: tuple[int, ...]


# Smallest first: the selection takes the first mode that covers the requested size.
ZED_MODES: tuple[ZedMode, ...] = (
    ZedMode("VGA", 672, 376, (15, 30, 60, 100)),
    ZedMode("HD720", 1280, 720, (15, 30, 60)),
    ZedMode("HD1080", 1920, 1080, (15, 30)),
    ZedMode("HD2K", 2208, 1242, (15,)),
)


def _describe_modes() -> str:
    return "; ".join(
        f"{mode.resolution} {mode.width}x{mode.height} @ {'/'.join(map(str, mode.fps))} fps" for mode in ZED_MODES
    )


def select_mode(config: CameraConfig) -> ZedMode:
    """The smallest native mode at least as large as ``width`` x ``height``, with the same aspect
    ratio and ``fps``; frames are then downscaled to the requested size. Raises ValueError for a
    size or rate no mode delivers, instead of stretching the image."""
    width, height, fps = config.width, config.height, config.fps
    if width <= 0 or height <= 0:
        raise ValueError(f"camera '{config.name}': width and height must be positive, got {width}x{height}")
    aspect = width / height
    sized = [
        mode
        for mode in ZED_MODES
        if mode.width >= width
        and mode.height >= height
        and abs(mode.width / mode.height - aspect) <= _ASPECT_TOLERANCE * aspect
    ]
    if not sized:
        raise ValueError(
            f"camera '{config.name}': no ZED mode delivers {width}x{height} without stretching; "
            f"use a 16:9 size no larger than a native mode ({_describe_modes()})"
        )
    for mode in sized:
        if fps in mode.fps:
            return mode
    raise ValueError(
        f"camera '{config.name}': {fps} fps is not available at {width}x{height}; "
        f"native modes: {_describe_modes()}"
    )
