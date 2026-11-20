"""Stereolabs ZED camera backend without the ZED SDK: the camera's UVC stream through OpenCV
(``cv2``, imported lazily). Color only: the left half of the side-by-side stereo frame.

A camera is identified by the USB serial string of its video interface (``rwrl discover`` lists
it) or by its ``/dev/videoN`` path; the ZED SDK's camera serial is a different number."""
from __future__ import annotations

import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, ClassVar

from real_world_rl.devices.backends.cameras.zed_common import ZED_USB_IDS, ZED_VIDEO_PRODUCTS, ZedMode, select_mode
from real_world_rl.devices.camera import Camera, Frame
from real_world_rl.devices.errors import DeviceFault, DeviceNotFound

_WARMUP_FRAMES = 3


def _read(path: Path) -> str:
    try:
        return path.read_text(encoding="utf-8").strip()
    except (OSError, UnicodeError):
        return ""


@dataclass
class _Handle:
    capture: Any
    mode: ZedMode
    node: str


@Camera.register("zed_uvc")
class ZedUvcCamera(Camera):
    SDK = "cv2"
    PIP_EXTRA = "zed-uvc"
    PIP_REQUIREMENTS = ("opencv-python-headless",)
    USB_KIND = "zed"
    USB_IDS = ZED_USB_IDS
    USB_KIND_TITLE = "ZED"
    SYSFS: ClassVar[Path] = Path("/sys")
    DEV: ClassVar[Path] = Path("/dev")

    @classmethod
    def video_nodes(cls) -> dict[str, str]:
        """USB serial (or the node path when the camera reports none) -> ``/dev/videoN`` of each
        attached ZED's capture node; metadata nodes (``index`` other than 0) are skipped."""
        return {serial: node for serial, (node, _) in cls._scan().items()}

    @classmethod
    def describe(cls) -> dict[str, str]:
        return {serial: ZED_VIDEO_PRODUCTS[product] for serial, (_, product) in cls._scan().items()}

    @classmethod
    def _scan(cls) -> dict[str, tuple[str, int]]:
        nodes: dict[str, tuple[str, int]] = {}
        for entry in sorted((cls.SYSFS / "class/video4linux").glob("video*")):
            if _read(entry / "index") not in ("", "0"):
                continue
            usb = next(
                (parent for parent in (entry / "device").resolve().parents if (parent / "idVendor").exists()),
                None,
            )
            if usb is None:
                continue
            try:
                ids = int(_read(usb / "idVendor"), 16), int(_read(usb / "idProduct"), 16)
            except ValueError:
                continue
            if ids not in ZED_USB_IDS:
                continue
            node = str(cls.DEV / entry.name)
            nodes.setdefault(_read(usb / "serial") or node, (node, ids[1]))
        return nodes

    @classmethod
    def discover(cls) -> set[str]:
        return set(cls.video_nodes())

    def _open_device(self) -> _Handle:
        cv2 = self.require_sdk()
        c = self.config
        if c.enable_depth:
            raise DeviceFault(
                f"camera '{self.name}': the zed_uvc backend has no depth",
                hint="set enable_depth: false, or use backend: zed (ZED SDK) for depth",
                device=self.name,
            )
        try:
            mode = select_mode(c)
        except ValueError as error:
            raise DeviceFault(str(error), hint="fix the camera entry in the station YAML", device=self.name) from error
        node = self._resolve_node(self.video_nodes())
        capture = cv2.VideoCapture(node, cv2.CAP_V4L2)
        try:
            if not capture.isOpened():
                raise DeviceFault(
                    f"ZED '{node}' could not be opened",
                    hint="check that no other process holds the camera and that you may read the video node "
                    "(group video); run `rwrl doctor`",
                    device=self.name,
                    recoverable=True,
                )
            capture.set(cv2.CAP_PROP_FRAME_WIDTH, 2 * mode.width)
            capture.set(cv2.CAP_PROP_FRAME_HEIGHT, mode.height)
            capture.set(cv2.CAP_PROP_FPS, c.fps)
            capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            actual = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH)), int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
            if actual != (2 * mode.width, mode.height):
                raise DeviceFault(
                    f"ZED '{node}' delivers {actual[0]}x{actual[1]}, not the side-by-side "
                    f"{2 * mode.width}x{mode.height} of {mode.resolution}",
                    hint="the camera must be on a USB 3 port for the larger modes",
                    device=self.name,
                    recoverable=True,
                )
            for _ in range(_WARMUP_FRAMES):
                capture.read()
        except BaseException:
            capture.release()
            raise
        return _Handle(capture, mode, node)

    def _resolve_node(self, nodes: dict[str, str]) -> str:
        serial = self.config.serial
        if serial:
            if serial in nodes:
                return nodes[serial]
            if serial in nodes.values():
                return serial
            raise DeviceNotFound(
                f"ZED '{serial}' for camera '{self.name}' not attached",
                hint=f"discovered: {sorted(nodes)}; run `rwrl discover`",
                device=self.name,
            )
        if len(nodes) == 1:
            return next(iter(nodes.values()))
        if not nodes:
            raise DeviceNotFound(
                f"no ZED camera found for camera '{self.name}'",
                hint="plug in the camera and run `rwrl discover`",
                device=self.name,
            )
        raise DeviceNotFound(
            f"camera '{self.name}' has no serial and {len(nodes)} ZED cameras are attached",
            hint=f"set serial to one of {sorted(nodes)}",
            device=self.name,
        )

    def _capture_frame(self, handle: _Handle) -> Frame:
        cv2 = self.require_sdk()
        ok, frame = handle.capture.read()
        if not ok or frame is None:
            raise RuntimeError(f"ZED '{handle.node}' returned no frame")
        c, mode = self.config, handle.mode
        left = frame[:, : mode.width]
        if (mode.width, mode.height) != (c.width, c.height):
            left = cv2.resize(left, (c.width, c.height), interpolation=cv2.INTER_AREA)
        # cvtColor allocates: the frame never aliases OpenCV's capture buffer.
        color = cv2.cvtColor(left, cv2.COLOR_BGR2RGB)
        return Frame(color=color, depth=None, timestamp=time.monotonic())

    def _close_device(self, handle: _Handle) -> None:
        try:
            handle.capture.release()
        except Exception:
            pass
