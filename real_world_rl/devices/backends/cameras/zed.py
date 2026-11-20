"""Stereolabs ZED camera backend over the ZED SDK (``pyzed.sl``, imported lazily).

Delivers the left eye as RGB and, with ``enable_depth``, the depth registered to the left eye in
meters (invalid pixels 0, as for RealSense). The SDK is not on the package index: it is installed
with the ZED SDK and its ``get_python_api.py``, and depth runs on an NVIDIA GPU."""
from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Any

import numpy as np

from real_world_rl.devices.backends.cameras.zed_common import ZED_USB_IDS, select_mode
from real_world_rl.devices.camera import Camera, Frame
from real_world_rl.devices.errors import DeviceFault, DeviceNotFound

DEFAULT_DEPTH_MODE = "NEURAL_LIGHT"
# Depth is reported in this unit; the env contract is meters.
_UNIT = "METER"


@dataclass
class _Handle:
    camera: Any
    runtime: Any
    image: Any
    depth: Any | None
    resolution: Any
    serial: str


@Camera.register("zed")
class ZedCamera(Camera):
    SDK = "pyzed.sl"
    SDK_INSTALL_HINT = (
        "install the ZED SDK (stereolabs.com/developers/release), then its Python API into this "
        "environment: python /usr/local/zed/get_python_api.py"
    )
    USB_KIND = "zed"
    USB_IDS = ZED_USB_IDS
    USB_KIND_TITLE = "ZED"
    USB_SERIAL_IS_DEVICE_SERIAL = False

    @classmethod
    def discover(cls) -> set[str]:
        try:
            sl = cls.require_sdk()
        except Exception:
            return set()
        try:
            return {str(device.serial_number) for device in sl.Camera.get_device_list()}
        except Exception:
            return set()

    @classmethod
    def describe(cls) -> dict[str, str]:
        try:
            sl = cls.require_sdk()
            return {
                str(device.serial_number): str(getattr(device, "camera_model", "")).rpartition(".")[2]
                for device in sl.Camera.get_device_list()
            }
        except Exception:
            return {}

    def _open_device(self) -> _Handle:
        sl = self.require_sdk()
        c = self.config
        try:
            mode = select_mode(c)
            depth_mode = self._depth_mode(sl)
        except ValueError as error:
            raise DeviceFault(str(error), hint="fix the camera entry in the station YAML", device=self.name) from error
        serial = self._resolve_serial(self.discover())
        init = sl.InitParameters()
        init.camera_resolution = getattr(sl.RESOLUTION, mode.resolution)
        init.camera_fps = c.fps
        init.depth_mode = depth_mode
        init.coordinate_units = getattr(sl.UNIT, _UNIT)
        init.set_from_serial_number(int(serial))
        camera = sl.Camera()
        status = camera.open(init)
        if status != sl.ERROR_CODE.SUCCESS:
            try:
                camera.close()
            except Exception:
                pass
            raise DeviceFault(
                f"ZED '{serial}' failed to open: {status}",
                hint="check the USB 3 cable/port, that no other process holds the camera, and that the "
                "ZED SDK matches the CUDA driver (ZED_Diagnostic)",
                device=self.name,
                recoverable=True,
            )
        return _Handle(
            camera=camera,
            runtime=sl.RuntimeParameters(),
            image=sl.Mat(),
            depth=sl.Mat() if c.enable_depth else None,
            resolution=sl.Resolution(c.width, c.height),
            serial=serial,
        )

    def _depth_mode(self, sl: Any) -> Any:
        if not self.config.enable_depth:
            return sl.DEPTH_MODE.NONE
        name = (self.config.depth_mode or DEFAULT_DEPTH_MODE).upper()
        mode = getattr(sl.DEPTH_MODE, name, None)
        if mode is None or name == "NONE":
            raise ValueError(
                f"camera '{self.name}': unknown ZED depth_mode {self.config.depth_mode!r}; "
                "use NEURAL_LIGHT, NEURAL or NEURAL_PLUS"
            )
        return mode

    def _resolve_serial(self, found: set[str]) -> str:
        serial = self.config.serial
        if serial:
            if serial not in found:
                raise DeviceNotFound(
                    f"ZED serial '{serial}' for camera '{self.name}' not attached",
                    hint=f"discovered: {sorted(found)}; run `rwrl discover`",
                    device=self.name,
                )
            return serial
        if len(found) == 1:
            return next(iter(found))
        if not found:
            raise DeviceNotFound(
                f"no ZED camera found for camera '{self.name}'",
                hint="plug in the camera and run `rwrl discover` (ZED_Explorer must list it)",
                device=self.name,
            )
        raise DeviceNotFound(
            f"camera '{self.name}' has no serial and {len(found)} ZED cameras are attached",
            hint=f"set serial to one of {sorted(found)}",
            device=self.name,
        )

    def _capture_frame(self, handle: _Handle) -> Frame:
        sl = self.require_sdk()
        status = handle.camera.grab(handle.runtime)
        if status != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError(f"ZED '{handle.serial}' grab failed: {status}")
        handle.camera.retrieve_image(handle.image, sl.VIEW.LEFT, sl.MEM.CPU, handle.resolution)
        # BGRA in SDK-owned memory reused by the next grab: reorder to RGB into a fresh array.
        color = np.ascontiguousarray(handle.image.get_data()[:, :, 2::-1])
        depth = None
        if handle.depth is not None:
            handle.camera.retrieve_measure(handle.depth, sl.MEASURE.DEPTH, sl.MEM.CPU, handle.resolution)
            # NaN (occluded) and +-inf (out of range) become 0, the "no reading" value.
            depth = np.nan_to_num(
                np.asarray(handle.depth.get_data(), dtype=np.float32), nan=0.0, posinf=0.0, neginf=0.0
            )
        return Frame(color=color, depth=depth, timestamp=time.monotonic())

    def _close_device(self, handle: _Handle) -> None:
        try:
            handle.camera.close()
        except Exception:
            pass
