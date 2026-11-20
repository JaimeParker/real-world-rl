"""Hardware-free tests for the ZED camera backends: ``zed`` against an injected fake ``pyzed.sl``,
``zed_uvc`` against a fake sysfs tree and a fake ``cv2``, and the shared mode selection."""
from __future__ import annotations

import sys
import time
from types import ModuleType, SimpleNamespace

import numpy as np
import pytest

from real_world_rl.devices import Camera, CameraConfig, DeviceFault
from real_world_rl.devices.backends.cameras.zed import ZedCamera
from real_world_rl.devices.backends.cameras.zed_common import ZED_USB_IDS, select_mode
from real_world_rl.devices.backends.cameras.zed_uvc import ZedUvcCamera
from real_world_rl.devices.errors import DeviceNotFound, SdkMissing

WIDTH, HEIGHT, FPS = 640, 360, 30  # downscaled from VGA 672x376


def config(**kwargs) -> CameraConfig:
    values = dict(name="cam", width=WIDTH, height=HEIGHT, fps=FPS)
    values.update(kwargs)
    return CameraConfig(**values)


# -- mode selection ------------------------------------------------------------------------


@pytest.mark.parametrize(
    "width, height, fps, resolution",
    [
        (672, 376, 100, "VGA"),
        (640, 360, 30, "VGA"),
        (1280, 720, 60, "HD720"),
        (1280, 720, 15, "HD720"),
        (960, 540, 30, "HD720"),
        (1920, 1080, 30, "HD1080"),
        (2208, 1242, 15, "HD2K"),
        (1280, 720, 100, None),  # 100 fps only in VGA, which is smaller
        (640, 480, 30, None),  # 4:3: no ZED mode has that aspect ratio
        (2208, 1242, 30, None),
        (4000, 2250, 15, None),
    ],
)
def test_select_mode(width, height, fps, resolution):
    camera = config(width=width, height=height, fps=fps)
    if resolution is None:
        with pytest.raises(ValueError, match="native modes|without stretching"):
            select_mode(camera)
    else:
        assert select_mode(camera).resolution == resolution


def test_usb_ids_are_the_video_interfaces_of_the_supported_models():
    assert ZED_USB_IDS == {(0x2B03, 0xF582), (0x2B03, 0xF780), (0x2B03, 0xF880)}


def test_both_backends_are_registered_with_their_declarations():
    assert Camera.backend("zed") is ZedCamera and Camera.backend("zed_uvc") is ZedUvcCamera
    assert ZedCamera.USB_KIND == ZedUvcCamera.USB_KIND == "zed"
    assert ZedCamera.PIP_EXTRA is None and "get_python_api.py" in ZedCamera.SDK_INSTALL_HINT
    assert ZedUvcCamera.PIP_EXTRA == "zed-uvc" and ZedUvcCamera.PIP_REQUIREMENTS == ("opencv-python-headless",)


# -- zed (ZED SDK) ----------------------------------------------------------------------------


def make_fake_sl(serials, *, open_status="SUCCESS", grab_status="SUCCESS"):
    state = SimpleNamespace(init=None, opened=False, closed=False, grabs=0, retrieved=[])

    class InitParameters:
        def __init__(self):
            self.serial = None

        def set_from_serial_number(self, serial):
            self.serial = serial

    class Mat:
        data = None

        def get_data(self):
            return self.data

    class Camera:
        @staticmethod
        def get_device_list():
            return [SimpleNamespace(serial_number=s) for s in serials]

        def open(self, init):
            state.init = init
            state.opened = open_status == "SUCCESS"
            return open_status

        def grab(self, runtime):
            time.sleep(0.002)
            state.grabs += 1
            return grab_status

        def retrieve_image(self, mat, view, mem, resolution):
            state.retrieved.append(("image", view, mem, resolution.width, resolution.height))
            bgra = np.zeros((resolution.height, resolution.width, 4), np.uint8)
            bgra[..., 0], bgra[..., 1], bgra[..., 2], bgra[..., 3] = 10, 20, 30, 255  # B, G, R, A
            mat.data = bgra

        def retrieve_measure(self, mat, measure, mem, resolution):
            state.retrieved.append(("measure", measure, mem, resolution.width, resolution.height))
            depth = np.full((resolution.height, resolution.width), 1.25, np.float32)
            depth[0, 0], depth[0, 1], depth[0, 2] = np.nan, np.inf, -np.inf
            mat.data = depth

        def close(self):
            state.closed = True

    sl = ModuleType("pyzed.sl")
    sl.ERROR_CODE = SimpleNamespace(SUCCESS="SUCCESS")
    sl.RESOLUTION = SimpleNamespace(VGA="VGA", HD720="HD720", HD1080="HD1080", HD2K="HD2K")
    sl.DEPTH_MODE = SimpleNamespace(NONE="NONE", NEURAL_LIGHT="NEURAL_LIGHT", NEURAL="NEURAL", NEURAL_PLUS="NEURAL_PLUS")
    sl.UNIT = SimpleNamespace(METER="METER")
    sl.VIEW = SimpleNamespace(LEFT="LEFT")
    sl.MEM = SimpleNamespace(CPU="CPU")
    sl.MEASURE = SimpleNamespace(DEPTH="DEPTH")
    sl.InitParameters = InitParameters
    sl.RuntimeParameters = lambda: SimpleNamespace()
    sl.Resolution = lambda width, height: SimpleNamespace(width=width, height=height)
    sl.Mat = Mat
    sl.Camera = Camera
    return sl, state


@pytest.fixture
def install_sl(monkeypatch):
    def install(serials, **kwargs):
        sl, state = make_fake_sl(serials, **kwargs)
        package = ModuleType("pyzed")
        package.sl = sl
        monkeypatch.setitem(sys.modules, "pyzed", package)
        monkeypatch.setitem(sys.modules, "pyzed.sl", sl)
        return state

    return install


def test_zed_discover_returns_string_serials(install_sl):
    install_sl([12345678, 23456789])
    assert ZedCamera.discover() == {"12345678", "23456789"}


def test_zed_discover_without_sdk_is_empty(monkeypatch):
    monkeypatch.setitem(sys.modules, "pyzed", None)
    monkeypatch.setitem(sys.modules, "pyzed.sl", None)
    assert ZedCamera.discover() == set()


def test_zed_missing_sdk_hint_names_the_python_api_installer(monkeypatch):
    monkeypatch.setitem(sys.modules, "pyzed", None)
    monkeypatch.setitem(sys.modules, "pyzed.sl", None)
    with pytest.raises(SdkMissing) as info:
        ZedCamera(config()).connect()
    assert "get_python_api.py" in info.value.hint


def test_zed_open_color_only_selects_mode_and_no_depth(install_sl):
    state = install_sl([12345678])
    camera = ZedCamera(config())
    camera.connect()
    try:
        init = state.init
        assert (init.camera_resolution, init.camera_fps, init.depth_mode) == ("VGA", FPS, "NONE")
        assert init.coordinate_units == "METER" and init.serial == 12345678
        frame = camera.read()
        assert frame.depth is None
        assert frame.color.shape == (HEIGHT, WIDTH, 3) and frame.color.dtype == np.uint8
        assert frame.color.flags["C_CONTIGUOUS"]
        assert tuple(frame.color[0, 0]) == (30, 20, 10)  # BGRA -> RGB
        assert all(row[0] == "image" for row in state.retrieved)
        assert state.retrieved[0][1:] == ("LEFT", "CPU", WIDTH, HEIGHT)
    finally:
        camera.disconnect()
    assert state.closed


def test_zed_depth_is_meters_with_invalid_pixels_zero(install_sl):
    state = install_sl([12345678])
    camera = ZedCamera(config(enable_depth=True))
    camera.connect()
    try:
        assert state.init.depth_mode == "NEURAL_LIGHT"
        frame = camera.read()
        assert frame.depth.shape == (HEIGHT, WIDTH) and frame.depth.dtype == np.float32
        assert list(frame.depth[0, :4]) == [0.0, 0.0, 0.0, 1.25]
        assert ("measure", "DEPTH", "CPU", WIDTH, HEIGHT) in state.retrieved
        assert camera.get_observation()["depth"].shape == (HEIGHT, WIDTH)
    finally:
        camera.disconnect()


def test_zed_depth_mode_from_config(install_sl):
    state = install_sl([12345678])
    camera = ZedCamera(config(enable_depth=True, depth_mode="neural_plus"))
    camera.connect()
    try:
        assert state.init.depth_mode == "NEURAL_PLUS"
    finally:
        camera.disconnect()


@pytest.mark.parametrize("depth_mode", ["ULTRA_FAST", "none"])
def test_zed_unknown_depth_mode_is_a_config_fault(install_sl, depth_mode):
    install_sl([12345678])
    with pytest.raises(DeviceFault, match="depth_mode") as info:
        ZedCamera(config(enable_depth=True, depth_mode=depth_mode)).connect()
    assert "station YAML" in info.value.hint


def test_zed_unsupported_size_is_a_config_fault_before_opening(install_sl):
    state = install_sl([12345678])
    with pytest.raises(DeviceFault, match="without stretching"):
        ZedCamera(config(width=640, height=480)).connect()
    assert state.init is None


def test_zed_serial_selection(install_sl):
    state = install_sl([12345678, 23456789])
    camera = ZedCamera(config(serial="23456789"))
    camera.connect()
    try:
        assert state.init.serial == 23456789
    finally:
        camera.disconnect()
    with pytest.raises(DeviceNotFound) as info:
        ZedCamera(config()).connect()
    assert "12345678" in info.value.hint and "23456789" in info.value.hint
    with pytest.raises(DeviceNotFound, match="not attached"):
        ZedCamera(config(serial="99")).connect()


def test_zed_no_camera_points_to_discover(install_sl):
    install_sl([])
    with pytest.raises(DeviceNotFound) as info:
        ZedCamera(config()).connect()
    assert "rwrl discover" in info.value.hint


def test_zed_open_failure_is_recoverable(install_sl):
    install_sl([12345678], open_status="CAMERA_NOT_DETECTED")
    camera = ZedCamera(config())
    with pytest.raises(DeviceFault, match="CAMERA_NOT_DETECTED") as info:
        camera.connect()
    assert info.value.recoverable is True
    assert not camera.is_connected


def test_zed_grab_failures_surface_as_read_fault(install_sl):
    install_sl([12345678], grab_status="CAMERA_REBOOTING")
    camera = ZedCamera(config(), read_timeout_s=0.2)
    camera.connect()
    try:
        with pytest.raises(DeviceFault, match="no frame"):
            camera.read()
        assert "CAMERA_REBOOTING" in camera.health().detail
    finally:
        camera.disconnect()


# -- zed_uvc (OpenCV) ---------------------------------------------------------------------------


def make_sysfs(root, devices):
    """``devices``: (video name, vid, pid, USB serial, index) tuples."""
    for number, (video, vid, pid, serial, index) in enumerate(devices, start=1):
        usb = root / "devices" / f"usb{number}" / f"{number}-1"
        interface = usb / f"{number}-1:1.0"
        interface.mkdir(parents=True)
        (usb / "idVendor").write_text(f"{vid:04x}\n")
        (usb / "idProduct").write_text(f"{pid:04x}\n")
        if serial:
            (usb / "serial").write_text(serial + "\n")
        entry = root / "class/video4linux" / video
        entry.mkdir(parents=True)
        (entry / "index").write_text(f"{index}\n")
        (entry / "device").symlink_to(interface)


def make_fake_cv2(state):
    class VideoCapture:
        def __init__(self, node, api):
            state.opened_node, state.api = node, api
            self.props = {}

        def isOpened(self):
            return state.can_open

        def set(self, prop, value):
            self.props[prop] = value
            return True

        def get(self, prop):
            if prop in ("W", "H") and state.force_size is not None:
                return state.force_size[0 if prop == "W" else 1]
            return self.props.get(prop, 0)

        def read(self):
            time.sleep(0.002)
            state.reads += 1
            width, height = int(self.props["W"]), int(self.props["H"])
            bgr = np.zeros((height, width, 3), np.uint8)
            bgr[:, : width // 2] = (1, 2, 3)  # left eye, BGR
            bgr[:, width // 2 :] = (9, 9, 9)
            return True, bgr

        def release(self):
            state.released = True

    def resize(image, size, interpolation):
        state.resized = (image.shape[1], image.shape[0], *size)
        return np.ascontiguousarray(np.resize(image, (size[1], size[0], 3)))

    return SimpleNamespace(
        VideoCapture=VideoCapture,
        CAP_V4L2="V4L2",
        CAP_PROP_FRAME_WIDTH="W",
        CAP_PROP_FRAME_HEIGHT="H",
        CAP_PROP_FPS="FPS",
        CAP_PROP_BUFFERSIZE="BUF",
        INTER_AREA="AREA",
        COLOR_BGR2RGB="BGR2RGB",
        resize=resize,
        cvtColor=lambda image, code: np.ascontiguousarray(image[:, :, ::-1]),
    )


@pytest.fixture
def uvc(tmp_path, monkeypatch):
    monkeypatch.setattr(ZedUvcCamera, "SYSFS", tmp_path / "sys")
    monkeypatch.setattr(ZedUvcCamera, "DEV", tmp_path / "dev")
    state = SimpleNamespace(can_open=True, force_size=None, reads=0, released=False, resized=None,
                            opened_node=None, api=None)
    monkeypatch.setitem(sys.modules, "cv2", make_fake_cv2(state))

    def install(devices):
        make_sysfs(tmp_path / "sys", devices)
        return state

    state.dev = tmp_path / "dev"
    state.install = install
    return state


def test_uvc_discover_lists_zed_capture_nodes_only(uvc):
    uvc.install([
        ("video0", 0x2B03, 0xF582, "OV0001", 0),
        ("video1", 0x2B03, 0xF582, "OV0001", 1),  # metadata node of the same camera
        ("video2", 0x2B03, 0xF880, "", 0),  # no USB serial: the node identifies it
        ("video4", 0x8086, 0x0B07, "RS1", 0),  # a RealSense
        ("video6", 0x2B03, 0xF881, "HID", 0),  # not a video interface id
    ])
    assert ZedUvcCamera.video_nodes() == {"OV0001": str(uvc.dev / "video0"), str(uvc.dev / "video2"): str(uvc.dev / "video2")}
    assert ZedUvcCamera.discover() == {"OV0001", str(uvc.dev / "video2")}


def test_uvc_reads_the_left_eye_downscaled_to_rgb(uvc):
    uvc.install([("video0", 0x2B03, 0xF582, "OV0001", 0)])
    camera = ZedUvcCamera(config())
    camera.connect()
    try:
        assert uvc.opened_node == str(uvc.dev / "video0") and uvc.api == "V4L2"
        frame = camera.read()
        assert frame.depth is None
        assert frame.color.shape == (HEIGHT, WIDTH, 3)
        assert uvc.resized == (672, 376, WIDTH, HEIGHT)  # only the left half was resized
        assert tuple(frame.color[0, 0]) == (3, 2, 1)
    finally:
        camera.disconnect()
    assert uvc.released


def test_uvc_native_size_is_not_resized(uvc):
    uvc.install([("video0", 0x2B03, 0xF780, "Z2", 0)])
    camera = ZedUvcCamera(config(width=1280, height=720, fps=60))
    camera.connect()
    try:
        assert camera.read().color.shape == (720, 1280, 3)
        assert uvc.resized is None
    finally:
        camera.disconnect()


def test_uvc_selects_by_serial_or_node(uvc):
    uvc.install([("video0", 0x2B03, 0xF582, "A", 0), ("video2", 0x2B03, 0xF880, "B", 0)])
    for serial, node in (("B", "video2"), (str(uvc.dev / "video0"), "video0")):
        camera = ZedUvcCamera(config(serial=serial))
        camera.connect()
        try:
            assert uvc.opened_node == str(uvc.dev / node)
        finally:
            camera.disconnect()
    with pytest.raises(DeviceNotFound) as info:
        ZedUvcCamera(config()).connect()
    assert "'A'" in info.value.hint and "'B'" in info.value.hint


def test_uvc_rejects_depth(uvc):
    uvc.install([("video0", 0x2B03, 0xF582, "A", 0)])
    with pytest.raises(DeviceFault, match="no depth") as info:
        ZedUvcCamera(config(enable_depth=True)).connect()
    assert "backend: zed" in info.value.hint


def test_uvc_wrong_delivered_size_is_a_recoverable_fault_and_releases(uvc):
    uvc.install([("video0", 0x2B03, 0xF582, "A", 0)])
    uvc.force_size = (1280, 480)
    with pytest.raises(DeviceFault, match="side-by-side") as info:
        ZedUvcCamera(config()).connect()
    assert info.value.recoverable is True and uvc.released


def test_uvc_unopenable_node_is_a_recoverable_fault(uvc):
    uvc.install([("video0", 0x2B03, 0xF582, "A", 0)])
    uvc.can_open = False
    with pytest.raises(DeviceFault, match="could not be opened") as info:
        ZedUvcCamera(config()).connect()
    assert info.value.recoverable is True and uvc.released
