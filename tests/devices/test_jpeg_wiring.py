"""--jpeg-quality wiring: service encodes camera color on every camera path; clients restore it."""
from __future__ import annotations

import sys
import time

import numpy as np
import pytest

import real_world_rl.devices.claims as claims
from real_world_rl.devices import CameraConfig, Frame, GripperConfig
from real_world_rl.devices.backends.cameras.fake import FakeCamera
from real_world_rl.devices.base import Health
from real_world_rl.devices.errors import SdkMissing
from real_world_rl.devices.robots.franka import FRANKA_HOME_JOINTS
from real_world_rl.devices.robots.franka import FrankaConfig, FrankaRobot
from real_world_rl.server import RemoteRobot
from real_world_rl.server.codec import pack, unpack
from real_world_rl.server.remote import _rebuild
from real_world_rl.server.service import RobotService
from real_world_rl.server.transport import LocalClient

pytest.importorskip("PIL")

HEIGHT, WIDTH = 96, 128
ACTION = {"arm": {"joint_pos": np.zeros(7)}, "gripper": {"gripper": [0.5]}}


def textured_frame() -> np.ndarray:
    ys, xs = np.mgrid[0:HEIGHT, 0:WIDTH]
    gradient = np.stack([xs, ys * 2, xs + ys], axis=-1)
    noise = np.random.default_rng(0).integers(0, 8, size=gradient.shape)
    return (gradient + noise).astype(np.uint8)


FRAME = textured_frame()


@pytest.fixture(autouse=True)
def fixed_frames(tmp_path, monkeypatch):
    monkeypatch.setattr(claims, "_DEFAULT_LOCK_DIR", tmp_path / "claims")

    depth_frame = np.full((HEIGHT, WIDTH), 0.5, dtype=np.float32)

    def capture(self, handle):
        time.sleep(1.0 / self._config.fps)
        depth = depth_frame.copy() if self._config.enable_depth else None
        return Frame(color=FRAME.copy(), depth=depth, timestamp=123.0)

    monkeypatch.setattr(FakeCamera, "_capture_frame", capture)
    return depth_frame


def make_robot(enable_depth: bool = False) -> FrankaRobot:
    return FrankaRobot(
        FrankaConfig(
            arm_backend="fake",
            gripper=GripperConfig(backend="fake"),
            cameras=[
                CameraConfig(
                    name="wrist", backend="fake", fps=60, width=WIDTH, height=HEIGHT, enable_depth=enable_depth
                )
            ],
        )
    )


@pytest.fixture
def make_service():
    services = []

    def build(jpeg_quality=None, enable_depth=False) -> RobotService:
        service = RobotService(make_robot(enable_depth), watchdog_s=30.0, jpeg_quality=jpeg_quality)
        services.append(service)
        service.dispatch("robot.connect")
        return service

    yield build
    for service in services:
        service.close()


def wire_roundtrip(reply):
    wire = pack(reply)
    return wire, unpack(wire)


def test_raw_when_quality_none(make_service):
    service = make_service()
    _, reply = wire_roundtrip(service.dispatch("camera.wrist.read", {"timeout_s": 2.0}))
    assert np.array_equal(reply["color"], FRAME)


def test_jpeg_camera_read_is_lossy_and_small(make_service):
    raw_size = len(pack({"color": FRAME}))
    service = make_service(jpeg_quality=90)
    reply = service.dispatch("camera.wrist.read", {"timeout_s": 2.0})
    assert reply["color"]["jpeg"] is True
    wire, decoded = wire_roundtrip(reply)
    assert decoded["color"].dtype == np.uint8 and decoded["color"].shape == FRAME.shape
    diff = np.abs(decoded["color"].astype(int) - FRAME.astype(int))
    assert 0 < diff.max() <= 40
    assert len(wire) < raw_size / 3
    assert decoded["timestamp"] == 123.0


@pytest.mark.parametrize("method", ["robot.get_observation", "robot.step"])
def test_observation_paths_encode_color_only(make_service, method):
    service = make_service(jpeg_quality=90, enable_depth=True)
    kwargs = None
    if method == "robot.step":
        service.dispatch("arm.set_control_mode", {"mode": "joint_servo"})
        kwargs = {"action": ACTION}
    reply = service.dispatch(method, kwargs)
    assert reply["cameras"]["wrist"]["color"]["jpeg"] is True
    _, decoded = wire_roundtrip(reply)
    camera = decoded["cameras"]["wrist"]
    assert camera["color"].dtype == np.uint8 and camera["color"].shape == FRAME.shape
    assert not np.array_equal(camera["color"], FRAME)
    assert camera["depth"].dtype == np.float32
    assert np.array_equal(camera["depth"], np.full((HEIGHT, WIDTH), 0.5, dtype=np.float32))
    assert decoded["arm"].keys() == service.dispatch("robot.get_observation")["arm"].keys()


def test_camera_read_keeps_depth_and_timestamp(make_service):
    service = make_service(jpeg_quality=90, enable_depth=True)
    _, reply = wire_roundtrip(service.dispatch("camera.wrist.read", {"timeout_s": 2.0}))
    assert reply["depth"].dtype == np.float32 and np.all(reply["depth"] == 0.5)
    assert reply["timestamp"] == 123.0


@pytest.mark.parametrize("quality, expected", [(None, None), (90, {"jpeg": 90})])
def test_describe_and_remote_camera_report_encoding(make_service, quality, expected):
    service = make_service(jpeg_quality=quality)
    description = service.dispatch("robot.describe")
    assert description["image_encoding"] == expected
    # An older client unpacks each camera dict as CameraConfig(**dict): no extra keys allowed.
    assert CameraConfig(**description["cameras"]["wrist"]).name == "wrist"
    remote = RemoteRobot.from_client(LocalClient(service))
    remote.connect()
    try:
        assert remote.cameras["wrist"].image_encoding == expected
        color = remote.cameras["wrist"].read().color
        assert color.dtype == np.uint8 and color.shape == FRAME.shape
        assert np.array_equal(color, FRAME) == (quality is None)
        assert remote.get_observation()["cameras"]["wrist"]["color"].shape == FRAME.shape
    finally:
        remote.disconnect()


def test_grpc_loopback_decodes_jpeg(make_service):
    pytest.importorskip("grpc")
    from real_world_rl.devices import Robot
    from real_world_rl.server.transport import GrpcTransport

    service = make_service(jpeg_quality=90)
    transport = GrpcTransport()
    transport.start(service, "127.0.0.1:0")
    remote = Robot.from_dict({"type": "remote", "address": f"127.0.0.1:{transport.port}", "timeout_s": 5.0})
    try:
        remote.connect()
        assert remote.cameras["wrist"].image_encoding == {"jpeg": 90}
        color = remote.cameras["wrist"].read().color
        assert color.dtype == np.uint8 and color.shape == FRAME.shape
        observed = remote.get_observation()["cameras"]["wrist"]["color"]
        assert observed.dtype == np.uint8 and observed.shape == FRAME.shape
    finally:
        remote.disconnect()
        transport.stop(grace_s=0.1)


@pytest.mark.parametrize("quality", [0, 101, -1])
def test_invalid_quality_rejected(quality):
    with pytest.raises(ValueError, match="jpeg_quality"):
        RobotService(make_robot(), jpeg_quality=quality)


def test_missing_encoder_libraries_fail_at_construction(monkeypatch):
    monkeypatch.setitem(sys.modules, "PIL", None)
    monkeypatch.setitem(sys.modules, "cv2", None)
    with pytest.raises(SdkMissing) as info:
        RobotService(make_robot(), jpeg_quality=90)
    assert "[jpeg]" in info.value.hint
    RobotService(make_robot()).close()  # raw mode needs no encoder


def test_unknown_health_keys_are_ignored(make_service):
    assert _rebuild(Health, {"ok": True, "detail": "x", "future_field": 1}) == Health(ok=True, detail="x")
    service = make_service()
    original = service._methods["arm.health"]
    service._methods["arm.health"] = lambda: {**original(), "future_field": 1}
    remote = RemoteRobot.from_client(LocalClient(service))
    remote.connect()
    try:
        assert remote.arm.health().ok
    finally:
        remote.disconnect()


def block_jpeg_libraries(monkeypatch):
    import builtins

    real_import = builtins.__import__

    def guarded(name, *args, **kwargs):
        if name.split(".")[0] in ("PIL", "cv2"):
            raise ImportError(name)
        return real_import(name, *args, **kwargs)

    monkeypatch.setattr(builtins, "__import__", guarded)


def test_client_without_jpeg_libraries_raises_sdk_missing_on_unpack(make_service, monkeypatch):
    wire = pack(make_service(jpeg_quality=90).dispatch("camera.wrist.read", {"timeout_s": 2.0}))
    block_jpeg_libraries(monkeypatch)
    with pytest.raises(SdkMissing) as info:
        unpack(wire)
    assert "[jpeg]" in info.value.hint


def test_client_without_jpeg_libraries_fails_at_connect(make_service, monkeypatch):
    service = make_service(jpeg_quality=90)
    block_jpeg_libraries(monkeypatch)
    remote = RemoteRobot.from_client(LocalClient(service))
    with pytest.raises(SdkMissing) as info:
        remote.connect()
    assert "[jpeg]" in info.value.hint


@pytest.mark.parametrize("quality", [None, 85])
def test_negotiated_frame_preprocessing(make_service, fixed_frames, quality):
    from real_world_rl.devices.preprocess import CameraPreprocessConfig, preprocess_frame
    from real_world_rl.server.codec import encode_jpeg

    fixed_frames[:] = FRAME[..., 0] / 128.0
    fixed_frames[::3] = np.nan
    service = make_service(jpeg_quality=quality, enable_depth=True)
    remote = RemoteRobot.from_client(LocalClient(service))
    remote.connect()
    config = dict(source_resolution=[HEIGHT, WIDTH], roi=[7, 11, 51, 81], depth_clip_m=[0.1, 0.4])
    size = (13, 19)
    request = {"wrist": dict(size=size, config=config, color=True, depth=True)}
    try:
        native = remote.get_observation()
        result = remote.get_observation(frame_preprocess=request)
        assert result["frames_preprocessed"]
        camera = result["cameras"]["wrist"]
        preprocessing = CameraPreprocessConfig(**config)
        expected = preprocess_frame(FRAME, preprocessing, size)
        if quality is not None:
            expected = unpack(pack(encode_jpeg(expected, quality)))
        np.testing.assert_array_equal(camera["color"], expected)
        depth = native["cameras"]["wrist"]["depth"]
        np.testing.assert_array_equal(camera["depth"], preprocess_frame(depth, preprocessing, size, depth=True))
        service.dispatch("arm.set_control_mode", {"mode": "joint_servo"})
        stepped = remote.step(ACTION, frame_preprocess=request)
        np.testing.assert_array_equal(stepped["cameras"]["wrist"]["color"], expected)
        if quality is None:
            assert len(pack(result)) < len(pack(native)) / 4
    finally:
        remote.disconnect()


def test_cv2_advertisement_requires_opencv(make_service, monkeypatch):
    service = make_service()
    monkeypatch.setitem(sys.modules, "cv2", None)
    description = service.dispatch("robot.describe")
    assert description["frame_preprocess_v1"]
    assert not description["frame_preprocess_cv2_v1"]


@pytest.mark.parametrize("cv2_support", ["supported", "unsupported", "legacy", "cv2_only"])
def test_cv2_env_negotiation_and_exact_fallback(make_service, fixed_frames, cv2_support):
    from real_world_rl.devices.preprocess import CameraPreprocessConfig, preprocess_frame
    from real_world_rl.envs.real_robot import RealRobotEnv, RealRobotEnvConfig

    try:
        import cv2  # noqa: F401
    except ImportError as error:
        pytest.skip(f"OpenCV unavailable: {error}")

    fixed_frames[:] = FRAME[..., 0] / 128.0
    fixed_frames[::3] = np.nan
    service = make_service(enable_depth=True)
    description = service.dispatch("robot.describe")
    assert description["frame_preprocess_cv2_v1"]
    if cv2_support == "legacy":
        description.pop("frame_preprocess_cv2_v1")
    elif cv2_support == "unsupported":
        description["frame_preprocess_cv2_v1"] = False
    elif cv2_support == "cv2_only":
        description["frame_preprocess_v1"] = False
    service._methods["robot.describe"] = lambda: description
    negotiated = cv2_support in ("supported", "cv2_only")

    # Validate both paths against equivalent client preprocessing of the same source.
    size = (13, 17)
    region = (0.13, 0.24, 0.83, 0.91)
    preprocessing = CameraPreprocessConfig(resize="cv2_linear", crop_region=region)
    env = RealRobotEnv(
        RealRobotEnvConfig(
            remote="local", home_joints=list(FRANKA_HOME_JOINTS), reset_settle_s=0, image_size=size,
            cameras=["wrist"], depth_cameras=["wrist"], preprocess={"wrist": preprocessing},
        ),
        robot=RemoteRobot.from_client(LocalClient(service)),
    )
    try:
        observation, _ = env.reset()
        assert bool(env._last_raw.get("frames_preprocessed")) == negotiated
        native_or_processed = env._last_raw["cameras"]["wrist"]["color"]
        assert native_or_processed.shape[:2] == (size if negotiated else (HEIGHT, WIDTH))
        resolved = preprocessing.for_source(HEIGHT, WIDTH)
        np.testing.assert_array_equal(observation["frames"]["wrist"], preprocess_frame(FRAME, resolved, size))
        np.testing.assert_array_equal(
            observation["depth_frames"]["wrist"], preprocess_frame(fixed_frames, resolved, size, depth=True),
        )
        request = env._frame_kwargs["frame_preprocess"]["wrist"]["config"]
        assert request["roi"] == [12, 30, 67, 86]
        assert request["source_resolution"] == [HEIGHT, WIDTH]
        assert request["resize"] == "cv2_linear" and "crop_region" not in request
    finally:
        env.close()


def test_pil_request_keeps_original_daemon_wire_config(make_service):
    service = make_service()
    description = service.dispatch("robot.describe")
    description.pop("frame_preprocess_cv2_v1")
    service._methods["robot.describe"] = lambda: description
    remote = RemoteRobot.from_client(LocalClient(service))
    remote.connect()
    try:
        request = {"wrist": dict(size=(7, 9), config={"resize": "pil_bilinear"}, color=True, depth=False)}
        assert remote._frame_kwargs(request)["frame_preprocess"]["wrist"]["config"] == {}
        assert remote.get_observation(frame_preprocess=request)["frames_preprocessed"]
    finally:
        remote.disconnect()
