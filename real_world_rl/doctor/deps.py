"""Doctor layer 0: which pip extras of ``real-world-rl`` a robot config needs, and
whether their modules import. Also the franky wheel install command for ``--install-sdk``."""
from __future__ import annotations

import importlib.util
import sys
from dataclasses import dataclass

from real_world_rl.devices import RobotConfig
from real_world_rl.devices.backends.arms.franky import FRANKY_DISTRIBUTION, FRANKY_WHEEL_INDEX_URL

# extra -> modules; the extra counts as installed when any of them imports.
EXTRA_MODULES: dict[str, tuple[str, ...]] = {
    "franky": ("franky",),
    "pyagxarm": ("pyAgxArm",),
    "piper-sdk": ("piper_sdk",),
    "realsense": ("pyrealsense2",),
    "spacemouse": ("hid",),
    "gello": ("dynamixel_sdk",),
    "gello-events": ("evdev",),
    "grpc": ("grpc",),
    "jpeg": ("PIL", "cv2"),
}
# backend name -> extra; backends missing here (fakes, serl_http) need none.
ARM_BACKEND_EXTRAS = {"franky": "franky", "pyagxarm": "pyagxarm", "piper_sdk": "piper-sdk"}
GRIPPER_BACKEND_EXTRAS = {"franky": "franky", "pyagxarm": "pyagxarm", "piper_sdk": "piper-sdk"}
CAMERA_BACKEND_EXTRAS = {"realsense": "realsense"}
TELEOP_BACKEND_EXTRAS = {"spacemouse": "spacemouse", "gello": "gello"}
GELLO_EVENTS_EXTRA = "gello-events"
# The daemon's only transport: required when the station declares a ``daemon:`` block, else optional.
TRANSPORT_EXTRA = "grpc"
TRANSPORT_PURPOSE = "daemon route"
# extra -> what needs it; reported when missing but never a failure.
OPTIONAL_EXTRAS = {"jpeg": "daemon --jpeg-quality"}


@dataclass(frozen=True)
class DepsOutcome:
    ok: bool
    message: str
    hint: str = ""
    warning: bool = False


def required_extras(config: RobotConfig, *, serves_daemon: bool = False) -> list[str]:
    """Extras for the arm, gripper, cameras and teleop of ``config``, then the transport when the
    robot is served by the daemon, in order."""
    extras = [ARM_BACKEND_EXTRAS.get(config.arm_backend)]
    if config.gripper is not None:
        extras.append(GRIPPER_BACKEND_EXTRAS.get(config.gripper.backend))
    extras += [CAMERA_BACKEND_EXTRAS.get(camera.backend) for camera in config.cameras]
    if config.teleop is not None:
        extras.append(TELEOP_BACKEND_EXTRAS.get(config.teleop.backend))
        if config.teleop.backend == "gello" and config.teleop.options.get("event_device"):
            extras.append(GELLO_EVENTS_EXTRA)
    needed = [extra for extra in extras if extra]
    if serves_daemon:
        needed.append(TRANSPORT_EXTRA)
    return list(dict.fromkeys(needed))


def _importable(module: str) -> bool:
    try:
        return importlib.util.find_spec(module) is not None
    except (ImportError, ValueError):
        return False


def extra_installed(extra: str) -> bool:
    return any(_importable(module) for module in EXTRA_MODULES[extra])


def missing_extras(extras: list[str]) -> list[str]:
    return [extra for extra in extras if not extra_installed(extra)]


def install_extras_hint(extras: list[str]) -> str:
    joined = ",".join(extras)
    hint = f"pip install -e '.[{joined}]' (from the repository root)"
    if "franky" in extras:
        hint += "; if the handshake then reports a libfranka mismatch, rerun the doctor with --install-sdk"
    return hint


def deps_outcome(config: RobotConfig, *, serves_daemon: bool = False) -> DepsOutcome:
    """FAIL on a missing required extra. Without a ``daemon:`` block the single-process route needs
    no grpc, so a missing grpc is then only a warning; a missing jpeg is always just a note."""
    extras = required_extras(config, serves_daemon=serves_daemon)
    if not extras:
        return DepsOutcome(True, "no extras needed (fake or SDK-free backends)")
    missing = missing_extras(extras)
    if missing:
        return DepsOutcome(
            False, f"missing extras for this station: {', '.join(missing)}", install_extras_hint(missing)
        )
    optional = dict(OPTIONAL_EXTRAS) if serves_daemon else {TRANSPORT_EXTRA: TRANSPORT_PURPOSE, **OPTIONAL_EXTRAS}
    missing_optional = [extra for extra in optional if not extra_installed(extra)]
    message = f"extras ok: {', '.join(extras)}"
    if missing_optional:
        listed = ", ".join(f"{extra} ({optional[extra]})" for extra in missing_optional)
        message += f"; optional, not installed: {listed}"
    if TRANSPORT_EXTRA in missing_optional:
        hint = f"only needed for the {TRANSPORT_PURPOSE}: {install_extras_hint([TRANSPORT_EXTRA])}"
        return DepsOutcome(True, message, hint, warning=True)
    return DepsOutcome(True, message)


def franky_wheel_install_argv(server_version: int, python: str | None = None) -> list[str]:
    """``pip install`` of the franky wheel built for ``server_version``, with this interpreter."""
    return [
        python or sys.executable,
        "-m",
        "pip",
        "install",
        FRANKY_DISTRIBUTION,
        "--extra-index-url",
        FRANKY_WHEEL_INDEX_URL.format(server_version=server_version),
        "--force-reinstall",
        "--no-deps",
    ]
