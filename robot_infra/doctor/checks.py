"""Layered diagnostic: 0 deps, 1 sdk, 2 os, 3 handshake, 4 feedback, 5 mode. Each check is a
small closure so ``fix`` can run its autofix and then re-run exactly that check."""
from __future__ import annotations

import dataclasses
import glob
import math
import os
from dataclasses import dataclass, field
from typing import Callable

from robot_infra.devices import (
    Arm,
    Camera,
    DeviceError,
    DeviceFault,
    Gripper,
    Robot,
    RobotConfig,
    SdkMissing,
    Teleop,
    VersionMismatch,
)
from robot_infra.devices.backends.arms.franky import (
    franka_unreachable_hint,
    parse_libfranka_mismatch,
    server_version_cross_check,
)
from robot_infra.devices.backends.teleops._spacemouse_hid import UDEV_HINT
from robot_infra.devices.claims import DeviceClaim, claim_key
from robot_infra.discovery import enumerate as probes
from robot_infra.doctor import probes as os_probes
from robot_infra.doctor.deps import deps_outcome

LAYER_NAMES = {0: "deps", 1: "sdk", 2: "os", 3: "handshake", 4: "feedback", 5: "mode"}
LIBFRANKA_SERVER_VERSION = "libfranka_server_version"
LIBFRANKA_LIBRARY_VERSION = "libfranka_library_version"
_FRANKA_FCI_PORT = 1337
_CAMERA_READY_TIMEOUT_S = 2.0
_HIDRAW_GLOB = "/dev/hidraw*"
_SERIAL_GROUP = "dialout"
_EVDEV_GROUP = "input"
_UNREACHABLE_HINTS = {"franky": franka_unreachable_hint}
_GENERIC_PING_HINT = "check the Ethernet cable to the robot's Control port and the robot IP in Desk"


@dataclass
class CheckResult:
    layer: int
    name: str
    ok: bool
    message: str
    hint: str = ""
    skipped: bool = False
    # ok, but worth acting on (printed as WARN with its hint); never fails the run.
    warning: bool = False
    autofix: Callable[[], None] | None = field(default=None, repr=False, compare=False)
    # Structured findings for the CLI (e.g. the libfranka versions); not part of the JSON report.
    details: dict = field(default_factory=dict, repr=False, compare=False)


Outcome = tuple | CheckResult  # tuples: (ok, message[, hint[, autofix]]); CheckResult also supports WARN
Check = Callable[[], CheckResult]


def _make_check(layer: int, name: str, fn: Callable[[], Outcome]) -> Check:
    def run() -> CheckResult:
        try:
            outcome = fn()
        except DeviceError as error:
            return CheckResult(layer, name, False, error.message, error.hint)
        except Exception as error:
            return CheckResult(layer, name, False, repr(error))
        if isinstance(outcome, CheckResult):
            return outcome
        ok, message = outcome[0], outcome[1]
        if ok:
            return CheckResult(layer, name, True, message)
        hint = outcome[2] if len(outcome) > 2 else ""
        autofix = outcome[3] if len(outcome) > 3 else None
        return CheckResult(layer, name, False, message, hint, autofix=autofix)

    return run


def _guarded(layer: int, name: str, fn: Callable[[], CheckResult]) -> Check:
    """A check that builds its own ``CheckResult`` (e.g. a warning); a raised error is a failure."""

    def run() -> CheckResult:
        try:
            return fn()
        except Exception as error:
            return CheckResult(layer, name, False, repr(error))

    return run


def _skipped(layer: int, name: str, message: str) -> CheckResult:
    return CheckResult(layer, name, True, message, skipped=True)


def _mocked(config: RobotConfig) -> RobotConfig:
    gripper = config.gripper
    return dataclasses.replace(
        config,
        arm_backend="fake",
        gripper=dataclasses.replace(gripper, backend="fake") if gripper else None,
        cameras=[dataclasses.replace(camera, backend="fake") for camera in config.cameras],
        teleop=_mocked_teleop(config.teleop),
    )


mocked_config = _mocked


def _mocked_teleop(teleop):
    if teleop is None or teleop.backend == "fake":
        return teleop
    # Another backend's private options mean nothing to the fake.
    return dataclasses.replace(teleop, backend="fake", options={})


def _sdk_outcome(backend_cls: type) -> Outcome:
    if backend_cls.SDK is None:
        return True, "no SDK needed"
    try:
        backend_cls.require_sdk()
    except (SdkMissing, VersionMismatch) as error:
        return False, error.message, error.hint
    return True, f"'{backend_cls.SDK}' importable"


def _sdk_layer(config: RobotConfig) -> tuple[list[Check], bool]:
    """Returns the layer-1 checks and whether every backend name resolved."""
    targets: list[tuple[str, Callable[[], type]]] = [
        ("sdk:arm", lambda: Arm.backend(config.arm_backend))
    ]
    if config.gripper:
        targets.append(("sdk:gripper", lambda: Gripper.backend(config.gripper.backend)))
    for camera in config.cameras:
        targets.append(
            (f"sdk:camera:{camera.name}", lambda camera=camera: Camera.backend(camera.backend))
        )
    if config.teleop:
        targets.append(
            (f"sdk:teleop:{config.teleop.backend}", lambda: Teleop.backend(config.teleop.backend))
        )
    checks: list[Check] = []
    resolved = True
    for name, resolve in targets:
        try:
            backend_cls = resolve()
            fn = lambda backend_cls=backend_cls: _sdk_outcome(backend_cls)
        except ValueError as error:
            resolved = False
            fn = lambda error=error: (
                False,
                str(error),
                "pip install the backend extra or check arm_backend",
            )
        checks.append(_make_check(1, name, fn))
    return checks, resolved


def _os_checks(config: RobotConfig) -> list[tuple[str, Check]]:
    named: list[tuple[str, Callable[[], Outcome]]] = []
    robot_ip = getattr(config, "robot_ip", None)
    if robot_ip:
        unreachable_hint = _UNREACHABLE_HINTS.get(config.arm_backend)
        ping_hint = unreachable_hint(robot_ip) if unreachable_hint else _GENERIC_PING_HINT
        named += [("os:ping", _ping_outcome(robot_ip, ping_hint)), ("os:tcp", _tcp_outcome(robot_ip))]
    can_interface = getattr(config, "can_interface", None)
    if can_interface:
        named += _can_outcomes(can_interface, getattr(config, "bitrate", None))
    arm_needs_no_device = _backend_needs_no_device(Arm, config.arm_backend)
    checks: list[tuple[str, Check]] = []
    for name, fn in named:
        if arm_needs_no_device:
            checks.append((name, _no_device_check(name, config.arm_backend)))
        else:
            checks.append((name, _make_check(2, name, fn)))
    for camera in config.cameras:
        name = f"os:camera:{camera.name}"
        if _backend_needs_no_device(Camera, camera.backend):
            checks.append((name, _no_device_check(name, camera.backend)))
        else:
            checks.append((name, _make_check(2, name, _camera_outcome(camera))))
    if config.teleop:
        name = f"os:teleop:{config.teleop.name}"
        if _backend_needs_no_device(Teleop, config.teleop.backend):
            checks.append((name, _no_device_check(name, config.teleop.backend)))
        elif config.teleop.backend == "spacemouse":
            checks.append((name, _make_check(2, name, _spacemouse_outcome(config.teleop, name))))
        else:
            checks.append((name, _make_check(2, name, _teleop_outcome(config.teleop))))
        if config.teleop.backend == "spacemouse":
            checks.append(("os:hid", _make_check(2, "os:hid", _hid_outcome())))
        if config.teleop.backend == "gello":
            checks += _gello_device_checks(config.teleop)
    return checks


def _gello_device_checks(teleop) -> list[tuple[str, Check]]:
    """The serial port, the groups its devices need, brltty on CH340 adapters and the USB latency timer."""
    port = teleop.options.get("port")
    event_device = teleop.options.get("event_device")
    checks: list[tuple[str, Check]] = [("os:serial", _make_check(2, "os:serial", _serial_outcome(port)))]
    group_devices = {group: device for group, device in ((_SERIAL_GROUP, port), (_EVDEV_GROUP, event_device)) if device}
    if group_devices:
        checks.append(("os:groups", _guarded(2, "os:groups", lambda: _group_membership_result(group_devices))))
    if port:
        checks.append(("os:brltty", _guarded(2, "os:brltty", lambda: _brltty_result(port))))
        checks.append(("os:latency_timer", _guarded(2, "os:latency_timer", lambda: _latency_timer_result(port))))
    return checks


def _backend_needs_no_device(part_cls: type, backend: str) -> bool:
    try:
        return part_cls.backend(backend).SDK is None
    except ValueError:
        return False


def _no_device_check(name: str, backend: str) -> Check:
    return lambda: _skipped(2, name, f"backend '{backend}' needs no OS device")


def _ping_outcome(ip: str, hint: str = _GENERIC_PING_HINT) -> Callable[[], Outcome]:
    def outcome() -> Outcome:
        reachable = probes.ping(ip)
        return reachable, f"{ip} {'replies to ping' if reachable else 'does not reply'}", hint

    return outcome


def _tcp_outcome(ip: str) -> Callable[[], Outcome]:
    def outcome() -> Outcome:
        state = "open" if probes.tcp_open(ip, _FRANKA_FCI_PORT) else "closed"
        return True, f"port {_FRANKA_FCI_PORT} {state} (informational)"

    return outcome


def _can_outcomes(iface: str, bitrate: int | None) -> list[tuple[str, Callable[[], Outcome]]]:
    activate_hint = (
        f"run `bash can_activate.sh {iface} {bitrate}` from piper_sdk, or "
        f"`sudo ip link set {iface} up type can bitrate {bitrate}`"
    )

    def exists() -> Outcome:
        found = iface in probes.list_can_interfaces()
        return found, f"CAN interface {iface} {'exists' if found else 'not found'}", activate_hint

    def up() -> Outcome:
        info = probes.list_can_interfaces().get(iface)
        state = info.operstate if info else "missing"
        return state == "up", f"{iface} operstate={state}", activate_hint

    def bitrate_matches() -> Outcome:
        info = probes.list_can_interfaces().get(iface)
        actual = info.bitrate if info else None
        return (
            actual == bitrate,
            f"{iface} bitrate={actual}, expected {bitrate}",
            f"expected bitrate {bitrate} but found {actual}; " + activate_hint,
        )

    return [("os:can_exists", exists), ("os:can_up", up), ("os:can_bitrate", bitrate_matches)]


def _camera_outcome(camera) -> Callable[[], Outcome]:
    def outcome() -> Outcome:
        discovered = sorted(Camera.backend(camera.backend).discover())
        if not camera.serial and len(discovered) == 1:
            return True, f"will use {discovered[0]}"
        if camera.serial in discovered:
            return True, f"serial {camera.serial} found"
        if not camera.serial:
            if not discovered:
                return False, "no RealSense device found (serial unset)", "plug in the camera and run `rwrl discover`"
            return (
                False,
                f"serial unset and {len(discovered)} devices attached; set serial to one of {discovered}",
                f"set serial; discovered: {discovered}",
            )
        return (
            False,
            f"camera '{camera.name}' serial '{camera.serial}' not found",
            f"set serial; discovered: {discovered}",
        )

    return outcome


def _teleop_outcome(teleop) -> Callable[[], Outcome]:
    def outcome() -> Outcome:
        discovered = sorted(Teleop.backend(teleop.backend).discover())
        if discovered:
            message = f"teleop '{teleop.name}' devices: {discovered}"
        else:
            message = f"no '{teleop.backend}' teleop device found"
        return (
            bool(discovered),
            message,
            "plug in the teleop device and run `rwrl discover`",
        )

    return outcome


def _spacemouse_outcome(teleop, name: str) -> Callable[[], Outcome]:
    """``_teleop_outcome`` against the teleop's device table, naming HID devices that look like a
    SpaceMouse but are missing from it (WARN beside a supported one, FAIL without one)."""

    def outcome() -> Outcome:
        from robot_infra.devices.backends.teleops import _spacemouse_hid

        try:
            table = _spacemouse_hid.load_device_specs(options=teleop.options)
        except ValueError as error:
            return False, str(error), f"fix the device table entry; see {_spacemouse_hid.SITE_TABLE_DOC}"
        discovered = sorted(Teleop.backend(teleop.backend).discover(table=table))
        unknown = "; ".join(device["message"] + f" ({device['path']})"
                            for device in _spacemouse_hid.unknown_hid_devices(table))
        if discovered:
            message = f"teleop '{teleop.name}' devices: {discovered}"
            if unknown:
                return CheckResult(2, name, True, message, unknown, warning=True)
            return True, message
        if unknown:
            return False, f"no supported '{teleop.backend}' teleop device found; {unknown}", (
                f"add the device to the local device table; see {_spacemouse_hid.SITE_TABLE_DOC}")
        return (
            False,
            f"no '{teleop.backend}' teleop device found",
            "plug in the teleop device and run `rwrl discover`",
        )

    return outcome


def _hid_outcome(pattern: str | None = None) -> Callable[[], Outcome]:
    def outcome() -> Outcome:
        paths = sorted(glob.glob(pattern or _HIDRAW_GLOB))
        if not paths:
            return (
                False,
                "no HID device found",
                "no /dev/hidraw* node: check the USB cable and port; inside Docker pass it with "
                "`docker run --device=/dev/hidrawN`",
            )
        denied = [path for path in paths if not os.access(path, os.R_OK | os.W_OK)]
        if denied:
            return (
                False,
                f"no read/write access to {denied}",
                UDEV_HINT + "; inside Docker pass the node with "
                "`docker run --device=/dev/hidrawN`",
            )
        return True, f"{len(paths)} HID device(s) readable and writable: {paths}"

    return outcome


def _serial_outcome(port: str | None) -> Callable[[], Outcome]:
    def outcome() -> Outcome:
        if not port:
            return False, "teleop option 'port' is unset", "set teleop.options.port to /dev/serial/by-id/..."
        if not os.path.exists(port):
            return (
                False,
                f"serial port {port} not found",
                "check the USB cable to the U2D2 / OpenRB-150 and list /dev/serial/by-id/; inside "
                "Docker pass the node with `docker run --device=<port>`",
            )
        if not os.access(port, os.R_OK | os.W_OK):
            return (
                False,
                f"no read/write access to {port}",
                f"grant this user access now: `setfacl -m u:$USER:rw {port}` (prefix sudo unless you own "
                "the node; --fix tries it without sudo; lost on replug); for good add the user to the "
                f"'{_SERIAL_GROUP}' group (`sudo usermod -aG {_SERIAL_GROUP} $USER`, then log in again) or add "
                'a udev rule giving the adapter MODE="0666"',
                lambda: os_probes.grant_user_acl(port),
            )
        return True, f"serial port {port} readable and writable"

    return outcome


def _accessible(path: str) -> bool:
    return os.access(path, os.R_OK | os.W_OK)


def _group_membership_result(group_devices: dict[str, str]) -> CheckResult:
    """FAIL when a group is missing and its device exists but is not accessible; WARN when the
    device is reachable anyway (an ACL or udev rule, which a replug or reboot may drop)."""
    name = "os:groups"
    missing = [group for group in group_devices if group not in os_probes.session_group_names()]
    if not missing:
        return CheckResult(2, name, True, f"user in group(s) {', '.join(group_devices)}")
    user = os_probes.current_user()
    pending = [group for group in missing if user in os_probes.listed_group_members(group)]
    if pending == missing:
        hint = f"log out and back in so this session picks up {', '.join(missing)} (or run `newgrp {missing[0]}`)"
    else:
        hint = f"sudo usermod -aG {','.join(missing)} $USER, then log out and back in"
    message = f"not in group(s) {', '.join(missing)}"
    if pending:
        message += f" (added to {', '.join(pending)} after this login)"
    denied = [
        group_devices[group]
        for group in missing
        if os.path.exists(group_devices[group]) and not _accessible(group_devices[group])
    ]
    if denied:
        return CheckResult(2, name, False, f"{message}; no read/write access to {denied}", hint)
    return CheckResult(
        2,
        name,
        True,
        f"{message}; the devices are reachable only through an ACL or udev rule, which a replug or reboot may drop",
        hint,
        warning=True,
    )


def _is_ch340(port: str) -> bool:
    if os_probes.CH340_VENDOR_ID in port.lower():
        return True
    tty = os_probes.serial_tty(port)
    driver = os_probes.usb_serial_driver(tty) if tty else None
    if driver is not None:
        return driver.startswith("ch34")
    return os_probes.CH340_VENDOR_ID in os_probes.usb_vendor_ids()


def _brltty_result(port: str) -> CheckResult:
    """brltty's udev rules claim CH340 (1a86:7523) adapters as braille displays and the tty vanishes."""
    name = "os:brltty"
    if not os_probes.brltty_installed():
        return CheckResult(2, name, True, "brltty not installed")
    if os_probes.brltty_udev_masked():
        return CheckResult(2, name, True, f"brltty installed, {os_probes.BRLTTY_UDEV_UNIT} masked")
    if not _is_ch340(port):
        return CheckResult(2, name, True, "brltty active; the adapter is not a CH340")
    hint = "sudo systemctl mask --now brltty-udev.service brltty.service, then replug the adapter"
    if os.path.exists(port):
        return CheckResult(
            2,
            name,
            True,
            f"brltty active and the adapter is a CH340 (USB id {os_probes.CH340_VENDOR_ID}); "
            "brltty may claim it on the next replug",
            hint,
            warning=True,
        )
    return CheckResult(
        2,
        name,
        False,
        f"brltty active and a CH340 adapter (USB id {os_probes.CH340_VENDOR_ID}) is attached; "
        f"brltty has likely claimed {port}",
        hint,
    )


def _latency_timer_result(port: str) -> CheckResult:
    """FTDI adapters buffer reads for ``latency_timer`` ms (16 by default); 1 ms keeps Dynamixel reads fast."""
    name = "os:latency_timer"
    if not os.path.exists(port):
        return _skipped(2, name, f"serial port {port} not found (see os:serial)")
    tty = os_probes.serial_tty(port)
    driver = os_probes.usb_serial_driver(tty) if tty else None
    if driver is None:
        return _skipped(2, name, f"{port} is not a USB-serial adapter tty")
    latency = os_probes.latency_timer_ms(tty)
    if latency is None:
        return CheckResult(2, name, True, f"{tty} ({driver}) has no latency_timer")
    if latency == 1:
        return CheckResult(2, name, True, f"{tty} latency_timer is 1 ms")
    timer_path = os_probes.USB_SERIAL_SYSFS / tty / "latency_timer"
    return CheckResult(
        2,
        name,
        True,
        f"{tty} latency_timer is {latency} ms; 1 ms is recommended for Dynamixel reads",
        f'add the udev rule `ACTION=="add", SUBSYSTEM=="usb-serial", DRIVER=="{driver}", '
        'ATTR{latency_timer}="1"` to /etc/udev/rules.d/99-usb-serial-latency.rules, then '
        "`sudo udevadm control --reload && sudo udevadm trigger` and replug; until the next replug: "
        f"`echo 1 | sudo tee {timer_path}`",
        warning=True,
    )


def run_doctor(
    config: RobotConfig,
    *,
    mock: bool = False,
    fix: bool = False,
    config_path: str | None = None,
    serves_daemon: bool = False,
) -> list[CheckResult]:
    """``config_path`` only fills the ``--config`` of the commands quoted in hints.
    ``serves_daemon`` (the station declares a ``daemon:`` block) makes grpc a required extra;
    ``--mock`` never requires it."""
    if mock:
        config = _mocked(config)
    results: list[CheckResult] = []
    reruns: dict[int, Check] = {}

    def record(check: Check) -> CheckResult:
        result = check()
        reruns[len(results)] = check
        results.append(result)
        return result

    record(_guarded(0, LAYER_NAMES[0], lambda: _deps_result(config, serves_daemon=serves_daemon and not mock)))
    sdk_checks, backends_resolved = _sdk_layer(config)
    for check in sdk_checks:
        record(check)

    for name, check in _os_checks(config):
        if mock:
            results.append(_skipped(2, name, "skipped (--mock)"))
        else:
            record(check)

    if not backends_resolved:
        for layer in (3, 4, 5):
            results.append(_skipped(layer, LAYER_NAMES[layer], "skipped: unknown backend"))
        return results

    robot: Robot | None = None
    claim: DeviceClaim | None = None
    try:
        robot = Robot.from_config(config)
        claim = DeviceClaim(claim_key(robot)) if config.claim else None
        handshake = record(_handshake_check(robot, claim, config_path))
        if not handshake.ok:
            for layer in (4, 5):
                results.append(_skipped(layer, LAYER_NAMES[layer], "skipped: handshake failed"))
        else:
            _feedback_and_mode_layers(robot, record, results, reruns)
        if fix:
            _apply_fixes(results, reruns)
    except Exception as error:
        results.append(CheckResult(3, "handshake", False, repr(error)))
    finally:
        if robot is not None:
            _disconnect(robot, results)
        if claim is not None and claim.held:
            claim.release()
    return results


def _deps_result(config: RobotConfig, *, serves_daemon: bool) -> CheckResult:
    outcome = deps_outcome(config, serves_daemon=serves_daemon)
    return CheckResult(0, LAYER_NAMES[0], outcome.ok, outcome.message, outcome.hint, warning=outcome.warning)


def libfranka_mismatch_hint(server_version: int, config_path: str | None) -> str:
    return (
        f"{server_version_cross_check(server_version)}; then: "
        f"rwrl doctor --station {config_path or '<cfg>'} --install-sdk"
    )


def _version_mismatch_result(error: VersionMismatch, robot: Robot, config_path: str | None) -> CheckResult:
    """A libfranka mismatch of the arm names both versions; the CLI's --install-sdk reads them
    from ``details``. The franky wheel index is keyed by robot server version, so a gripper
    mismatch keeps the backend's own message."""
    versions = parse_libfranka_mismatch(error.message)
    if versions is None or error.device != robot.arm.name:
        return CheckResult(3, "handshake", False, error.message, error.hint)
    server_version, library_version = versions
    return CheckResult(
        3,
        "handshake",
        False,
        f"libfranka mismatch: robot server {server_version}, installed library {library_version}",
        libfranka_mismatch_hint(server_version, config_path),
        details={LIBFRANKA_SERVER_VERSION: server_version, LIBFRANKA_LIBRARY_VERSION: library_version},
    )


def _handshake_check(robot: Robot, claim: DeviceClaim | None, config_path: str | None = None) -> Check:
    def run() -> CheckResult:
        try:
            if claim is not None:
                claim.acquire()
            robot.connect()
        except VersionMismatch as error:
            return _version_mismatch_result(error, robot, config_path)
        except DeviceFault as error:
            return CheckResult(3, "handshake", False, error.message, error.hint)
        except DeviceError as error:
            return CheckResult(3, "handshake", False, error.message, error.hint)
        except Exception as error:
            return CheckResult(3, "handshake", False, repr(error))
        return CheckResult(3, "handshake", True, "robot connected")

    return run


def _feedback_and_mode_layers(robot: Robot, record, results: list[CheckResult], reruns) -> None:
    arm = robot.arm

    def arm_feedback() -> Outcome:
        arm.get_observation()
        received = arm.health().first_feedback_received
        return (
            received,
            "arm feedback received" if received else "arm reports no feedback",
            "the arm reports no feedback: check power, e-stop and, for CAN arms, follower/slave mode",
        )

    record(_make_check(4, "feedback:arm", arm_feedback))
    if robot.gripper is not None:
        gripper = robot.gripper

        def gripper_feedback() -> Outcome:
            gripper.get_observation()
            norm = gripper.width
            max_w = gripper.max_width_m
            metres = norm * max_w
            grasped = gripper.is_grasped
            message = (
                f"gripper width {metres:.4f} m ({norm:.2f} of max_width_m {max_w:.3f}), "
                f"grasped={grasped if grasped is not None else 'unknown'}, source {type(gripper).__name__}"
            )
            warning = not math.isfinite(norm) or not 0 <= norm <= 1 or not math.isfinite(max_w) or max_w <= 0
            hint = "reading is untrustworthy; check max_width_m and the SDK's width units" if warning else ""
            return CheckResult(4, "feedback:gripper", True, message, hint, warning=warning)

        record(_make_check(4, "feedback:gripper", gripper_feedback))
    for name, camera in robot.cameras.items():
        record(_camera_feedback_check(name, camera))
    if robot.teleop is not None:
        record(_make_check(4, f"feedback:teleop:{robot.teleop.name}", _teleop_feedback(robot.teleop)))

    try:
        mode_results = arm.preflight()
    except Exception as error:
        mode_results = [CheckResult(5, "mode:preflight", False, repr(error))]
    if not mode_results:
        record(_make_check(5, "mode:none", lambda: (True, "backend declares no mode checks")))
    for mode_result in mode_results:
        reruns[len(results)] = _preflight_rerun(arm, mode_result)
        results.append(mode_result)


def _camera_feedback_check(name: str, camera: Camera) -> Check:
    def run() -> CheckResult:
        ready = camera.is_ready(_CAMERA_READY_TIMEOUT_S)
        if ready:
            return CheckResult(4, f"feedback:camera:{name}", True, "frames arriving")
        return CheckResult(
            4,
            f"feedback:camera:{name}",
            False,
            "no frames within 2s",
            "no frames within 2s: check USB 3 bandwidth; try another port; run with --fix to reopen",
            autofix=camera.reopen,
        )

    return run


def _teleop_feedback(teleop: Teleop) -> Callable[[], Outcome]:
    def outcome() -> Outcome:
        health = teleop.health()
        teleop.latest()
        problem = health.detail or "poll thread not running"
        return (
            health.ok,
            "teleop poll thread alive" if health.ok else f"teleop unhealthy: {problem}",
            "check the teleop device cable and permissions, then reconnect",
        )

    return outcome


def _preflight_rerun(arm: Arm, original: CheckResult) -> Check:
    def run() -> CheckResult:
        for result in arm.preflight():
            if result.name == original.name:
                return result
        return original

    return run


def _apply_fixes(results: list[CheckResult], reruns: dict[int, Check]) -> None:
    for index, result in enumerate(list(results)):
        if result.ok or result.autofix is None or index not in reruns:
            continue
        try:
            result.autofix()
        except Exception as error:
            result.message = f"{result.message}; autofix failed: {error!r}"
            continue
        results[index] = reruns[index]()


def _disconnect(robot: Robot, results: list[CheckResult]) -> None:
    try:
        robot.disconnect()
    except Exception as error:
        results.append(CheckResult(3, "disconnect", False, repr(error)))


def summarize(results: list[CheckResult]) -> tuple[int, int, int]:
    skipped = sum(result.skipped for result in results)
    failed = sum(not result.ok and not result.skipped for result in results)
    return len(results) - skipped - failed, failed, skipped


def exit_code(results: list[CheckResult]) -> int:
    return 1 if summarize(results)[1] else 0
