"""Franka arm backend on franky (https://github.com/TimSchneider42/franky), imported lazily.

Servo modes stream targets into franky's torque-level impedance trackers; ``move_to_*`` runs
Ruckig-planned position motions. ``cartesian_controller: internal`` instead hands Cartesian servo
targets to the robot's own controller: each target becomes a Cartesian velocity motion (libfranka's
velocity interface, the robot solves the joint motion) that preempts the previous one; see
``_servo_internal``. That interface aborts on any 1 kHz discontinuity, so it wants a PREEMPT_RT
kernel (``realtime: enforce``). ``cartesian_controller: serl`` runs SERL's own controllers from the
``serl_controller`` wheel (built together with the franky wheel): Cartesian servo is SERL's
Cartesian impedance controller as a franky torque motion (targets are its equilibrium pose,
parameters live-updatable like SERL's dynamic_reconfigure), and ``move_to_joints`` is SERL's joint
reset (10 s linear joint blend, finished the way HIL-SERL's franka_server does). libfranka refuses
to switch control-signal type while a motion runs, so every mode change joins the running motion
and recovers before starting the next one.
"""
from __future__ import annotations

import inspect
import contextlib
import hashlib
import importlib.util
import json
import logging
import importlib.metadata
import re
import copy
import math
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from types import ModuleType
from typing import TYPE_CHECKING, Any, Iterator

import numpy as np

from real_world_rl.devices.arm import Arm, ControlMode
from real_world_rl.devices.control import (
    SERL_PARAM_NAMES,
    franka_load,
    resolve_serl_params,
    serl_auto_recover_config,
    serl_param_updates,
)
from real_world_rl.devices.errors import (
    DeviceFault,
    DeviceNotFound,
    SdkMissing,
    VersionMismatch,
)
from real_world_rl.devices.safety import ArmSafety

if TYPE_CHECKING:
    from real_world_rl.devices.robot import RobotConfig
    from real_world_rl.doctor.checks import CheckResult

logger = logging.getLogger(__name__)

REALTIME_KERNEL_FLAG_PATH = "/sys/kernel/realtime"
FRANKY_DISTRIBUTION = "franky-control"
FRANKY_WHEEL_INDEX_URL = "https://timschneider42.github.io/franky/whl/by-robot-server-version/{server_version}/"
DESK_SYSTEM_VERSION_PATH = "Desk → Settings → System"
FRANKA_COMPATIBILITY_URL = "https://frankarobotics.github.io/docs/compatibility.html"
FRANKY_VERSION_HINT = (
    "install the franky wheel matching your robot server version: pip install franky-control "
    f"--extra-index-url {FRANKY_WHEEL_INDEX_URL.format(server_version='<N>')} "
    f"(to find N, read the system version in {DESK_SYSTEM_VERSION_PATH} and look it up in the "
    f"compatibility matrix {FRANKA_COMPATIBILITY_URL}; `rwrl doctor --install-sdk` "
    "reads N from the handshake error and installs the wheel)"
)
_BLOCKING_ROBOT_MODES = ("Reflex", "UserStopped", "Guiding")
CARTESIAN_CONTROLLERS = ("impedance", "internal", "serl")
SERL_CONTROLLER_DISTRIBUTION = "serl-controller"
SERL_WHEELS_DOC = "docs/serl-controller.md (Installing franky and serl_controller)"
SERL_NATIVE_INSTALL = "native/serl_controller/scripts/install-on-ubuntu.sh"
SERL_INSTALL_HINT = (
    "franky and serl_controller must come from the same build: on the robot PC run "
    f"LIBFRANKA_VERSION=<x.y.z> bash {SERL_NATIVE_INSTALL} (builds both from source in the active env), "
    "or install both wheels of one wheel build directory together: pip uninstall -y franky-control "
    "serl-controller && pip install <dir>/franky_control-*.whl <dir>/serl_controller-*.whl; "
    f"see {SERL_WHEELS_DOC}"
)
# HIL-SERL's franka_server.reset_joint (rail-berkeley/hil-serl @ c32939b), after it launches the
# joint controller: sleep 1 s, clear errors, sleep 1 s, then poll the joints once per second until
# np.allclose(target - q, 0, atol=1e-2, rtol=1e-2) or 30 more polls ("joint reset TIMEOUT", not an
# error), stop the joint controller, clear errors; the caller restarts the impedance controller.
SERL_JOINT_RESET_FIRST_CHECK_S = 2.0
SERL_JOINT_RESET_POLL_S = 1.0
SERL_JOINT_RESET_MAX_POLLS = 30
SERL_JOINT_RESET_ATOL = 1e-2
SERL_JOINT_RESET_RTOL = 1e-2
# rwrl ends the joint motion with MotionFinished only once the 10 s blend is over (holding the
# target), instead of killing it mid-blend like terminating joint.launch does.
SERL_JOINT_RESET_FINISH_SLACK_S = 2.0
# franka_control's robot_description: franka_description robots/<model> (franka_control.launch
# default robot:=panda); its joint limits feed franka_hw's soft-limit stage. The station's
# robot_model (panda | fr3) picks the table; this is the default.
SERL_JOINT_RESET_ROBOT_MODEL = "panda"
# serl_auto_recover: after restarting SERL's controller, wait (bounded) for its first tick so the
# new target is not overwritten by starting() (which seeds the target with the measured pose).
SERL_RESTART_FIRST_TICK_TIMEOUT_S = 0.05
_sleep = time.sleep  # patched by the tests
# cartesian_controller: internal. A servo target becomes the twist that closes the pose error in
# INTERNAL_HORIZON_S, clipped to the speed limits, held for INTERNAL_HOLD_MS; with no newer target
# the motion then decelerates to rest on its own, so a stalled client stops the arm.
INTERNAL_HORIZON_S = 0.2
INTERNAL_MAX_SPEED_M_S = 0.25
INTERNAL_MAX_SPEED_RAD_S = 1.0
INTERNAL_HOLD_MS = 300
# "libfranka: Incompatible library version (server version: 10, library version: 9)."; other
# builds word it "Incompatible library version. Server version: 10\nLibrary version: 9".
_LIBFRANKA_MISMATCH = re.compile(
    r"(?:incompatible\s+library\s+version\W*)?server\s+version\W*(\d+)\W+library\s+version\W*(\d+)",
    re.IGNORECASE,
)


def franka_unreachable_hint(robot_ip: str) -> str:
    return (
        f"check: robot powered on and brakes released; Desk reachable at https://{robot_ip}/desk; "
        "you hold control in Desk; FCI activated (Desk → Activate FCI); cable on the Control port; "
        "firewall allows UDP"
    )


def server_version_cross_check(server_version: int) -> str:
    """Desk shows the system version (e.g. 5.9.2), not the robot server version; the matrix maps one to the other."""
    return (
        f"the robot server version {server_version} comes from libfranka's handshake error (authoritative); "
        f"to cross-check, read the system version in {DESK_SYSTEM_VERSION_PATH} and look it up in the "
        f"compatibility matrix {FRANKA_COMPATIBILITY_URL}"
    )


# Franka's compatibility matrix (FRANKA_COMPATIBILITY_URL, libfranka's compatibility_matrix): robot
# server versions 1-5 belong to robot system versions < 5 (Franka Emika Robot; the last FER system is
# 4.2.x, server 5), 6-10 to system 5.x (5.2.0 -> 6 ... 5.7.2 -> 9, 5.9.0 -> 10), which is FR3's.
FR3_MIN_SERVER_VERSION = 6


def robot_model_for_server_version(server_version: int) -> str:
    """``fr3`` or ``panda``, the robot model a robot server version implies."""
    return "fr3" if server_version >= FR3_MIN_SERVER_VERSION else "panda"


def robot_model_from_urdf(urdf: object) -> str | None:
    """``fr3`` or ``panda`` from the URDF the robot serves (franky 2.0 ``Robot.model_urdf``,
    libfranka >= 0.15), by its joint names (``fr3_joint1`` / ``panda_joint1``); None if neither or both."""
    if not isinstance(urdf, str) or not urdf:
        return None
    text = urdf.lower()
    fr3, panda = "fr3_joint" in text, "panda_joint" in text
    if fr3 == panda:
        return None
    return "fr3" if fr3 else "panda"


def parse_libfranka_mismatch(message: str) -> tuple[int, int] | None:
    """``(server_version, library_version)`` from a libfranka version-mismatch message, else None."""
    match = _LIBFRANKA_MISMATCH.search(message)
    if match is None:
        return None
    return int(match.group(1)), int(match.group(2))


def require_franky(device_cls: type, device: str) -> ModuleType:
    """Import franky; the PyPI distribution is ``franky-control``, not ``franky``."""
    try:
        return importlib.import_module("franky")
    except ImportError:
        raise SdkMissing(
            f"{device_cls.__name__} needs the 'franky' package",
            hint=f"pip install {FRANKY_DISTRIBUTION} (or, from the repository root: pip install -e '.[franky]')",
            device=device,
        ) from None


def franky_connect_error(franky: ModuleType, error: Exception, robot_ip: str, device: str) -> Exception:
    """Map a franky/libfranka connect failure to a typed device error."""
    if isinstance(error, franky.IncompatibleVersionException):
        return VersionMismatch(
            f"libfranka/robot firmware mismatch: {error}", hint=FRANKY_VERSION_HINT, device=device
        )
    if isinstance(error, franky.RealtimeException):
        return DeviceFault(
            f"libfranka realtime setup failed for {robot_ip}: {error}",
            hint="install a PREEMPT_RT kernel (and the realtime group limits) or set realtime: ignore",
            device=device,
        )
    return DeviceNotFound(
        f"cannot reach Franka at {robot_ip}: {error}", hint=franka_unreachable_hint(robot_ip), device=device
    )


def franky_version() -> str:
    """franky version; wheels built against a non-default libfranka carry a ``+libfranka.X`` label."""
    try:
        version = importlib.metadata.version(FRANKY_DISTRIBUTION)
    except importlib.metadata.PackageNotFoundError:
        return "unknown"
    _, _, local = version.partition("+")
    libfranka = local.split("libfranka.", 1)[1] if "libfranka." in local else "bundled default"
    return f"{version} (libfranka: {libfranka})"


def require_serl_controller(device: str) -> ModuleType:
    """Import serl_controller (built with franky); a missing or mismatched wheel says how to install the pair."""
    try:
        module = importlib.import_module("serl_controller")
    except ModuleNotFoundError as error:
        if error.name not in ("serl_controller", "serl_controller._serl_controller"):
            raise
        raise SdkMissing("cartesian_controller: serl needs the 'serl_controller' package",
                         hint=SERL_INSTALL_HINT, device=device) from None
    except ImportError as error:
        # e.g. "libfranka-<hash>.so.0.17.0: cannot open shared object file": the installed franky
        # wheel is not the one serl_controller was built against.
        raise VersionMismatch(f"serl_controller does not load next to the installed franky: {error}",
                              hint=SERL_INSTALL_HINT, device=device) from error
    if not getattr(module, "WITH_FRANKY", False) or not hasattr(module, "SerlJointPositionMotion"):
        raise VersionMismatch("the installed serl_controller was built without franky (law only) or "
                              "predates the joint reset", hint=SERL_INSTALL_HINT, device=device)
    return module


def _package_dir(name: str) -> Path | None:
    """The directory of package ``name``, found without importing it."""
    try:
        spec = importlib.util.find_spec(name)
    except (ImportError, ValueError):
        return None
    if spec is None or not spec.submodule_search_locations:
        return None
    return Path(next(iter(spec.submodule_search_locations)))


def serl_build_pairing() -> tuple[bool, str, str, dict]:
    """Whether serl_controller and franky come from the same build, without importing either
    extension: ``(ok, message, hint, details)``. serl_controller's ``build_info.json`` records the
    franky_control wheel version, the sha256 of the libfranky.so it linked against and the bundled
    libfranka file; all three must match the installed franky wheel."""
    serl_dir = _package_dir("serl_controller")
    if serl_dir is None:
        return False, "serl_controller is not installed", SERL_INSTALL_HINT, {}
    info_path = serl_dir / "build_info.json"
    try:
        info = json.loads(info_path.read_text())
    except (OSError, ValueError) as error:
        return (False, f"serl_controller has no readable build_info.json ({error}); it is not from a "
                "build this check can verify", SERL_INSTALL_HINT, {})
    details = {"serl_controller": info}
    if not info.get("with_franky"):
        return False, "serl_controller was built without franky (law-only build)", SERL_INSTALL_HINT, details
    try:
        franky_version_installed = importlib.metadata.version(FRANKY_DISTRIBUTION)
    except importlib.metadata.PackageNotFoundError:
        return False, "franky (franky-control) is not installed", SERL_INSTALL_HINT, details
    details["franky_control_version"] = franky_version_installed
    built = info.get("franky_dist_version")
    if franky_version_installed != built:
        return (False, f"serl_controller {info.get('serl_controller_version')} was built against "
                f"franky_control {built}, installed is {franky_version_installed}", SERL_INSTALL_HINT, details)
    franky_dir = _package_dir("franky")
    libfranky = None if franky_dir is None else franky_dir / "libfranky.so"
    try:
        digest = hashlib.sha256(libfranky.read_bytes()).hexdigest() if libfranky is not None else None
    except OSError:
        digest = None
    details["libfranky_sha256"] = digest
    if digest != info.get("libfranky_sha256"):
        return (False, f"the installed franky_control {franky_version_installed} is not the wheel serl_controller "
                f"was built against (libfranky.so sha256 {str(digest)[:12]} != {str(info.get('libfranky_sha256'))[:12]}; "
                "e.g. the official wheel instead of the one from the same build)", SERL_INSTALL_HINT, details)
    libfranka = info.get("libfranka_file") or ""
    if libfranka:
        # Wheel layout: libfranka is bundled in the franky wheel (auditwheel), next to franky/.
        libs_dir = None if franky_dir is None else franky_dir.parent / "franky_control.libs"
        if libs_dir is None or not (libs_dir / libfranka).is_file():
            return (False, f"the bundled libfranka '{libfranka}' serl_controller links against is missing from "
                    f"{libs_dir}", SERL_INSTALL_HINT, details)
        linked = libfranka
    else:
        # Source layout (native/serl_controller/scripts/install-on-ubuntu.sh): franky and
        # serl_controller both link the system libfranka recorded at build time.
        path = info.get("libfranka_path") or ""
        if not path:
            return (False, "serl_controller's build_info.json records neither a bundled nor a system libfranka",
                    SERL_INSTALL_HINT, details)
        try:
            system_digest = hashlib.sha256(Path(path).read_bytes()).hexdigest()
        except OSError:
            return (False, f"the libfranka serl_controller was built against ({path}) is gone; "
                    "was libfranka removed or replaced by another version?", SERL_INSTALL_HINT, details)
        details["libfranka_sha256"] = system_digest
        if info.get("libfranka_sha256") and system_digest != info.get("libfranka_sha256"):
            return (False, f"{path} changed since serl_controller was built (sha256 {system_digest[:12]} != "
                    f"{str(info.get('libfranka_sha256'))[:12]}; libfranka was rebuilt or reinstalled)",
                    SERL_INSTALL_HINT, details)
        linked = f"source build, {path}"
    return (True, f"serl_controller {info.get('serl_controller_version')} and franky_control "
            f"{franky_version_installed} from the same build ({info.get('build_id') or 'unnamed build'}; "
            f"{linked})", "", details)


@dataclass
class _FrankyHandle:
    robot: Any
    tracker: Any = None
    streaming: bool = False  # cartesian_controller: internal, a velocity motion was sent
    serl_motion: Any = None  # cartesian_controller: serl, the running SerlCartesianImpedanceMotion
    joint_reset: Any = None  # cartesian_controller: serl, a running SerlJointPositionMotion


@Arm.register("franky")
class FrankyArm(Arm):
    SDK = "franky"
    CONTROL_CAPABILITIES = ("translational_stiffness", "rotational_stiffness", "cartesian_damping", "joint_stiffness", "joint_damping", "translational_error_clip", "rotational_error_clip", "max_delta_tau", "gains_time_constant")
    SUPPORTED_STATE_FIELDS = ("tcp_pose", "tcp_vel", "joint_pos", "joint_vel", "tcp_force", "tcp_torque")

    def __init__(
        self,
        name: str = "arm",
        *,
        robot_ip: str,
        relative_dynamics_factor: float = 0.2,
        realtime: str = "ignore",
        cartesian_controller: str = "impedance",
        home_joints: list[float] | None = None,
        safety: ArmSafety | None = None,
        serl: dict | None = None,
        load: dict | None = None,
        robot_model: str = SERL_JOINT_RESET_ROBOT_MODEL,
        serl_auto_recover: dict | None = None,
    ) -> None:
        super().__init__(name, num_joints=7, safety=safety)
        if realtime not in ("ignore", "enforce"):
            raise ValueError(f"realtime '{realtime}' invalid; choose from ['ignore', 'enforce']")
        if cartesian_controller not in CARTESIAN_CONTROLLERS:
            raise ValueError(f"cartesian_controller '{cartesian_controller}' invalid; "
                             f"choose from {list(CARTESIAN_CONTROLLERS)}")
        self.cartesian_controller = cartesian_controller
        # SERL compliance parameters: the station ``serl:`` block resolved over the .cfg defaults,
        # then every update_serl_params() merged in (they persist like dynamic_reconfigure values).
        station_serl, self._serl_station_clamped = serl_param_updates(serl, where="serl")
        self._serl_station = resolve_serl_params(station_serl)
        self._serl_params = dict(self._serl_station)
        self._serl_clamped = dict(self._serl_station_clamped)
        self.load = franka_load(load)
        from real_world_rl.devices.robots.franka import ROBOT_MODELS
        if robot_model not in ROBOT_MODELS:
            raise ValueError(f"robot_model '{robot_model}' invalid; choose from {list(ROBOT_MODELS)}")
        self.robot_model = robot_model
        self._last_joint_reset: dict | None = None
        self._serl_auto_recover = serl_auto_recover_config(serl_auto_recover)
        self._serl_recoveries_episode = 0
        self._serl_recoveries_total = 0
        self._last_serl_recovery: dict | None = None
        self.robot_ip = robot_ip
        self.relative_dynamics_factor = relative_dynamics_factor
        self.realtime = realtime
        self._stop_lock = threading.RLock()
        self._stop_recorder = None
        self._last_stop_trace = None
        self._trace_in_progress = False
        self._trace_preparing = False
        self._stop_generation = 0
        self._stop_metadata = None
        self.home_joints = None if home_joints is None else list(home_joints)

    @classmethod
    def from_config(cls, config: "RobotConfig", *, name: str = "arm") -> "FrankyArm":
        return cls(
            name,
            robot_ip=config.robot_ip,
            relative_dynamics_factor=config.relative_dynamics_factor,
            realtime=config.realtime,
            cartesian_controller=getattr(config, "cartesian_controller", "impedance"),
            home_joints=config.home_joints,
            safety=ArmSafety.from_config(config),
            serl=getattr(config, "serl", None),
            load=getattr(config, "load", None),
            robot_model=getattr(config, "robot_model", SERL_JOINT_RESET_ROBOT_MODEL),
            serl_auto_recover=getattr(config, "serl_auto_recover", None),
        )

    @classmethod
    def require_sdk(cls) -> ModuleType:
        return require_franky(cls, cls.__name__)

    def require_serl(self) -> ModuleType:
        return require_serl_controller(self.name)

    # --- lifecycle ---

    def _open(self) -> _FrankyHandle:
        franky = self.require_sdk()
        realtime_config = (
            franky.RealtimeConfig.Enforce if self.realtime == "enforce" else franky.RealtimeConfig.Ignore
        )
        try:
            robot = franky.Robot(
                self.robot_ip,
                relative_dynamics_factor=self.relative_dynamics_factor,
                realtime_config=realtime_config,
            )
        except Exception as error:
            raise franky_connect_error(franky, error, self.robot_ip, self.name) from error
        self._stop_metadata = None  # cache lifetime is this connection
        handle = _FrankyHandle(robot=robot)
        if self.load is not None:
            self._apply_load(franky, robot)
        self._read_state_from(robot)
        return handle

    def _apply_load(self, franky: ModuleType, robot: Any) -> None:
        """libfranka Robot::setLoad with the station ``load:``, before any motion runs (HIL-SERL's
        franka_server /set_load calls franka_control's SetLoad service the same way)."""
        load = self.load
        try:
            robot.set_load(load["mass_kg"], list(load["center_of_mass_m"]), list(load["inertia"]))
        except Exception as error:
            raise DeviceFault(f"franka set_load({load}) failed: {error}",
                              hint="check the load: values; libfranka refuses setLoad while a motion runs",
                              device=self.name) from error

    def _release(self, handle: _FrankyHandle) -> None:
        try:
            self._stop_motion(handle)
        finally:
            if self._stop_recorder is not None:
                self._stop_recorder.finish()
            handle.robot = None

    # --- state ---

    def read_state(self) -> dict[str, np.ndarray]:
        self._require_connected()
        return self._remember_state(self._read_state_from(self._handle.robot))

    def _read_state_from(self, robot: Any) -> dict[str, np.ndarray]:
        franky = self.require_sdk()
        state = robot.state
        ee_pose = state.O_T_EE
        dq = np.asarray(state.dq, dtype=np.float64)
        # O_dP_EE_c is the commanded twist (zero under torque control), so measure it: J(q) dq.
        jacobian = np.asarray(robot.model.zero_jacobian(franky.Frame.EndEffector, state), dtype=np.float64)
        wrench = np.asarray(state.O_F_ext_hat_K, dtype=np.float64)
        self._robot_mode = getattr(state.robot_mode, "name", str(state.robot_mode))
        self._note_feedback()
        return {
            "tcp_pose": np.concatenate(
                [np.asarray(ee_pose.translation, dtype=np.float64), np.asarray(ee_pose.quaternion, dtype=np.float64)]
            ),
            "tcp_vel": jacobian @ dq,
            "joint_pos": np.asarray(state.q, dtype=np.float64),
            "joint_vel": dq,
            "tcp_force": wrench[:3],
            "tcp_torque": wrench[3:],
        }

    def stop_sample(self) -> dict:
        """One raw SDK snapshot for a same-connection stop sampler.

        Uses SDK state directly, never the arm cache/Jacobian or a second FCI
        connection. Callers must keep the connection alive until sampling ends.
        """
        self._require_connected()
        state = self._handle.robot.state
        pose = state.O_T_EE
        duration = getattr(state, "time", None)
        to_seconds = getattr(duration, "to_sec", None)
        errors = getattr(state, "current_errors", None)
        # SDK Errors has named boolean fields, but no Python __bool__: bool(errors)
        # is True even for an error-free object. Do not invent a fault from it.
        if isinstance(errors, (list, tuple)):
            active_errors = [str(value) for value in errors]
        elif errors is not None:
            flags = {name: getattr(errors, name) for name in dir(errors) if not name.startswith("_")}
            flags = {name: value for name, value in flags.items() if isinstance(value, (bool, np.bool_))}
            active_errors = [name for name, value in flags.items() if value] if flags else None
        else:
            active_errors = None
        return {
            "device_time_s": float(to_seconds()) if callable(to_seconds) else None,
            "tcp_pose": [*np.asarray(pose.translation, dtype=float).tolist(),
                         *np.asarray(pose.quaternion, dtype=float).tolist()],
            "joint_pos": np.asarray(state.q, dtype=float).tolist(),
            "joint_vel": np.asarray(state.dq, dtype=float).tolist(),
            "robot_mode": getattr(state.robot_mode, "name", str(state.robot_mode)),
            "errors": active_errors,
            "sdk_errors_repr": str(errors),
            "last_cartesian_target": None if self.last_cartesian_target is None else self.last_cartesian_target.tolist(),
        }

    # --- commands ---

    def _servo_cartesian(self, pose: np.ndarray) -> None:
        if self.cartesian_controller == "internal":
            self._servo_internal(self._checked(pose, (7,)))
            return
        if self.cartesian_controller == "serl":
            pose = self._checked(pose, (7,))
            motion = self._live_serl_motion()
            # SERL's equilibrium pose: base frame position + quaternion (x, y, z, w), like
            # franka_server.move() publishing a PoseStamped to equilibrium_pose.
            with self._control_exceptions_as_faults("SERL set_target"):
                motion.set_target(pose[:3].tolist(), pose[3:].tolist())
            return
        tracker = self._live_tracker(ControlMode.CARTESIAN_SERVO)
        tracker.set_target(self._affine(pose))

    def _servo_internal(self, pose: np.ndarray) -> None:
        """Hand ``pose`` to the robot's own controller as a Cartesian velocity motion.

        The twist (base frame) closes the error between ``pose`` and the measured TCP pose in
        ``INTERNAL_HORIZON_S``, clipped to ``INTERNAL_MAX_SPEED_*``. Each call preempts the
        running velocity motion, so Ruckig re-plans from the current velocity to the new one
        within ``relative_dynamics_factor`` of the robot's limits; the robot solves the joint
        motion. The motion holds the twist ``INTERNAL_HOLD_MS`` and then comes to rest."""
        franky = self.require_sdk()
        self._require_servo_mode(ControlMode.CARTESIAN_SERVO)
        handle = self._handle
        robot = handle.robot
        if handle.streaming and not robot.is_in_control:
            # The last motion ended: at rest after its hold time, or killed by a reflex.
            handle.streaming = False
            try:
                robot.join_motion()
            except Exception as error:
                if not _is_preemption(error):
                    message = f"franka Cartesian velocity motion stopped: {error}"
                    self._note_error(message)
                    raise DeviceFault(message, hint="call recover(); the next servo command restarts it",
                                      device=self.name, recoverable=True) from error
        if self._errors or getattr(robot, "has_errors", False):
            details = self._errors or [str(robot.state.current_errors)]
            raise DeviceFault(f"arm '{self.name}' has unrecovered errors: {details}",
                              hint="call recover() before servoing", device=self.name, recoverable=True)
        measured = robot.state.O_T_EE
        linear, angular = internal_servo_twist(
            pose, np.asarray(measured.translation, dtype=np.float64),
            np.asarray(measured.quaternion, dtype=np.float64))
        motion = franky.CartesianVelocityMotion(
            franky.RobotVelocity(franky.Twist(linear, angular)),
            duration=franky.Duration(INTERNAL_HOLD_MS),
            relative_dynamics_factor=self.relative_dynamics_factor,
        )
        with self._control_exceptions_as_faults("Cartesian velocity motion"):
            robot.move(motion, asynchronous=True)
        handle.streaming = True

    def _servo_joints(self, q: np.ndarray) -> None:
        tracker = self._live_tracker(ControlMode.JOINT_SERVO)
        tracker.set_target(self._checked(q, (self.num_joints,)))

    def _move_to_pose(self, pose: np.ndarray, blocking: bool = True) -> None:
        self._require_idle("move_to_pose")
        franky = self.require_sdk()
        motion = franky.CartesianMotion(self._affine(pose))
        with self._control_exceptions_as_faults("move_to_pose"):
            self._handle.robot.move(motion, asynchronous=not blocking)

    def _move_to_joints(self, q: np.ndarray, blocking: bool = True) -> None:
        self._require_idle("move_to_joints")
        franky = self.require_sdk()
        target = self._checked(q, (self.num_joints,))
        if self.cartesian_controller == "serl":
            self._serl_joint_reset(target, blocking=blocking)
            return
        with self._control_exceptions_as_faults("move_to_joints"):
            self._handle.robot.move(franky.JointMotion(target), asynchronous=not blocking)

    # --- cartesian_controller: serl ---

    @property
    def serl_params(self) -> dict | None:
        return dict(self._serl_params) if self.cartesian_controller == "serl" else None

    def update_serl_params(self, updates) -> dict:
        if self.cartesian_controller != "serl":
            return super().update_serl_params(updates)
        values, clamped = serl_param_updates(updates, where="update_serl_params")
        handle = self._handle if self.is_connected else None
        motion = None if handle is None else handle.serl_motion
        merged = {**self._serl_params, **values}
        if motion is not None and values:
            # Applied by the RT thread at the next tick (between two SERL update() calls).
            effective = motion.update_params(values)
            merged = {name: float(effective[name]) for name in SERL_PARAM_NAMES}
        self._serl_params = merged
        self._serl_clamped.update(clamped)
        return dict(merged)

    def _new_serl_motion(self, robot: Any) -> Any:
        """Start SERL's Cartesian impedance controller (a fresh controller per start, like SERL's
        franka_server relaunching impedance.launch) with franka_control's torque filters."""
        sc = self.require_serl()
        motion = sc.SerlCartesianImpedanceMotion(dict(self._serl_params))
        with self._control_exceptions_as_faults("starting the SERL impedance controller"):
            robot.move(motion, asynchronous=True, limit_rate=sc.FRANKA_ROS_LIMIT_RATE,
                       cutoff_frequency=sc.FRANKA_ROS_CUTOFF_FREQUENCY)
        return motion

    def _live_serl_motion(self) -> Any:
        """The running SERL controller; restarted after a plain stop(). A reflex (any motion error)
        that ended it is cleared and the controller restarted here when serl_auto_recover allows
        it (HIL-SERL's /clearerr before every /pose); otherwise, or past max_per_episode, it is a
        recoverable fault (refused until recover())."""
        self._require_servo_mode(ControlMode.CARTESIAN_SERVO)
        handle = self._handle
        if handle.serl_motion is not None and not handle.robot.is_in_control:
            handle.serl_motion = None
            try:
                handle.robot.join_motion()
                error = None
            except Exception as exc:
                error = exc
            text = "control ended without an error" if error is None else str(error)
            if error is not None and _is_preemption(error):
                error = None  # ended by a stop, not a reflex: not auto-recovered
            message = f"franka SERL impedance controller stopped: {text}"
            policy = self._serl_auto_recover
            if error is not None and policy["enabled"]:
                if self._serl_recoveries_episode < policy["max_per_episode"]:
                    return self._serl_auto_recover_motion(handle, text)
                message += (f" (serl_auto_recover: max_per_episode {policy['max_per_episode']} reached, "
                            f"{self._serl_recoveries_episode} auto-recoveries this episode)")
            self._note_error(message)
            raise DeviceFault(message, hint="call recover(); the next servo command restarts the controller",
                              device=self.name, recoverable=True)
        if handle.serl_motion is None:
            if self._errors:
                raise DeviceFault(f"arm '{self.name}' has unrecovered errors: {self._errors}",
                                  hint="call recover() before servoing", device=self.name, recoverable=True)
            self._enter_control_mode(ControlMode.CARTESIAN_SERVO)
        return handle.serl_motion

    def _serl_auto_recover_motion(self, handle: _FrankyHandle, error_text: str) -> Any:
        """Clear the errors (libfranka automatic error recovery, what franka_control's
        error_recovery action runs) and start a fresh SERL controller at the measured pose; the
        caller then applies the new target. Waits briefly for the controller's first tick, whose
        starting() would otherwise overwrite a target sent before it."""
        robot = handle.robot
        number = self._serl_recoveries_episode + 1
        cap = self._serl_auto_recover["max_per_episode"]
        logger.warning("arm '%s': SERL impedance controller stopped (%s); auto-recovery %d/%d this episode: "
                       "clearing errors and restarting the controller at the measured pose",
                       self.name, error_text, number, cap)
        try:
            recovered = robot.recover_from_errors()
        except Exception as recover_error:  # e.g. a franky CommandException while UserStopped
            recovered, error_text = False, f"{error_text}; recover_from_errors failed: {recover_error}"
        if not recovered:
            message = f"franka SERL impedance controller stopped: {error_text} (serl_auto_recover could not clear it)"
            self._note_error(message)
            raise DeviceFault(message, hint="release the external activation device / clear errors in Desk, "
                              "then call recover()", device=self.name, recoverable=True)
        self._clear_errors()
        self._serl_recoveries_episode = number
        self._serl_recoveries_total += 1
        self._last_serl_recovery = {"error": error_text, "episode_count": number, "time_s": time.time(),
                                    "tcp_pose": self._read_state_from(robot)["tcp_pose"].tolist()}
        motion = self._new_serl_motion(robot)
        handle.serl_motion = motion
        snapshot = getattr(motion, "snapshot", None)
        if callable(snapshot):
            deadline = time.monotonic() + SERL_RESTART_FIRST_TICK_TIMEOUT_S
            while int(dict(snapshot()).get("tick", 0)) < 1 and time.monotonic() < deadline:
                _sleep(0.001)
        return motion

    @property
    def serl_auto_recovery(self) -> dict | None:
        if self.cartesian_controller != "serl":
            return None
        return {**self._serl_auto_recover, "episode_count": self._serl_recoveries_episode,
                "total_count": self._serl_recoveries_total, "last": copy.deepcopy(self._last_serl_recovery)}

    def reset_serl_auto_recovery(self) -> None:
        self._serl_recoveries_episode = 0

    def set_control_mode(self, mode: ControlMode) -> None:
        if mode is ControlMode.CARTESIAN_SERVO and mode is not self._control_mode:
            self._serl_recoveries_episode = 0  # entering Cartesian servo starts a serl_auto_recover episode
        super().set_control_mode(mode)

    def _serl_joint_reset(self, target: np.ndarray, *, blocking: bool) -> None:
        """SERL's joint reset: JointPositionController (10 s linear blend to ``target``, then hold)
        under franka_control's position settings (internal joint impedance controller = franky's
        default controller mode, rate limiting on, 100 Hz cutoff), finished like HIL-SERL's
        franka_server.reset_joint. ``blocking=False`` starts it and returns; the hold then lasts
        until the next stop() or mode change."""
        sc = self.require_serl()
        handle = self._handle
        robot = handle.robot
        # franka_server: stop the impedance controller, clear errors, then launch joint.launch.
        self._stop_motion(handle)
        self._settle_and_recover(robot)
        # franka_hw's PositionJointSoftLimitsInterface runs inside the motion, with the URDF limits
        # franka_control loads for this robot model.
        motion = sc.SerlJointPositionMotion(target.tolist(), robot_model=self.robot_model)
        handle.joint_reset = motion
        started = time.monotonic()
        with self._control_exceptions_as_faults("SERL joint reset"):
            robot.move(motion, asynchronous=True, limit_rate=sc.FRANKA_ROS_LIMIT_RATE,
                       cutoff_frequency=sc.FRANKA_ROS_CUTOFF_FREQUENCY)
        if not blocking:
            return
        reached, polls = False, 0
        self._joint_reset_wait(handle, motion, SERL_JOINT_RESET_FIRST_CHECK_S)
        while True:
            q = np.asarray(robot.state.q, dtype=np.float64)
            if np.allclose(target - q, 0, atol=SERL_JOINT_RESET_ATOL, rtol=SERL_JOINT_RESET_RTOL):
                reached = True
                break
            self._joint_reset_wait(handle, motion, SERL_JOINT_RESET_POLL_S)
            polls += 1
            if polls > SERL_JOINT_RESET_MAX_POLLS:
                break
        # End in the hold phase: wait for the blend to be over, then MotionFinished.
        blend_s = float(getattr(sc, "JOINT_BLEND_DURATION_S", 10.0))
        deadline = time.monotonic() + blend_s + SERL_JOINT_RESET_FINISH_SLACK_S
        while float(motion.snapshot().get("elapsed_s", 0.0)) <= blend_s and time.monotonic() < deadline:
            self._joint_reset_wait(handle, motion, 0.05)
        motion.finish()
        try:
            with self._control_exceptions_as_faults("SERL joint reset"):
                completed = robot.join_motion(timeout=SERL_JOINT_RESET_FINISH_SLACK_S)
            if completed is False:
                robot.stop()
                with self._control_exceptions_as_faults("SERL joint reset"):
                    try:
                        robot.join_motion()
                    except Exception as error:
                        if not _is_preemption(error):
                            raise
        finally:
            handle.joint_reset = None
        snapshot = dict(motion.snapshot())
        robot.recover_from_errors()  # franka_server clear() after stopping the joint controller
        final = np.asarray(robot.state.q, dtype=np.float64)
        self._last_joint_reset = {"target": target.tolist(), "reached": reached, "polls": polls,
                                  "max_abs_error_rad": float(np.max(np.abs(target - final))),
                                  "duration_s": time.monotonic() - started,
                                  "soft_limited_ticks": int(snapshot.get("soft_limited_ticks", 0)),
                                  "robot_model": self.robot_model}
        if not reached:
            logger.warning("SERL joint reset TIMEOUT on arm '%s' after %d polls: max joint error %.4f rad",
                           self.name, polls, self._last_joint_reset["max_abs_error_rad"])

    @property
    def last_joint_reset(self) -> dict | None:
        return copy.deepcopy(self._last_joint_reset)

    def _joint_reset_wait(self, handle: _FrankyHandle, motion: Any, seconds: float) -> None:
        """Sleep while the joint reset holds control; a stop() or a reflex ends it with a fault."""
        _sleep(seconds)
        if handle.joint_reset is not motion:
            raise DeviceFault("SERL joint reset was stopped", device=self.name, recoverable=True)
        if not handle.robot.is_in_control:
            handle.joint_reset = None
            message = f"franka SERL joint reset stopped: {self._motion_error(handle.robot)}"
            self._note_error(message)
            raise DeviceFault(message, hint="call recover(); check the workspace / Desk",
                              device=self.name, recoverable=True)

    @property
    def safety_capabilities(self):
        return (*super().safety_capabilities, "stop_trace_v1")

    def _new_stop_recorder(self):
        """Collect slow static evidence without holding the motion/stop lock."""
        from real_world_rl.devices.stop_sampling import StopTraceRecorder
        from real_world_rl.devices.provenance import sdk_artifacts, portable_path
        if self._stop_metadata is None:
            sdk = self.require_sdk()
            robot = self._handle.robot
            metadata = {"sdk_version": franky_version(), "sdk_module": portable_path(getattr(sdk, "__file__", None)),
                        "sdk_artifacts": sdk_artifacts(sdk, implementation_type=type(robot)),
                        "server_version": getattr(robot, "server_version", None)}
            state = robot.state
            model = {}
            for name in ("m_ee", "m_load", "m_total", "F_x_Cee", "F_x_Cload", "F_x_Ctotal",
                         "I_ee", "I_load", "I_total", "F_T_EE", "EE_T_K"):
                value = getattr(state, name, None)
                model[name] = None if value is None else np.asarray(value).tolist()
            metadata.update(reported_model=model, model_verified=False)
            self._stop_metadata = metadata
        limits = self.safety_limits or {}
        return StopTraceRecorder(
            self.stop_sample, pre_window_s=.75, workspace=limits.get("workspace"),
            task_frame=limits.get("task_frame"),
            metadata={**self._stop_metadata, "control": self.control_metadata}).start()

    def begin_stop_trace(self) -> None:
        """Enable pre-stop sampling; slow preparation never holds the stop lock."""
        self._require_connected()
        with self._stop_lock:
            if self._trace_in_progress or self._trace_preparing:
                raise DeviceFault("normal stop trace is already in progress", device=self.name)
            self._trace_preparing = True
            generation = self._stop_generation
            previous = self._stop_recorder
        recorder = None
        try:
            if previous is not None:
                previous.finish()  # bounded join outside _stop_lock
                if previous.is_running:
                    raise DeviceFault("previous stop sample read is still blocked", device=self.name)
            recorder = self._new_stop_recorder()
            with self._stop_lock:
                if generation != self._stop_generation or self._trace_in_progress:
                    raise DeviceFault("stop occurred during trace preparation", device=self.name)
                self._stop_recorder = recorder
                recorder = None  # ownership transferred
        finally:
            if recorder is not None:
                recorder.finish()
            with self._stop_lock:
                self._trace_preparing = False

    @property
    def last_stop_trace(self):
        return copy.deepcopy(self._last_stop_trace)

    def stop_with_trace(self, *, post_stop_s=2.1) -> dict:
        """Stop first, then collect evidence. Only this path raises stop failures."""
        if not math.isfinite(post_stop_s) or not 0 <= post_stop_s <= 30:
            raise ValueError("post_stop_s must be in [0, 30]")
        self._require_connected()
        from real_world_rl.devices.stop_trace import evaluate_stop_trace
        sampling_errors = []
        with self._stop_lock:
            if self._trace_in_progress:
                raise DeviceFault("normal stop trace is already in progress", device=self.name)
            self._trace_in_progress = True
            self._stop_generation += 1
            recorder = self._stop_recorder
            control_mode = self._control_mode.value
            try:
                tracker_active = bool((self._handle.tracker is not None or self._handle.serl_motion is not None)
                                      and self._handle.robot.is_in_control)
            except Exception as error:
                tracker_active = None
                sampling_errors.append(f"active control unavailable: {type(error).__name__}: {error}")
            start = time.monotonic()
            failure = None
            if recorder is not None:
                try:
                    recorder.mark_stop_start(control_mode=control_mode, tracker_active=tracker_active,
                                             monotonic_s=start)
                except Exception as error:
                    sampling_errors.append(f"stop-start evidence failed: {type(error).__name__}: {error}")
            # Like stop(): the control segment ends here, even if the stop itself fails.
            self._last_cartesian_target = None
            try:
                self._stop_motion(self._handle, raise_on_error=True)
                self._control_mode = ControlMode.IDLE
            except BaseException as error:
                failure = error
            stop_failure = failure
            end = time.monotonic()
            if recorder is not None:
                try:
                    recorder.mark_stop_end(failure, monotonic_s=end)
                except Exception as error:
                    sampling_errors.append(f"stop-end evidence failed: {type(error).__name__}: {error}")
        try:
            if recorder is None:
                try:
                    if self._trace_preparing:
                        raise RuntimeError("pre-stop recorder preparation still in progress")
                    recorder = self._new_stop_recorder()  # stop has already been sent
                    recorder.mark_stop_start(control_mode=control_mode, tracker_active=tracker_active,
                                             monotonic_s=start)
                    recorder.mark_stop_end(stop_failure, monotonic_s=end)
                except Exception as error:
                    sampling_errors.append(f"trace preparation failed: {type(error).__name__}: {error}")
            if failure is None and recorder is not None:
                try:
                    time.sleep(post_stop_s)
                except BaseException as error:
                    failure = error
            trace = {"schema_version": 1, "samples": [], "sampling_errors": []}
            if recorder is not None:
                try:
                    trace = recorder.finish()  # join outside _stop_lock
                except Exception as error:
                    sampling_errors.append(f"trace finish failed: {type(error).__name__}: {error}")
            # Always retain the actual call's boundaries, even when recorder marking failed.
            trace.update(stop_start_s=start, stop_end_s=end,
                         control_mode=control_mode, tracker_active=tracker_active)
            trace.setdefault("sampling_errors", []).extend(sampling_errors)
            if failure is not None and not trace.get("stop_error"):
                # A stop failure is FAIL evidence; observation interruptions are gaps.
                if stop_failure is not None:
                    trace["stop_error"] = f"{type(failure).__name__}: {failure}"
                else:
                    trace["sampling_errors"].append(f"post-stop observation interrupted: {failure}")
            try:
                assessment = evaluate_stop_trace(trace)
            except (ValueError, TypeError, KeyError) as error:
                assessment = {"status": "FAIL" if trace.get("stop_error") else "BLOCKED",
                              "evaluation_error": str(error)}
            self._last_stop_trace = {"trace": trace, "assessment": assessment}
            with self._stop_lock:
                self._stop_recorder = recorder if recorder is not None and recorder.is_running else None
            if failure is not None:
                raise failure
            return self.last_stop_trace
        finally:
            with self._stop_lock:
                self._trace_in_progress = False

    def _stop(self) -> None:
        """Bring any running tracker or position motion to rest. Power and brakes are untouched."""
        self._require_connected()
        self._stop_motion(self._handle)

    def recover(self) -> None:
        self._require_connected()
        if not self._handle.robot.recover_from_errors():
            raise DeviceFault(
                f"arm '{self.name}' did not recover from its errors: {self._errors}",
                hint="release the external activation device / clear errors in Desk, then retry",
                device=self.name,
                recoverable=True,
            )
        self._clear_errors()

    def _enter_control_mode(self, mode: ControlMode) -> None:
        if self._trace_in_progress and mode is not ControlMode.IDLE:
            raise DeviceFault("normal stop observation in progress", device=self.name)
        handle = self._handle
        self._stop_motion(handle)
        self._settle_and_recover(handle.robot)
        handle.streaming = False
        if mode is ControlMode.CARTESIAN_SERVO and self.cartesian_controller == "internal":
            return  # the first servo target starts the velocity motion
        if mode is ControlMode.CARTESIAN_SERVO and self.cartesian_controller == "serl":
            handle.serl_motion = self._new_serl_motion(handle.robot)
            return
        if mode is not ControlMode.IDLE:
            handle.tracker = self._new_tracker(handle.robot, mode)

    # --- health / doctor ---

    def health(self):
        if self.is_connected and getattr(self._handle.robot, "has_errors", False):
            message = f"franka errors: {self._handle.robot.state.current_errors}"
            if message not in self._errors:
                self._note_error(message)
        return super().health()

    def preflight(self) -> list["CheckResult"]:
        from real_world_rl.doctor.checks import CheckResult

        results = [
            self._realtime_kernel_check(CheckResult),
            self._robot_mode_check(CheckResult),
            CheckResult(5, "mode:libfranka_version", True, f"franky {franky_version()}"),
        ]
        if self.cartesian_controller == "serl":
            results.append(self._serl_realtime_check(CheckResult))
        elif self.cartesian_controller == "internal":
            results.append(self._internal_realtime_check(CheckResult))
        results.append(self._robot_model_check(CheckResult))
        return results

    def _robot_model_check(self, check_result: type) -> "CheckResult":
        """robot_model against the robot server version: per Franka's compatibility matrix,
        server versions <= 5 are robot system 4.x (FER / Panda) and >= 6 system 5.x (FR3)."""
        # franky 2.0 binds no Robot.server_version (only Gripper.server_version, the hand's own
        # protocol); it is read when a franky build has it, else the model is taken from the
        # robot-served URDF (Robot.model_urdf, libfranka >= 0.15).
        robot = self._handle.robot if self.is_connected else None
        server = getattr(robot, "server_version", None)
        try:
            server = None if server is None or isinstance(server, bool) else int(server)
        except (TypeError, ValueError):
            server = None
        urdf_model = None
        if server is None and robot is not None:
            try:
                urdf_model = robot_model_from_urdf(getattr(robot, "model_urdf", None))
            except Exception:  # noqa: BLE001 - a property that raises means "unknown", like a missing one
                urdf_model = None
        details = {"robot_model": self.robot_model, "server_version": server, "urdf_robot_model": urdf_model}
        confirm = (f"confirm the robot model in Desk ({DESK_SYSTEM_VERSION_PATH}: system 4.x = Franka Emika "
                   f"Robot / panda, 5.x = Franka Research 3 / fr3; {FRANKA_COMPATIBILITY_URL}) and set "
                   "robot_model: panda | fr3 in the station")
        if urdf_model is not None:
            if urdf_model != self.robot_model:
                return check_result(
                    5, "mode:robot_model", True,
                    f"robot_model: {self.robot_model} but the robot's URDF (Robot.model_urdf) is a {urdf_model}"
                    f"; SERL's joint reset would use the {self.robot_model} joint limits",
                    hint=confirm, warning=True, details=details)
            return check_result(5, "mode:robot_model", True,
                                f"robot_model: {self.robot_model} (robot URDF: {urdf_model})", details=details)
        if server is not None:
            implied = robot_model_for_server_version(server)
            if implied != self.robot_model:
                return check_result(
                    5, "mode:robot_model", True,
                    f"robot_model: {self.robot_model} but robot server version {server} is a "
                    f"{'Franka Research 3 (system 5.x)' if implied == 'fr3' else 'Franka Emika Robot (system 4.x)'}"
                    f"; SERL's joint reset would use the {self.robot_model} joint limits",
                    hint=confirm, warning=True, details=details)
            return check_result(5, "mode:robot_model", True,
                                f"robot_model: {self.robot_model} (robot server version {server})", details=details)
        if self.cartesian_controller == "serl":
            return check_result(
                5, "mode:robot_model", True,
                f"robot_model: {self.robot_model} could not be cross-checked (robot server version and URDF "
                "unknown); "
                "SERL's joint reset uses its joint limits", hint=confirm, warning=True, details=details)
        return check_result(5, "mode:robot_model", True,
                            f"robot_model: {self.robot_model} (robot server version and URDF unknown)",
                            details=details)

    def _serl_realtime_check(self, check_result: type) -> "CheckResult":
        """SERL ran under franka_control with realtime_config: enforce (franka_control_node.yaml)."""
        if self.realtime == "enforce":
            return check_result(5, "mode:serl_realtime", True, "cartesian_controller: serl with realtime: enforce")
        return check_result(
            5, "mode:serl_realtime", True,
            f"cartesian_controller: serl with realtime: {self.realtime}: SERL's 1 kHz torque loop ran with "
            "realtime enforced (franka_control); without it, jitter can trip communication reflexes",
            hint="install a PREEMPT_RT kernel and set realtime: enforce", warning=True)

    def _internal_realtime_check(self, check_result: type) -> "CheckResult":
        """libfranka's velocity interface aborts on any 1 kHz discontinuity (see _servo_internal)."""
        if self.realtime == "enforce":
            return check_result(5, "mode:internal_realtime", True,
                                "cartesian_controller: internal with realtime: enforce")
        return check_result(
            5, "mode:internal_realtime", True,
            f"cartesian_controller: internal with realtime: {self.realtime}: libfranka's Cartesian velocity "
            "interface aborts on any 1 kHz discontinuity, which control jitter without a PREEMPT_RT kernel trips",
            hint="install a PREEMPT_RT kernel and set realtime: enforce", warning=True)

    def _realtime_kernel_check(self, check_result: type) -> "CheckResult":
        has_rt_kernel = _read_realtime_flag() == "1"
        if has_rt_kernel:
            return check_result(5, "mode:realtime_kernel", True, f"PREEMPT_RT kernel; realtime: {self.realtime}")
        if self.realtime == "ignore":
            return check_result(
                5,
                "mode:realtime_kernel",
                True,
                "no PREEMPT_RT kernel; running with RealtimeConfig.Ignore (control jitter possible)",
            )
        return check_result(
            5,
            "mode:realtime_kernel",
            False,
            "realtime: enforce but the kernel is not PREEMPT_RT",
            hint="install a PREEMPT_RT kernel or set realtime: ignore",
        )

    def _robot_mode_check(self, check_result: type) -> "CheckResult":
        if self.is_connected:
            self.read_state()  # re-read so the doctor sees the mode after an earlier recover
        if self._robot_mode in _BLOCKING_ROBOT_MODES:
            return check_result(
                5,
                "mode:robot_mode",
                False,
                f"robot mode is {self._robot_mode}",
                hint="release the external activation device / clear errors in Desk, then run rwrl recover",
            )
        return check_result(5, "mode:robot_mode", True, f"robot mode is {self._robot_mode or 'unknown'}")

    # --- helpers ---

    @property
    def control_metadata(self) -> dict:
        metadata = {**super().control_metadata, "sdk_version": franky_version(), "sdk_defaults": {},
                    "cartesian_controller": self.cartesian_controller}
        if self.is_connected:
            franky = self.require_sdk()
            for mode, tracker in (("cartesian_servo", franky.CartesianImpedanceTracker), ("joint_servo", franky.JointImpedanceTracker)):
                try:
                    parameters = inspect.signature(tracker).parameters
                except (TypeError, ValueError):
                    continue
                defaults = {name: parameter.default for name, parameter in parameters.items()
                            if name not in ("self", "robot") and parameter.default is not inspect.Parameter.empty
                            and isinstance(parameter.default, (int, float, bool, str))}
                metadata["sdk_defaults"][mode] = defaults
        if self.load is not None:
            metadata["load"] = copy.deepcopy(self.load)
        if self.cartesian_controller == "serl":
            metadata["serl"] = self._serl_metadata()
        return metadata

    def _serl_metadata(self) -> dict:
        """What cartesian_servo and move_to_joints run in serl mode: the controller build and its
        parameters (station defaults, current effective set, values clamped into the .cfg ranges)."""
        serl = {"controller": "serl_franka_controllers CartesianImpedanceController",
                "joint_reset": "serl_franka_controllers JointPositionController",
                "station_params": dict(self._serl_station), "params": dict(self._serl_params),
                "clamped": copy.deepcopy(self._serl_clamped), "robot_model": self.robot_model,
                "auto_recover": dict(self._serl_auto_recover)}
        try:
            sc = self.require_serl()
        except (SdkMissing, VersionMismatch) as error:
            serl["error"] = error.message
            return serl
        serl.update(version=getattr(sc, "__version__", "unknown"), serl_commit=getattr(sc, "SERL_COMMIT", None),
                    limit_rate=sc.FRANKA_ROS_LIMIT_RATE, cutoff_frequency=sc.FRANKA_ROS_CUTOFF_FREQUENCY,
                    build_info=dict(getattr(sc, "BUILD_INFO", {}) or {}))
        return serl

    def _check_control_backend(self, config) -> None:
        if not config.to_dict():
            return
        franky = self.require_sdk()
        for mode, cls in ((ControlMode.CARTESIAN_SERVO, franky.CartesianImpedanceTracker),
                          (ControlMode.JOINT_SERVO, franky.JointImpedanceTracker)):
            requested = config.for_mode(mode.value)
            if not requested:
                continue
            try:
                parameters = inspect.signature(cls).parameters
            except (TypeError, ValueError):
                raise VersionMismatch("cannot inspect installed franky tracker control capabilities", device=self.name) from None
            if not any(p.kind == inspect.Parameter.VAR_KEYWORD for p in parameters.values()):
                unsupported = requested.keys() - parameters.keys()
                if unsupported:
                    raise VersionMismatch(f"installed franky tracker does not support {sorted(unsupported)}", device=self.name)

    def _new_tracker(self, robot: Any, mode: ControlMode) -> Any:
        """Constructing a franky tracker seeds the current pose and starts the torque motion."""
        franky = self.require_sdk()
        tracker_cls = (
            franky.CartesianImpedanceTracker
            if mode is ControlMode.CARTESIAN_SERVO
            else franky.JointImpedanceTracker
        )
        with self._control_exceptions_as_faults(f"starting the {mode.value} tracker"):
            self._check_control_backend(self._control_config)
            kwargs = self._control_config.for_mode(mode.value)
            kwargs = {key: np.asarray(value, dtype=np.float64) if isinstance(value, list) else value for key, value in kwargs.items()}
            return tracker_cls(robot, **kwargs)

    @contextlib.contextmanager
    def _control_exceptions_as_faults(self, operation: str) -> Iterator[None]:
        """Record a libfranka ControlException (e.g. a reflex) and raise it as a recoverable fault."""
        franky = self.require_sdk()
        try:
            yield
        except franky.ControlException as error:
            if _is_preemption(error):
                raise
            message = f"franka {operation} failed: {error}"
            self._note_error(message)
            raise DeviceFault(
                message,
                hint="call recover(); check the workspace / Desk",
                device=self.name,
                recoverable=True,
            ) from error

    def _live_tracker(self, mode: ControlMode) -> Any:
        """Return the running tracker for ``mode``, restarting it after a plain stop() and
        surfacing a reflex that killed it (franky would otherwise drop targets silently)."""
        self._require_servo_mode(mode)
        handle = self._handle
        if handle.tracker is not None and not handle.robot.is_in_control:
            handle.tracker = None
            message = f"franka impedance tracker stopped: {self._motion_error(handle.robot)}"
            self._note_error(message)
            raise DeviceFault(
                message,
                hint="call recover(); the next servo command restarts the tracker",
                device=self.name,
                recoverable=True,
            )
        if handle.tracker is None:
            if self._errors:
                raise DeviceFault(
                    f"arm '{self.name}' has unrecovered errors: {self._errors}",
                    hint="call recover() before servoing",
                    device=self.name,
                    recoverable=True,
                )
            self._enter_control_mode(mode)
        return handle.tracker

    def _require_servo_mode(self, mode: ControlMode) -> None:
        self._require_connected()
        if self._trace_in_progress:
            raise DeviceFault("normal stop observation in progress", device=self.name)
        if self._control_mode is not mode:
            raise DeviceFault(
                f"arm '{self.name}' is in {self._control_mode.value} mode, not {mode.value}",
                hint=f"call set_control_mode(ControlMode.{mode.name}) first",
                device=self.name,
            )

    def _stop_motion(self, handle: _FrankyHandle, *, raise_on_error=False) -> None:
        with self._stop_lock:
            self._stop_motion_unlocked(handle, raise_on_error=raise_on_error)

    def _stop_motion_unlocked(self, handle: _FrankyHandle, *, raise_on_error=False) -> None:
        # Detach before attempting stop: cleanup may retry the robot-level stop,
        # but must not call the same failed tracker a second time.
        tracker, handle.tracker = handle.tracker, None
        serl_motion, handle.serl_motion = handle.serl_motion, None
        handle.joint_reset = None  # a running SERL joint reset is stopped like any position motion
        handle.streaming = False
        operation = ("impedance tracker" if tracker is not None
                     else "SERL impedance controller" if serl_motion is not None else "franka motion")
        try:
            if tracker is not None:
                tracker.stop()
            elif serl_motion is not None:
                self._stop_torque_motion(handle.robot)
            elif handle.robot is not None and handle.robot.is_in_control:
                handle.robot.stop()
                try:
                    completed = handle.robot.join_motion()
                except Exception as error:
                    if not _is_preemption(error):
                        raise
                else:
                    if completed is False:
                        raise RuntimeError("motion join did not complete")
        except Exception as error:
            message = f"stopping the {operation} failed: {error}"
            self._note_error(message)
            if raise_on_error:
                raise DeviceFault(message, device=self.name,
                                  hint="verify robot state before recovery or restarting control") from error

    def _stop_torque_motion(self, robot: Any) -> None:
        """End a running torque motion gracefully, the way franky's trackers stop: franky's
        TorqueStopMotion takes over the same 1 kHz loop, ramps into joint damping and finishes.
        A motion that already ended is joined so its error surfaces."""
        franky = self.require_sdk()
        if robot.is_in_control:
            robot.move(franky.TorqueStopMotion(), asynchronous=True)
        try:
            robot.join_motion()
        except Exception as error:
            if not _is_preemption(error):
                raise

    def _settle_and_recover(self, robot: Any) -> None:
        """Join the previous motion and recover so the next one can start. A real motion error
        stays recorded (health() reports it, servoing refuses to restart) until recover()."""
        try:
            robot.join_motion()
        except Exception as error:
            if not _is_preemption(error):
                self._note_error(f"previous franka motion ended with: {error}")
        robot.recover_from_errors()

    def _require_idle(self, command: str) -> None:
        self._require_connected()
        if self._trace_in_progress:
            raise DeviceFault("normal stop observation in progress", device=self.name)
        if self._control_mode is not ControlMode.IDLE:
            raise DeviceFault(
                f"{command} needs the arm idle, but it is in {self._control_mode.value} mode",
                hint="call set_control_mode(ControlMode.IDLE) first",
                device=self.name,
            )

    def _affine(self, pose: np.ndarray) -> Any:
        franky = self.require_sdk()
        pose = self._checked(pose, (7,))
        return franky.Affine(pose[:3], pose[3:])

    @staticmethod
    def _motion_error(robot: Any) -> str:
        try:
            robot.join_motion()
        except Exception as error:
            return str(error)
        return "control ended without an error"


    @staticmethod
    def _checked(value: np.ndarray, shape: tuple[int, ...]) -> np.ndarray:
        value = np.asarray(value, dtype=np.float64)
        if value.shape != shape:
            raise ValueError(f"expected shape {shape}, got {value.shape}")
        return value


def internal_servo_twist(target: np.ndarray, position: np.ndarray, quaternion_xyzw: np.ndarray):
    """``(linear m/s, angular rad/s)`` in the base frame that move the measured pose
    ``(position, quaternion_xyzw)`` to ``target`` (``xyz + quat_xyzw``) in ``INTERNAL_HORIZON_S``,
    each clipped by norm to ``INTERNAL_MAX_SPEED_M_S``/``INTERNAL_MAX_SPEED_RAD_S``."""
    linear = (np.asarray(target[:3], dtype=np.float64) - position) / INTERNAL_HORIZON_S
    error = _quaternion_matrix(target[3:]) @ _quaternion_matrix(quaternion_xyzw).T
    angular = _rotation_vector(error) / INTERNAL_HORIZON_S
    return (_clip_norm(linear, INTERNAL_MAX_SPEED_M_S).tolist(),
            _clip_norm(angular, INTERNAL_MAX_SPEED_RAD_S).tolist())


def _clip_norm(vector: np.ndarray, limit: float) -> np.ndarray:
    norm = float(np.linalg.norm(vector))
    return vector if norm <= limit else vector * (limit / norm)


def _quaternion_matrix(quaternion_xyzw) -> np.ndarray:
    x, y, z, w = np.asarray(quaternion_xyzw, dtype=np.float64) / np.linalg.norm(quaternion_xyzw)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _rotation_vector(matrix: np.ndarray) -> np.ndarray:
    """Axis * angle of a rotation matrix, angle in [0, pi]."""
    cos = float(np.clip((np.trace(matrix) - 1.0) / 2.0, -1.0, 1.0))
    angle = math.acos(cos)
    axis = np.array([matrix[2, 1] - matrix[1, 2], matrix[0, 2] - matrix[2, 0], matrix[1, 0] - matrix[0, 1]])
    if angle < 1e-9:
        return 0.5 * axis  # small angle: the skew part is the rotation vector
    if math.pi - angle < 1e-6:
        # Near pi the skew part vanishes; take the axis from the symmetric part.
        diagonal = np.sqrt(np.clip((np.diag(matrix) + 1.0) / 2.0, 0.0, None))
        index = int(np.argmax(diagonal))
        axis = (matrix[:, index] + np.eye(3)[:, index]) / 2.0
        return angle * axis / np.linalg.norm(axis)
    return angle * axis / (2.0 * math.sin(angle))


def _is_preemption(error: Exception) -> bool:
    """libfranka reports a motion ended by Robot.stop() as a 'Move command preempted!' ControlException."""
    return "command preempted!" in str(error)


def _read_realtime_flag() -> str:
    try:
        return Path(REALTIME_KERNEL_FLAG_PATH).read_text().strip()
    except OSError:
        return ""
