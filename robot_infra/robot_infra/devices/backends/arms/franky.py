"""Franka arm backend on franky (https://github.com/TimSchneider42/franky), imported lazily.

Servo modes stream targets into franky's torque-level impedance trackers; ``move_to_*`` runs
Ruckig-planned position motions. libfranka refuses to switch control-signal type while a motion
runs, so every mode change joins the running motion and recovers before starting the next one.
"""
from __future__ import annotations

import inspect
import contextlib
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

from robot_infra.devices.arm import Arm, ControlMode
from robot_infra.devices.errors import (
    DeviceFault,
    DeviceNotFound,
    SdkMissing,
    VersionMismatch,
)
from robot_infra.devices.safety import ArmSafety

if TYPE_CHECKING:
    from robot_infra.devices.robot import RobotConfig
    from robot_infra.doctor.checks import CheckResult

REALTIME_KERNEL_FLAG_PATH = "/sys/kernel/realtime"
FRANKY_DISTRIBUTION = "franky-control"
FRANKY_WHEEL_INDEX_URL = "https://timschneider42.github.io/franky/whl/by-robot-server-version/{server_version}/"
DESK_SYSTEM_VERSION_PATH = "Desk → Settings → System"
FRANKA_COMPATIBILITY_URL = "https://frankarobotics.github.io/docs/compatibility.html"
FRANKY_VERSION_HINT = (
    "install the franky wheel matching your robot server version: pip install franky-control "
    f"--extra-index-url {FRANKY_WHEEL_INDEX_URL.format(server_version='<N>')} "
    f"(to find N, read the system version in {DESK_SYSTEM_VERSION_PATH} and look it up in the "
    f"compatibility matrix {FRANKA_COMPATIBILITY_URL}; `robot-infra doctor --install-sdk` "
    "reads N from the handshake error and installs the wheel)"
)
_BLOCKING_ROBOT_MODES = ("Reflex", "UserStopped", "Guiding")
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
            hint=f"pip install {FRANKY_DISTRIBUTION} (or: pip install 'rlgarden-robot-infra[franky]')",
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


@dataclass
class _FrankyHandle:
    robot: Any
    tracker: Any = None


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
        home_joints: list[float] | None = None,
        safety: ArmSafety | None = None,
    ) -> None:
        super().__init__(name, num_joints=7, safety=safety)
        if realtime not in ("ignore", "enforce"):
            raise ValueError(f"realtime '{realtime}' invalid; choose from ['ignore', 'enforce']")
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
            home_joints=config.home_joints,
            safety=ArmSafety.from_config(config),
        )

    @classmethod
    def require_sdk(cls) -> ModuleType:
        return require_franky(cls, cls.__name__)

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
        self._read_state_from(robot)
        return handle

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
        tracker = self._live_tracker(ControlMode.CARTESIAN_SERVO)
        tracker.set_target(self._affine(pose))

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
        with self._control_exceptions_as_faults("move_to_joints"):
            self._handle.robot.move(franky.JointMotion(target), asynchronous=not blocking)

    @property
    def safety_capabilities(self):
        return (*super().safety_capabilities, "stop_trace_v1")

    def _new_stop_recorder(self):
        """Collect slow static evidence without holding the motion/stop lock."""
        from robot_infra.devices.stop_sampling import StopTraceRecorder
        from robot_infra.devices.provenance import sdk_artifacts, portable_path
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
        from robot_infra.devices.stop_trace import evaluate_stop_trace
        sampling_errors = []
        with self._stop_lock:
            if self._trace_in_progress:
                raise DeviceFault("normal stop trace is already in progress", device=self.name)
            self._trace_in_progress = True
            self._stop_generation += 1
            recorder = self._stop_recorder
            control_mode = self._control_mode.value
            try:
                tracker_active = bool(self._handle.tracker is not None and self._handle.robot.is_in_control)
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
        from robot_infra.doctor.checks import CheckResult

        return [
            self._realtime_kernel_check(CheckResult),
            self._robot_mode_check(CheckResult),
            CheckResult(5, "mode:libfranka_version", True, f"franky {franky_version()}"),
        ]

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
            self.read_state()  # re-read so a doctor rerun after the recover autofix sees the new mode
        if self._robot_mode in _BLOCKING_ROBOT_MODES:
            return check_result(
                5,
                "mode:robot_mode",
                False,
                f"robot mode is {self._robot_mode}",
                hint="release the external activation device / clear errors in Desk, then run with --fix (recover)",
                autofix=self.recover,
            )
        return check_result(5, "mode:robot_mode", True, f"robot mode is {self._robot_mode or 'unknown'}")

    # --- helpers ---

    @property
    def control_metadata(self) -> dict:
        metadata = {**super().control_metadata, "sdk_version": franky_version(), "sdk_defaults": {}}
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
        return metadata

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
        self._require_connected()
        if self._trace_in_progress:
            raise DeviceFault("normal stop observation in progress", device=self.name)
        if self._control_mode is not mode:
            raise DeviceFault(
                f"arm '{self.name}' is in {self._control_mode.value} mode, not {mode.value}",
                hint=f"call set_control_mode(ControlMode.{mode.name}) first",
                device=self.name,
            )
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

    def _stop_motion(self, handle: _FrankyHandle, *, raise_on_error=False) -> None:
        with self._stop_lock:
            self._stop_motion_unlocked(handle, raise_on_error=raise_on_error)

    def _stop_motion_unlocked(self, handle: _FrankyHandle, *, raise_on_error=False) -> None:
        # Detach before attempting stop: cleanup may retry the robot-level stop,
        # but must not call the same failed tracker a second time.
        tracker, handle.tracker = handle.tracker, None
        operation = "impedance tracker" if tracker is not None else "franka motion"
        try:
            if tracker is not None:
                tracker.stop()
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


def _is_preemption(error: Exception) -> bool:
    """libfranka reports a motion ended by Robot.stop() as a 'Move command preempted!' ControlException."""
    return "command preempted!" in str(error)


def _read_realtime_flag() -> str:
    try:
        return Path(REALTIME_KERNEL_FLAG_PATH).read_text().strip()
    except OSError:
        return ""
