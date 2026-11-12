"""RealRobotEnv: a numpy gym.Env over any device-layer ``Robot`` (local or daemon-served).

Observation is a nested dict ``{"state": {field: float32}, "frames": {cam: uint8 HxWx3}}``;
the rl-garden backend (``envs/backends/real_robot.py``) flattens it to ``state``/``rgb_<cam>``
torch tensors. Action is ``[-1, 1]^D``: 6 (xyz + rotvec) for ``cartesian_delta`` or
``num_joints`` for the joint modes, plus one gripper dimension when ``gripper_in_action``.

Unlike ``FrankaRealEnv``, this env paces itself to ``control_hz``: each ``step()`` sleeps the
remainder of the control period since the previous step's command was sent.
The opt-in SERL branch sleeps after each pose send and before observing instead.
"""
from __future__ import annotations

import logging
import sys
import time
from typing import Any, Optional

import gymnasium as gym
import numpy as np
import torch
from gymnasium import spaces
from gymnasium.vector.utils import batch_space

from real_world_rl.envs.real_robot.config import RealRobotEnvConfig
from real_world_rl.envs.real_robot.preprocess import CameraPreprocessConfig, preprocess_frame
from real_world_rl.envs.real_robot.runtime import RuntimeReportPaths
from real_world_rl.tasks import build_task

_CARTESIAN_ACTION_DIM = 6
_GRIPPER_OPEN_COMMAND = 1.0
_SAFETY_TOLERANCE = 1e-9
# Reading safety_clips costs a daemon round trip on a RemoteRobot, so it is refreshed sparsely.
_SAFETY_CLIPS_REFRESH_STEPS = 10

logger = logging.getLogger(__name__)


def _state_field_shape(field: str, num_joints: int) -> tuple[int, ...]:
    return {
        "tcp_pose": (7,),
        "tcp_vel": (6,),
        "joint_pos": (num_joints,),
        "joint_vel": (num_joints,),
        "gripper_pos": (1,),
        "tcp_force": (3,),
        "tcp_torque": (3,),
    }[field]


def _build_robot(config: RealRobotEnvConfig):
    """Construct (never connect) the robot described by ``config``."""
    if config.robot is not None:
        import real_world_rl.devices.backends  # noqa: F401  (registers backends; SDK imports are lazy)
        from real_world_rl.devices import Robot

        return Robot.from_dict(config.robot)
    from real_world_rl.server import RemoteRobot, RemoteRobotConfig

    return RemoteRobot(RemoteRobotConfig(address=config.remote, token=config.token))


class RealRobotEnv(gym.Env):
    metadata = {"render_modes": ["rgb_array"]}

    def __init__(
        self, config: RealRobotEnvConfig, *, robot: Any = None,
        report_paths: Optional[RuntimeReportPaths] = None,
    ) -> None:
        config.require_robot_source()
        self.config = config
        self._report_paths = RuntimeReportPaths.from_environ() if report_paths is None else report_paths
        self.robot = robot if robot is not None else _build_robot(config)
        self.task_runtime = build_task(config.task, ("arm",))
        self.task_description: str = "" if config.task.name == "none" else config.task.name
        self._expected_control_metadata = None
        self._control_metadata = {}
        self._spatial_contract = self._requested_spatial_contract()
        self._measured_workspace = None
        self._workspace_fault = None
        self._diagnostic_stream = None
        self._output_paths = {}
        self._closed = False
        self._session_events = None
        self._session_events_closed = False
        self._session_stop_reason = None
        self._pending_teleop_diagnostics = None
        self._diagnostic_reset_index = -1
        if "workspace" in self._spatial_contract and config.action_mode != "cartesian_delta":
            raise ValueError("task workspace requires cartesian_delta; joint trajectories lack TCP path enforcement")
        self.render_mode = "rgb_array"

        self._frame_sizes = {cam: self._frame_size(cam) for cam in config.capture_cameras}
        num_joints = config.num_joints
        state_space = spaces.Dict(
            [
                (
                    field,
                    spaces.Box(
                        -np.inf, np.inf, shape=_state_field_shape(field, num_joints), dtype=np.float32
                    ),
                )
                for field in config.state_fields
            ]
        )
        frame_space = spaces.Dict(
            [
                (cam, spaces.Box(0, 255, shape=(h, w, 3), dtype=np.uint8))
                for cam, (h, w) in self._frame_sizes.items() if cam in config.cameras
            ]
        )
        depth_space = spaces.Dict([(cam, spaces.Box(0, np.inf, shape=(*self._frame_sizes[cam], 1), dtype=np.float32))
                                  for cam in config.depth_cameras])
        self.observation_space = spaces.Dict([("state", state_space), ("frames", frame_space)] + ([("depth_frames", depth_space)] if depth_space.spaces else []))
        arm_dim = _CARTESIAN_ACTION_DIM if config.action_mode == "cartesian_delta" else num_joints
        action_dim = arm_dim + (1 if config.gripper_in_action else 0)
        self.action_space = spaces.Box(-1.0, 1.0, shape=(action_dim,), dtype=np.float32)

        self._joint_limits = (
            np.asarray(config.joint_limits, dtype=np.float64)
            if config.joint_limits is not None
            else None
        )
        # Effective limits: the env config, tightened/inherited from the device layer on connect.
        self._effective_safety_box = (
            np.asarray(config.safety_box, dtype=np.float64) if config.safety_box is not None else None
        )
        self._device_max_delta: Optional[dict] = None
        self._safety_clips: Optional[int] = None
        self._period_s = 1.0 / config.control_hz
        self._validated = False
        self._last_raw: Optional[dict] = None
        self._last_obs: Optional[dict] = None
        self._last_send_time: Optional[float] = None
        self._elapsed_steps = 0
        self._step_watchdog = None  # device-layer StepWatchdog, built on the first reset when watchdog_s is set
        if config.cartesian_semantics == "serl":
            self._last_gripper_act = time.monotonic()

    def _frame_size(self, cam: str) -> tuple[int, int]:
        if self.config.image_size is not None:
            return self.config.image_size
        preprocessing = self.config.preprocess.get(cam)
        if preprocessing is not None and preprocessing.roi is not None:
            return tuple(preprocessing.roi[2:])
        camera_configs = {c.name: c for c in getattr(self.robot.config, "cameras", [])}
        if cam not in camera_configs:
            raise ValueError(
                f"camera {cam!r} is not in the robot config (cameras: {sorted(camera_configs)}); "
                "add it there or set image_size"
            )
        return camera_configs[cam].height, camera_configs[cam].width

    def _requested_spatial_contract(self):
        from real_world_rl.devices.workspace import spatial_contract
        local = self.config.robot or {}
        frame = self.config.task_frame if self.config.task_frame is not None else local.get("task_frame")
        workspace = self.config.workspace if self.config.workspace is not None else local.get("workspace")
        return spatial_contract(workspace, frame)

    # -- connection ----------------------------------------------------------------

    def _connect_and_validate(self) -> None:
        if self._validated:
            return
        if not self.robot.is_connected:
            self.robot.connect()
        try:
            self._validate_robot()
        except BaseException:
            try:
                self.robot.disconnect()
            except BaseException:
                logger.exception("disconnect failed after robot validation error")
            raise
        self._validated = True

    def _validate_robot(self) -> None:
        from real_world_rl.devices.errors import DeviceFault

        robot, config = self.robot, self.config

        def fault(message: str, hint: str) -> DeviceFault:
            return DeviceFault(message, hint=hint, device=robot.name)

        arm = robot.arm
        if getattr(self, "_session_hold_required", False):
            self._validate_session_hold(arm)
        if arm.num_joints != config.num_joints:
            raise fault(
                f"robot arm has {arm.num_joints} joints, env config expects {config.num_joints}",
                f"set num_joints={arm.num_joints}",
            )
        available = set(arm.SUPPORTED_STATE_FIELDS)
        if robot.gripper is not None:
            available.add("gripper_pos")
        required = list(config.state_fields)
        required.append("tcp_pose" if config.action_mode == "cartesian_delta" else "joint_pos")
        missing = sorted({f for f in required if f not in available})
        if missing:
            raise fault(
                f"robot does not report state fields {missing} (available: {sorted(available)})",
                "drop them from state_fields (Piper: tcp_pose, joint_pos, joint_vel, gripper_pos)"
                " or pick an action_mode the robot supports",
            )
        if config.gripper_in_action and robot.gripper is None:
            raise fault("robot has no gripper", "set gripper_in_action=False")
        unknown_cams = [cam for cam in config.capture_cameras if cam not in robot.cameras]
        if unknown_cams:
            raise fault(
                f"robot has no cameras {unknown_cams} (available: {sorted(robot.cameras)})",
                "fix the camera names in the env config",
            )
        for camera in config.depth_cameras:
            if not getattr(robot.cameras[camera].config, "enable_depth", False):
                raise ValueError(f"camera {camera} depth is requested but disabled on the station")
        if self._report_paths.stop_trace and "stop_trace_v1" not in getattr(arm, "safety_capabilities", ()):
            raise fault("robot does not support stop_trace_v1", "upgrade the daemon/backend for stop evidence")
        self._merge_device_safety(arm, fault)
        if self._report_paths.diagnostics:
            # Resolve optional geometry imports before reset/home, not on the first
            # timed control step (which may otherwise pay a large import cost).
            import importlib
            importlib.import_module("scipy.spatial.transform")
            if config.action_mode != "cartesian_delta" or "accepted_cartesian_target_v1" not in getattr(arm, "safety_capabilities", ()):
                raise fault("Cartesian target diagnostics are unsupported", "use a current Cartesian backend/daemon")
            if self._diagnostic_stream is None:
                import os
                path = self.output_path("diagnostics")
                path.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
                descriptor = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
                self._diagnostic_stream = os.fdopen(descriptor, "w")
        # joint_abs maps each action onto an absolute target, so consecutive actions can jump
        # across the whole joint range; only the device-layer step limit bounds that.
        if config.action_mode == "joint_abs" and not (self._device_max_delta or {}).get("joint_rad"):
            raise fault(
                "action_mode joint_abs needs the robot's per-step joint limit, but the robot "
                "reports no max_delta_per_step.joint_rad",
                "set max_delta_per_step: {joint_rad: ...} in the robot yaml on the robot PC "
                "(and run a daemon that reports its safety limits)",
            )

    def _merge_device_safety(self, arm, fault) -> None:
        """Inherit the device layer's box/joint limits; an env limit looser than it is a fault."""
        device = getattr(arm, "safety_limits", None)  # None: no limits, or a daemon predating them
        from real_world_rl.devices.workspace import spatial_contract
        actual = spatial_contract((device or {}).get("workspace"), (device or {}).get("task_frame"))
        expected_workspace = self._spatial_contract.get("workspace")
        if expected_workspace is not None:
            if "task_workspace_v1" not in getattr(arm, "safety_capabilities", ()):
                raise fault("robot does not advertise task_workspace_v1 enforcement",
                            "upgrade the daemon before using a task workspace")
        if actual != self._spatial_contract:
            raise fault("configured workspace differs from the robot's enforced workspace",
                        "set real_robot.workspace/task_frame to match the station; do not weaken limits")
        if actual.get("workspace") is not None:
            from real_world_rl.devices.workspace import Workspace
            self._measured_workspace = Workspace(device["workspace"], device.get("task_frame"))
        if device is None:
            return
        self._device_max_delta = device.get("max_delta_per_step")
        hint = (
            "tighten the env safety_box/joint_limits, or loosen the robot yaml on the robot PC"
        )

        device_box = device.get("safety_box")
        if device_box is not None:
            device_box = np.asarray(device_box, dtype=np.float64)
            env_box = self._effective_safety_box
            if env_box is None:
                self._effective_safety_box = device_box
            elif np.any(env_box[:3] < device_box[:3] - _SAFETY_TOLERANCE) or np.any(
                env_box[3:] > device_box[3:] + _SAFETY_TOLERANCE
            ):
                raise fault(
                    f"env safety_box {env_box.tolist()} is looser than the robot's "
                    f"{device_box.tolist()}",
                    hint,
                )

        device_limits = device.get("joint_limits")
        if device_limits is not None:
            device_limits = np.asarray(device_limits, dtype=np.float64)
            env_limits = self._joint_limits
            if env_limits is None:
                self._joint_limits = device_limits
            elif np.any(env_limits[:, 0] < device_limits[:, 0] - _SAFETY_TOLERANCE) or np.any(
                env_limits[:, 1] > device_limits[:, 1] + _SAFETY_TOLERANCE
            ):
                raise fault(
                    f"env joint_limits {env_limits.tolist()} are looser than the robot's "
                    f"{device_limits.tolist()}",
                    hint,
                )

    def _home_joints(self) -> np.ndarray:
        from real_world_rl.devices.errors import DeviceFault

        home = self.config.home_joints
        if home is None:
            home = getattr(self.robot.arm, "home_joints", None)  # RemoteArm, from describe
        if home is None:
            home = getattr(self.robot.config, "home_joints", None)
        if home is None:
            raise DeviceFault(
                "no home joint configuration for reset",
                hint="set home_joints in the env config or the robot yaml",
                device=self.robot.name,
            )
        return np.asarray(home, dtype=np.float64)

    def _send_and_observe(self, action: dict) -> dict:
        """One command + observation; a single round trip on a ``RemoteRobot``."""
        step = getattr(self.robot, "step", None)
        if step is not None:
            return step(action)
        self.robot.send_action(action)
        return self.robot.get_observation()

    # -- gym API ---------------------------------------------------------------------

    # -- step watchdog (config.watchdog_s) ------------------------------------------

    def _activate_step_watchdog(self) -> None:
        if self.config.watchdog_s is None:
            return
        if self._step_watchdog is None:
            from real_world_rl.supervise import StepWatchdog
            self._step_watchdog = StepWatchdog(self.robot.arm, timeout_s=self.config.watchdog_s)
        self._step_watchdog.activate()

    def _deactivate_step_watchdog(self) -> None:
        """Before a reset: the reset motion blocks far longer than a step."""
        watchdog = self._step_watchdog
        if watchdog is None:
            return
        watchdog.deactivate()
        if watchdog.tripped:
            # Silence after the last step of an episode (finalization, saving) is not an error;
            # the stop already happened and the reset re-homes the arm.
            logger.warning("step watchdog stopped the arm before this reset: %s", watchdog.trip_message)

    def _check_in_step_watchdog(self) -> None:
        if self._step_watchdog is not None:
            self._step_watchdog.check_in()

    def _close_step_watchdog(self) -> None:
        if self._step_watchdog is not None:
            self._step_watchdog.close()
            self._step_watchdog = None

    def _check_workspace_feedback(self, pose=None):
        """Latch measured violations and stop; never command a return to the box."""
        from real_world_rl.devices.errors import SafetyViolation
        if self._workspace_fault is not None:
            raise self._workspace_fault
        if self._measured_workspace is None:
            return
        if pose is None:
            pose = self.robot.arm.read_state()["tcp_pose"]
        try:
            violations = self._measured_workspace.violations(pose, measured=True)
        except ValueError as error:
            violations = [str(error)]
        if not violations:
            return
        fault = SafetyViolation("measured TCP outside workspace or invalid: " + "; ".join(violations),
                                device=self.robot.name,
                                hint="inspect feedback and robot state; no automatic return/reset is allowed")
        self._workspace_fault = fault
        self._deactivate_step_watchdog()
        try:
            self.robot.arm.stop()
        except Exception:
            logger.exception("arm stop also failed after measured workspace violation")
        raise fault

    def _reset_begin(self):
        self._closed = False
        from real_world_rl.devices.arm import ControlMode
        self._deactivate_step_watchdog()
        self._connect_and_validate()
        self._check_workspace_feedback()
        self.robot.arm.set_control_mode(ControlMode.IDLE)
        self.robot.arm.configure_control(self.config.control or {})
        from real_world_rl.action_contract import json_normalize
        self._control_metadata = json_normalize({**dict(getattr(self.robot.arm, "control_metadata", {})),
            "resolved_robot": {"home_joints": self.config.home_joints or getattr(self.robot.arm, "home_joints", None) or getattr(self.robot.config, "home_joints", None), "safety_box": self._effective_safety_box,
                               "joint_limits": self._joint_limits, "max_delta_per_step": self._device_max_delta}})
        if self._expected_control_metadata is not None and self._control_metadata != self._expected_control_metadata:
            raise ValueError("effective control/SDK metadata differs from the saved experiment")
        if self.config.preprocess or self.config.depth_cameras:
            self._observe(self.robot.get_observation())

    def _reset_finish(self):
        from real_world_rl.devices.arm import ControlMode
        self.robot.arm.set_control_mode(ControlMode.CARTESIAN_SERVO if self.config.action_mode == "cartesian_delta" else ControlMode.JOINT_SERVO)
        raw = self.robot.get_observation()
        self._elapsed_steps = 0
        self._diagnostic_reset_index += 1
        self._pending_teleop_diagnostics = None
        self._last_send_time = None
        self._refresh_safety_clips()
        obs = self._observe(raw)
        if self._report_paths.stop_trace:
            self.robot.arm.begin_stop_trace()
        self._activate_step_watchdog()
        return obs, {"control_metadata": self._control_metadata}

    def reset(self, *, seed: Optional[int] = None, options: Optional[dict] = None):
        super().reset(seed=seed)
        del options
        if self.config.task.name != "none":
            return self.task_runtime.reset(self, self.np_random)["arm"]
        self._reset_begin()
        self.robot.arm.move_to_joints(self._home_joints(), blocking=True)
        if self.robot.gripper is not None:
            self._send_and_observe({"gripper": {"gripper": np.array([_GRIPPER_OPEN_COMMAND])}})
        if self.config.reset_settle_s > 0: time.sleep(self.config.reset_settle_s)
        obs, _ = self._reset_finish()
        return obs, {}

    def step(self, action):
        if self._last_raw is not None:
            self._check_workspace_feedback(self._last_raw["arm"].get("tcp_pose"))
        if self.config.cartesian_semantics == "serl":
            return self._step_serl(action)
        if self._last_raw is None:
            raise RuntimeError("RealRobotEnv.step() called before reset()")
        self._check_in_step_watchdog()
        action = np.asarray(action, dtype=np.float64).reshape(-1)
        if action.shape != self.action_space.shape:
            raise ValueError(f"expected action shape {self.action_space.shape}, got {action.shape}")
        if not np.isfinite(action).all():
            raise ValueError("action must contain finite numbers")
        input_action = action.copy()
        action = np.clip(action, -1.0, 1.0)

        command: dict[str, Any] = {"arm": self._arm_command(action)}
        if self.config.gripper_in_action:
            command["gripper"] = {"gripper": np.array([(action[-1] + 1.0) / 2.0])}

        if self._last_send_time is not None:
            remaining = self._period_s - (time.monotonic() - self._last_send_time)
            if remaining > 0:
                time.sleep(remaining)
        self._last_send_time = time.monotonic()
        try:
            raw = self._send_and_observe(command)
        except Exception as error:
            self._record_command_failure(input_action, getattr(self, "_diagnostic_requested_pose", None),
                                         command["arm"].get("tcp_pose"), error)
            raise
        if self._diagnostic_stream is not None:
            self._record_control_diagnostic(input_action, self._diagnostic_requested_pose, command["arm"]["tcp_pose"], raw)
        self._check_in_step_watchdog()  # a send that blocked past watchdog_s raises here

        self._elapsed_steps += 1
        obs = self._observe(raw)
        truncated = (
            self.config.max_episode_steps is not None
            and self._elapsed_steps >= self.config.max_episode_steps
        )
        if self._elapsed_steps % _SAFETY_CLIPS_REFRESH_STEPS == 0:
            self._refresh_safety_clips()
        client = getattr(self.robot, "client", None)
        info = {
            "tcp_pose": np.asarray(raw["arm"]["tcp_pose"], dtype=np.float32).copy()
            if "tcp_pose" in raw["arm"]
            else None,
            "rtt_ms": getattr(client, "last_rtt_ms", None),
            "safety_clips": self._safety_clips,
            "elapsed_steps": self._elapsed_steps,
            "task_tcp_positions": {"arm": np.asarray(raw["arm"]["tcp_pose"][:3]).copy()} if "tcp_pose" in raw["arm"] else {},
        }
        return obs, 0.0, False, bool(truncated), info

    def _step_serl(self, action):
        from real_world_rl.envs.serl_cartesian import clip_safety_box, gripper_command, target_pose

        start = time.monotonic()
        if self._last_raw is None:
            raise RuntimeError("RealRobotEnv.step() called before reset()")
        self._check_in_step_watchdog()
        action = np.asarray(action, dtype=np.float64).reshape(-1)
        if action.shape != self.action_space.shape:
            raise ValueError(f"expected action shape {self.action_space.shape}, got {action.shape}")
        if not np.isfinite(action).all():
            raise ValueError("action must contain finite numbers")
        input_action = action.copy()
        action = np.clip(action, self.action_space.low, self.action_space.high)
        config = self.config
        pose = target_pose(self._last_raw["arm"]["tcp_pose"], action,
                           config.xyz_scale_m, config.rot_scale_rad)
        if config.gripper_in_action:
            now = time.monotonic()
            command = gripper_command(
                float(action[-1]), float(np.asarray(self._last_raw["gripper"]["gripper_pos"]).item()),
                now, self._last_gripper_act,
                gripper_action_scale=config.gripper_action_scale,
                gripper_sleep_s=config.gripper_sleep_s,
            )
            if command is not None:
                self.robot.gripper.command_binary(command, force=True)
                self._last_gripper_act = time.monotonic()
                time.sleep(config.gripper_sleep_s)
        requested_pose = pose.copy()
        box = self._effective_safety_box
        pose = clip_safety_box(pose, box[:3], box[3:],
                               config.safety_box_rpy_low, config.safety_box_rpy_high)
        if self._measured_workspace is not None:
            pose = self._measured_workspace.clip_pose(pose)
        # Keep the device's own pose box and per-step safety caps in the send path.
        try:
            self.robot.send_action({"arm": {"tcp_pose": pose}})
        except Exception as error:
            self._record_command_failure(input_action, requested_pose, pose, error)
            raise
        time.sleep(max(0.0, self._period_s - (time.monotonic() - start)))
        raw = self.robot.get_observation()
        if self._diagnostic_stream is not None:
            self._record_control_diagnostic(input_action, requested_pose, pose, raw)
        self._check_in_step_watchdog()

        self._elapsed_steps += 1
        obs = self._observe(raw)
        truncated = (
            config.max_episode_steps is not None
            and self._elapsed_steps >= config.max_episode_steps
        )
        if self._elapsed_steps % _SAFETY_CLIPS_REFRESH_STEPS == 0:
            self._refresh_safety_clips()
        client = getattr(self.robot, "client", None)
        info = {
            "tcp_pose": np.asarray(raw["arm"]["tcp_pose"], dtype=np.float32).copy(),
            "rtt_ms": getattr(client, "last_rtt_ms", None),
            "safety_clips": self._safety_clips,
            "elapsed_steps": self._elapsed_steps,
            "task_tcp_positions": {"arm": np.asarray(raw["arm"]["tcp_pose"][:3]).copy()},
        }
        return obs, 0.0, False, bool(truncated), info

    def set_teleop_diagnostics(self, data):
        if self._report_paths.diagnostics:
            import copy
            self._pending_teleop_diagnostics = copy.deepcopy(data)

    def _record_command_failure(self, action, requested, limited, error):
        """Preserve attempted targets without another query or claiming execution."""
        if self._diagnostic_stream is None:
            return
        try:
            import json
            row = {"schema_version": 1, "kind": "command_failed", "monotonic_s": time.monotonic(),
                   "execution_status": "unknown", "error_type": type(error).__name__, "error": str(error),
                   "input_action": np.asarray(action).tolist(),
                   "requested_pose": None if requested is None else np.asarray(requested).tolist(),
                   "env_limited_pose": None if limited is None else np.asarray(limited).tolist(),
                   "device_accepted_pose": None, "measured_base_pose": None,
                   "step": self._elapsed_steps, "reset_index": self._diagnostic_reset_index,
                   "teleop": self._pending_teleop_diagnostics}
            self._diagnostic_stream.write(json.dumps(row, allow_nan=False) + "\n")
            self._diagnostic_stream.flush()
        except Exception:
            logger.exception("command failure evidence could not be written; preserving device error")
        finally:
            self._pending_teleop_diagnostics = None

    def _record_control_diagnostic(self, input_action, requested, limited, raw, *, kind="step", accepted=None):
        import json
        from scipy.spatial.transform import Rotation
        from real_world_rl.devices.workspace import quaternion_matrix
        if accepted is None:
            accepted = self.robot.arm.last_cartesian_target
        measured = np.asarray(raw["arm"]["tcp_pose"], dtype=float)
        task_pose = None
        frame = self.config.task_frame or (self.config.robot or {}).get("task_frame")
        transform = frame.get("base_T_frame") if frame else None
        if transform is not None:
            transform = np.asarray(transform)
            rotation = transform[:3, :3].T
            task_pose = np.concatenate([rotation @ (measured[:3] - transform[:3, 3]),
                                       Rotation.from_matrix(rotation @ quaternion_matrix(measured[3:])).as_quat()]).tolist()
        reasons = []
        if not np.allclose(requested, limited, atol=1e-9, rtol=0):
            reasons.append("env_pose_limits")
        if accepted is not None and not np.allclose(limited, accepted, atol=1e-9, rtol=0):
            reasons.append("device_limits")
        row = {"schema_version": 1, "monotonic_s": time.monotonic(),
               "kind": kind, "measurement_phase": "before_command" if kind == "hold" else "after_command",
               "input_action": None if input_action is None else np.asarray(input_action).tolist(),
               "requested_pose": np.asarray(requested).tolist(),
               "env_limited_pose": np.asarray(limited).tolist(),
               "device_accepted_pose": None if accepted is None else np.asarray(accepted).tolist(),
               "measured_base_pose": measured.tolist(), "measured_task_pose": task_pose,
               "limit_reasons": reasons, "step": self._elapsed_steps,
               "reset_index": self._diagnostic_reset_index,
               "teleop": self._pending_teleop_diagnostics if kind == "step" else None}
        self._pending_teleop_diagnostics = None
        self._diagnostic_stream.write(json.dumps(row, allow_nan=False) + "\n")
        self._diagnostic_stream.flush()

    def _refresh_safety_clips(self) -> None:
        """Cumulative device-layer clip count; None when the robot cannot report it."""
        from real_world_rl.devices.errors import DeviceError

        try:
            self._safety_clips = int(self.robot.arm.health().safety_clips)
        except (DeviceError, AttributeError):
            self._safety_clips = None

    def _arm_command(self, action: np.ndarray) -> dict[str, np.ndarray]:
        config = self.config
        arm_state = self._last_raw["arm"]
        if config.action_mode == "cartesian_delta":
            from scipy.spatial.transform import Rotation

            pose = np.asarray(arm_state["tcp_pose"], dtype=np.float64)
            xyz = pose[:3] + action[:3] * config.xyz_scale_m
            requested_xyz = xyz.copy()
            if self._effective_safety_box is not None:
                box = self._effective_safety_box
                xyz = np.clip(xyz, box[:3], box[3:])
            rotation = Rotation.from_rotvec(action[3:6] * config.rot_scale_rad) * Rotation.from_quat(
                pose[3:7]
            )
            self._diagnostic_requested_pose = np.concatenate([requested_xyz, rotation.as_quat()])
            target = np.concatenate([xyz, rotation.as_quat()])
            if self._measured_workspace is not None:
                target = self._measured_workspace.clip_pose(target)
            return {"tcp_pose": target}

        num_joints = config.num_joints
        if config.action_mode == "joint_delta":
            q = np.asarray(arm_state["joint_pos"], dtype=np.float64) + (
                action[:num_joints] * config.joint_scale_rad
            )
        else:  # joint_abs; joint_limits is guaranteed by the config
            low, high = self._joint_limits[:, 0], self._joint_limits[:, 1]
            q = low + (action[:num_joints] + 1.0) / 2.0 * (high - low)
        if self._joint_limits is not None:
            q = np.clip(q, self._joint_limits[:, 0], self._joint_limits[:, 1])
        return {"joint_pos": q}

    def _observe(self, raw: dict) -> dict:
        if self._measured_workspace is not None:
            self._check_workspace_feedback(raw["arm"]["tcp_pose"])
        self._last_raw = raw
        state = {}
        for field in self.config.state_fields:
            source = raw["gripper"] if field == "gripper_pos" else raw["arm"]
            state[field] = np.asarray(source[field], dtype=np.float32).reshape(
                self.observation_space["state"][field].shape
            )
        frames, depth_frames = {}, {}
        for cam, size in self._frame_sizes.items():
            preprocessing = self.config.preprocess.get(cam, CameraPreprocessConfig())
            if cam in self.config.cameras:
                frames[cam] = preprocess_frame(raw["cameras"][cam]["color"], preprocessing, size)
            if cam in self.config.depth_cameras:
                depth = raw["cameras"][cam].get("depth")
                if depth is None: raise ValueError(f"camera {cam} returned no requested depth")
                depth_frames[cam] = preprocess_frame(depth, preprocessing, size, depth=True)
        self._last_obs = {"state": state, "frames": frames}
        if depth_frames: self._last_obs["depth_frames"] = depth_frames
        return self._last_obs

    @staticmethod
    def _fit_frame(frame: np.ndarray, size: tuple[int, int]) -> np.ndarray:
        height, width = size
        if frame.shape[:2] == (height, width):
            return frame
        try:
            from PIL import Image
        except ImportError:
            from real_world_rl.devices.errors import SdkMissing

            raise SdkMissing(
                f"resizing camera frames {frame.shape[:2]} -> {size} needs Pillow",
                hint="pip install pillow",
                device="RealRobotEnv",
            ) from None
        return np.asarray(Image.fromarray(frame).resize((width, height), Image.BILINEAR))

    @property
    def last_observation(self) -> Optional[dict]:
        """Nested ``{"state", "frames"}`` dict from the latest reset/step, or None before reset."""
        return self._last_obs

    @property
    def action_mode(self) -> str:
        return self.config.action_mode

    @property
    def gripper_in_action(self) -> bool:
        return self.config.gripper_in_action

    @property
    def arm_action_dim(self) -> int:
        """Action dims excluding the gripper: 6 for cartesian_delta, ``num_joints`` for joint modes."""
        if self.config.action_mode == "cartesian_delta":
            return _CARTESIAN_ACTION_DIM
        return self.config.num_joints

    @property
    def requested_experiment(self):
        import dataclasses
        return {"task": dataclasses.asdict(self.config.task), "control": self.config.control,
                "control_hz": self.config.control_hz, "safety_box": self.config.safety_box,
                "joint_limits": self.config.joint_limits, "home_joints": self.config.home_joints,
                "max_episode_steps": self.config.max_episode_steps, "reset_settle_s": self.config.reset_settle_s,
                "cameras": self.config.cameras, "depth_cameras": self.config.depth_cameras,
                "image_size": self.config.image_size,
                "preprocess": {name: dataclasses.asdict(value) for name, value in self.config.preprocess.items()},
                **self._serl_experiment(),
                **({"spatial_constraints": self._spatial_contract} if self._spatial_contract else {})}

    def _serl_experiment(self) -> dict:
        if self.config.cartesian_semantics != "serl":
            return {}
        from real_world_rl.envs.serl_cartesian import semantics_contract

        config = self.config
        return semantics_contract(
            config.safety_box[:3], config.safety_box[3:],
            config.safety_box_rpy_low, config.safety_box_rpy_high,
            config.gripper_action_scale, config.gripper_sleep_s,
        )

    @property
    def runtime_control(self):
        return {"arm": self._control_metadata}

    @property
    def action_contract(self) -> dict:
        config = self.config
        return {
            "experiment": self.requested_experiment,
            "action_mode": config.action_mode,
            "gripper_in_action": config.gripper_in_action,
            "arm_action_dim": self.arm_action_dim,
            "num_arms": 1,
            "arm_names": ["arm"],
            "num_joints": config.num_joints,
            "joint_limits": config.joint_limits,
            "xyz_scale_m": config.xyz_scale_m,
            "rot_scale_rad": config.rot_scale_rad,
            "joint_scale_rad": config.joint_scale_rad,
            "state_fields": config.state_fields,
        }

    def render(self):
        if self._last_obs is None or not self._last_obs["frames"]:
            return None
        return next(iter(self._last_obs["frames"].values()))

    def require_session_hold(self):
        """Declare the opt-in before reset; defer remote capability checks to connect."""
        if self.config.action_mode != "cartesian_delta":
            raise ValueError("session hold currently requires cartesian_delta")
        first = not getattr(self, "_session_hold_required", False)
        self._session_hold_required = True
        if first:
            self.session_event("READY", "awaiting_reset")
        if self._validated:
            self._validate_session_hold(self.robot.arm)

    def output_path(self, name):
        from real_world_rl.common.json_io import unique_output_path
        if name not in self._output_paths:
            self._output_paths[name] = unique_output_path(getattr(self._report_paths, name))
        return self._output_paths[name]

    def session_observe(self):
        """Fresh post-gate observation, without a step or reset."""
        return self._observe(self.robot.get_observation())

    def session_event(self, phase, reason):
        if phase == "STOPPING":
            if self._session_stop_reason is not None:
                return
            self._session_stop_reason = reason
        if self._report_paths.session_events:
            if self._session_events is None:
                from real_world_rl.teleop.core.session_events import SessionEvents
                self._session_events = SessionEvents(self.output_path("session_events"))
            self._session_events.emit(phase, reason=reason,
                                      episode_index=self._diagnostic_reset_index)

    @staticmethod
    def _validate_session_hold(arm):
        from real_world_rl.devices.errors import DeviceFault
        if "accepted_cartesian_target_v1" not in getattr(arm, "safety_capabilities", ()):
            raise DeviceFault("session hold requires accepted_cartesian_target_v1",
                              hint="upgrade the backend/daemon before starting a hold session")

    def session_start_target(self):
        if self.config.action_mode != "cartesian_delta":
            raise ValueError("session hold currently requires cartesian_delta")
        return np.asarray(self.robot.arm.read_state()["tcp_pose"], dtype=np.float64).copy()

    def session_hold_target(self):
        """The episode's last accepted target. Nothing between the final step and this read
        stops the arm; any stop ends the control segment and leaves no target to hold."""
        if self.config.action_mode != "cartesian_delta":
            raise ValueError("session hold currently requires cartesian_delta")
        # A step-watchdog trip already stopped the arm (and cleared the target): report the trip.
        self._check_in_step_watchdog()
        target = self.robot.arm.last_cartesian_target
        if target is None:
            raise RuntimeError("no accepted Cartesian target to hold")
        return np.asarray(target, dtype=np.float64).copy()

    def session_hold_tick(self, target):
        """Refresh a fixed accepted target without stepping a terminated episode."""
        self._check_in_step_watchdog()
        health = self.robot.arm.health()
        if not health.ok:
            raise RuntimeError(f"cannot hold an unhealthy arm: {health}")
        raw = self.robot.arm.read_state()
        self._check_workspace_feedback(raw["tcp_pose"])
        from real_world_rl.devices.safety import ArmSafety
        limits = self.robot.arm.safety_limits
        if limits:
            ArmSafety.from_dict(limits).require_pose_inside(target)
        self.robot.arm.servo_cartesian(target)
        accepted = self.robot.arm.last_cartesian_target
        if accepted is None or not np.allclose(accepted, target, atol=1e-9, rtol=0):
            raise RuntimeError("holding target was changed by device limits")
        if self._diagnostic_stream is not None:
            self._record_control_diagnostic(None, target, target, {"arm": raw}, kind="hold", accepted=accepted)
        self._check_in_step_watchdog()
        return raw

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        active_exception = sys.exc_info()[0]
        active_failure = active_exception is not None
        exception_type = active_exception.__name__ if active_failure else None
        failure = None
        evidence = None
        try:
            if self._session_events is not None and not self._session_events_closed:
                self.session_event("STOPPING", f"active_exception:{exception_type}" if active_failure else "normal_close")
        except BaseException as error:
            failure = error
        try:
            self._close_step_watchdog()
        except BaseException as error:
            if failure is None:
                failure = error
            else:
                logger.exception("watchdog close also failed")
        try:
            if (self.robot.is_connected and self._report_paths.stop_trace and self._validated
                    and not active_failure and failure is None):
                try:
                    evidence = self.robot.arm.stop_with_trace()
                except BaseException as error:
                    failure = error
                    try:
                        evidence = self.robot.arm.last_stop_trace
                    except Exception:
                        logger.exception("cannot retrieve failed stop trace")
        finally:
            try:
                if self.robot.is_connected:
                    self.robot.disconnect()
            except BaseException as error:
                if failure is None:
                    failure = error
                else:
                    logger.exception("disconnect also failed after stop failure")
            self._validated = False
            self._last_raw = None
        if evidence is not None:
            try:
                import dataclasses
                from real_world_rl.common.provenance import digest_document
                metadata = evidence["trace"].setdefault("metadata", {})
                metadata["session_exit"] = {
                    "reason": self._session_stop_reason or "normal_close",
                    "episode_index": self._diagnostic_reset_index,
                    "active_exception": exception_type,
                    "env_config_sha256": digest_document(dataclasses.asdict(self.config)),
                    "scope": "environment close context; does not establish successful physical stop",
                }
                from real_world_rl.common.json_io import write_json
                target = self.output_path("stop_trace")
                target.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
                write_json(target, evidence["trace"])
                write_json(target.with_name(target.stem + ".assessment.json"), evidence["assessment"])
                if evidence["assessment"].get("status") == "FAIL" and failure is None:
                    failure = RuntimeError(f"normal stop acceptance failed; evidence: {target}")
                elif evidence["assessment"].get("status") == "BLOCKED":
                    logger.warning("normal stop evidence is incomplete: %s", target)
            except Exception as error:
                if failure is None:
                    failure = error
                else:
                    logger.exception("stop evidence export also failed")
        if self._diagnostic_stream is not None:
            stream = self._diagnostic_stream
            self._diagnostic_stream = None
            try:
                stream.close()
            except BaseException as error:
                if failure is None:
                    failure = error
                else:
                    logger.exception("diagnostic stream close also failed")
        try:
            if self._session_events is not None and not self._session_events_closed:
                self.session_event("FAULT" if active_failure or failure else "CLOSED",
                                   f"active_exception:{exception_type}" if active_failure else
                                   "cleanup_failure" if failure else "control_released")
                self._session_events_closed = True
        except BaseException as error:
            if failure is None:
                failure = error
            else:
                logger.exception("session event export also failed")
        if failure is not None:
            if active_failure:
                logger.error("cleanup failed while preserving active rollout error: %s", failure)
            else:
                raise failure


class RealRobotVectorEnv(gym.Env):
    """Batch-of-1 torch view of a ``RealRobotEnv`` satisfying rl-garden's observation vocabulary.

    ``state`` is ``config.state_fields`` concatenated in config order (``(1, D)`` float32) and
    each camera becomes ``rgb_<cam>`` (``(1, H, W, 3)`` uint8). Tensors live on CPU. Like
    ``FrankaRealEnv`` there is no autoreset: the caller resets on ``terminated``/``truncated``
    (``ActorLoop`` does), so the robot is never homed twice per episode boundary.
    """

    num_envs = 1

    def __init__(self, env: RealRobotEnv, *, include_state: bool = True) -> None:
        self.env = env
        self.include_state = include_state
        entries: list[tuple[str, spaces.Space]] = [
            (f"rgb_{cam}", env.observation_space["frames"][cam]) for cam in env.config.cameras
        ]
        entries.extend((f"depth_{cam}", env.observation_space["depth_frames"][cam]) for cam in env.config.depth_cameras)
        if include_state:
            state_dim = sum(
                env.observation_space["state"][field].shape[0] for field in env.config.state_fields
            )
            entries.append(
                ("state", spaces.Box(-np.inf, np.inf, shape=(state_dim,), dtype=np.float32))
            )
        self.single_observation_space = spaces.Dict(entries)
        self.single_action_space = env.action_space
        self.observation_space = batch_space(self.single_observation_space, 1)
        self.action_space = batch_space(self.single_action_space, 1)
        self.metadata = env.metadata

    @property
    def runtime_control(self): return self.env.runtime_control

    @property
    def robot(self):
        return self.env.robot

    @property
    def task_description(self) -> str:
        return self.env.task_description

    @property
    def action_mode(self) -> str:
        return self.env.action_mode

    @property
    def gripper_in_action(self) -> bool:
        return self.env.gripper_in_action

    @property
    def arm_action_dim(self) -> int:
        return self.env.arm_action_dim

    @property
    def action_contract(self) -> dict:
        return self.env.action_contract

    def _to_torch(self, obs: dict) -> dict[str, torch.Tensor]:
        out = {
            f"rgb_{cam}": torch.from_numpy(np.require(obs["frames"][cam], requirements=["C", "W"])).unsqueeze(0)
            for cam in self.env.config.cameras
        }
        out.update({f"depth_{cam}": torch.from_numpy(np.ascontiguousarray(obs["depth_frames"][cam])).unsqueeze(0)
                    for cam in self.env.config.depth_cameras})
        if self.include_state:
            state = np.concatenate([obs["state"][field] for field in self.env.config.state_fields])
            out["state"] = torch.from_numpy(state.astype(np.float32, copy=False)).unsqueeze(0)
        return out

    def reset(self, *, seed: Optional[int] = None, options: Optional[dict] = None):
        obs, info = self.env.reset(seed=seed, options=options)
        return self._to_torch(obs), info

    def step(self, action):
        if isinstance(action, torch.Tensor):
            action = action.detach().cpu().numpy()
        obs, reward, terminated, truncated, info = self.env.step(np.asarray(action).reshape(-1))
        return (
            self._to_torch(obs),
            torch.tensor([reward], dtype=torch.float32),
            torch.tensor([terminated], dtype=torch.bool),
            torch.tensor([truncated], dtype=torch.bool),
            info,
        )

    def session_observe(self):
        return self._to_torch(self.env.session_observe())

    def render(self):
        return self.env.render()

    def close(self) -> None:
        self.env.close()
