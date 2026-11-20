"""Adapt single-robot observations to the served policy's input contract."""
from __future__ import annotations

from dataclasses import replace
from typing import ClassVar

import numpy as np
from scipy.spatial.transform import Rotation

from real_world_rl.devices.preprocess import CameraPreprocessConfig
from real_world_rl.envs import rlinf_cartesian
from real_world_rl.envs.real_robot import RealRobotEnvConfig
from real_world_rl.envs.real_robot.env import _state_field_shape


class GenericProfile:
    """Flat state in config.state_fields order; actions go to env.step unchanged."""

    env_overrides: ClassVar[dict] = {}
    default_cameras: ClassVar[dict[str, str]] = {}
    default_image_size: ClassVar[tuple[int, int]] = (224, 224)
    image_channels: ClassVar[slice] = slice(None)

    def __init__(self, config: RealRobotEnvConfig, camera_keys: dict[str, str],
                 state_key: str, prompt_key: str, prompt: str) -> None:
        self.config = config
        self.camera_keys = camera_keys
        self.state_key = state_key
        self.prompt_key = prompt_key
        self.prompt = prompt

    @property
    def action_dim(self) -> int:
        config = self.config
        arm_dim = 6 if config.action_mode == "cartesian_delta" else config.num_joints
        return arm_dim + int(config.gripper_in_action)

    def reset(self, observation: dict, env) -> None:
        """Initialize profile state from a reset observation, if required."""

    def state_vector(self, state: dict) -> np.ndarray:
        return np.concatenate([
            np.asarray(state[name]).reshape(-1) for name in self.config.state_fields
        ]).astype(np.float32)

    def observation(self, state: np.ndarray, frames: dict) -> dict:
        return {
            self.state_key: state,
            self.prompt_key: self.prompt,
            # Snapshot camera buffers before handing the request to a background thread.
            **{key: frames[cam][..., self.image_channels].copy() for cam, key in self.camera_keys.items()},
        }

    def synthetic_observation(self) -> dict:
        state = {}
        for name in self.config.state_fields:
            shape = _state_field_shape(name, self.config.num_joints)
            state[name] = np.zeros(shape, dtype=np.float32)
            if name == "tcp_pose":
                state[name][-1] = 1.0  # Identity xyzw quaternion.
        return self.observation(self.state_vector(state), self._blank_frames())

    def _blank_frames(self) -> dict[str, np.ndarray]:
        return {
            camera: np.zeros((*self.config.image_size, 3), dtype=np.uint8)
            for camera in self.camera_keys
        }


class RLinfFrankaProfile(GenericProfile):
    """LeRobotRealworldDataConfig / RealworldInputs on an RLinf FrankaEnv."""

    env_overrides = {"state_fields": rlinf_cartesian.STATE_FIELDS, "cartesian_semantics": "rlinf"}
    default_cameras = {
        "wrist_1": "observation/image",
        "wrist_2": "observation/extra_view_image",
    }
    default_image_size = (128, 128)
    image_channels = slice(None, None, -1)  # RLinf FrankaEnv._frames_from returns BGR.

    @staticmethod
    def preprocess_config(config: RealRobotEnvConfig, crops: dict) -> dict[str, CameraPreprocessConfig]:
        unknown = set(crops) - set(config.capture_cameras)
        if unknown:
            raise ValueError(f"crop_regions contain unconfigured cameras: {sorted(unknown)}")
        result = {}
        for name in config.capture_cameras:
            camera = config.preprocess.get(name, CameraPreprocessConfig())
            region = crops.get(name, camera.crop_region)
            if region is None and camera.roi is None:
                region = "center_square"
            result[name] = replace(camera, crop_region=region, resize="cv2_linear")
        return result

    def __init__(self, config: RealRobotEnvConfig, camera_keys: dict[str, str],
                 state_key: str, prompt_key: str, prompt: str, *,
                 stiffness_quat: tuple[float, float, float, float]) -> None:
        super().__init__(config, camera_keys, state_key, prompt_key, prompt)
        if not np.isfinite(stiffness_quat).all():
            raise ValueError("stiffness_quat must be a finite xyzw quaternion")
        self._ee_to_stiffness = Rotation.from_quat(stiffness_quat)

    def reset(self, observation: dict, env) -> None:
        self._reset_pose = observation["state"]["tcp_pose"].copy()
        self._max_width_m = env.robot.gripper.max_width_m

    def state_vector(self, state: dict) -> np.ndarray:
        return rlinf_cartesian.policy_state(state, self._reset_pose, self._max_width_m, self._ee_to_stiffness)

    def synthetic_observation(self) -> dict:
        return self.observation(np.zeros(19, dtype=np.float32), self._blank_frames())


# A new policy contract (e.g. DROID joint velocity) can supply its own profile here.
PROFILES = {"generic": GenericProfile, "rlinf_franka": RLinfFrankaProfile}
