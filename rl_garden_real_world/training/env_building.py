"""Env construction shared by every HIL-style real-world entrypoint (hil_serl,
residual, SiLRI): the wrapper stack that was ``hil_serl._build_env``.

``build_hil_env`` composes: temporal observation stacking (innermost, optional)
-> rotvec observation conversion (optional) -> reward classifier (optional) ->
human intervention -> gripper-flip penalty (optional) -> reset-free forward/
backward (outermost, per ``FWBWActorLoop``/``FWBWResetFreeWrapper``'s
documented requirement). ``args`` is any Args that carries the fields of
``HilEnvArgs`` plus ``RealWorldFrankaArgs`` and the env-request fields of
``rl_garden.common.env_args``.

``build_hil_env(args, role=..., enable_teleop=...)`` is the entry point: the actor and eval
processes keep the human-intervention wrapper (and so open the teleop device), the learner passes
``enable_teleop=False`` because it only needs the env's spaces and action contract and must not
open a HID/ZMQ device on the training box. ``hil_serl.py``, ``residual_hil_serl.py`` and
``silri.py`` all call it; ``compose_hil_env`` is the wrapper composition alone, for callers
that already hold an env request.
"""
from __future__ import annotations

import dataclasses
from typing import Any, Optional

_FRANKA_REAL_QUAT_SLICE = (3, 7)
_CARTESIAN_ARM_DIM = 6


def tcp_pose_quat_slice(args) -> tuple[int, int] | None:
    """FrankaRealEnv's state always starts with tcp_pose; real_robot's follows state_fields."""
    if args.env_backend != "real_robot":
        return _FRANKA_REAL_QUAT_SLICE
    from rl_garden_real_world.envs.wrappers.obs_chain import tcp_pose_quat_slice as _real_robot_slice

    return _real_robot_slice(args.real_robot)


def hil_env_request(args, run_name: str):
    from rl_garden.common.env_args import make_env_request

    env_request = make_env_request(args, run_name)
    if args.env_backend == "real_robot":
        # The backend cannot stack frames; apply_obs_chain stacks them (as deploy does).
        env_request.observation = dataclasses.replace(env_request.observation, frame_stack=1)
    return env_request


def robot_teleop_kwargs(args) -> dict:
    """``TeleopPartSource`` arguments for ``--teleop-device robot``.

    A ``RemoteRobot`` only learns its teleop part when it connects, so the device is declared
    here (mode + gripper; the arm width follows the env) and checked against the real part on
    connect. A local robot's teleop part is checked at construction either way.
    """
    if args.env_backend != "real_robot":
        raise ValueError("--teleop-device robot needs --env-backend real_robot")
    real_robot = args.real_robot
    kwargs = {"joint_limits": real_robot.joint_limits, "joint_scale_rad": real_robot.joint_scale_rad}
    mode = args.robot_teleop_action_mode
    if mode is not None:
        arm_dim = _CARTESIAN_ARM_DIM if mode == "cartesian_delta" else real_robot.num_joints
        kwargs["teleop_contract"] = (mode, arm_dim, args.robot_teleop_has_gripper)
    return kwargs


def compose_hil_env(args, env_request, *, enable_teleop: bool = True):
    """Builds the backend env for ``env_request`` and applies the wrapper stack
    described in the module docstring. ``enable_teleop=False`` skips the
    intervention wrapper (and so never opens a teleop device or checks the
    teleop source against the env's action contract)."""
    from rl_garden.envs.backend_registry import make_training_envs
    from rl_garden.envs.wrappers.fwbw_reset_free import FWBWResetFreeWrapper
    from rl_garden.envs.wrappers.reward_classifier import RewardClassifierWrapper
    from rl_garden_real_world.envs.wrappers.grasp_penalty import GraspPenaltyWrapper
    from rl_garden_real_world.envs.wrappers.obs_chain import apply_obs_chain

    env, _ = make_training_envs(args.env_backend, env_request)

    env = apply_obs_chain(
        env,
        frame_stack=args.obs.frame_stack,
        # Reuses classifier_image_keys: the same "which keys are cameras" question the
        # classifier wrapper below answers.
        image_keys=args.classifier_image_keys,
        convert_obs_rotation=args.convert_obs_rotation,
        quat_slice=tcp_pose_quat_slice(args),
    )

    if args.classifier_checkpoint is not None:
        from rl_garden.common.utils import get_device
        from rl_garden.models.reward.success.model import load_classifier_fn

        # FrankaRealEnv's observation space is {"state": ..., <camera_key>: image, ...}
        # (see rl_garden_real_world/envs/franka_real/env.py) -- camera keys are whatever
        # the user configured (cfg.camera_keys), not necessarily "rgb*"/"depth*"
        # prefixed, so every non-"state" key is an image key here.
        image_keys = args.classifier_image_keys or tuple(
            k for k in env.single_observation_space.spaces if k != "state"
        )
        classifier_fn = load_classifier_fn(
            args.classifier_checkpoint,
            env.single_observation_space,
            image_keys,
            device=get_device("auto"),
        )
        env = RewardClassifierWrapper(env, classifier_fn, threshold=args.classifier_threshold)

    if enable_teleop:
        from rl_garden_real_world.envs.wrappers.teleop_intervention import TeleopInterventionWrapper

        # Validates the teleop source's action_mode/width against the env's action contract
        # (also rejects gripper_in_action=false, which GraspPenaltyWrapper below relies on).
        teleop_kwargs = robot_teleop_kwargs(args) if args.teleop_device == "robot" else {}
        env = TeleopInterventionWrapper(env, device=args.teleop_device, **teleop_kwargs)

    if args.use_grasp_penalty:
        # Must wrap outside TeleopInterventionWrapper so
        # info["intervene_action"] (the actually-executed action under
        # human intervention) is already present when this runs.
        env = GraspPenaltyWrapper(env, penalty=args.grasp_penalty_value)

    if args.fwbw:
        env = FWBWResetFreeWrapper(env)

    return env


def build_hil_env(
    args,
    *,
    role: str,
    enable_teleop: bool = True,
    run_name: Optional[str] = None,
) -> Any:
    """The HIL env for process ``role`` (``"actor"``, ``"learner"``, ``"eval"``).

    ``run_name`` names the env request (``real_world_<role>`` by default). The
    learner passes ``enable_teleop=False``: it only needs the env's spaces and
    action contract, and must not open a HID/ZMQ device on the training box.
    """
    env_request = hil_env_request(args, run_name or f"real_world_{role}")
    return compose_hil_env(args, env_request, enable_teleop=enable_teleop)
