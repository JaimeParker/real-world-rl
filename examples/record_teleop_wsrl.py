from __future__ import annotations

import json
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Literal, Optional

import numpy as np
import tyro

from real_world_rl.common.tree import index_tree


@dataclass
class Args:
    output_path: str
    device: Literal["pico_twist", "spacemouse_twist", "fake"] = "pico_twist"
    """pico_twist / spacemouse_twist: the Pico / SpaceMouse as a raw twist (sim only; a real robot
    uses --teleop-device pico or spacemouse, over the command tracker)."""
    zmq_url: str = "tcp://192.168.6.2:7777"
    """pico_twist only."""
    hand: Literal["left", "right"] = "right"
    """pico_twist only."""
    dt: float = 1 / 30

    env_id: str = "PegInsertionSidePegOnly-v1"
    obs_mode: Literal["state", "rgb", "rgbd"] = "rgb"
    include_state: bool = True
    control_mode: Optional[str] = "pd_ee_twist"
    sim_backend: str = "cpu"
    render_backend: str = "gpu"
    reward_mode: Optional[str] = "normalized_dense"
    robot_uids: Optional[str] = "panda_wristcam_gripper_closed_wo_norm"
    camera_width: Optional[int] = None
    camera_height: Optional[int] = None

    fix_box: Optional[bool] = None
    fix_peg_pose: Optional[bool] = None
    peg_density: Optional[float] = None
    debug_pose_vis: Optional[bool] = None
    env_kwargs_json: Optional[str] = None
    end_on_env_done: bool = False

    pos_scale: Optional[float] = None
    rot_scale: Optional[float] = None
    twist_limit: Optional[float] = None
    intervention_threshold: float = 1e-4
    spacemouse_index: int = 0
    """SpaceMouse only: which connected device to open."""
    spacemouse_deadzone: float = 1e-3
    spacemouse_invert_axes: list[int] = field(default_factory=list)
    """SpaceMouse only: output twist indices (0-5) whose sign is flipped."""
    spacemouse_hold_s: float = 0.0


# Only the Pico twist is in physical units; the SpaceMouse (and fake) actions are already in
# the env's normalized [-1, 1] space.
PHYSICAL_TWIST_DEVICES = ("pico_twist",)


def build_teleop_kwargs(args: Args) -> dict:
    if args.device == "pico_twist":
        kwargs = dict(
            zmq_url=args.zmq_url,
            hand=args.hand,
            intervention_threshold=args.intervention_threshold,
        )
        for name in ("pos_scale", "rot_scale", "twist_limit"):
            if getattr(args, name) is not None:
                kwargs[name] = getattr(args, name)
        return kwargs
    if args.device == "spacemouse_twist":
        return dict(
            device_index=args.spacemouse_index,
            deadzone=args.spacemouse_deadzone,
            invert_axes=args.spacemouse_invert_axes,
            hold_s=args.spacemouse_hold_s,
        )
    return {}


def reset_teleop(teleop, *, episode_end_pressed: bool = False) -> None:
    """Resets the source; after a manual episode end the Pico button is still held, so it is told
    not to report it again (other sources have no such state)."""
    from real_world_rl.teleop.pico.twist_sim import PicoTwistTeleopSource

    if episode_end_pressed and isinstance(teleop, PicoTwistTeleopSource):
        teleop.reset(episode_end_pressed=True)
    else:
        teleop.reset()


def main():
    args = tyro.cli(Args)

    import torch
    from mani_skill.utils import gym_utils
    from rl_garden.datasets import PolicySource, WSRLTrajectoryWriter
    from rl_garden.envs import ManiSkillEnvConfig, make_maniskill_env

    from real_world_rl.teleop import make_teleop_source

    env_kwargs = load_env_kwargs(args)
    env_cfg = ManiSkillEnvConfig(
        env_id=args.env_id,
        num_envs=1,
        obs_mode=args.obs_mode,
        include_state=args.include_state,
        control_mode=args.control_mode,
        render_mode="human",
        sim_backend=args.sim_backend,
        render_backend=args.render_backend,
        reward_mode=args.reward_mode,
        robot_uids=args.robot_uids,
        fix_peg_pose=args.fix_peg_pose,
        fix_box=args.fix_box,
        env_kwargs=env_kwargs,
        camera_width=args.camera_width,
        camera_height=args.camera_height,
        ignore_terminations=False,
    )
    env = make_maniskill_env(env_cfg)
    env.auto_reset = False
    max_episode_steps = gym_utils.find_max_episode_steps_value(env)

    # A ManiSkill env has no action contract: sources assume the cartesian twist + gripper convention.
    teleop = make_teleop_source(args.device, env, **build_teleop_kwargs(args))
    teleop.start()

    source = PolicySource(
        tier="success",
        name="teleop",
        path=None,
        target_transitions=0,
        success_rate=1.0,
    )
    metadata = dict(
        collector="teleop",
        env_id=args.env_id,
        obs_mode=args.obs_mode,
        control_mode="" if args.control_mode is None else args.control_mode,
        reward_mode="" if args.reward_mode is None else args.reward_mode,
        device=args.device,
        end_on_env_done=args.end_on_env_done,
        env_kwargs=json.dumps(env_kwargs, sort_keys=True),
    )

    output_path = resolve_output_path(args.output_path)
    print(f"recording dataset to {output_path}", flush=True)
    writer = WSRLTrajectoryWriter(output_path, metadata=metadata)
    try:
        obs, _ = env.reset()
        env.render()
        episode = new_episode(obs)
        twist_normalizer = get_twist_normalizer(env)
        if twist_normalizer is None:
            print("normalize_action=False: saving raw ee_twist actions.", flush=True)
        else:
            print("normalize_action=True: saving normalized env actions.", flush=True)

        while True:
            reading = teleop.poll()
            if reading.episode_end:
                finish_episode(
                    episode=episode,
                    max_episode_steps=max_episode_steps,
                    source=source,
                    writer=writer,
                    reason="manual",
                )
                obs, _ = env.reset()
                env.render()
                episode = new_episode(obs)
                reset_teleop(teleop, episode_end_pressed=True)
                time.sleep(args.dt)
                continue

            if not reading.intervened:
                time.sleep(args.dt)
                continue

            teleop_action = reading.action
            if args.device in PHYSICAL_TWIST_DEVICES:
                teleop_action = normalize_twist(teleop_action, twist_normalizer)
            teleop_action = fit_action(teleop_action, env.action_space.shape)
            action = torch.as_tensor(
                teleop_action, dtype=torch.float32, device=obs_device(obs)
            ).reshape(env.action_space.shape)

            next_obs, reward, terminated, truncated, infos = env.step(action)
            env.render()

            success = bool(extract_success(infos))
            episode["success"] = episode["success"] or success
            term_i = scalar_bool(terminated)
            trunc_i = scalar_bool(truncated)
            env_done = term_i or trunc_i or success
            done = args.end_on_env_done and env_done
            write_term_i = bool(args.end_on_env_done and (term_i or success))
            write_trunc_i = bool(args.end_on_env_done and trunc_i and not write_term_i)

            episode["actions"].append(single_action(action))
            episode["rewards"].append(scalar_tensor(reward))
            episode["terminated"].append(write_term_i)
            episode["truncated"].append(write_trunc_i)
            episode["obs"].append(index_tree(next_obs, 0))
            episode["return"] += float(scalar_tensor(reward).item())
            episode["steps"] += 1

            if done:
                reason = "success" if success else "env"
                finish_episode(
                    episode=episode,
                    max_episode_steps=max_episode_steps,
                    source=source,
                    writer=writer,
                    reason=reason,
                )
                obs, _ = env.reset()
                episode = new_episode(obs)
                reset_teleop(teleop)
            else:
                obs = next_obs

            time.sleep(args.dt)
    except KeyboardInterrupt:
        print("stopped", flush=True)
    finally:
        writer.close()
        teleop.close()
        env.close()


def load_env_kwargs(args) -> dict[str, Any]:
    env_kwargs: dict[str, Any] = {}
    if args.env_kwargs_json is not None:
        env_kwargs = json.loads(args.env_kwargs_json)
        if not isinstance(env_kwargs, dict):
            raise ValueError("--env-kwargs-json must decode to a JSON object.")
    if args.peg_density is not None:
        env_kwargs["peg_density"] = args.peg_density
    if args.debug_pose_vis is not None:
        env_kwargs["debug_pose_vis"] = args.debug_pose_vis
    return env_kwargs


def resolve_output_path(output_path: str) -> Path:
    path = Path(output_path)
    if path.exists() and path.is_dir():
        raise ValueError("--output-path must be an H5 file path, not a directory.")
    if path.suffix not in {".h5", ".hdf5"}:
        print(
            "warning: --output-path is an H5 file path; using a .h5 suffix is recommended",
            flush=True,
        )
    return path


def new_episode(obs) -> dict[str, Any]:
    return {
        "obs": [index_tree(obs, 0)],
        "actions": [],
        "rewards": [],
        "terminated": [],
        "truncated": [],
        "steps": 0,
        "return": 0.0,
        "success": False,
    }


def finish_episode(
    *,
    episode: dict[str, Any],
    max_episode_steps: int | None,
    source,
    writer,
    reason: str,
) -> None:
    if reason == "manual" and episode["truncated"]:
        episode["terminated"][-1] = False
        episode["truncated"][-1] = True

    print("episode ended", flush=True)
    print(
        f"steps={episode['steps']} "
        f"max_episode_steps={max_episode_steps} "
        f"success={episode['success']} "
        f"return={episode['return']:.4f} "
        f"reason={reason}",
        flush=True,
    )
    if episode["steps"] == 0:
        print("no transitions recorded; skipped saving", flush=True)
        return
    if ask_save():
        written = writer.write_episode(
            obs=episode["obs"],
            actions=episode["actions"],
            rewards=episode["rewards"],
            terminated=episode["terminated"],
            truncated=episode["truncated"],
            source=source,
            success=episode["success"],
        )
        print(f"saved episode transitions={written}", flush=True)
    else:
        print("discarded episode", flush=True)


def ask_save() -> bool:
    return input("save this episode? (y/n): ").strip().lower().startswith("y")


def obs_device(obs):
    import torch

    if isinstance(obs, dict):
        return obs_device(next(iter(obs.values())))
    if isinstance(obs, torch.Tensor):
        return obs.device
    return torch.device("cpu")


def single_action(action):
    import torch

    if isinstance(action, torch.Tensor):
        if action.ndim >= 2 and action.shape[0] == 1:
            action = action[0]
        return action.detach().cpu()
    action = np.asarray(action)
    if action.ndim >= 2 and action.shape[0] == 1:
        action = action[0]
    return action


def scalar_tensor(value):
    import torch

    if isinstance(value, torch.Tensor):
        return value.reshape(-1)[0].detach().cpu()
    return torch.as_tensor(np.asarray(value).reshape(-1)[0])


def scalar_bool(value) -> bool:
    return bool(scalar_tensor(value).item())


def extract_success(infos: dict[str, Any]) -> bool | None:
    keys = ("success_at_end", "success_once", "success")
    final_info = infos.get("final_info")
    if isinstance(final_info, dict):
        episode = final_info.get("episode")
        if isinstance(episode, dict):
            for key in keys:
                if key in episode:
                    return scalar_bool(episode[key])
        for key in keys:
            if key in final_info:
                return scalar_bool(final_info[key])
    for key in keys:
        if key in infos:
            return scalar_bool(infos[key])
    return None


def fit_action(action, shape):
    size = int(np.prod(shape))
    if action.size >= size:
        return action[:size]
    return np.pad(action, (0, size - action.size))


def get_twist_normalizer(env):
    controller = env.unwrapped.agent.controller
    arm_controller = getattr(controller, "controllers", {}).get("arm", controller)
    cfg = arm_controller.config
    normalize_action = getattr(arm_controller, "_normalize_action", False)
    if not normalize_action:
        return None

    lower = np.broadcast_to(np.asarray(cfg.twist_lower, dtype=np.float32), 6)
    upper = np.broadcast_to(np.asarray(cfg.twist_upper, dtype=np.float32), 6)
    if np.any(upper <= lower):
        raise ValueError(f"Invalid twist bounds: lower={lower}, upper={upper}")
    return lower, upper


def normalize_twist(action, normalizer):
    if normalizer is None:
        return action

    lower, upper = normalizer
    action = action.copy()
    raw_twist = np.clip(action[:6], lower, upper)
    action[:6] = 2.0 * (raw_twist - lower) / (upper - lower) - 1.0
    return action


if __name__ == "__main__":
    main()
