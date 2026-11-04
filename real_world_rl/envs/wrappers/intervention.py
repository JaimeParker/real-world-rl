"""Human-in-the-loop intervention wrapper.

``InterventionWrapper`` polls a ``TeleopSource`` before every step and, while the operator
intervenes, executes the operator's action instead of the policy's. It writes into ``info``:

- ``intervene_action``: the executed ``(1, A)`` action, only while intervening;
- ``policy_action``: the action the policy proposed (as received), always;
- ``intervened``: whether the operator's action was executed (python bool), always;
- ``intervention_onset``: the first intervened step after a non-intervened one in the episode
  (python bool), always;
- ``human_episode_end``: the operator's episode-end button (python bool), always;
- ``teleop_events``: the device's ``success``/``failure``/``rerecord`` edges, only when some fired.

An episode-end press ends the episode: ``step()`` returns ``truncated=True`` (as in HIL-SERL), so
actor loops reset without treating it as terminal. ``reset()`` resets the env first and the
source second, so events latched while the robot homes are dropped.

``TeleopInterventionWrapper`` builds the source from a device name (``"pico"``,
``"spacemouse"``, ``"robot"``, ``"fake"`` or anything registered with
``register_teleop_source``), or adapts a duck-typed source object passed as ``teleop``. The
``"robot"`` device (the teleop part of the ``RealRobotEnv``'s robot) takes its mapping inputs
through ``teleop_kwargs`` (``teleop_contract``, ``joint_limits``, ``joint_scale_rad``,
``read_timeout_s``); a teleop read failure propagates out of ``step()``. The wrapper keeps the
gripper actions last sent to the env, one per arm (the last ``num_arms`` action dims, see
``action_contract``; +1, open, after a reset) as a ``(num_arms,)`` tensor, which a gripper-less
teleop part holds: it passes ``last_gripper_action_getter`` (returning that tensor; a source
reads the entry of its arm; ``"bimanual"`` hands each per-arm child a view of its entry) to the
``GRIPPER_HOLDING_DEVICES`` and to the motion half of a ``"composite"`` whose motion device is one
of them.
"""
from __future__ import annotations

from typing import Any, Optional

import gymnasium as gym
import torch

from real_world_rl.action_contract import (
    LEGACY_ACTION_CONTRACT,
    describe_contract,
    get_env_action_contract,
    num_arms_of,
    num_grippers_of,
    with_arm_layout,
)
from real_world_rl.teleop.base import TeleopSource, as_teleop_source
from real_world_rl.teleop.registry import make_teleop_source

# RealRobotEnv opens the gripper on reset.
_GRIPPER_ACTION_AFTER_RESET = 1.0
# Sources that read the last sent gripper action (they take ``last_gripper_action_getter``): the
# ones mapping a robot_infra teleop part hold it when the part has no gripper channel, and
# ``spacemouse`` starts every take-over from it; ``bimanual`` holds one per arm.
GRIPPER_HOLDING_DEVICES = ("robot", "leader_arm", "gello", "leader_cartesian", "spacemouse", "bimanual")


class InterventionWrapper(gym.Wrapper):
    """Operator override of the policy action; see the module docstring for the info keys.

    ``source`` is validated against the env's action contract, then started (``source.start()``).
    """

    def __init__(self, env: gym.Env, source: TeleopSource, *, selected_parts=None, idle_behavior="hold") -> None:
        super().__init__(env)
        # Set before anything reads them: __getattr__ forwards unknown attributes to the env.
        # One gripper per arm: the env must have gripper_in_action (checked below).
        self.source = as_teleop_source(source)
        try:
            self._num_grippers = num_grippers_of(get_env_action_contract(env) or LEGACY_ACTION_CONTRACT)
            from real_world_rl.teleop.action_parts import part_mask
            self._selected_mask = part_mask(get_env_action_contract(env) or LEGACY_ACTION_CONTRACT, selected_parts)
            if idle_behavior not in ("hold", "policy"):
                raise ValueError("idle_behavior must be hold or policy")
            self.idle_behavior = idle_behavior
            self._last_gripper_action = self._gripper_actions_after_reset()
            self._previously_intervened = False
            self._action_dim = self._validate_against_env(env, self.source)
            from real_world_rl.teleop.hold import hold_action_fn_for
            self._hold_action = hold_action_fn_for(env)
            self.source.start()
        except BaseException:
            import logging
            for release in (self.source.close, env.close):
                try:
                    release()
                except BaseException:
                    logging.getLogger(__name__).exception("intervention construction cleanup failed")
            raise

    @property
    def teleop(self) -> TeleopSource:
        """Alias of ``source`` for code written against the single-device wrapper."""
        return self.source

    @staticmethod
    def _validate_against_env(env: gym.Env, source: TeleopSource) -> Optional[int]:
        """Checks the source's declared action space against the env's contract.

        Compares ``action_mode`` and not just widths: a 6-joint ``joint_delta`` arm plus gripper
        is 7-D, the same width as a cartesian twist plus gripper. A source drives one arm unless
        it declares ``num_arms``; it must drive as many arms as the env has, and when it also
        declares ``arm_names`` they must be the env's, in the env's order.
        """
        contract = get_env_action_contract(env) or LEGACY_ACTION_CONTRACT
        source_mode = source.action_mode
        source_arm_dim = int(source.arm_action_dim)
        source_has_gripper = bool(source.has_gripper)
        source_num_arms, source_arm_names = _declared_arm_layout(source)
        source_dim = source_num_arms * (source_arm_dim + int(source_has_gripper))
        if source_mode != contract["action_mode"]:
            raise ValueError(
                f"teleop source produces action_mode={source_mode!r} ({source_dim}-D) but the env "
                f"expects action_mode={contract['action_mode']!r} ({describe_contract(contract)}). "
                "Use --real_robot.action_mode cartesian_delta, or a teleop source that declares "
                "a joint action space."
            )
        env_num_arms = num_arms_of(contract)
        if source_num_arms != env_num_arms:
            raise ValueError(
                f"teleop source drives {source_num_arms} arm(s) but the env has num_arms={env_num_arms} "
                f"({describe_contract(contract)}); use a teleop source that drives {env_num_arms} arm(s)."
            )
        env_arm_names = list(with_arm_layout(contract)["arm_names"])
        if source_arm_names is not None and source_arm_names != env_arm_names:
            raise ValueError(
                f"teleop source drives arms {source_arm_names} but the env's arms are {env_arm_names} "
                f"({describe_contract(contract)}); build the source for the env's arm_names, in order."
            )
        action_space = getattr(env, "single_action_space", None)
        if action_space is not None and source_dim != action_space.shape[-1]:
            arms = f" per arm, {source_num_arms} arms" if source_num_arms != 1 else ""
            raise ValueError(
                f"teleop source produces {source_dim}-D actions (arm {source_arm_dim} + gripper "
                f"{int(source_has_gripper)}{arms}, action_mode={source_mode!r}) but the env action "
                f"space is {action_space.shape[-1]}-D ({describe_contract(contract)})"
            )
        return None if action_space is None else int(action_space.shape[-1])

    def __getattr__(self, name: str):
        # gymnasium.Wrapper (>=1.0) no longer forwards arbitrary attributes to
        # ``self.env`` -- but this repo's env-backend contract (num_envs,
        # single_observation_space, ...) relies on direct attribute access,
        # not ``get_wrapper_attr()``, so this wrapper must still be
        # transparent to algorithm code built against the unwrapped env.
        return getattr(self.env, name)

    def _gripper_actions_after_reset(self) -> torch.Tensor:
        return torch.full((self._num_grippers,), _GRIPPER_ACTION_AFTER_RESET)

    def reset(self, **kwargs):
        # Env first: events latched while the robot homes are then dropped by the source reset.
        result = self.env.reset(**kwargs)
        self.source.reset()
        self._last_gripper_action = self._gripper_actions_after_reset()
        self._previously_intervened = False
        return result

    def step(self, action: torch.Tensor):
        reading = self.source.poll()
        requested = bool(reading.intervened)
        human_mask = torch.as_tensor(self._selected_mask.copy())
        if reading.human_action_mask is not None:
            human_mask &= torch.as_tensor(reading.human_action_mask.copy())
        human_mask &= requested
        intervened = bool(human_mask.any())
        onset = intervened and not self._previously_intervened
        self._previously_intervened = intervened
        policy_action = action if isinstance(action, torch.Tensor) else torch.as_tensor(action)
        info_extra = {}
        if intervened:
            device = action.device if isinstance(action, torch.Tensor) else torch.device("cpu")
            action = torch.as_tensor(reading.action, device=device, dtype=torch.float32).reshape(
                1, -1 if self._action_dim is None else self._action_dim
            )
            human_mask = human_mask.to(action.device).reshape(1, -1)
            proposed = policy_action.to(action.device)
            if self.idle_behavior == "hold" and not bool(human_mask.all()):
                held = self._hold_action(self._last_gripper_action).to(action.device).reshape(1, -1)
                action = torch.where(human_mask, action, held)
                override_mask = torch.ones_like(human_mask)
            else:
                action = torch.where(human_mask, action, proposed)
                override_mask = human_mask.clone()
            info_extra["intervene_action"] = action
        if not intervened:
            human_mask = human_mask.reshape(1, -1).to(policy_action.device)
            override_mask = torch.zeros_like(human_mask)
        obs, reward, terminated, truncated, info = self.env.step(action)
        # The env clips actions to [-1, 1], so these are the gripper actions it actually sent.
        self._last_gripper_action = (
            torch.as_tensor(action).detach().reshape(-1)[-self._num_grippers :].clamp(-1.0, 1.0).cpu()
        )
        if not self._num_grippers:
            self._last_gripper_action = torch.empty(0)
        info = {
            **info,
            "human_action_mask": human_mask,
            "syncing": bool(reading.events.get("teleop_syncing", False)),
            "override_mask": override_mask,
            **info_extra,
            "policy_action": policy_action,
            "intervened": intervened,
            "intervention_onset": onset,
            "human_episode_end": bool(reading.episode_end),
        }
        if reading.events:
            info["teleop_events"] = dict(reading.events)
            names = list(with_arm_layout(get_env_action_contract(self.env) or LEGACY_ACTION_CONTRACT)["arm_names"])
            arm_events = {name: {event: True for event in ("success", "failure") if reading.events.get(f"{name}.{event}", reading.events.get(f"{name}_{event}", False))} for name in names}
            if any(arm_events.values()):
                info["teleop_arm_events"] = arm_events
        if reading.episode_end:
            truncated = (
                torch.ones_like(truncated, dtype=torch.bool)
                if isinstance(truncated, torch.Tensor)
                else True
            )
        return obs, reward, terminated, truncated, info

    def close(self):
        try:
            self.source.close()
        finally:
            self.env.close()


def _declared_arm_layout(source: TeleopSource) -> tuple[int, Optional[list[str]]]:
    """``(num_arms, arm_names)`` the source declares: one arm and no names unless it declares
    them (a composite delegates to its motion half)."""
    arm_names = getattr(source, "arm_names", None)
    num_arms = int(getattr(source, "num_arms", 1))
    return num_arms, None if arm_names is None else [str(name) for name in arm_names]


class TeleopInterventionWrapper(InterventionWrapper):
    """``InterventionWrapper`` that builds its source by device name.

    ``teleop`` (any object with ``poll()``; a ``TeleopSource`` or a duck-typed legacy source)
    takes precedence over ``device``; otherwise ``make_teleop_source(device, env,
    **teleop_kwargs)`` builds it.
    """

    def __init__(
        self,
        env: gym.Env,
        teleop: Optional[Any] = None,
        device: str = "pico",
        **teleop_kwargs: Any,
    ) -> None:
        selected_parts = teleop_kwargs.pop("selected_parts", None)
        idle_behavior = teleop_kwargs.pop("idle_behavior", "hold")
        if teleop is not None:
            source = as_teleop_source(teleop)
        else:
            # Read lazily: the wrapper's bookkeeping attribute does not exist yet.
            hold_gripper = {"last_gripper_action_getter": lambda: self._last_gripper_action}
            if device in GRIPPER_HOLDING_DEVICES:
                teleop_kwargs = {**hold_gripper, **teleop_kwargs}
            elif device == "composite" and teleop_kwargs.get("motion") in GRIPPER_HOLDING_DEVICES:
                motion_kwargs = dict(teleop_kwargs.get("motion_kwargs") or {})
                teleop_kwargs["motion_kwargs"] = {**hold_gripper, **motion_kwargs}
            source = make_teleop_source(device, env, **teleop_kwargs)
        super().__init__(env, source, selected_parts=selected_parts, idle_behavior=idle_behavior)
