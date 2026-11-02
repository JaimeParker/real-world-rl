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
gripper action last sent to the env (+1, open, after a reset), which a gripper-less teleop holds.
"""
from __future__ import annotations

from typing import Any, Optional

import gymnasium as gym
import torch

from rl_garden_real_world.action_contract import (
    LEGACY_ACTION_CONTRACT,
    describe_contract,
    get_env_action_contract,
)
from rl_garden_real_world.teleop.base import TeleopSource, as_teleop_source
from rl_garden_real_world.teleop.registry import make_teleop_source

# RealRobotEnv opens the gripper on reset.
_GRIPPER_ACTION_AFTER_RESET = 1.0


class InterventionWrapper(gym.Wrapper):
    """Operator override of the policy action; see the module docstring for the info keys.

    ``source`` is validated against the env's action contract, then started (``source.start()``).
    """

    def __init__(self, env: gym.Env, source: TeleopSource) -> None:
        super().__init__(env)
        # Set before anything reads them: __getattr__ forwards unknown attributes to the env.
        self._last_gripper_action = _GRIPPER_ACTION_AFTER_RESET
        self._previously_intervened = False
        self.source = as_teleop_source(source)
        self._action_dim = self._validate_against_env(env, self.source)
        self.source.start()

    @property
    def teleop(self) -> TeleopSource:
        """Alias of ``source`` for code written against the single-device wrapper."""
        return self.source

    @staticmethod
    def _validate_against_env(env: gym.Env, source: TeleopSource) -> Optional[int]:
        """Checks the source's declared action space against the env's contract.

        Compares ``action_mode`` and not just widths: a 6-joint ``joint_delta`` arm plus gripper
        is 7-D, the same width as a cartesian twist plus gripper.
        """
        contract = get_env_action_contract(env) or LEGACY_ACTION_CONTRACT
        source_mode = source.action_mode
        source_arm_dim = int(source.arm_action_dim)
        source_has_gripper = bool(source.has_gripper)
        source_dim = source_arm_dim + int(source_has_gripper)
        if source_mode != contract["action_mode"]:
            raise ValueError(
                f"teleop source produces action_mode={source_mode!r} ({source_dim}-D) but the env "
                f"expects action_mode={contract['action_mode']!r} ({describe_contract(contract)}). "
                "Use --real_robot.action_mode cartesian_delta, or a teleop source that declares "
                "a joint action space."
            )
        if not contract["gripper_in_action"]:
            raise ValueError(
                "the intervention wrapper requires gripper_in_action=true (hil_serl, "
                "residual_hil_serl and silri treat the last action dim as the gripper); "
                f"gripper-less envs are not supported yet. Env contract: {describe_contract(contract)}"
            )
        action_space = getattr(env, "single_action_space", None)
        if action_space is not None and source_dim != action_space.shape[-1]:
            raise ValueError(
                f"teleop source produces {source_dim}-D actions (arm {source_arm_dim} + gripper "
                f"{int(source_has_gripper)}, action_mode={source_mode!r}) but the env action space "
                f"is {action_space.shape[-1]}-D ({describe_contract(contract)})"
            )
        return None if action_space is None else int(action_space.shape[-1])

    def __getattr__(self, name: str):
        # gymnasium.Wrapper (>=1.0) no longer forwards arbitrary attributes to
        # ``self.env`` -- but this repo's env-backend contract (num_envs,
        # single_observation_space, ...) relies on direct attribute access,
        # not ``get_wrapper_attr()``, so this wrapper must still be
        # transparent to algorithm code built against the unwrapped env.
        return getattr(self.env, name)

    def reset(self, **kwargs):
        # Env first: events latched while the robot homes are then dropped by the source reset.
        result = self.env.reset(**kwargs)
        self.source.reset()
        self._last_gripper_action = _GRIPPER_ACTION_AFTER_RESET
        self._previously_intervened = False
        return result

    def step(self, action: torch.Tensor):
        reading = self.source.poll()
        intervened = bool(reading.intervened)
        onset = intervened and not self._previously_intervened
        self._previously_intervened = intervened
        policy_action = action if isinstance(action, torch.Tensor) else torch.as_tensor(action)
        info_extra = {}
        if intervened:
            device = action.device if isinstance(action, torch.Tensor) else torch.device("cpu")
            action = torch.as_tensor(reading.action, device=device, dtype=torch.float32).reshape(
                1, -1 if self._action_dim is None else self._action_dim
            )
            info_extra["intervene_action"] = action
        obs, reward, terminated, truncated, info = self.env.step(action)
        # The env clips actions to [-1, 1], so this is the gripper action it actually sent.
        self._last_gripper_action = float(
            torch.as_tensor(action).reshape(-1)[-1].clamp(-1.0, 1.0)
        )
        info = {
            **info,
            **info_extra,
            "policy_action": policy_action,
            "intervened": intervened,
            "intervention_onset": onset,
            "human_episode_end": bool(reading.episode_end),
        }
        if reading.events:
            info["teleop_events"] = dict(reading.events)
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
        if teleop is not None:
            source = as_teleop_source(teleop)
        else:
            if device == "robot":
                # Read lazily: the wrapper's bookkeeping attribute does not exist yet.
                teleop_kwargs.setdefault("last_gripper_action_getter", lambda: self._last_gripper_action)
            source = make_teleop_source(device, env, **teleop_kwargs)
        super().__init__(env, source)
