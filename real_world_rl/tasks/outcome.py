"""Task reward/success arbitration outside classifier and human intervention wrappers."""
from __future__ import annotations
import gymnasium as gym
import numpy as np
import torch


def _flag(value):
    return bool(torch.as_tensor(value).any())


def _like(value, scalar):
    if isinstance(value, torch.Tensor): return torch.full_like(value, scalar)
    if isinstance(value, np.ndarray): return np.full_like(value, scalar)
    return type(value)(scalar)


class TaskOutcomeWrapper(gym.Wrapper):
    def __init__(self, env, config, arm_names):
        super().__init__(env)
        self.task_config, self.arm_names = config, tuple(arm_names)
        self._steps = 0
        self._held = 0
        self._arm_labels = {}
        self._whole_confirmation = False
        self._manual_failure = False
        self._proxy_held = 0

    def __getattr__(self, name): return getattr(self.env, name)

    def reset(self, **kwargs):
        self._steps = 0
        self._held = 0
        self._arm_labels = {}
        self._whole_confirmation = False
        self._manual_failure = False
        self._proxy_held = 0
        return self.env.reset(**kwargs)

    def step(self, action):
        obs, reward, terminated, truncated, info = self.env.step(action)
        info = dict(info)
        config = self.task_config
        self._steps += 1
        positions = info.get("task_tcp_positions", {})
        distances = {name: float(np.linalg.norm(np.asarray(positions[name])-np.asarray(target)))
                     for name, target in config.targets.items() if name in positions}
        proxy = len(distances) == len(self.arm_names) and all(d <= config.position_tolerance_m for d in distances.values())
        info["tcp_proxy_success"] = proxy
        info["tcp_proxy_distances_m"] = distances
        events = info.get("teleop_events") or {}
        arm_events = info.get("teleop_arm_events")
        per_arm = {**dict(info.get("arm_task_outcomes") or {}), **self._arm_labels}
        for name in self.arm_names:
            event = (arm_events or {}).get(name, {})
            if not event:
                event = {"success": events.get(f"{name}.success", events.get(f"{name}_success", False)),
                         "failure": events.get(f"{name}.failure", events.get(f"{name}_failure", False))}
            if _flag(event.get("failure", False)): per_arm[name] = {"success": False, "source": "manual"}
            elif _flag(event.get("success", False)): per_arm[name] = {"success": True, "source": "manual"}
        self._arm_labels = {name: label for name, label in per_arm.items() if label.get("source") == "manual"}
        parallel_labels = len(self.arm_names) > 1 and (arm_events is not None or bool(per_arm))
        failure = (_flag(events.get("task_failure", False)) or _flag(events.get("failure", False))
                   or _flag(info.get("manual_failure", False))) or any(
            item.get("source") == "manual" and item.get("success") is False for item in per_arm.values())
        manual = _flag(events.get("task_success", False))
        if not parallel_labels:
            failure = failure or _flag(events.get("failure", False)) or _flag(info.get("manual_failure", False))
            manual = manual or _flag(events.get("success", False)) or (info.get("task_success_source") == "manual" and _flag(info.get("task_success", False)))
        self._manual_failure = self._manual_failure or failure
        failure = self._manual_failure
        self._whole_confirmation = (self._whole_confirmation or manual) and not failure
        manual = self._whole_confirmation
        self._proxy_held = self._proxy_held + 1 if proxy else 0
        proxy_done = config.success_source == "tcp_proxy" and self._proxy_held >= config.success_hold_steps
        candidate = None
        source = config.success_source
        if manual:
            candidate, source = True, "manual"
            info["outcome_scope"] = "whole_task"
        elif per_arm:
            labels = [per_arm.get(name, {}).get("success") for name in self.arm_names]
            candidate = False if any(value is False for value in labels) else True if all(value is True for value in labels) else None
            sources = {per_arm[name].get("source", "unknown") for name in self.arm_names
                       if per_arm.get(name, {}).get("success") is not None}
            source = next(iter(sources)) if len(sources) == 1 else "mixed" if sources else "unknown"
            info["outcome_scope"] = "all_arms"
        elif len(self.arm_names) == 1 and info.get("task_success_source") in ("manual", "classifier") and info.get("task_success") is not None:
            candidate = _flag(info["task_success"])
            source = info["task_success_source"]
        if failure: candidate, source = False, "manual"
        info["arm_task_outcomes"] = per_arm
        self._held = self._held + 1 if candidate is True else 0
        success = self._held >= config.success_hold_steps
        info["task_outcome_finalized"] = True
        info["task_success"] = None if candidate is None else success
        info["task_success_source"] = source if candidate is not None else "unknown"
        info["task_name"], info["task_version"] = config.name, config.version
        original_reward = reward
        dense = -config.dense_distance_weight * sum(distances.values())
        reward = _like(reward, config.failure_reward if failure else config.success_reward if success or proxy_done else dense)
        if failure:
            info["manual_failure"] = True
            if config.terminate_on_failure: truncated = _like(truncated, True)
        elif (success or proxy_done) and config.terminate_on_success:
            terminated = _like(terminated, True)
        if config.max_episode_steps is not None and self._steps >= config.max_episode_steps:
            truncated = _like(truncated, True)
        info["task_reward_adjustment"] = reward - original_reward
        info["reward_total"] = reward
        return obs, reward, terminated, truncated, info


def apply_task_outcome(env, config):
    task = getattr(config, "task", None)
    if task is None or task.name == "none": return env
    arms = getattr(config, "arm_names", ("arm",))
    return TaskOutcomeWrapper(env, task, arms)
