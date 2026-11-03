"""Reward source adapters that expose the source's explicit success decision."""
from rl_garden.envs.wrappers.reward_classifier import RewardClassifierWrapper


class ExplicitRewardClassifierWrapper(RewardClassifierWrapper):
    def step(self, action):
        obs, reward, terminated, truncated, info = super().step(action)
        info = dict(info)
        info["task_success"] = info["classifier_success_prob"] > self.threshold
        info["task_success_source"] = "classifier"
        info["reward_base"] = reward
        info["reward_base_source"] = "classifier"
        return obs, reward, terminated, truncated, info


class ArmClassifiersWrapper(RewardClassifierWrapper):
    """Independent per-arm labels; task wrapper owns the all-arms decision."""
    def __init__(self, env, classifiers, threshold):
        import gymnasium as gym
        gym.Wrapper.__init__(self, env)
        self.classifiers, self.threshold = classifiers, threshold

    def step(self, action):
        import torch
        obs, reward, terminated, truncated, info = self.env.step(action)
        info = dict(info)
        with torch.no_grad():
            labels = {name: {"success": bool((function(obs) > self.threshold).any()), "source": "classifier"}
                      for name, function in self.classifiers.items()}
        info["arm_task_outcomes"] = labels
        return obs, reward, terminated, truncated, info
