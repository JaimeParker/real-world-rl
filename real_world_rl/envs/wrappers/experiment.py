"""Final observation/task contract shared by training, recording and deployment."""
import gymnasium as gym
from real_world_rl.action_contract import get_env_action_contract, json_normalize


class ExperimentContractWrapper(gym.Wrapper):
    def __init__(self, env, *, frame_stack=1, convert_obs_rotation=False, source_config=None):
        super().__init__(env)
        self._frame_stack = frame_stack
        self._convert_rotation = convert_obs_rotation
        self._source_config = source_config or {}

    def __getattr__(self, name): return getattr(self.env, name)

    def reset(self, **kwargs):
        return self.env.reset(**kwargs)

    def step(self, action):
        obs, reward, terminated, truncated, info = self.env.step(action)
        info = dict(info)
        manual = info.get("manual_reward_adjustment", 0.0)
        task = info.get("task_reward_adjustment", 0.0)
        intervention = info.get("intervention_reward_adjustment", 0.0)
        base = info.get("reward_base", reward - manual - task - intervention)
        # Each entry is an additive contribution to the final environment reward.
        # Grasp shaping is supplied separately to the critic target.
        info["reward_components"] = {"base": base, "task": task, "manual": manual,
                                     "intervention": intervention}
        info["reward_component_sources"] = {
            "base": info.get("reward_base_source", "environment"),
            "task": info.get("task_name", "none"), "manual": "manual_floor_adjustment",
            "intervention": "intervention_adjustment"}
        info["reward_side_channels"] = {"grasp_penalty": info.get("grasp_penalty", 0.0)}
        info["reward_side_channel_semantics"] = {"grasp_penalty": "critic_target_only"}
        info["reward_total"] = reward
        return obs, reward, terminated, truncated, info

    @property
    def action_contract(self):
        contract = get_env_action_contract(self.env)
        if contract is None: return None
        space = self.env.single_observation_space
        observation = {key: {"shape": list(value.shape), "dtype": str(value.dtype)}
                       for key, value in space.spaces.items()}
        return {**contract, "contract_version": 2,
                "experiment": json_normalize({**contract.get("experiment", {}), "sources": self._source_config,
                    "observation": {"entries": observation, "camera_order": [key for key in observation if key != "state"],
                                    "frame_stack": self._frame_stack, "convert_obs_rotation": self._convert_rotation}}),
                "effective_control": json_normalize(getattr(self.env, "runtime_control", {}))}


def apply_experiment_contract(env, *, frame_stack=1, convert_obs_rotation=False, source_config=None):
    return ExperimentContractWrapper(env, frame_stack=frame_stack, convert_obs_rotation=convert_obs_rotation, source_config=source_config)
