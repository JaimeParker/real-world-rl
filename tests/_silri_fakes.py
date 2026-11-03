"""Shared fakes for the SiLRI tests: a spaces-only dict-observation env, agent factories,
observation/batch builders and buffer fillers. CPU only."""
from __future__ import annotations

from typing import Any, Optional, Sequence

import numpy as np
import torch
from gymnasium import spaces
from gymnasium.vector.utils import batch_space
from real_world_rl.algorithms.silri import SiLRI
from real_world_rl.buffers.intervention_replay_buffer import InterventionSample

from rl_garden.encoders.config import EncoderConfig

STATE_DIM = 4
ACTION_DIM = 3  # two arm dims + the gripper
CONTINUOUS_DIM = ACTION_DIM - 1
TWO_ARM_ACTION_DIM = 6  # two 2-D arm blocks, then the left and right grippers
IMAGE_SHAPE = (64, 64, 3)
# "gap" pooling tolerates the small test image; random_shift exercises prepare_batch.
RGB_ENCODER = EncoderConfig(
    features_dim=16,
    plain_conv_pooling="gap",
    proprio_latent_dim=4,
    image_augmentation="random_shift",
)


class SpacesEnv:
    """Spaces-only one-env vector env: SiLRI training reads its buffers, never steps."""

    num_envs = 1

    def __init__(self, rgb: bool = False, action_dim: int = ACTION_DIM) -> None:
        obs_spaces = {"state": spaces.Box(-1.0, 1.0, (STATE_DIM,), dtype=np.float32)}
        if rgb:
            obs_spaces["rgb_wrist"] = spaces.Box(0, 255, IMAGE_SHAPE, dtype=np.uint8)
        self.single_observation_space = spaces.Dict(obs_spaces)
        self.single_action_space = spaces.Box(-1.0, 1.0, (action_dim,), dtype=np.float32)
        self.observation_space = batch_space(self.single_observation_space, 1)
        self.action_space = batch_space(self.single_action_space, 1)


def silri_agent(rgb: bool = False, action_dim: int = ACTION_DIM, **overrides: Any) -> SiLRI:
    kwargs: dict[str, Any] = dict(
        device="cpu",
        buffer_device="cpu",
        buffer_size=64,
        batch_size=8,
        learning_starts=2,
        demo_learning_starts=2,
        feature_dim=16,
        hidden_dim=16,
        behavior_hidden_dims=(16,),
        lambda_hidden_dims=(16,),
        discrete_gripper=True,
        discrete_hidden_dim=8,
        bc_pretrain_steps=2,
        behavior_refresh_steps=2,
        behavior_refresh_every=3,
        log_freq=0,
        std_log=False,
        seed=0,
    )
    if rgb:
        kwargs["encoder_config"] = RGB_ENCODER
    kwargs.update(overrides)
    return SiLRI(SpacesEnv(rgb=rgb, action_dim=action_dim), **kwargs)


def random_obs(space: spaces.Dict, batch: int = 1) -> dict[str, torch.Tensor]:
    obs = {"state": torch.rand(batch, STATE_DIM) * 2 - 1}
    if "rgb_wrist" in space.spaces:
        obs["rgb_wrist"] = torch.randint(0, 256, (batch, *IMAGE_SHAPE), dtype=torch.uint8)
    return obs


def random_actions(
    batch: int, action_dim: int = ACTION_DIM, gripper: bool = True, num_grippers: int = 1
) -> torch.Tensor:
    """Continuous dims in [-1, 1]; with ``gripper`` each of the last ``num_grippers`` dims is one
    of -1/0/1."""
    actions = torch.rand(batch, action_dim) * 2 - 1
    if gripper and num_grippers == 1:
        actions[:, -1] = torch.randint(-1, 2, (batch,)).float()
    elif gripper:
        actions[:, -num_grippers:] = torch.randint(-1, 2, (batch, num_grippers)).float()
    return actions


def fill(
    agent: SiLRI,
    count: int,
    *,
    demo: bool = False,
    intervened: Optional[bool] = None,
    onset: bool = False,
) -> None:
    """Adds ``count`` random transitions to the online buffer (or the demo buffer). Rows are
    intervened when ``intervened`` is set, else on the demo buffer only."""
    flag = demo if intervened is None else intervened
    space = agent.env.single_observation_space
    action_dim = agent.env.single_action_space.shape[0]
    add = agent.add_demo_transition if demo else agent.replay_buffer.add
    for _ in range(count):
        action = random_actions(1, action_dim, gripper=agent.discrete_gripper, num_grippers=agent.num_grippers)
        add(
            random_obs(space),
            random_obs(space),
            action,
            torch.randn(1),
            torch.zeros(1),
            intervened=flag,
            policy_action=action.clone(),
            intervention_onset=onset,
        )


def hand_batch(
    intervened: Sequence[bool],
    *,
    actions: Optional[torch.Tensor] = None,
    rewards: Optional[torch.Tensor] = None,
    dones: Optional[torch.Tensor] = None,
) -> InterventionSample:
    """A state-only ``InterventionSample`` built by hand."""
    batch = len(intervened)
    actions = random_actions(batch) if actions is None else actions
    return InterventionSample(
        obs={"state": torch.rand(batch, STATE_DIM) * 2 - 1},
        next_obs={"state": torch.rand(batch, STATE_DIM) * 2 - 1},
        actions=actions,
        rewards=torch.randn(batch) if rewards is None else rewards,
        dones=torch.zeros(batch) if dones is None else dones,
        intervened=torch.tensor(list(intervened), dtype=torch.bool),
        policy_actions=actions.clone(),
        intervention_onset=torch.zeros(batch, dtype=torch.bool),
    )
