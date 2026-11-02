"""Leader-arm interventions end to end: a fake joint-space leader part read through
``TeleopInterventionWrapper(device="leader_arm")`` on a follower env, driven by the hil_serl,
residual and SiLRI actor loops, with the pushed rows fed to the learner loops' demo buffers.

Covers what the rows record during the ``ramp`` and ``arm`` onsets (``joint_abs``), in
``joint_delta``, where SiLRI's onset tag lands, and the gripper labels of a binarized trigger."""
from __future__ import annotations

import types

import numpy as np
import pytest
import torch
from _leader_teleop_fakes import (
    JOINT_LIMITS,
    JOINT_SCALE_RAD,
    JointEnv,
    connected_joint_part,
    executed_joints,
    normalized_joints,
)
from _residual_fakes import DictEnv, FakeSyncClient, RecordingProvider, residual_agent
from gymnasium import spaces
from rl_garden.algorithms.rlpd_hybrid import RLPDHybrid, _discrete_labels_from_actions
from robot_infra.devices.backends.teleops.gello import gripper_width_from_raw

from rl_garden_real_world.algorithms.residual import ResidualRLPDHybrid
from rl_garden_real_world.envs.wrappers.intervention import TeleopInterventionWrapper
from rl_garden_real_world.hil_serl import HilSerlActorLoop, HilSerlLearnerLoop
from rl_garden_real_world.residual import ResidualActorLoop, ResidualLearnerLoop
from rl_garden_real_world.silri import SilriActorLoop
from rl_garden_real_world.teleop.leader_arm import TELEOP_SYNCING_EVENT

NUM_JOINTS = 7
ACTION_DIM = NUM_JOINTS + 1
STATE_DIM = 4
RAMP_STEP_RAD = 0.05
ARM_TOLERANCE_RAD = 0.1
# Reached from a follower at zero in four ramp steps (joint 0 limits it: 0.17 rad).
LEADER = [0.17, -0.08, 0.03, 0.0, 0.0, 0.0, 0.12]
FAR = [0.3, -0.02, 0.0, 0.0, 0.0, 0.0, 0.12]
NEAR = [0.05, -0.02, 0.0, 0.0, 0.0, 0.0, 0.0]
NEARER = [0.02, -0.01, 0.0, 0.0, 0.0, 0.0, 0.0]
POLICY_STEPS = 2  # policy steps before the operator's toggle
# Raw trigger angles of the gello backend's gripper decoding.
TRIGGER_OPEN_RAD = 3.0
TRIGGER_CLOSED_RAD = 2.2


class FollowingJointEnv(JointEnv):
    """A ``JointEnv`` whose follower executes the arm action it is stepped with, as
    ``RealRobotEnv`` does (``joint_abs``: the action denormalised onto the joint limits;
    ``joint_delta``: ``follower + action * joint_scale_rad``). Observations are
    ``{"state": (1, 4)}`` filled with ``100 + reset count`` after a reset and with the step
    count after a step. ``followers_before_step[i]`` is the follower step ``i`` started from."""

    def __init__(self, action_mode: str, *, robot) -> None:
        super().__init__(action_mode, num_joints=NUM_JOINTS, robot=robot)
        self.single_action_space = spaces.Box(-1.0, 1.0, (ACTION_DIM,), dtype=np.float32)
        self.followers_before_step: list[np.ndarray] = []
        self.steps = 0

    def reset(self, **kwargs):
        super().reset(**kwargs)
        return {"state": torch.full((1, STATE_DIM), 100.0 + self.reset_calls)}, {}

    def step(self, action):
        self.step_actions.append(action)
        self.followers_before_step.append(np.array(self.follower, dtype=np.float64))
        arm = np.clip(torch.as_tensor(action).detach().double().reshape(-1)[:NUM_JOINTS].numpy(), -1.0, 1.0)
        if self.action_contract["action_mode"] == "joint_abs":
            self.follower = executed_joints(arm)
        else:
            self.follower = self.follower + arm * JOINT_SCALE_RAD
        self.steps += 1
        return (
            {"state": torch.full((1, STATE_DIM), float(self.steps))},
            torch.ones(1),
            torch.tensor([False]),
            torch.tensor([False]),
            {},
        )


class InfoLog:
    """Outermost env layer for the actor loops: keeps every step's ``info``."""

    num_envs = 1

    def __init__(self, env) -> None:
        self.env = env
        self.infos: list[dict] = []

    def reset(self, **kwargs):
        return self.env.reset(**kwargs)

    def step(self, action):
        result = self.env.step(action)
        self.infos.append(result[-1])
        return result


class ZeroPolicy:
    """Proposes the zero action: in ``joint_abs`` the follower holds the middle of its limits
    (zero), in ``joint_delta`` it stays where it is."""

    def eval(self):
        pass

    def load_state_dict(self, state_dict):
        del state_dict

    def predict(self, obs, deterministic=False):
        del obs, deterministic
        return torch.zeros(1, ACTION_DIM)


@pytest.fixture
def leader_env():
    """``make(action_mode, script, onset=...)`` -> ``(follower env, InfoLog over the intervention
    wrapper)``: the leader is a connected fake ``joint_abs`` part on the env's robot, read with
    ``device="leader_arm"`` and the kwargs ``env_building`` derives (a ``joint_delta`` run declares
    the part as with ``--robot-teleop-action-mode joint_delta``)."""
    parts = []

    def make(action_mode: str, script: list, *, onset: str = "ramp"):
        part = connected_joint_part(script, num_joints=NUM_JOINTS)
        parts.append(part)
        env = FollowingJointEnv(action_mode, robot=types.SimpleNamespace(teleop=part))
        kwargs = {
            "joint_limits": JOINT_LIMITS,
            "joint_scale_rad": JOINT_SCALE_RAD,
            "onset": onset,
            "arm_tolerance_rad": ARM_TOLERANCE_RAD,
            "ramp_step_rad": RAMP_STEP_RAD,
        }
        if action_mode == "joint_delta":
            kwargs["teleop_contract"] = ("joint_delta", NUM_JOINTS, True)
        return env, InfoLog(TeleopInterventionWrapper(env, device="leader_arm", **kwargs))

    yield make
    for part in parts:
        part.disconnect()


def toggle_script(leader=LEADER, gripper: float = 1.0) -> list:
    """The operator leaves the policy ``POLICY_STEPS`` steps, then toggles on and holds the
    leader at ``leader`` (the last step repeats)."""
    return [
        {"action": leader, "gripper": gripper, "active": False, "hold": POLICY_STEPS},
        {"action": leader, "gripper": gripper, "active": True},
    ]


def run_loop(loop_cls, env, total_steps: int, **loop_kwargs):
    sync_client = FakeSyncClient()
    loop = loop_cls(env, loop_kwargs.pop("policy", ZeroPolicy()), sync_client, control_hz=1_000_000.0, **loop_kwargs)
    loop.run(total_steps=total_steps)
    return loop, sync_client.pushed


def syncing_flags(infos) -> list[bool]:
    return [bool(info.get("teleop_events", {}).get(TELEOP_SYNCING_EVENT, False)) for info in infos]


def ramp_target(follower: np.ndarray, leader=LEADER) -> np.ndarray:
    """The rate-limited joint target of one ``ramp`` step from ``follower``."""
    return follower + np.clip(np.asarray(leader) - follower, -RAMP_STEP_RAD, RAMP_STEP_RAD)


def joint_delta_action(follower: np.ndarray, leader=LEADER) -> np.ndarray:
    return np.clip((np.asarray(leader) - follower) / JOINT_SCALE_RAD, -1.0, 1.0)


def expected_arm_action(action_mode: str, follower: np.ndarray, leader=LEADER) -> np.ndarray:
    """The leader source's arm action for one intervened ``ramp`` step from ``follower``."""
    if action_mode == "joint_abs":
        return normalized_joints(ramp_target(follower, leader))
    return joint_delta_action(follower, leader)


def as_array(action) -> np.ndarray:
    return torch.as_tensor(action).detach().double().reshape(-1).numpy()


def rlpd_hybrid_agent() -> RLPDHybrid:
    agent = RLPDHybrid(
        env=DictEnv(action_dim=ACTION_DIM),
        device="cpu",
        buffer_device="cpu",
        buffer_size=32,
        batch_size=4,
        learning_starts=1,
        training_freq=4,
        eval_freq=0,
        log_freq=0,
        net_arch=[8],
        discrete_hidden_dim=8,
    )
    agent.init_demo_buffer(buffer_size=16, demo_data_ratio=0.5)
    return agent


def feed_hil_serl_learner(pushed, tmp_path) -> RLPDHybrid:
    agent = rlpd_hybrid_agent()
    learner = HilSerlLearnerLoop(agent, "127.0.0.1", 0, checkpoint_dir=str(tmp_path), buffer_period=1000)
    for transition in pushed:
        learner._on_transition(transition)
    return agent


# -- hil_serl ------------------------------------------------------------------------------


def test_hil_serl_ramp_rows_store_the_executed_targets_and_reach_the_demo_buffer(leader_env, tmp_path):
    env, logged = leader_env("joint_abs", toggle_script())
    _, pushed = run_loop(HilSerlActorLoop, logged, total_steps=POLICY_STEPS + 4)

    assert [t["intervened"] for t in pushed] == [False, False, True, True, True, True]
    # Syncing steps are intervened (and so demo rows), flagged in info for a downstream filter.
    assert syncing_flags(logged.infos) == [False, False, True, True, True, False]
    expected_targets = [
        [0.05, -0.05, 0.03, 0.0, 0.0, 0.0, 0.05],
        [0.10, -0.08, 0.03, 0.0, 0.0, 0.0, 0.10],
        [0.15, -0.08, 0.03, 0.0, 0.0, 0.0, 0.12],
        LEADER,  # caught up: the leader's own normalised target
    ]
    for row, executed, target in zip(pushed[POLICY_STEPS:], env.step_actions[POLICY_STEPS:], expected_targets):
        # The stored action is the command the env executed: the rate-limited target.
        np.testing.assert_allclose(executed_joints(as_array(row["action"])[:NUM_JOINTS]), target, atol=1e-6)
        np.testing.assert_array_equal(as_array(row["action"]), as_array(executed))
        assert as_array(row["action"])[-1] == 1.0
    np.testing.assert_allclose(as_array(pushed[-1]["action"])[:NUM_JOINTS], normalized_joints(LEADER), atol=1e-6)
    for row in pushed[:POLICY_STEPS]:
        np.testing.assert_array_equal(as_array(row["action"]), np.zeros(ACTION_DIM))

    agent = feed_hil_serl_learner(pushed, tmp_path)
    assert len(agent.replay_buffer) == POLICY_STEPS + 4
    assert len(agent.offline_replay_buffer) == 4
    demo_actions = agent.offline_replay_buffer.actions[:4, 0]
    torch.testing.assert_close(demo_actions, torch.cat([row["action"] for row in pushed[POLICY_STEPS:]]))


def test_hil_serl_arm_onset_intervenes_only_once_the_leader_is_within_tolerance(leader_env, tmp_path):
    script = [
        {"action": FAR, "gripper": 1.0, "active": False},
        {"action": FAR, "gripper": 1.0, "active": True, "hold": 2},  # toggled, leader too far
        {"action": NEAR, "gripper": 1.0, "active": True},  # within tolerance: armed
        {"action": NEARER, "gripper": 1.0, "active": True},
    ]
    env, logged = leader_env("joint_abs", script, onset="arm")
    _, pushed = run_loop(HilSerlActorLoop, logged, total_steps=5)

    assert [t["intervened"] for t in pushed] == [False, False, False, True, True]
    assert syncing_flags(logged.infos) == [False] * 5
    # Unarmed toggled steps execute the policy; armed ones follow the leader directly.
    for row in pushed[:3]:
        np.testing.assert_array_equal(as_array(row["action"]), np.zeros(ACTION_DIM))
    for row, leader in zip(pushed[3:], (NEAR, NEARER)):
        np.testing.assert_allclose(as_array(row["action"])[:NUM_JOINTS], normalized_joints(leader), atol=1e-6)
    np.testing.assert_allclose(env.follower, NEARER, atol=1e-6)

    agent = feed_hil_serl_learner(pushed, tmp_path)
    assert len(agent.offline_replay_buffer) == 2


def test_hil_serl_joint_delta_rows_store_the_saturated_leader_minus_follower(leader_env, tmp_path):
    env, logged = leader_env("joint_delta", toggle_script())
    # Four intervened steps bring the follower onto the leader; the fifth is aligned.
    _, pushed = run_loop(HilSerlActorLoop, logged, total_steps=POLICY_STEPS + 5)

    assert [t["intervened"] for t in pushed] == [False] * POLICY_STEPS + [True] * 5
    assert syncing_flags(logged.infos) == [False] * (POLICY_STEPS + 5)  # joint_delta saturates instead
    for index in range(POLICY_STEPS, POLICY_STEPS + 5):
        np.testing.assert_allclose(
            as_array(pushed[index]["action"])[:NUM_JOINTS],
            joint_delta_action(env.followers_before_step[index]),
            atol=1e-5,
        )
    np.testing.assert_allclose(
        as_array(pushed[POLICY_STEPS]["action"])[:NUM_JOINTS], [1.0, -1.0, 0.6, 0.0, 0.0, 0.0, 1.0], atol=1e-6
    )
    np.testing.assert_allclose(as_array(pushed[-1]["action"])[:NUM_JOINTS], np.zeros(NUM_JOINTS), atol=1e-5)
    np.testing.assert_allclose(env.follower, LEADER, atol=1e-6)

    agent = feed_hil_serl_learner(pushed, tmp_path)
    assert len(agent.offline_replay_buffer) == 5


@pytest.mark.parametrize(
    "binarize_threshold, expected_gripper, expected_labels",
    [
        (0.5, [1.0, 1.0, -1.0, -1.0], [2, 2, 0, 0]),
        # Continuous widths: part-squeezed triggers land on the DQN's "stay" label.
        (None, [1.0, 0.375, -0.25, -1.0], [2, 1, 1, 0]),
    ],
)
def test_binarized_trigger_stores_exact_gripper_labels(
    leader_env, tmp_path, binarize_threshold, expected_gripper, expected_labels
):
    raw_trigger_rad = [3.0, 2.75, 2.5, 2.2]  # open, slightly squeezed, mostly squeezed, closed
    script = [
        {
            "action": [0.0] * NUM_JOINTS,
            "active": True,
            # What the gello backend reports for this trigger angle.
            "gripper": gripper_width_from_raw(raw, TRIGGER_OPEN_RAD, TRIGGER_CLOSED_RAD, binarize_threshold),
        }
        for raw in raw_trigger_rad
    ]
    _, logged = leader_env("joint_delta", script)
    _, pushed = run_loop(HilSerlActorLoop, logged, total_steps=len(script))

    agent = feed_hil_serl_learner(pushed, tmp_path)
    demo_actions = agent.offline_replay_buffer.actions[: len(script), 0]
    if binarize_threshold is None:
        torch.testing.assert_close(demo_actions[:, -1], torch.tensor(expected_gripper))
    else:
        assert torch.equal(demo_actions[:, -1], torch.tensor(expected_gripper))
    assert _discrete_labels_from_actions(demo_actions).tolist() == expected_labels


# -- residual ------------------------------------------------------------------------------


@pytest.mark.parametrize("action_mode", ["joint_abs", "joint_delta"])
def test_residual_rows_store_the_human_total_action_next_to_the_base_action(leader_env, tmp_path, action_mode):
    agent = residual_agent(ResidualRLPDHybrid, env=DictEnv(action_dim=ACTION_DIM))
    agent.init_demo_buffer(buffer_size=16, demo_data_ratio=0.5)
    env, logged = leader_env(action_mode, toggle_script())
    provider = RecordingProvider(action_dim=ACTION_DIM)
    total_steps = POLICY_STEPS + 3
    _, pushed = run_loop(
        ResidualActorLoop, logged, total_steps, policy=agent.policy, base_provider=provider, deterministic=True
    )

    assert [t["intervened"] for t in pushed] == [False] * POLICY_STEPS + [True] * 3
    for index, row in enumerate(pushed):
        base = torch.full((1, ACTION_DIM), RecordingProvider.value_for(float(row["obs"]["state"][0, 0])))
        torch.testing.assert_close(row["base_actions"], base)
        # Warm-up: the policy proposes the pure base action, also under intervention.
        torch.testing.assert_close(row["policy_action"], base)
        np.testing.assert_array_equal(as_array(row["action"]), as_array(env.step_actions[index]))
    for index in range(POLICY_STEPS, total_steps):
        # The stored action is the total action the human commanded, not an offset from the base.
        np.testing.assert_allclose(
            as_array(pushed[index]["action"])[:NUM_JOINTS],
            expected_arm_action(action_mode, env.followers_before_step[index]),
            atol=1e-5,
        )
    assert syncing_flags(logged.infos)[POLICY_STEPS] == (action_mode == "joint_abs")

    learner = ResidualLearnerLoop(agent, "127.0.0.1", 0, checkpoint_dir=str(tmp_path), buffer_period=1000)
    for transition in pushed:
        learner._on_transition(transition)
    demo = agent.offline_replay_buffer
    assert len(demo) == 3
    assert bool(demo.intervened[:3].all())
    intervened = pushed[POLICY_STEPS:]
    torch.testing.assert_close(demo.actions[:3, 0], torch.cat([row["action"] for row in intervened]))
    torch.testing.assert_close(demo.base_actions[:3, 0], torch.cat([row["base_actions"] for row in intervened]))


# -- SiLRI ---------------------------------------------------------------------------------


@pytest.mark.parametrize("action_mode", ["joint_abs", "joint_delta"])
def test_silri_onset_tag_lands_on_the_policy_step_before_the_toggle(leader_env, action_mode):
    _, logged = leader_env(action_mode, toggle_script())
    total_steps = POLICY_STEPS + 4
    # The run stops mid-episode: the held last row is still pushed.
    loop, pushed = run_loop(SilriActorLoop, logged, total_steps)

    assert len(pushed) == total_steps and loop._held_transition is None
    torch.testing.assert_close(pushed[-1]["next_obs"]["state"], torch.full((1, STATE_DIM), float(total_steps)))
    assert [t["intervened"] for t in pushed] == [False] * POLICY_STEPS + [True] * 4
    onset = [False] * total_steps
    onset[POLICY_STEPS - 1] = True
    assert [t["intervention_onset"] for t in pushed] == onset
    # The tagged row is a policy step; the wrapper marks the first intervened one.
    tagged = pushed[POLICY_STEPS - 1]
    np.testing.assert_array_equal(as_array(tagged["action"]), np.zeros(ACTION_DIM))
    assert [info["intervention_onset"] for info in logged.infos].index(True) == POLICY_STEPS
    for row in pushed[POLICY_STEPS:]:
        np.testing.assert_array_equal(as_array(row["policy_action"]), np.zeros(ACTION_DIM))


def test_silri_arm_onset_tags_the_last_unarmed_step(leader_env):
    script = [
        {"action": FAR, "gripper": 1.0, "active": False},
        {"action": FAR, "gripper": 1.0, "active": True, "hold": 2},
        {"action": NEAR, "gripper": 1.0, "active": True},
    ]
    _, logged = leader_env("joint_abs", script, onset="arm")
    loop, pushed = run_loop(SilriActorLoop, logged, total_steps=5)

    assert [t["intervened"] for t in pushed] == [False, False, False, True, True]
    # The toggle came on at step 1, but steps 1-2 executed the policy: the human's takeover
    # (and so the tag) starts once the leader is armed.
    assert [t["intervention_onset"] for t in pushed] == [False, False, True, False, False]
    assert loop._held_transition is None
