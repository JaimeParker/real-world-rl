"""The real-robot demo recorder over a joint-space fake env: the hold action per mode, which steps
are recorded, episode outcomes, the episode-start leader check, the saved pkl + contract sidecar
and the example CLI."""
from __future__ import annotations

import importlib.util
import pickle
import sys
from pathlib import Path

import numpy as np
import pytest
import torch
from _bimanual_fakes import bimanual_config, fake_franka
from _leader_teleop_fakes import (
    JointEnv,
    connected_joint_part,
    executed_joints,
    normalized_joints,
)
from _teleop_fakes import ContractEnv, TwoArmContractEnv

import robot_infra.devices.claims as claims

from rl_garden_real_world.action_contract import (
    contract_sidecar_path,
    demo_contract_of,
    validate_transitions_against_contract,
)
from rl_garden_real_world.demos import record
from rl_garden_real_world.demos.record import (
    LeaderFollowerGapError,
    RecordDemosArgs,
    check_leader_near_follower,
    hold_action_fn_for,
    record_demos,
    run_recorder,
    save_demos,
)
from rl_garden_real_world.envs.bimanual import BimanualRealRobotEnv, BimanualRealRobotVectorEnv
from rl_garden_real_world.envs.wrappers.intervention import InterventionWrapper
from rl_garden_real_world.envs.wrappers.teleop_event_reward import TeleopEventRewardWrapper
from rl_garden_real_world.teleop import TeleopReading
from rl_garden_real_world.teleop.bimanual import normalized_joint_target
from rl_garden_real_world.teleop.composite import CompositeTeleopSource
from rl_garden_real_world.teleop.fake import ScriptedTeleopSource
from rl_garden_real_world.teleop.leader_arm import (
    TELEOP_SYNCING_EVENT,
    LeaderArmPartSource,
)
from rl_garden_real_world.training import env_building
from rl_garden_real_world.training.demo_loading import load_demo_pkls

EXAMPLES = Path(__file__).resolve().parents[1] / "examples"
Q1 = [0.2, -0.1, 0.0, 0.3, 0.0, 0.1, 0.0]
Q2 = [0.25, -0.1, 0.05, 0.3, 0.0, 0.1, 0.0]
OPEN, CLOSED = 1.0, -1.0


class RecordingEnv(JointEnv):
    """``JointEnv`` whose follower tracks the executed ``joint_abs`` targets, returning
    ``{"state": full((1, 4), global step)}`` and playing ``rewards``/``terminate_steps``
    (keyed by the global step count, from 1)."""

    def __init__(self, action_mode="joint_abs", rewards=None, terminate_steps=()):
        super().__init__(action_mode)
        self.rewards = dict(rewards or {})
        self.terminate_steps = set(terminate_steps)
        self.steps = 0

    def reset(self, **kwargs):
        super().reset(**kwargs)
        return {"state": torch.zeros(1, 4)}, {}

    def step(self, action):
        self.step_actions.append(action)
        self.steps += 1
        if self.action_contract["action_mode"] == "joint_abs":
            self.follower = executed_joints(torch.as_tensor(action).reshape(-1)[:7].numpy())
        return (
            {"state": torch.full((1, 4), float(self.steps))},
            torch.tensor([self.rewards.get(self.steps, 0.0)]),
            torch.tensor([self.steps in self.terminate_steps]),
            torch.tensor([False]),
            {},
        )


class CountingScriptedSource(ScriptedTeleopSource):
    def __init__(self, readings, **kwargs):
        super().__init__(readings, **kwargs)
        self.polls = 0

    def poll(self):
        self.polls += 1
        return super().poll()


def _reading(q=None, *, intervened=True, gripper=CLOSED, episode_end=False, events=(), delta=None):
    """A scripted reading: ``q`` are joint targets (normalised for joint_abs), ``delta`` a raw
    joint_delta arm action."""
    arm = np.asarray(delta, dtype=np.float64) if delta is not None else normalized_joints(q or [0.0] * 7)
    return TeleopReading(
        action=np.append(arm, gripper).astype(np.float32),
        intervened=intervened,
        episode_end=episode_end,
        events={name: True for name in events},
    )


IDLE = _reading(intervened=False, gripper=OPEN)
SYNC = _reading(intervened=False, gripper=OPEN)  # the episode-start leader read (leader at 0)


def _wrap(env, readings):
    source = ScriptedTeleopSource(readings, action_mode=env.action_contract["action_mode"], arm_action_dim=7)
    return InterventionWrapper(env, source)


def _args(**overrides) -> RecordDemosArgs:
    return RecordDemosArgs(**{"num_successes": 1, "control_hz": 1e6, **overrides})


def _record(wrapped, **overrides):
    episodes = []
    transitions = record_demos(
        wrapped, _args(**overrides), hold_action_fn=hold_action_fn_for(wrapped), on_episode=episodes.append
    )
    return transitions, episodes


def _sent(env, index) -> np.ndarray:
    return torch.as_tensor(env.step_actions[index]).reshape(-1).numpy()


# -- hold action and recorded steps ------------------------------------------------------


def test_joint_abs_holds_the_follower_and_records_only_intervened_steps():
    env = RecordingEnv("joint_abs")
    readings = [
        SYNC,
        IDLE,
        _reading(Q1, events=("success",)),
        IDLE,
        _reading(Q2, episode_end=True),
    ]
    transitions, episodes = _record(_wrap(env, readings))

    assert len(env.step_actions) == 4
    np.testing.assert_allclose(_sent(env, 0), [0.0] * 7 + [OPEN], atol=1e-6)
    # After the takeover the hold is the follower's new pose and the gripper the operator left.
    np.testing.assert_allclose(_sent(env, 2), list(normalized_joints(Q1)) + [CLOSED], atol=1e-5)

    assert len(transitions) == 2 and episodes[0].kept and episodes[0].steps == 4
    first, second = transitions
    np.testing.assert_allclose(first["action"].reshape(-1).numpy(), readings[2].action)
    np.testing.assert_allclose(second["action"].reshape(-1).numpy(), readings[4].action)
    assert first["obs"]["state"][0, 0] == 1.0 and first["next_obs"]["state"][0, 0] == 2.0
    assert second["obs"]["state"][0, 0] == 3.0 and not bool(second["done"])
    assert set(first) == {"obs", "next_obs", "action", "reward", "done"}


def test_joint_delta_holds_with_a_zero_arm_action_and_skips_the_start_check():
    env = RecordingEnv("joint_delta")
    step = [0.5, 0.0, 0.0, 0.0, 0.0, 0.0, -0.5]
    readings = [
        IDLE,
        _reading(delta=step, events=("success",)),
        _reading(intervened=False, episode_end=True),
    ]
    transitions, _ = _record(_wrap(env, readings))

    assert len(env.step_actions) == 3  # no extra leader read before the episode
    np.testing.assert_allclose(_sent(env, 0), [0.0] * 7 + [OPEN])
    np.testing.assert_allclose(_sent(env, 2), [0.0] * 7 + [CLOSED])
    assert len(transitions) == 1
    np.testing.assert_allclose(transitions[0]["action"].reshape(-1).numpy(), readings[1].action)


def test_cartesian_hold_is_a_zero_twist_with_the_held_gripper():
    hold = hold_action_fn_for(ContractEnv("cartesian_delta", 6))
    np.testing.assert_allclose(hold(CLOSED).numpy(), [0.0] * 6 + [CLOSED])


def test_two_arm_hold_is_both_arm_blocks_then_one_held_gripper_per_arm():
    hold = hold_action_fn_for(TwoArmContractEnv("cartesian_delta", 6))
    np.testing.assert_allclose(hold([CLOSED, OPEN]).numpy(), [0.0] * 12 + [CLOSED, OPEN])
    np.testing.assert_allclose(hold(CLOSED).numpy(), [0.0] * 12 + [CLOSED, CLOSED])


def test_two_arm_joint_abs_hold_needs_each_arms_follower():
    env = TwoArmContractEnv("joint_abs", 7)
    env.action_contract["joint_limits"] = [[[-2.0, 2.0]] * 7] * 2
    with pytest.raises(ValueError, match="real_robot_bimanual"):
        hold_action_fn_for(env)


# -- two arms in joint_abs over the two-arm env -----------------------------------------------

LEFT_LIMITS = [[-1.0, 3.0]] * 7
RIGHT_LIMITS = [[-2.0, 2.0]] * 7


@pytest.fixture
def two_arm_joint_abs_env(tmp_path, monkeypatch):
    """A joint_abs two-arm vector env over fake Frankas (left limits [-1, 3], right [-2, 2])."""
    monkeypatch.setattr(claims, "_DEFAULT_LOCK_DIR", tmp_path / "claims")
    config = bimanual_config(
        shared={"action_mode": "joint_abs", "control_hz": 1000.0},
        left={"joint_limits": LEFT_LIMITS},
        right={"joint_limits": RIGHT_LIMITS},
    )
    # joint_abs needs the device layer's per-step joint limit.
    robots = {arm: fake_franka(arm, max_delta_per_step={"joint_rad": 10.0}) for arm in ("left", "right")}
    env = BimanualRealRobotVectorEnv(BimanualRealRobotEnv(config, robots=robots))
    yield env
    env.close()


def _two_arm_joint_reading(left_q, right_q, grippers=(OPEN, OPEN), **kwargs) -> TeleopReading:
    arm = np.concatenate([normalized_joint_target(left_q, LEFT_LIMITS), normalized_joint_target(right_q, RIGHT_LIMITS)])
    return TeleopReading(action=np.append(arm, grippers).astype(np.float32), **kwargs)


def _arm_followers(env) -> dict:
    return {arm: arm_env.last_observation["state"]["joint_pos"] for arm, arm_env in env.env.arms.items()}


def test_two_arm_joint_abs_hold_is_each_arms_follower_on_its_own_limits(two_arm_joint_abs_env):
    env = two_arm_joint_abs_env
    env.reset()
    hold = hold_action_fn_for(env)
    # Both followers at zero: -0.5 on the left's [-1, 3], 0 on the right's [-2, 2].
    np.testing.assert_allclose(hold([CLOSED, OPEN]).numpy(), [-0.5] * 7 + [0.0] * 7 + [CLOSED, OPEN], atol=1e-6)

    step = torch.zeros(1, 16)
    step[0, :7] = 0.0  # left target 1.0 rad on [-1, 3]
    step[0, 7] = 0.5  # right joint 0 target 1.0 rad on [-2, 2]
    env.step(step)
    followers = _arm_followers(env)
    expected = np.concatenate(
        [
            np.clip(2.0 * (followers["left"] - (-1.0)) / 4.0 - 1.0, -1.0, 1.0),
            np.clip(2.0 * (followers["right"] - (-2.0)) / 4.0 - 1.0, -1.0, 1.0),
        ]
    )
    assert not np.allclose(followers["left"], 0.0)  # the hold follows the moved followers
    np.testing.assert_allclose(hold(OPEN).numpy()[:14], expected, atol=1e-5)


def test_two_arm_start_check_compares_each_leader_with_its_own_follower(two_arm_joint_abs_env):
    env = two_arm_joint_abs_env
    env.reset()
    contract = env.action_contract
    near = ScriptedTeleopSource(
        [_two_arm_joint_reading([0.1] * 7, [0.0] * 6 + [-0.2], intervened=False)],
        action_mode="joint_abs",
        arm_action_dim=7,
        num_arms=2,
    )
    errors = check_leader_near_follower(env, near, contract, 0.8)
    assert errors.shape == (2, 7)
    np.testing.assert_allclose(errors[0], [0.1] * 7, atol=1e-5)
    np.testing.assert_allclose(errors[1], [0.0] * 6 + [0.2], atol=1e-5)

    far_right = ScriptedTeleopSource(
        [_two_arm_joint_reading([0.1] * 7, [0.9] + [0.0] * 6, intervened=False)],
        action_mode="joint_abs",
        arm_action_dim=7,
        num_arms=2,
    )
    with pytest.raises(LeaderFollowerGapError, match=r"leader arm 'right'.*0\.9.*start_tolerance_rad = 0\.8"):
        check_leader_near_follower(env, far_right, contract, 0.8)


def test_two_arm_joint_abs_recording_keeps_both_arms_intervened_steps(two_arm_joint_abs_env):
    env = two_arm_joint_abs_env
    zeros = [0.0] * 7
    readings = [
        _two_arm_joint_reading(zeros, zeros, intervened=False),  # the episode-start leader read
        _two_arm_joint_reading(zeros, zeros, intervened=False),
        _two_arm_joint_reading([0.1] * 7, zeros, (CLOSED, OPEN), intervened=True, events={"success": True}),
        _two_arm_joint_reading([0.1] * 7, zeros, (CLOSED, OPEN), intervened=False, episode_end=True),
    ]
    source = ScriptedTeleopSource(readings, action_mode="joint_abs", arm_action_dim=7, num_arms=2)
    transitions, episodes = _record(InterventionWrapper(env, source))

    assert episodes[0].kept and episodes[0].steps == 3 and len(transitions) == 1
    np.testing.assert_allclose(transitions[0]["action"].reshape(-1).numpy(), readings[2].action)


def test_max_episode_steps_caps_the_two_arm_env():
    args = record._with_episode_step_cap(_args(env_backend="real_robot_bimanual", max_episode_steps=40))
    assert args.real_robot_bimanual.max_episode_steps == 40
    assert args.real_robot_bimanual.left.max_episode_steps is None


class _TwoArmScriptedSource(ScriptedTeleopSource):
    num_arms = 2


def test_two_arm_recording_holds_both_last_executed_grippers():
    def two_arm(grippers, **kwargs):
        return TeleopReading(action=np.array([0.0] * 12 + list(grippers), dtype=np.float32), **kwargs)

    env = TwoArmContractEnv("cartesian_delta", 6)
    readings = [
        two_arm([OPEN, OPEN], intervened=False),
        two_arm([CLOSED, OPEN], intervened=True, events={"success": True}),
        two_arm([OPEN, OPEN], intervened=False, episode_end=True),
    ]
    transitions, _ = _record(InterventionWrapper(env, _TwoArmScriptedSource(readings)))

    assert len(env.step_actions) == 3 and len(transitions) == 1
    np.testing.assert_allclose(_sent(env, 0), [0.0] * 12 + [OPEN, OPEN])
    np.testing.assert_allclose(_sent(env, 2), [0.0] * 12 + [CLOSED, OPEN])


@pytest.mark.parametrize("keep_syncing_steps, recorded", [(False, 1), (True, 3)])
def test_syncing_steps_are_dropped_unless_kept(keep_syncing_steps, recorded):
    env = RecordingEnv("joint_abs")
    readings = [
        SYNC,
        _reading(Q1, events=(TELEOP_SYNCING_EVENT,)),
        _reading(Q1, events=(TELEOP_SYNCING_EVENT,)),
        _reading(Q1, events=("success",), episode_end=True),
    ]
    transitions, _ = _record(_wrap(env, readings), keep_syncing_steps=keep_syncing_steps)
    assert len(transitions) == recorded


# -- episode outcomes ----------------------------------------------------------------------


def test_rerecord_drops_the_episode_and_starts_the_next_one():
    env = RecordingEnv("joint_abs")
    readings = [
        SYNC,
        _reading(Q1),
        _reading(Q1, events=("success", "rerecord")),
        _reading(Q1, intervened=False),  # next episode's leader read: the follower is at Q1
        _reading(Q2, events=("success",), episode_end=True),
    ]
    transitions, episodes = _record(_wrap(env, readings))

    assert [episode.discarded_reason for episode in episodes] == ["rerecord", None]
    assert env.reset_calls == 2
    assert len(transitions) == 1
    np.testing.assert_allclose(transitions[0]["action"].reshape(-1).numpy(), readings[4].action)


def test_stops_after_num_successes_counting_success_events_and_positive_rewards():
    # One step per episode (the reading before each is the episode-start leader read); episode 3
    # is global step 3, where the env pays a reward and terminates.
    env = RecordingEnv("joint_abs", rewards={3: 1.0}, terminate_steps={3})
    readings = [
        SYNC, _reading(episode_end=True),
        SYNC, _reading(events=("success", "failure"), episode_end=True),
        SYNC, _reading(),
        SYNC, _reading(events=("success",), episode_end=True),
    ]
    transitions, episodes = _record(_wrap(env, readings), num_successes=2)

    assert [episode.discarded_reason for episode in episodes] == ["no success", "failure event", None, None]
    assert [episode.success for episode in episodes] == [False, False, True, True]
    assert env.reset_calls == 4
    assert len(transitions) == 2
    assert bool(transitions[0]["done"]) and float(transitions[0]["reward"]) == 1.0


def test_successful_episode_without_intervention_is_not_kept():
    env = RecordingEnv("joint_abs")
    readings = [
        SYNC, _reading(intervened=False, events=("success",), episode_end=True),
        SYNC, _reading(events=("success",), episode_end=True),
    ]
    transitions, episodes = _record(_wrap(env, readings))
    assert [episode.discarded_reason for episode in episodes] == ["no intervened steps", None]
    assert len(transitions) == 1


def test_teleop_event_reward_stores_the_success_step_as_a_rewarded_terminal_transition():
    # The env never pays a reward: the success key alone gives the demo its reward and done.
    env = RecordingEnv("joint_abs")
    readings = [SYNC, _reading(Q1), _reading(Q2, events=("success",)), _reading(Q2)]
    transitions, episodes = _record(TeleopEventRewardWrapper(_wrap(env, readings)))

    assert [(episode.steps, episode.discarded_reason) for episode in episodes] == [(2, None)]
    assert [float(transition["reward"]) for transition in transitions] == [0.0, 1.0]
    assert [bool(transition["done"]) for transition in transitions] == [False, True]

    # Without the wrapper the same episode stores the env's reward 0 and runs on.
    env = RecordingEnv("joint_abs")
    readings = [SYNC, _reading(Q1), _reading(Q2, events=("success",)), _reading(Q2, episode_end=True)]
    transitions, _ = _record(_wrap(env, readings))
    assert [float(transition["reward"]) for transition in transitions] == [0.0, 0.0, 0.0]
    assert not any(bool(transition["done"]) for transition in transitions)


# -- episode-start leader check ------------------------------------------------------------


def test_refuses_to_start_when_the_leader_is_too_far_from_the_follower():
    env = RecordingEnv("joint_abs")
    far = [0.9, 0.0, 0.0, 0.0, 0.0, 0.0, 0.3]
    with pytest.raises(LeaderFollowerGapError, match=r"0\.9.*start_tolerance_rad = 0\.8"):
        _record(_wrap(env, [_reading(far, intervened=False)]))
    assert env.step_actions == []


def test_start_check_tolerance_is_configurable():
    env = RecordingEnv("joint_abs")
    near = [0.3, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    wrapped = _wrap(env, [_reading(near, intervened=False)])
    env.reset()
    error = check_leader_near_follower(env, wrapped.source, env.action_contract, 0.8)
    np.testing.assert_allclose(error, np.abs(near), atol=1e-6)
    with pytest.raises(LeaderFollowerGapError):
        check_leader_near_follower(env, wrapped.source, env.action_contract, 0.2)


def test_start_check_reads_the_motion_half_of_a_composite_only():
    env = RecordingEnv("joint_abs")
    motion = CountingScriptedSource([SYNC], action_mode="joint_abs", arm_action_dim=7)
    events = CountingScriptedSource([IDLE], action_mode="joint_abs", arm_action_dim=7)
    check_leader_near_follower(env, CompositeTeleopSource(motion, events), env.action_contract, 0.8)
    assert (motion.polls, events.polls) == (1, 0)


@pytest.fixture
def make_part():
    parts = []

    def make(script):
        part = connected_joint_part(script)
        parts.append(part)
        return part

    yield make
    for part in parts:
        part.disconnect()


def _leader_source(part, env) -> LeaderArmPartSource:
    contract = env.action_contract
    return LeaderArmPartSource(
        part,
        env_contract=contract,
        joint_limits=contract["joint_limits"],
        joint_scale_rad=contract["joint_scale_rad"],
        last_observation_getter=lambda: env.last_observation,
        ramp_step_rad=0.05,
    )


def test_start_check_sees_the_unramped_leader_while_the_toggle_is_held(make_part):
    env = RecordingEnv("joint_abs")
    far = [0.9, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    wrapped = InterventionWrapper(env, _leader_source(make_part([{"action": far, "active": True}]), env))
    with pytest.raises(LeaderFollowerGapError):
        _record(wrapped)


@pytest.mark.parametrize("keep_syncing_steps, recorded", [(False, 2), (True, 8)])
def test_leader_arm_ramp_steps_are_tagged_and_dropped(make_part, keep_syncing_steps, recorded):
    # 0.32 rad apart: six ramp steps of 0.05 rad (gaps 0.32 ... 0.07), then caught up.
    env = RecordingEnv("joint_abs")
    leader = [0.32, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    script = [
        {"action": leader, "gripper": 1.0, "active": True},  # the episode-start read
        {"action": leader, "active": True, "hold": 7},
        {"action": leader, "active": True, "events": {"success": True, "episode_end": True}},
    ]
    wrapped = InterventionWrapper(env, _leader_source(make_part(script), env))
    transitions, episodes = _record(wrapped, keep_syncing_steps=keep_syncing_steps)

    assert episodes[0].steps == 8 and len(transitions) == recorded
    np.testing.assert_allclose(env.follower, leader, atol=1e-5)


# -- output --------------------------------------------------------------------------------


def _one_demo(env):
    readings = [SYNC, _reading(Q1), _reading(Q2, events=("success",), episode_end=True)]
    wrapped = _wrap(env, readings)
    transitions, _ = _record(wrapped)
    return wrapped, transitions


def test_saved_demos_load_with_their_contract(tmp_path):
    wrapped, transitions = _one_demo(RecordingEnv("joint_abs"))
    contract = demo_contract_of(wrapped)
    path = save_demos(transitions, tmp_path / "demos" / "task.pkl", contract)

    assert contract_sidecar_path(path).exists()
    added = []
    count = load_demo_pkls(
        [str(path)], lambda *columns, **extra: added.append(columns), "cpu", expected_contract=contract
    )
    assert count == len(added) == 2
    np.testing.assert_allclose(added[0][2].reshape(-1).numpy(), transitions[0]["action"].reshape(-1).numpy())

    with open(path, "rb") as stream:
        saved = pickle.load(stream)
    delta_contract = demo_contract_of(RecordingEnv("joint_delta"))
    with pytest.raises(ValueError, match="action_mode"):
        validate_transitions_against_contract(saved, delta_contract, source=str(path), saved=contract)


def test_run_recorder_builds_the_actor_env_saves_and_closes(tmp_path, monkeypatch):
    env = RecordingEnv("joint_abs")
    readings = [SYNC, _reading(Q1, events=("success",), episode_end=True)]
    wrapped = _wrap(env, readings)
    built = {}

    def fake_build_hil_env(args, *, role, **kwargs):
        built.update(args=args, role=role)
        return wrapped

    monkeypatch.setattr(env_building, "build_hil_env", fake_build_hil_env)
    output = tmp_path / "demos.pkl"
    args = _args(output_path=str(output), env_backend="real_robot", max_episode_steps=40)

    assert run_recorder(args) == output
    assert built["role"] == "actor" and built["args"].real_robot.max_episode_steps == 40
    assert output.exists() and contract_sidecar_path(output).exists()
    assert env.close_calls == 1

    with pytest.raises(ValueError, match="exists"):
        run_recorder(args)


def test_run_recorder_saves_the_kept_episodes_when_interrupted(tmp_path, monkeypatch):
    env = RecordingEnv("joint_abs")
    readings = [SYNC, _reading(Q1, events=("success",), episode_end=True)]
    wrapped = _wrap(env, readings)
    monkeypatch.setattr(env_building, "build_hil_env", lambda args, **kwargs: wrapped)
    calls = {"reset": 0}
    original_reset = wrapped.reset

    def reset_then_interrupt(**kwargs):
        calls["reset"] += 1
        if calls["reset"] == 2:
            raise KeyboardInterrupt
        return original_reset(**kwargs)

    monkeypatch.setattr(wrapped, "reset", reset_then_interrupt)
    output = tmp_path / "demos.pkl"
    with pytest.raises(KeyboardInterrupt):
        run_recorder(_args(output_path=str(output), num_successes=3))
    with open(output, "rb") as stream:
        assert len(pickle.load(stream)) == 1
    assert env.close_calls == 1


def test_output_path_must_be_a_new_pkl(tmp_path):
    with pytest.raises(ValueError, match="required"):
        run_recorder(_args())
    with pytest.raises(ValueError, match=r"\.pkl"):
        run_recorder(_args(output_path=str(tmp_path / "demos.h5")))


# -- example CLI ---------------------------------------------------------------------------


@pytest.fixture
def example():
    module_name = "_example_record_real_demos"
    spec = importlib.util.spec_from_file_location(module_name, EXAMPLES / "record_real_demos.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    try:
        spec.loader.exec_module(module)
        yield module
    finally:
        sys.modules.pop(module_name, None)


def test_example_parses_the_hil_env_flags_over_a_training_preset(example, monkeypatch, tmp_path):
    preset = tmp_path / "hil.yaml"
    preset.write_text(
        "env_backend: real_robot\n"
        "batch_size: 256\n"  # an algorithm field of the training run: skipped
        "teleop_onset: ramp\n"
        "real_robot:\n"
        "  action_mode: joint_abs\n"
        "  joint_limits: [[-1.0, 1.0], [-1.0, 1.0], [-1.0, 1.0], [-1.0, 1.0], [-1.0, 1.0], [-1.0, 1.0], [-1.0, 1.0]]\n",
        encoding="utf-8",
    )
    received = []
    monkeypatch.setattr("rl_garden_real_world.demos.__main__.run_recorder", received.append)
    example.main(
        [
            "--config", str(preset),
            "--teleop-device", "gello",
            "--teleop-event-device", "keyboard",
            "--teleop-onset", "arm",
            "--output-path", "demos/task.pkl",
            "--num-successes", "3",
            "--max-episode-steps", "40",
            "--start-tolerance-rad", "0.5",
            "--keep-syncing-steps",
        ]
    )

    (args,) = received
    assert isinstance(args, RecordDemosArgs)
    assert (args.env_backend, args.real_robot.action_mode) == ("real_robot", "joint_abs")
    assert args.real_robot.joint_limits == [[-1.0, 1.0]] * 7
    assert (args.teleop_device, args.teleop_event_device, args.teleop_onset) == ("gello", "keyboard", "arm")
    assert (args.output_path, args.num_successes, args.max_episode_steps) == ("demos/task.pkl", 3, 40)
    assert args.start_tolerance_rad == 0.5 and args.keep_syncing_steps
    assert args.eval_freq == 0


def test_example_parses_the_two_arm_flags_over_a_training_preset(example, monkeypatch, tmp_path):
    limits = "[[-1.0, 1.0], [-1.0, 1.0], [-1.0, 1.0], [-1.0, 1.0], [-1.0, 1.0], [-1.0, 1.0], [-1.0, 1.0]]"
    preset = tmp_path / "bimanual.yaml"
    preset.write_text(
        "env_backend: real_robot_bimanual\n"
        "batch_size: 256\n"  # an algorithm field of the training run: skipped
        "real_robot_bimanual:\n"
        "  left:\n"
        "    action_mode: joint_abs\n"
        f"    joint_limits: {limits}\n"
        "  right:\n"
        "    action_mode: joint_abs\n"
        f"    joint_limits: {limits}\n",
        encoding="utf-8",
    )
    received = []
    monkeypatch.setattr("rl_garden_real_world.demos.__main__.run_recorder", received.append)
    example.main(
        [
            "--config", str(preset),
            "--teleop-device", "bimanual",
            "--teleop-arm-device", "leader_arm",
            "--robot-teleop-action-mode", "joint_abs",
            "--teleop-event-device", "keyboard",
            "--real_robot_bimanual.left.remote", "10.0.0.2:50051",
            "--real_robot_bimanual.right.remote", "10.0.0.3:50052",
            "--output-path", "demos/bimanual.pkl",
            "--max-episode-steps", "40",
        ]
    )

    (args,) = received
    assert args.env_backend == "real_robot_bimanual"
    assert (args.teleop_device, args.teleop_arm_device, args.robot_teleop_action_mode) == (
        "bimanual",
        "leader_arm",
        "joint_abs",
    )
    bimanual = args.real_robot_bimanual
    assert (bimanual.left.remote, bimanual.right.remote) == ("10.0.0.2:50051", "10.0.0.3:50052")
    assert bimanual.left.action_mode == bimanual.right.action_mode == "joint_abs"
    assert bimanual.right.joint_limits == [[-1.0, 1.0]] * 7
    assert record._with_episode_step_cap(args).real_robot_bimanual.max_episode_steps == 40
    kwargs = env_building.teleop_wrapper_kwargs(args)
    assert kwargs["motion"] == "bimanual" and kwargs["motion_kwargs"]["device_per_arm"] == "leader_arm"


def test_example_parses_the_teleop_event_reward_flags(example, monkeypatch):
    received = []
    monkeypatch.setattr("rl_garden_real_world.demos.__main__.run_recorder", received.append)
    example.main(["--output-path", "demos/task.pkl", "--reward-from-teleop-events", "--teleop-success-reward", "2.0"])
    (args,) = received
    assert args.reward_from_teleop_events and args.teleop_success_reward == 2.0


def test_recorder_defaults():
    args = record.RecordDemosArgs()
    assert (args.start_tolerance_rad, args.keep_syncing_steps, args.max_episode_steps) == (0.8, False, None)
    # Unlike the training entrypoints, the recorder rewards the success key by default.
    assert args.reward_from_teleop_events and args.teleop_success_reward == 1.0
    assert args.output_path is None and args.env_backend == "franka_real"


def test_example_turns_the_teleop_event_reward_off_with_the_negated_flag(example, monkeypatch):
    received = []
    monkeypatch.setattr("rl_garden_real_world.demos.__main__.run_recorder", received.append)
    example.main(["--output-path", "demos/task.pkl", "--no-reward-from-teleop-events"])
    (args,) = received
    assert not args.reward_from_teleop_events
