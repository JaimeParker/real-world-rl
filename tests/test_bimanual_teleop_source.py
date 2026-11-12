"""BimanualTeleopSource: assembly of two single-arm sources into the two-arm action layout, the
"any"/"all" intervention rule and the hold of an idle arm, event merging, toggle forwarding,
the per-arm gripper views, declaration checks, and ``from_env`` over the two-arm env with fake
robots whose teleop parts are scripted per arm."""
from __future__ import annotations

import numpy as np
import pytest
import torch
from _bimanual_fakes import ARMS, bimanual_config, fake_franka
from _teleop_fakes import TwoArmContractEnv

import real_world_rl.devices.claims as claims
from real_world_rl.devices.teleop import TeleopConfig

from real_world_rl.envs.bimanual import BimanualRealRobotEnv, BimanualRealRobotVectorEnv
from real_world_rl.envs.wrappers.intervention import (
    InterventionWrapper,
    TeleopInterventionWrapper,
)
from real_world_rl.teleop import TeleopReading, make_teleop_source
from real_world_rl.teleop.core.bimanual import (
    BimanualTeleopSource,
    find_bimanual_env,
    normalized_joint_target,
)
from real_world_rl.teleop.core.composite import (
    CompositeTeleopSource,
    ToggleableTeleopSource,
)
from real_world_rl.teleop.core.fake import ScriptedTeleopSource
from real_world_rl.teleop.leader.joint import LeaderArmPartSource

CART = 6
LEFT_ARM = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
RIGHT_ARM = [-0.1, -0.2, -0.3, -0.4, -0.5, -0.6]


def _reading(arm, gripper=None, *, intervened=True, episode_end=False, events=(), timestamp=0.0):
    action = list(arm) + ([] if gripper is None else [gripper])
    return TeleopReading(
        action=np.asarray(action, dtype=np.float32),
        intervened=intervened,
        episode_end=episode_end,
        events={name: True for name in events},
        timestamp=timestamp,
    )


def _scripted(*readings, action_mode="cartesian_delta", arm_action_dim=CART, has_gripper=True):
    return ScriptedTeleopSource(
        list(readings), action_mode=action_mode, arm_action_dim=arm_action_dim, has_gripper=has_gripper
    )


def _two_arms(left, right, **kwargs) -> BimanualTeleopSource:
    return BimanualTeleopSource({"left": left, "right": right}, **kwargs)


# -- assembly ------------------------------------------------------------------------------


def test_arm_blocks_come_first_in_arm_order_then_one_gripper_per_arm():
    source = _two_arms(_scripted(_reading(LEFT_ARM, -1.0)), _scripted(_reading(RIGHT_ARM, 0.5)))
    reading = source.poll()

    assert (source.num_arms, source.arm_names) == (2, ["left", "right"])
    assert (source.action_mode, source.arm_action_dim, source.has_gripper) == ("cartesian_delta", CART, True)
    np.testing.assert_allclose(reading.action, LEFT_ARM + RIGHT_ARM + [-1.0, 0.5], atol=1e-6)
    assert reading.action.dtype == np.float32 and reading.intervened is True


def test_gripperless_children_are_padded_with_each_arms_held_gripper():
    held = torch.tensor([-1.0, 0.25])
    source = _two_arms(
        _scripted(_reading(LEFT_ARM), has_gripper=False),
        _scripted(_reading(RIGHT_ARM), has_gripper=False),
        last_gripper_action_getter=lambda: held,
    )
    assert source.has_gripper is True
    np.testing.assert_allclose(source.poll().action, LEFT_ARM + RIGHT_ARM + [-1.0, 0.25], atol=1e-6)


def test_gripperless_children_without_a_getter_emit_no_gripper_dims():
    source = _two_arms(
        _scripted(_reading(LEFT_ARM), has_gripper=False), _scripted(_reading(RIGHT_ARM), has_gripper=False)
    )
    assert source.has_gripper is False
    np.testing.assert_allclose(source.poll().action, LEFT_ARM + RIGHT_ARM, atol=1e-6)


def test_a_reading_of_the_wrong_width_names_the_arm():
    source = _two_arms(_scripted(_reading(LEFT_ARM, 1.0)), _scripted(_reading(RIGHT_ARM)))
    with pytest.raises(ValueError, match="arm 'right'"):
        source.poll()


# -- intervention rule and the hold of an idle arm -------------------------------------------


def test_any_intervenes_when_one_arm_does_and_holds_the_idle_arm_and_its_gripper():
    held = torch.tensor([0.5, -1.0])
    source = _two_arms(
        _scripted(_reading(LEFT_ARM, 1.0, intervened=True)),
        # An idle leader with its trigger released: neither its arm nor its trigger may act.
        _scripted(_reading(RIGHT_ARM, 1.0, intervened=False)),
        last_gripper_action_getter=lambda: held,
    )
    reading = source.poll()
    assert reading.intervened is True
    np.testing.assert_allclose(reading.action, LEFT_ARM + [0.0] * CART + [1.0, -1.0], atol=1e-6)


def test_idle_arm_keeps_its_childs_gripper_without_a_getter():
    source = _two_arms(
        _scripted(_reading(LEFT_ARM, 1.0, intervened=True)),
        _scripted(_reading(RIGHT_ARM, -1.0, intervened=False)),
    )
    np.testing.assert_allclose(source.poll().action, LEFT_ARM + [0.0] * CART + [1.0, -1.0], atol=1e-6)


def test_all_intervenes_only_when_every_arm_does():
    def make(right_intervened: bool) -> BimanualTeleopSource:
        return _two_arms(
            _scripted(_reading(LEFT_ARM, 1.0, intervened=True)),
            _scripted(_reading(RIGHT_ARM, 1.0, intervened=right_intervened)),
            intervention_from="all",
        )

    assert make(False).poll().intervened is False
    reading = make(True).poll()
    assert reading.intervened is True
    np.testing.assert_allclose(reading.action, LEFT_ARM + RIGHT_ARM + [1.0, 1.0], atol=1e-6)


def test_no_arm_intervening_leaves_the_childrens_actions():
    source = _two_arms(
        _scripted(_reading(LEFT_ARM, 1.0, intervened=False)),
        _scripted(_reading(RIGHT_ARM, 1.0, intervened=False)),
    )
    reading = source.poll()
    assert reading.intervened is False
    np.testing.assert_allclose(reading.action, LEFT_ARM + RIGHT_ARM + [1.0, 1.0], atol=1e-6)


def test_joint_abs_holds_an_idle_arm_with_its_hold_action():
    joints = 7
    right_hold = [0.25] * joints

    def joint(*readings):
        return _scripted(*readings, action_mode="joint_abs", arm_action_dim=joints)

    with pytest.raises(ValueError, match="arm_hold_actions"):
        _two_arms(joint(_reading([0.0] * joints, 1.0)), joint(_reading([0.0] * joints, 1.0)))

    source = _two_arms(
        joint(_reading([0.5] * joints, 1.0, intervened=True)),
        joint(_reading([-0.9] * joints, 1.0, intervened=False)),
        arm_hold_actions={"left": lambda: [0.0] * joints, "right": lambda: right_hold},
    )
    np.testing.assert_allclose(source.poll().action, [0.5] * joints + right_hold + [1.0, 1.0], atol=1e-6)
    # "all" never holds an arm, so it needs no hold actions.
    _two_arms(joint(_reading([0.0] * joints, 1.0)), joint(_reading([0.0] * joints, 1.0)), intervention_from="all")


def test_normalized_joint_target_maps_onto_the_limits_and_clips():
    limits = [[-1.0, 3.0], [-2.0, 2.0]]
    np.testing.assert_allclose(normalized_joint_target([0.0, 0.0], limits), [-0.5, 0.0])
    np.testing.assert_allclose(normalized_joint_target([5.0, -3.0], limits), [1.0, -1.0])


# -- events --------------------------------------------------------------------------------


def test_events_are_merged_per_arm_and_as_any_and_episode_end_is_ored():
    source = _two_arms(
        _scripted(_reading(LEFT_ARM, 1.0, events=("success",), timestamp=1.0)),
        _scripted(_reading(RIGHT_ARM, 1.0, episode_end=True, events=("teleop_syncing",), timestamp=2.0)),
    )
    reading = source.poll()
    assert reading.episode_end is True and reading.timestamp == 2.0
    assert dict(reading.events) == {
        "left_success": True,
        "left.success": True,
        "success": True,
        "right_teleop_syncing": True,
        "right.teleop_syncing": True,
        "teleop_syncing": True,
    }


def test_no_events_and_no_episode_end_when_no_arm_fired():
    reading = _two_arms(_scripted(_reading(LEFT_ARM, 1.0)), _scripted(_reading(RIGHT_ARM, 1.0))).poll()
    assert reading.episode_end is False and dict(reading.events) == {}


# -- toggle forwarding ---------------------------------------------------------------------


class _ToggleRecorder(ToggleableTeleopSource):
    """Intervenes exactly when toggled; records every toggle it was handed."""

    action_mode = "cartesian_delta"
    arm_action_dim = CART
    has_gripper = True

    def __init__(self, arm) -> None:
        self.arm = list(arm)
        self.toggles: list[bool] = []
        self.polls = 0

    def poll(self) -> TeleopReading:
        self.polls += 1
        return _reading(self.arm, 1.0, intervened=False)

    def poll_with_toggle(self, toggle: bool) -> TeleopReading:
        self.toggles.append(toggle)
        return _reading(self.arm, 1.0, intervened=toggle)


def test_poll_with_toggle_reaches_every_toggleable_arm_and_drives_the_others():
    left = _ToggleRecorder(LEFT_ARM)
    right = _scripted(_reading(RIGHT_ARM, 1.0, intervened=False))
    source = _two_arms(left, right)

    reading = source.poll_with_toggle(True)
    assert left.toggles == [True] and reading.intervened is True
    # The non-toggleable arm takes the toggle as its own flag: it is not held.
    np.testing.assert_allclose(reading.action, LEFT_ARM + RIGHT_ARM + [1.0, 1.0], atol=1e-6)

    assert source.poll_with_toggle(False).intervened is False
    assert left.toggles == [True, False] and left.polls == 0


def test_one_event_device_toggles_both_arms_through_a_composite():
    left, right = _ToggleRecorder(LEFT_ARM), _ToggleRecorder(RIGHT_ARM)
    keyboard = _scripted(_reading([0.0] * CART, 0.0, intervened=True, events=("success",)))
    composite = CompositeTeleopSource(_two_arms(left, right), keyboard)

    reading = composite.poll()
    assert left.toggles == [True] and right.toggles == [True]
    assert reading.intervened is True and reading.events["success"] is True
    np.testing.assert_allclose(reading.action, LEFT_ARM + RIGHT_ARM + [1.0, 1.0], atol=1e-6)


def test_intervention_wrapper_reads_the_arm_layout_of_a_composites_motion_half():
    left, right = _ToggleRecorder(LEFT_ARM), _ToggleRecorder(RIGHT_ARM)
    keyboard = _scripted(_reading([0.0] * CART, 0.0, intervened=True))
    wrapped = InterventionWrapper(TwoArmContractEnv(), CompositeTeleopSource(_two_arms(left, right), keyboard))
    _, _, _, _, info = wrapped.step(torch.zeros(1, 14))
    assert info["intervened"] is True and info["intervene_action"].shape == (1, 14)


def test_intervention_wrapper_rejects_arms_named_or_ordered_differently_from_the_env():
    swapped = BimanualTeleopSource(
        {"right": _scripted(_reading(RIGHT_ARM, 1.0)), "left": _scripted(_reading(LEFT_ARM, 1.0))}
    )
    with pytest.raises(ValueError, match=r"arms \['right', 'left'\].*\['left', 'right'\]"):
        InterventionWrapper(TwoArmContractEnv(), swapped)


# -- declaration checks and fan-out ----------------------------------------------------------


def test_children_must_declare_the_same_action_space():
    with pytest.raises(ValueError, match="same action space"):
        _two_arms(_scripted(_reading(LEFT_ARM, 1.0)), _scripted(_reading([0.0] * 7, 1.0), arm_action_dim=7))
    with pytest.raises(ValueError, match="same action space"):
        _two_arms(_scripted(_reading(LEFT_ARM, 1.0)), _scripted(_reading(RIGHT_ARM), has_gripper=False))


def test_every_child_must_drive_one_arm():
    two_armed = ScriptedTeleopSource([_reading([0.0] * 14)], num_arms=2)
    with pytest.raises(ValueError, match="drive one arm"):
        _two_arms(_scripted(_reading(LEFT_ARM, 1.0)), two_armed)


def test_bad_intervention_from_and_hold_action_names_are_rejected():
    def children():
        return _scripted(_reading(LEFT_ARM, 1.0)), _scripted(_reading(RIGHT_ARM, 1.0))

    with pytest.raises(ValueError, match="intervention_from"):
        _two_arms(*children(), intervention_from="either")
    with pytest.raises(ValueError, match="arm_hold_actions"):
        _two_arms(*children(), arm_hold_actions={"left": lambda: [0.0] * CART})
    with pytest.raises(ValueError, match="none"):
        BimanualTeleopSource({})


class _FailingStart(ScriptedTeleopSource):
    def start(self) -> None:
        raise OSError("bus not found")


def test_start_reset_and_close_reach_every_arm():
    left, right = _scripted(_reading(LEFT_ARM, 1.0)), _scripted(_reading(RIGHT_ARM, 1.0))
    source = _two_arms(left, right)
    source.start()
    source.reset()
    source.close()
    assert [(child.start_calls, child.reset_calls, child.close_calls) for child in (left, right)] == [(1, 1, 1)] * 2


def test_a_failed_start_closes_the_arms_already_started():
    left = _scripted(_reading(LEFT_ARM, 1.0))
    right = _FailingStart([_reading(RIGHT_ARM, 1.0)])
    with pytest.raises(OSError, match="bus not found"):
        _two_arms(left, right).start()
    assert (left.start_calls, left.close_calls) == (1, 1)


class _FailingClose(ScriptedTeleopSource):
    def close(self) -> None:
        super().close()
        raise OSError("close failed")


def test_close_reaches_every_arm_before_raising():
    left = _FailingClose([_reading(LEFT_ARM, 1.0)])
    right = _scripted(_reading(RIGHT_ARM, 1.0))
    with pytest.raises(OSError, match="close failed"):
        _two_arms(left, right).close()
    assert right.close_calls == 1


# -- the fake device on two arms -------------------------------------------------------------


def test_fake_device_declares_the_envs_arm_count_and_a_full_width_idle_reading():
    source = make_teleop_source("fake", TwoArmContractEnv())
    assert source.num_arms == 2 and source.poll().action.shape == (14,)
    wrapped = InterventionWrapper(TwoArmContractEnv(), source)
    assert wrapped.step(torch.zeros(1, 14))[4]["intervened"] is False


# -- from_env over the two-arm env --------------------------------------------------------------

JOINTS = 7
JOINT_SCALE_RAD = 0.05
LEFT_Q = [0.02, -0.01, 0.0, 0.0, 0.0, 0.0, 0.01]
RIGHT_Q = [0.3, 0.3, 0.0, 0.0, 0.0, 0.0, 0.0]


@pytest.fixture(autouse=True)
def _claim_dir(tmp_path, monkeypatch):
    monkeypatch.setattr(claims, "_DEFAULT_LOCK_DIR", tmp_path / "claims")


def leader_part(script, *, has_gripper=True) -> TeleopConfig:
    """A fake joint_abs leader (a GELLO stand-in) as a robot's teleop part."""
    options = {"action_space": "joint_abs", "num_joints": JOINTS, "has_gripper": has_gripper, "script": script}
    return TeleopConfig(name="leader", backend="fake", options=options)


@pytest.fixture
def bimanual_env():
    """``make(scripts, **teleop_kwargs)`` -> ``(BimanualRealRobotEnv, intervention wrapper)``: two
    fake Frankas in ``joint_delta``, each with a scripted leader part, read with
    ``device="bimanual"``. ``scripts`` maps an arm to its ``(script, has_gripper)``."""
    wrappers = []

    def make(scripts, **teleop_kwargs):
        config = bimanual_config(shared={"action_mode": "joint_delta", "control_hz": 1000.0})
        robots = {
            arm: fake_franka(arm, teleop=leader_part(script, has_gripper=has_gripper))
            for arm, (script, has_gripper) in scripts.items()
        }
        env = BimanualRealRobotEnv(config, robots=robots)
        wrapped = TeleopInterventionWrapper(BimanualRealRobotVectorEnv(env), device="bimanual", **teleop_kwargs)
        wrappers.append(wrapped)
        return env, wrapped

    yield make
    for wrapped in wrappers:
        wrapped.close()


def _joint_delta(leader, follower) -> np.ndarray:
    return np.clip((np.asarray(leader) - np.asarray(follower, dtype=np.float64)) / JOINT_SCALE_RAD, -1.0, 1.0)


def test_from_env_builds_one_leader_arm_child_per_arm_over_that_arms_env(bimanual_env):
    env, wrapped = bimanual_env(
        {
            "left": ([{"action": LEFT_Q, "gripper": 0.0, "active": True}], True),
            # The right operator is not intervening; their trigger is squeezed.
            "right": ([{"action": RIGHT_Q, "gripper": 0.0, "active": False}], True),
        },
        common_kwargs={"onset": "ramp"},
    )
    source = wrapped.source
    assert isinstance(source, BimanualTeleopSource) and source.arm_names == list(ARMS)
    assert all(isinstance(child, LeaderArmPartSource) for child in source.sources.values())
    assert find_bimanual_env(wrapped) is env

    wrapped.reset()
    left_follower = env.arms["left"].last_observation["state"]["joint_pos"]
    _, _, _, _, info = wrapped.step(torch.zeros(1, 2 * JOINTS + 2))

    assert info["intervened"] is True
    executed = info["intervene_action"].numpy()[0]
    np.testing.assert_allclose(executed[:JOINTS], _joint_delta(LEFT_Q, left_follower), atol=1e-5)
    np.testing.assert_allclose(executed[JOINTS : 2 * JOINTS], np.zeros(JOINTS))  # held
    # Left: its trigger (2 * 0 - 1); right: held at the gripper action after the reset (open).
    np.testing.assert_allclose(executed[-2:], [-1.0, 1.0])


def test_from_env_hands_each_gripperless_arm_its_own_held_gripper(bimanual_env):
    script = [{"action": [0.0] * JOINTS, "active": False}, {"action": [0.0] * JOINTS, "active": True}]
    _, wrapped = bimanual_env({"left": (script, False), "right": (script, False)})
    wrapped.reset()
    policy = torch.zeros(1, 2 * JOINTS + 2)
    policy[0, -2:] = torch.tensor([-1.0, -0.5])
    wrapped.step(policy)  # nobody intervenes: the policy sets both grippers

    _, _, _, _, info = wrapped.step(torch.zeros(1, 2 * JOINTS + 2))
    assert info["intervened"] is True
    np.testing.assert_allclose(info["intervene_action"].numpy()[0, -2:], [-1.0, -0.5])


def test_from_env_rejects_unsupported_devices_unknown_arms_and_single_arm_envs(bimanual_env):
    script = [{"action": [0.0] * JOINTS, "active": False}]
    env, wrapped = bimanual_env({"left": (script, True), "right": (script, True)})
    with pytest.raises(ValueError, match="leader_cartesian"):
        BimanualTeleopSource.from_env(wrapped.env, device_per_arm="leader_cartesian")
    with pytest.raises(ValueError, match=r"unknown arms \['middle'\]"):
        BimanualTeleopSource.from_env(wrapped.env, per_arm_kwargs={"middle": {}})
    with pytest.raises(ValueError, match="real_robot_bimanual"):
        BimanualTeleopSource.from_env(TwoArmContractEnv())
