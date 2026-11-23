"""Shared operator frame math and device behavior, without hardware."""

from _station_limits import station_limits, env_limits, tcp_box
import logging

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from _teleop_fakes import HOME_TCP
from test_pico_teleop import _source as pico_source, _take_over, _poll
from test_spacemouse_teleop_source import _Part, _Clock, NO_GRIPPER

from real_world_rl.teleop.core.input_frame import (
    resolve_input_rotation, rotate_pose, rotate_twist, task_frame_rotation,
)
from real_world_rl.teleop.core.registry import source_class
from real_world_rl.teleop.core.targets.twist_input import rotate_input
from real_world_rl.teleop.spacemouse.source import SpaceMouseTeleopSource
from real_world_rl.teleop.spacemouse.serl import SpaceMouseSerlTeleopSource


RX = Rotation.from_euler("x", 45, degrees=True).as_matrix()
RZ = Rotation.from_euler("z", 90, degrees=True).as_matrix()


def task_frame():
    transform = np.eye(4)
    transform[:3, :3] = RX
    transform[:3, 3] = [0.2, -0.1, 0.3]
    return {"name": "table", "base_T_frame": transform.tolist()}


def test_rotation_precedence_and_labels():
    assert resolve_input_rotation() == (None, "base")
    rotation, label = resolve_input_rotation(task_frame_rotation=RX)
    np.testing.assert_array_equal(rotation, RX)
    assert label == "task_frame"
    rotation, label = resolve_input_rotation(device_rotation=RZ, task_frame_rotation=RX)
    np.testing.assert_array_equal(rotation, RZ)
    assert label == "device_profile"
    rotation[0, 0] = 100
    assert RZ[0, 0] != 100  # validated rotations are independent copies
    with pytest.raises(ValueError, match="input_rotation"):
        resolve_input_rotation(device_rotation=RZ, task_frame_rotation=np.zeros((3, 3)))


def test_task_frame_rotation_extracts_only_valid_rotation():
    assert task_frame_rotation(None) is None
    np.testing.assert_array_equal(task_frame_rotation(task_frame()), RX)
    for matrix in (np.eye(3), np.full((4, 4), np.nan), np.diag([2, 1, 1, 1])):
        with pytest.raises(ValueError):
            task_frame_rotation({"name": "table", "base_T_frame": matrix})


@pytest.mark.parametrize("rotation", [None, RX, RZ])
def test_rotate_twist_compatibility_and_cube_normalization(rotation):
    twist = np.array([0.2, 1., 1., -0.2, 1., -1.])
    original = twist.copy()
    result = rotate_twist(twist, rotation)
    np.testing.assert_array_equal(result, rotate_input(twist, rotation))
    if rotation is None:
        assert result is twist
    else:
        for start in (0, 3):
            vector = rotation @ twist[start:start + 3]
            np.testing.assert_allclose(
                result[start:start + 3], vector / max(1., np.max(np.abs(vector))))
    np.testing.assert_array_equal(twist, original)


def test_rotate_pose_math():
    position = np.array([0.3, -0.4, 0.9])
    p, r = rotate_pose(position, RZ, RX)
    np.testing.assert_allclose(p, RX @ position)
    np.testing.assert_allclose(r, RX @ RZ)
    p, r = rotate_pose(position, RZ, None)
    assert p is position and r is RZ


def test_spacemouse_tracker_and_serl_default_rotate_positive_z():
    part = _Part()
    tracker = SpaceMouseTeleopSource(
        part, follower_pose_getter=lambda: HOME_TCP.copy(), env_contract=NO_GRIPPER,
        clock=_Clock(), task_frame_rotation=RX,
    )
    tracker.reset()
    part.press(0)
    tracker.poll()
    part.twist = np.array([0., 0., 1., 0., 0., 0.])
    reading = tracker.poll()
    np.testing.assert_allclose(
        reading.diagnostics["base_input_after_filter"][:3], RX @ [0, 0, 1])
    delta = tracker.cartesian_tracker.command[:3] - HOME_TCP[:3]
    np.testing.assert_allclose(delta / np.linalg.norm(delta), RX @ [0, 0, 1], atol=1e-8)

    serl = SpaceMouseSerlTeleopSource(
        part, env_contract=NO_GRIPPER, task_frame_rotation=RX, input_rotation=RZ,
    )
    np.testing.assert_allclose(serl.poll().action[:3], RX @ [0, 0, 1], atol=1e-7)


def test_serl_explicit_profile_without_matrix_keeps_base_behavior():
    part = _Part()
    part.twist = np.array([0., 0., 1., 0., 0., 0.])
    source = SpaceMouseSerlTeleopSource(
        part, env_contract=NO_GRIPPER, task_frame_rotation=RX, use_profile_rotation=True,
    )
    np.testing.assert_array_equal(source.poll().action, part.twist)
    spec = source.control_spec(
        tracker_options=None,
        source_options=lambda: {"task_frame_rotation": RX, "use_profile_rotation": True},
    )
    assert spec["input_frame"] == "profile"


@pytest.mark.parametrize("explicit,frame,label", [
    (None, True, "task_frame"),
    (RZ, True, "device_profile"),
    (None, False, "base"),
])
def test_pico_frame_label_and_hand_displacement(explicit, frame, label, caplog):
    source, part, clock = pico_source(
        input_rotation=explicit, task_frame=task_frame() if frame else None,
    )
    assert source.input_frame == label
    with pytest.raises(AttributeError):
        source.input_frame = "base"
    with caplog.at_level(logging.INFO):
        source.start()
    message = "pico: no input_rotation; driving in task frame table"
    assert caplog.messages.count(message) == (1 if label == "task_frame" else 0)
    _take_over(source, part, clock)
    part.pose[:3] += [0., 0., .004]
    _poll(source, clock)
    expected_rotation = explicit if explicit is not None else RX if frame else np.eye(3)
    np.testing.assert_allclose(
        source.cartesian_tracker.command[:3] - HOME_TCP[:3],
        expected_rotation @ [0., 0., .004], atol=1e-9,
    )


@pytest.mark.parametrize("device,options,expected", [
    ("spacemouse", {}, "twist"),
    ("spacemouse", {"mode": "serl"}, "twist"),
    ("pico", {}, "pose"),
    ("gello", {}, None),
    ("leader_arm", {}, None),
    ("keyboard", {}, None),
    ("stdin", {}, None),
])
def test_input_frame_capabilities(device, options, expected):
    assert source_class(device, options).INPUT_FRAME == expected


def test_pico_from_env_uses_workspace_task_frame():
    from real_world_rl.envs.real_robot import RealRobotEnv, RealRobotEnvConfig
    from real_world_rl.teleop.pico.source import PicoTeleopSource
    from test_pico_teleop import _PicoPart, _Clock as PicoClock

    # No reset/connect: exercise the actual env config -> workspace kwargs path.
    env = RealRobotEnv(
        RealRobotEnvConfig(
            robot={"limits": station_limits(), "type": "franka"}, task_frame=task_frame(),
            action_mode="cartesian_delta",
        ),
        robot=object(),
    )
    part, clock = _PicoPart(), PicoClock()
    source = PicoTeleopSource.from_env(
        env, teleop=part, clock=clock, follower_pose_getter=lambda: HOME_TCP.copy(),
        limits=None,
    )
    assert source.input_frame == "task_frame"
    source.reset()
    _take_over(source, part, clock)
    part.pose[:3] += [0., 0., .004]
    _poll(source, clock)
    np.testing.assert_allclose(
        source.cartesian_tracker.command[:3] - HOME_TCP[:3],
        RX @ [0., 0., .004], atol=1e-9,
    )
