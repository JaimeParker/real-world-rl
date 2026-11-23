"""Task-frame helpers remain available to operator input consumers."""
import numpy as np
import pytest
from real_world_rl.devices.workspace import task_transform, quaternion_matrix


def test_task_transform_and_quaternion_sign():
    frame = {"name": "table", "base_T_frame": [[1, 0, 0, .5], [0, .8, -.6, 0],
                                                [0, .6, .8, .2], [0, 0, 0, 1]]}
    np.testing.assert_allclose(task_transform(frame), frame["base_T_frame"])
    np.testing.assert_array_equal(task_transform(None), np.eye(4))
    np.testing.assert_array_equal(quaternion_matrix([0, 0, 0, 1]), quaternion_matrix([0, 0, 0, -1]))
    frame["base_T_frame"][0][0] = -1
    with pytest.raises(ValueError, match="proper rigid"):
        task_transform(frame)


@pytest.mark.parametrize("frame", [{}, {"name": "base"}, {"unknown": 1},
                                   {"name": "table", "base_T_frame": [[0]*4]*4}])
def test_invalid_task_frames(frame):
    with pytest.raises(ValueError):
        task_transform(frame)
