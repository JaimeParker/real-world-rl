import numpy as np
import pytest

from real_world_rl.teleop.calibration import fit_yaw, input_rotation, yaw_rotation


def samples(yaw):
    result = {}
    for label, vector in {"+x": [1, 0, 0], "+y": [0, 1, 0],
                          "-x": [-1, 0, 0], "-y": [0, -1, 0]}.items():
        v = yaw_rotation(yaw).T @ vector
        result[label] = [list(v * .5) + [0, 0, 0]] * 5
    return result


@pytest.mark.parametrize("yaw", [0, .37, -1.2, np.pi / 2, np.pi, -np.pi / 2])
def test_fit_and_mounting_composition(yaw):
    result = fit_yaw(samples(yaw))
    np.testing.assert_allclose(yaw_rotation(result["yaw_rad"]), yaw_rotation(yaw), atol=1e-12)
    mounting = np.array([[1, 0, 0], [0, .8, -.6], [0, .6, .8]])
    np.testing.assert_allclose(input_rotation(mounting, result["yaw_rad"]), mounting @ yaw_rotation(yaw), atol=1e-12)
    assert not result["rotation_validated"]


def test_negative_labels_are_independent_validation():
    data = samples(.7)
    data["-x"] = data["+x"]
    with pytest.raises(ValueError, match="direction error"):
        fit_yaw(data)


@pytest.mark.parametrize("bad", [[], [[0] * 6], [[float("nan")] * 6], [[2] * 6]])
def test_bad_or_weak_samples_rejected(bad):
    data = samples(.4)
    data["+x"] = bad
    with pytest.raises(ValueError):
        fit_yaw(data)


def test_repeated_trials_equal_weight_not_duration_or_strength():
    data = samples(0)
    short = samples(.2)['+x'][:1]
    long = samples(-.2)['+x'] * 100
    data['+x'] = {'trials': [short, long]}
    assert fit_yaw(data)['yaw_rad'] == pytest.approx(0, abs=1e-12)
    assert fit_yaw(data)['directions']['+x']['trial_count'] == 2


def test_bad_trial_cannot_hide_in_good_group():
    data = samples(0)
    data['-x'] = {'trials': [data['-x'] * 100, data['+x']]}
    with pytest.raises(ValueError, match='direction error'):
        fit_yaw(data)


def test_rotation_trials_are_individually_validated():
    from real_world_rl.teleop.calibration import validate_axes
    good = [[0, 0, 0, .4, 0, 0]]
    bad = [[0, 0, 0, -.4, 0, 0]]
    with pytest.raises(ValueError, match='direction error'):
        validate_axes({'+rx': {'trials': [good * 100, bad]}}, 0)
