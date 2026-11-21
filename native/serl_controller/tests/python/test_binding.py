"""Tests of the serl_controller Python binding. No robot needed.

Skipped entirely when the compiled extension is not installed, so a pytest run
from the repository root never fails on it.
"""

from __future__ import annotations

import numpy as np
import pytest

sc = pytest.importorskip("serl_controller")


def _state(rng: np.random.Generator):
    T = np.eye(4)
    T[:3, 3] = [0.4, 0.0, 0.3]
    return dict(
        O_T_EE=T,
        q=rng.normal(size=7) * 0.3,
        dq=rng.normal(size=7) * 0.1,
        tau_J_d=rng.normal(size=7) * 0.1,
        zero_jacobian=rng.normal(size=(6, 7)) * 0.4,
        coriolis=rng.normal(size=7) * 0.1,
    )


def test_constants():
    assert sc.SERL_COMMIT == "1f140ef"
    assert sc.FILTER_PARAMS == 0.005
    assert sc.DELTA_TAU_MAX == 1.0
    assert sc.PINV_LAMBDA == 0.2
    names = [f[0] for f in sc.PARAM_FIELDS]
    assert len(names) == 20 and names[0] == "translational_stiffness"


def test_params_merge_and_negation():
    p = sc.SerlParams({"translational_clip_neg_y": 0.03, "translational_stiffness": 1e9})
    d = p.to_dict()
    assert d["translational_clip_neg_y"] == 0.03
    assert d["translational_stiffness"] == 6000.0  # clamped like dynamic_reconfigure
    assert d["rotational_stiffness"] == 150.0
    law = sc.SerlCartesianImpedanceLaw(p)
    st = law.state()
    assert st["translational_clip_min"][1] == -0.03
    assert st["translational_clip_max"][1] == 0.01
    with pytest.raises(ValueError):
        sc.SerlParams({"nope": 1.0})


def test_law_step_ramp_and_saturation():
    rng = np.random.default_rng(0)
    law = sc.SerlCartesianImpedanceLaw()
    s = _state(rng)
    law.starting(s["O_T_EE"], s["q"])
    assert np.all(law.state()["cartesian_stiffness"] == 0.0)
    tau = sc.law_step(law, **s)
    assert tau.shape == (7,)
    assert np.all(np.abs(tau - s["tau_J_d"]) <= 1.0 + 1e-12)
    np.testing.assert_allclose(np.diag(law.state()["cartesian_stiffness"])[:3], 0.005 * 2000.0)
    tau2 = law.step(**s)
    assert np.all(np.isfinite(tau2))


def test_set_target_hemisphere_flip_and_error_i_reset():
    law = sc.SerlCartesianImpedanceLaw()
    T = np.eye(4)
    T[:3, 3] = [0.5, 0, 0]
    law.starting(T, np.zeros(7))
    T2 = T.copy()
    T2[0, 3] = 0.6
    for _ in range(20):
        law.step(T2, np.zeros(7), np.zeros(7), np.zeros(7), np.eye(6, 7), np.zeros(7))
    assert law.state()["error_i"][0] == pytest.approx(0.1)
    law.set_target([0.5, 0, 0], [0, 0, 0, -1])
    st = law.state()
    assert np.all(st["error_i"] == 0.0)
    assert st["orientation_d_target_xyzw"][3] == 1.0


franky_required = pytest.mark.skipif(not sc.WITH_FRANKY, reason="built without franky")


@franky_required
def test_motion_is_a_franky_torque_motion():
    import franky

    m = sc.SerlCartesianImpedanceMotion()
    assert isinstance(m, franky.BaseTorqueMotion)
    # franky's own C++ functions accept it as std::shared_ptr<Motion<franka::Torques>>
    reaction = franky.TorqueReaction(franky.Condition(False), m)
    m.add_reaction(franky.TorqueReaction(franky.Condition(False), sc.SerlCartesianImpedanceMotion()))
    assert len(m.reactions) == 1
    assert reaction is not None


@franky_required
def test_motion_offline_matches_law():
    rng = np.random.default_rng(1)
    m = sc.SerlCartesianImpedanceMotion({"translational_stiffness": 1000.0})
    law = sc.SerlCartesianImpedanceLaw({"translational_stiffness": 1000.0})
    assert m.params["translational_stiffness"] == 1000.0
    s0 = _state(rng)
    law.starting(s0["O_T_EE"], s0["q"])
    for t in range(300):
        s = s0 if t == 0 else _state(rng)
        if t == 100:
            m.set_target([0.41, 0.0, 0.3], [0, 0, 0, 1])
            law.set_target([0.41, 0.0, 0.3], [0, 0, 0, 1])
        if t == 200:
            eff = m.update_params({"rotational_clip_neg_z": 0.01})
            assert eff["rotational_clip_neg_z"] == 0.01
            law.apply_params(sc.SerlParams(eff))
        np.testing.assert_array_equal(m._offline_step(**s), law.step(**s))
    snap = m.snapshot()
    assert snap["tick"] == 300
    assert snap["zero_jacobian"].shape == (6, 7)
    assert snap["applied_target_seq"] == 1
    with pytest.raises(ValueError):
        m.set_target([0, 0, float("nan")], [0, 0, 0, 1])
    with pytest.raises(ValueError):
        m.update_params({"translational_stiffness": float("inf")})


def test_build_info_file_matches_the_compiled_copy():
    assert sc.BUILD_INFO_FILE.is_file()
    assert sc.build_info() == dict(sc.BUILD_INFO)
    assert sc.build_info()["with_franky"] is sc.WITH_FRANKY
    if sc.WITH_FRANKY:
        info = sc.build_info()
        assert len(info["libfranky_sha256"]) == 64
        assert info["libfranka_file"].startswith("libfranka")


def test_joint_law_blends_for_ten_seconds_then_holds():
    target = np.linspace(-0.5, 0.5, 7)
    start = np.zeros(7)
    law = sc.SerlJointPositionLaw(target)
    law.starting(start)
    np.testing.assert_array_equal(law.update(0), start)  # first libfranka tick: period 0
    for _ in range(5000):
        q = law.update(1_000_000)
    assert law.elapsed_s == 5.0
    np.testing.assert_allclose(q, 0.5 * target, rtol=0, atol=1e-15)
    for _ in range(5001):
        q = law.update(1_000_000)
    np.testing.assert_array_equal(q, target)
    assert sc.SerlJointPositionLaw.period_ns_from_milliseconds(3) == 3_000_000
    assert sc.JOINT_BLEND_DURATION_S == 10.0
    with pytest.raises(ValueError):
        sc.SerlJointPositionLaw([0, 0, 0, float("nan"), 0, 0, 0])


@franky_required
def test_joint_motion_is_a_franky_joint_position_motion_and_finishes_cleanly():
    import franky

    target = np.array([0.0, 0.0, 0.0, -1.9, 0.0, 2.0, 0.0])  # inside Panda's limits: no soft limiting
    m = sc.SerlJointPositionMotion(target)
    assert isinstance(m, franky.BaseJointPositionMotion)
    m.add_reaction(franky.JointPositionReaction(franky.Condition(False), sc.SerlJointPositionMotion(target)))
    law = sc.SerlJointPositionLaw(target)
    q0 = np.array([0.1, -0.3, 0.05, -2.2, 0.02, 1.8, 0.7])
    law.starting(q0)
    for step in range(12_000):
        ms = 0 if step == 0 else (2 if step % 97 == 0 else 1)
        command, finished = m._offline_step(q0, ms)
        np.testing.assert_array_equal(command, law.update(ms * 1_000_000))
        assert not finished
    np.testing.assert_array_equal(command, target)
    m.finish()
    assert m.finish_requested
    final, finished = m._offline_step(q0, 1)
    assert finished
    np.testing.assert_array_equal(final, command)
    snap = m.snapshot()
    assert snap["finished"] and snap["elapsed_s"] == law.elapsed_s


def test_soft_limits_tables_and_enforce():
    panda = sc.FRANKA_JOINT_LIMITS["panda"]
    assert len(panda) == 7 and panda[3]["max_position"] == -0.0698 and panda[3]["k_position"] == 100.0
    assert sc.FRANKA_JOINT_LIMITS["fr3"][1]["max_velocity"] == 1.0
    limits = sc.PositionJointSoftLimits("panda")
    q = np.zeros(7)
    q[3] = -1.0
    out = limits.enforce(q + 1.0, q, 1_000_000)  # 1 rad jump in 1 ms: capped by max_velocity
    np.testing.assert_allclose(out - q, [0.002175] * 4 + [0.00261] * 3, rtol=1e-12)
    with pytest.raises(ValueError):
        sc.PositionJointSoftLimits("fr2")


@franky_required
def test_joint_motion_soft_limits_and_robot_model():
    upper = sc.FRANKA_JOINT_LIMITS["fr3"][3]["max_position"]
    start = np.array([0.0, 0.0, 0.0, upper - 0.01, 0.0, 2.0, 0.0])
    target = start.copy()
    target[3] = upper + 0.05
    m = sc.SerlJointPositionMotion(target, robot_model="fr3")
    q = start
    for step in range(10_500):
        q, _ = m._offline_step(q, 0 if step == 0 else 1)
    assert upper - 1e-6 < q[3] <= upper
    assert m.snapshot()["soft_limited_ticks"] > 0
