"""Tests of the joint_controller Python binding. No robot needed.

Skipped entirely when the compiled extension is not installed, so a pytest run
from the repository root never fails on it.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

jc = pytest.importorskip("joint_controller")

HOME = np.array([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785])
TELEMETRY_KEYS = {
    "q_ref", "dq_ref", "ddq_ref", "target", "tau_cmd", "segment_t", "segment_T", "timed_out", "ticks",
}  # fmt: skip


def test_default_params():
    p = jc.default_params("panda")
    assert set(p) == set(jc.PARAM_FIELDS) and len(p) == 12
    np.testing.assert_array_equal(p["stiffness"], [240, 240, 240, 240, 100, 60, 20])
    np.testing.assert_array_equal(p["damping"], [20, 20, 20, 10, 10, 10, 5])
    assert p["velocity_filter_alpha"] == 0.99
    np.testing.assert_array_equal(p["max_velocity"], [0.6] * 4 + [0.75] * 3)
    np.testing.assert_array_equal(p["max_acceleration"], [4.0] * 7)
    assert p["min_segment_duration_s"] == 0.1
    assert p["limit_margin"] == 0.05
    assert p["command_timeout_s"] == 0.5
    assert p["max_delta_tau"] == 1.0
    assert p["compensate_coriolis"] is True
    assert p["upper_limit"][3] == -0.0698
    assert jc.default_params("fr3")["lower_limit"][5] == 0.8521
    assert jc.GELLO_COMMIT == "9fc8c26"
    with pytest.raises(ValueError):
        jc.default_params("fr2")


def test_validate_params_requires_limits_and_rejects_bad_values():
    with pytest.raises(ValueError, match="lower_limit"):
        jc.validate_params({})
    p = jc.default_params("panda")
    assert jc.validate_params(p)["max_delta_tau"] == 1.0
    for key, value in [
        ("stiffness", [-1.0] * 7),
        ("velocity_filter_alpha", 1.5),
        ("max_velocity", [0.0] * 7),
        ("lower_limit", p["upper_limit"]),
        ("max_delta_tau", True),  # a bool is not a number
        ("compensate_coriolis", 1),  # nor a number a bool
        ("stiffness", [1.0] * 6),  # wrong length
        ("bogus", 1.0),
    ]:
        with pytest.raises(ValueError):
            jc.validate_params({**p, key: value})
    # numpy scalars and bools are accepted
    q = jc.validate_params({**p, "max_delta_tau": np.float32(0.5), "compensate_coriolis": np.bool_(False)})
    assert q["max_delta_tau"] == 0.5 and q["compensate_coriolis"] is False


def test_law_offline_start_target_and_telemetry():
    law = jc.TrajectoryImpedanceLaw(jc.default_params("panda"))
    law.starting(HOME)
    tau = law.step(HOME, np.zeros(7), np.zeros(7), 0.0)
    np.testing.assert_array_equal(tau, np.zeros(7))  # reference seeded at q: no spring torque
    assert law.set_target(HOME + 0.1)
    assert not law.set_target(np.full(7, np.nan))
    for k in range(3000):
        if k % 100 == 0:  # a 10 Hz stream (a single target would time out after 0.5 s)
            assert law.set_target(HOME + 0.1)
        tau = law.step(law.telemetry()["q_ref"], np.zeros(7), np.zeros(7))
    t = law.telemetry()
    assert TELEMETRY_KEYS <= set(t)
    np.testing.assert_array_equal(t["q_ref"], HOME + 0.1)
    np.testing.assert_array_equal(t["dq_ref"], np.zeros(7))
    assert t["ticks"] == 3001 and not t["timed_out"] and not t["target_pending"]
    assert t["segment_T"] == 0.0  # the repeated target is reached: nothing left to plan
    eff = law.apply_params({"max_delta_tau": 0.5})
    assert eff["max_delta_tau"] == 0.5 and law.params()["max_delta_tau"] == 0.5


def test_law_timeout_stops_and_holds():
    law = jc.TrajectoryImpedanceLaw(jc.default_params("panda"))
    law.starting(HOME)
    law.set_target(HOME + np.array([1.0, 0, 0, 0, 0, 0, 0]))
    for _ in range(2500):
        law.step(law.telemetry()["q_ref"], np.zeros(7), np.zeros(7))
    t = law.telemetry()
    assert t["timed_out"] and t["stopped"]
    np.testing.assert_array_equal(t["dq_ref"], np.zeros(7))
    assert 0.0 < t["q_ref"][0] - HOME[0] < 1.0


franky_required = pytest.mark.skipif(not jc.WITH_FRANKY, reason="built without franky")


@franky_required
def test_motion_is_a_franky_torque_motion():
    import franky

    m = jc.TrajectoryImpedanceMotion(jc.default_params("panda"))
    assert isinstance(m, franky.BaseTorqueMotion)
    # franky's own C++ functions accept it as std::shared_ptr<Motion<franka::Torques>>
    m.add_reaction(franky.TorqueReaction(franky.Condition(False), jc.TrajectoryImpedanceMotion(jc.default_params("fr3"))))
    assert len(m.reactions) == 1
    with pytest.raises(ValueError):
        jc.TrajectoryImpedanceMotion({})  # joint limits are required


@franky_required
def test_motion_offline_matches_law():
    params = jc.default_params("panda")
    m = jc.TrajectoryImpedanceMotion(params)
    law = jc.TrajectoryImpedanceLaw(params)
    rng = np.random.default_rng(1)
    inputs = [(HOME + rng.normal(size=7) * 1e-3, rng.normal(size=7) * 1e-2, rng.normal(size=7) * 0.1) for _ in range(2000)]
    assert m.set_target(HOME + 0.05)  # before the start: applied on the first tick
    law.starting(inputs[0][0])  # the motion starts at the first measured q
    law.set_target(HOME + 0.05)
    for k, (q, dq, c) in enumerate(inputs):
        if k == 500:
            assert m.set_target(HOME - 0.05) and law.set_target(HOME - 0.05)
        if k == 900:
            eff = m.update_params({"stiffness": np.full(7, 100.0)})
            np.testing.assert_array_equal(eff["stiffness"], np.full(7, 100.0))
            law.apply_params({"stiffness": np.full(7, 100.0)})
        if k == 1500:
            m.request_stop()
            law.request_stop()
        dt = 0.0 if k == 0 else 0.001
        np.testing.assert_array_equal(m._offline_step(q, dq, c, dt), law.step(q, dq, c, dt))
    t = m.telemetry()
    assert TELEMETRY_KEYS <= set(t)
    assert t["ticks"] == 2000 and t["stopped"] and not t["timed_out"]
    assert t["applied_command_seq"] == 3 and t["applied_params_seq"] == 1
    np.testing.assert_array_equal(m.params()["stiffness"], np.full(7, 100.0))
    assert not m.set_target([0, 0, 0, float("nan"), 0, 0, 0])
    with pytest.raises(ValueError):
        m.update_params({"velocity_filter_alpha": 2.0})
    assert m.params()["velocity_filter_alpha"] == 0.99


def test_build_info_file_matches_the_compiled_copy():
    assert jc.BUILD_INFO_FILE.is_file()
    assert jc.build_info() == dict(jc.BUILD_INFO)
    assert jc.build_info()["with_franky"] is jc.WITH_FRANKY
    if jc.WITH_FRANKY:
        info = jc.build_info()
        assert len(info["libfranky_sha256"]) == 64
        assert info["franky_layout"] in ("wheel", "source")
        if info["franky_layout"] == "wheel":  # libfranka bundled in the franky wheel
            assert info["libfranka_file"].startswith("libfranka")
        else:  # franky built from source against a system libfranka
            assert info["libfranka_file"] == "" and Path(info["libfranka_path"]).is_file()
            assert len(info["libfranka_sha256"]) == 64
