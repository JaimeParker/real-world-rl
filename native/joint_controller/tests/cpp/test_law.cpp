// SPDX-License-Identifier: MIT
// TrajectoryImpedanceLaw: start, target clipping / rejection, the GELLO torque law with its
// velocity filter, coriolis, torque-rate limit, timeout stop and resume, request_stop, params.
#include <doctest/doctest.h>

#include <Eigen/Dense>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <random>
#include <stdexcept>

#include "joint_controller/law.hpp"
#include "joint_controller/params.hpp"
#include "joint_controller/planner.hpp"
#include "test_util.hpp"

using joint_controller::defaultParams;
using joint_controller::ReferenceState;
using joint_controller::TrajectoryImpedanceLaw;
using joint_controller::TrajectoryImpedanceParams;
using joint_controller::Vector7d;
using joint_controller::test::continuousStep;
using joint_controller::test::limitsOf;
using joint_controller::test::uniform;
using joint_controller::test::withinAcceleration;
using joint_controller::test::withinVelocity;

namespace {

constexpr double kDt = 0.001;

Vector7d home() { return (Vector7d() << 0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785).finished(); }

TrajectoryImpedanceLaw::Input input(const Vector7d &q, const Vector7d &dq = Vector7d::Zero(),
                                    const Vector7d &coriolis = Vector7d::Zero(), double dt = kDt) {
  TrajectoryImpedanceLaw::Input in;
  in.q = q;
  in.dq = dq;
  in.coriolis = coriolis;
  in.dt = dt;
  return in;
}

}  // namespace

TEST_CASE("start seeds the reference at the measured q: zero spring torque at tick 0") {
  TrajectoryImpedanceLaw law(defaultParams("panda"));
  const Vector7d q0 = home() + Vector7d::Constant(0.01);
  law.starting(q0);
  CHECK(law.reference().q == q0);
  CHECK(law.reference().dq.isZero(0.0));
  const Vector7d tau = law.update(input(q0, Vector7d::Zero(), Vector7d::Zero(), 0.0));  // franky: first time_step 0
  CHECK(tau.isZero(0.0));
  CHECK(law.reference().q == q0);
  // with joint velocity at start only the damping term acts (rate-limited from 0)
  TrajectoryImpedanceLaw law2(defaultParams("panda"));
  law2.starting(q0);
  const Vector7d dq = Vector7d::Constant(0.001);
  const Vector7d tau2 = law2.update(input(q0, dq));
  const Vector7d expected = -law2.params().damping.cwiseProduct(0.99 * dq);
  CHECK((tau2 - expected).cwiseAbs().maxCoeff() < 1e-15);
}

TEST_CASE("targets: clipped into the envelope with margin; non-finite rejected, state unchanged") {
  const TrajectoryImpedanceParams p = defaultParams("panda");
  TrajectoryImpedanceLaw law(p);
  law.starting(home());
  Vector7d beyond = home();
  beyond(0) = 10.0;
  beyond(3) = -10.0;
  REQUIRE(law.setTarget(beyond));
  CHECK(law.lastTarget()(0) == p.upper_limit(0) - 0.05);
  CHECK(law.lastTarget()(3) == p.lower_limit(3) + 0.05);
  CHECK(law.lastTarget()(1) == home()(1));

  TrajectoryImpedanceLaw a(p);
  TrajectoryImpedanceLaw b(p);
  a.starting(home());
  b.starting(home());
  const Vector7d t = home() + Vector7d::Constant(0.1);
  REQUIRE(a.setTarget(t));
  REQUIRE(b.setTarget(t));
  for (int k = 0; k < 300; ++k) {
    if (k == 100) {
      Vector7d bad = t;
      bad(4) = std::numeric_limits<double>::quiet_NaN();
      CHECK_FALSE(b.setTarget(bad));
      bad(4) = std::numeric_limits<double>::infinity();
      CHECK_FALSE(b.setTarget(bad));
    }
    const Vector7d q = home() + Vector7d::Constant(1e-4 * k);
    REQUIRE((a.update(input(q)).array() == b.update(input(q)).array()).all());
    REQUIRE(a.reference().q == b.reference().q);
    REQUIRE(a.lastTarget() == b.lastTarget());
  }
}

TEST_CASE("torque = K (q_ref - q) - D dq_f (+ coriolis), dq_f = (1 - alpha) dq_f + alpha dq") {
  std::mt19937_64 rng(11);
  for (const bool coriolis_on : {true, false}) {
    TrajectoryImpedanceParams p = defaultParams("fr3");
    p.max_delta_tau = 1e9;  // isolate the law from the rate limit
    p.velocity_filter_alpha = 0.7;
    p.compensate_coriolis = coriolis_on;
    TrajectoryImpedanceLaw law(p);
    const Vector7d q0 = (p.lower_limit + p.upper_limit) / 2;
    law.starting(q0);
    REQUIRE(law.setTarget(q0 + Vector7d::Constant(0.2)));
    Vector7d dq_f = Vector7d::Zero();
    for (int k = 0; k < 3000; ++k) {
      const Vector7d q = q0 + uniform(rng, Vector7d::Constant(-0.01), Vector7d::Constant(0.01));
      const Vector7d dq = uniform(rng, Vector7d::Constant(-0.5), Vector7d::Constant(0.5));
      const Vector7d c = uniform(rng, Vector7d::Constant(-2.0), Vector7d::Constant(2.0));
      const Vector7d tau = law.update(input(q, dq, c));
      dq_f = (1.0 - 0.7) * dq_f + 0.7 * dq;
      Vector7d expected = p.stiffness.cwiseProduct(law.reference().q - q) - p.damping.cwiseProduct(dq_f);
      if (coriolis_on) expected += c;
      REQUIRE((tau - expected).cwiseAbs().maxCoeff() <= 1e-12);
      REQUIRE((law.dqFiltered() - dq_f).cwiseAbs().maxCoeff() <= 1e-15);
    }
  }
}

TEST_CASE("max_delta_tau limits the torque change per tick") {
  TrajectoryImpedanceParams p = defaultParams("panda");
  p.max_delta_tau = 0.25;
  TrajectoryImpedanceLaw law(p);
  law.starting(home(), Vector7d::Constant(3.0));  // previous command of another motion
  Vector7d prev = Vector7d::Constant(3.0);
  const Vector7d q_off = home() + Vector7d::Constant(0.05);  // large spring torque at once
  int saturated = 0;
  for (int k = 0; k < 500; ++k) {
    const Vector7d tau = law.update(input(q_off));
    REQUIRE((tau - prev).cwiseAbs().maxCoeff() <= 0.25 + 1e-12);
    if ((tau - prev).cwiseAbs().maxCoeff() >= 0.25 - 1e-12) ++saturated;
    prev = tau;
  }
  CHECK(saturated > 10);
  // settled: equals the unlimited law
  const Vector7d expected = p.stiffness.cwiseProduct(home() - q_off);
  CHECK((prev - expected).cwiseAbs().maxCoeff() < 1e-9);
}

TEST_CASE("timeout: bounded stop (unimodal |v| falling to rest, within limits), then a target resumes") {
  const TrajectoryImpedanceParams p = defaultParams("panda");
  const auto l = limitsOf(p);
  TrajectoryImpedanceLaw law(p);
  law.starting(home());
  Vector7d far = home();
  far(0) += 1.5;
  far(3) += 0.8;
  far(6) -= 1.2;
  REQUIRE(law.setTarget(far));
  int k = 0;
  int timeout_tick = -1;
  std::array<bool, 7> falling{};
  Vector7d prev_speed = Vector7d::Zero();
  ReferenceState prev = law.reference();
  for (; k < 20000; ++k) {
    (void)law.update(input(law.reference().q));
    const ReferenceState &s = law.reference();
    REQUIRE(withinVelocity(s.dq, p.max_velocity));
    REQUIRE(withinAcceleration(s.ddq, p.max_acceleration));
    REQUIRE(continuousStep(prev, s, kDt, l));
    if (law.timedOut() && timeout_tick < 0) {
      timeout_tick = k;
      CHECK(law.timeSinceTarget() >= 0.5);
      CHECK(law.timeSinceTarget() < 0.5 + 1.5 * kDt);
      CHECK(law.stopped());
      CHECK(law.segmentTime() == 0.0);
    }
    if (timeout_tick >= 0) {
      for (int j = 0; j < 7; ++j) {
        if (law.generator().stopPending(j)) continue;
        const auto u = static_cast<size_t>(j);
        const double speed = std::abs(s.dq(j));
        if (falling[u]) REQUIRE(speed <= prev_speed(j) + 1e-12);
        if (speed < prev_speed(j)) falling[u] = true;
      }
      if (law.generator().atRest() && !law.generator().stopPending()) break;
    }
    prev_speed = s.dq.cwiseAbs();
    prev = s;
  }
  // tick 0 took the target; 500 periods of 1 ms later (rounding of the summed clock: +1)
  REQUIRE(timeout_tick >= 500);
  REQUIRE(timeout_tick <= 501);
  CHECK(law.reference().dq.isZero(0.0));
  CHECK(law.reference().ddq.isZero(0.0));
  CHECK(law.segmentDuration() < 2.0);
  const Vector7d stopped_at = law.reference().q;
  CHECK((stopped_at - far).cwiseAbs().maxCoeff() > 0.1);  // stopped on the way, not at the target
  for (int t = 0; t < 1000; ++t) (void)law.update(input(stopped_at));
  CHECK(law.reference().q == stopped_at);  // holds
  CHECK(law.timedOut());

  REQUIRE(law.setTarget(far));  // resumes
  (void)law.update(input(stopped_at));
  CHECK_FALSE(law.timedOut());
  CHECK_FALSE(law.stopped());
  for (int t = 0; t < 20000 && !law.generator().atRest(); ++t) {
    if (t % 100 == 0) REQUIRE(law.setTarget(far));  // keep it alive at 10 Hz
    (void)law.update(input(law.reference().q));
  }
  CHECK(law.reference().q == law.lastTarget());
}

TEST_CASE("request_stop: bounded stop, keeps running, next target resumes") {
  const TrajectoryImpedanceParams p = defaultParams("panda");
  TrajectoryImpedanceLaw law(p);
  law.starting(home());
  REQUIRE(law.setTarget(home() + Vector7d::Constant(0.4)));
  for (int k = 0; k < 300; ++k) (void)law.update(input(law.reference().q));
  CHECK(law.reference().dq.cwiseAbs().maxCoeff() > 0.05);
  law.requestStop();
  (void)law.update(input(law.reference().q));
  CHECK(law.stopped());
  CHECK_FALSE(law.timedOut());
  for (int k = 0; k < 5000 && (!law.generator().atRest() || law.generator().stopPending()); ++k) {
    REQUIRE(withinVelocity(law.reference().dq, p.max_velocity));
    (void)law.update(input(law.reference().q));
  }
  CHECK(law.reference().dq.isZero(0.0));
  const Vector7d target = home() - Vector7d::Constant(0.1);
  REQUIRE(law.setTarget(target));
  for (int k = 0; k < 5000; ++k) {
    if (k % 50 == 0) REQUIRE(law.setTarget(target));
    (void)law.update(input(law.reference().q));
  }
  CHECK(law.reference().q == target);
  // the newest command wins within one tick
  law.requestStop();
  REQUIRE(law.setTarget(home()));
  (void)law.update(input(law.reference().q));
  CHECK_FALSE(law.stopped());
}

TEST_CASE("params: a limit change replans the re-clipped target with the new limits") {
  const TrajectoryImpedanceParams p = defaultParams("panda");
  TrajectoryImpedanceLaw law(p);
  law.starting(home());
  Vector7d target = home();
  target(0) = 2.0;
  REQUIRE(law.setTarget(target));
  for (int k = 0; k < 100; ++k) (void)law.update(input(law.reference().q));
  TrajectoryImpedanceParams q = p;
  q.upper_limit(0) = 1.0;
  q.max_velocity = Vector7d::Constant(1.0);
  law.applyParams(q);
  CHECK(law.lastTarget()(0) == 1.0 - 0.05);
  for (int k = 0; k < 10000; ++k) {
    if (k % 100 == 0) REQUIRE(law.setTarget(target));
    (void)law.update(input(law.reference().q));
  }
  CHECK(law.reference().q(0) == 0.95);
  TrajectoryImpedanceParams bad = q;
  bad.velocity_filter_alpha = 3.0;
  CHECK_THROWS_AS(law.applyParams(bad), std::invalid_argument);
  CHECK(law.params() == q);
}

TEST_CASE("non-finite measurements hold the previous torque") {
  TrajectoryImpedanceLaw law(defaultParams("panda"));
  law.starting(home());
  (void)law.update(input(home() + Vector7d::Constant(0.001)));
  const Vector7d last = law.tauCommand();
  Vector7d bad = home();
  bad(2) = std::numeric_limits<double>::quiet_NaN();
  const Vector7d tau = law.update(input(bad));
  CHECK(tau == last);
  CHECK(law.dqFiltered().allFinite());
}
