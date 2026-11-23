// SPDX-License-Identifier: MIT
// Long randomized soak of TrajectoryImpedanceLaw against a simulated arm: GELLO-like retargeting
// at 10 - 30 Hz (small steps, occasional large jumps and targets beyond the limits), rejected
// non-finite targets, pauses longer than the command timeout, request_stop, lost packets
// (2 - 5 ms periods) and live gain updates. Every tick checks all invariants.
#include <doctest/doctest.h>

#include <Eigen/Dense>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <random>
#include <ratio>

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

constexpr double kPi = 3.14159265358979323846;

void soak(const char *robot_model, uint64_t seed, int ticks) {
  std::mt19937_64 rng(seed);
  std::uniform_real_distribution<double> u01(0.0, 1.0);
  std::normal_distribution<double> noise(0.0, 1.0);
  std::uniform_int_distribution<int> period_ticks(33, 100);   // 10 - 30 Hz
  std::uniform_int_distribution<int> pause_ticks(600, 1500);  // > command_timeout_s
  std::uniform_int_distribution<int> lost(2, 5);

  TrajectoryImpedanceParams p = defaultParams(robot_model);
  const auto limits = limitsOf(p);
  const Vector7d inertia = (Vector7d() << 3.0, 3.0, 2.5, 2.0, 1.0, 0.5, 0.3).finished();
  TrajectoryImpedanceLaw law(p);

  Vector7d q = (p.lower_limit + p.upper_limit) / 2;
  Vector7d dq = Vector7d::Zero();
  law.starting(q);
  Vector7d gello = q;  // the leader arm's position

  int next_target = 0;
  int pause_until = -1;
  int pending_run = 0;
  int max_pending_run = 0;
  uint64_t retargets = 0;
  uint64_t rejected = 0;
  uint64_t timeouts = 0;
  uint64_t stops = 0;
  uint64_t jumps = 0;
  uint64_t completed = 0;
  bool was_timed_out = false;
  double worst_us = 0.0;
  double total_us = 0.0;
  int slow_ticks = 0;
  Vector7d tau_prev = Vector7d::Zero();
  ReferenceState prev = law.reference();

  for (int k = 0; k < ticks; ++k) {
    // ---- user side: GELLO targets ----
    if (k >= next_target && k >= pause_until) {
      next_target = k + period_ticks(rng);
      const double r = u01(rng);
      if (r < 0.01) {
        Vector7d bad = gello;
        bad(static_cast<int>(u01(rng) * 7) % 7) = std::numeric_limits<double>::quiet_NaN();
        REQUIRE_FALSE(law.setTarget(bad));
        ++rejected;
      } else if (r < 0.02) {
        pause_until = k + pause_ticks(rng);  // operator lets go: no targets for a while
      } else if (r < 0.03) {
        law.requestStop();
        ++stops;
      } else {
        if (r < 0.06) {  // a large jump, sometimes beyond the limits (clipped)
          gello = uniform(rng, p.lower_limit.array() - 0.3, p.upper_limit.array() + 0.3);
          ++jumps;
        } else {  // the operator moves the leader arm
          for (int j = 0; j < 7; ++j) gello(j) += 0.03 * noise(rng);
          gello =
              gello.cwiseMax((p.lower_limit.array() - 0.1).matrix()).cwiseMin((p.upper_limit.array() + 0.1).matrix());
        }
        REQUIRE(law.setTarget(gello));
        ++retargets;
      }
    }
    if (k % 25000 == 12345) {  // live gain updates (the reference limits stay fixed)
      TrajectoryImpedanceParams q2 = law.params();
      q2.stiffness = TrajectoryImpedanceParams::gelloStiffness() * (0.5 + u01(rng));
      q2.velocity_filter_alpha = 0.5 + 0.5 * u01(rng);
      q2.max_delta_tau = 0.5 + u01(rng);
      law.applyParams(q2);
    }

    // ---- control tick ----
    const double dt = u01(rng) < 0.01 ? 0.001 * lost(rng) : 0.001;
    TrajectoryImpedanceLaw::Input in;
    in.q = q;
    in.dq = dq + 1e-4 * Vector7d::NullaryExpr([&] { return noise(rng); });
    in.coriolis = 0.05 * dq;
    in.dt = dt;
    const auto t0 = std::chrono::steady_clock::now();
    const Vector7d tau = law.update(in);
    const double us = std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - t0).count();
    worst_us = std::max(worst_us, us);
    if (us > 100.0) ++slow_ticks;
    total_us += us;

    // ---- invariants ----
    const ReferenceState &ref = law.reference();
    REQUIRE(tau.allFinite());
    REQUIRE((tau - tau_prev).cwiseAbs().maxCoeff() <= law.params().max_delta_tau * (1.0 + 1e-12));
    REQUIRE(withinVelocity(ref.dq, p.max_velocity));
    REQUIRE(withinAcceleration(ref.ddq, p.max_acceleration));
    REQUIRE(continuousStep(prev, ref, dt, limits));
    REQUIRE((ref.q.array() >= p.lower_limit.array()).all());
    REQUIRE((ref.q.array() <= p.upper_limit.array()).all());
    REQUIRE((law.lastTarget().array() >= p.targetLower().array()).all());
    REQUIRE((law.lastTarget().array() <= p.targetUpper().array()).all());
    if (!law.targetPending() && !law.stopped() && law.generator().atRest()) {
      if (ref.q != law.lastTarget()) FAIL("at rest but not exactly at the target, tick " << k);
    }
    if (!law.targetPending() && !law.stopped() && law.segmentTime() == 0.0 && law.segmentDuration() > 0.0) {
      REQUIRE(law.segmentDuration() >= p.min_segment_duration_s);
    }
    if (law.timeSinceTarget() >= p.command_timeout_s && !law.stopped()) FAIL("timeout not handled, tick " << k);
    if (law.timedOut() && !was_timed_out) ++timeouts;
    was_timed_out = law.timedOut();
    if (law.targetPending()) {
      ++pending_run;
      max_pending_run = std::max(max_pending_run, pending_run);
      REQUIRE(pending_run < 10000);  // a deferred target is planned within 10 s
    } else {
      pending_run = 0;
    }
    if (law.generator().atRest() && !(prev.dq.isZero(0.0) && prev.ddq.isZero(0.0))) ++completed;

    // ---- simulated arm (gravity compensated by libfranka, a little viscous friction) ----
    const Vector7d ddq = (tau - 0.05 * dq - in.coriolis).cwiseQuotient(inertia);
    dq += dt * ddq;
    q += dt * dq;

    tau_prev = tau;
    prev = ref;
  }
  std::printf(
      "soak %s: %d ticks, %llu targets (%llu jumps), %llu rejected, %llu timeouts, %llu stops, %llu segments "
      "completed, %llu plan retries (longest wait %d ticks); update(): mean %.2f us, worst %.1f us, %d ticks > 100 "
      "us\n",
      robot_model, ticks, static_cast<unsigned long long>(retargets), static_cast<unsigned long long>(jumps),
      static_cast<unsigned long long>(rejected), static_cast<unsigned long long>(timeouts),
      static_cast<unsigned long long>(stops), static_cast<unsigned long long>(completed),
      static_cast<unsigned long long>(law.planRetries()), max_pending_run, total_us / ticks, worst_us, slow_ticks);
  CHECK(retargets > static_cast<uint64_t>(ticks / 120));
  CHECK(timeouts > 5);
  CHECK(stops > 5);
  CHECK(rejected > 5);
}

}  // namespace

TEST_CASE("soak: panda, 300k ticks") { soak("panda", 2024, 300000); }
TEST_CASE("soak: fr3, 300k ticks") { soak("fr3", 77, 300000); }

TEST_CASE("smooth leader arm at 10 / 20 / 30 Hz: tracks with few braking events") {
  // A GELLO operator: a smooth joint motion (sum of sines, peak speed below max_velocity)
  // sampled at the leader's rate with a little sensor noise.
  for (const int rate_hz : {10, 20, 30}) {
    std::mt19937_64 rng(static_cast<uint64_t>(rate_hz));
    std::normal_distribution<double> noise(0.0, 1e-3);
    const TrajectoryImpedanceParams p = defaultParams("panda");
    const auto limits = limitsOf(p);
    TrajectoryImpedanceLaw law(p);
    const Vector7d center = (p.lower_limit + p.upper_limit) / 2;
    const auto leader = [&](double t) {
      Vector7d q;
      for (int j = 0; j < 7; ++j) {
        q(j) = center(j) + 0.25 * std::sin(2.0 * kPi * (0.12 + 0.03 * j) * t + j) +
               0.05 * std::sin(2.0 * kPi * (0.4 + 0.05 * j) * t + 2.0 * j);
      }
      return q;
    };
    law.starting(leader(0.0));
    const int period = 1000 / rate_hz;
    double max_err = 0.0;
    ReferenceState prev = law.reference();
    const int ticks = 60000;
    for (int k = 0; k < ticks; ++k) {
      const double t = 0.001 * k;
      if (k % period == 0) {
        Vector7d target = leader(t);
        for (int j = 0; j < 7; ++j) target(j) += noise(rng);
        REQUIRE(law.setTarget(target));
      }
      TrajectoryImpedanceLaw::Input in;
      in.q = law.reference().q;
      in.dt = 0.001;
      (void)law.update(in);
      const ReferenceState &ref = law.reference();
      REQUIRE(withinVelocity(ref.dq, p.max_velocity));
      REQUIRE(withinAcceleration(ref.ddq, p.max_acceleration));
      REQUIRE(continuousStep(prev, ref, 0.001, limits));
      if (k > 2000) max_err = std::max(max_err, (ref.q - leader(t)).cwiseAbs().maxCoeff());
      prev = ref;
    }
    std::printf("leader %d Hz: max |q_ref - q_leader| %.4f rad, plan retries %llu of %d ticks\n", rate_hz, max_err,
                static_cast<unsigned long long>(law.planRetries()), ticks);
    CHECK(max_err < 0.25);
    CHECK_FALSE(law.timedOut());
  }
}
