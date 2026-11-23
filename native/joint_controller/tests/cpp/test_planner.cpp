// SPDX-License-Identifier: MIT
// The quintic reference generator: boundary values, exact extrema, limits, common duration,
// minimality, continuity under retargeting, minimum duration, the bounded stop.
#include <doctest/doctest.h>

#include <Eigen/Dense>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <optional>
#include <random>
#include <ratio>

#include "joint_controller/params.hpp"
#include "joint_controller/planner.hpp"
#include "test_util.hpp"

using joint_controller::commonDuration;
using joint_controller::defaultParams;
using joint_controller::JointSample;
using joint_controller::Quintic;
using joint_controller::ReferenceGenerator;
using joint_controller::ReferenceLimits;
using joint_controller::ReferenceState;
using joint_controller::stopDuration;
using joint_controller::Vector7d;
using joint_controller::test::continuousStep;
using joint_controller::test::kLimitSlack;
using joint_controller::test::limitsOf;
using joint_controller::test::uniform;
using joint_controller::test::withinAcceleration;
using joint_controller::test::withinVelocity;

namespace {

constexpr double kDt = 0.001;

ReferenceLimits pandaLimits() { return limitsOf(defaultParams("panda")); }

Vector7d randomTarget(std::mt19937_64 &rng, const ReferenceLimits &l) {
  return uniform(rng, l.lower.array() + 0.05, l.upper.array() - 0.05);
}

/// Advance by kDt until at rest (or max_ticks), checking limits and continuity every tick.
/// Returns the number of ticks.
int runToRest(ReferenceGenerator &gen, const ReferenceLimits &l, int max_ticks = 200000) {
  int ticks = 0;
  while (!gen.atRest() && ticks < max_ticks) {
    const ReferenceState prev = gen.state();
    gen.advance(kDt);
    ++ticks;
    REQUIRE(withinVelocity(gen.state().dq, l.max_velocity));
    REQUIRE(withinAcceleration(gen.state().ddq, l.max_acceleration));
    REQUIRE(continuousStep(prev, gen.state(), kDt, l));
  }
  return ticks;
}

}  // namespace

TEST_CASE("quintic boundary values; exactly at rest at the end") {
  std::mt19937_64 rng(1);
  std::uniform_real_distribution<double> u(-1.0, 1.0);
  for (int i = 0; i < 2000; ++i) {
    const double p0 = u(rng);
    const double v0 = u(rng);
    const double a0 = 3.0 * u(rng);
    const double pf = 2.0 * u(rng);
    const double T = 0.05 + std::abs(3.0 * u(rng));
    const Quintic q = Quintic::toRest(p0, v0, a0, pf, T);
    const JointSample s0 = q.sample(0.0);
    CHECK(s0.p == p0);
    CHECK(s0.v == doctest::Approx(v0).epsilon(1e-12));
    CHECK(s0.a == doctest::Approx(a0).epsilon(1e-12));
    const JointSample near_end = q.sample(T * (1.0 - 1e-9));
    CHECK(std::abs(near_end.p - pf) < 1e-9);
    const JointSample end = q.sample(T);
    CHECK(end.p == pf);
    CHECK(end.v == 0.0);
    CHECK(end.a == 0.0);
    CHECK(q.sample(T + 1.0).p == pf);
  }
}

TEST_CASE("extrema equal dense sampling (root finding, not bounds)") {
  std::mt19937_64 rng(2);
  std::uniform_real_distribution<double> u(-1.0, 1.0);
  for (int i = 0; i < 300; ++i) {
    const double T = 0.1 + std::abs(2.0 * u(rng));
    const Quintic q = (i % 2 == 0) ? Quintic::toRest(u(rng), u(rng), 2.0 * u(rng), u(rng), T)
                                   : Quintic::stop(u(rng), u(rng), 2.0 * u(rng), T);
    const Quintic::Extrema e = q.extrema();
    double v_min = 1e300;
    double v_max = -1e300;
    double a_min = 1e300;
    double a_max = -1e300;
    const int n = 20000;
    for (int k = 0; k <= n; ++k) {
      const double t = T * k / n * (1.0 - 1e-15);
      const JointSample s = q.sample(t);
      v_min = std::min(v_min, s.v);
      v_max = std::max(v_max, s.v);
      a_min = std::min(a_min, s.a);
      a_max = std::max(a_max, s.a);
    }
    // never below a sampled value (up to rounding); dense sampling gets within O(1/n^2)
    CHECK(e.v_max >= v_max - 1e-12);
    CHECK(e.v_min <= v_min + 1e-12);
    CHECK(e.a_max >= a_max - 1e-12);
    CHECK(e.a_min <= a_min + 1e-12);
    const double vs = std::max(std::abs(v_min), std::abs(v_max)) + 1e-3;
    const double as = std::max(std::abs(a_min), std::abs(a_max)) + 1e-3;
    CHECK(e.v_max - v_max < 1e-6 * vs);
    CHECK(v_min - e.v_min < 1e-6 * vs);
    CHECK(e.a_max - a_max < 1e-6 * as);
    CHECK(a_min - e.a_min < 1e-6 * as);
  }
}

TEST_CASE("rest-to-rest: common duration is the analytic minimum, limits hold, endpoint exact") {
  // From rest the quintic peaks at |v| = 15/8 |h| / T and |a| = 10 / sqrt(3) |h| / T^2, so the
  // smallest feasible common duration is known in closed form.
  std::mt19937_64 rng(3);
  const ReferenceLimits l = pandaLimits();
  for (int i = 0; i < 200; ++i) {
    ReferenceGenerator gen;
    const Vector7d start = randomTarget(rng, l);
    gen.reset(start);
    Vector7d target = randomTarget(rng, l);
    if (i % 4 == 0) target = start + Vector7d::Constant(1e-3 * (i % 7));  // short moves -> min duration
    double expected = l.min_segment_duration_s;
    for (int j = 0; j < 7; ++j) {
      const double h = std::abs(target(j) - start(j));
      expected = std::max(expected, 1.875 * h / l.max_velocity(j));
      expected = std::max(expected, std::sqrt(10.0 / std::sqrt(3.0) * h / l.max_acceleration(j)));
    }
    REQUIRE(gen.planTo(target, l));
    const double T = gen.segmentDuration();
    if (target == start) {
      CHECK(T == 0.0);
    } else {
      CHECK(T == doctest::Approx(expected).epsilon(1e-5));
      CHECK(T >= expected * (1.0 - 1e-9));
    }
    for (int j = 0; j < 7; ++j) CHECK(gen.segment(j).duration() == T);  // one common duration
    const int ticks = runToRest(gen, l);
    CHECK(std::abs(ticks - static_cast<int>(std::ceil(T / kDt))) <= 1);
    CHECK(gen.state().q == target);  // exactly
    CHECK(gen.state().dq.isZero(0.0));
    CHECK(gen.state().ddq.isZero(0.0));
  }
}

TEST_CASE("random start states and targets: limits respected every 1 ms, duration minimal on the grid") {
  std::mt19937_64 rng(4);
  const ReferenceLimits l = pandaLimits();
  std::uniform_int_distribution<int> after(1, 400);
  int planned = 0;
  int deferred = 0;
  for (int i = 0; i < 400; ++i) {
    ReferenceGenerator gen;
    gen.reset(randomTarget(rng, l));
    REQUIRE(gen.planTo(randomTarget(rng, l), l));
    // a reachable moving start state: somewhere along a feasible segment
    const int k = after(rng);
    for (int t = 0; t < k; ++t) gen.advance(kDt);
    const ReferenceState from = gen.state();
    const Vector7d target = randomTarget(rng, l);
    const auto T_opt = commonDuration(from, target, l);
    if (!gen.planTo(target, l)) {
      ++deferred;
      CHECK_FALSE(T_opt.has_value());
      CHECK(gen.state().q == from.q);  // unchanged
      continue;
    }
    ++planned;
    REQUIRE(T_opt.has_value());
    const double T = T_opt.value_or(-1.0);
    CHECK(gen.segmentDuration() == T);
    CHECK(T >= l.min_segment_duration_s);
    for (int j = 0; j < 7; ++j) CHECK(gen.segment(j).duration() == T);
    // the bisected step below T is infeasible: some joint violates a limit at T (1 - 1e-5)
    if (T > l.min_segment_duration_s * (1.0 + 1e-6)) {
      const double shorter = T * (1.0 - 1e-5);
      bool violates = false;
      for (int j = 0; j < 7; ++j) {
        const Quintic q = Quintic::toRest(from.q(j), from.dq(j), from.ddq(j), target(j), shorter);
        const Quintic::Extrema e = q.extrema();
        violates = violates || std::max(-e.v_min, e.v_max) > l.max_velocity(j) ||
                   std::max(-e.a_min, e.a_max) > l.max_acceleration(j);
      }
      CHECK(violates);
    }
    runToRest(gen, l);
    CHECK(gen.state().q == target);
  }
  MESSAGE("planned " << planned << ", deferred (no feasible quintic from that state) " << deferred);
  CHECK(planned > 300);
}

TEST_CASE("retargeting mid-segment keeps q, dq, ddq continuous") {
  std::mt19937_64 rng(5);
  const ReferenceLimits l = pandaLimits();
  std::uniform_int_distribution<int> period(33, 100);  // 10 - 30 Hz
  ReferenceGenerator gen;
  gen.reset(randomTarget(rng, l));
  int retargets = 0;
  for (int t = 0; t < 60000; ++t) {
    const ReferenceState prev = gen.state();
    gen.advance(kDt);
    REQUIRE(continuousStep(prev, gen.state(), kDt, l));
    if (t % period(rng) == 0) {
      const ReferenceState before = gen.state();
      Vector7d target = before.q + uniform(rng, Vector7d::Constant(-0.2), Vector7d::Constant(0.2));
      target = target.cwiseMax((l.lower.array() + 0.05).matrix()).cwiseMin((l.upper.array() - 0.05).matrix());
      if (gen.planTo(target, l)) {
        ++retargets;
        const ReferenceState &after = gen.state();  // same instant, new segment
        REQUIRE((after.q - before.q).cwiseAbs().maxCoeff() <= 1e-12);
        REQUIRE((after.dq - before.dq).cwiseAbs().maxCoeff() <= 1e-12);
        REQUIRE((after.ddq - before.ddq).cwiseAbs().maxCoeff() <= 1e-12);
      }
    }
    REQUIRE(withinVelocity(gen.state().dq, l.max_velocity));
    REQUIRE(withinAcceleration(gen.state().ddq, l.max_acceleration));
  }
  CHECK(retargets > 500);
}

TEST_CASE("min_segment_duration_s is respected; no motion needs no segment") {
  ReferenceLimits l = pandaLimits();
  l.min_segment_duration_s = 0.25;
  ReferenceGenerator gen;
  const Vector7d start =
      Vector7d::Constant(0.0).cwiseMax((l.lower.array() + 0.1).matrix()).cwiseMin((l.upper.array() - 0.1).matrix());
  gen.reset(start);
  REQUIRE(gen.planTo(start + Vector7d::Constant(1e-6), l));
  CHECK(gen.segmentDuration() == 0.25);
  runToRest(gen, l);
  REQUIRE(gen.planTo(gen.state().q, l));  // already there, at rest
  CHECK(gen.segmentDuration() == 0.0);
  CHECK(gen.atRest());
}

TEST_CASE("infeasible start state: planTo refuses and changes nothing") {
  const ReferenceLimits l = pandaLimits();
  ReferenceGenerator gen;
  const Vector7d start = (l.lower + l.upper) / 2;
  gen.reset(start);
  REQUIRE(gen.planTo(start + Vector7d::Constant(0.5), l));
  for (int t = 0; t < 600; ++t) gen.advance(kDt);
  const ReferenceState from = gen.state();
  ReferenceLimits slower = l;
  slower.max_velocity = from.dq.cwiseAbs() * 0.5;  // the current velocity is now above the limit
  CHECK_FALSE(gen.planTo(start, slower));
  CHECK(gen.state().q == from.q);
  CHECK(gen.state().dq == from.dq);
  CHECK(gen.goal() == start + Vector7d::Constant(0.5));
}

TEST_CASE("bounded stop: unimodal |v|, monotone to rest, within limits, no reversal") {
  std::mt19937_64 rng(6);
  const ReferenceLimits l = pandaLimits();
  std::uniform_int_distribution<int> after(1, 1500);
  for (int i = 0; i < 300; ++i) {
    ReferenceGenerator gen;
    gen.reset(randomTarget(rng, l));
    REQUIRE(gen.planTo(randomTarget(rng, l), l));
    const int k = after(rng);
    for (int t = 0; t < k; ++t) gen.advance(kDt);
    gen.planStop(l);
    std::array<bool, 7> falling{};
    Vector7d prev_speed = gen.state().dq.cwiseAbs();
    int ticks = 0;
    while (!gen.atRest() || gen.stopPending()) {
      const ReferenceState prev = gen.state();
      gen.advance(kDt);
      gen.retryStop(l);
      ++ticks;
      REQUIRE(ticks < 100000);
      const ReferenceState &s = gen.state();
      REQUIRE(withinVelocity(s.dq, l.max_velocity));
      REQUIRE(withinAcceleration(s.ddq, l.max_acceleration));
      REQUIRE(continuousStep(prev, s, kDt, l));
      for (int j = 0; j < 7; ++j) {
        const auto k = static_cast<size_t>(j);
        if (gen.stopPending(j)) {  // still on its old segment
          falling[k] = false;
          continue;
        }
        const auto &b = gen.segment(j).coefficients();  // b1 = v0 T, b2 = a0 T^2 / 2 of the stop
        const double sign = b[1] != 0.0 ? b[1] : b[2];
        REQUIRE(s.dq(j) * sign >= -1e-12 * std::abs(sign));  // never reverses
        const double speed = std::abs(s.dq(j));
        if (falling[k]) REQUIRE(speed <= prev_speed(j) + 1e-12);  // once falling, monotone to 0
        if (speed < prev_speed(j)) falling[k] = true;
      }
      prev_speed = s.dq.cwiseAbs();
    }
    CHECK(gen.state().dq.isZero(0.0));
    CHECK(gen.state().ddq.isZero(0.0));
    CHECK((gen.state().q.array() >= l.lower.array()).all());
    CHECK((gen.state().q.array() <= l.upper.array()).all());
  }
}

TEST_CASE("stop duration: at rest 0, decelerating always feasible") {
  CHECK(stopDuration({0.3, 0.0, 0.0}, 1.0, 2.0, 0.1, -3, 3).value_or(-1.0) == 0.0);
  // decelerating strongly with little velocity left: T = 3 |v0| / |a0| below min duration
  const auto T = stopDuration({0.0, 0.01, -1.9}, 0.6, 2.0, 0.1, -3, 3);
  REQUIRE(T.has_value());
  CHECK(T.value_or(-1.0) > 0.0);
  // at the velocity limit and still accelerating: no stop is feasible yet
  CHECK_FALSE(stopDuration({0.0, 0.6, 2.0}, 0.6, 2.0, 0.1, -3, 3).has_value());
}

TEST_CASE("double integrator lower bound") {
  using joint_controller::doubleIntegratorMinTime;
  CHECK(doubleIntegratorMinTime(1.0, 0.0, 1.0) == doctest::Approx(2.0));
  CHECK(doubleIntegratorMinTime(-1.0, 0.0, 1.0) == doctest::Approx(2.0));
  CHECK(doubleIntegratorMinTime(0.0, 1.0, 1.0) == doctest::Approx(1.0 + std::sqrt(2.0)));
  CHECK(doubleIntegratorMinTime(0.5, 1.0, 1.0) == doctest::Approx(1.0));  // brake exactly into it
}

TEST_CASE("planning cost (informational)") {
  std::mt19937_64 rng(7);
  const ReferenceLimits l = pandaLimits();
  double worst_us = 0.0;
  double total_us = 0.0;
  int n = 0;
  for (int i = 0; i < 2000; ++i) {
    ReferenceGenerator gen;
    gen.reset(randomTarget(rng, l));
    REQUIRE(gen.planTo(randomTarget(rng, l), l));
    for (int t = 0; t < 1 + (i % 900); ++t) gen.advance(kDt);
    const Vector7d target = randomTarget(rng, l);
    const auto t0 = std::chrono::steady_clock::now();
    (void)gen.planTo(target, l);
    const auto t1 = std::chrono::steady_clock::now();
    const double us = std::chrono::duration<double, std::micro>(t1 - t0).count();
    worst_us = std::max(worst_us, us);
    total_us += us;
    ++n;
  }
  std::printf("planTo: mean %.2f us, worst %.2f us over %d plans\n", total_us / n, worst_us, n);
  CHECK(worst_us < 50000.0);  // wall clock incl. OS jitter: only a sanity bound
  (void)kLimitSlack;
}
