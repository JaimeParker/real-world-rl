// SPDX-License-Identifier: MIT
// Copyright (c) real-world-rl contributors.
//
// Online joint reference generator: one quintic polynomial per joint, replanned from the current
// reference state (position, velocity, acceleration) whenever a new target arrives, with a common
// duration chosen so that every joint respects its velocity and acceleration limit over the whole
// segment. Derived from the standard quintic boundary-value problem (no third-party code).
//
// Boundary-value problem. With s = t / T in [0, 1] and h = pf - p0, the quintic
//   q(s) = b0 + b1 s + b2 s^2 + b3 s^3 + b4 s^4 + b5 s^5
// with q(0) = p0, q'(0) = v0, q''(0) = a0 (time derivatives) and q(1) = pf, q'(1) = q''(1) = 0 is
//   b0 = p0,  b1 = v0 T,  b2 = a0 T^2 / 2,
//   b3 =  10 h - 6 v0 T - 1.5 a0 T^2,
//   b4 = -15 h + 8 v0 T + 1.5 a0 T^2,
//   b5 =   6 h - 3 v0 T - 0.5 a0 T^2.
// Velocity and acceleration are q_s / T and q_ss / T^2. Their extrema over [0, 1] are at the
// end points and at the real roots of the next derivative: q_sss is a quadratic (closed form);
// between consecutive roots of q_sss, q_ss is monotone and has at most one root (found by a
// bracketed Newton iteration), a velocity extremum; between consecutive velocity extrema q_s is
// monotone, and its root is a position extremum. Quintic::extrema() evaluates the polynomial at
// all of these points, so the feasibility test is exact up to rounding, not an analytic bound
// (Quintic::within() first tries the cheap sufficient bound sum |c_k| and falls back to it).
//
// Feasibility of a segment: per joint |v| <= max_velocity and |a| <= max_acceleration over the
// whole segment, and the position stays within [lower, upper] (the joint limits; widened to the
// start point if the reference starts outside them), so a fast approach of a target near a limit
// cannot overshoot past the limit.
//
// Duration selection (commonDuration). T starts at the largest of min_segment_duration_s and,
// per joint, two necessary conditions that hold for ANY trajectory: |h| / v_max (average speed)
// and the minimum time of a double integrator with |a| <= a_max from (p0, v0) to rest at pf
// (bang-bang). From there T grows geometrically (x kGridGrowth) until every joint's quintic is
// feasible; the step between the last infeasible and the first feasible grid point is then
// bisected (kBisectionSteps) and the feasible end is returned. Below the start no T can be
// feasible; from rest (v0 = a0 = 0) feasibility is monotone in T and the result is the exact
// minimum; from a moving state a feasible window narrower than one grid step could in principle be
// skipped. If no grid point up to kGridSteps steps is feasible -- the joints' feasible windows do
// not overlap, e.g. one joint still accelerating while another needs a long segment -- planTo()
// returns false and leaves the reference unchanged; the law then brakes (planStop) and retries
// every tick. At rest every target inside the limits is feasible.
//
// Bounded stop (planStop). Per joint, from (p0, v0, a0) to rest at p0 + v0 T / 2 + a0 T^2 / 12
// (the member of the boundary-value family with b5 = 0). Its velocity
//   v(s) = v0 (1 - 3 s^2 + 2 s^3) + a0 T s (1 - s)^2
// has a derivative (1 - s)(a0 T (1 - 3 s) - 6 v0 s) with one sign change at most, so |v| is
// unimodal: it rises only while the reference is still accelerating, then falls monotonically to
// 0. T is the smallest grid point >= min_segment_duration_s (and >= |v0| / a_max) for which the
// limits hold, the velocity never reverses sign and the stop stays within the position limits;
// when the joint is decelerating, T = 3 |v0| / |a0| (v(s) = v0 (1 - s)^3, |a| <= |a0|) is tried as
// well. Each joint stops with its own duration (the stop is not coordinated); a joint for which
// no stop is feasible yet keeps its current (feasible, rest-ending) segment and is retried
// (retryStop()) every tick.
//
// Real-time use: no allocation, no exceptions, bounded work (at most kGridSteps + kBisectionSteps
// feasibility checks per plan).
#pragma once

#include <Eigen/Dense>
#include <array>
#include <cstddef>
#include <optional>

namespace joint_controller {

using Vector7d = Eigen::Matrix<double, 7, 1>;

/// Position, velocity and acceleration of one joint.
struct JointSample {
  double p{0.0};
  double v{0.0};
  double a{0.0};
};

/// One joint's segment: q(t) = sum_k b_k (t / T)^k for 0 <= t < T, then rest at end().
class Quintic {
 public:
  /// At rest at p (duration 0).
  [[nodiscard]] static Quintic hold(double p) noexcept;
  /// (p0, v0, a0) at t = 0 to (pf, 0, 0) at t = T (T > 0).
  [[nodiscard]] static Quintic toRest(double p0, double v0, double a0, double pf, double T) noexcept;
  /// Bounded stop: (p0, v0, a0) to rest at p0 + v0 T / 2 + a0 T^2 / 12 at t = T (T > 0).
  [[nodiscard]] static Quintic stop(double p0, double v0, double a0, double T) noexcept;

  /// The sample at segment time t (t >= T: exactly (end(), 0, 0)).
  [[nodiscard]] JointSample sample(double t) const noexcept;
  [[nodiscard]] double duration() const noexcept { return duration_; }
  [[nodiscard]] double end() const noexcept { return end_; }
  [[nodiscard]] const std::array<double, 6> &coefficients() const noexcept { return b_; }

  struct Extrema {
    double v_min{0.0};
    double v_max{0.0};
    double a_min{0.0};
    double a_max{0.0};
    double p_min{0.0};
    double p_max{0.0};
  };
  /// Minimum and maximum velocity, acceleration and position over [0, T].
  [[nodiscard]] Extrema extrema() const noexcept;
  /// |v| <= v_max, |a| <= a_max and p in [p_lower, p_upper] over [0, T] (a cheap sufficient
  /// bound first, then extrema()).
  [[nodiscard]] bool within(double v_max, double a_max, double p_lower, double p_upper) const noexcept;

 private:
  std::array<double, 6> b_{};
  double duration_{0.0};
  double end_{0.0};
};

/// Limits of the reference generator.
struct ReferenceLimits {
  Vector7d max_velocity{Vector7d::Ones()};
  Vector7d max_acceleration{Vector7d::Ones()};
  double min_segment_duration_s{0.1};
  Vector7d lower{Vector7d::Constant(-1e9)};  // position limits (the stop point stays within them)
  Vector7d upper{Vector7d::Constant(1e9)};
};

struct ReferenceState {
  Vector7d q{Vector7d::Zero()};
  Vector7d dq{Vector7d::Zero()};
  Vector7d ddq{Vector7d::Zero()};
};

inline constexpr double kGridGrowth = 1.1;
inline constexpr int kGridSteps = 50;  // up to kGridGrowth^50 (~117x) above the lower bound
inline constexpr int kBisectionSteps = 20;

/// Minimum time of x'' = u, |u| <= a_max, from (0, v0) to rest at h (a lower bound for any
/// trajectory that respects a_max).
[[nodiscard]] double doubleIntegratorMinTime(double h, double v0, double a_max) noexcept;

/// The common duration planTo() would choose from `from` to (target, 0, 0), or nullopt if none is
/// feasible (see the header comment). A state with every joint already at its target and at rest
/// gives 0.
[[nodiscard]] std::optional<double> commonDuration(const ReferenceState &from, const Vector7d &target,
                                                   const ReferenceLimits &limits) noexcept;

/// The duration planStop() would choose for one joint, or nullopt (not feasible yet). 0 if the
/// joint is at rest.
[[nodiscard]] std::optional<double> stopDuration(const JointSample &from, double max_velocity, double max_acceleration,
                                                 double min_duration, double lower, double upper) noexcept;

class ReferenceGenerator {
 public:
  /// At rest at q.
  void reset(const Vector7d &q) noexcept;
  /// Advance every joint by dt seconds along its segment (dt < 0 or non-finite counts as 0).
  void advance(double dt) noexcept;
  [[nodiscard]] const ReferenceState &state() const noexcept { return state_; }

  /// Replan every joint from the current state to (target, 0, 0) with one common duration.
  /// Returns false, changing nothing, if no feasible duration exists (see commonDuration).
  bool planTo(const Vector7d &target, const ReferenceLimits &limits) noexcept;
  /// Start a bounded stop of every joint; joints without a feasible stop yet keep their segment
  /// and are retried by retryStop().
  void planStop(const ReferenceLimits &limits) noexcept;
  /// Retry the stop of the joints planStop() could not stop yet. Returns stopPending().
  bool retryStop(const ReferenceLimits &limits) noexcept;
  [[nodiscard]] bool stopPending() const noexcept;
  [[nodiscard]] bool stopPending(int joint) const noexcept { return stop_pending_[static_cast<size_t>(joint)]; }

  /// Where each joint comes to rest (the target, or the stop point).
  [[nodiscard]] const Vector7d &goal() const noexcept { return goal_; }
  /// Time since the last plan.
  [[nodiscard]] double segmentTime() const noexcept { return segment_time_; }
  /// Duration of the last plan (stop: until the last joint is at rest, as far as planned).
  [[nodiscard]] double segmentDuration() const noexcept { return segment_duration_; }
  /// Every joint at the end of its segment (at rest).
  [[nodiscard]] bool atRest() const noexcept;
  [[nodiscard]] const Quintic &segment(int joint) const noexcept { return segments_[static_cast<size_t>(joint)]; }

 private:
  bool tryStopJoint(int joint, const ReferenceLimits &limits) noexcept;
  void sampleAll() noexcept;

  std::array<Quintic, 7> segments_{};
  std::array<double, 7> time_{};  // per joint, time in its segment
  std::array<bool, 7> stop_pending_{};
  ReferenceState state_{};
  Vector7d goal_{Vector7d::Zero()};
  double segment_time_{0.0};
  double segment_duration_{0.0};
};

}  // namespace joint_controller
