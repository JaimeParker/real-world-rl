// SPDX-License-Identifier: MIT
// Shared helpers of the joint_controller C++ tests.
#pragma once

#include <Eigen/Dense>
#include <random>

#include "joint_controller/params.hpp"
#include "joint_controller/planner.hpp"

namespace joint_controller::test {

/// Relative slack of the limit checks: |v| <= max_velocity * (1 + kLimitSlack), same for |a|.
inline constexpr double kLimitSlack = 1e-9;

inline ReferenceLimits limitsOf(const TrajectoryImpedanceParams &p) {
  ReferenceLimits l;
  l.max_velocity = p.max_velocity;
  l.max_acceleration = p.max_acceleration;
  l.min_segment_duration_s = p.min_segment_duration_s;
  l.lower = p.lower_limit;
  l.upper = p.upper_limit;
  return l;
}

/// Uniform in [lo, hi] per joint.
inline Vector7d uniform(std::mt19937_64 &rng, const Vector7d &lo, const Vector7d &hi) {
  std::uniform_real_distribution<double> u(0.0, 1.0);
  Vector7d out;
  for (int i = 0; i < 7; ++i) out(i) = lo(i) + u(rng) * (hi(i) - lo(i));
  return out;
}

inline bool withinVelocity(const Vector7d &dq, const Vector7d &max_velocity) {
  return (dq.array().abs() <= max_velocity.array() * (1.0 + kLimitSlack)).all();
}

inline bool withinAcceleration(const Vector7d &ddq, const Vector7d &max_acceleration) {
  return (ddq.array().abs() <= max_acceleration.array() * (1.0 + kLimitSlack)).all();
}

/// Tick-to-tick continuity: a step of dt cannot move q / dq by more than the limits allow.
inline bool continuousStep(const ReferenceState &prev, const ReferenceState &next, double dt,
                           const ReferenceLimits &l) {
  const auto dq_ok =
      ((next.q - prev.q).array().abs() <= l.max_velocity.array() * dt * (1.0 + kLimitSlack) + 1e-15).all();
  const auto ddq_ok =
      ((next.dq - prev.dq).array().abs() <= l.max_acceleration.array() * dt * (1.0 + kLimitSlack) + 1e-15).all();
  return dq_ok && ddq_ok;
}

}  // namespace joint_controller::test
