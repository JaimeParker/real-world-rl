// SPDX-License-Identifier: MIT AND Apache-2.0
// Copyright (c) real-world-rl contributors.
// The joint impedance law and its velocity filter (steps 3 and 4 below) are ported from the
// public GELLO controller, wuphilipp/gello_software @ 9fc8c26
// ros2/src/franka_fr3_arm_controllers/src/joint_impedance_controller.cpp
// (JointImpedanceController::calculateTauDGains_), Copyright (c) 2025 Franka Robotics GmbH,
// Apache License 2.0:
//   dq_filtered_ = (1 - k_alpha_) * dq_filtered_ + k_alpha_ * dq_;
//   tau_d = k_gains_.cwiseProduct(q_goal - q_) + d_gains_.cwiseProduct(-dq_filtered_);
// with dq_filtered_ zeroed on activation. The reference generator, the coriolis term, the
// torque-rate limit, the timeout stop and everything else here are new code (MIT). See LICENSE.
//
// TrajectoryImpedanceLaw: per 1 kHz tick (dt = the measured period)
//   1. reference: advance (q_ref, dq_ref, ddq_ref) by dt; apply pending events at this instant:
//      a new target (finite, clipped into [lower + margin, upper - margin]) replans every joint
//      from the current reference state to (target, 0, 0) with one common duration within
//      max_velocity / max_acceleration and the joint limits (ReferenceGenerator::planTo). If no
//      duration is feasible from the current state, the target stays pending, the reference
//      brakes (the bounded stop of step 2) and the plan is retried every tick; at rest it always
//      succeeds;
//   2. timeout: no new target for command_timeout_s -> bounded stop (ReferenceGenerator::planStop)
//      and hold; request_stop does the same at once; the next target resumes tracking;
//   3. velocity filter: dq_f = (1 - alpha) dq_f + alpha dq;
//   4. torque: tau = K (q_ref - q) - D dq_f (+ coriolis if compensate_coriolis; never gravity:
//      libfranka adds it), then per joint |tau - tau_prev| <= max_delta_tau.
// starting(q) seeds the reference at the measured q, at rest, so the spring term is 0 at tick 0.
//
// Real-time use: update() is noexcept, fixed-size Eigen only (no allocation), bounded work.
#pragma once

#include <Eigen/Dense>
#include <cstdint>

#include "joint_controller/params.hpp"
#include "joint_controller/planner.hpp"

namespace joint_controller {

/// The state of the law after a tick (fixed size; the motion publishes one per tick).
struct JointTelemetry {
  uint64_t ticks{0};             // update() calls since starting()
  double rel_time{0.0};          // motion only: franky's rel_time of this tick
  Vector7d q{Vector7d::Zero()};  // measured (last update() input)
  Vector7d dq{Vector7d::Zero()};
  Vector7d dq_filtered{Vector7d::Zero()};
  Vector7d coriolis{Vector7d::Zero()};
  Vector7d q_ref{Vector7d::Zero()};
  Vector7d dq_ref{Vector7d::Zero()};
  Vector7d ddq_ref{Vector7d::Zero()};
  Vector7d target{Vector7d::Zero()};  // last accepted target, clipped (the measured q before any)
  Vector7d goal{Vector7d::Zero()};    // where the reference comes to rest (target or stop point)
  Vector7d tau_cmd{Vector7d::Zero()};
  double segment_t{0.0};  // time since the last plan
  double segment_T{0.0};  // its duration
  double time_since_target{0.0};
  bool timed_out{false};
  bool stopped{false};  // bounded stop active or holding (timeout or request_stop)
  bool target_pending{false};
  uint64_t plan_retries{0};
  uint64_t applied_command_seq{0};  // motion only
  uint64_t applied_params_seq{0};   // motion only
};

class TrajectoryImpedanceLaw {
 public:
  struct Input {
    Vector7d q{Vector7d::Zero()};         // measured joint positions
    Vector7d dq{Vector7d::Zero()};        // measured joint velocities
    Vector7d coriolis{Vector7d::Zero()};  // model coriolis torque at this state
    double dt{0.001};                     // period since the previous tick [s]
  };

  /// `params` must be valid (TrajectoryImpedanceParams::validate()); throws std::invalid_argument
  /// otherwise.
  explicit TrajectoryImpedanceLaw(TrajectoryImpedanceParams params);

  /// Replace the parameters (must already be validated). Takes effect immediately; a change of
  /// the reference limits replans the current target (re-clipped) from the current state.
  void applyValidatedParams(const TrajectoryImpedanceParams &params) noexcept;
  /// Validate, then applyValidatedParams. Throws std::invalid_argument (nothing changes).
  void applyParams(const TrajectoryImpedanceParams &params);

  /// Seed the reference at the measured q, at rest; zero the filter; previous torque = tau_prev.
  /// A target given before starting() is kept and applied at the first update().
  void starting(const Vector7d &q, const Vector7d &tau_prev = Vector7d::Zero()) noexcept;

  /// New joint target, applied at the next update(). Returns false (and changes nothing) if a
  /// value is not finite. The target is clipped into the limit envelope.
  bool setTarget(const Vector7d &q) noexcept;

  /// Bounded stop to rest at the next update() and hold; the next setTarget() resumes.
  void requestStop() noexcept;

  /// One tick. Returns the commanded torque (without gravity).
  [[nodiscard]] Vector7d update(const Input &in) noexcept;

  /// Telemetry of the last update() (motion-only fields left at 0).
  [[nodiscard]] JointTelemetry telemetry() const noexcept;

  // ---- state ----
  [[nodiscard]] const TrajectoryImpedanceParams &params() const noexcept { return params_; }
  [[nodiscard]] const ReferenceState &reference() const noexcept { return reference_.state(); }
  [[nodiscard]] const ReferenceGenerator &generator() const noexcept { return reference_; }
  /// The clipped target (or the stop point after a stop).
  [[nodiscard]] const Vector7d &target() const noexcept { return reference_.goal(); }
  [[nodiscard]] const Vector7d &lastTarget() const noexcept { return target_; }
  [[nodiscard]] const Vector7d &tauCommand() const noexcept { return tau_prev_; }
  [[nodiscard]] const Vector7d &dqFiltered() const noexcept { return dq_filtered_; }
  [[nodiscard]] double segmentTime() const noexcept { return reference_.segmentTime(); }
  [[nodiscard]] double segmentDuration() const noexcept { return reference_.segmentDuration(); }
  [[nodiscard]] bool timedOut() const noexcept { return timed_out_; }
  /// A bounded stop (timeout or requestStop) is active or holding.
  [[nodiscard]] bool stopped() const noexcept { return stopped_; }
  /// A target waits for a feasible plan.
  [[nodiscard]] bool targetPending() const noexcept { return target_pending_; }
  [[nodiscard]] double timeSinceTarget() const noexcept { return time_since_target_; }
  [[nodiscard]] uint64_t ticks() const noexcept { return ticks_; }
  /// Ticks on which a pending target could not be planned yet (since starting()).
  [[nodiscard]] uint64_t planRetries() const noexcept { return plan_retries_; }

 private:
  [[nodiscard]] ReferenceLimits limits() const noexcept;
  void beginStop() noexcept;

  TrajectoryImpedanceParams params_;
  ReferenceGenerator reference_;
  Input last_input_{};
  Vector7d raw_target_{Vector7d::Zero()};  // last accepted target as given
  Vector7d target_{Vector7d::Zero()};      // ... clipped into the limit envelope
  Vector7d dq_filtered_{Vector7d::Zero()};
  Vector7d tau_prev_{Vector7d::Zero()};
  double time_since_target_{0.0};
  uint64_t ticks_{0};
  uint64_t plan_retries_{0};
  bool has_target_{false};
  bool new_target_{false};      // setTarget() since the last update()
  bool stop_requested_{false};  // requestStop() since the last update()
  bool target_pending_{false};  // accepted, waiting for a feasible plan
  bool braking_{false};         // braking while target_pending_ (planStop until a plan succeeds)
  bool timed_out_{false};
  bool stopped_{false};
};

}  // namespace joint_controller
