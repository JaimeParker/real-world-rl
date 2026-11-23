// SPDX-License-Identifier: MIT AND Apache-2.0
// Copyright (c) real-world-rl contributors. The impedance law and velocity filter in update()
// are ported from GELLO's JointImpedanceController (wuphilipp/gello_software @ 9fc8c26,
// Copyright (c) 2025 Franka Robotics GmbH, Apache-2.0); see law.hpp and LICENSE.
#include "joint_controller/law.hpp"

#include <cmath>
#include <utility>

#include "joint_controller/params.hpp"
#include "joint_controller/planner.hpp"

namespace joint_controller {

TrajectoryImpedanceLaw::TrajectoryImpedanceLaw(TrajectoryImpedanceParams params) : params_(std::move(params)) {
  params_.validate();
  reference_.reset(Vector7d::Zero());
}

ReferenceLimits TrajectoryImpedanceLaw::limits() const noexcept {
  ReferenceLimits l;
  l.max_velocity = params_.max_velocity;
  l.max_acceleration = params_.max_acceleration;
  l.min_segment_duration_s = params_.min_segment_duration_s;
  l.lower = params_.lower_limit;
  l.upper = params_.upper_limit;
  return l;
}

void TrajectoryImpedanceLaw::applyValidatedParams(const TrajectoryImpedanceParams &params) noexcept {
  const TrajectoryImpedanceParams &old = params_;
  const bool limits_changed =
      old.max_velocity != params.max_velocity || old.max_acceleration != params.max_acceleration ||
      old.min_segment_duration_s != params.min_segment_duration_s || old.lower_limit != params.lower_limit ||
      old.upper_limit != params.upper_limit || old.limit_margin != params.limit_margin;
  params_ = params;
  if (limits_changed && has_target_) {
    target_ = raw_target_.cwiseMax(params_.targetLower()).cwiseMin(params_.targetUpper());
    if (!stopped_) target_pending_ = true;  // replan to the re-clipped target with the new limits
  }
}

void TrajectoryImpedanceLaw::applyParams(const TrajectoryImpedanceParams &params) {
  params.validate();
  applyValidatedParams(params);
}

void TrajectoryImpedanceLaw::starting(const Vector7d &q, const Vector7d &tau_prev) noexcept {
  reference_.reset(q);
  last_input_ = Input{};
  last_input_.q = q;
  dq_filtered_.setZero();  // GELLO: dq_filtered_.setZero() in on_activate()
  tau_prev_ = tau_prev.allFinite() ? tau_prev : Vector7d::Zero();
  time_since_target_ = 0.0;
  ticks_ = 0;
  plan_retries_ = 0;
  target_pending_ = false;
  braking_ = false;
  timed_out_ = false;
  stopped_ = false;
}

bool TrajectoryImpedanceLaw::setTarget(const Vector7d &q) noexcept {
  if (!q.allFinite()) return false;
  raw_target_ = q;
  target_ = q.cwiseMax(params_.targetLower()).cwiseMin(params_.targetUpper());
  has_target_ = true;
  new_target_ = true;
  stop_requested_ = false;  // the newest command wins
  return true;
}

void TrajectoryImpedanceLaw::requestStop() noexcept {
  stop_requested_ = true;
  new_target_ = false;  // the newest command wins
}

void TrajectoryImpedanceLaw::beginStop() noexcept {
  target_pending_ = false;
  braking_ = false;
  reference_.planStop(limits());
}

Vector7d TrajectoryImpedanceLaw::update(const Input &in) noexcept {
  const double dt = (std::isfinite(in.dt) && in.dt > 0.0) ? in.dt : 0.0;
  last_input_ = in;

  // 1. reference: advance to this tick, then apply events at this instant (continuous state).
  reference_.advance(dt);
  ++ticks_;
  if (stop_requested_) {
    stop_requested_ = false;
    stopped_ = true;
    timed_out_ = false;
    beginStop();
  }
  if (new_target_) {
    new_target_ = false;
    time_since_target_ = 0.0;
    target_pending_ = true;  // braking_ (if set) carries over: keep braking until a plan succeeds
    timed_out_ = false;
    stopped_ = false;
  } else {
    time_since_target_ += dt;
  }
  // 2. timeout -> bounded stop and hold
  if (!stopped_ && time_since_target_ >= params_.command_timeout_s) {
    timed_out_ = true;
    stopped_ = true;
    beginStop();
  }
  if (target_pending_) {
    if (reference_.planTo(target_, limits())) {
      target_pending_ = false;
      braking_ = false;
    } else {
      // No quintic from this state reaches the target within the limits (e.g. still
      // accelerating near max_velocity): brake (bounded stop) and retry every tick; at rest
      // every target is feasible.
      ++plan_retries_;
      if (!braking_) {
        braking_ = true;
        reference_.planStop(limits());
      } else if (reference_.stopPending()) {
        reference_.retryStop(limits());
      }
    }
  } else if (reference_.stopPending()) {
    reference_.retryStop(limits());
  }

  if (!in.q.allFinite() || !in.dq.allFinite() || (params_.compensate_coriolis && !in.coriolis.allFinite())) {
    return tau_prev_;  // a corrupt measurement never reaches the filter; hold the last command
  }

  // 3. velocity filter (GELLO)
  const double alpha = params_.velocity_filter_alpha;
  dq_filtered_ = (1.0 - alpha) * dq_filtered_ + alpha * in.dq;

  // 4. joint impedance (GELLO) + coriolis, then the torque-rate limit
  const ReferenceState &ref = reference_.state();
  Vector7d tau = params_.stiffness.cwiseProduct(ref.q - in.q) + params_.damping.cwiseProduct(-dq_filtered_);
  if (params_.compensate_coriolis) tau += in.coriolis;
  const double m = params_.max_delta_tau;
  tau_prev_ += (tau - tau_prev_).cwiseMax(-m).cwiseMin(m);
  return tau_prev_;
}

JointTelemetry TrajectoryImpedanceLaw::telemetry() const noexcept {
  JointTelemetry t;
  t.ticks = ticks_;
  t.q = last_input_.q;
  t.dq = last_input_.dq;
  t.dq_filtered = dq_filtered_;
  t.coriolis = last_input_.coriolis;
  const ReferenceState &ref = reference_.state();
  t.q_ref = ref.q;
  t.dq_ref = ref.dq;
  t.ddq_ref = ref.ddq;
  t.target = has_target_ ? target_ : reference_.goal();
  t.goal = reference_.goal();
  t.tau_cmd = tau_prev_;
  t.segment_t = reference_.segmentTime();
  t.segment_T = reference_.segmentDuration();
  t.time_since_target = time_since_target_;
  t.timed_out = timed_out_;
  t.stopped = stopped_;
  t.target_pending = target_pending_;
  t.plan_retries = plan_retries_;
  return t;
}

}  // namespace joint_controller
