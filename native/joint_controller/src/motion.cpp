// SPDX-License-Identifier: MIT AND Apache-2.0
// Copyright (c) real-world-rl contributors. The franky torque-loop tick around
// TrajectoryImpedanceLaw (GELLO's impedance law, Copyright (c) 2025 Franka Robotics GmbH,
// Apache-2.0; see law.hpp and LICENSE).
#include "joint_controller/motion.hpp"

#include <array>
#include <mutex>
#include <optional>

#include "franka/control_types.h"
#include "franka/duration.h"
#include "franky/robot.hpp"
#include "franky/robot_state.hpp"
#include "joint_controller/law.hpp"
#include "joint_controller/params.hpp"

namespace joint_controller {

TrajectoryImpedanceMotion::TrajectoryImpedanceMotion(const TrajectoryImpedanceParams &params)
    : requested_params_([&params] {
        params.validate();
        return params;
      }()),
      law_(requested_params_) {
  params_buffer_.setUnsafe(ParamsMsg{0, requested_params_});  // constructor: no concurrency yet
  command_buffer_.setUnsafe(CommandMsg{});
  telemetry_buffer_.setUnsafe(JointTelemetry{});
}

void TrajectoryImpedanceMotion::publishCommand(bool stop, const Vector7d &q) {
  const std::lock_guard lock(user_mutex_);
  CommandMsg msg;
  msg.seq = ++command_seq_;
  msg.stop = stop;
  msg.q = q;
  command_buffer_.set(msg);
}

bool TrajectoryImpedanceMotion::setTarget(const Vector7d &q) {
  if (!q.allFinite()) return false;
  publishCommand(false, q);
  return true;
}

void TrajectoryImpedanceMotion::requestStop() { publishCommand(true, Vector7d::Zero()); }

TrajectoryImpedanceParams TrajectoryImpedanceMotion::updateParams(const ParamUpdates &updates) {
  const std::lock_guard lock(user_mutex_);
  TrajectoryImpedanceParams next = requested_params_;
  next.merge(updates);  // validates; throws without side effects
  requested_params_ = next;
  params_buffer_.set(ParamsMsg{++params_seq_, next});
  return next;
}

TrajectoryImpedanceParams TrajectoryImpedanceMotion::params() const {
  const std::lock_guard lock(user_mutex_);
  return requested_params_;
}

JointTelemetry TrajectoryImpedanceMotion::telemetry() { return telemetry_buffer_.get(); }

void TrajectoryImpedanceMotion::applyPendingParams() noexcept {
  if (!params_buffer_.hasNewData()) return;
  const ParamsMsg msg = params_buffer_.getUnsafe();
  if (msg.seq == applied_params_seq_) return;
  law_.applyValidatedParams(msg.params);  // validated on the user thread (updateParams / constructor)
  applied_params_seq_ = msg.seq;
}

void TrajectoryImpedanceMotion::applyPendingCommand() noexcept {
  if (!command_buffer_.hasNewData()) return;
  const CommandMsg msg = command_buffer_.getUnsafe();
  if (msg.seq == applied_command_seq_) return;
  if (msg.stop) {
    law_.requestStop();
  } else {
    (void)law_.setTarget(msg.q);  // finite: checked by setTarget() on the user thread
  }
  applied_command_seq_ = msg.seq;
}

void TrajectoryImpedanceMotion::initImpl(const franky::RobotState &robot_state,
                                         const std::optional<franka::Torques> &previous_command) {
  applyPendingParams();
  Vector7d tau_prev = Vector7d::Zero();
  if (previous_command.has_value()) tau_prev = Eigen::Map<const Vector7d>(previous_command->tau_J.data());
  // Seeds the reference at the measured q. A command sent before the start stays in its mailbox
  // and is applied on the first tick.
  law_.starting(robot_state.q, tau_prev);
  JointTelemetry t = law_.telemetry();
  t.applied_params_seq = applied_params_seq_;
  telemetry_buffer_.setUnsafe(t);
}

Vector7d TrajectoryImpedanceMotion::computeTorques(const Vector7d &q, const Vector7d &dq, const Vector7d &coriolis,
                                                   double dt, double rel_time) noexcept {
  applyPendingParams();
  applyPendingCommand();
  TrajectoryImpedanceLaw::Input in;
  in.q = q;
  in.dq = dq;
  in.coriolis = coriolis;
  in.dt = dt;
  const Vector7d tau = law_.update(in);
  JointTelemetry t = law_.telemetry();
  t.rel_time = rel_time;
  t.applied_command_seq = applied_command_seq_;
  t.applied_params_seq = applied_params_seq_;
  telemetry_buffer_.setUnsafe(t);  // single RT writer, wait-free
  return tau;
}

franka::Torques TrajectoryImpedanceMotion::nextCommandImpl(
    const franky::RobotState &robot_state, franka::Duration time_step, franka::Duration rel_time,
    franka::Duration /*abs_time*/, const std::optional<franka::Torques> & /*previous_command*/) {
  // The model's coriolis torque at the current state (libfranka adds gravity itself).
  const auto model = robot()->model();  // robot(): non-owning, set by franky for the motion's run
  const Vector7d coriolis = model->coriolis(robot_state);
  const Vector7d tau = computeTorques(robot_state.q, robot_state.dq, coriolis, time_step.toSec(), rel_time.toSec());
  std::array<double, 7> tau_array{};
  Eigen::Map<Vector7d>(tau_array.data()) = tau;
  return {tau_array};
}

}  // namespace joint_controller
