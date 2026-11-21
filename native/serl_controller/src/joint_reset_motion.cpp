// SPDX-License-Identifier: MIT
// Copyright (c) 2024 RAIL Berkeley (serl_franka_controllers); port: real-world-rl contributors.
// The franka_hw position-interface tick around the port of serl_franka_controllers @ 1f140ef
// src/joint_position_controller.cpp (see joint_reset.hpp). MIT License (see LICENSE).
#include "serl_controller/joint_reset_motion.hpp"

#include <array>
#include <atomic>
#include <cstdint>
#include <optional>

#include "franka/control_types.h"
#include "franka/duration.h"
#include "franky/robot_state.hpp"
#include "serl_controller/joint_reset.hpp"

namespace serl_controller {

SerlJointPositionMotion::SerlJointPositionMotion(const Vector7d &target, const JointLimits7 &limits)
    : target_(target), law_(target), soft_limits_(limits) {
  snapshot_buffer_.setUnsafe(SerlJointResetSnapshot{});  // constructor: no concurrency yet
}

void SerlJointPositionMotion::initImpl(const franky::RobotState &robot_state,
                                       const std::optional<franka::JointPositions> & /*previous_command*/) {
  // franka_hw's JointHandle::getPosition() is franka::RobotState::q (measured).
  law_.starting(robot_state.q);
  soft_limits_.reset();
  tick_ = 0;
  soft_limited_ticks_ = 0;
  finished_ = false;
}

SerlJointResetCommand SerlJointPositionMotion::computeCommand(const Vector7d &q_measured, uint64_t time_step_ms) {
  Vector7d command;
  if (finish_requested_.load(std::memory_order_relaxed) && tick_ > 0) {
    command = last_command_;  // hold the previous command; the control loop ends here
    finished_ = true;
  } else {
    const int64_t period_ns = SerlJointPositionLaw::periodNsFromMilliseconds(time_step_ms);
    command = law_.update(period_ns);
    // franka_control_node: period 0 -> reset(); otherwise enforceLimits(period) after update().
    if (period_ns == 0) {
      soft_limits_.reset();
    } else {
      const Vector7d limited = soft_limits_.enforce(command, q_measured, period_ns);
      if (limited != command) ++soft_limited_ticks_;
      command = limited;
    }
    last_command_ = command;
    ++tick_;
  }
  SerlJointResetSnapshot snap;
  snap.tick = tick_;
  snap.elapsed_s = law_.elapsedSec();
  snap.q = q_measured;
  snap.command = command;
  snap.initial = law_.initial();
  snap.controller_command = law_.command();
  snap.soft_limited_ticks = soft_limited_ticks_;
  snap.finished = finished_;
  snapshot_buffer_.setUnsafe(snap);  // single RT writer, wait-free
  return SerlJointResetCommand{command, finished_};
}

franka::JointPositions SerlJointPositionMotion::nextCommandImpl(
    const franky::RobotState &robot_state, franka::Duration time_step, franka::Duration /*rel_time*/,
    franka::Duration /*abs_time*/, const std::optional<franka::JointPositions> & /*previous_command*/) {
  const SerlJointResetCommand tick = computeCommand(robot_state.q, time_step.toMSec());
  std::array<double, 7> q{};
  Eigen::Map<Vector7d>(q.data()) = tick.command;
  if (tick.finished) return franka::MotionFinished(franka::JointPositions(q));
  return {q};
}

}  // namespace serl_controller
