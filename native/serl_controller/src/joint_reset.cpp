// SPDX-License-Identifier: MIT AND BSD-3-Clause AND Apache-2.0
// Copyright (c) 2024 RAIL Berkeley (serl_franka_controllers, MIT);
// Copyright (C) 2013, PAL Robotics S.L. and Copyright (c) 2008, Willow Garage, Inc.
//   (ros_control joint_limits_interface, BSD-3-Clause; full notice in src/joint_reset.cpp);
// joint limit values from franka_ros (Copyright (c) 2023 Franka Robotics GmbH, Apache-2.0);
// port: real-world-rl contributors. See LICENSE in this package.
//
// SerlJointPositionLaw is ported from https://github.com/rail-berkeley/serl_franka_controllers
// @ 1f140ef, src/joint_position_controller.cpp (init / starting / update), with the ROS
// plumbing removed. MIT License.
//
// Keep the arithmetic as in the original: tests/cpp/test_joint_reset_golden.cpp
// compares this file against the vendored original tick by tick.
#include "serl_controller/joint_reset.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>

namespace serl_controller {

namespace {
constexpr int64_t kNsPerSec = 1000000000LL;
}  // namespace

SerlJointPositionLaw::SerlJointPositionLaw(const Vector7d &target) : reset_pose_(target) {
  if (!target.allFinite()) throw std::invalid_argument("SERL joint reset target must be finite");
}

void SerlJointPositionLaw::starting(const Vector7d &q) noexcept {
  initial_pose_ = q;
  elapsed_ns_ = 0;  // elapsed_time_ = ros::Duration(0.0)
}

int64_t SerlJointPositionLaw::periodNsFromMilliseconds(uint64_t milliseconds) noexcept {
  // ros::Duration(double d) rounds (d - floor(d)) * 1e9 to the nearest nanosecond;
  // for d = ms / 1000.0 that is exactly ms * 1e6 (checked by the golden test).
  return static_cast<int64_t>(milliseconds) * 1000000LL;
}

double SerlJointPositionLaw::elapsedSec() const noexcept {
  // ros::DurationBase::toSec() of the normalised (sec, nsec) pair, nsec in [0, 1e9).
  const int64_t sec = elapsed_ns_ / kNsPerSec;
  const int64_t nsec = elapsed_ns_ % kNsPerSec;
  return static_cast<double>(static_cast<int32_t>(sec)) + 1e-9 * static_cast<double>(static_cast<int32_t>(nsec));
}

SerlJointPositionLaw::Vector7d SerlJointPositionLaw::update(int64_t period_ns) {
  if (period_ns < 0) throw std::invalid_argument("period must be non-negative");
  elapsed_ns_ += period_ns;  // elapsed_time_ += period

  for (int i = 0; i < 7; ++i) {
    if (elapsedSec() > 10) {
      command_(i) = reset_pose_(i);
    } else {
      command_(i) = ((elapsedSec()) / 10.0) * reset_pose_(i) + ((10 - elapsedSec()) / 10.0) * initial_pose_(i);
    }
  }
  return command_;
}

// ---------------------------------------------------------------------------
// franka_hw's PositionJointSoftLimitsInterface. Ported from ros-controls/ros_control
// noetic-devel @ 633c374, joint_limits_interface/include/joint_limits_interface/
// joint_limits_interface.h (PositionJointSoftLimitsHandle::enforceLimits / reset,
// internal::saturate). The original's notice, which applies to this ported part:
//
// Copyright (C) 2013, PAL Robotics S.L.
// Copyright (c) 2008, Willow Garage, Inc.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of PAL Robotics S.L. nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// (ros_control's repository LICENSE: tests/cpp/golden/ros_control_original/LICENSE.)
//
// The limit values below are franka_ros @ ddd2fff, franka_description/robots/{panda,fr3}/
// joint_limits.yaml, Apache-2.0 (licence text: tests/cpp/golden/franka_ros_original/LICENSE).
//
// Keep the arithmetic in the same order: tests/cpp/test_joint_reset_golden.cpp
// compares it with the vendored original tick by tick.

namespace {

double saturate(const double val, const double min_val, const double max_val) noexcept {
  return std::min(std::max(val, min_val), max_val);
}

JointLimit joint(double lower, double upper, double velocity) noexcept {
  // franka-limits macro: soft limits == limits, k_position 100 (k_velocity 40 is unused here).
  return JointLimit{lower, upper, velocity, lower, upper, 100.0};
}

}  // namespace

JointLimits7 frankaPandaJointLimits() noexcept {
  return {joint(-2.8973, 2.8973, 2.1750),  joint(-1.7628, 1.7628, 2.1750), joint(-2.8973, 2.8973, 2.1750),
          joint(-3.0718, -0.0698, 2.1750), joint(-2.8973, 2.8973, 2.6100), joint(-0.0175, 3.7525, 2.6100),
          joint(-2.8973, 2.8973, 2.6100)};
}

JointLimits7 frankaFr3JointLimits() noexcept {
  return {joint(-2.3093, 2.3093, 2.0),   joint(-1.5133, 1.5133, 1.0),  joint(-2.4937, 2.4937, 1.50),
          joint(-2.7478, -0.4461, 1.25), joint(-2.4800, 2.4800, 3.00), joint(0.8521, 4.2094, 1.50),
          joint(-2.6895, 2.6895, 3.00)};
}

JointLimits7 frankaJointLimits(const std::string &robot_model) {
  if (robot_model == "panda") return frankaPandaJointLimits();
  if (robot_model == "fr3") return frankaFr3JointLimits();
  throw std::invalid_argument("robot_model must be 'panda' or 'fr3', got '" + robot_model + "'");
}

PositionJointSoftLimits::PositionJointSoftLimits(const JointLimits7 &limits) noexcept : limits_(limits) { reset(); }

void PositionJointSoftLimits::reset() noexcept { prev_cmd_.fill(std::numeric_limits<double>::quiet_NaN()); }

PositionJointSoftLimits::Vector7d PositionJointSoftLimits::enforce(const Vector7d &command, const Vector7d &q_measured,
                                                                   int64_t period_ns) {
  if (period_ns <= 0) throw std::invalid_argument("enforce needs a positive period");
  // period.toSec() of the ros::Duration franka_hw passes
  const double dt = static_cast<double>(static_cast<int32_t>(period_ns / kNsPerSec)) +
                    1e-9 * static_cast<double>(static_cast<int32_t>(period_ns % kNsPerSec));
  Vector7d out;
  for (int i = 0; i < 7; ++i) {
    const JointLimit &limits = limits_[static_cast<size_t>(i)];
    if (std::isnan(prev_cmd_[i])) prev_cmd_[i] = q_measured(i);  // happens once after reset()
    const double pos = prev_cmd_[i];
    const double soft_min_vel =
        saturate(-limits.k_position * (pos - limits.soft_min_position), -limits.max_velocity, limits.max_velocity);
    const double soft_max_vel =
        saturate(-limits.k_position * (pos - limits.soft_max_position), -limits.max_velocity, limits.max_velocity);
    double pos_low = pos + soft_min_vel * dt;
    double pos_high = pos + soft_max_vel * dt;
    pos_low = std::max(pos_low, limits.min_position);
    pos_high = std::min(pos_high, limits.max_position);
    const double pos_cmd = saturate(command(i), pos_low, pos_high);
    out(i) = pos_cmd;
    prev_cmd_[i] = pos_cmd;
  }
  return out;
}

}  // namespace serl_controller
