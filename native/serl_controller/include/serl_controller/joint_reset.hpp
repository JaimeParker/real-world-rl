// SPDX-License-Identifier: MIT AND BSD-3-Clause AND Apache-2.0
// Copyright (c) 2024 RAIL Berkeley (serl_franka_controllers, MIT);
// Copyright (C) 2013, PAL Robotics S.L. and Copyright (c) 2008, Willow Garage, Inc.
//   (ros_control joint_limits_interface, BSD-3-Clause; full notice in src/joint_reset.cpp);
// joint limit values from franka_ros (Copyright (c) 2023 Franka Robotics GmbH, Apache-2.0);
// port: real-world-rl contributors. See LICENSE in this package.
//
// SerlJointPositionLaw: the control law of serl_franka_controllers'
// JointPositionController (SERL's joint reset), without ROS and without franky.
//
// Ported from https://github.com/rail-berkeley/serl_franka_controllers @ 1f140ef
// (src/joint_position_controller.cpp,
//  include/serl_franka_controllers/joint_position_controller.h), MIT License,
// which itself refers to franka_ros' joint_position_example_controller.
// PositionJointSoftLimits is ported from https://github.com/ros-controls/ros_control
// noetic-devel @ 633c374 (joint_limits_interface/include/joint_limits_interface/
// joint_limits_interface.h, PositionJointSoftLimitsHandle), BSD-3-Clause. The limit
// tables are the values of https://github.com/frankaemika/franka_ros @ ddd2fff,
// franka_description/robots/{panda,fr3}/joint_limits.yaml, Apache-2.0.
//
// The original, every control tick (franka_hw position interface, 1 kHz):
//
//   starting():  initial_pose_ = measured q;  elapsed_time_ = 0
//   update(period):
//     elapsed_time_ += period
//     t = elapsed_time_.toSec()
//     command_i = t > 10 ? reset_pose_i
//                        : (t / 10.0) * reset_pose_i + ((10 - t) / 10.0) * initial_pose_i
//
// i.e. a 10 s linear blend from the start posture to the target, then hold.
// elapsed_time_ is a ros::Duration (int32 sec + int32 nsec); franka_hw passes
// period = ros::Duration(franka::Duration(time_step).toSec()), so every period is
// a whole number of milliseconds and elapsed_time_ stays an exact nanosecond
// count. This port keeps that representation (an exact nanosecond count, read
// back with ros::Duration::toSec()'s formula sec + 1e-9 * nsec) and the same
// expression order, so tests/cpp/test_joint_reset_golden.cpp can require
// per-tick agreement with the original compiled from the vendored source.
//
// Lifetime: one law object == one roslaunch of SERL's joint.launch. The
// constructor is init() (reads /target_joint_positions), starting() is the
// controller's starting() (called by controller_manager on the first libfranka
// tick, whose period is 0).
#pragma once

#include <Eigen/Dense>
#include <array>
#include <cstdint>
#include <string>

namespace serl_controller {

/// One joint's limits as franka_hw builds them for its PositionJointSoftLimitsInterface
/// (joint_limits_interface::getJointLimits / getSoftJointLimits from the URDF, which
/// franka_description's `franka-limits` macro writes from <robot>/joint_limits.yaml:
/// <limit lower upper velocity> and <safety_controller k_position="100" soft_lower_limit=lower
/// soft_upper_limit=upper>). Revolute joints, so has_position_limits is true.
struct JointLimit {
  double min_position;       // limit lower
  double max_position;       // limit upper
  double max_velocity;       // limit velocity
  double soft_min_position;  // safety_controller soft_lower_limit
  double soft_max_position;  // safety_controller soft_upper_limit
  double k_position;         // safety_controller k_position
};
using JointLimits7 = std::array<JointLimit, 7>;

/// franka_description (franka_ros @ ddd2fff) robots/panda/joint_limits.yaml, k_position 100.
[[nodiscard]] JointLimits7 frankaPandaJointLimits() noexcept;
/// franka_description (franka_ros @ ddd2fff) robots/fr3/joint_limits.yaml, k_position 100.
[[nodiscard]] JointLimits7 frankaFr3JointLimits() noexcept;
/// "panda" or "fr3"; throws std::invalid_argument otherwise.
[[nodiscard]] JointLimits7 frankaJointLimits(const std::string &robot_model);

/// joint_limits_interface::PositionJointSoftLimitsHandle (ros_control noetic-devel @ 633c374)
/// for the 7 joints, i.e. franka_hw's position_joint_limit_interface_. franka_control_node
/// calls reset() on the period-0 tick and enforceLimits(period) after every other controller
/// update; the first enforce starts from the measured position, later ones from the previous
/// (limited) command.
class PositionJointSoftLimits {
 public:
  using Vector7d = Eigen::Matrix<double, 7, 1>;
  explicit PositionJointSoftLimits(const JointLimits7 &limits) noexcept;
  void reset() noexcept;
  /// Saturate `command` (the controller's setCommand values) given the measured positions
  /// (JointHandle::getPosition) and the period; returns the command franka_hw sends.
  /// Throws std::invalid_argument for period_ns <= 0 (franka_control_node never enforces
  /// on the period-0 tick; SerlJointPositionMotion calls reset() there instead).
  [[nodiscard]] Vector7d enforce(const Vector7d &command, const Vector7d &q_measured, int64_t period_ns);
  [[nodiscard]] const JointLimits7 &limits() const noexcept { return limits_; }

 private:
  JointLimits7 limits_;
  std::array<double, 7> prev_cmd_{};  // NaN after reset(): "start from the measured position"
};

class SerlJointPositionLaw {
 public:
  using Vector7d = Eigen::Matrix<double, 7, 1>;

  /// SERL's blend duration, hard-coded in joint_position_controller.cpp.
  static constexpr double kBlendDurationS = 10.0;

  /// init(): the target posture (SERL's /target_joint_positions). Throws
  /// std::invalid_argument for non-finite values.
  explicit SerlJointPositionLaw(const Vector7d &target);

  /// starting(): blend from the measured posture `q`, elapsed time 0.
  void starting(const Vector7d &q) noexcept;

  /// update(): advance the elapsed time by `period_ns` (>= 0) and return the
  /// commanded joint positions of this tick. Throws std::invalid_argument for a
  /// negative period. Never allocates.
  [[nodiscard]] Vector7d update(int64_t period_ns);

  /// franka::Duration (whole milliseconds) -> the nanoseconds ros::Duration(toSec()) holds.
  [[nodiscard]] static int64_t periodNsFromMilliseconds(uint64_t milliseconds) noexcept;

  /// elapsed_time_.toSec(): static_cast<double>(sec) + 1e-9 * static_cast<double>(nsec).
  [[nodiscard]] double elapsedSec() const noexcept;
  [[nodiscard]] int64_t elapsedNs() const noexcept { return elapsed_ns_; }
  [[nodiscard]] const Vector7d &target() const noexcept { return reset_pose_; }
  [[nodiscard]] const Vector7d &initial() const noexcept { return initial_pose_; }
  [[nodiscard]] const Vector7d &command() const noexcept { return command_; }

 private:
  Vector7d reset_pose_;
  Vector7d initial_pose_{Vector7d::Zero()};
  Vector7d command_{Vector7d::Zero()};
  int64_t elapsed_ns_{0};
};

}  // namespace serl_controller
