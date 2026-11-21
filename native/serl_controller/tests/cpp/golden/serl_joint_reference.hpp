// SPDX-License-Identifier: MIT
// Driver around the ORIGINAL, unmodified SERL JointPositionController (vendored in
// serl_original/, compiled against ros_stubs/), the way franka_control runs it:
// controller_manager calls starting() and update(period = 0) on the first
// libfranka tick (reset), then update(period) with period =
// ros::Duration(franka::Duration(time_step).toSec()).
#pragma once

#include <Eigen/Dense>
#include <cstdint>
#include <memory>

#include "serl_controller/joint_reset.hpp"

namespace serl_controller::golden {

class SerlJointReference {
 public:
  using Vector7d = Eigen::Matrix<double, 7, 1>;
  /// init() with /target_joint_positions = target.
  explicit SerlJointReference(const Vector7d &target);
  ~SerlJointReference();

  /// starting() with the measured joint positions q.
  void starting(const Vector7d &q);
  /// update() with period = ros::Duration(time_step_ms / 1000.0) (franka::Duration::toSec());
  /// returns the commanded joint positions (what setCommand() received).
  Vector7d update(uint64_t time_step_ms);
  /// elapsed_time_.toSec() of the original.
  [[nodiscard]] double elapsedSec() const;
  /// elapsed_time_.toNSec() of the original.
  [[nodiscard]] int64_t elapsedNs() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

/// The original controller as franka_control runs it in position mode: per libfranka tick,
/// read() (measured q), controller_manager update, then on the period-0 tick
/// position_joint_limit_interface_.reset(), otherwise enforceLimits(period) with the ORIGINAL
/// joint_limits_interface::PositionJointSoftLimitsHandle (vendored verbatim from ros_control),
/// built from the URDF joint data the way franka_hw's setupLimitInterface does.
class SerlJointHwReference {
 public:
  using Vector7d = Eigen::Matrix<double, 7, 1>;
  SerlJointHwReference(const Vector7d &target, const serl_controller::JointLimits7 &urdf_limits);
  ~SerlJointHwReference();
  void starting(const Vector7d &q);
  /// One libfranka tick: returns the command franka_hw writes to libfranka.
  Vector7d tick(const Vector7d &q_measured, uint64_t time_step_ms);
  /// The controller's own setCommand values of the last tick (before the limits).
  [[nodiscard]] Vector7d controllerCommand() const { return controller_command_; }

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
  Vector7d controller_command_{Vector7d::Zero()};
};

}  // namespace serl_controller::golden
