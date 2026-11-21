// SPDX-License-Identifier: MIT
#include "serl_joint_reference.hpp"

// Standard headers first, so the access-specifier override below only affects the SERL class.
#include <Eigen/Dense>
#include <array>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <franka_hw/fake_hardware.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>

#include <joint_limits_interface/joint_limits_interface.h>  // ros_control original, unmodified

#define private public  // NOLINT -- test-only access to the original's private members
#include <serl_franka_controllers/joint_position_controller.h>
#undef private

namespace serl_controller::golden {

struct SerlJointReference::Impl {
  hardware_interface::RobotHW robot_hw;
  ros::NodeHandle node_handle{"joint_position_controller"};
  serl_franka_controllers::JointPositionController controller;
};

SerlJointReference::SerlJointReference(const Vector7d &target) : impl_(std::make_unique<Impl>()) {
  auto &hw = franka_hw::FakeHardware::instance();
  hw.target_joint_positions.assign(target.data(), target.data() + 7);
  if (!impl_->controller.init(&impl_->robot_hw, impl_->node_handle)) {
    throw std::runtime_error("original JointPositionController::init() failed");
  }
}

SerlJointReference::~SerlJointReference() = default;

void SerlJointReference::starting(const Vector7d &q) {
  auto &hw = franka_hw::FakeHardware::instance();
  Eigen::Map<Vector7d>(hw.state.q.data()) = q;
  impl_->controller.starting(ros::Time());
}

SerlJointReference::Vector7d SerlJointReference::update(uint64_t time_step_ms) {
  auto &hw = franka_hw::FakeHardware::instance();
  hw.command.fill(std::nan(""));
  // franka_hw: ros::Duration(time_step.toSec()); franka::Duration::toSec() is
  // std::chrono::duration<double>(milliseconds).count() == ms / 1000.0.
  const double seconds = static_cast<double>(time_step_ms) / 1000.0;
  impl_->controller.update(ros::Time(), ros::Duration(seconds));
  return Eigen::Map<const Vector7d>(hw.command.data());
}

double SerlJointReference::elapsedSec() const { return impl_->controller.elapsed_time_.toSec(); }

int64_t SerlJointReference::elapsedNs() const { return impl_->controller.elapsed_time_.toNSec(); }

struct SerlJointHwReference::Impl {
  hardware_interface::RobotHW robot_hw;
  ros::NodeHandle node_handle{"joint_position_controller"};
  serl_franka_controllers::JointPositionController controller;
  std::vector<joint_limits_interface::PositionJointSoftLimitsHandle> handles;
};

SerlJointHwReference::SerlJointHwReference(const Vector7d &target, const serl_controller::JointLimits7 &urdf_limits)
    : impl_(std::make_unique<Impl>()) {
  auto &hw = franka_hw::FakeHardware::instance();
  hw.target_joint_positions.assign(target.data(), target.data() + 7);
  if (!impl_->controller.init(&impl_->robot_hw, impl_->node_handle)) {
    throw std::runtime_error("original JointPositionController::init() failed");
  }
  auto *position_interface = impl_->robot_hw.get<hardware_interface::PositionJointInterface>();
  for (int i = 0; i < 7; ++i) {
    const auto &u = urdf_limits[static_cast<size_t>(i)];
    // joint_limits_interface::getJointLimits / getSoftJointLimits of a revolute URDF joint,
    // then franka_hw sets the (unused here) acceleration and jerk limits.
    joint_limits_interface::JointLimits limits;
    limits.has_position_limits = true;
    limits.min_position = u.min_position;
    limits.max_position = u.max_position;
    limits.has_velocity_limits = true;
    limits.max_velocity = u.max_velocity;
    limits.has_acceleration_limits = true;
    limits.has_jerk_limits = true;
    joint_limits_interface::SoftJointLimits soft;
    soft.min_position = u.soft_min_position;
    soft.max_position = u.soft_max_position;
    soft.k_position = u.k_position;
    soft.k_velocity = 40.0;
    impl_->handles.emplace_back(position_interface->getHandle("panda_joint" + std::to_string(i + 1)), limits, soft);
  }
}

SerlJointHwReference::~SerlJointHwReference() = default;

void SerlJointHwReference::starting(const Vector7d &q) {
  auto &hw = franka_hw::FakeHardware::instance();
  Eigen::Map<Vector7d>(hw.state.q.data()) = q;
  impl_->controller.starting(ros::Time());
}

SerlJointHwReference::Vector7d SerlJointHwReference::tick(const Vector7d &q_measured, uint64_t time_step_ms) {
  auto &hw = franka_hw::FakeHardware::instance();
  Eigen::Map<Vector7d>(hw.state.q.data()) = q_measured;  // FrankaHW::read()
  hw.command.fill(std::nan(""));
  const ros::Duration period(static_cast<double>(time_step_ms) / 1000.0);
  impl_->controller.update(ros::Time(), period);
  controller_command_ = Eigen::Map<const Vector7d>(hw.command.data());
  if (period.toSec() == 0.0) {
    for (auto &handle : impl_->handles) handle.reset();  // franka_control.reset()
  } else {
    for (auto &handle : impl_->handles) handle.enforceLimits(period);  // franka_control.enforceLimits(period)
  }
  return Eigen::Map<const Vector7d>(hw.command.data());  // FrankaHW::write()
}

}  // namespace serl_controller::golden
