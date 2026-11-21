// SPDX-License-Identifier: MIT
#include "serl_reference.hpp"

// Standard and Eigen headers first, so that the access-specifier override below
// only affects the vendored SERL class (needed to read its private state).
#include <Eigen/Dense>
#include <array>
#include <cstring>
#include <fstream>
#include <functional>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PoseStamped.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <realtime_tools/realtime_publisher.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <serl_franka_controllers/ZeroJacobian.h>
#include <serl_franka_controllers/compliance_paramConfig.h>

#define private public  // NOLINT -- test-only access to the original's private members
#include <serl_franka_controllers/cartesian_impedance_controller.h>
#undef private

namespace serl_controller::golden {

namespace {

void writeState(const SerlCartesianImpedanceLaw::Input &in) {
  auto &hw = franka_hw::FakeHardware::instance();
  Eigen::Map<Eigen::Matrix4d>(hw.state.O_T_EE.data()) = in.O_T_EE;
  Eigen::Map<Eigen::Matrix<double, 7, 1>>(hw.state.q.data()) = in.q;
  Eigen::Map<Eigen::Matrix<double, 7, 1>>(hw.state.dq.data()) = in.dq;
  Eigen::Map<Eigen::Matrix<double, 7, 1>>(hw.state.tau_J_d.data()) = in.tau_J_d;
  Eigen::Map<Eigen::Matrix<double, 7, 1>>(hw.coriolis.data()) = in.coriolis;
  Eigen::Map<Eigen::Matrix<double, 6, 7>>(hw.zero_jacobian_ee.data()) = in.zero_jacobian;
}

serl_franka_controllers::compliance_paramConfig toConfig(const SerlParams &p) {
  serl_franka_controllers::compliance_paramConfig c;
  c.translational_stiffness = p.translational_stiffness;
  c.translational_damping = p.translational_damping;
  c.rotational_stiffness = p.rotational_stiffness;
  c.rotational_damping = p.rotational_damping;
  c.nullspace_stiffness = p.nullspace_stiffness;
  c.joint1_nullspace_stiffness = p.joint1_nullspace_stiffness;
  c.translational_clip_neg_x = p.translational_clip_neg_x;
  c.translational_clip_neg_y = p.translational_clip_neg_y;
  c.translational_clip_neg_z = p.translational_clip_neg_z;
  c.translational_clip_x = p.translational_clip_x;
  c.translational_clip_y = p.translational_clip_y;
  c.translational_clip_z = p.translational_clip_z;
  c.rotational_clip_neg_x = p.rotational_clip_neg_x;
  c.rotational_clip_neg_y = p.rotational_clip_neg_y;
  c.rotational_clip_neg_z = p.rotational_clip_neg_z;
  c.rotational_clip_x = p.rotational_clip_x;
  c.rotational_clip_y = p.rotational_clip_y;
  c.rotational_clip_z = p.rotational_clip_z;
  c.translational_Ki = p.translational_Ki;
  c.rotational_Ki = p.rotational_Ki;
  return c;
}

}  // namespace

struct SerlReference::Impl {
  hardware_interface::RobotHW robot_hw;
  ros::NodeHandle node_handle{"cartesian_impedance_controller"};
  serl_franka_controllers::CartesianImpedanceController controller;
};

SerlReference::SerlReference(const SerlParams &params) : impl_(std::make_unique<Impl>()) {
  // error_i and Ki_ have no initialiser in the original. A freshly started
  // controller lives in fresh (zeroed) heap memory, so zero is what runs on the
  // robot; set it explicitly to keep the reference deterministic.
  impl_->controller.error_i.setZero();
  impl_->controller.Ki_.setZero();
  impl_->controller.error_.setZero();
  if (!impl_->controller.init(&impl_->robot_hw, impl_->node_handle)) throw std::runtime_error("SERL init() failed");
  if (params != SerlParams{}) reconfigure(params);
}

SerlReference::~SerlReference() = default;

void SerlReference::starting(const SerlCartesianImpedanceLaw::Input &state) {
  writeState(state);
  impl_->controller.starting(ros::Time{});
}

Eigen::Matrix<double, 7, 1> SerlReference::update(const SerlCartesianImpedanceLaw::Input &state) {
  writeState(state);
  impl_->controller.update(ros::Time{}, ros::Duration{});
  return Eigen::Map<const Eigen::Matrix<double, 7, 1>>(franka_hw::FakeHardware::instance().command.data());
}

void SerlReference::reconfigure(const SerlParams &params) {
  impl_->controller.dynamic_server_compliance_param_->updateConfig(toConfig(params));
}

void SerlReference::setTarget(const Eigen::Vector3d &position, const Eigen::Vector4d &quat_xyzw) {
  auto msg = std::make_shared<geometry_msgs::PoseStamped>();
  msg->pose.position.x = position.x();
  msg->pose.position.y = position.y();
  msg->pose.position.z = position.z();
  msg->pose.orientation.x = quat_xyzw(0);
  msg->pose.orientation.y = quat_xyzw(1);
  msg->pose.orientation.z = quat_xyzw(2);
  msg->pose.orientation.w = quat_xyzw(3);
  const geometry_msgs::PoseStampedConstPtr const_msg = msg;
  ros::stub::Registry::instance().equilibrium_pose(&const_msg);
}

ReferenceState SerlReference::state() const {
  const auto &c = impl_->controller;
  ReferenceState s;
  s.K = c.cartesian_stiffness_;
  s.K_target = c.cartesian_stiffness_target_;
  s.D = c.cartesian_damping_;
  s.D_target = c.cartesian_damping_target_;
  s.Ki = c.Ki_;
  s.Ki_target = c.Ki_target_;
  s.nullspace_stiffness = c.nullspace_stiffness_;
  s.nullspace_stiffness_target = c.nullspace_stiffness_target_;
  s.joint1_nullspace_stiffness = c.joint1_nullspace_stiffness_;
  s.joint1_nullspace_stiffness_target = c.joint1_nullspace_stiffness_target_;
  s.position_d = c.position_d_;
  s.position_d_target = c.position_d_target_;
  s.orientation_d_xyzw = c.orientation_d_.coeffs();
  s.orientation_d_target_xyzw = c.orientation_d_target_.coeffs();
  s.error = c.error_;
  s.error_i = c.error_i;
  s.q_d_nullspace = c.q_d_nullspace_;
  s.translational_clip_min = c.translational_clip_min_;
  s.translational_clip_max = c.translational_clip_max_;
  s.rotational_clip_min = c.rotational_clip_min_;
  s.rotational_clip_max = c.rotational_clip_max_;
  s.published_jacobian = c.publisher_franka_jacobian_.last_published_.zero_jacobian;
  return s;
}

}  // namespace serl_controller::golden
