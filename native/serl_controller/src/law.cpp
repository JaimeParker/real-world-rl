// SPDX-License-Identifier: MIT
// Copyright (c) 2024 RAIL Berkeley (serl_franka_controllers); port: real-world-rl contributors.
//
// Ported from https://github.com/rail-berkeley/serl_franka_controllers @ 1f140ef,
// src/cartesian_impedance_controller.cpp (init / starting / update / saturateTorqueRate /
// complianceParamCallback / equilibriumPoseCallback), with the ROS plumbing removed.
// MIT License (see LICENSE in this package).
//   Reference: franka_ros franka_example_controllers/src/cartesian_impedance_example_controller.cpp
//
// Keep the arithmetic in the same order and with the same Eigen types as the
// original: tests/cpp/test_golden_equivalence.cpp compares this file against the
// vendored original tick by tick.
#include "serl_controller/law.hpp"

#include <algorithm>
#include <cmath>

#include "serl_controller/params.hpp"

#include "serl_controller/pseudo_inversion.hpp"

namespace serl_controller {

SerlCartesianImpedanceLaw::SerlCartesianImpedanceLaw(const SerlParams &params) {
  // init(): position_d_ = 0, orientation_d_ = identity, K = D = 0 (member initialisers),
  // then dynamic_reconfigure's Server::setCallback() invokes the callback once.
  applyParams(params);
}

void SerlCartesianImpedanceLaw::applyParams(const SerlParams &params) {
  SerlParams config = params;
  config.validateFinite();
  config.clampToRanges();  // dynamic_reconfigure clamps before the callback runs
  applyClampedParams(config);
}

void SerlCartesianImpedanceLaw::applyClampedParams(const SerlParams &config) noexcept {
  params_ = config;

  cartesian_stiffness_target_.setIdentity();
  cartesian_stiffness_target_.topLeftCorner(3, 3) << config.translational_stiffness * Eigen::Matrix3d::Identity();
  cartesian_stiffness_target_.bottomRightCorner(3, 3) << config.rotational_stiffness * Eigen::Matrix3d::Identity();
  cartesian_damping_target_.setIdentity();
  // Damping ratio = 1
  cartesian_damping_target_.topLeftCorner(3, 3) << config.translational_damping * Eigen::Matrix3d::Identity();
  cartesian_damping_target_.bottomRightCorner(3, 3) << config.rotational_damping * Eigen::Matrix3d::Identity();

  nullspace_stiffness_target_ = config.nullspace_stiffness;
  joint1_nullspace_stiffness_target_ = config.joint1_nullspace_stiffness;

  translational_clip_min_ << -config.translational_clip_neg_x, -config.translational_clip_neg_y,
      -config.translational_clip_neg_z;
  translational_clip_max_ << config.translational_clip_x, config.translational_clip_y, config.translational_clip_z;
  rotational_clip_min_ << -config.rotational_clip_neg_x, -config.rotational_clip_neg_y, -config.rotational_clip_neg_z;
  rotational_clip_max_ << config.rotational_clip_x, config.rotational_clip_y, config.rotational_clip_z;

  Ki_target_.setIdentity();
  Ki_target_.topLeftCorner(3, 3) << config.translational_Ki * Eigen::Matrix3d::Identity();
  Ki_target_.bottomRightCorner(3, 3) << config.rotational_Ki * Eigen::Matrix3d::Identity();
}

void SerlCartesianImpedanceLaw::setTarget(const Eigen::Vector3d &position, const Eigen::Vector4d &quat_xyzw) noexcept {
  position_d_target_ << position.x(), position.y(), position.z();
  error_i_.setZero();
  const Eigen::Quaterniond last_orientation_d_target(orientation_d_target_);
  orientation_d_target_.coeffs() << quat_xyzw(0), quat_xyzw(1), quat_xyzw(2), quat_xyzw(3);
  if (last_orientation_d_target.coeffs().dot(orientation_d_target_.coeffs()) < 0.0) {
    orientation_d_target_.coeffs() << -orientation_d_target_.coeffs();
  }
}
void SerlCartesianImpedanceLaw::starting(const Eigen::Matrix4d &O_T_EE, const Vector7d &q) noexcept {
  const Eigen::Affine3d initial_transform(O_T_EE);

  // set equilibrium point to current state
  position_d_ = initial_transform.translation();
  orientation_d_ = Eigen::Quaterniond(initial_transform.linear());
  position_d_target_ = initial_transform.translation();
  orientation_d_target_ = Eigen::Quaterniond(initial_transform.linear());

  // set nullspace equilibrium configuration to initial q
  q_d_nullspace_ = q;
}

SerlCartesianImpedanceLaw::Vector7d SerlCartesianImpedanceLaw::update(const Input &in) {
  const Vector7d &coriolis = in.coriolis;
  const Jacobian &jacobian = in.zero_jacobian;
  const Vector7d &q = in.q;
  const Vector7d &dq = in.dq;
  const Vector7d &tau_J_d = in.tau_J_d;
  const Eigen::Affine3d transform(in.O_T_EE);
  const Eigen::Vector3d position(transform.translation());
  Eigen::Quaterniond orientation(transform.linear());

  // compute error to desired pose
  // Clip translational error
  error_.head(3) << position - position_d_;
  for (int i = 0; i < 3; i++) {
    error_(i) = std::min(std::max(error_(i), translational_clip_min_(i)), translational_clip_max_(i));
  }

  // orientation error
  if (orientation_d_.coeffs().dot(orientation.coeffs()) < 0.0) {
    orientation.coeffs() << -orientation.coeffs();
  }
  // "difference" quaternion
  const Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d_);
  error_.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
  // Transform to base frame
  // Clip rotation error
  error_.tail(3) << -transform.linear() * error_.tail(3);
  for (int i = 0; i < 3; i++) {
    error_(i + 3) = std::min(std::max(error_(i + 3), rotational_clip_min_(i)), rotational_clip_max_(i));
  }

  error_i_.head(3) << (error_i_.head(3) + error_.head(3)).cwiseMax(-0.1).cwiseMin(0.1);
  error_i_.tail(3) << (error_i_.tail(3) + error_.tail(3)).cwiseMax(-0.3).cwiseMin(0.3);

  // compute control
  // allocate variables (dynamic-size on purpose, as in SERL: see law.hpp, "Allocation")
  Eigen::VectorXd tau_task(7);
  Eigen::VectorXd tau_nullspace(7);
  Eigen::VectorXd tau_d(7);

  // pseudoinverse for nullspace handling
  // kinematic pseuoinverse
  Eigen::MatrixXd jacobian_transpose_pinv;
  pseudoInverse(jacobian.transpose(), jacobian_transpose_pinv);

  tau_task << jacobian.transpose() *
                  (-cartesian_stiffness_ * error_ - cartesian_damping_ * (jacobian * dq) - Ki_ * error_i_);

  Eigen::Matrix<double, 7, 1> dqe;
  Eigen::Matrix<double, 7, 1> qe;

  qe << q_d_nullspace_ - q;
  qe.head(1) << qe.head(1) * joint1_nullspace_stiffness_;
  dqe << dq;
  dqe.head(1) << dqe.head(1) * 2.0 * std::sqrt(joint1_nullspace_stiffness_);
  tau_nullspace << (Eigen::MatrixXd::Identity(7, 7) - jacobian.transpose() * jacobian_transpose_pinv) *
                       (nullspace_stiffness_ * qe - (2.0 * std::sqrt(nullspace_stiffness_)) * dqe);
  // Desired torque
  tau_d << tau_task + tau_nullspace + coriolis;
  // Saturate torque rate to avoid discontinuities
  tau_d << saturateTorqueRate(tau_d, tau_J_d);

  const Vector7d command = tau_d;

  // update parameters changed online either through dynamic reconfigure or through the interactive
  // target by filtering
  cartesian_stiffness_ = filter_params_ * cartesian_stiffness_target_ + (1.0 - filter_params_) * cartesian_stiffness_;
  cartesian_damping_ = filter_params_ * cartesian_damping_target_ + (1.0 - filter_params_) * cartesian_damping_;
  nullspace_stiffness_ = filter_params_ * nullspace_stiffness_target_ + (1.0 - filter_params_) * nullspace_stiffness_;
  joint1_nullspace_stiffness_ =
      filter_params_ * joint1_nullspace_stiffness_target_ + (1.0 - filter_params_) * joint1_nullspace_stiffness_;
  position_d_ = filter_params_ * position_d_target_ + (1.0 - filter_params_) * position_d_;
  orientation_d_ = orientation_d_.slerp(filter_params_, orientation_d_target_);
  Ki_ = filter_params_ * Ki_target_ + (1.0 - filter_params_) * Ki_;

  return command;
}
SerlCartesianImpedanceLaw::Vector7d SerlCartesianImpedanceLaw::saturateTorqueRate(const Vector7d &tau_d_calculated,
                                                                                  const Vector7d &tau_J_d) noexcept {
  Vector7d tau_d_saturated{};
  for (int i = 0; i < 7; i++) {
    const double difference = tau_d_calculated[i] - tau_J_d[i];
    tau_d_saturated[i] = tau_J_d[i] + std::max(std::min(difference, kDeltaTauMax), -kDeltaTauMax);
  }
  return tau_d_saturated;
}

}  // namespace serl_controller
