// SPDX-License-Identifier: MIT
// Copyright (c) 2024 RAIL Berkeley (serl_franka_controllers); port: real-world-rl contributors.
// The franka_hw torque-interface tick around the port of serl_franka_controllers @ 1f140ef
// src/cartesian_impedance_controller.cpp (see law.hpp). MIT License (see LICENSE).
#include "serl_controller/motion.hpp"

#include <array>
#include <cmath>
#include <cstdint>
#include <map>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>

#include "franka/control_types.h"
#include "franka/duration.h"
#include "franka/model.h"
#include "franky/robot.hpp"
#include "franky/robot_state.hpp"
#include "serl_controller/params.hpp"

namespace serl_controller {

SerlCartesianImpedanceMotion::SerlCartesianImpedanceMotion(const SerlParams &params)
    : requested_params_([&params] {
        SerlParams p = params;
        p.validateFinite();
        p.clampToRanges();
        return p;
      }()),
      law_(requested_params_) {
  params_buffer_.setUnsafe(ParamsMsg{0, requested_params_});  // constructor: no concurrency yet
  target_buffer_.setUnsafe(TargetMsg{});
}

uint64_t SerlCartesianImpedanceMotion::setTarget(const Eigen::Vector3d &position, const Eigen::Vector4d &quat_xyzw) {
  if (!position.allFinite() || !quat_xyzw.allFinite()) {
    throw std::invalid_argument("SERL target must be finite");
  }
  if (std::abs(quat_xyzw.norm() - 1.0) > 1e-3) {
    throw std::invalid_argument("SERL target quaternion (x, y, z, w) must be unit norm");
  }
  const std::lock_guard lock(user_mutex_);
  TargetMsg msg;
  msg.seq = ++target_seq_;
  msg.position = position;
  msg.quat_xyzw = quat_xyzw;
  target_buffer_.set(msg);
  return msg.seq;
}

SerlParams SerlCartesianImpedanceMotion::updateParams(const std::map<std::string, double> &updates) {
  const std::lock_guard lock(user_mutex_);
  SerlParams next = requested_params_;
  next.merge(updates);  // throws without side effects
  requested_params_ = next;
  params_buffer_.set(ParamsMsg{++params_seq_, next});
  return next;
}

SerlParams SerlCartesianImpedanceMotion::params() const {
  const std::lock_guard lock(user_mutex_);
  return requested_params_;
}

SerlSnapshot SerlCartesianImpedanceMotion::snapshot() { return snapshot_buffer_.get(); }

void SerlCartesianImpedanceMotion::applyPendingParams() noexcept {
  if (!params_buffer_.hasNewData()) return;
  const ParamsMsg msg = params_buffer_.getUnsafe();
  if (msg.seq == applied_params_seq_) return;
  law_.applyClampedParams(msg.params);  // validated + clamped on the user thread (updateParams / constructor)
  applied_params_seq_ = msg.seq;
}

void SerlCartesianImpedanceMotion::initImpl(const franky::RobotState &robot_state,
                                            const std::optional<franka::Torques> & /*previous_command*/) {
  // dynamic_reconfigure may have changed parameters before the controller started
  applyPendingParams();
  // A target that arrived before starting() is overwritten by starting(), as in SERL.
  applied_target_seq_ = target_buffer_.getUnsafe().seq;
  law_.starting(robot_state.O_T_EE.matrix(), robot_state.q);
  tick_ = 0;
}

Eigen::Matrix<double, 7, 1> SerlCartesianImpedanceMotion::computeTorques(
    const franky::RobotState &robot_state, const Eigen::Matrix<double, 6, 7> &zero_jacobian,
    const Eigen::Matrix<double, 7, 1> &coriolis, double rel_time) {
  // ROS callbacks run between two update() calls: apply user updates first.
  applyPendingParams();
  if (target_buffer_.hasNewData()) {
    const TargetMsg msg = target_buffer_.getUnsafe();
    if (msg.seq != applied_target_seq_) {
      law_.setTarget(msg.position, msg.quat_xyzw);
      applied_target_seq_ = msg.seq;
    }
  }

  SerlCartesianImpedanceLaw::Input in;
  in.O_T_EE = robot_state.O_T_EE.matrix();
  in.q = robot_state.q;
  in.dq = robot_state.dq;
  in.tau_J_d = robot_state.tau_J_d;
  in.zero_jacobian = zero_jacobian;
  in.coriolis = coriolis;
  const Eigen::Matrix<double, 7, 1> tau = law_.update(in);
  ++tick_;

  SerlSnapshot snap;
  snap.tick = tick_;
  snap.rel_time = rel_time;
  snap.zero_jacobian = zero_jacobian;
  snap.O_T_EE = in.O_T_EE;
  snap.q = in.q;
  snap.dq = in.dq;
  snap.tau_J_d = in.tau_J_d;
  snap.coriolis = coriolis;
  snap.tau_command = tau;
  snap.error = law_.error();
  snap.error_i = law_.errorI();
  snap.position_d = law_.positionD();
  snap.orientation_d_xyzw = law_.orientationD().coeffs();
  snap.position_d_target = law_.positionDTarget();
  snap.orientation_d_target_xyzw = law_.orientationDTarget().coeffs();
  snap.stiffness_diag = law_.cartesianStiffness().diagonal();
  snap.damping_diag = law_.cartesianDamping().diagonal();
  snap.ki_diag = law_.ki().diagonal();
  snap.nullspace_stiffness = law_.nullspaceStiffness();
  snap.joint1_nullspace_stiffness = law_.joint1NullspaceStiffness();
  snap.applied_target_seq = applied_target_seq_;
  snap.applied_params_seq = applied_params_seq_;
  snapshot_buffer_.setUnsafe(snap);  // single RT writer, wait-free
  return tau;
}

franka::Torques SerlCartesianImpedanceMotion::nextCommandImpl(
    const franky::RobotState &robot_state, franka::Duration /*time_step*/, franka::Duration rel_time,
    franka::Duration /*abs_time*/, const std::optional<franka::Torques> & /*previous_command*/) {
  // Same model quantities franka_hw's FrankaModelHandle hands to the SERL controller:
  // getCoriolis() and getZeroJacobian(kEndEffector) evaluated at the current state.
  const auto model = robot()->model();  // robot(): non-owning, set by franky for the motion's run
  const Eigen::Matrix<double, 7, 1> coriolis = model->coriolis(robot_state);
  const Eigen::Matrix<double, 6, 7> jacobian = model->zeroJacobian(franka::Frame::kEndEffector, robot_state);
  const Eigen::Matrix<double, 7, 1> tau = computeTorques(robot_state, jacobian, coriolis, rel_time.toSec());
  std::array<double, 7> tau_array{};
  Eigen::Map<Eigen::Matrix<double, 7, 1>>(tau_array.data()) = tau;
  return {tau_array};
}

}  // namespace serl_controller
