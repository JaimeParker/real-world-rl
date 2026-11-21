// SPDX-License-Identifier: MIT
// Copyright (c) 2024 RAIL Berkeley (serl_franka_controllers); port: real-world-rl contributors.
//
// SerlCartesianImpedanceLaw: the control law of serl_franka_controllers'
// CartesianImpedanceController, without ROS and without franky.
//
// Ported from https://github.com/rail-berkeley/serl_franka_controllers @ 1f140ef
// (src/cartesian_impedance_controller.cpp,
//  include/serl_franka_controllers/cartesian_impedance_controller.h).
// MIT License (see LICENSE in this package).
// which itself refers to franka_ros' cartesian_impedance_example_controller.
//
// Everything that affects numbers is kept as in the original, including the
// order of Eigen operations and the dynamic-size temporaries, so the golden test
// (tests/cpp/test_golden_equivalence.cpp) can require agreement per tick with
// the original update() compiled from the vendored SERL source:
//
//  * the translational error is clipped per axis into [-clip_neg, clip];
//  * the orientation error is the vector part of q^-1 * q_d (after flipping q
//    into q_d's hemisphere), rotated to the base frame with a sign flip, then
//    clipped per axis;
//  * error_i accumulates every tick (even with Ki = 0), is clipped to +-0.1
//    (translation) / +-0.3 (rotation) and is reset only when a new target is set;
//  * damped pseudo-inverse (lambda = 0.2) of J^T for the nullspace projector;
//  * nullspace PD towards q at starting(), joint 1 with its own stiffness;
//  * coriolis added, no gravity (libfranka compensates gravity itself);
//  * torque-rate saturation of +-1 Nm per tick against tau_J_d;
//  * AFTER the torque is computed, K, D, nullspace stiffnesses, Ki and the
//    position target move towards their targets with an exponential filter
//    (filter_params 0.005), the orientation target with slerp;
//  * a new orientation target is flipped into the previous target's hemisphere.
//
// Lifetime, mirroring SERL's effective runtime behaviour (SERL's franka_server
// re-runs `roslaunch impedance.launch` for every (re)start, so every start is a
// fresh controller object):
//  * the constructor == init() followed by dynamic_reconfigure's first callback:
//    K, D, Ki = 0 (Ki, error_i are uninitialised in the original; fresh heap
//    memory is zero, so 0 is what runs), nullspace stiffness 20 and joint-1
//    nullspace stiffness 20 (header member initialisers), and all *targets* from
//    the given parameters (cfg defaults unless overridden). Hence K and D ramp
//    from 0 to their targets with time constant ~200 ticks (0.2 s at 1 kHz), and
//    the nullspace stiffnesses ramp 20 -> 0.2 and 20 -> 100.
//  * starting() == the controller's starting(): sets the pose target, filtered
//    target and nullspace posture to the current state. It does not reset gains
//    or error_i (as in the original).
//
// Allocation: update() allocates on the heap every tick (Eigen::VectorXd tau_task /
// tau_nullspace / tau_d, the MatrixXd pseudo-inverse and its JacobiSVD), exactly like
// SERL's update(). This is deliberate: fixed-size types let Eigen pick different
// kernels / vectorisation and evaluation order, which breaks the bit-identical golden
// comparison. The blocks are small and the same every tick (glibc usually serves them
// from the thread's tcache), i.e. the same cost SERL has under ros_control; its timing
// jitter is measured on the robot (docs: serl robot validation). The only exception update()
// can throw is std::bad_alloc. Everything else here (applyClampedParams, setTarget,
// starting) neither allocates nor throws.
#pragma once

#include <Eigen/Dense>

#include "serl_controller/params.hpp"

namespace serl_controller {

class SerlCartesianImpedanceLaw {
 public:
  using Vector7d = Eigen::Matrix<double, 7, 1>;
  using Matrix6d = Eigen::Matrix<double, 6, 6>;
  using Jacobian = Eigen::Matrix<double, 6, 7>;

  /// Everything update() reads from the robot in one tick.
  struct Input {
    Eigen::Matrix4d O_T_EE{Eigen::Matrix4d::Identity()};
    Vector7d q{Vector7d::Zero()};
    Vector7d dq{Vector7d::Zero()};
    Vector7d tau_J_d{Vector7d::Zero()};
    Jacobian zero_jacobian{Jacobian::Zero()};  // EE frame, base-frame zero Jacobian
    Vector7d coriolis{Vector7d::Zero()};
  };

  static constexpr double kFilterParams = 0.005;
  static constexpr double kDeltaTauMax = 1.0;
  static constexpr double kPinvLambda = 0.2;

  /// init() + first dynamic_reconfigure callback with `params` (clamped into range).
  explicit SerlCartesianImpedanceLaw(const SerlParams &params = SerlParams{});

  /// complianceParamCallback(): sets the *target* gains, the clip bounds
  /// (min = -clip_neg, max = clip) and the Ki targets. Values are clamped into
  /// the .cfg ranges first, as dynamic_reconfigure does.
  /// Throws std::invalid_argument for non-finite values (then nothing changes).
  void applyParams(const SerlParams &params);

  /// applyParams() for a parameter set that is already finite and clamped (the
  /// motion validates on the user thread): the real-time variant, no checks.
  void applyClampedParams(const SerlParams &config) noexcept;

  /// equilibriumPoseCallback(): new pose target. quat is (x, y, z, w). Resets
  /// error_i and flips the quaternion into the previous target's hemisphere.
  void setTarget(const Eigen::Vector3d &position, const Eigen::Vector4d &quat_xyzw) noexcept;

  /// starting(): pose targets and nullspace posture := current state.
  void starting(const Eigen::Matrix4d &O_T_EE, const Vector7d &q) noexcept;

  /// update(): returns the commanded joint torques of this tick and then
  /// advances the parameter / target filters. Allocates (see above).
  [[nodiscard]] Vector7d update(const Input &in);

  // --- internal state, for tests and telemetry -----------------------------
  [[nodiscard]] const SerlParams &params() const { return params_; }
  [[nodiscard]] const Matrix6d &cartesianStiffness() const { return cartesian_stiffness_; }
  [[nodiscard]] const Matrix6d &cartesianStiffnessTarget() const { return cartesian_stiffness_target_; }
  [[nodiscard]] const Matrix6d &cartesianDamping() const { return cartesian_damping_; }
  [[nodiscard]] const Matrix6d &cartesianDampingTarget() const { return cartesian_damping_target_; }
  [[nodiscard]] const Matrix6d &ki() const { return Ki_; }
  [[nodiscard]] const Matrix6d &kiTarget() const { return Ki_target_; }
  [[nodiscard]] double nullspaceStiffness() const { return nullspace_stiffness_; }
  [[nodiscard]] double nullspaceStiffnessTarget() const { return nullspace_stiffness_target_; }
  [[nodiscard]] double joint1NullspaceStiffness() const { return joint1_nullspace_stiffness_; }
  [[nodiscard]] double joint1NullspaceStiffnessTarget() const { return joint1_nullspace_stiffness_target_; }
  [[nodiscard]] const Eigen::Vector3d &positionD() const { return position_d_; }
  [[nodiscard]] const Eigen::Quaterniond &orientationD() const { return orientation_d_; }
  [[nodiscard]] const Eigen::Vector3d &positionDTarget() const { return position_d_target_; }
  [[nodiscard]] const Eigen::Quaterniond &orientationDTarget() const { return orientation_d_target_; }
  [[nodiscard]] const Vector7d &qDNullspace() const { return q_d_nullspace_; }
  [[nodiscard]] const Eigen::Matrix<double, 6, 1> &error() const { return error_; }
  [[nodiscard]] const Eigen::Matrix<double, 6, 1> &errorI() const { return error_i_; }
  [[nodiscard]] const Eigen::Matrix<double, 3, 1> &translationalClipMin() const { return translational_clip_min_; }
  [[nodiscard]] const Eigen::Matrix<double, 3, 1> &translationalClipMax() const { return translational_clip_max_; }
  [[nodiscard]] const Eigen::Matrix<double, 3, 1> &rotationalClipMin() const { return rotational_clip_min_; }
  [[nodiscard]] const Eigen::Matrix<double, 3, 1> &rotationalClipMax() const { return rotational_clip_max_; }

 private:
  [[nodiscard]] static Vector7d saturateTorqueRate(const Vector7d &tau_d_calculated, const Vector7d &tau_J_d) noexcept;

  SerlParams params_{};

  double filter_params_{kFilterParams};  // a member, as in SERL (never changed)
  double nullspace_stiffness_{20.0};
  double nullspace_stiffness_target_{20.0};
  double joint1_nullspace_stiffness_{20.0};
  double joint1_nullspace_stiffness_target_{20.0};
  Matrix6d cartesian_stiffness_{Matrix6d::Zero()};
  Matrix6d cartesian_stiffness_target_{Matrix6d::Zero()};
  Matrix6d cartesian_damping_{Matrix6d::Zero()};
  Matrix6d cartesian_damping_target_{Matrix6d::Zero()};
  Matrix6d Ki_{Matrix6d::Zero()};
  Matrix6d Ki_target_{Matrix6d::Zero()};

  Eigen::Matrix<double, 3, 1> translational_clip_min_{Eigen::Matrix<double, 3, 1>::Zero()};
  Eigen::Matrix<double, 3, 1> translational_clip_max_{Eigen::Matrix<double, 3, 1>::Zero()};
  Eigen::Matrix<double, 3, 1> rotational_clip_min_{Eigen::Matrix<double, 3, 1>::Zero()};
  Eigen::Matrix<double, 3, 1> rotational_clip_max_{Eigen::Matrix<double, 3, 1>::Zero()};
  Vector7d q_d_nullspace_{Vector7d::Zero()};
  Eigen::Vector3d position_d_{Eigen::Vector3d::Zero()};
  Eigen::Matrix<double, 6, 1> error_{Eigen::Matrix<double, 6, 1>::Zero()};
  Eigen::Matrix<double, 6, 1> error_i_{Eigen::Matrix<double, 6, 1>::Zero()};
  Eigen::Quaterniond orientation_d_{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d position_d_target_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond orientation_d_target_{Eigen::Quaterniond::Identity()};
};

}  // namespace serl_controller
