// SPDX-License-Identifier: MIT AND Apache-2.0
// Copyright (c) real-world-rl contributors.
// Default joint impedance gains (stiffness, damping, velocity_filter_alpha) and the command
// timeout are those of the public GELLO joint impedance controller,
// wuphilipp/gello_software @ 9fc8c26 ros2/src/franka_fr3_arm_controllers/config/controllers.yaml
// (k_gains, d_gains, k_alpha) and include/.../joint_impedance_controller.hpp (kJointStateTimeout),
// Copyright (c) 2025 Franka Robotics GmbH, Apache License 2.0. The Panda / FR3 joint position
// limits are franka_description's (frankaemika/franka_ros @ ddd2fff, Apache-2.0). See LICENSE.
//
// TrajectoryImpedanceParams: every parameter of TrajectoryImpedanceLaw in one struct, with a
// validated partial update (merge) in the style of serl_controller's SerlParams. Unlike SerlParams
// nothing is clamped: an invalid value is rejected with std::invalid_argument and the struct is
// left unchanged.
#pragma once

#include <Eigen/Dense>
#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <map>
#include <string>
#include <variant>

namespace joint_controller {

using Vector7d = Eigen::Matrix<double, 7, 1>;

/// The value of one parameter: a scalar, a flag or a per-joint vector.
using ParamValue = std::variant<double, bool, Vector7d>;
/// A (partial) set of parameters by name, e.g. {"stiffness": Vector7d, "max_delta_tau": 0.5}.
using ParamUpdates = std::map<std::string, ParamValue>;

/// Joint position limits of one robot model (rad).
struct JointPositionLimits {
  Vector7d lower;
  Vector7d upper;
};

/// franka_description's position limits: "panda" or "fr3". Throws std::invalid_argument otherwise.
[[nodiscard]] JointPositionLimits frankaJointPositionLimits(const std::string &robot_model);

struct TrajectoryImpedanceParams {
  // ---- joint impedance (GELLO's JointImpedanceController) ----
  Vector7d stiffness{gelloStiffness()};  // Nm/rad, K
  Vector7d damping{gelloDamping()};      // Nms/rad, D
  double velocity_filter_alpha{0.99};    // dq_f = (1 - alpha) dq_f + alpha dq, alpha in [0, 1]
  // ---- reference generator ----
  Vector7d max_velocity{defaultMaxVelocity()};         // rad/s, > 0
  Vector7d max_acceleration{Vector7d::Constant(4.0)};  // rad/s^2, > 0 (4.0: ~0.06 rad lag at a 0.5 rad/s leader)
  double min_segment_duration_s{0.1};                  // s, > 0
  Vector7d lower_limit{Vector7d::Constant(kUnset)};    // rad, required
  Vector7d upper_limit{Vector7d::Constant(kUnset)};    // rad, required
  double limit_margin{0.05};                           // rad, >= 0
  double command_timeout_s{0.5};                       // s, > 0
  // ---- torque output ----
  double max_delta_tau{1.0};       // Nm per tick, > 0
  bool compensate_coriolis{true};  // add the model's coriolis torque (never gravity)

  /// Unset joint limit (NaN): validate() rejects it, so the limits must be given.
  static constexpr double kUnset = std::numeric_limits<double>::quiet_NaN();

  [[nodiscard]] static Vector7d gelloStiffness() noexcept;
  [[nodiscard]] static Vector7d gelloDamping() noexcept;
  [[nodiscard]] static Vector7d defaultMaxVelocity() noexcept;

  enum class Kind : std::uint8_t { kScalar, kFlag, kVector7 };

  /// One row of the parameter table (an immutable row of a static table).
  struct Field {
    const char *name;  // non-owning, points at a string literal
    Kind kind;
    double TrajectoryImpedanceParams::*scalar;    // kScalar, else nullptr
    bool TrajectoryImpedanceParams::*flag;        // kFlag, else nullptr
    Vector7d TrajectoryImpedanceParams::*vector;  // kVector7, else nullptr
  };

  static constexpr std::size_t kNumFields = 12;
  /// The 12 fields in declaration order.
  [[nodiscard]] static const std::array<Field, kNumFields> &fields() noexcept;
  /// The field called `name`, or nullptr.
  [[nodiscard]] static const Field *findField(const std::string &name) noexcept;

  /// Throws std::invalid_argument naming the first violated rule:
  /// every value finite; stiffness, damping >= 0; velocity_filter_alpha in [0, 1];
  /// max_velocity, max_acceleration, min_segment_duration_s, command_timeout_s, max_delta_tau > 0;
  /// limit_margin >= 0; lower_limit, upper_limit set and
  /// lower_limit + 2 limit_margin < upper_limit per joint.
  void validate() const;

  /// Partial update: only the given keys change, then the result is validated. Throws
  /// std::invalid_argument for an unknown key, a value of the wrong kind or an invalid result;
  /// on throw *this is unchanged.
  void merge(const ParamUpdates &updates);

  [[nodiscard]] ParamUpdates toMap() const;

  /// The clipping envelope of targets: [lower_limit + limit_margin, upper_limit - limit_margin].
  [[nodiscard]] Vector7d targetLower() const { return lower_limit.array() + limit_margin; }
  [[nodiscard]] Vector7d targetUpper() const { return upper_limit.array() - limit_margin; }

  [[nodiscard]] bool operator==(const TrajectoryImpedanceParams &other) const noexcept;
  [[nodiscard]] bool operator!=(const TrajectoryImpedanceParams &other) const noexcept { return !(*this == other); }
};

/// Defaults with lower_limit / upper_limit of `robot_model` ("panda" | "fr3").
[[nodiscard]] TrajectoryImpedanceParams defaultParams(const std::string &robot_model);

}  // namespace joint_controller
