// SPDX-License-Identifier: MIT
// Driver around the ORIGINAL, unmodified SERL CartesianImpedanceController
// (vendored in serl_original/, compiled against ros_stubs/). Exposes a plain
// interface so the golden test never sees ROS types.
#pragma once

#include <Eigen/Dense>
#include <array>
#include <memory>

#include "serl_controller/law.hpp"
#include "serl_controller/params.hpp"

namespace serl_controller::golden {

struct ReferenceState {
  Eigen::Matrix<double, 6, 6> K, K_target, D, D_target, Ki, Ki_target;  // NOLINT
  double nullspace_stiffness{}, nullspace_stiffness_target{};
  double joint1_nullspace_stiffness{}, joint1_nullspace_stiffness_target{};
  Eigen::Vector3d position_d, position_d_target;
  Eigen::Vector4d orientation_d_xyzw, orientation_d_target_xyzw;
  Eigen::Matrix<double, 6, 1> error, error_i;
  Eigen::Matrix<double, 7, 1> q_d_nullspace;
  Eigen::Vector3d translational_clip_min, translational_clip_max, rotational_clip_min, rotational_clip_max;
  std::array<double, 42> published_jacobian{};
};

class SerlReference {
 public:
  /// Constructs the controller and runs its init() (which triggers the first
  /// dynamic_reconfigure callback with the .cfg defaults). `params` is then
  /// applied through the dynamic_reconfigure callback if it differs from defaults.
  explicit SerlReference(const SerlParams &params = SerlParams{});
  ~SerlReference();

  void starting(const SerlCartesianImpedanceLaw::Input &state);
  Eigen::Matrix<double, 7, 1> update(const SerlCartesianImpedanceLaw::Input &state);
  /// dynamic_reconfigure update; `params` must already be clamped into range.
  void reconfigure(const SerlParams &params);
  /// Publishes a geometry_msgs/PoseStamped on equilibrium_pose.
  void setTarget(const Eigen::Vector3d &position, const Eigen::Vector4d &quat_xyzw);
  [[nodiscard]] ReferenceState state() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace serl_controller::golden
