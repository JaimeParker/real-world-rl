// SPDX-License-Identifier: MIT
// Minimal ROS stand-in for compiling the vendored SERL controller (see README.md); no control math.
#pragma once
#include <array>
namespace franka {
// Only the fields the SERL controller reads.
struct RobotState {
  std::array<double, 16> O_T_EE{};  // NOLINT
  std::array<double, 7> q{};
  std::array<double, 7> dq{};
  std::array<double, 7> tau_J_d{};  // NOLINT
};
enum class Frame { kJoint1, kJoint2, kJoint3, kJoint4, kJoint5, kJoint6, kJoint7, kFlange, kEndEffector, kStiffness };
}  // namespace franka
