// SPDX-License-Identifier: MIT
// Minimal ROS stand-in for compiling the vendored SERL controller (see README.md); no control math.
#pragma once
#include <array>
namespace serl_franka_controllers {
struct ZeroJacobian {
  std::array<double, 42> zero_jacobian{};
};
}  // namespace serl_franka_controllers
