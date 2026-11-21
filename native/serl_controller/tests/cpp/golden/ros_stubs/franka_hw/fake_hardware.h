// SPDX-License-Identifier: MIT
// Minimal ROS stand-in for compiling the vendored SERL controller (see README.md); no control math.
#pragma once
// The test writes the "measured" quantities of the current tick here.
#include <array>
#include <vector>
#include <franka/robot_state.h>
namespace franka_hw {
struct FakeHardware {
  franka::RobotState state{};
  std::array<double, 7> coriolis{};
  std::array<double, 42> zero_jacobian_ee{};
  std::array<double, 7> command{};
  // the rosparam /target_joint_positions read by JointPositionController::init()
  std::vector<double> target_joint_positions{};
  static FakeHardware &instance() {
    static FakeHardware hw;
    return hw;
  }
};
}  // namespace franka_hw
