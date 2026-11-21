// SPDX-License-Identifier: MIT
// Minimal ROS stand-in for compiling the vendored SERL controller (see README.md); no control math.
#pragma once
#include <string>
#include <franka/robot_state.h>
#include <franka_hw/fake_hardware.h>
namespace franka_hw {
class FrankaStateHandle {
 public:
  const franka::RobotState &getRobotState() const { return FakeHardware::instance().state; }
};
class FrankaStateInterface {
 public:
  FrankaStateHandle getHandle(const std::string &) { return {}; }
};
}  // namespace franka_hw
