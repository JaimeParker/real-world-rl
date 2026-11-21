// SPDX-License-Identifier: MIT
// Minimal ROS stand-in for compiling the vendored SERL controller (see README.md); no control math.
#pragma once
#include <array>
#include <string>
#include <franka/robot_state.h>
#include <franka_hw/fake_hardware.h>
namespace franka_hw {
class FrankaModelHandle {
 public:
  std::array<double, 7> getCoriolis() const { return FakeHardware::instance().coriolis; }
  std::array<double, 42> getZeroJacobian(const franka::Frame frame) const {
    (void)frame;  // the controller only ever asks for kEndEffector
    return FakeHardware::instance().zero_jacobian_ee;
  }
};
class FrankaModelInterface {
 public:
  FrankaModelHandle getHandle(const std::string &) { return {}; }
};
}  // namespace franka_hw
