// SPDX-License-Identifier: MIT
// Minimal ROS stand-in for compiling the vendored SERL controller (see README.md); no control math.
#pragma once
#include <string>
#include <franka_hw/fake_hardware.h>
namespace hardware_interface {
struct HardwareInterfaceException : std::exception {
  const char *what() const noexcept override { return "stub"; }
};
class JointHandle {
 public:
  explicit JointHandle(int index = 0) : index_(index) {}
  void setCommand(double command) { franka_hw::FakeHardware::instance().command[index_] = command; }
  double getCommand() const { return franka_hw::FakeHardware::instance().command[index_]; }
  std::string getName() const { return "panda_joint" + std::to_string(index_ + 1); }
  // franka_hw's joint state handle: the measured position, franka::RobotState::q.
  double getPosition() const { return franka_hw::FakeHardware::instance().state.q[index_]; }
  double getVelocity() const { return franka_hw::FakeHardware::instance().state.dq[index_]; }
  double getEffort() const { return franka_hw::FakeHardware::instance().state.tau_J_d[index_]  /* unused by the golden tests */; }

 private:
  int index_;
};
class EffortJointInterface {
 public:
  JointHandle getHandle(const std::string &name) { return JointHandle(name.back() - '1'); }
};
class PositionJointInterface {
 public:
  JointHandle getHandle(const std::string &name) { return JointHandle(name.back() - '1'); }
};
}  // namespace hardware_interface
