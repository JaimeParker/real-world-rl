// SPDX-License-Identifier: MIT
// Minimal ROS stand-in for compiling the vendored SERL controller (see README.md); no control math.
#pragma once
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
namespace hardware_interface {
class RobotHW {
 public:
  template <class T>
  T *get();
 private:
  franka_hw::FrankaModelInterface model_;
  franka_hw::FrankaStateInterface state_;
  EffortJointInterface effort_;
  PositionJointInterface position_;
};
template <>
inline franka_hw::FrankaModelInterface *RobotHW::get<franka_hw::FrankaModelInterface>() { return &model_; }
template <>
inline franka_hw::FrankaStateInterface *RobotHW::get<franka_hw::FrankaStateInterface>() { return &state_; }
template <>
inline EffortJointInterface *RobotHW::get<EffortJointInterface>() { return &effort_; }
template <>
inline PositionJointInterface *RobotHW::get<PositionJointInterface>() { return &position_; }
}  // namespace hardware_interface
