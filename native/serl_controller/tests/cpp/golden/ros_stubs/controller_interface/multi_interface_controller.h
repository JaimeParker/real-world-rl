// SPDX-License-Identifier: MIT
// Minimal ROS stand-in for compiling the vendored SERL controller (see README.md); no control math.
#pragma once
#include <controller_interface/controller_base.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
namespace controller_interface {
template <class... Interfaces>
class MultiInterfaceController : public ControllerBase {
 public:
  virtual bool init(hardware_interface::RobotHW *, ros::NodeHandle &) { return true; }
  virtual void starting(const ros::Time &) {}
  virtual void update(const ros::Time &, const ros::Duration &) = 0;
};
}  // namespace controller_interface
