// SPDX-License-Identifier: MIT
// Minimal ROS stand-in for compiling the vendored SERL controller (see README.md); no control math.
#pragma once
namespace controller_interface {
class ControllerBase {
 public:
  virtual ~ControllerBase() = default;
};
}  // namespace controller_interface
