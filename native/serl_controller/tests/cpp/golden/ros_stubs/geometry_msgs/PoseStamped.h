// SPDX-License-Identifier: MIT
// Minimal ROS stand-in for compiling the vendored SERL controller (see README.md); no control math.
#pragma once
#include <memory>
namespace geometry_msgs {
struct Point {
  double x{0}, y{0}, z{0};
};
struct Quaternion {
  double x{0}, y{0}, z{0}, w{1};
};
struct Pose {
  Point position;
  Quaternion orientation;
};
struct PoseStamped {
  Pose pose;
};
using PoseStampedConstPtr = std::shared_ptr<const PoseStamped>;
}  // namespace geometry_msgs
