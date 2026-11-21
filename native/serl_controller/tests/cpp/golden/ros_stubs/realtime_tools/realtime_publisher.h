// SPDX-License-Identifier: MIT
// Minimal ROS stand-in for compiling the vendored SERL controller (see README.md); no control math.
#pragma once
#include <string>
#include <ros/node_handle.h>
namespace realtime_tools {
template <class Msg>
class RealtimePublisher {
 public:
  void init(const ros::NodeHandle &, const std::string &, int) {}
  bool trylock() { return true; }
  void unlockAndPublish() { last_published_ = msg_; ++published_count_; }
  Msg msg_{};
  Msg last_published_{};
  long published_count_{0};
};
}  // namespace realtime_tools
