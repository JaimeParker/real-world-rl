// SPDX-License-Identifier: MIT
// Minimal ROS stand-in for compiling the vendored SERL controller (see README.md); no control math.
#pragma once
// Stand-in for dynamic_reconfigure::Server. Like the real server, setCallback()
// immediately invokes the callback once with the current (default) config.
// Clamping into the .cfg ranges is done by the test before calling update(),
// mirroring the real server's __clamp__().
#include <functional>
#include <cstdint>
#include <ros/node_handle.h>
namespace dynamic_reconfigure {
template <class ConfigType>
class Server {
 public:
  using CallbackType = std::function<void(ConfigType &, uint32_t)>;
  explicit Server(const ros::NodeHandle &) {}
  void setCallback(const CallbackType &callback) {
    callback_ = callback;
    callback_(config_, ~0u);
  }
  void updateConfig(const ConfigType &config) {
    config_ = config;
    callback_(config_, 0u);
  }

 private:
  CallbackType callback_;
  ConfigType config_{};
};
}  // namespace dynamic_reconfigure
