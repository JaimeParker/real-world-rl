// SPDX-License-Identifier: MIT
// Minimal ROS stand-in for compiling the vendored SERL controller (see README.md); no control math.
#pragma once
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <franka_hw/fake_hardware.h>
#include <ros/time.h>

namespace ros {

struct TransportHints {
  TransportHints &reliable() { return *this; }
  TransportHints &tcpNoDelay() { return *this; }
};

struct Subscriber {};

namespace stub {
// Callback registry: the test calls the equilibrium-pose callback through this.
struct Registry {
  std::function<void(const void *)> equilibrium_pose;
  static Registry &instance() {
    static Registry r;
    return r;
  }
};
}  // namespace stub

class NodeHandle {
 public:
  NodeHandle() = default;
  explicit NodeHandle(const std::string &ns) : ns_(ns) {}
  std::string getNamespace() const { return ns_; }

  bool getParam(const std::string &key, std::string &value) const {
    if (key != "arm_id") return false;
    value = "panda";
    return true;
  }
  bool getParam(const std::string &key, std::vector<std::string> &value) const {
    if (key != "joint_names") return false;
    value.clear();
    for (int i = 1; i <= 7; ++i) value.push_back("panda_joint" + std::to_string(i));
    return true;
  }

  bool getParam(const std::string &key, std::vector<double> &value) const {
    if (key != "/target_joint_positions") return false;
    value = franka_hw::FakeHardware::instance().target_joint_positions;
    return true;
  }

  template <class M, class T>
  Subscriber subscribe(const std::string &, unsigned, void (T::*fp)(const std::shared_ptr<const M> &), T *obj,
                       const TransportHints &) {
    stub::Registry::instance().equilibrium_pose = [fp, obj](const void *msg) {
      (obj->*fp)(*static_cast<const std::shared_ptr<const M> *>(msg));
    };
    return {};
  }

 private:
  std::string ns_;
};

}  // namespace ros
