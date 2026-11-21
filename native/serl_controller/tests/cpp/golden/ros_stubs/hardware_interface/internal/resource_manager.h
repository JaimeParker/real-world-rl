// SPDX-License-Identifier: MIT
// Minimal ROS stand-in for compiling the vendored SERL controller (see README.md); no control math.
#pragma once
// Minimal stand-in for hardware_interface::ResourceManager (a name -> handle map), enough
// for the vendored joint_limits_interface.h to compile; the golden test calls the handles
// directly, the way JointLimitsInterface::enforceLimits iterates them.
#include <map>
#include <stdexcept>
#include <string>
namespace hardware_interface {
template <class ResourceHandle>
class ResourceManager {
 public:
  virtual ~ResourceManager() = default;
  void registerHandle(const ResourceHandle &handle) { resource_map_[handle.getName()] = handle; }
  ResourceHandle getHandle(const std::string &name) {
    auto it = resource_map_.find(name);
    if (it == resource_map_.end()) throw std::logic_error("Could not find resource '" + name + "'");
    return it->second;
  }

 protected:
  std::map<std::string, ResourceHandle> resource_map_;
};
}  // namespace hardware_interface
