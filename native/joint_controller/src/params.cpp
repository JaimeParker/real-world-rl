// SPDX-License-Identifier: MIT AND Apache-2.0
// Copyright (c) real-world-rl contributors. Default gains: GELLO (Copyright (c) 2025 Franka
// Robotics GmbH, Apache-2.0); joint limits: franka_description (Franka Robotics GmbH,
// Apache-2.0). See params.hpp and LICENSE.
#include "joint_controller/params.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <string>
#include <variant>

namespace joint_controller {

namespace {

using P = TrajectoryImpedanceParams;

std::string jointName(const char *field, int joint) { return std::string(field) + "[" + std::to_string(joint) + "]"; }

void requireFinite(const char *name, double value) {
  if (!std::isfinite(value)) throw std::invalid_argument(std::string("parameter '") + name + "' must be finite");
}

void requireFinite(const char *name, const Vector7d &value) {
  for (int i = 0; i < 7; ++i) {
    if (!std::isfinite(value(i))) {
      throw std::invalid_argument("parameter '" + jointName(name, i) + "' must be finite (joint limits are required)");
    }
  }
}

void requirePositive(const char *name, double value) {
  if (!(value > 0.0)) throw std::invalid_argument(std::string("parameter '") + name + "' must be > 0");
}

void requirePositive(const char *name, const Vector7d &value) {
  for (int i = 0; i < 7; ++i) {
    if (!(value(i) > 0.0)) throw std::invalid_argument("parameter '" + jointName(name, i) + "' must be > 0");
  }
}

void requireNonNegative(const char *name, const Vector7d &value) {
  for (int i = 0; i < 7; ++i) {
    if (!(value(i) >= 0.0)) throw std::invalid_argument("parameter '" + jointName(name, i) + "' must be >= 0");
  }
}

const char *kindName(P::Kind kind) noexcept {
  switch (kind) {
    case P::Kind::kScalar:
      return "a number";
    case P::Kind::kFlag:
      return "a bool";
    case P::Kind::kVector7:
      return "a vector of 7 numbers";
  }
  return "?";
}

}  // namespace

JointPositionLimits frankaJointPositionLimits(const std::string &robot_model) {
  // franka_ros @ ddd2fff franka_description/robots/{panda,fr3}/joint_limits.yaml (position limits),
  // the same tables as serl_controller's frankaPandaJointLimits / frankaFr3JointLimits.
  JointPositionLimits l;
  if (robot_model == "panda") {
    l.lower << -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973;
    l.upper << 2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973;
    return l;
  }
  if (robot_model == "fr3") {
    l.lower << -2.3093, -1.5133, -2.4937, -2.7478, -2.4800, 0.8521, -2.6895;
    l.upper << 2.3093, 1.5133, 2.4937, -0.4461, 2.4800, 4.2094, 2.6895;
    return l;
  }
  throw std::invalid_argument("robot_model must be 'panda' or 'fr3', got '" + robot_model + "'");
}

Vector7d TrajectoryImpedanceParams::gelloStiffness() noexcept {
  // GELLO controllers.yaml k_gains
  return (Vector7d() << 240.0, 240.0, 240.0, 240.0, 100.0, 60.0, 20.0).finished();
}

Vector7d TrajectoryImpedanceParams::gelloDamping() noexcept {
  // GELLO controllers.yaml d_gains
  return (Vector7d() << 20.0, 20.0, 20.0, 10.0, 10.0, 10.0, 5.0).finished();
}

Vector7d TrajectoryImpedanceParams::defaultMaxVelocity() noexcept {
  return (Vector7d() << 0.6, 0.6, 0.6, 0.6, 0.75, 0.75, 0.75).finished();
}

const std::array<P::Field, P::kNumFields> &TrajectoryImpedanceParams::fields() noexcept {
  static const std::array<Field, kNumFields> kFields{{
      {"stiffness", Kind::kVector7, nullptr, nullptr, &P::stiffness},
      {"damping", Kind::kVector7, nullptr, nullptr, &P::damping},
      {"velocity_filter_alpha", Kind::kScalar, &P::velocity_filter_alpha, nullptr, nullptr},
      {"max_velocity", Kind::kVector7, nullptr, nullptr, &P::max_velocity},
      {"max_acceleration", Kind::kVector7, nullptr, nullptr, &P::max_acceleration},
      {"min_segment_duration_s", Kind::kScalar, &P::min_segment_duration_s, nullptr, nullptr},
      {"lower_limit", Kind::kVector7, nullptr, nullptr, &P::lower_limit},
      {"upper_limit", Kind::kVector7, nullptr, nullptr, &P::upper_limit},
      {"limit_margin", Kind::kScalar, &P::limit_margin, nullptr, nullptr},
      {"command_timeout_s", Kind::kScalar, &P::command_timeout_s, nullptr, nullptr},
      {"max_delta_tau", Kind::kScalar, &P::max_delta_tau, nullptr, nullptr},
      {"compensate_coriolis", Kind::kFlag, nullptr, &P::compensate_coriolis, nullptr},
  }};
  return kFields;
}

const P::Field *TrajectoryImpedanceParams::findField(const std::string &name) noexcept {
  const auto &all = fields();
  const auto it = std::find_if(all.begin(), all.end(), [&name](const Field &f) { return name == f.name; });
  return it == all.end() ? nullptr : &*it;
}

void TrajectoryImpedanceParams::validate() const {
  requireFinite("stiffness", stiffness);
  requireNonNegative("stiffness", stiffness);
  requireFinite("damping", damping);
  requireNonNegative("damping", damping);
  requireFinite("velocity_filter_alpha", velocity_filter_alpha);
  if (velocity_filter_alpha < 0.0 || velocity_filter_alpha > 1.0) {  // finite: checked above
    throw std::invalid_argument("parameter 'velocity_filter_alpha' must be in [0, 1]");
  }
  requireFinite("max_velocity", max_velocity);
  requirePositive("max_velocity", max_velocity);
  requireFinite("max_acceleration", max_acceleration);
  requirePositive("max_acceleration", max_acceleration);
  requireFinite("min_segment_duration_s", min_segment_duration_s);
  requirePositive("min_segment_duration_s", min_segment_duration_s);
  requireFinite("lower_limit", lower_limit);
  requireFinite("upper_limit", upper_limit);
  requireFinite("limit_margin", limit_margin);
  if (!(limit_margin >= 0.0)) throw std::invalid_argument("parameter 'limit_margin' must be >= 0");
  for (int i = 0; i < 7; ++i) {
    if (!(lower_limit(i) < upper_limit(i))) {
      throw std::invalid_argument("parameter 'lower_limit[" + std::to_string(i) + "]' must be < 'upper_limit[" +
                                  std::to_string(i) + "]'");
    }
    if (!(lower_limit(i) + 2.0 * limit_margin < upper_limit(i))) {
      throw std::invalid_argument("parameter 'limit_margin' leaves no room between lower_limit[" + std::to_string(i) +
                                  "] and upper_limit[" + std::to_string(i) + "]");
    }
  }
  requireFinite("command_timeout_s", command_timeout_s);
  requirePositive("command_timeout_s", command_timeout_s);
  requireFinite("max_delta_tau", max_delta_tau);
  requirePositive("max_delta_tau", max_delta_tau);
}

void TrajectoryImpedanceParams::merge(const ParamUpdates &updates) {
  TrajectoryImpedanceParams merged = *this;
  for (const auto &[key, value] : updates) {
    const Field *field = findField(key);
    if (field == nullptr) throw std::invalid_argument("unknown joint_controller parameter '" + key + "'");
    bool ok = false;
    switch (field->kind) {
      case Kind::kScalar:
        if (const auto *v = std::get_if<double>(&value)) {
          merged.*(field->scalar) = *v;
          ok = true;
        }
        break;
      case Kind::kFlag:
        if (const auto *v = std::get_if<bool>(&value)) {
          merged.*(field->flag) = *v;
          ok = true;
        }
        break;
      case Kind::kVector7:
        if (const auto *v = std::get_if<Vector7d>(&value)) {
          merged.*(field->vector) = *v;
          ok = true;
        }
        break;
    }
    if (!ok) throw std::invalid_argument("parameter '" + key + "' must be " + kindName(field->kind));
  }
  merged.validate();
  *this = merged;
}

ParamUpdates TrajectoryImpedanceParams::toMap() const {
  ParamUpdates out;
  for (const auto &field : fields()) {
    switch (field.kind) {
      case Kind::kScalar:
        out[field.name] = this->*field.scalar;
        break;
      case Kind::kFlag:
        out[field.name] = this->*field.flag;
        break;
      case Kind::kVector7:
        out[field.name] = this->*field.vector;
        break;
    }
  }
  return out;
}

bool TrajectoryImpedanceParams::operator==(const TrajectoryImpedanceParams &other) const noexcept {
  for (const auto &field : fields()) {
    switch (field.kind) {
      case Kind::kScalar:
        if (this->*field.scalar != other.*field.scalar) return false;
        break;
      case Kind::kFlag:
        if (this->*field.flag != other.*field.flag) return false;
        break;
      case Kind::kVector7:
        // NaN (an unset limit) compares unequal, like the scalars.
        if ((this->*field.vector).array().cwiseNotEqual((other.*field.vector).array()).any()) return false;
        break;
    }
  }
  return true;
}

TrajectoryImpedanceParams defaultParams(const std::string &robot_model) {
  const JointPositionLimits limits = frankaJointPositionLimits(robot_model);
  TrajectoryImpedanceParams p;
  p.lower_limit = limits.lower;
  p.upper_limit = limits.upper;
  p.validate();
  return p;
}

}  // namespace joint_controller
