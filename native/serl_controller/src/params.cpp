// SPDX-License-Identifier: MIT
// Copyright (c) 2024 RAIL Berkeley (serl_franka_controllers); port: real-world-rl contributors.
//
// Ported from https://github.com/rail-berkeley/serl_franka_controllers @ 1f140ef,
// cfg/compliance_param.cfg (the gen.add(...) table). MIT License (see LICENSE in this package).
#include "serl_controller/params.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <map>
#include <stdexcept>
#include <string>

namespace serl_controller {

const std::array<SerlParams::Field, 20> &SerlParams::fields() noexcept {
  // name, member, min, max -- exactly the gen.add(...) lines of compliance_param.cfg
  static constexpr std::array<Field, 20> kFields{{
      {"translational_stiffness", &SerlParams::translational_stiffness, 0.0, 6000.0},
      {"translational_damping", &SerlParams::translational_damping, 0.0, 400.0},
      {"rotational_stiffness", &SerlParams::rotational_stiffness, 0.0, 300.0},
      {"rotational_damping", &SerlParams::rotational_damping, 0.0, 30.0},
      {"nullspace_stiffness", &SerlParams::nullspace_stiffness, 0.0, 100.0},
      {"joint1_nullspace_stiffness", &SerlParams::joint1_nullspace_stiffness, 0.0, 100.0},
      {"translational_clip_neg_x", &SerlParams::translational_clip_neg_x, 0.0, 0.1},
      {"translational_clip_neg_y", &SerlParams::translational_clip_neg_y, 0.0, 0.1},
      {"translational_clip_neg_z", &SerlParams::translational_clip_neg_z, 0.0, 0.1},
      {"translational_clip_x", &SerlParams::translational_clip_x, 0.0, 0.1},
      {"translational_clip_y", &SerlParams::translational_clip_y, 0.0, 0.1},
      {"translational_clip_z", &SerlParams::translational_clip_z, 0.0, 0.1},
      {"rotational_clip_neg_x", &SerlParams::rotational_clip_neg_x, 0.0, 0.1},
      {"rotational_clip_neg_y", &SerlParams::rotational_clip_neg_y, 0.0, 0.1},
      {"rotational_clip_neg_z", &SerlParams::rotational_clip_neg_z, 0.0, 0.1},
      {"rotational_clip_x", &SerlParams::rotational_clip_x, 0.0, 0.1},
      {"rotational_clip_y", &SerlParams::rotational_clip_y, 0.0, 0.1},
      {"rotational_clip_z", &SerlParams::rotational_clip_z, 0.0, 0.1},
      {"translational_Ki", &SerlParams::translational_Ki, 0.0, 100.0},
      {"rotational_Ki", &SerlParams::rotational_Ki, 0.0, 100.0},
  }};
  return kFields;
}

void SerlParams::clampToRanges() noexcept {
  for (const auto &field : fields())
    this->*field.member = std::min(std::max(this->*field.member, field.min), field.max);
}

void SerlParams::validateFinite() const {
  for (const auto &field : fields()) {
    if (!std::isfinite(this->*field.member)) {
      throw std::invalid_argument(std::string("SERL parameter '") + field.name + "' must be finite");
    }
  }
}

std::map<std::string, double> SerlParams::toMap() const {
  std::map<std::string, double> out;
  for (const auto &field : fields()) out[field.name] = this->*field.member;
  return out;
}

void SerlParams::merge(const std::map<std::string, double> &updates) {
  SerlParams merged = *this;
  for (const auto &[key, value] : updates) {
    const auto &all = fields();
    const auto it = std::find_if(all.begin(), all.end(), [&key](const Field &f) { return key == f.name; });
    if (it == all.end()) throw std::invalid_argument("unknown SERL parameter '" + key + "'");
    merged.*(it->member) = value;
  }
  merged.validateFinite();
  merged.clampToRanges();
  *this = merged;
}

bool SerlParams::operator==(const SerlParams &other) const noexcept {
  for (const auto &field : fields()) {
    if (this->*field.member != other.*field.member) return false;
  }
  return true;
}

}  // namespace serl_controller
