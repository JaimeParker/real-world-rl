// SPDX-License-Identifier: MIT
// Copyright (c) 2024 RAIL Berkeley (serl_franka_controllers); port: real-world-rl contributors.
//
// Compliance parameters of the SERL Cartesian impedance controller.
//
// Ported from https://github.com/rail-berkeley/serl_franka_controllers @ 1f140ef,
// cfg/compliance_param.cfg. MIT License (see LICENSE in this package).
//
// The field names, defaults and [min, max] ranges are exactly those of the
// dynamic_reconfigure .cfg file. dynamic_reconfigure clamps every value into its
// range before invoking the controller callback; clampToRanges() reproduces that.
#pragma once

#include <array>
#include <map>
#include <string>

namespace serl_controller {

struct SerlParams {
  double translational_stiffness{2000.0};
  double translational_damping{89.0};
  double rotational_stiffness{150.0};
  double rotational_damping{7.0};
  double nullspace_stiffness{0.2};
  double joint1_nullspace_stiffness{100.0};
  double translational_clip_neg_x{0.01};
  double translational_clip_neg_y{0.01};
  double translational_clip_neg_z{0.01};
  double translational_clip_x{0.01};
  double translational_clip_y{0.01};
  double translational_clip_z{0.01};
  double rotational_clip_neg_x{0.05};
  double rotational_clip_neg_y{0.05};
  double rotational_clip_neg_z{0.05};
  double rotational_clip_x{0.05};
  double rotational_clip_y{0.05};
  double rotational_clip_z{0.05};
  double translational_Ki{0.0};
  double rotational_Ki{0.0};

  /// One entry of the .cfg table (an immutable row of a static table).
  struct Field {
    const char *name;  // non-owning, points at a string literal
    double SerlParams::*member;
    double min;
    double max;
  };

  /// The 20 fields in .cfg order with their dynamic_reconfigure ranges.
  [[nodiscard]] static const std::array<Field, 20> &fields() noexcept;

  /// Clamp every field into its .cfg range, as dynamic_reconfigure's Server does.
  void clampToRanges() noexcept;

  /// Throws std::invalid_argument if any field is not finite.
  void validateFinite() const;

  [[nodiscard]] std::map<std::string, double> toMap() const;

  /// Partial merge (dynamic_reconfigure Client.update_configuration semantics):
  /// only the given keys change, then the result is clamped into range.
  /// Throws std::invalid_argument for unknown keys or non-finite values; on throw
  /// *this is unchanged.
  void merge(const std::map<std::string, double> &updates);

  [[nodiscard]] bool operator==(const SerlParams &other) const noexcept;
  [[nodiscard]] bool operator!=(const SerlParams &other) const noexcept { return !(*this == other); }
};

}  // namespace serl_controller
