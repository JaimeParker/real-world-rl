// SPDX-License-Identifier: MIT
// The single definition of the numerical-equivalence acceptance criterion.
#pragma once

#include <algorithm>
#include <cmath>

namespace serl_controller::test {

/// A component of our port agrees with the original SERL value iff
///   |ours - serl| <= kAbsTol   and   |ours - serl| / max(|ours|, |serl|) <= kRelTol
/// (the relative error of two exact zeros is 0). Both compared in double.
inline constexpr double kAbsTol = 1e-12;
inline constexpr double kRelTol = 1e-12;

inline double absError(double ours, double serl) { return std::abs(ours - serl); }

inline double relError(double ours, double serl) {
  const double diff = std::abs(ours - serl);
  if (diff == 0.0) return 0.0;
  return diff / std::max(std::abs(ours), std::abs(serl));
}

inline bool withinTolerance(double ours, double serl) {
  return absError(ours, serl) <= kAbsTol && relError(ours, serl) <= kRelTol;
}

}  // namespace serl_controller::test
