// SPDX-License-Identifier: MIT
// Copyright (c) real-world-rl contributors. See planner.hpp for the derivation.
#include "joint_controller/planner.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <optional>

namespace joint_controller {

namespace {

// d^k q / ds^k of q(s) = sum b_i s^i, evaluated by Horner's rule.
double d1(const std::array<double, 6> &b, double s) noexcept {
  return b[1] + s * (2.0 * b[2] + s * (3.0 * b[3] + s * (4.0 * b[4] + s * 5.0 * b[5])));
}
double d2(const std::array<double, 6> &b, double s) noexcept {
  return 2.0 * b[2] + s * (6.0 * b[3] + s * (12.0 * b[4] + s * 20.0 * b[5]));
}
double d3(const std::array<double, 6> &b, double s) noexcept {
  return 6.0 * b[3] + s * (24.0 * b[4] + s * 60.0 * b[5]);
}

// Real roots of A s^2 + B s + C in the open interval (0, 1), ascending. Returns their count.
int quadraticRootsInUnit(double A, double B, double C, std::array<double, 2> &out) noexcept {
  std::array<double, 2> r{};
  int n = 0;
  if (A == 0.0) {
    if (B != 0.0) r[n++] = -C / B;
  } else {
    const double disc = B * B - 4.0 * A * C;
    if (disc >= 0.0) {
      // Numerically stable form: no cancellation between B and sqrt(disc).
      const double q = -0.5 * (B + std::copysign(std::sqrt(disc), B));
      if (q != 0.0) {
        r[n++] = q / A;
        r[n++] = C / q;
      } else {
        r[n++] = 0.0;  // B = 0 and C = 0
      }
    }
  }
  int m = 0;
  for (int i = 0; i < n; ++i) {
    if (r[i] > 0.0 && r[i] < 1.0) out[m++] = r[i];
  }
  if (m == 2 && out[0] > out[1]) std::swap(out[0], out[1]);
  return m;
}

double position(const std::array<double, 6> &b, double s) noexcept {
  return b[0] + s * (b[1] + s * (b[2] + s * (b[3] + s * (b[4] + s * b[5]))));
}

// Root of f on [lo, hi] where f(lo) and f(hi) have opposite signs and f is monotone there:
// Newton's method (derivative df) kept inside the shrinking bracket, bisection otherwise.
template <typename F, typename DF>
double bracketedRoot(const F &f, const DF &df, double lo, double hi) noexcept {
  if (f(lo) > 0.0) std::swap(lo, hi);  // orient so that f(lo) < 0 < f(hi)
  double x = 0.5 * (lo + hi);
  for (int it = 0; it < 64; ++it) {
    const double fx = f(x);
    if (fx == 0.0) return x;
    if (fx < 0.0) {
      lo = x;
    } else {
      hi = x;
    }
    const double dfx = df(x);
    double next = 0.5 * (lo + hi);
    if (dfx != 0.0) {
      const double newton = x - fx / dfx;
      if (newton > std::min(lo, hi) && newton < std::max(lo, hi)) next = newton;
    }
    if (std::abs(next - x) <= 1e-15) return next;
    x = next;
  }
  return x;
}

bool oppositeSigns(double a, double b) noexcept { return (a < 0.0 && b > 0.0) || (a > 0.0 && b < 0.0); }

bool withinLimits(const Quintic::Extrema &e, double v_max, double a_max) noexcept {
  return std::max(-e.v_min, e.v_max) <= v_max && std::max(-e.a_min, e.a_max) <= a_max;
}

// The segment stays within [lower, upper], widened to the start point if it starts outside (it
// may then move back in, never further out).
bool withinPositionLimits(const Quintic::Extrema &e, double p0, double lower, double upper) noexcept {
  return e.p_min >= std::min(lower, p0) && e.p_max <= std::max(upper, p0);
}

// Smallest feasible T on the grid t_start * kGridGrowth^k (k = 0 .. kGridSteps), refined by
// bisection against the previous (infeasible) grid point.
template <typename Feasible>
std::optional<double> smallestFeasible(double t_start, const Feasible &feasible) noexcept {
  if (feasible(t_start)) return t_start;
  double lo = t_start;
  double hi = t_start;
  bool found = false;
  for (int k = 1; k <= kGridSteps; ++k) {
    lo = hi;
    hi = lo * kGridGrowth;
    if (feasible(hi)) {
      found = true;
      break;
    }
  }
  if (!found) return std::nullopt;
  for (int k = 0; k < kBisectionSteps; ++k) {
    const double mid = 0.5 * (lo + hi);
    if (feasible(mid)) {
      hi = mid;
    } else {
      lo = mid;
    }
  }
  return hi;
}

JointSample jointSample(const ReferenceState &s, int j) noexcept { return {s.q(j), s.dq(j), s.ddq(j)}; }

}  // namespace

// ---------------------------------------------------------------------------------------------
Quintic Quintic::hold(double p) noexcept {
  Quintic q;
  q.b_[0] = p;
  q.end_ = p;
  return q;
}

Quintic Quintic::toRest(double p0, double v0, double a0, double pf, double T) noexcept {
  Quintic q;
  const double h = pf - p0;
  const double vT = v0 * T;
  const double aT2 = a0 * T * T;
  q.b_ = {p0,
          vT,
          0.5 * aT2,
          10.0 * h - 6.0 * vT - 1.5 * aT2,
          -15.0 * h + 8.0 * vT + 1.5 * aT2,
          6.0 * h - 3.0 * vT - 0.5 * aT2};
  q.duration_ = T;
  q.end_ = pf;
  return q;
}

Quintic Quintic::stop(double p0, double v0, double a0, double T) noexcept {
  // toRest() with h = v0 T / 2 + a0 T^2 / 12, which makes b5 = 0:
  //   b3 = -v0 T - (2/3) a0 T^2,  b4 = v0 T / 2 + a0 T^2 / 4.
  Quintic q;
  const double vT = v0 * T;
  const double aT2 = a0 * T * T;
  q.b_ = {p0, vT, 0.5 * aT2, -vT - (2.0 / 3.0) * aT2, 0.5 * vT + 0.25 * aT2, 0.0};
  q.duration_ = T;
  q.end_ = p0 + 0.5 * vT + aT2 / 12.0;
  return q;
}

JointSample Quintic::sample(double t) const noexcept {
  if (!(t < duration_)) return {end_, 0.0, 0.0};  // also duration_ == 0 (hold)
  const double s = t / duration_;
  const auto &b = b_;
  const double p = b[0] + s * (b[1] + s * (b[2] + s * (b[3] + s * (b[4] + s * b[5]))));
  return {p, d1(b, s) / duration_, d2(b, s) / (duration_ * duration_)};
}

Quintic::Extrema Quintic::extrema() const noexcept {
  if (!(duration_ > 0.0)) return {0.0, 0.0, 0.0, 0.0, end_, end_};
  const auto &b = b_;
  const auto f1 = [&b](double s) { return d1(b, s); };
  const auto f2 = [&b](double s) { return d2(b, s); };
  const auto f3 = [&b](double s) { return d3(b, s); };
  // Acceleration extrema: at 0, 1 and the roots of q_sss (a quadratic). Between consecutive such
  // points q_ss is monotone, so it has at most one root there: a velocity extremum. Between
  // consecutive velocity extrema q_s is monotone: at most one root, a position extremum.
  std::array<double, 2> r3{};
  const int n3 = quadraticRootsInUnit(60.0 * b[5], 24.0 * b[4], 6.0 * b[3], r3);
  std::array<double, 4> breaks3{};
  int nb3 = 0;
  breaks3[nb3++] = 0.0;
  for (int i = 0; i < n3; ++i) breaks3[nb3++] = r3[i];
  breaks3[nb3++] = 1.0;

  Extrema e{f1(0.0), f1(0.0), f2(0.0), f2(0.0), b[0], b[0]};
  std::array<double, 5> breaks2{};  // 0, roots of q_ss in (0, 1), 1
  int nb2 = 0;
  breaks2[nb2++] = 0.0;
  for (int i = 1; i < nb3; ++i) {
    const double l = breaks3[i - 1];
    const double r = breaks3[i];
    const double a_r = f2(r);
    e.a_min = std::min(e.a_min, a_r);
    e.a_max = std::max(e.a_max, a_r);
    if (oppositeSigns(f2(l), a_r)) breaks2[nb2++] = bracketedRoot(f2, f3, l, r);
  }
  breaks2[nb2++] = 1.0;
  for (int i = 1; i < nb2; ++i) {
    const double l = breaks2[i - 1];
    const double r = breaks2[i];
    const double v_r = f1(r);
    e.v_min = std::min(e.v_min, v_r);
    e.v_max = std::max(e.v_max, v_r);
    const double p_r = position(b, r);
    if (oppositeSigns(f1(l), v_r)) {
      const double p_x = position(b, bracketedRoot(f1, f2, l, r));
      e.p_min = std::min(e.p_min, p_x);
      e.p_max = std::max(e.p_max, p_x);
    }
    e.p_min = std::min(e.p_min, p_r);
    e.p_max = std::max(e.p_max, p_r);
  }
  const double inv_T = 1.0 / duration_;
  e.v_min *= inv_T;
  e.v_max *= inv_T;
  e.a_min *= inv_T * inv_T;
  e.a_max *= inv_T * inv_T;
  return e;
}

bool Quintic::within(double v_max, double a_max, double p_lower, double p_upper) const noexcept {
  if (!(duration_ > 0.0)) return end_ >= p_lower && end_ <= p_upper;
  // Cheap sufficient test first: on s in [0, 1], |sum c_k s^k| <= sum |c_k|.
  const auto &b = b_;
  const double inv_T = 1.0 / duration_;
  double p_bound = 0.0;
  double v_bound = 0.0;
  double a_bound = 0.0;
  for (int k = 1; k < 6; ++k) {
    const double m = std::abs(b[static_cast<size_t>(k)]);
    p_bound += m;
    v_bound += k * m;
    a_bound += k * (k - 1) * m;
  }
  if (v_bound * inv_T <= v_max && a_bound * inv_T * inv_T <= a_max && b[0] - p_bound >= p_lower &&
      b[0] + p_bound <= p_upper) {
    return true;
  }
  const Extrema e = extrema();
  return withinLimits(e, v_max, a_max) && e.p_min >= p_lower && e.p_max <= p_upper;
}

// ---------------------------------------------------------------------------------------------
double doubleIntegratorMinTime(double h, double v0, double a_max) noexcept {
  if (h < 0.0) {  // mirror so that h >= 0
    h = -h;
    v0 = -v0;
  }
  const double stopping = v0 * std::abs(v0) / (2.0 * a_max);  // signed distance to stop at full braking
  if (h >= stopping) return (-v0 + 2.0 * std::sqrt(0.5 * v0 * v0 + a_max * h)) / a_max;
  // overshoots: brake through h, come back
  return (v0 + 2.0 * std::sqrt(std::max(0.0, 0.5 * v0 * v0 - a_max * h))) / a_max;
}

std::optional<double> commonDuration(const ReferenceState &from, const Vector7d &target,
                                     const ReferenceLimits &limits) noexcept {
  bool moving = false;
  double t_start = limits.min_segment_duration_s;
  for (int j = 0; j < 7; ++j) {
    const double h = target(j) - from.q(j);
    const double v0 = from.dq(j);
    const double a0 = from.ddq(j);
    if (h != 0.0 || v0 != 0.0 || a0 != 0.0) moving = true;
    // The initial state is part of every candidate segment.
    if (std::abs(v0) > limits.max_velocity(j) || std::abs(a0) > limits.max_acceleration(j)) return std::nullopt;
    t_start = std::max(t_start, std::abs(h) / limits.max_velocity(j));
    t_start = std::max(t_start, doubleIntegratorMinTime(h, v0, limits.max_acceleration(j)));
  }
  if (!moving) return 0.0;
  if (!std::isfinite(t_start)) return std::nullopt;
  int first = 0;  // check the joint that failed last first: most candidates fail on it
  const auto feasible = [&](double T) {
    for (int i = 0; i < 7; ++i) {
      const int j = (first + i) % 7;
      const Quintic q = Quintic::toRest(from.q(j), from.dq(j), from.ddq(j), target(j), T);
      if (!q.within(limits.max_velocity(j), limits.max_acceleration(j), std::min(limits.lower(j), from.q(j)),
                    std::max(limits.upper(j), from.q(j)))) {
        first = j;
        return false;
      }
    }
    return true;
  };
  return smallestFeasible(t_start, feasible);
}

std::optional<double> stopDuration(const JointSample &from, double max_velocity, double max_acceleration,
                                   double min_duration, double lower, double upper) noexcept {
  const double v0 = from.v;
  const double a0 = from.a;
  if (v0 == 0.0 && a0 == 0.0) return 0.0;
  if (std::abs(v0) > max_velocity || std::abs(a0) > max_acceleration) return std::nullopt;
  const double sign = v0 != 0.0 ? std::copysign(1.0, v0) : std::copysign(1.0, a0);
  const auto feasible = [&](double T) {
    const Quintic q = Quintic::stop(from.p, v0, a0, T);
    const Quintic::Extrema e = q.extrema();
    if (!withinLimits(e, max_velocity, max_acceleration)) return false;
    if (!withinPositionLimits(e, from.p, lower, upper)) return false;
    // No reversal: the velocity keeps the sign of the motion (rounding at s = 1 tolerated).
    const double tol = 1e-12 * (std::abs(v0) + std::abs(a0) * T);
    return sign > 0.0 ? e.v_min >= -tol : e.v_max <= tol;
  };
  const double t_start = std::max(min_duration, std::abs(v0) / max_acceleration);
  if (const auto T = smallestFeasible(t_start, feasible)) return T;
  if (v0 * a0 < 0.0) {  // decelerating: v(s) = v0 (1 - s)^3, |a| <= |a0|
    const double T = 3.0 * std::abs(v0) / std::abs(a0);
    if (T > 0.0 && std::isfinite(T) && feasible(T)) return T;
  }
  return std::nullopt;
}

// ---------------------------------------------------------------------------------------------
void ReferenceGenerator::reset(const Vector7d &q) noexcept {
  for (int j = 0; j < 7; ++j) {
    segments_[static_cast<size_t>(j)] = Quintic::hold(q(j));
    time_[static_cast<size_t>(j)] = 0.0;
    stop_pending_[static_cast<size_t>(j)] = false;
  }
  goal_ = q;
  segment_time_ = 0.0;
  segment_duration_ = 0.0;
  sampleAll();
}

void ReferenceGenerator::sampleAll() noexcept {
  for (int j = 0; j < 7; ++j) {
    const JointSample s = segments_[static_cast<size_t>(j)].sample(time_[static_cast<size_t>(j)]);
    state_.q(j) = s.p;
    state_.dq(j) = s.v;
    state_.ddq(j) = s.a;
  }
}

void ReferenceGenerator::advance(double dt) noexcept {
  if (!(dt > 0.0) || !std::isfinite(dt)) return;
  for (size_t j = 0; j < 7; ++j) {
    // Saturate at the segment end so the clock never grows without bound while holding.
    time_[j] = std::min(time_[j] + dt, segments_[j].duration());
  }
  segment_time_ += dt;
  sampleAll();
}

bool ReferenceGenerator::planTo(const Vector7d &target, const ReferenceLimits &limits) noexcept {
  const auto T = commonDuration(state_, target, limits);
  if (!T) return false;
  for (int j = 0; j < 7; ++j) {
    const auto k = static_cast<size_t>(j);
    segments_[k] =
        *T > 0.0 ? Quintic::toRest(state_.q(j), state_.dq(j), state_.ddq(j), target(j), *T) : Quintic::hold(target(j));
    time_[k] = 0.0;
    stop_pending_[k] = false;
  }
  goal_ = target;
  segment_time_ = 0.0;
  segment_duration_ = *T;
  sampleAll();  // same instant: q, dq, ddq unchanged up to rounding (exactly the target if T == 0)
  return true;
}

bool ReferenceGenerator::tryStopJoint(int joint, const ReferenceLimits &limits) noexcept {
  const auto k = static_cast<size_t>(joint);
  const JointSample from = jointSample(state_, joint);
  const auto T = stopDuration(from, limits.max_velocity(joint), limits.max_acceleration(joint),
                              limits.min_segment_duration_s, limits.lower(joint), limits.upper(joint));
  if (!T) return false;
  segments_[k] = *T > 0.0 ? Quintic::stop(from.p, from.v, from.a, *T) : Quintic::hold(from.p);
  time_[k] = 0.0;
  goal_(joint) = segments_[k].end();
  segment_duration_ = std::max(segment_duration_, segment_time_ + *T);
  return true;
}

void ReferenceGenerator::planStop(const ReferenceLimits &limits) noexcept {
  segment_time_ = 0.0;
  segment_duration_ = 0.0;
  for (int j = 0; j < 7; ++j) {
    const auto k = static_cast<size_t>(j);
    stop_pending_[k] = !tryStopJoint(j, limits);
    if (stop_pending_[k]) {
      // Keeps its (feasible, rest-ending) segment for now.
      segment_duration_ = std::max(segment_duration_, segments_[k].duration() - time_[k]);
    }
  }
  sampleAll();
}

bool ReferenceGenerator::retryStop(const ReferenceLimits &limits) noexcept {
  bool changed = false;
  for (int j = 0; j < 7; ++j) {
    const auto k = static_cast<size_t>(j);
    if (stop_pending_[k] && tryStopJoint(j, limits)) {
      stop_pending_[k] = false;
      changed = true;
    }
  }
  if (changed) sampleAll();
  return stopPending();
}

bool ReferenceGenerator::stopPending() const noexcept {
  return std::any_of(stop_pending_.begin(), stop_pending_.end(), [](bool p) { return p; });
}

bool ReferenceGenerator::atRest() const noexcept {
  for (size_t j = 0; j < 7; ++j) {
    if (time_[j] < segments_[j].duration()) return false;
  }
  return true;
}

}  // namespace joint_controller
