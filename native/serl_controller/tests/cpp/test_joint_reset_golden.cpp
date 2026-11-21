// SPDX-License-Identifier: MIT
//
// Golden equivalence: SerlJointPositionLaw vs the ORIGINAL SERL
// JointPositionController (serl_franka_controllers @ 1f140ef, vendored verbatim
// under golden/serl_original and compiled against ROS stand-ins whose
// ros::Duration reproduces roscpp_core). Both are driven like franka_control
// drives the original: starting() + update(period 0) on the first libfranka tick,
// then update(period) with whole-millisecond periods (1 ms nominal, 2-5 ms when
// packets are lost). Every tick compares the commanded joint positions and the
// elapsed time with the tolerance defined in tolerance.hpp.
#include <doctest/doctest.h>

#include <Eigen/Dense>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <map>
#include <random>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <string>

#include "golden/ros_stubs/ros/time.h"
#include "golden/serl_joint_reference.hpp"
#include "serl_controller/joint_reset.hpp"
#include "tolerance.hpp"

using serl_controller::SerlJointPositionLaw;
using serl_controller::golden::SerlJointReference;
namespace tol = serl_controller::test;
using Vector7d = Eigen::Matrix<double, 7, 1>;

namespace {

// Panda joint limits (rad), as in franka_description.
const Vector7d kLower = (Vector7d() << -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973).finished();
const Vector7d kUpper = (Vector7d() << 2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973).finished();

struct Stat {
  double max_abs{0.0};
  double max_rel{0.0};
  long components{0};
  long violations{0};
};

struct Stats {
  std::map<std::string, Stat> by_name;
  void add(const std::string &name, double ours, double serl) {
    Stat &s = by_name[name];
    s.max_abs = std::max(s.max_abs, tol::absError(ours, serl));
    s.max_rel = std::max(s.max_rel, tol::relError(ours, serl));
    ++s.components;
    if (!tol::withinTolerance(ours, serl) || std::isnan(ours) != std::isnan(serl)) ++s.violations;
  }
  void merge(const Stats &o) {
    for (const auto &[k, v] : o.by_name) {
      Stat &s = by_name[k];
      s.max_abs = std::max(s.max_abs, v.max_abs);
      s.max_rel = std::max(s.max_rel, v.max_rel);
      s.components += v.components;
      s.violations += v.violations;
    }
  }
  [[nodiscard]] long violations() const {
    long n = 0;
    for (const auto &kv : by_name) n += kv.second.violations;
    return n;
  }
  void print(const std::string &title) const {
    std::printf("\n%s  (tolerance: abs <= %.0e and rel <= %.0e)\n", title.c_str(), tol::kAbsTol, tol::kRelTol);
    std::printf("  %-34s %14s %14s %12s %10s\n", "quantity", "max_abs", "max_rel", "components", "violations");
    for (const auto &[k, v] : by_name) {
      std::printf("  %-34s %14.3e %14.3e %12ld %10ld\n", k.c_str(), v.max_abs, v.max_rel, v.components, v.violations);
    }
    std::fflush(stdout);
  }
};

struct Scenario {
  const char *name;
  unsigned seed;
  int starts;        // controller lifetimes (each a fresh init + starting)
  long ticks;        // per start
  double p_lost;     // probability that a tick's period is 2..5 ms
  bool small_moves;  // target within 1e-2 of the start (SERL's allclose case)
};

Vector7d randomPosture(std::mt19937_64 &rng) {
  Vector7d q;
  for (int i = 0; i < 7; ++i) q(i) = std::uniform_real_distribution<double>(kLower(i), kUpper(i))(rng);
  return q;
}

Stats runScenario(const Scenario &sc) {
  std::mt19937_64 rng(sc.seed);
  Stats stats;
  long hold_ticks = 0, lost_ticks = 0;
  for (int start = 0; start < sc.starts; ++start) {
    const Vector7d initial = randomPosture(rng);
    Vector7d target = randomPosture(rng);
    if (sc.small_moves) {
      for (int i = 0; i < 7; ++i) target(i) = initial(i) + std::uniform_real_distribution<double>(-1e-2, 1e-2)(rng);
    }
    SerlJointReference reference(target);
    SerlJointPositionLaw law(target);
    reference.starting(initial);
    law.starting(initial);
    for (long tick = 0; tick < sc.ticks; ++tick) {
      uint64_t ms = 1;
      if (tick == 0) {
        ms = 0;  // libfranka's first callback: period 0 (controller_manager reset tick)
      } else if (std::bernoulli_distribution(sc.p_lost)(rng)) {
        ms = std::uniform_int_distribution<uint64_t>(2, 5)(rng);
        ++lost_ticks;
      }
      const Vector7d serl = reference.update(ms);
      const Vector7d ours = law.update(SerlJointPositionLaw::periodNsFromMilliseconds(ms));
      for (int i = 0; i < 7; ++i) stats.add("command q_d", ours(i), serl(i));
      stats.add("elapsed_time_.toSec()", law.elapsedSec(), reference.elapsedSec());
      stats.add("elapsed_time_ (ns)", static_cast<double>(law.elapsedNs()), static_cast<double>(reference.elapsedNs()));
      if (reference.elapsedSec() > 10) ++hold_ticks;
    }
  }
  std::printf("[%s] seed=%u starts=%d ticks/start=%ld lost-packet ticks=%ld hold ticks=%ld\n", sc.name, sc.seed,
              sc.starts, sc.ticks, lost_ticks, hold_ticks);
  return stats;
}

}  // namespace

TEST_CASE("joint reset: periodNsFromMilliseconds == ros::Duration(franka::Duration::toSec())") {
  for (uint64_t ms = 0; ms <= 100000; ++ms) {
    const ros::Duration d(static_cast<double>(ms) / 1000.0);
    REQUIRE(SerlJointPositionLaw::periodNsFromMilliseconds(ms) == d.toNSec());
  }
}

TEST_CASE("joint reset: golden equivalence with the original SERL JointPositionController") {
  const std::array<Scenario, 5> scenarios{{
      {"nominal 1 ms, 12 s", 11, 20, 12000, 0.0, false},
      {"lost packets 2-5 ms, 12 s", 12, 20, 12000, 0.05, false},
      {"heavy loss, 15 s", 13, 10, 15000, 0.3, false},
      {"small moves (allclose at start)", 14, 10, 11000, 0.02, true},
      {"long hold, 60 s", 15, 2, 60000, 0.01, false},
  }};
  Stats total;
  long ticks = 0;
  for (const Scenario &sc : scenarios) {
    const Stats s = runScenario(sc);
    s.print(std::string("scenario: ") + sc.name);
    CHECK_MESSAGE(s.violations() == 0, "scenario " << sc.name);
    total.merge(s);
    ticks += sc.starts * sc.ticks;
  }
  total.print("ALL SCENARIOS (" + std::to_string(ticks) + " ticks)");
  CHECK(total.violations() == 0);
}

TEST_CASE("joint reset: blend endpoints and hold") {
  const Vector7d target = Vector7d::Constant(0.3);
  const Vector7d initial = Vector7d::Constant(-0.1);
  SerlJointPositionLaw law(target);
  law.starting(initial);
  CHECK(law.update(0) == initial);  // t = 0
  for (int i = 0; i < 9999; ++i) static_cast<void>(law.update(1000000));
  CHECK(law.elapsedSec() == doctest::Approx(9.999).epsilon(1e-15));
  static_cast<void>(law.update(1000000));  // t = 10.000 s: still the blend formula, which gives the target
  CHECK(law.elapsedSec() == 10.0);
  CHECK((law.command() - target).cwiseAbs().maxCoeff() <= 1e-15);
  static_cast<void>(law.update(1000000));  // t > 10: exactly the target
  CHECK(law.command() == target);
  CHECK_THROWS_AS(SerlJointPositionLaw(Vector7d::Constant(std::nan(""))), std::invalid_argument);
  CHECK_THROWS_AS(static_cast<void>(law.update(-1)), std::invalid_argument);
}

// ---------------------------------------------------------------------------
// franka_hw's command path: controller + PositionJointSoftLimitsInterface
namespace {

using serl_controller::JointLimits7;
using serl_controller::PositionJointSoftLimits;
using serl_controller::golden::SerlJointHwReference;

struct HwScenario {
  const char *name;
  const char *robot_model;
  unsigned seed;
  int starts;
  long ticks;
  double p_lost;
  double noise;             // measured q = previous sent command + N(0, noise)
  double tick1_offset;      // extra offset of the measured q on the first enforced tick
  double start_margin;      // start this far inside (> 0) or outside (< 0) a random joint limit
  double target_overshoot;  // target this far beyond that limit (> 0) or inside
};

/// Joint `j` starts near its upper or lower limit and the target is at / beyond it.
void nearLimitPosture(std::mt19937_64 &rng, const JointLimits7 &lim, double margin, double overshoot, Vector7d &initial,
                      Vector7d &target) {
  for (int i = 0; i < 7; ++i) {
    const auto &l = lim[static_cast<size_t>(i)];
    const double span = l.max_position - l.min_position;
    initial(i) = l.min_position + std::uniform_real_distribution<double>(0.2, 0.8)(rng) * span;
    target(i) = l.min_position + std::uniform_real_distribution<double>(0.2, 0.8)(rng) * span;
  }
  const int j = std::uniform_int_distribution<int>(0, 6)(rng);
  const auto &l = lim[static_cast<size_t>(j)];
  if (std::bernoulli_distribution(0.5)(rng)) {
    initial(j) = l.max_position - margin;
    target(j) = l.max_position + overshoot;
  } else {
    initial(j) = l.min_position + margin;
    target(j) = l.min_position - overshoot;
  }
}

Stats runHwScenario(const HwScenario &sc, long *limited_ticks_out) {
  std::mt19937_64 rng(sc.seed);
  const JointLimits7 lim = serl_controller::frankaJointLimits(sc.robot_model);
  Stats stats;
  long limited_ticks = 0;
  std::normal_distribution<double> noise(0.0, sc.noise > 0 ? sc.noise : 1.0);
  for (int start = 0; start < sc.starts; ++start) {
    Vector7d initial, target;
    nearLimitPosture(rng, lim, sc.start_margin, sc.target_overshoot, initial, target);
    SerlJointHwReference reference(target, lim);
    SerlJointPositionLaw law(target);
    PositionJointSoftLimits limits(lim);
    reference.starting(initial);
    law.starting(initial);
    Vector7d sent = initial;
    for (long tick = 0; tick < sc.ticks; ++tick) {
      uint64_t ms = 1;
      if (tick == 0) {
        ms = 0;
      } else if (std::bernoulli_distribution(sc.p_lost)(rng)) {
        ms = std::uniform_int_distribution<uint64_t>(2, 5)(rng);
      }
      Vector7d q = tick == 0 ? initial : sent;
      if (sc.noise > 0)
        for (int i = 0; i < 7; ++i) q(i) += noise(rng);
      if (tick == 1) q.array() += sc.tick1_offset;
      const Vector7d serl = reference.tick(q, ms);
      // ours, composed exactly like SerlJointPositionMotion::computeCommand
      const int64_t period_ns = SerlJointPositionLaw::periodNsFromMilliseconds(ms);
      Vector7d ours = law.update(period_ns);
      const Vector7d unlimited = ours;
      if (period_ns == 0) {
        limits.reset();
      } else {
        ours = limits.enforce(ours, q, period_ns);
      }
      if (ours != unlimited) ++limited_ticks;
      for (int i = 0; i < 7; ++i) stats.add("sent command (soft-limited)", ours(i), serl(i));
      for (int i = 0; i < 7; ++i) stats.add("controller command", unlimited(i), reference.controllerCommand()(i));
      sent = ours;
    }
  }
  std::printf("[%s] %s seed=%u starts=%d ticks/start=%ld soft-limit-active ticks=%ld\n", sc.name, sc.robot_model,
              sc.seed, sc.starts, sc.ticks, limited_ticks);
  *limited_ticks_out = limited_ticks;
  return stats;
}

}  // namespace

TEST_CASE("joint reset + franka_hw soft limits: golden equivalence with the original handles") {
  const std::array<HwScenario, 6> scenarios{{
      // name, model, seed, starts, ticks, p_lost, noise, tick1_offset, start_margin, target_overshoot
      {"inside the limits", "panda", 21, 10, 11000, 0.02, 1e-6, 0.0, 0.5, -0.3},
      {"target at the limit", "panda", 22, 10, 11000, 0.02, 0.0, 0.0, 0.2, 0.0},
      {"target beyond the limit", "panda", 23, 10, 11000, 0.05, 1e-6, 0.0, 0.1, 0.05},
      {"start outside the limit", "panda", 24, 10, 3000, 0.02, 0.0, 0.0, -0.005, 0.02},
      {"measured q jumps (velocity cap)", "panda", 25, 10, 3000, 0.1, 2e-3, 0.03, 0.5, -0.3},
      {"fr3 limits", "fr3", 26, 10, 11000, 0.05, 1e-6, 0.02, 0.05, 0.01},
  }};
  Stats total;
  long ticks = 0, limited = 0;
  for (const HwScenario &sc : scenarios) {
    long scenario_limited = 0;
    const Stats s = runHwScenario(sc, &scenario_limited);
    s.print(std::string("scenario: ") + sc.name);
    CHECK_MESSAGE(s.violations() == 0, "scenario " << sc.name);
    // every scenario but the control one must exercise the soft limits
    if (std::string(sc.name) == "inside the limits") {
      CHECK(scenario_limited == 0);
    } else {
      CHECK_MESSAGE(scenario_limited > 0, "scenario " << sc.name << " never hit the soft limits");
    }
    total.merge(s);
    ticks += sc.starts * sc.ticks;
    limited += scenario_limited;
  }
  total.print("ALL SOFT-LIMIT SCENARIOS (" + std::to_string(ticks) + " ticks, " + std::to_string(limited) +
              " with the soft limits active)");
  CHECK(total.violations() == 0);
}

TEST_CASE("joint limit tables == the vendored franka_description joint_limits.yaml and franka-limits macro") {
  for (const char *model : {"panda", "fr3"}) {
    const std::ifstream file(std::string(FRANKA_ROS_ORIGINAL_DIR "/franka_description/robots/") + model +
                             "/joint_limits.yaml");
    REQUIRE(file.good());
    std::stringstream text;
    text << file.rdbuf();
    const std::string yaml = text.str();
    const std::regex row(
        R"(joint(\d):\s*\n\s*limit:\s*\n\s*lower:\s*([-\d.]+)\s*\n\s*upper:\s*([-\d.]+)\s*\n\s*velocity:\s*([-\d.]+))");
    const auto limits = serl_controller::frankaJointLimits(model);
    int found = 0;
    for (auto it = std::sregex_iterator(yaml.begin(), yaml.end(), row); it != std::sregex_iterator(); ++it) {
      const int j = std::stoi((*it)[1]) - 1;
      const auto &l = limits[static_cast<size_t>(j)];
      CHECK(l.min_position == std::stod((*it)[2]));
      CHECK(l.max_position == std::stod((*it)[3]));
      CHECK(l.max_velocity == std::stod((*it)[4]));
      CHECK(l.soft_min_position == l.min_position);  // soft_lower_limit="${limits.lower}"
      CHECK(l.soft_max_position == l.max_position);
      CHECK(l.k_position == 100.0);
      ++found;
    }
    CHECK(found == 7);
  }
  const std::ifstream xacro(FRANKA_ROS_ORIGINAL_DIR "/franka_description/robots/common/utils.xacro");
  std::stringstream xtext;
  xtext << xacro.rdbuf();
  CHECK(xtext.str().find(R"(<safety_controller k_position="100.0")") != std::string::npos);
  CHECK(xtext.str().find(R"(soft_lower_limit="${limits.lower}")") != std::string::npos);
  CHECK_THROWS_AS(static_cast<void>(serl_controller::frankaJointLimits("fr2")), std::invalid_argument);
}
