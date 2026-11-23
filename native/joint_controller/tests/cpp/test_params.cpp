// SPDX-License-Identifier: MIT
// TrajectoryImpedanceParams: defaults, required limits, partial validated updates.
#include <doctest/doctest.h>

#include <Eigen/Dense>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include "joint_controller/params.hpp"

using joint_controller::defaultParams;
using joint_controller::frankaJointPositionLimits;
using joint_controller::ParamUpdates;
using joint_controller::TrajectoryImpedanceParams;
using joint_controller::Vector7d;

namespace {
Vector7d v7(double a, double b, double c, double d, double e, double f, double g) {
  return (Vector7d() << a, b, c, d, e, f, g).finished();
}
}  // namespace

TEST_CASE("defaults: GELLO gains, spec limits; joint limits required") {
  const TrajectoryImpedanceParams p;
  // gello_software @ 9fc8c26 ros2/src/franka_fr3_arm_controllers/config/controllers.yaml
  CHECK(p.stiffness == v7(240, 240, 240, 240, 100, 60, 20));
  CHECK(p.damping == v7(20, 20, 20, 10, 10, 10, 5));
  CHECK(p.velocity_filter_alpha == 0.99);
  CHECK(p.command_timeout_s == 0.5);  // GELLO kJointStateTimeout
  CHECK(p.max_velocity == v7(0.6, 0.6, 0.6, 0.6, 0.75, 0.75, 0.75));
  CHECK(p.max_acceleration == Vector7d::Constant(4.0));
  CHECK(p.min_segment_duration_s == 0.1);
  CHECK(p.limit_margin == 0.05);
  CHECK(p.max_delta_tau == 1.0);
  CHECK(p.compensate_coriolis);
  CHECK_THROWS_AS(p.validate(), std::invalid_argument);  // no joint limits

  const TrajectoryImpedanceParams panda = defaultParams("panda");
  CHECK(panda.lower_limit(3) == -3.0718);
  CHECK(panda.upper_limit(3) == -0.0698);
  CHECK(panda.upper_limit(5) == 3.7525);
  const TrajectoryImpedanceParams fr3 = defaultParams("fr3");
  CHECK(fr3.lower_limit(5) == 0.8521);
  CHECK(fr3.upper_limit(1) == 1.5133);
  CHECK(fr3.stiffness == panda.stiffness);
  CHECK_THROWS_AS((void)defaultParams("fr2"), std::invalid_argument);
  CHECK(frankaJointPositionLimits("panda").lower == panda.lower_limit);
}

TEST_CASE("toMap: 12 fields, round trip through merge") {
  const TrajectoryImpedanceParams p = defaultParams("fr3");
  const ParamUpdates m = p.toMap();
  CHECK(m.size() == TrajectoryImpedanceParams::kNumFields);
  CHECK(std::get<bool>(m.at("compensate_coriolis")));
  CHECK(std::get<double>(m.at("max_delta_tau")) == 1.0);
  CHECK(std::get<Vector7d>(m.at("lower_limit")) == p.lower_limit);
  TrajectoryImpedanceParams q;  // no limits yet
  q.merge(m);
  CHECK(q == p);
}

TEST_CASE("partial update: only the given keys change; effective params") {
  TrajectoryImpedanceParams p = defaultParams("panda");
  const TrajectoryImpedanceParams before = p;
  p.merge({{"max_delta_tau", 0.5}, {"stiffness", Vector7d::Constant(100.0)}, {"compensate_coriolis", false}});
  CHECK(p.max_delta_tau == 0.5);
  CHECK(p.stiffness == Vector7d::Constant(100.0));
  CHECK_FALSE(p.compensate_coriolis);
  CHECK(p.damping == before.damping);
  CHECK(p.lower_limit == before.lower_limit);
  p.merge({});
  CHECK(p.max_delta_tau == 0.5);
}

TEST_CASE("invalid updates are rejected and leave the params unchanged") {
  const TrajectoryImpedanceParams good = defaultParams("panda");
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  Vector7d negative = good.stiffness;
  negative(2) = -1.0;
  Vector7d zero = good.max_velocity;
  zero(6) = 0.0;
  Vector7d crossed = good.lower_limit;
  crossed(1) = good.upper_limit(1);
  Vector7d with_nan = good.damping;
  with_nan(0) = nan;
  const std::vector<ParamUpdates> bad = {
      {{"stiffness", negative}},
      {{"damping", Vector7d(-good.damping)}},
      {{"damping", with_nan}},
      {{"velocity_filter_alpha", -0.01}},
      {{"velocity_filter_alpha", 1.01}},
      {{"velocity_filter_alpha", nan}},
      {{"max_velocity", zero}},
      {{"max_acceleration", Vector7d(-good.max_acceleration)}},
      {{"min_segment_duration_s", 0.0}},
      {{"lower_limit", crossed}},
      {{"upper_limit", good.lower_limit}},
      {{"limit_margin", -0.1}},
      {{"limit_margin", 10.0}},  // no room left between the limits
      {{"command_timeout_s", 0.0}},
      {{"command_timeout_s", inf}},
      {{"max_delta_tau", 0.0}},
      {{"max_delta_tau", -1.0}},
      {{"bogus", 1.0}},                      // unknown key
      {{"stiffness", 240.0}},                // wrong kind
      {{"compensate_coriolis", 1.0}},        // wrong kind
      {{"max_delta_tau", true}},             // wrong kind
      {{"max_delta_tau", 0.5}, {"x", 1.0}},  // atomic: the valid part is not applied either
  };
  for (const auto &u : bad) {
    TrajectoryImpedanceParams p = good;
    CHECK_THROWS_AS(p.merge(u), std::invalid_argument);
    CHECK(p == good);
  }
  // the boundaries are valid
  TrajectoryImpedanceParams p = good;
  p.merge({{"velocity_filter_alpha", 0.0}, {"limit_margin", 0.0}, {"stiffness", Vector7d::Zero()}});
  p.merge({{"velocity_filter_alpha", 1.0}});
  CHECK(p.velocity_filter_alpha == 1.0);
}

TEST_CASE("error messages name the parameter") {
  TrajectoryImpedanceParams p = defaultParams("panda");
  try {
    p.merge({{"velocity_filter_alpha", 2.0}});
    FAIL("no throw");
  } catch (const std::invalid_argument &e) {
    CHECK(std::string(e.what()).find("velocity_filter_alpha") != std::string::npos);
  }
  try {
    TrajectoryImpedanceParams{}.validate();
    FAIL("no throw");
  } catch (const std::invalid_argument &e) {
    CHECK(std::string(e.what()).find("lower_limit") != std::string::npos);
  }
}
