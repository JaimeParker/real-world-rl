// SPDX-License-Identifier: MIT
// TrajectoryImpedanceMotion without a robot: the motion must behave exactly like the bare law
// with the same events, and its mailboxes must be usable from other threads while the "control
// thread" steps.
#include <doctest/doctest.h>

#include <Eigen/Dense>
#include <atomic>
#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <random>
#include <stdexcept>
#include <thread>

#include "franky/robot_state.hpp"
#include "joint_controller/law.hpp"
#include "joint_controller/motion.hpp"
#include "joint_controller/params.hpp"

using joint_controller::defaultParams;
using joint_controller::TrajectoryImpedanceLaw;
using joint_controller::TrajectoryImpedanceMotion;
using joint_controller::TrajectoryImpedanceParams;
using joint_controller::Vector7d;

namespace {

Vector7d home() { return (Vector7d() << 0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785).finished(); }

struct Tick {
  Vector7d q;
  Vector7d dq;
  Vector7d coriolis;
};

Tick makeTick(std::mt19937_64 &rng, double t) {
  std::normal_distribution<double> n(0.0, 1.0);
  Tick k;
  for (int i = 0; i < 7; ++i) {
    k.q(i) = home()(i) + 0.02 * std::sin(t + i) + 1e-4 * n(rng);
    k.dq(i) = 0.02 * std::cos(t + i) + 1e-3 * n(rng);
    k.coriolis(i) = 0.1 * n(rng);
  }
  return k;
}

TrajectoryImpedanceLaw::Input toInput(const Tick &k, double dt) {
  TrajectoryImpedanceLaw::Input in;
  in.q = k.q;
  in.dq = k.dq;
  in.coriolis = k.coriolis;
  in.dt = dt;
  return in;
}

void start(TrajectoryImpedanceMotion &motion, const Vector7d &q) {
  franky::RobotState state{};
  state.q = q;
  motion.init(nullptr, state, std::nullopt);
}

}  // namespace

TEST_CASE("motion == law, with commands and params applied at the next tick") {
  std::mt19937_64 rng(42);
  const TrajectoryImpedanceParams p = defaultParams("panda");
  auto motion = std::make_shared<TrajectoryImpedanceMotion>(p);
  TrajectoryImpedanceLaw law(p);

  // A target before the start is applied on the first tick.
  REQUIRE(motion->setTarget(home() + Vector7d::Constant(0.1)));
  const Tick first = makeTick(rng, 0.0);
  start(*motion, first.q);
  law.starting(first.q);
  REQUIRE(law.setTarget(home() + Vector7d::Constant(0.1)));
  CHECK(motion->telemetry().ticks == 0);

  TrajectoryImpedanceParams q = p;
  for (int t = 0; t < 6000; ++t) {
    if (t == 1000) {
      REQUIRE(motion->setTarget(home() - Vector7d::Constant(0.1)));
      REQUIRE(law.setTarget(home() - Vector7d::Constant(0.1)));
    }
    if (t == 2000) {
      Vector7d bad = home();
      bad(1) = std::numeric_limits<double>::infinity();
      CHECK_FALSE(motion->setTarget(bad));
      motion->updateParams({{"stiffness", Vector7d::Constant(150.0)}, {"max_delta_tau", 0.5}});
      q.merge({{"stiffness", Vector7d::Constant(150.0)}, {"max_delta_tau", 0.5}});
      law.applyParams(q);
    }
    if (t == 3000) {
      motion->requestStop();
      law.requestStop();
    }
    if (t == 4000) {  // two commands in one tick: the newest wins
      REQUIRE(motion->setTarget(home()));
      motion->requestStop();
      law.requestStop();
    }
    const Tick k = (t == 0) ? first : makeTick(rng, 1e-3 * t);
    const double dt = (t == 0) ? 0.0 : (t % 97 == 0 ? 0.003 : 0.001);
    const Vector7d tau_motion = motion->computeTorques(k.q, k.dq, k.coriolis, dt, 1e-3 * t);
    const Vector7d tau_law = law.update(toInput(k, dt));
    REQUIRE((tau_motion.array() == tau_law.array()).all());
  }
  const auto tel = motion->telemetry();
  CHECK(tel.ticks == 6000);
  CHECK(tel.applied_command_seq == 5);
  CHECK(tel.applied_params_seq == 1);
  CHECK(tel.q_ref == law.reference().q);
  CHECK(tel.tau_cmd == law.tauCommand());
  CHECK(tel.stopped);
  CHECK(motion->params() == q);
}

TEST_CASE("input validation leaves the motion unchanged") {
  CHECK_THROWS_AS(TrajectoryImpedanceMotion(TrajectoryImpedanceParams{}), std::invalid_argument);  // no limits
  const TrajectoryImpedanceParams p = defaultParams("fr3");
  TrajectoryImpedanceMotion motion(p);
  CHECK_THROWS_AS(motion.updateParams({{"bogus", 1.0}}), std::invalid_argument);
  CHECK_THROWS_AS(motion.updateParams({{"velocity_filter_alpha", -1.0}}), std::invalid_argument);
  CHECK(motion.params() == p);
  Vector7d nan = home();
  nan(0) = std::numeric_limits<double>::quiet_NaN();
  CHECK_FALSE(motion.setTarget(nan));
}

TEST_CASE("concurrent user commands while the control thread steps") {
  std::mt19937_64 rng(7);
  auto motion = std::make_shared<TrajectoryImpedanceMotion>(defaultParams("panda"));
  start(*motion, home());
  std::atomic<bool> stop{false};
  std::thread writer([&] {
    int i = 0;
    while (!stop.load()) {
      CHECK(motion->setTarget(home() + Vector7d::Constant(1e-3 * (i % 100))));
      if (i % 10 == 0) motion->updateParams({{"max_delta_tau", 0.5 + 0.01 * (i % 7)}});
      if (i % 97 == 0) motion->requestStop();
      (void)motion->telemetry();
      ++i;
    }
  });
  for (int t = 0; t < 20000; ++t) {
    const Tick k = makeTick(rng, 1e-3 * t);
    const Vector7d tau = motion->computeTorques(k.q, k.dq, k.coriolis, 0.001, 1e-3 * t);
    REQUIRE(tau.allFinite());
  }
  stop = true;
  writer.join();
  const auto tel = motion->telemetry();
  CHECK(tel.applied_command_seq > 0);
  CHECK(tel.applied_params_seq > 0);
  CHECK(tel.ticks == 20000);
}
