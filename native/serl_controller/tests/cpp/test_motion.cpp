// SPDX-License-Identifier: MIT
// SerlCartesianImpedanceMotion without a robot: the motion must behave exactly
// like the bare law with the same events, and its mailboxes must be usable from
// other threads while the "control thread" steps.
#include <doctest/doctest.h>

#include <Eigen/Dense>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <random>
#include <stdexcept>
#include <thread>

#include "franky/robot_state.hpp"
#include "serl_controller/joint_reset.hpp"
#include "serl_controller/joint_reset_motion.hpp"
#include "serl_controller/law.hpp"
#include "serl_controller/motion.hpp"
#include "serl_controller/params.hpp"

using serl_controller::SerlCartesianImpedanceLaw;
using serl_controller::SerlCartesianImpedanceMotion;
using serl_controller::SerlParams;
using Vector7d = Eigen::Matrix<double, 7, 1>;

namespace {

struct Tick {
  franky::RobotState state;
  Eigen::Matrix<double, 6, 7> jacobian;
  Vector7d coriolis;
};

Tick makeTick(std::mt19937_64 &rng, double t) {
  std::normal_distribution<double> n(0.0, 0.3);
  Tick k;
  Eigen::Affine3d T = Eigen::Affine3d::Identity();
  T.linear() = Eigen::AngleAxisd(0.3 * std::sin(t), Eigen::Vector3d(0.2, 0.5, 0.8).normalized()).toRotationMatrix();
  T.translation() = Eigen::Vector3d(0.4 + 0.01 * std::sin(t), 0.02 * std::cos(t), 0.3);
  k.state.O_T_EE = T;
  for (int i = 0; i < 7; ++i) {
    k.state.q(i) = 0.1 * i + 0.05 * std::sin(t + i);
    k.state.dq(i) = n(rng);
    k.state.tau_J_d(i) = n(rng);
    k.coriolis(i) = 0.1 * n(rng);
  }
  for (int i = 0; i < 6; ++i)
    for (int j = 0; j < 7; ++j) k.jacobian(i, j) = n(rng);
  return k;
}

SerlCartesianImpedanceLaw::Input toInput(const Tick &k) {
  SerlCartesianImpedanceLaw::Input in;
  in.O_T_EE = k.state.O_T_EE.matrix();
  in.q = k.state.q;
  in.dq = k.state.dq;
  in.tau_J_d = k.state.tau_J_d;
  in.zero_jacobian = k.jacobian;
  in.coriolis = k.coriolis;
  return in;
}

}  // namespace

TEST_CASE("motion == law, with targets and params applied at the next tick") {
  std::mt19937_64 rng(42);
  auto motion = std::make_shared<SerlCartesianImpedanceMotion>();
  SerlCartesianImpedanceLaw law;

  // Updates before start: params apply at start, a target is overwritten by starting().
  motion->updateParams({{"translational_stiffness", 1500.0}});
  motion->setTarget(Eigen::Vector3d(9, 9, 9), Eigen::Vector4d(0, 0, 0, 1));
  SerlParams p{};
  p.merge({{"translational_stiffness", 1500.0}});
  law.applyParams(p);

  const Tick first = makeTick(rng, 0.0);
  motion->init(nullptr, first.state, std::nullopt);
  law.starting(first.state.O_T_EE.matrix(), first.state.q);
  CHECK(motion->snapshot().tick == 0);

  for (int t = 0; t < 5000; ++t) {
    if (t == 1000) {
      const Eigen::Vector3d pos(0.45, 0.01, 0.31);
      const Eigen::Vector4d quat = Eigen::Quaterniond(Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitZ())).coeffs();
      motion->setTarget(pos, -quat);
      law.setTarget(pos, -quat);
    }
    if (t == 2500) {
      motion->updateParams({{"rotational_clip_neg_x", 0.02}, {"translational_Ki", 3.0}});
      p.merge({{"rotational_clip_neg_x", 0.02}, {"translational_Ki", 3.0}});
      law.applyParams(p);
    }
    const Tick k = (t == 0) ? first : makeTick(rng, 1e-3 * t);
    const Vector7d tau_motion = motion->computeTorques(k.state, k.jacobian, k.coriolis, 1e-3 * t);
    const Vector7d tau_law = law.update(toInput(k));
    REQUIRE((tau_motion.array() == tau_law.array()).all());
  }
  const auto snap = motion->snapshot();
  CHECK(snap.tick == 5000);
  CHECK(snap.applied_target_seq == 2);
  CHECK(snap.applied_params_seq == 2);
  CHECK(snap.stiffness_diag(0) == law.cartesianStiffness()(0, 0));
  CHECK(snap.ki_diag(0) == law.ki()(0, 0));
  CHECK(motion->params().translational_Ki == 3.0);
}

TEST_CASE("input validation leaves the motion unchanged") {
  SerlCartesianImpedanceMotion motion;
  CHECK_THROWS_AS(motion.setTarget(Eigen::Vector3d(0, 0, std::nan("")), Eigen::Vector4d(0, 0, 0, 1)),
                  std::invalid_argument);
  CHECK_THROWS_AS(motion.setTarget(Eigen::Vector3d::Zero(), Eigen::Vector4d(0, 0, 0, 2)), std::invalid_argument);
  CHECK_THROWS_AS(motion.updateParams({{"bogus", 1.0}}), std::invalid_argument);
  CHECK(motion.params() == SerlParams{});
}

TEST_CASE("concurrent user updates while the control thread steps") {
  std::mt19937_64 rng(7);
  auto motion = std::make_shared<SerlCartesianImpedanceMotion>();
  const Tick first = makeTick(rng, 0.0);
  motion->init(nullptr, first.state, std::nullopt);
  std::atomic<bool> stop{false};
  std::thread writer([&] {
    int i = 0;
    while (!stop.load()) {
      motion->setTarget(Eigen::Vector3d(0.4, 0.0, 0.3 + 1e-4 * (i % 100)), Eigen::Vector4d(0, 0, 0, 1));
      if (i % 10 == 0) motion->updateParams({{"translational_stiffness", 1000.0 + (i % 7)}});
      (void)motion->snapshot();
      ++i;
    }
  });
  for (int t = 0; t < 20000; ++t) {
    const Tick k = makeTick(rng, 1e-3 * t);
    const Vector7d tau = motion->computeTorques(k.state, k.jacobian, k.coriolis, 1e-3 * t);
    REQUIRE(tau.allFinite());
  }
  stop = true;
  writer.join();
  CHECK(motion->snapshot().applied_target_seq > 0);
}

TEST_CASE("joint reset motion == joint reset law; finish() ends with a hold") {
  using serl_controller::SerlJointPositionLaw;
  using serl_controller::SerlJointPositionMotion;
  const Vector7d target = (Vector7d() << 0.0, 0.0, 0.0, -1.9, 0.0, 2.0, 0.0).finished();  // HIL-SERL default
  const Vector7d start = (Vector7d() << 0.1, -0.3, 0.05, -2.2, 0.02, 1.8, 0.7).finished();
  auto motion = std::make_shared<SerlJointPositionMotion>(target);
  SerlJointPositionLaw law(target);
  franky::RobotState state{};
  state.q = start;
  motion->init(nullptr, state, std::nullopt);
  law.starting(start);
  CHECK(motion->snapshot().tick == 0);
  Vector7d command;
  for (int tick = 0; tick < 11000; ++tick) {
    const uint64_t ms = tick == 0 ? 0 : (tick % 50 == 0 ? 3 : 1);
    const auto result = motion->computeCommand(start, ms);
    command = result.command;
    CHECK_FALSE(result.finished);
    REQUIRE(command == law.update(SerlJointPositionLaw::periodNsFromMilliseconds(ms)));
  }
  CHECK(command == target);
  const auto snap = motion->snapshot();
  CHECK(snap.tick == 11000);
  CHECK(snap.elapsed_s == law.elapsedSec());
  CHECK(snap.initial == start);
  motion->finish();
  const auto final_tick = motion->computeCommand(start, 1);
  CHECK(final_tick.finished);
  CHECK(final_tick.command == command);
  CHECK(motion->snapshot().finished);
}

TEST_CASE("joint reset motion applies franka_hw's soft limits like franka_control_node") {
  using serl_controller::PositionJointSoftLimits;
  using serl_controller::SerlJointPositionLaw;
  using serl_controller::SerlJointPositionMotion;
  const auto limits = serl_controller::frankaPandaJointLimits();
  Vector7d start = (Vector7d() << 0.1, -0.3, 0.05, -2.2, 0.02, 1.8, 0.7).finished();
  start(3) = limits[3].max_position - 0.01;
  Vector7d target = start;
  target(3) = limits[3].max_position + 0.05;  // beyond the limit: clamped to it, approached softly
  auto motion = std::make_shared<SerlJointPositionMotion>(target, limits);
  SerlJointPositionLaw law(target);
  PositionJointSoftLimits soft(limits);
  franky::RobotState state{};
  state.q = start;
  motion->init(nullptr, state, std::nullopt);
  law.starting(start);
  Vector7d sent = start;
  for (int tick = 0; tick < 11000; ++tick) {
    const uint64_t ms = tick == 0 ? 0 : 1;
    const Vector7d command = motion->computeCommand(sent, ms).command;
    Vector7d expected = law.update(SerlJointPositionLaw::periodNsFromMilliseconds(ms));
    if (ms == 0) {
      soft.reset();
    } else {
      expected = soft.enforce(expected, sent, SerlJointPositionLaw::periodNsFromMilliseconds(ms));
    }
    REQUIRE(command == expected);
    sent = command;
  }
  CHECK(sent(3) <= limits[3].max_position);
  CHECK(sent(3) > limits[3].max_position - 1e-6);
  CHECK(motion->snapshot().soft_limited_ticks > 0);
  CHECK(motion->snapshot().controller_command(3) == target(3));
}
