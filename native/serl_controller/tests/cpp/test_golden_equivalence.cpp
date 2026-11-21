// SPDX-License-Identifier: MIT
//
// Golden equivalence: SerlCartesianImpedanceLaw vs the ORIGINAL SERL update()
// (serl_franka_controllers @ 1f140ef, vendored verbatim under golden/serl_original
// and compiled against ROS stand-ins). Both are driven with identical synthetic
// state sequences and identical target / parameter events; every tick compares the
// commanded torque and the filtered internal state with the tolerance defined in
// tolerance.hpp.
#include <doctest/doctest.h>

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <map>
#include <random>
#include <regex>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>

#include "golden/serl_reference.hpp"
#include "serl_controller/law.hpp"
#include "serl_controller/params.hpp"
#include "tolerance.hpp"

using serl_controller::SerlCartesianImpedanceLaw;
using serl_controller::SerlParams;
using serl_controller::golden::ReferenceState;
using serl_controller::golden::SerlReference;
namespace tol = serl_controller::test;

namespace {

using Vector7d = Eigen::Matrix<double, 7, 1>;

// ---------------------------------------------------------------------------
// error statistics per quantity
struct Stat {
  double max_abs{0.0};
  double max_rel{0.0};
  long components{0};
  long violations{0};
  long first_violation_tick{-1};
};

class Stats {
 public:
  template <typename A, typename B>
  void add(const std::string &name, const A &ours, const B &serl, long tick) {
    Stat &s = stats_[name];
    const Eigen::MatrixXd o = ours;
    const Eigen::MatrixXd r = serl;
    REQUIRE(o.size() == r.size());
    for (Eigen::Index i = 0; i < o.size(); ++i) {
      const double a = o.data()[i], b = r.data()[i];
      s.max_abs = std::max(s.max_abs, tol::absError(a, b));
      s.max_rel = std::max(s.max_rel, tol::relError(a, b));
      ++s.components;
      if (!tol::withinTolerance(a, b) || std::isnan(a) != std::isnan(b)) {
        if (s.violations == 0) s.first_violation_tick = tick;
        ++s.violations;
      }
    }
  }
  void addScalar(const std::string &name, double ours, double serl, long tick) {
    add(name, Eigen::Matrix<double, 1, 1>::Constant(ours), Eigen::Matrix<double, 1, 1>::Constant(serl), tick);
  }
  void merge(const Stats &other) {
    for (const auto &[k, v] : other.stats_) {
      Stat &s = stats_[k];
      s.max_abs = std::max(s.max_abs, v.max_abs);
      s.max_rel = std::max(s.max_rel, v.max_rel);
      s.components += v.components;
      if (s.violations == 0 && v.violations > 0) s.first_violation_tick = v.first_violation_tick;
      s.violations += v.violations;
    }
  }
  [[nodiscard]] long totalViolations() const {
    long n = 0;
    for (const auto &kv : stats_) n += kv.second.violations;
    return n;
  }
  void print(const std::string &title) const {
    std::printf("\n%s  (tolerance: abs <= %.0e and rel <= %.0e)\n", title.c_str(), tol::kAbsTol, tol::kRelTol);
    std::printf("  %-34s %14s %14s %12s %10s\n", "quantity", "max_abs", "max_rel", "components", "violations");
    for (const auto &[k, v] : stats_) {
      std::printf("  %-34s %14.3e %14.3e %12ld %10ld\n", k.c_str(), v.max_abs, v.max_rel, v.components, v.violations);
    }
    std::fflush(stdout);
  }

 private:
  std::map<std::string, Stat> stats_;
};

void compareState(Stats &stats, const SerlCartesianImpedanceLaw &law, const ReferenceState &ref, long tick) {
  stats.add("K (cartesian_stiffness_)", law.cartesianStiffness(), ref.K, tick);
  stats.add("K_target", law.cartesianStiffnessTarget(), ref.K_target, tick);
  stats.add("D (cartesian_damping_)", law.cartesianDamping(), ref.D, tick);
  stats.add("D_target", law.cartesianDampingTarget(), ref.D_target, tick);
  stats.add("Ki_", law.ki(), ref.Ki, tick);
  stats.add("Ki_target", law.kiTarget(), ref.Ki_target, tick);
  stats.addScalar("nullspace_stiffness_", law.nullspaceStiffness(), ref.nullspace_stiffness, tick);
  stats.addScalar("nullspace_stiffness_target", law.nullspaceStiffnessTarget(), ref.nullspace_stiffness_target, tick);
  stats.addScalar("joint1_nullspace_stiffness_", law.joint1NullspaceStiffness(), ref.joint1_nullspace_stiffness, tick);
  stats.addScalar("joint1_nullspace_stiffness_target", law.joint1NullspaceStiffnessTarget(),
                  ref.joint1_nullspace_stiffness_target, tick);
  stats.add("position_d_", law.positionD(), ref.position_d, tick);
  stats.add("position_d_target_", law.positionDTarget(), ref.position_d_target, tick);
  stats.add("orientation_d_ (xyzw)", law.orientationD().coeffs(), ref.orientation_d_xyzw, tick);
  stats.add("orientation_d_target_ (xyzw)", law.orientationDTarget().coeffs(), ref.orientation_d_target_xyzw, tick);
  stats.add("error_ (clipped)", law.error(), ref.error, tick);
  stats.add("error_i", law.errorI(), ref.error_i, tick);
  stats.add("q_d_nullspace_", law.qDNullspace(), ref.q_d_nullspace, tick);
  stats.add("translational_clip_min/max", law.translationalClipMin(), ref.translational_clip_min, tick);
  stats.add("translational_clip_min/max", law.translationalClipMax(), ref.translational_clip_max, tick);
  stats.add("rotational_clip_min/max", law.rotationalClipMin(), ref.rotational_clip_min, tick);
  stats.add("rotational_clip_min/max", law.rotationalClipMax(), ref.rotational_clip_max, tick);
}

// ---------------------------------------------------------------------------
// synthetic robot-state sequences
enum class JacobianKind : std::uint8_t { kWellConditioned, kNearSingular, kRankDeficient, kMixed };

struct ScenarioConfig {
  const char *name;
  unsigned seed;
  long ticks;
  JacobianKind jacobian_kind;
  double p_new_target;   // per tick
  double p_reconfigure;  // per tick
  double p_restart;      // per tick: call starting() again (controller restart)
  double rotation_jump;  // rad, scale of random target / measured rotation jumps
  bool random_initial_params;
};

class Generator {
 public:
  explicit Generator(unsigned seed) : rng_(seed) {}

  double uniform(double a, double b) { return std::uniform_real_distribution<double>(a, b)(rng_); }
  double normal(double s) { return std::normal_distribution<double>(0.0, s)(rng_); }
  bool bernoulli(double p) { return std::bernoulli_distribution(p)(rng_); }

  Eigen::Quaterniond randomQuaternion() {
    Eigen::Vector4d v(normal(1), normal(1), normal(1), normal(1));
    v.normalize();
    return {v(3), v(0), v(1), v(2)};
  }
  Eigen::Quaterniond perturb(const Eigen::Quaterniond &q, double angle_scale) {
    Eigen::Vector3d axis(normal(1), normal(1), normal(1));
    axis.normalize();
    return (q * Eigen::Quaterniond(Eigen::AngleAxisd(normal(angle_scale), axis))).normalized();
  }

  Eigen::Matrix<double, 6, 7> jacobian(JacobianKind kind) {
    if (kind == JacobianKind::kMixed) {
      const double u = uniform(0, 1);
      kind = u < 0.6 ? JacobianKind::kWellConditioned
                     : (u < 0.85 ? JacobianKind::kNearSingular : JacobianKind::kRankDeficient);
    }
    Eigen::Matrix<double, 6, 7> J;
    for (int i = 0; i < 6; ++i)
      for (int j = 0; j < 7; ++j) J(i, j) = normal(0.4);
    if (kind == JacobianKind::kWellConditioned) return J;
    const Eigen::JacobiSVD<Eigen::MatrixXd> svd(J, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::VectorXd s = svd.singularValues();
    if (kind == JacobianKind::kNearSingular) {
      // one or two singular values around the damping lambda = 0.2 or tiny
      s(5) = std::pow(10.0, uniform(-12, -1));
      if (bernoulli(0.5)) s(4) = std::pow(10.0, uniform(-6, 0));
    } else {  // rank deficient: exact zeros / duplicated rows
      if (bernoulli(0.5)) {
        s(5) = 0.0;
        if (bernoulli(0.5)) s(4) = 0.0;
      } else {
        J.row(5) = J.row(4);
        return J;
      }
    }
    Eigen::MatrixXd S = Eigen::MatrixXd::Zero(6, 7);
    for (int i = 0; i < 6; ++i) S(i, i) = s(i);
    return svd.matrixU() * S * svd.matrixV().transpose();
  }

 private:
  std::mt19937_64 rng_;
};

SerlParams randomParamUpdate(Generator &g, const SerlParams &current, bool full) {
  std::map<std::string, double> updates;
  for (const auto &field : SerlParams::fields()) {
    if (!full && !g.bernoulli(0.3)) continue;
    // mostly in range; sometimes outside to exercise dynamic_reconfigure clamping
    const double span = field.max - field.min;
    double value = g.uniform(field.min, field.max);
    if (g.bernoulli(0.05)) value = field.max + g.uniform(0.0, span);
    if (g.bernoulli(0.05)) value = field.min - g.uniform(0.0, span);
    if (g.bernoulli(0.1)) value = 0.0;  // zero clips / gains
    updates[field.name] = value;
  }
  SerlParams next = current;
  next.merge(updates);  // clamps like dynamic_reconfigure::Server
  return next;
}

Stats runScenario(const ScenarioConfig &cfg) {
  Generator g(cfg.seed);
  SerlParams params{};
  if (cfg.random_initial_params) params = randomParamUpdate(g, params, true);

  SerlCartesianImpedanceLaw law(params);
  SerlReference ref(params);
  Stats stats;
  compareState(stats, law, ref.state(), -1);

  // initial robot state
  Eigen::Vector3d position(g.uniform(0.3, 0.6), g.uniform(-0.2, 0.2), g.uniform(0.1, 0.5));
  Eigen::Quaterniond orientation = g.randomQuaternion();
  Vector7d q, dq = Vector7d::Zero(), tau_prev = Vector7d::Zero();
  for (int i = 0; i < 7; ++i) q(i) = g.uniform(-2.0, 2.0);

  auto makeInput = [&]() {
    SerlCartesianImpedanceLaw::Input in;
    Eigen::Affine3d T = Eigen::Affine3d::Identity();
    T.linear() = orientation.toRotationMatrix();
    T.translation() = position;
    in.O_T_EE = T.matrix();
    in.q = q;
    in.dq = dq;
    // tau_J_d is the last command the robot accepted: mostly ours, sometimes
    // perturbed (low-pass / rate limiting in libfranka) or far off.
    in.tau_J_d = tau_prev;
    for (int i = 0; i < 7; ++i) in.tau_J_d(i) += g.normal(0.05);
    if (g.bernoulli(0.02)) {
      for (int i = 0; i < 7; ++i) in.tau_J_d(i) = g.normal(10.0);
    }
    in.zero_jacobian = g.jacobian(cfg.jacobian_kind);
    for (int i = 0; i < 7; ++i) in.coriolis(i) = g.normal(0.5);
    return in;
  };

  {
    const auto in = makeInput();
    law.starting(in.O_T_EE, in.q);
    ref.starting(in);
  }
  compareState(stats, law, ref.state(), 0);

  long target_changes = 0, flips = 0, reconfigs = 0, restarts = 0;
  for (long tick = 0; tick < cfg.ticks; ++tick) {
    // asynchronous events (ROS callbacks between two control ticks)
    if (g.bernoulli(cfg.p_new_target)) {
      const Eigen::Vector3d p = position + Eigen::Vector3d(g.normal(0.05), g.normal(0.05), g.normal(0.05));
      Eigen::Quaterniond o = g.perturb(orientation, cfg.rotation_jump);
      Eigen::Vector4d xyzw = o.coeffs();
      if (g.bernoulli(0.5)) {
        xyzw = -xyzw;  // same rotation, other hemisphere
        ++flips;
      }
      if (g.bernoulli(0.05)) xyzw = g.randomQuaternion().coeffs();  // arbitrary far target
      law.setTarget(p, xyzw);
      ref.setTarget(p, xyzw);
      ++target_changes;
    }
    if (g.bernoulli(cfg.p_reconfigure)) {
      params = randomParamUpdate(g, params, g.bernoulli(0.2));
      law.applyParams(params);
      ref.reconfigure(params);
      ++reconfigs;
    }
    const auto in = makeInput();
    if (g.bernoulli(cfg.p_restart)) {
      law.starting(in.O_T_EE, in.q);
      ref.starting(in);
      ++restarts;
    }

    const Vector7d tau_ours = law.update(in);
    const Vector7d tau_serl = ref.update(in);
    stats.add("tau (commanded torque)", tau_ours, tau_serl, tick);
    const auto rs = ref.state();
    compareState(stats, law, rs, tick);
    stats.add("published zero jacobian", Eigen::Map<const Eigen::Matrix<double, 6, 7>>(rs.published_jacobian.data()),
              in.zero_jacobian, tick);
    tau_prev = tau_serl;

    // integrate a plausible motion; sometimes flip the measured orientation far away
    for (int i = 0; i < 7; ++i) {
      dq(i) = 0.98 * dq(i) + g.normal(0.02);
      q(i) += 1e-3 * dq(i);
    }
    position += Eigen::Vector3d(g.normal(2e-4), g.normal(2e-4), g.normal(2e-4));
    orientation = g.perturb(orientation, 2e-3);
    if (g.bernoulli(0.001)) orientation = g.perturb(orientation, cfg.rotation_jump);
  }
  std::printf("[%s] seed=%u ticks=%ld target_changes=%ld (hemisphere-flipped %ld) reconfigures=%ld restarts=%ld\n",
              cfg.name, cfg.seed, cfg.ticks, target_changes, flips, reconfigs, restarts);
  return stats;
}

}  // namespace

TEST_CASE("SerlParams table matches the vendored compliance_param.cfg") {
  std::ifstream file(SERL_ORIGINAL_DIR "/cfg/compliance_param.cfg");
  REQUIRE(file.good());
  const std::regex add_re(
      R"re(gen\.add\("(\w+)",\s*double_t,\s*0,\s*"[^"]*",\s*([-\d.eE]+),\s*([-\d.eE]+),\s*([-\d.eE]+)\))re");
  std::string line;
  std::vector<std::tuple<std::string, double, double, double>> rows;
  while (std::getline(file, line)) {
    std::smatch m;
    if (std::regex_search(line, m, add_re))
      rows.emplace_back(m[1].str(), std::stod(m[2].str()), std::stod(m[3].str()), std::stod(m[4].str()));
  }
  const auto &fields = SerlParams::fields();
  REQUIRE(rows.size() == fields.size());
  const SerlParams defaults{};
  for (size_t i = 0; i < rows.size(); ++i) {
    CAPTURE(std::get<0>(rows[i]));
    CHECK(std::get<0>(rows[i]) == fields[i].name);
    CHECK(std::get<1>(rows[i]) == defaults.*(fields[i].member));
    CHECK(std::get<2>(rows[i]) == fields[i].min);
    CHECK(std::get<3>(rows[i]) == fields[i].max);
  }
}

TEST_CASE("golden equivalence with the original SERL update(), long sequences") {
  const std::vector<ScenarioConfig> scenarios = {
      // name, seed, ticks, jacobian, p_target, p_reconf, p_restart, rot_jump, random_initial_params
      {"defaults/well-conditioned", 1, 20000, JacobianKind::kWellConditioned, 0.002, 0.0, 0.0, 0.3, false},
      {"targets+reconfigure/mixed", 2, 20000, JacobianKind::kMixed, 0.003, 0.001, 0.0, 0.5, false},
      {"near-singular", 3, 20000, JacobianKind::kNearSingular, 0.002, 0.0005, 0.0, 0.3, true},
      {"rank-deficient", 4, 20000, JacobianKind::kRankDeficient, 0.002, 0.0005, 0.0, 0.3, true},
      {"big rotations/hemisphere", 5, 20000, JacobianKind::kMixed, 0.01, 0.002, 0.0, 2.5, true},
      {"restarts", 6, 20000, JacobianKind::kMixed, 0.003, 0.001, 0.0005, 0.8, true},
      {"long drift (filters only)", 7, 100000, JacobianKind::kWellConditioned, 0.0002, 0.0001, 0.0, 0.3, false},
  };
  Stats total;
  for (const auto &scenario : scenarios) {
    const Stats s = runScenario(scenario);
    s.print(std::string("scenario: ") + scenario.name);
    CHECK_MESSAGE(s.totalViolations() == 0, "scenario " << scenario.name);
    total.merge(s);
  }
  total.print("ALL SCENARIOS");
  CHECK(total.totalViolations() == 0);
}

TEST_CASE("first ticks: K and D ramp from zero, nullspace from 20") {
  SerlCartesianImpedanceLaw law;
  const SerlCartesianImpedanceLaw::Input in;
  law.starting(in.O_T_EE, in.q);
  CHECK(law.cartesianStiffness().isZero(0.0));
  CHECK(law.cartesianDamping().isZero(0.0));
  CHECK(law.nullspaceStiffness() == 20.0);
  CHECK(law.joint1NullspaceStiffness() == 20.0);
  static_cast<void>(law.update(in));
  CHECK(law.cartesianStiffness()(0, 0) == doctest::Approx(0.005 * 2000.0).epsilon(1e-15));
  CHECK(law.cartesianStiffness()(3, 3) == doctest::Approx(0.005 * 150.0).epsilon(1e-15));
  CHECK(law.cartesianDamping()(0, 0) == doctest::Approx(0.005 * 89.0).epsilon(1e-15));
  CHECK(law.nullspaceStiffness() == doctest::Approx(0.005 * 0.2 + 0.995 * 20.0).epsilon(1e-15));
  CHECK(law.joint1NullspaceStiffness() == doctest::Approx(0.005 * 100.0 + 0.995 * 20.0).epsilon(1e-15));
  // off-diagonal target entries are zero (setIdentity then diagonal blocks)
  CHECK(law.cartesianStiffnessTarget()(0, 1) == 0.0);
}

TEST_CASE("clip bounds: min = -clip_neg, max = clip (asymmetric)") {
  SerlParams p{};
  p.merge({{"translational_clip_neg_x", 0.02}, {"translational_clip_x", 0.005}, {"rotational_clip_neg_z", 0.07}});
  const SerlCartesianImpedanceLaw law(p);
  CHECK(law.translationalClipMin()(0) == -0.02);
  CHECK(law.translationalClipMax()(0) == 0.005);
  CHECK(law.rotationalClipMin()(2) == -0.07);
  CHECK(law.rotationalClipMax()(2) == 0.05);
}

TEST_CASE("params merge: partial, clamped like dynamic_reconfigure, rejects unknown / non-finite") {
  SerlParams p{};
  p.merge({{"translational_stiffness", 9000.0}, {"rotational_clip_x", -1.0}});
  CHECK(p.translational_stiffness == 6000.0);
  CHECK(p.rotational_clip_x == 0.0);
  CHECK(p.translational_damping == 89.0);
  const SerlParams before = p;
  CHECK_THROWS_AS(p.merge({{"no_such_key", 1.0}}), std::invalid_argument);
  CHECK_THROWS_AS(p.merge({{"translational_Ki", std::nan("")}}), std::invalid_argument);
  CHECK(p == before);
}

TEST_CASE("setTarget resets error_i and flips into the previous target hemisphere") {
  SerlCartesianImpedanceLaw law;
  SerlCartesianImpedanceLaw::Input in;
  in.O_T_EE(0, 3) = 0.5;
  law.starting(in.O_T_EE, in.q);
  in.O_T_EE(0, 3) = 0.6;  // 0.1 m error -> clipped to 0.01
  for (int i = 0; i < 20; ++i) static_cast<void>(law.update(in));
  CHECK(law.errorI()(0) == doctest::Approx(0.1));  // accumulated and clipped although Ki = 0
  law.setTarget(Eigen::Vector3d(0.5, 0, 0), Eigen::Vector4d(0, 0, 0, -1));
  CHECK(law.errorI().isZero(0.0));
  CHECK(law.orientationDTarget().w() == 1.0);  // flipped
}
