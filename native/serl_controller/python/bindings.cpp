// SPDX-License-Identifier: MIT
// Copyright (c) real-world-rl contributors. Binds the ports of serl_franka_controllers
// (Copyright (c) 2024 RAIL Berkeley, MIT; see LICENSE in this package).
//
// pybind11 module serl_controller._serl_controller.
//
// With SERL_CONTROLLER_WITH_FRANKY, SerlCartesianImpedanceMotion is registered as
// a subclass of franky's own Python binding of franky::Motion<franka::Torques>
// (franky.BaseTorqueMotion). That requires franky's extension to be imported
// first (done below) and both extensions to share pybind11's internals, i.e. to
// be built with the same pybind11 version and compiler/stdlib ABI: build both
// wheels together (see docker/).
#include <pybind11/eigen.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <string>

#include "serl_controller/joint_reset.hpp"
#include "serl_controller/law.hpp"
#include "serl_controller/params.hpp"
#include "serl_controller/version.hpp"

#ifdef SERL_CONTROLLER_WITH_FRANKY
#include "franka/control_types.h"
#include "franky/motion/motion.hpp"
#include "franky/robot_state.hpp"
#include "serl_controller/joint_reset_motion.hpp"
#include "serl_controller/motion.hpp"
#endif

namespace py = pybind11;
using namespace pybind11::literals;
using serl_controller::SerlCartesianImpedanceLaw;
using serl_controller::SerlJointPositionLaw;
using serl_controller::SerlParams;

namespace {

using ParamDict = std::map<std::string, double>;
using Vector7d = Eigen::Matrix<double, 7, 1>;

SerlParams paramsFromDict(const ParamDict &d) {
  SerlParams p{};
  p.merge(d);
  return p;
}

py::dict lawState(const SerlCartesianImpedanceLaw &law) {
  py::dict d;
  d["cartesian_stiffness"] = law.cartesianStiffness();
  d["cartesian_stiffness_target"] = law.cartesianStiffnessTarget();
  d["cartesian_damping"] = law.cartesianDamping();
  d["cartesian_damping_target"] = law.cartesianDampingTarget();
  d["Ki"] = law.ki();
  d["Ki_target"] = law.kiTarget();
  d["nullspace_stiffness"] = law.nullspaceStiffness();
  d["nullspace_stiffness_target"] = law.nullspaceStiffnessTarget();
  d["joint1_nullspace_stiffness"] = law.joint1NullspaceStiffness();
  d["joint1_nullspace_stiffness_target"] = law.joint1NullspaceStiffnessTarget();
  d["position_d"] = law.positionD();
  d["orientation_d_xyzw"] = Eigen::Vector4d(law.orientationD().coeffs());
  d["position_d_target"] = law.positionDTarget();
  d["orientation_d_target_xyzw"] = Eigen::Vector4d(law.orientationDTarget().coeffs());
  d["q_d_nullspace"] = law.qDNullspace();
  d["error"] = law.error();
  d["error_i"] = law.errorI();
  d["translational_clip_min"] = law.translationalClipMin();
  d["translational_clip_max"] = law.translationalClipMax();
  d["rotational_clip_min"] = law.rotationalClipMin();
  d["rotational_clip_max"] = law.rotationalClipMax();
  return d;
}

Vector7d lawStep(SerlCartesianImpedanceLaw &law, const Eigen::Matrix4d &O_T_EE, const Vector7d &q, const Vector7d &dq,
                 const Vector7d &tau_J_d, const Eigen::Matrix<double, 6, 7> &zero_jacobian, const Vector7d &coriolis) {
  SerlCartesianImpedanceLaw::Input in;
  in.O_T_EE = O_T_EE;
  in.q = q;
  in.dq = dq;
  in.tau_J_d = tau_J_d;
  in.zero_jacobian = zero_jacobian;
  in.coriolis = coriolis;
  return law.update(in);
}

#ifdef SERL_CONTROLLER_WITH_FRANKY
py::dict snapshotDict(const serl_controller::SerlSnapshot &s) {
  py::dict d;
  d["tick"] = s.tick;
  d["rel_time"] = s.rel_time;
  d["zero_jacobian"] = s.zero_jacobian;
  d["O_T_EE"] = s.O_T_EE;
  d["q"] = s.q;
  d["dq"] = s.dq;
  d["tau_J_d"] = s.tau_J_d;
  d["coriolis"] = s.coriolis;
  d["tau_command"] = s.tau_command;
  d["error"] = s.error;
  d["error_i"] = s.error_i;
  d["position_d"] = s.position_d;
  d["orientation_d_xyzw"] = s.orientation_d_xyzw;
  d["position_d_target"] = s.position_d_target;
  d["orientation_d_target_xyzw"] = s.orientation_d_target_xyzw;
  d["stiffness_diag"] = s.stiffness_diag;
  d["damping_diag"] = s.damping_diag;
  d["ki_diag"] = s.ki_diag;
  d["nullspace_stiffness"] = s.nullspace_stiffness;
  d["joint1_nullspace_stiffness"] = s.joint1_nullspace_stiffness;
  d["applied_target_seq"] = s.applied_target_seq;
  d["applied_params_seq"] = s.applied_params_seq;
  return d;
}

py::dict jointResetSnapshotDict(const serl_controller::SerlJointResetSnapshot &s) {
  py::dict d;
  d["tick"] = s.tick;
  d["elapsed_s"] = s.elapsed_s;
  d["q"] = s.q;
  d["command"] = s.command;
  d["initial"] = s.initial;
  d["controller_command"] = s.controller_command;
  d["soft_limited_ticks"] = s.soft_limited_ticks;
  d["finished"] = s.finished;
  return d;
}
#endif

}  // namespace

PYBIND11_MODULE(_serl_controller, m) {
  m.doc() = "SERL Cartesian impedance controller (serl_franka_controllers @ " SERL_CONTROLLER_SERL_COMMIT
            ") for franky's torque loop";
  m.attr("__version__") = SERL_CONTROLLER_VERSION;
  m.attr("SERL_COMMIT") = SERL_CONTROLLER_SERL_COMMIT;
  m.attr("FILTER_PARAMS") = SerlCartesianImpedanceLaw::kFilterParams;
  m.attr("DELTA_TAU_MAX") = SerlCartesianImpedanceLaw::kDeltaTauMax;
  m.attr("PINV_LAMBDA") = SerlCartesianImpedanceLaw::kPinvLambda;
  m.attr("JOINT_BLEND_DURATION_S") = SerlJointPositionLaw::kBlendDurationS;
  {
    py::dict info;
    info["serl_controller_version"] = SERL_CONTROLLER_VERSION;
    info["serl_commit"] = SERL_CONTROLLER_SERL_COMMIT;
#ifdef SERL_CONTROLLER_WITH_FRANKY
    info["with_franky"] = true;
#else
    info["with_franky"] = false;
#endif
    info["franky_dist_version"] = SERL_CONTROLLER_FRANKY_DIST_VERSION;
    info["franky_commit"] = SERL_CONTROLLER_FRANKY_COMMIT;
    info["libfranky_sha256"] = SERL_CONTROLLER_LIBFRANKY_SHA256;
    info["libfranka_file"] = SERL_CONTROLLER_LIBFRANKA_FILE;
    info["libfranka_version"] = SERL_CONTROLLER_LIBFRANKA_VERSION;
    info["build_id"] = SERL_CONTROLLER_BUILD_ID;
    m.attr("BUILD_INFO") = info;  // compiled-in copy of build_info.json
  }

  // ---- SerlParams ---------------------------------------------------------
  auto params_cls = py::class_<SerlParams>(
      m, "SerlParams", "Compliance parameters (names, defaults and ranges of SERL's compliance_param.cfg).");
  params_cls.def(py::init<>())
      .def(py::init(&paramsFromDict), "params"_a,
           "Defaults updated with a (partial) dict, clamped into the .cfg ranges.")
      .def("to_dict", &SerlParams::toMap)
      .def(
          "merged",
          [](const SerlParams &self, const ParamDict &updates) {
            SerlParams p = self;
            p.merge(updates);
            return p;
          },
          "updates"_a, "Return a copy with a partial update applied (dynamic_reconfigure semantics).")
      .def(py::self == py::self)  // NOLINT(misc-redundant-expression): pybind11 operator idiom
      .def("__repr__", [](const SerlParams &self) {
        std::string out = "SerlParams(";
        bool first = true;
        for (const auto &[k, v] : self.toMap()) {
          out += (first ? "" : ", ") + k + "=" + std::to_string(v);
          first = false;
        }
        return out + ")";
      });
  py::list field_info;
  for (const auto &field : SerlParams::fields()) {
    const auto member = field.member;
    params_cls.def_property(
        field.name, [member](const SerlParams &self) { return self.*member; },
        [member](SerlParams &self, double value) { self.*member = value; });
    field_info.append(py::make_tuple(field.name, SerlParams{}.*member, field.min, field.max));
  }
  m.attr("PARAM_FIELDS") = py::tuple(field_info);

  // ---- the pure control law -----------------------------------------------
  py::class_<SerlCartesianImpedanceLaw>(
      m, "SerlCartesianImpedanceLaw",
      "The franky-independent SERL control law (init + starting + update + callbacks). For tests and offline use.")
      .def(py::init<const SerlParams &>(), "params"_a = SerlParams{})
      .def(py::init([](const ParamDict &d) { return SerlCartesianImpedanceLaw(paramsFromDict(d)); }), "params"_a)
      .def("apply_params", &SerlCartesianImpedanceLaw::applyParams, "params"_a)
      .def("set_target", &SerlCartesianImpedanceLaw::setTarget, "position"_a, "quat_xyzw"_a)
      .def("starting", &SerlCartesianImpedanceLaw::starting, "O_T_EE"_a, "q"_a)
      .def("step", &lawStep, "O_T_EE"_a, "q"_a, "dq"_a, "tau_J_d"_a, "zero_jacobian"_a, "coriolis"_a,
           "One update(): torques of this tick, then the filters advance.")
      .def_property_readonly("params", &SerlCartesianImpedanceLaw::params)
      .def("state", &lawState);
  // ---- SERL joint reset (JointPositionController) law ---------------------
  py::class_<SerlJointPositionLaw>(
      m, "SerlJointPositionLaw",
      "The franky-independent SERL joint reset law: 10 s linear blend to the target, then hold.")
      .def(py::init<const Vector7d &>(), "target"_a)
      .def("starting", &SerlJointPositionLaw::starting, "q"_a)
      .def("update", &SerlJointPositionLaw::update, "period_ns"_a,
           "Advance by period_ns and return this tick's commanded joint positions.")
      .def_static("period_ns_from_milliseconds", &SerlJointPositionLaw::periodNsFromMilliseconds, "milliseconds"_a)
      .def_property_readonly("elapsed_s", &SerlJointPositionLaw::elapsedSec)
      .def_property_readonly("elapsed_ns", &SerlJointPositionLaw::elapsedNs)
      .def_property_readonly("target", &SerlJointPositionLaw::target)
      .def_property_readonly("initial", &SerlJointPositionLaw::initial)
      .def_property_readonly("command", &SerlJointPositionLaw::command);

  // ---- franka_hw's PositionJointSoftLimitsInterface ---------------------------
  {
    py::dict tables;
    for (const char *model : {"panda", "fr3"}) {
      py::list joints;
      for (const auto &l : serl_controller::frankaJointLimits(model)) {
        py::dict j;
        j["min_position"] = l.min_position;
        j["max_position"] = l.max_position;
        j["max_velocity"] = l.max_velocity;
        j["soft_min_position"] = l.soft_min_position;
        j["soft_max_position"] = l.soft_max_position;
        j["k_position"] = l.k_position;
        joints.append(j);
      }
      tables[model] = joints;
    }
    m.attr("FRANKA_JOINT_LIMITS") = tables;  // franka_description joint_limits.yaml + safety_controller
  }
  py::class_<serl_controller::PositionJointSoftLimits>(
      m, "PositionJointSoftLimits",
      "franka_hw's PositionJointSoftLimitsInterface (joint_limits_interface) for the 7 joints.")
      .def(py::init([](const std::string &robot_model) {
             return serl_controller::PositionJointSoftLimits(serl_controller::frankaJointLimits(robot_model));
           }),
           "robot_model"_a = "panda")
      .def("reset", &serl_controller::PositionJointSoftLimits::reset)
      .def("enforce", &serl_controller::PositionJointSoftLimits::enforce, "command"_a, "q_measured"_a, "period_ns"_a);

  m.def("law_step", &lawStep, "law"_a, "O_T_EE"_a, "q"_a, "dq"_a, "tau_J_d"_a, "zero_jacobian"_a, "coriolis"_a,
        "Run one SERL update() of `law` on the given state; returns the commanded torques.");

#ifdef SERL_CONTROLLER_WITH_FRANKY
  m.attr("WITH_FRANKY") = true;
  // franky.BaseTorqueMotion (franky::Motion<franka::Torques>) must be registered first.
  py::module_::import("franky");

  using serl_controller::SerlCartesianImpedanceMotion;
  py::class_<SerlCartesianImpedanceMotion, franky::Motion<franka::Torques>,
             std::shared_ptr<SerlCartesianImpedanceMotion>>(
      m, "SerlCartesianImpedanceMotion",
      "SERL Cartesian impedance controller as a franky torque motion. Pass it to "
      "franky.Robot.move(motion, asynchronous=True, limit_rate=True, cutoff_frequency=100.0) "
      "to reproduce franka_control's defaults.")
      .def(py::init<const SerlParams &>(), "params"_a = SerlParams{})
      .def(py::init(
               [](const ParamDict &d) { return std::make_shared<SerlCartesianImpedanceMotion>(paramsFromDict(d)); }),
           "params"_a)
      .def("set_target", &SerlCartesianImpedanceMotion::setTarget, "position"_a, "quat_xyzw"_a,
           "New equilibrium pose in the base frame; quaternion is (x, y, z, w). Returns its sequence number.")
      .def(
          "update_params",
          [](SerlCartesianImpedanceMotion &self, const ParamDict &updates) {
            return self.updateParams(updates).toMap();
          },
          "updates"_a, "Partial update (dynamic_reconfigure semantics). Returns the full effective dict.")
      .def_property_readonly("params", [](const SerlCartesianImpedanceMotion &self) { return self.params().toMap(); })
      .def(
          "snapshot", [](SerlCartesianImpedanceMotion &self) { return snapshotDict(self.snapshot()); },
          "Latest telemetry from the control thread (zero Jacobian, state, command, filtered gains, ...).")
      .def(
          "_offline_step",
          [](SerlCartesianImpedanceMotion &self, const Eigen::Matrix4d &O_T_EE, const Vector7d &q, const Vector7d &dq,
             const Vector7d &tau_J_d, const Eigen::Matrix<double, 6, 7> &zero_jacobian, const Vector7d &coriolis) {
            // Test hook: drive the motion without a robot. Starts it on first use, after which it
            // can no longer be passed to Robot.move().
            franky::RobotState state{};
            state.O_T_EE = franky::Affine(O_T_EE);
            state.q = q;
            state.dq = dq;
            state.tau_J_d = tau_J_d;
            if (!self.has_started()) self.init(nullptr, state, std::nullopt);
            return self.computeTorques(state, zero_jacobian, coriolis, 0.0);
          },
          "O_T_EE"_a, "q"_a, "dq"_a, "tau_J_d"_a, "zero_jacobian"_a, "coriolis"_a);

  // franky.BaseJointPositionMotion (franky::Motion<franka::JointPositions>), registered by franky.
  using serl_controller::SerlJointPositionMotion;
  py::class_<SerlJointPositionMotion, franky::Motion<franka::JointPositions>, std::shared_ptr<SerlJointPositionMotion>>(
      m, "SerlJointPositionMotion",
      "SERL's joint reset (JointPositionController) as a franky joint position motion: 10 s linear "
      "blend from the start posture to the target, then hold until finish(). Pass it to "
      "franky.Robot.move(motion, asynchronous=True, limit_rate=True, cutoff_frequency=100.0) "
      "with the robot's default JointImpedance controller mode to reproduce franka_control.")
      .def(py::init([](const Vector7d &target, const std::string &robot_model) {
             return std::make_shared<SerlJointPositionMotion>(target, serl_controller::frankaJointLimits(robot_model));
           }),
           "target"_a, "robot_model"_a = "panda",
           "robot_model selects franka_description's joint limits for the soft-limit stage: 'panda' or 'fr3'.")
      .def_property_readonly("target", &SerlJointPositionMotion::target)
      .def("finish", &SerlJointPositionMotion::finish,
           "End the motion at the next tick, holding the previous command (franka::MotionFinished).")
      .def_property_readonly("finish_requested", &SerlJointPositionMotion::finishRequested)
      .def(
          "snapshot", [](SerlJointPositionMotion &self) { return jointResetSnapshotDict(self.snapshot()); },
          "Latest telemetry from the control thread (tick, elapsed_s, q, command, initial, finished).")
      .def(
          "_offline_step",
          [](SerlJointPositionMotion &self, const Vector7d &q, uint64_t time_step_ms) {
            // Test hook: drive the motion without a robot. Starts it on first use, after which it
            // can no longer be passed to Robot.move().
            franky::RobotState state{};
            state.q = q;
            if (!self.has_started()) self.init(nullptr, state, std::nullopt);
            const serl_controller::SerlJointResetCommand tick = self.computeCommand(q, time_step_ms);
            return py::make_tuple(tick.command, tick.finished);
          },
          "q"_a, "time_step_ms"_a);
#else
  m.attr("WITH_FRANKY") = false;
#endif
}
