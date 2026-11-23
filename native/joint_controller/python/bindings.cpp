// SPDX-License-Identifier: MIT AND Apache-2.0
// Copyright (c) real-world-rl contributors. Binds TrajectoryImpedanceLaw, whose impedance law,
// velocity filter and default gains are ported from GELLO (wuphilipp/gello_software @ 9fc8c26,
// Copyright (c) 2025 Franka Robotics GmbH, Apache-2.0; see LICENSE in this package).
//
// pybind11 module joint_controller._joint_controller.
//
// With JOINT_CONTROLLER_WITH_FRANKY, TrajectoryImpedanceMotion is registered as a subclass of
// franky's own Python binding of franky::Motion<franka::Torques> (franky.BaseTorqueMotion). That
// requires franky's extension to be imported first (done below) and both extensions to share
// pybind11's internals, i.e. to be built with the same pybind11 version and compiler/stdlib ABI:
// build both wheels together (see docker/ and scripts/).
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <variant>

#include "joint_controller/law.hpp"
#include "joint_controller/params.hpp"
#include "joint_controller/planner.hpp"
#include "joint_controller/version.hpp"

#ifdef JOINT_CONTROLLER_WITH_FRANKY
#include "franka/control_types.h"
#include "franky/motion/motion.hpp"
#include "franky/robot_state.hpp"
#include "joint_controller/motion.hpp"
#endif

namespace py = pybind11;
using namespace pybind11::literals;
using joint_controller::JointTelemetry;
using joint_controller::ParamUpdates;
using joint_controller::TrajectoryImpedanceLaw;
using joint_controller::TrajectoryImpedanceParams;
using joint_controller::Vector7d;

namespace {

using Kind = TrajectoryImpedanceParams::Kind;

bool isBoolLike(const py::handle &v) {
  if (py::isinstance<py::bool_>(v)) return true;
  if (!py::hasattr(v, "dtype") || py::isinstance<py::array>(v)) return false;
  return py::str(v.attr("dtype").attr("kind")).cast<std::string>() == "b";  // numpy.bool_
}

/// {name: value} from Python (numbers, bools, sequences / arrays of 7) to typed updates.
/// Throws std::invalid_argument (ValueError) for unknown keys and values of the wrong kind.
ParamUpdates updatesFromDict(const py::dict &d) {
  ParamUpdates out;
  for (const auto &item : d) {
    if (!py::isinstance<py::str>(item.first)) throw std::invalid_argument("parameter names must be strings");
    const auto key = item.first.cast<std::string>();
    const auto *field = TrajectoryImpedanceParams::findField(key);
    if (field == nullptr) throw std::invalid_argument("unknown joint_controller parameter '" + key + "'");
    const py::handle value = item.second;
    switch (field->kind) {
      case Kind::kScalar: {
        if (isBoolLike(value)) throw std::invalid_argument("parameter '" + key + "' must be a number, not a bool");
        try {
          out[key] = value.cast<double>();
        } catch (const py::cast_error &) {
          throw std::invalid_argument("parameter '" + key + "' must be a number");
        }
        break;
      }
      case Kind::kFlag:
        if (!isBoolLike(value)) throw std::invalid_argument("parameter '" + key + "' must be a bool");
        out[key] = value.cast<bool>();
        break;
      case Kind::kVector7: {
        if (py::isinstance<py::str>(value) || isBoolLike(value)) {
          throw std::invalid_argument("parameter '" + key + "' must be a vector of 7 numbers");
        }
        py::array_t<double, py::array::c_style | py::array::forcecast> arr;
        try {
          arr = py::array_t<double, py::array::c_style | py::array::forcecast>::ensure(value);
        } catch (const py::error_already_set &) {
          arr = {};
        }
        if (!arr || arr.ndim() != 1 || arr.size() != 7) {
          throw std::invalid_argument("parameter '" + key + "' must be a vector of 7 numbers");
        }
        Vector7d v;
        for (py::ssize_t i = 0; i < 7; ++i) v(i) = arr.at(i);
        out[key] = v;
        break;
      }
    }
  }
  return out;
}

py::dict paramsToDict(const TrajectoryImpedanceParams &p) {
  py::dict d;
  for (const auto &[key, value] : p.toMap()) {
    if (const auto *x = std::get_if<double>(&value)) {
      d[py::str(key)] = *x;
    } else if (const auto *b = std::get_if<bool>(&value)) {
      d[py::str(key)] = *b;
    } else {
      d[py::str(key)] = Vector7d(std::get<Vector7d>(value));  // a new numpy array
    }
  }
  return d;
}

/// Defaults (joint limits unset) updated with the dict, validated.
TrajectoryImpedanceParams paramsFromDict(const py::dict &d) {
  TrajectoryImpedanceParams p{};
  p.merge(updatesFromDict(d));
  return p;
}

py::dict telemetryDict(const JointTelemetry &t) {
  py::dict d;
  d["ticks"] = t.ticks;
  d["rel_time"] = t.rel_time;
  d["q"] = t.q;
  d["dq"] = t.dq;
  d["dq_filtered"] = t.dq_filtered;
  d["coriolis"] = t.coriolis;
  d["q_ref"] = t.q_ref;
  d["dq_ref"] = t.dq_ref;
  d["ddq_ref"] = t.ddq_ref;
  d["target"] = t.target;
  d["goal"] = t.goal;
  d["tau_cmd"] = t.tau_cmd;
  d["segment_t"] = t.segment_t;
  d["segment_T"] = t.segment_T;
  d["time_since_target"] = t.time_since_target;
  d["timed_out"] = t.timed_out;
  d["stopped"] = t.stopped;
  d["target_pending"] = t.target_pending;
  d["plan_retries"] = t.plan_retries;
  d["applied_command_seq"] = t.applied_command_seq;
  d["applied_params_seq"] = t.applied_params_seq;
  return d;
}

Vector7d lawStep(TrajectoryImpedanceLaw &law, const Vector7d &q, const Vector7d &dq, const Vector7d &coriolis,
                 double dt) {
  TrajectoryImpedanceLaw::Input in;
  in.q = q;
  in.dq = dq;
  in.coriolis = coriolis;
  in.dt = dt;
  return law.update(in);
}

}  // namespace

PYBIND11_MODULE(_joint_controller, m) {
  m.doc() =
      "Trajectory joint impedance controller (quintic reference + GELLO joint impedance "
      "@ " JOINT_CONTROLLER_GELLO_COMMIT ") for franky's torque loop";
  m.attr("__version__") = JOINT_CONTROLLER_VERSION;
  m.attr("GELLO_COMMIT") = JOINT_CONTROLLER_GELLO_COMMIT;
  {
    py::dict info;
    info["joint_controller_version"] = JOINT_CONTROLLER_VERSION;
    info["gello_commit"] = JOINT_CONTROLLER_GELLO_COMMIT;
#ifdef JOINT_CONTROLLER_WITH_FRANKY
    info["with_franky"] = true;
#else
    info["with_franky"] = false;
#endif
    info["franky_dist_version"] = JOINT_CONTROLLER_FRANKY_DIST_VERSION;
    info["franky_commit"] = JOINT_CONTROLLER_FRANKY_COMMIT;
    info["libfranky_sha256"] = JOINT_CONTROLLER_LIBFRANKY_SHA256;
    info["libfranka_file"] = JOINT_CONTROLLER_LIBFRANKA_FILE;
    info["libfranka_version"] = JOINT_CONTROLLER_LIBFRANKA_VERSION;
    info["franky_layout"] = JOINT_CONTROLLER_FRANKY_LAYOUT;
    info["libfranka_path"] = JOINT_CONTROLLER_LIBFRANKA_PATH;
    info["libfranka_sha256"] = JOINT_CONTROLLER_LIBFRANKA_SHA256;
    info["build_id"] = JOINT_CONTROLLER_BUILD_ID;
    m.attr("BUILD_INFO") = info;  // compiled-in copy of build_info.json
  }
  {
    py::list names;
    for (const auto &field : TrajectoryImpedanceParams::fields()) names.append(field.name);
    m.attr("PARAM_FIELDS") = py::tuple(names);
  }

  m.def(
      "default_params",
      [](const std::string &robot_model) { return paramsToDict(joint_controller::defaultParams(robot_model)); },
      "robot_model"_a,
      "The default parameters, with lower_limit / upper_limit of robot_model ('panda' | 'fr3', "
      "franka_description's joint position limits).");
  m.def(
      "validate_params", [](const py::dict &params) { return paramsToDict(paramsFromDict(params)); }, "params"_a,
      "The effective parameters of a full parameter dict (defaults for missing keys; lower_limit and "
      "upper_limit are required). Raises ValueError.");

  // ---- the pure control law ---------------------------------------------------
  py::class_<TrajectoryImpedanceLaw>(
      m, "TrajectoryImpedanceLaw",
      "The franky-independent law (reference generator + joint impedance). For tests and offline use.")
      .def(py::init([](const py::dict &params) { return TrajectoryImpedanceLaw(paramsFromDict(params)); }), "params"_a)
      .def(
          "apply_params",
          [](TrajectoryImpedanceLaw &self, const py::dict &updates) {
            TrajectoryImpedanceParams p = self.params();
            p.merge(updatesFromDict(updates));
            self.applyValidatedParams(p);
            return paramsToDict(p);
          },
          "updates"_a, "Partial update, effective immediately. Returns the effective parameters.")
      .def("params", [](const TrajectoryImpedanceLaw &self) { return paramsToDict(self.params()); })
      .def("starting", &TrajectoryImpedanceLaw::starting, "q"_a, "tau_prev"_a = Vector7d::Zero())
      .def("set_target", &TrajectoryImpedanceLaw::setTarget, "q"_a)
      .def("request_stop", &TrajectoryImpedanceLaw::requestStop)
      .def("step", &lawStep, "q"_a, "dq"_a, "coriolis"_a, "dt"_a = 0.001,
           "One tick: returns the commanded torque (without gravity).")
      .def("telemetry", [](const TrajectoryImpedanceLaw &self) { return telemetryDict(self.telemetry()); });

#ifdef JOINT_CONTROLLER_WITH_FRANKY
  m.attr("WITH_FRANKY") = true;
  // franky.BaseTorqueMotion (franky::Motion<franka::Torques>) must be registered first.
  py::module_::import("franky");

  using joint_controller::TrajectoryImpedanceMotion;
  py::class_<TrajectoryImpedanceMotion, franky::Motion<franka::Torques>, std::shared_ptr<TrajectoryImpedanceMotion>>(
      m, "TrajectoryImpedanceMotion",
      "Joint impedance tracking of a smooth online reference (one quintic per joint, replanned "
      "within the velocity / acceleration limits on every target) as a franky torque motion. "
      "Start it with franky.Robot.move(motion, asynchronous=True).")
      .def(py::init([](const py::dict &params) {
             return std::make_shared<TrajectoryImpedanceMotion>(paramsFromDict(params));
           }),
           "params"_a,
           "Defaults updated with params; lower_limit and upper_limit are required (see default_params). "
           "Raises ValueError.")
      .def("set_target", &TrajectoryImpedanceMotion::setTarget, "q"_a,
           "New joint target (7,). Returns False (and sends nothing) if a value is not finite.")
      .def(
          "update_params",
          [](TrajectoryImpedanceMotion &self, const py::dict &updates) {
            return paramsToDict(self.updateParams(updatesFromDict(updates)));
          },
          "updates"_a, "Partial update, applied at the next tick. Returns the full effective dict. Raises ValueError.")
      .def(
          "params", [](const TrajectoryImpedanceMotion &self) { return paramsToDict(self.params()); },
          "The full parameter set most recently requested.")
      .def(
          "telemetry", [](TrajectoryImpedanceMotion &self) { return telemetryDict(self.telemetry()); },
          "Latest telemetry from the control thread.")
      .def("request_stop", &TrajectoryImpedanceMotion::requestStop,
           "Bounded stop to rest and hold; the motion keeps running, the next target resumes.")
      .def(
          "_offline_step",
          [](TrajectoryImpedanceMotion &self, const Vector7d &q, const Vector7d &dq, const Vector7d &coriolis,
             double dt) {
            // Test hook: drive the motion without a robot. Starts it on first use, after which it
            // can no longer be passed to Robot.move().
            if (!self.has_started()) {
              franky::RobotState state{};
              state.q = q;
              state.dq = dq;
              self.init(nullptr, state, std::nullopt);
            }
            return self.computeTorques(q, dq, coriolis, dt, 0.0);
          },
          "q"_a, "dq"_a, "coriolis"_a, "dt"_a = 0.001);
#else
  m.attr("WITH_FRANKY") = false;
#endif
}
