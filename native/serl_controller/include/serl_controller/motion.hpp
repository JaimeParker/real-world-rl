// SPDX-License-Identifier: MIT
// Copyright (c) 2024 RAIL Berkeley (serl_franka_controllers); port: real-world-rl contributors.
// Runs the port of serl_franka_controllers @ 1f140ef src/cartesian_impedance_controller.cpp
// (see law.hpp for provenance). MIT License (see LICENSE in this package).
//
// SerlCartesianImpedanceMotion: runs SerlCartesianImpedanceLaw inside franky's
// 1 kHz torque loop, as a franky::Motion<franka::Torques>. franky itself is not
// modified; this is a separate library that links against libfranky.so, the
// way serl_franka_controllers is a separate ros_control plugin next to franka_ros.
//
// One motion object == one SERL controller lifetime (SERL's franka_server
// relaunches impedance.launch for every start): initImpl() runs starting(), so K
// and D ramp from 0 at every new motion. franky motions cannot be restarted.
//
// Threading, ownership and memory ordering (C++ Core Guidelines CP.*)
// -------------------------------------------------------------------
// Two kinds of threads touch a motion: any number of user threads (Python / gRPC calling
// setTarget, updateParams, params, snapshot) and exactly one control (RT) thread (franky's
// libfranka callback, calling initImpl / nextCommandImpl / computeTorques).
//
//  * user -> RT: target_buffer_ and params_buffer_ are franky::WaitFreeTripleBuffer (an SPSC
//    triple buffer: the writer publishes a whole message by an acq_rel exchange of a packed
//    index + "new" flag, the reader takes the newest one by another acq_rel exchange, so a
//    message is never torn and its payload happens-before the reader's use).
//    User threads write with set() while holding user_mutex_; user_mutex_ also makes
//    "increment the sequence number, merge the parameters, publish" one atomic step, so
//    sequence numbers are published in increasing order. The RT thread is the single reader
//    and uses only hasNewData() (an acquire load) and getUnsafe() -- it never takes
//    user_mutex_ or a buffer mutex, so a user thread can never block it.
//  * RT -> user: the RT thread is the single writer of snapshot_buffer_ (setUnsafe());
//    user threads read with get(), which serialises readers on the buffer's read mutex.
//  * RT-owned state: law_, applied_target_seq_, applied_params_seq_, tick_ (no other thread).
//  * User-owned state, guarded by user_mutex_: requested_params_, target_seq_, params_seq_.
//  * Construction publishes the initial messages with setUnsafe() before the object is
//    shared, i.e. before any concurrency.
//
// Updates take effect at the start of the next control tick, i.e. "between two update()
// calls", like ROS callbacks in SERL. If several targets arrive within one tick only the
// latest is applied (the hemisphere flip is then relative to the last applied target).
//
// Real-time path (per tick): no lock, no blocking call, no throw on valid input (messages are
// validated and clamped on the user thread; the RT thread applies them with the non-throwing
// applyClampedParams / setTarget). The one deliberate exception is the per-tick heap
// allocation inside SerlCartesianImpedanceLaw::update(), kept from SERL for bit-identical
// results (see law.hpp, "Allocation"); its only failure mode is std::bad_alloc, which franky
// turns into a control exception that stops the motion. The snapshot is fixed-size.
#pragma once

#include <Eigen/Dense>
#include <cstdint>
#include <map>
#include <mutex>
#include <optional>
#include <string>

#include "franka/control_types.h"
#include "franka/duration.h"
#include "franky/motion/motion.hpp"
#include "franky/robot_state.hpp"
#include "franky/wait_free_triple_buffer.hpp"
#include "serl_controller/law.hpp"
#include "serl_controller/params.hpp"

namespace serl_controller {

/// Telemetry written by the RT thread once per tick.
struct SerlSnapshot {
  uint64_t tick{0};  // ticks since start (0 = not started yet)
  double rel_time{0.0};
  Eigen::Matrix<double, 6, 7> zero_jacobian{Eigen::Matrix<double, 6, 7>::Zero()};
  Eigen::Matrix4d O_T_EE{Eigen::Matrix4d::Identity()};
  Eigen::Matrix<double, 7, 1> q{Eigen::Matrix<double, 7, 1>::Zero()};
  Eigen::Matrix<double, 7, 1> dq{Eigen::Matrix<double, 7, 1>::Zero()};
  Eigen::Matrix<double, 7, 1> tau_J_d{Eigen::Matrix<double, 7, 1>::Zero()};
  Eigen::Matrix<double, 7, 1> coriolis{Eigen::Matrix<double, 7, 1>::Zero()};
  Eigen::Matrix<double, 7, 1> tau_command{Eigen::Matrix<double, 7, 1>::Zero()};
  Eigen::Matrix<double, 6, 1> error{Eigen::Matrix<double, 6, 1>::Zero()};
  Eigen::Matrix<double, 6, 1> error_i{Eigen::Matrix<double, 6, 1>::Zero()};
  Eigen::Vector3d position_d{Eigen::Vector3d::Zero()};
  Eigen::Vector4d orientation_d_xyzw{0, 0, 0, 1};
  Eigen::Vector3d position_d_target{Eigen::Vector3d::Zero()};
  Eigen::Vector4d orientation_d_target_xyzw{0, 0, 0, 1};
  Eigen::Matrix<double, 6, 1> stiffness_diag{Eigen::Matrix<double, 6, 1>::Zero()};
  Eigen::Matrix<double, 6, 1> damping_diag{Eigen::Matrix<double, 6, 1>::Zero()};
  Eigen::Matrix<double, 6, 1> ki_diag{Eigen::Matrix<double, 6, 1>::Zero()};
  double nullspace_stiffness{0.0};
  double joint1_nullspace_stiffness{0.0};
  uint64_t applied_target_seq{0};
  uint64_t applied_params_seq{0};
};

class SerlCartesianImpedanceMotion : public franky::Motion<franka::Torques> {
 public:
  explicit SerlCartesianImpedanceMotion(const SerlParams &params = SerlParams{});

  /// New equilibrium pose (base frame), quaternion (x, y, z, w). Throws
  /// std::invalid_argument for non-finite values or a quaternion whose norm is
  /// not within 1e-3 of 1. Returns the sequence number of this target.
  uint64_t setTarget(const Eigen::Vector3d &position, const Eigen::Vector4d &quat_xyzw);

  /// Partial update with dynamic_reconfigure semantics (merge, clamp to the .cfg
  /// ranges). Throws for unknown keys / non-finite values (then nothing changes).
  /// Returns the full effective parameter set.
  SerlParams updateParams(const std::map<std::string, double> &updates);

  /// The full parameter set most recently requested (may not be applied yet).
  [[nodiscard]] SerlParams params() const;

  /// Latest RT telemetry. Never blocks the RT thread.
  [[nodiscard]] SerlSnapshot snapshot();

  /// The tick body, separated from franky's model access so it can be tested
  /// without a robot: applies pending user updates, runs the law, publishes the
  /// snapshot. Must only be called from the control thread.
  [[nodiscard]] Eigen::Matrix<double, 7, 1> computeTorques(const franky::RobotState &robot_state,
                                                           const Eigen::Matrix<double, 6, 7> &zero_jacobian,
                                                           const Eigen::Matrix<double, 7, 1> &coriolis,
                                                           double rel_time);

 protected:
  void initImpl(const franky::RobotState &robot_state, const std::optional<franka::Torques> &previous_command) override;

  franka::Torques nextCommandImpl(const franky::RobotState &robot_state, franka::Duration time_step,
                                  franka::Duration rel_time, franka::Duration abs_time,
                                  const std::optional<franka::Torques> &previous_command) override;

 private:
  struct TargetMsg {
    uint64_t seq{0};
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};
    Eigen::Vector4d quat_xyzw{0, 0, 0, 1};
  };
  struct ParamsMsg {
    uint64_t seq{0};
    SerlParams params{};
  };

  void applyPendingParams() noexcept;

  // user side
  mutable std::mutex user_mutex_;  // user threads only (see "Threading" above); never the RT thread
  SerlParams requested_params_;
  uint64_t target_seq_{0};
  uint64_t params_seq_{0};

  // user -> RT
  franky::WaitFreeTripleBuffer<TargetMsg> target_buffer_;
  franky::WaitFreeTripleBuffer<ParamsMsg> params_buffer_;
  // RT -> user
  franky::WaitFreeTripleBuffer<SerlSnapshot> snapshot_buffer_;

  // RT side only
  SerlCartesianImpedanceLaw law_;
  uint64_t applied_target_seq_{0};
  uint64_t applied_params_seq_{0};
  uint64_t tick_{0};
};

}  // namespace serl_controller
