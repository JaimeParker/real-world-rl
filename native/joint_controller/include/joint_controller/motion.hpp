// SPDX-License-Identifier: MIT AND Apache-2.0
// Copyright (c) real-world-rl contributors. Runs TrajectoryImpedanceLaw, whose impedance law and
// velocity filter are ported from GELLO's JointImpedanceController (wuphilipp/gello_software @
// 9fc8c26, Copyright (c) 2025 Franka Robotics GmbH, Apache-2.0); see law.hpp and LICENSE.
//
// TrajectoryImpedanceMotion: runs TrajectoryImpedanceLaw inside franky's 1 kHz torque loop, as a
// franky::Motion<franka::Torques>. franky itself is not modified; this is a separate library that
// links against libfranky.so (like serl_controller's SerlCartesianImpedanceMotion).
//
// One motion object == one controller lifetime: initImpl() runs starting() (reference seeded at
// the measured q, filter zeroed, the torque-rate limit continuing from franky's previous command
// if a torque motion is replaced). franky motions cannot be restarted. The motion never ends by
// itself: stop it with franky (robot.stop(), a reaction) or let a reflex end it.
//
// Threading, ownership and memory ordering (C++ Core Guidelines CP.*)
// -------------------------------------------------------------------
// Two kinds of threads touch a motion: any number of user threads (Python / gRPC calling
// setTarget, requestStop, updateParams, params, telemetry) and exactly one control (RT) thread
// (franky's libfranka callback, calling initImpl / nextCommandImpl / computeTorques).
//
//  * user -> RT: command_buffer_ (targets and stop requests: one mailbox, so the newest command
//    wins) and params_buffer_ are franky::WaitFreeTripleBuffer (an SPSC triple buffer: the writer
//    publishes a whole message by an acq_rel exchange of a packed index + "new" flag, the reader
//    takes the newest one by another acq_rel exchange, so a message is never torn and its payload
//    happens-before the reader's use). User threads write with set() while holding user_mutex_;
//    user_mutex_ also makes "increment the sequence number, merge the parameters, publish" one
//    atomic step, so sequence numbers are published in increasing order. The RT thread is the
//    single reader and uses only hasNewData() (an acquire load) and getUnsafe() -- it never takes
//    user_mutex_ or a buffer mutex, so a user thread can never block it.
//  * RT -> user: the RT thread is the single writer of telemetry_buffer_ (setUnsafe()); user
//    threads read with get(), which serialises readers on the buffer's read mutex.
//  * RT-owned state: law_, applied_command_seq_, applied_params_seq_ (no other thread).
//  * User-owned state, guarded by user_mutex_: requested_params_, command_seq_, params_seq_.
//  * Construction publishes the initial messages with setUnsafe() before the object is shared,
//    i.e. before any concurrency.
//
// Commands and parameters take effect at the start of the next control tick. If several commands
// arrive within one tick only the newest is applied. A target sent before the motion starts is
// applied on its first tick.
//
// Real-time path (per tick): no lock, no blocking call, no heap allocation (fixed-size Eigen and
// fixed-size messages only), no exception (messages are validated on the user thread; the law's
// update() is noexcept). The work per tick is bounded (see planner.hpp).
#pragma once

#include <Eigen/Dense>
#include <cstdint>
#include <mutex>
#include <optional>

#include "franka/control_types.h"
#include "franka/duration.h"
#include "franky/motion/motion.hpp"
#include "franky/robot_state.hpp"
#include "franky/wait_free_triple_buffer.hpp"
#include "joint_controller/law.hpp"
#include "joint_controller/params.hpp"

namespace joint_controller {

class TrajectoryImpedanceMotion : public franky::Motion<franka::Torques> {
 public:
  /// Throws std::invalid_argument if `params` is invalid (e.g. joint limits missing).
  explicit TrajectoryImpedanceMotion(const TrajectoryImpedanceParams &params);

  /// New joint target. Returns false (nothing is sent) if a value is not finite.
  bool setTarget(const Vector7d &q);

  /// Bounded stop to rest, then hold; the motion keeps running and the next target resumes.
  void requestStop();

  /// Partial update (validated). Throws std::invalid_argument (then nothing changes). Returns
  /// the full effective parameter set, applied at the next tick.
  TrajectoryImpedanceParams updateParams(const ParamUpdates &updates);

  /// The full parameter set most recently requested (may not be applied yet).
  [[nodiscard]] TrajectoryImpedanceParams params() const;

  /// Latest RT telemetry. Never blocks the RT thread.
  [[nodiscard]] JointTelemetry telemetry();

  /// The tick body, separated from franky's model access so it can be tested without a robot:
  /// applies pending user messages, runs the law, publishes the telemetry. Must only be called
  /// from the control thread.
  [[nodiscard]] Vector7d computeTorques(const Vector7d &q, const Vector7d &dq, const Vector7d &coriolis, double dt,
                                        double rel_time) noexcept;

 protected:
  void initImpl(const franky::RobotState &robot_state, const std::optional<franka::Torques> &previous_command) override;

  franka::Torques nextCommandImpl(const franky::RobotState &robot_state, franka::Duration time_step,
                                  franka::Duration rel_time, franka::Duration abs_time,
                                  const std::optional<franka::Torques> &previous_command) override;

 private:
  struct CommandMsg {
    uint64_t seq{0};
    bool stop{false};  // true: requestStop(), else setTarget(q)
    Vector7d q{Vector7d::Zero()};
  };
  struct ParamsMsg {
    uint64_t seq{0};
    TrajectoryImpedanceParams params{};
  };

  void applyPendingParams() noexcept;
  void applyPendingCommand() noexcept;
  void publishCommand(bool stop, const Vector7d &q);

  // user side
  mutable std::mutex user_mutex_;  // user threads only (see "Threading" above); never the RT thread
  TrajectoryImpedanceParams requested_params_;
  uint64_t command_seq_{0};
  uint64_t params_seq_{0};

  // user -> RT
  franky::WaitFreeTripleBuffer<CommandMsg> command_buffer_;
  franky::WaitFreeTripleBuffer<ParamsMsg> params_buffer_;
  // RT -> user
  franky::WaitFreeTripleBuffer<JointTelemetry> telemetry_buffer_;

  // RT side only
  TrajectoryImpedanceLaw law_;
  uint64_t applied_command_seq_{0};
  uint64_t applied_params_seq_{0};
};

}  // namespace joint_controller
