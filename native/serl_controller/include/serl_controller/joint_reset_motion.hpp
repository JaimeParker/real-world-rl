// SPDX-License-Identifier: MIT
// Copyright (c) 2024 RAIL Berkeley (serl_franka_controllers); port: real-world-rl contributors.
// Runs the port of serl_franka_controllers @ 1f140ef src/joint_position_controller.cpp
// (see joint_reset.hpp for provenance and licences). MIT License (see LICENSE in this package).
//
// SerlJointPositionMotion: SERL's joint reset (SerlJointPositionLaw) as a
// franky::Motion<franka::JointPositions>, i.e. a joint *position* motion run by
// the robot's internal joint impedance controller, exactly like franka_hw's
// position interface under SERL's joint.launch:
//
//   franka_hw (franka_control_node.yaml)   franky / this package
//   ------------------------------------   ---------------------------------------------
//   robot.control(cb, joint_impedance,     Robot(controller_mode=JointImpedance (default))
//                 rate_limiting: true,     robot.move(motion, asynchronous=True,
//                 cutoff_frequency: 100)              limit_rate=True, cutoff_frequency=100.0)
//
// franka_hw also passes the command through its PositionJointSoftLimitsInterface (URDF
// soft limits, k_position 100, max velocity): reset on the period-0 tick, enforced on every
// other tick right after the controller's update(). The motion does the same with
// PositionJointSoftLimits, so the command sent is the one franka_hw would send.
//
// One motion object == one SERL joint-controller lifetime: initImpl() runs
// starting() (blend from the measured q), each tick runs update(time_step). The
// motion never ends by itself (SERL holds the target until franka_server kills
// joint.launch); finish() makes the next tick return franka::MotionFinished with
// the previous command (a hold), so the control loop ends without an error.
//
// Threading and ownership (C++ Core Guidelines CP.*):
//  * The control (RT) thread owns everything except finish_requested_ and the read side of
//    snapshot_buffer_: law_, soft_limits_, last_command_, tick_, soft_limited_ticks_ and
//    finished_ are only touched from initImpl() / nextCommandImpl() / computeCommand().
//  * finish() stores the atomic flag; the RT thread loads it once per tick. The flag carries
//    no payload (the hold repeats the RT-owned last_command_), so relaxed ordering is enough.
//  * Telemetry: the RT thread is the single writer of snapshot_buffer_ and uses the wait-free
//    setUnsafe(); user threads read with get(), which serialises readers on the buffer's
//    read mutex. The RT thread never takes a lock.
//  * Real-time path: no heap allocation (fixed-size Eigen only) and no exception on valid
//    input (periods are whole milliseconds >= 0, so update() / enforce() never throw).
#pragma once

#include <Eigen/Dense>
#include <atomic>
#include <cstdint>
#include <optional>

#include "franka/control_types.h"
#include "franka/duration.h"
#include "franky/motion/motion.hpp"
#include "franky/robot_state.hpp"
#include "franky/wait_free_triple_buffer.hpp"
#include "serl_controller/joint_reset.hpp"

namespace serl_controller {

struct SerlJointResetSnapshot {
  uint64_t tick{0};  // ticks since start (0 = not started yet)
  double elapsed_s{0.0};
  Eigen::Matrix<double, 7, 1> q{Eigen::Matrix<double, 7, 1>::Zero()};        // measured
  Eigen::Matrix<double, 7, 1> command{Eigen::Matrix<double, 7, 1>::Zero()};  // commanded q
  Eigen::Matrix<double, 7, 1> initial{Eigen::Matrix<double, 7, 1>::Zero()};
  Eigen::Matrix<double, 7, 1> controller_command{Eigen::Matrix<double, 7, 1>::Zero()};  // before soft limits
  uint64_t soft_limited_ticks{0};  // ticks where the soft limits changed the command
  bool finished{false};
};

/// One tick of SerlJointPositionMotion: the command and whether it is the final one.
struct SerlJointResetCommand {
  Eigen::Matrix<double, 7, 1> command{Eigen::Matrix<double, 7, 1>::Zero()};
  bool finished{false};
};

class SerlJointPositionMotion : public franky::Motion<franka::JointPositions> {
 public:
  using Vector7d = Eigen::Matrix<double, 7, 1>;

  explicit SerlJointPositionMotion(const Vector7d &target, const JointLimits7 &limits = frankaPandaJointLimits());

  [[nodiscard]] const Vector7d &target() const { return target_; }

  /// Request a clean end: the next tick repeats the previous command as
  /// franka::MotionFinished. Has no effect before the motion started.
  void finish() noexcept { finish_requested_.store(true, std::memory_order_relaxed); }
  [[nodiscard]] bool finishRequested() const noexcept { return finish_requested_.load(std::memory_order_relaxed); }

  /// Latest RT telemetry (user threads; takes the buffer's read mutex, never blocks the RT thread).
  [[nodiscard]] SerlJointResetSnapshot snapshot() { return snapshot_buffer_.get(); }

  /// The tick body without franky's robot (for tests): returns the command and
  /// whether it is the final one. Must only be called from the control thread.
  [[nodiscard]] SerlJointResetCommand computeCommand(const Vector7d &q_measured, uint64_t time_step_ms);

 protected:
  void initImpl(const franky::RobotState &robot_state,
                const std::optional<franka::JointPositions> &previous_command) override;

  franka::JointPositions nextCommandImpl(const franky::RobotState &robot_state, franka::Duration time_step,
                                         franka::Duration rel_time, franka::Duration abs_time,
                                         const std::optional<franka::JointPositions> &previous_command) override;

 private:
  Vector7d target_;
  SerlJointPositionLaw law_;
  PositionJointSoftLimits soft_limits_;
  Vector7d last_command_{Vector7d::Zero()};
  uint64_t soft_limited_ticks_{0};
  std::atomic<bool> finish_requested_{false};
  franky::WaitFreeTripleBuffer<SerlJointResetSnapshot> snapshot_buffer_;
  uint64_t tick_{0};
  bool finished_{false};
};

}  // namespace serl_controller
