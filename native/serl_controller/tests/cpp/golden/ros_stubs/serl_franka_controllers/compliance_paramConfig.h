// SPDX-License-Identifier: MIT
// Minimal ROS stand-in for compiling the vendored SERL controller (see README.md); no control math.
#pragma once
// Stand-in for the header dynamic_reconfigure generates from cfg/compliance_param.cfg:
// the same fields with the same defaults (see ../../serl_original/cfg/compliance_param.cfg).
namespace serl_franka_controllers {
struct compliance_paramConfig {  // NOLINT
  double translational_stiffness{2000};
  double translational_damping{89};
  double rotational_stiffness{150};
  double rotational_damping{7};
  double nullspace_stiffness{0.2};
  double joint1_nullspace_stiffness{100};
  double translational_clip_neg_x{0.01};
  double translational_clip_neg_y{0.01};
  double translational_clip_neg_z{0.01};
  double translational_clip_x{0.01};
  double translational_clip_y{0.01};
  double translational_clip_z{0.01};
  double rotational_clip_neg_x{0.05};
  double rotational_clip_neg_y{0.05};
  double rotational_clip_neg_z{0.05};
  double rotational_clip_x{0.05};
  double rotational_clip_y{0.05};
  double rotational_clip_z{0.05};
  double translational_Ki{0};  // NOLINT
  double rotational_Ki{0};     // NOLINT
};
}  // namespace serl_franka_controllers
