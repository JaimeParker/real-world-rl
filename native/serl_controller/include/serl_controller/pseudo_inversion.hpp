// SPDX-License-Identifier: MIT
// Copyright (c) 2024 RAIL Berkeley (serl_franka_controllers); the function itself is public domain.
//
// Ported verbatim (namespace and function body unchanged otherwise) from
// https://github.com/rail-berkeley/serl_franka_controllers @ 1f140ef,
// include/serl_franka_controllers/pseudo_inversion.h, whose header reads:
//
// Author: Enrico Corvaglia
// https://github.com/CentroEPiaggio/kuka-lwr/blob/master/lwr_controllers/include/utils/pseudo_inversion.h
// File provided under public domain
// pseudo_inverse() computes the pseudo inverse of matrix M_ using SVD decomposition (can choose
// between damped and not)
// returns the pseudo inverted matrix M_pinv_
//
// serl_franka_controllers is Copyright (c) 2024 RAIL Berkeley, MIT License (see LICENSE).
//
// Kept verbatim (names, output parameter, dynamic-size types) because the golden tests
// require bit-identical torques; hence excluded from clang-tidy / clang-format below.
// It allocates (MatrixXd, JacobiSVD), as in SERL: see law.hpp, "Allocation".
#pragma once

#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/SVD>

namespace serl_controller {

// clang-format off
// NOLINTBEGIN -- verbatim SERL code (see above)
inline void pseudoInverse(const Eigen::MatrixXd& M_, Eigen::MatrixXd& M_pinv_, bool damped = true) {
  double lambda_ = damped ? 0.2 : 0.0;

  Eigen::JacobiSVD<Eigen::MatrixXd> svd(M_, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::JacobiSVD<Eigen::MatrixXd>::SingularValuesType sing_vals_ = svd.singularValues();
  Eigen::MatrixXd S_ = M_;  // copying the dimensions of M_, its content is not needed.
  S_.setZero();

  for (int i = 0; i < sing_vals_.size(); i++)
    S_(i, i) = (sing_vals_(i)) / (sing_vals_(i) * sing_vals_(i) + lambda_ * lambda_);

  M_pinv_ = Eigen::MatrixXd(svd.matrixV() * S_.transpose() * svd.matrixU().transpose());
}
// NOLINTEND
// clang-format on

}  // namespace serl_controller
