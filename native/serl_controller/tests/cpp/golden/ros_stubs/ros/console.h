// SPDX-License-Identifier: MIT
// Minimal ROS stand-in for compiling the vendored SERL controller (see README.md); no control math.
#pragma once
#include <iostream>
#define ROS_ERROR_STREAM(args) (std::cerr << args << std::endl)
#define ROS_ERROR(...) (std::cerr << __VA_ARGS__ << std::endl)
