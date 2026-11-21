// SPDX-License-Identifier: MIT
// Minimal ROS stand-in for compiling the vendored SERL controller (see README.md); no control math.
#pragma once
// The vendored source calls boost::bind(&C::f, this, _1, _2) with unqualified
// placeholders. Force-included (-include) into the golden-reference target only.
#include <functional>
using std::placeholders::_1;
using std::placeholders::_2;
namespace boost {
template <class... A>
auto bind(A &&...a) {
  return std::bind(std::forward<A>(a)...);
}
}  // namespace boost
