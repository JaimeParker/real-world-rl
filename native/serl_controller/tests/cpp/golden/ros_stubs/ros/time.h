#pragma once
// ros::Time is unused by the controllers' math. ros::Duration is used by SERL's
// JointPositionController (elapsed_time_ += period; elapsed_time_.toSec()), so it
// reproduces roscpp_core's rostime arithmetic exactly (ros/roscpp_core
// noetic-devel @ a1a1942, rostime/include/ros/duration.h, impl/duration.h,
// src/duration.cpp: int32 sec + int32 nsec, fromSec() rounding to the nearest
// nanosecond, operator+ via toNSec()/fromNSec(), normalizeSecNSecSigned(), toSec()).
//
// SPDX-License-Identifier: BSD-3-Clause
// The ros::Duration arithmetic below is reproduced from roscpp_core, whose notice reads:
//
// Software License Agreement (BSD License)
//
// Copyright (c) 2008, Willow Garage, Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of Willow Garage, Inc. nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>

namespace ros {
struct Time {};

inline void normalizeSecNSecSigned(int64_t &sec, int64_t &nsec) {
  int64_t nsec_part = nsec % 1000000000L;
  int64_t sec_part = sec + nsec / 1000000000L;
  if (nsec_part < 0) {
    nsec_part += 1000000000L;
    --sec_part;
  }
  if (sec_part < std::numeric_limits<int32_t>::min() || sec_part > std::numeric_limits<int32_t>::max())
    throw std::runtime_error("Duration is out of dual 32-bit range");
  sec = sec_part;
  nsec = nsec_part;
}

inline void normalizeSecNSecSigned(int32_t &sec, int32_t &nsec) {
  int64_t sec64 = sec;
  int64_t nsec64 = nsec;
  normalizeSecNSecSigned(sec64, nsec64);
  sec = (int32_t)sec64;
  nsec = (int32_t)nsec64;
}

class Duration {
 public:
  int32_t sec{0}, nsec{0};
  Duration() = default;
  Duration(int32_t _sec, int32_t _nsec) : sec(_sec), nsec(_nsec) { normalizeSecNSecSigned(sec, nsec); }
  explicit Duration(double t) { fromSec(t); }

  Duration &fromSec(double d) {
    if (!std::isfinite(d)) throw std::runtime_error("Duration has to be finite.");
    int64_t sec64 = static_cast<int64_t>(floor(d));
    if (sec64 < std::numeric_limits<int32_t>::min() || sec64 > std::numeric_limits<int32_t>::max())
      throw std::runtime_error("Duration is out of dual 32-bit range");
    sec = static_cast<int32_t>(sec64);
    nsec = static_cast<int32_t>(std::round((d - sec) * 1e9));  // boost::math::round: half away from zero
    int32_t rollover = nsec / 1000000000ul;
    sec += rollover;
    nsec %= 1000000000ul;
    return *this;
  }
  Duration &fromNSec(int64_t t) {
    int64_t sec64 = t / 1000000000LL;
    if (sec64 < std::numeric_limits<int32_t>::min() || sec64 > std::numeric_limits<int32_t>::max())
      throw std::runtime_error("Duration is out of dual 32-bit range");
    sec = static_cast<int32_t>(sec64);
    nsec = static_cast<int32_t>(t % 1000000000LL);
    normalizeSecNSecSigned(sec, nsec);
    return *this;
  }
  double toSec() const { return static_cast<double>(sec) + 1e-9 * static_cast<double>(nsec); }
  int64_t toNSec() const { return static_cast<int64_t>(sec) * 1000000000ll + static_cast<int64_t>(nsec); }
  Duration operator+(const Duration &rhs) const {
    Duration t;
    return t.fromNSec(toNSec() + rhs.toNSec());
  }
  Duration &operator+=(const Duration &rhs) {
    *this = *this + rhs;
    return *this;
  }
};
}  // namespace ros
