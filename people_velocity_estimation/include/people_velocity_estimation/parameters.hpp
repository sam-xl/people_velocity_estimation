// Copyright 2026 TU Delft / SAM XL
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

#include "rclcpp/rclcpp.hpp"

namespace people_velocity_estimation
{

/// Shared by every throttled log in the package: detections arrive at camera
/// rate, so one message per detection floods the console.
constexpr int kWarnThrottleMs = 5000;

/// Declare a parameter counting frames or pixels, which is meaningless below
/// zero. ROS 2 has no unsigned parameter type, so a negative value would wrap to
/// a huge `size_t` and invert whatever gate it feeds — a `-1` meant as "off"
/// would turn the gate permanently on.
inline std::size_t declareCount(rclcpp::Node& node, const std::string& name, std::size_t fallback)
{
  const auto declared = node.declare_parameter<std::int64_t>(name, static_cast<std::int64_t>(fallback));
  if (declared < 0)
  {
    RCLCPP_WARN(node.get_logger(), "%s of %ld is negative and cannot be a count; using 0.", name.c_str(), declared);
    return 0;
  }
  return static_cast<std::size_t>(declared);
}

}  // namespace people_velocity_estimation
