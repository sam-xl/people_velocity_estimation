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
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "riskam_msgs/msg/person_detection.hpp"

namespace people_velocity_estimation
{

/// Withholds a detection until it has been seen often enough to be believed.
///
/// A detector that draws a box in a random spot for one frame and drops it the
/// next has produced no evidence of a person. Nothing downstream can tell that
/// box from a real one, so it is stopped here, ahead of everything else. See
/// docs/confirmation.md.
///
/// Not a node: it declares its parameters on the owning node and logs through
/// it. Runs before the depth read rather than after, so an unconfirmed detection
/// costs nothing.
class ConfirmationFilter
{
public:
  explicit ConfirmationFilter(rclcpp::Node& node);

  /// Fold this frame's sighting into the detection's track, and report whether
  /// it has now been seen `confirm_frames` frames running.
  bool confirmed(const riskam_msgs::msg::PersonDetection& detection, double stamp);

  /// Forget tracks that have stopped being seen, so churning ids cannot grow the
  /// map. Also what makes the count consecutive; see docs/confirmation.md.
  void dropStale(double now);

private:
  /// How many frames running a track has been seen. Aged out on its own timeout,
  /// separate from the tracker's.
  struct State
  {
    std::size_t streak{ 0 };
    bool confirmed{ false };
    double stamp{ 0.0 };
  };

  /// Logger and clock come from it.
  rclcpp::Node& node_;

  std::unordered_map<std::int32_t, State> states_;

  // ── Parameters ──────────────────────────────────────────────────────────────
  std::size_t confirm_frames_;
  double confirm_reset_timeout_;
};

}  // namespace people_velocity_estimation
