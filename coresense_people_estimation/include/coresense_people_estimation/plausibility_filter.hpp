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
#include <opencv2/core.hpp>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "riskam_msgs/msg/person_detection.hpp"

namespace coresense_people_estimation
{

/// Verdict on whether a detection is a body or a picture of one.
struct Plausibility
{
  bool plausible{ true };
  double background_frac{ 0.0 };  ///< share of the head strip that lies behind the person
  std::size_t n_pixels{ 0 };      ///< pixels the strip covered, 0 when it could not be read
};

/// Rejects detections of people who are not there: posters, screens, banners.
///
/// A body has a scene behind its head; a printed one *is* the scene. The test is
/// hysteretic, so a single bad frame cannot blink a real person out. See the
/// README.
///
/// Not a node: it declares its parameters on the owning node and logs through
/// it. The valid-depth floor is passed in, because the node and the depth
/// read-out judge against the same parameter.
class PlausibilityFilter
{
public:
  PlausibilityFilter(rclcpp::Node& node, double depth_min_m);

  /// Decide whether this detection is a body or a picture of one. Accumulates a
  /// streak per track, so the verdict depends on the frames before it.
  Plausibility screen(const cv::Mat& depth_m, const riskam_msgs::msg::PersonDetection& detection, double depth_z,
                      double stamp);

  /// Forget streaks for tracks that have stopped being seen.
  void dropStale(double now);

private:
  /// How many consecutive frames a track has failed the gate. Aged out on its own
  /// timeout, separate from the tracker's. See the README.
  struct State
  {
    std::size_t reject_streak{ 0 };
    double stamp{ 0.0 };
  };

  /// Share of a strip across the top of the bounding box that is *behind* the
  /// person.
  Plausibility backgroundFraction(const cv::Mat& depth_m, const riskam_msgs::msg::PersonDetection& detection,
                                  double depth_z) const;

  /// Logger and clock come from it.
  rclcpp::Node& node_;

  std::unordered_map<std::int32_t, State> states_;

  // ── Parameters ──────────────────────────────────────────────────────────────
  double background_strip_frac_;
  double background_step_m_;
  double background_min_far_frac_;
  std::size_t background_min_pixels_;
  std::size_t implausible_patience_;
  double plausibility_reset_timeout_;

  // ── Injected by the owning node ─────────────────────────────────────────────
  const double depth_min_m_;
};

}  // namespace coresense_people_estimation
