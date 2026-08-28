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

// ROS 2 style mandates this <PACKAGE>__<PATH>_HPP_ guard and ament's cpplint
// enforces it, so the reserved-identifier rule loses here.
#ifndef CORESENSE_PEOPLE_ESTIMATION__DEPTH_READER_HPP_
// NOLINTNEXTLINE(bugprone-reserved-identifier,cert-dcl37-c,cert-dcl51-cpp)
#define CORESENSE_PEOPLE_ESTIMATION__DEPTH_READER_HPP_

#include <opencv2/core.hpp>

#include "rclcpp/rclcpp.hpp"
#include "riskam_msgs/msg/person_detection.hpp"

namespace coresense_people_estimation
{

/// Pixel the depth is attributed to: hip midpoint when available, else the bbox
/// centre. See the README.
cv::Point2d anchorPixel(const riskam_msgs::msg::PersonDetection& detection);

/// Result of reading depth for one detection.
struct DepthEstimate
{
  double depth_m;     ///< representative distance to the person
  double spread_m;    ///< robust scale of the readings that produced it
  double valid_frac;  ///< fraction of sampled pixels that carried valid depth
};

/// Reads how far away a detected person is, out of the registered depth image.
///
/// Median of small patches around the torso keypoints (shoulders + hips), so the
/// reading comes off the body. Falls back to a low percentile over the whole
/// bounding box when no torso keypoint carries depth. See the README.
///
/// Not a node: it declares its parameters on the owning node. The valid-depth
/// band is passed in rather than declared here, because the node and the
/// plausibility filter judge against the same two parameters.
class DepthReader
{
public:
  DepthReader(rclcpp::Node& node, double depth_min_m, double depth_max_m);

  DepthEstimate read(const cv::Mat& depth_m, const riskam_msgs::msg::PersonDetection& detection) const;

private:
  // ── Parameters ──────────────────────────────────────────────────────────────
  double depth_percentile_{ 10.0 };
  int keypoint_patch_radius_{ 3 };

  /// Declared by the owning node, not here. Readings outside this band are
  /// treated as missing.
  double depth_min_m_{ 0.1 };
  double depth_max_m_{ 3.0 };
};

}  // namespace coresense_people_estimation

#endif  // CORESENSE_PEOPLE_ESTIMATION__DEPTH_READER_HPP_
