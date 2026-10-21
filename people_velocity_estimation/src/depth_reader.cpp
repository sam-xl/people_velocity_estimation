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

#include "people_velocity_estimation/depth_reader.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iterator>
#include <opencv2/core.hpp>
#include <vector>

#include "people_velocity_estimation/keypoints.hpp"
#include "rclcpp/rclcpp.hpp"
#include "riskam_msgs/msg/person_detection.hpp"

namespace
{
/// Nearest-rank percentile. Partially reorders `values`.
double percentile(std::vector<float>& values, double percent)
{
  if (values.empty())
  {
    return 0.0;
  }
  const double rank = percent / 100.0 * static_cast<double>(values.size() - 1);
  const auto index = static_cast<std::size_t>(std::lround(rank));
  auto nth = values.begin() + static_cast<std::ptrdiff_t>(index);
  std::nth_element(values.begin(), nth, values.end());
  return static_cast<double>(*nth);
}

/// Appends every finite reading of `patch` inside [min_m, max_m] to `valid`.
void collectValidDepths(const cv::Mat& patch, double min_m, double max_m, std::vector<float>& valid)
{
  for (int y = 0; y < patch.rows; ++y)
  {
    const float* row = patch.ptr<float>(y);
    std::copy_if(row, row + patch.cols, std::back_inserter(valid),
                 [min_m, max_m](float d) { return std::isfinite(d) && d >= min_m && d <= max_m; });
  }
}
}  // namespace

namespace people_velocity_estimation
{

cv::Point2d anchorPixel(const riskam_msgs::msg::PersonDetection& detection)
{
  const double lhx = detection.keypoints[2 * kLeftHip];
  const double lhy = detection.keypoints[2 * kLeftHip + 1];
  const double rhx = detection.keypoints[2 * kRightHip];
  const double rhy = detection.keypoints[2 * kRightHip + 1];
  if (keypointPresent(lhx, lhy) && keypointPresent(rhx, rhy))
  {
    return { 0.5 * (lhx + rhx), 0.5 * (lhy + rhy) };
  }
  return { 0.5 * (detection.bbox[0] + detection.bbox[2]), 0.5 * (detection.bbox[1] + detection.bbox[3]) };
}

DepthReader::DepthReader(rclcpp::Node& node, double depth_min_m, double depth_max_m)
  : depth_min_m_(depth_min_m)
  , depth_max_m_(depth_max_m)
{
  const double requested_percentile = node.declare_parameter<double>("depth_percentile", 10.0);
  depth_percentile_ = std::clamp(requested_percentile, 0.0, 100.0);
  if (depth_percentile_ != requested_percentile)
  {
    RCLCPP_WARN(node.get_logger(), "depth_percentile %.1f is outside [0, 100]; using %.1f.", requested_percentile,
                depth_percentile_);
  }
  keypoint_patch_radius_ = static_cast<int>(node.declare_parameter<std::int64_t>("keypoint_patch_radius", 3));
}

DepthEstimate DepthReader::read(const cv::Mat& depth_m, const riskam_msgs::msg::PersonDetection& detection) const
{
  // Shoulders and hips: the keypoints that sit on the trunk, so their depth is
  // the person's depth.
  static constexpr std::array<std::size_t, 4> kTorso{ { kLeftShoulder, kRightShoulder, kLeftHip, kRightHip } };

  const cv::Rect image(0, 0, depth_m.cols, depth_m.rows);
  std::vector<float> valid;
  std::size_t sampled = 0;

  for (const std::size_t k : kTorso)
  {
    if (!keypointPresent(detection, k))
    {
      continue;
    }
    const cv::Point centre(static_cast<int>(std::lround(detection.keypoints[2 * k])),
                           static_cast<int>(std::lround(detection.keypoints[2 * k + 1])));
    // A patch rather than a single pixel: depth images are speckled with dropouts.
    const int side = 2 * keypoint_patch_radius_ + 1;
    const cv::Rect patch =
        cv::Rect(centre.x - keypoint_patch_radius_, centre.y - keypoint_patch_radius_, side, side) & image;
    sampled += static_cast<std::size_t>(patch.area());
    collectValidDepths(depth_m(patch), depth_min_m_, depth_max_m_, valid);
  }

  if (!valid.empty() && sampled > 0)
  {
    // Median, not a low percentile: every sample here is already on the body.
    const double median = percentile(valid, 50.0);
    const double low = percentile(valid, depth_percentile_);
    return DepthEstimate{ median, std::max(0.0, median - low),
                          static_cast<double>(valid.size()) / static_cast<double>(sampled), false };
  }

  // ── Fallback: the whole bounding box ────────────────────────────────────────
  // A worse estimate beats dropping a person nav2 should be avoiding.
  // Inclusive corners, so the box is one pixel wider than its corner difference;
  // an inverted box has a negative width and intersects to nothing.
  const cv::Rect bbox = cv::Rect(detection.bbox[0], detection.bbox[1], detection.bbox[2] - detection.bbox[0] + 1,
                                 detection.bbox[3] - detection.bbox[1] + 1) &
                        image;
  if (bbox.empty())
  {
    return DepthEstimate{ 0.0, 0.0, 0.0, true };
  }
  const auto bbox_sampled = static_cast<std::size_t>(bbox.area());

  valid.clear();
  valid.reserve(bbox_sampled);
  collectValidDepths(depth_m(bbox), depth_min_m_, depth_max_m_, valid);
  if (valid.empty())
  {
    return DepthEstimate{ 0.0, 0.0, 0.0, true };
  }

  // Low percentile = "closest valid surface in this box", which is the person
  // rather than the wall behind them. See the README.
  const double low = percentile(valid, depth_percentile_);
  const double median = percentile(valid, 50.0);
  return DepthEstimate{ low, std::max(0.0, median - low),
                        static_cast<double>(valid.size()) / static_cast<double>(bbox_sampled), true };
}

}  // namespace people_velocity_estimation
