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

#include "coresense_people_estimation/depth_reader.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <opencv2/core.hpp>
#include <vector>

#include "coresense_people_estimation/keypoints.hpp"
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
}  // namespace

namespace coresense_people_estimation
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

  std::vector<float> valid;
  std::size_t sampled = 0;

  for (const std::size_t k : kTorso)
  {
    if (!keypointPresent(detection, k))
    {
      continue;
    }
    const double u = detection.keypoints[2 * k];
    const double v = detection.keypoints[2 * k + 1];
    // A patch rather than a single pixel: depth images are speckled with dropouts.
    const int u0 = std::max(0, static_cast<int>(std::lround(u)) - keypoint_patch_radius_);
    const int u1 = std::min(depth_m.cols - 1, static_cast<int>(std::lround(u)) + keypoint_patch_radius_);
    const int v0 = std::max(0, static_cast<int>(std::lround(v)) - keypoint_patch_radius_);
    const int v1 = std::min(depth_m.rows - 1, static_cast<int>(std::lround(v)) + keypoint_patch_radius_);

    for (int y = v0; y <= v1; ++y)
    {
      for (int x = u0; x <= u1; ++x)
      {
        ++sampled;
        const float d = depth_m.at<float>(y, x);
        if (std::isfinite(d) && d >= depth_min_m_ && d <= depth_max_m_)
        {
          valid.push_back(d);
        }
      }
    }
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
  const int x0 = std::clamp(detection.bbox[0], 0, depth_m.cols - 1);
  const int y0 = std::clamp(detection.bbox[1], 0, depth_m.rows - 1);
  const int x1 = std::clamp(detection.bbox[2], 0, depth_m.cols - 1);
  const int y1 = std::clamp(detection.bbox[3], 0, depth_m.rows - 1);
  // Clamping collapses a box that misses the image entirely onto the edge pixel,
  // so overlap is tested before it. A one-pixel-wide box is legitimate: the
  // sampling loops below are inclusive.
  const bool overlaps_image = detection.bbox[2] >= 0 && detection.bbox[0] < depth_m.cols && detection.bbox[3] >= 0 &&
                              detection.bbox[1] < depth_m.rows;
  if (!overlaps_image || x1 < x0 || y1 < y0)
  {
    return DepthEstimate{ 0.0, 0.0, 0.0, true };
  }

  // Multiply in std::size_t rather than widening the int product.
  const auto bbox_sampled = static_cast<std::size_t>(x1 - x0 + 1) * static_cast<std::size_t>(y1 - y0 + 1);

  valid.clear();
  valid.reserve(bbox_sampled);
  for (int y = y0; y <= y1; ++y)
  {
    for (int x = x0; x <= x1; ++x)
    {
      const float d = depth_m.at<float>(y, x);
      if (std::isfinite(d) && d >= depth_min_m_ && d <= depth_max_m_)
      {
        valid.push_back(d);
      }
    }
  }

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

}  // namespace coresense_people_estimation
