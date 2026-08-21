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

#include "coresense_people_estimation/plausibility_filter.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iterator>
#include <opencv2/core.hpp>

#include "coresense_people_estimation/parameters.hpp"
#include "rclcpp/rclcpp.hpp"
#include "riskam_msgs/msg/person_detection.hpp"

namespace coresense_people_estimation
{

PlausibilityFilter::PlausibilityFilter(rclcpp::Node& node, double depth_min_m)
  : node_(node)
  , depth_min_m_(depth_min_m)
{
  // See parameters.hpp for the count helper.
  background_strip_frac_ = node_.declare_parameter<double>("background_strip_frac", 0.25);
  background_step_m_ = node_.declare_parameter<double>("background_step_m", 0.3);
  background_min_far_frac_ = node_.declare_parameter<double>("background_min_far_frac", 0.03);
  background_min_pixels_ = declareCount(node_, "background_min_pixels", 40);
  implausible_patience_ = declareCount(node_, "implausible_patience", 3);
  plausibility_reset_timeout_ = node_.declare_parameter<double>("plausibility_reset_timeout", 2.0);
}

Plausibility PlausibilityFilter::screen(const cv::Mat& depth_m, const riskam_msgs::msg::PersonDetection& detection,
                                        double depth_z, double stamp)
{
  Plausibility verdict = backgroundFraction(depth_m, detection, depth_z);

  // Too few pixels to read is not evidence of anything, either way.
  const bool readable = verdict.n_pixels >= background_min_pixels_;

  // background_min_far_frac of 0 disables the gate.
  if (background_min_far_frac_ > 0.0 && readable && verdict.background_frac < background_min_far_frac_)
  {
    verdict.plausible = false;
  }

  // No identity to accumulate a verdict against, so judged on this frame alone.
  if (detection.track_id < 0)
  {
    return verdict;
  }

  State& state = states_[detection.track_id];
  state.stamp = stamp;

  // An unreadable strip leaves the streak untouched rather than clearing it.
  // Clearing would let a banner that intermittently clips the frame edge, or
  // recedes until the strip is too small, never reach implausible_patience.
  if (!readable)
  {
    return verdict;
  }

  if (verdict.plausible)
  {
    state.reject_streak = 0;
    return verdict;
  }

  ++state.reject_streak;
  // Hysteresis: only a track that keeps failing is suppressed. See the README.
  if (state.reject_streak <= implausible_patience_)
  {
    verdict.plausible = true;
    return verdict;
  }

  RCLCPP_INFO_THROTTLE(node_.get_logger(), *node_.get_clock(), kWarnThrottleMs,
                       "Suppressing track %d for %zu frames running: only %.0f%% of the strip above it is behind it "
                       "(needs %.0f%%) at %.2f m, so this is a picture of a person.",
                       detection.track_id, state.reject_streak, 100.0 * verdict.background_frac,
                       100.0 * background_min_far_frac_, depth_z);
  return verdict;
}

Plausibility PlausibilityFilter::backgroundFraction(const cv::Mat& depth_m,
                                                    const riskam_msgs::msg::PersonDetection& detection,
                                                    double depth_z) const
{
  Plausibility result;

  // A strip across the top of the box, full width: the head is narrow and the box
  // is not, so most of it reads whatever is behind the person. Bounds are
  // half-open on both axes, matching the bbox convention and the loops below.
  const int x0 = std::clamp(std::min(detection.bbox[0], detection.bbox[2]), 0, depth_m.cols);
  const int x1 = std::clamp(std::max(detection.bbox[0], detection.bbox[2]), 0, depth_m.cols);
  const int y0 = std::clamp(std::min(detection.bbox[1], detection.bbox[3]), 0, depth_m.rows);
  const int y_bottom = std::clamp(std::max(detection.bbox[1], detection.bbox[3]), 0, depth_m.rows);
  const int height = y_bottom - y0;
  if (x1 <= x0 || height <= 0)
  {
    return result;
  }

  // Rounded down to nothing for a short box, so floored at one row: a box worth
  // measuring at all is worth measuring one row of.
  const int strip_rows = std::max(1, static_cast<int>(std::lround(background_strip_frac_ * height)));
  const int y1 = std::min(y_bottom, y0 + strip_rows);

  // Behind the person: past them by more than a body's depth, or no return at all.
  // Deliberately not clipped to depth_max_m. See the README.
  const double far_threshold = depth_z + background_step_m_;
  std::size_t behind = 0;
  for (int y = y0; y < y1; ++y)
  {
    // decodeDepth guarantees CV_32F, so the row is addressable directly.
    const float* row = depth_m.ptr<float>(y);
    for (int x = x0; x < x1; ++x)
    {
      const float d = row[x];
      if (!std::isfinite(d) || d < depth_min_m_ || static_cast<double>(d) > far_threshold)
      {
        ++behind;
      }
    }
  }

  const auto total = static_cast<std::size_t>(y1 - y0) * static_cast<std::size_t>(x1 - x0);
  result.n_pixels = total;
  result.background_frac = total > 0 ? static_cast<double>(behind) / static_cast<double>(total) : 0.0;
  return result;
}

void PlausibilityFilter::dropStale(double now)
{
  // Its own timeout, longer than the spike filter's; see the README. Absolute
  // value because a looping bag walks the clock backwards.
  for (auto it = states_.begin(); it != states_.end();)
  {
    it = std::abs(now - it->second.stamp) > plausibility_reset_timeout_ ? states_.erase(it) : std::next(it);
  }
}

}  // namespace coresense_people_estimation
