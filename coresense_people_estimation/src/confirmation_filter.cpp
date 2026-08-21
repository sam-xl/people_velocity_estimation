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

#include "coresense_people_estimation/confirmation_filter.hpp"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iterator>

#include "coresense_people_estimation/parameters.hpp"
#include "rclcpp/rclcpp.hpp"
#include "riskam_msgs/msg/person_detection.hpp"

namespace coresense_people_estimation
{

ConfirmationFilter::ConfirmationFilter(rclcpp::Node& node)
  : node_(node)
{
  // See parameters.hpp for why a count is not declared as a plain int64.
  confirm_frames_ = declareCount(node_, "confirm_frames", 3);
  confirm_reset_timeout_ = node_.declare_parameter<double>("confirm_reset_timeout", 0.5);
}

bool ConfirmationFilter::confirmed(const riskam_msgs::msg::PersonDetection& detection, double stamp)
{
  // One frame of evidence is what every detection already has, so anything below
  // two confirms on sight. This is the A/B switch for judging whether the gate
  // helps: at 0 the node behaves exactly as it did before it existed.
  if (confirm_frames_ <= 1)
  {
    return true;
  }

  // ByteTrack reports every unassociated detection as -1, so they all share one
  // id and no streak can be kept per detection. Unassociated is also the state a
  // one-frame phantom arrives in, so this is a rejection, not a pass. See the
  // README.
  if (detection.track_id < 0)
  {
    RCLCPP_INFO_THROTTLE(node_.get_logger(), *node_.get_clock(), kWarnThrottleMs,
                         "Dropping a detection with no ByteTrack id: it has no identity to hold a history against.");
    return false;
  }

  State& state = states_[detection.track_id];
  state.stamp = stamp;
  if (state.confirmed)
  {
    return true;
  }

  ++state.streak;
  if (state.streak >= confirm_frames_)
  {
    state.confirmed = true;
    return true;
  }

  RCLCPP_INFO_THROTTLE(node_.get_logger(), *node_.get_clock(), kWarnThrottleMs,
                       "Withholding track %d: seen %zu frames running, needs %zu.", detection.track_id, state.streak,
                       confirm_frames_);
  return false;
}

void ConfirmationFilter::dropStale(double now)
{
  // A track unseen for this long starts its streak over, which is what makes the
  // count consecutive. The stamp is refreshed on every sighting, so the timeout
  // has to clear one detection interval, not the whole window. Absolute value
  // because a looping bag walks the clock backwards.
  for (auto it = states_.begin(); it != states_.end();)
  {
    it = std::abs(now - it->second.stamp) > confirm_reset_timeout_ ? states_.erase(it) : std::next(it);
  }
}

}  // namespace coresense_people_estimation
