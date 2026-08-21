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

#include "coresense_people_estimation/track_coaster.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iterator>
#include <string>
#include <utility>
#include <vector>

#include "people_msgs/msg/people.hpp"
#include "people_msgs/msg/person.hpp"
#include "rclcpp/rclcpp.hpp"

namespace coresense_people_estimation
{

TrackCoaster::TrackCoaster(rclcpp::Node& node)
  : node_(node)
{
  coast_max_s_ = node_.declare_parameter<double>("coast_max_s", 3.0);
  coast_process_sigma_ = node_.declare_parameter<double>("coast_process_sigma", 0.3);
}

std::vector<people_msgs::msg::Person> TrackCoaster::coast(double stamp, const std::vector<std::int32_t>& seen_now,
                                                          const InViewTest& in_view)
{
  std::vector<people_msgs::msg::Person> coasted;
  // 0 disables coasting entirely, which is the A/B switch for judging whether it
  // helps.
  if (coast_max_s_ <= 0.0)
  {
    return coasted;
  }

  for (auto it = states_.begin(); it != states_.end();)
  {
    if (std::find(seen_now.begin(), seen_now.end(), it->first) != seen_now.end())
    {
      it = std::next(it);
      continue;
    }

    const State& state = it->second;
    const double dt = stamp - state.stamp;
    // Non-positive means this frame or a bag that looped; past the horizon means
    // the track is somebody else's problem now.
    if (dt <= 0.0 || dt > coast_max_s_)
    {
      it = std::next(it);
      continue;
    }

    people_msgs::msg::Person person = state.person;
    // Constant velocity, from the anchor rather than from the last extrapolation,
    // so the error stays proportional to dt instead of compounding frame by frame.
    person.position.x = state.person.position.x + state.person.velocity.x * dt;
    person.position.y = state.person.position.y + state.person.velocity.y * dt;
    person.position.z = state.person.position.z + state.person.velocity.z * dt;

    // Erased rather than skipped, so turning away cannot bring the ghost back.
    if (in_view && in_view(person.position))
    {
      it = states_.erase(it);
      continue;
    }

    // The velocity itself is held, untouched: not seeing somebody is no evidence
    // that they stopped. Decaying it towards zero would park a walking person in
    // the costmap and let nav2 plan into the space they are still crossing.

    // Confidence falls linearly to nothing at the horizon, so a coasted person
    // fades out of nav2's social layer instead of blinking out of it.
    person.reliability = state.person.reliability * std::max(0.0, 1.0 - dt / coast_max_s_);

    // Appended whatever `publish_covariance_tags` says, because these two are not
    // diagnostics: a consumer that cannot tell an extrapolation from a
    // measurement will trust this position as much as a measured one, and after
    // a second of coasting that is not a position anybody should be trusting.
    person.tagnames.push_back("coast_s");
    person.tags.push_back(std::to_string(dt));
    person.tagnames.push_back("coast_position_sigma_m");
    person.tags.push_back(std::to_string(coast_process_sigma_ * dt));

    coasted.push_back(std::move(person));
    it = std::next(it);
  }
  return coasted;
}

void TrackCoaster::observe(const people_msgs::msg::People& people, const std::vector<std::int32_t>& track_ids,
                           double stamp)
{
  if (coast_max_s_ <= 0.0)
  {
    return;
  }

  // Only as far as `track_ids` reaches. The coasted people the map transform
  // appended sit past that end, and re-recording an extrapolation as if it were a
  // sighting would let a track coast forever on its own output.
  const std::size_t n = std::min(track_ids.size(), people.people.size());
  for (std::size_t i = 0; i < n; ++i)
  {
    // ByteTrack reports every unassociated detection as -1, so they all share the
    // one id and carry no identity to coast against.
    if (track_ids[i] < 0)
    {
      continue;
    }
    State& state = states_[track_ids[i]];
    state.person = people.people[i];
    state.stamp = stamp;
  }
}

void TrackCoaster::forget(std::int32_t track_id)
{
  states_.erase(track_id);
}

void TrackCoaster::dropStale(double now)
{
  for (auto it = states_.begin(); it != states_.end();)
  {
    // Absolute value because a looping bag walks the clock backwards.
    it = std::abs(now - it->second.stamp) > coast_max_s_ ? states_.erase(it) : std::next(it);
  }
}

}  // namespace coresense_people_estimation
