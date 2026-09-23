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

#include <cstdint>
#include <functional>
#include <unordered_map>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "people_msgs/msg/people.hpp"
#include "people_msgs/msg/person.hpp"
#include "rclcpp/rclcpp.hpp"

namespace coresense_people_estimation
{

/// Keeps publishing a person the detector has stopped reporting.
///
/// Everything else in this package is driven by a detection arriving: no
/// detection, no `Person`, so somebody the camera loses — clipped out of frame
/// by a turn, or walking out past `depth_max_m` — leaves `/people` on the very
/// next frame and nav2 plans straight through where they are. That is the
/// failure this class exists to prevent: it carries a track forward on its last
/// known velocity until either the detector finds it again or `coast_max_s` runs
/// out. See the README.
///
/// **Only somebody the camera can no longer see.** A track whose extrapolated
/// position is still inside the image is dropped instead, because the camera
/// looked there and the detector reported nobody. That deliberately gives up the
/// in-frame cases — a person behind a pillar, a detector stuttering under load —
/// in exchange for never inventing a person the camera is looking straight at.
///
/// It works on the **output** side, on people already re-framed into
/// `target_frame`, and that placement is the whole trick: the target frame is
/// world-fixed, so a person standing still has a constant position in it and
/// extrapolating needs no knowledge of what the robot did meanwhile. Coasting in
/// the camera frame would have to undo the robot's own motion first, which is
/// exactly the error the map transform exists to correct.
///
/// Not a node: it declares its parameters on the owning node and logs through it.
class TrackCoaster
{
public:
  /// Whether a point in the frame these are published in is inside the camera image
  /// right now. Only the caller knows where the camera is pointing.
  typedef std::function<bool(const geometry_msgs::msg::Point&)> InViewTest;

  explicit TrackCoaster(rclcpp::Node& node);

  /// Everything to publish this frame for tracks that were *not* detected in it.
  /// `seen_now` is this frame's ByteTrack ids, which are skipped: a track with a
  /// live measurement must never also appear as an extrapolation of itself.
  ///
  /// A track `in_view` accepts is erased, not withheld: the camera looked and the
  /// detector found nobody, and withholding would republish the ghost as soon as the
  /// robot turned away. `in_view` is empty only when TF could not say where the
  /// camera points, which runs coasting ungated rather than dropping everybody.
  /// See the README.
  ///
  /// The result is already in `target_frame`, so it is appended to the outgoing
  /// message rather than transformed with it.
  std::vector<people_msgs::msg::Person> coast(double stamp, const std::vector<std::int32_t>& seen_now,
                                              const InViewTest& in_view);

  /// Remember what was published for the detected people, as the anchor the next
  /// coasted frame extrapolates from. `track_ids[i]` is the ByteTrack id behind
  /// `people.people[i]`; anything past the end of `track_ids` is ignored.
  ///
  /// Call *after* the map transform, so what is stored is world-fixed, and
  /// *before* `coast()` appends to the same message — an extrapolation recorded
  /// as a sighting would let a track coast forever on its own output.
  void observe(const people_msgs::msg::People& people, const std::vector<std::int32_t>& track_ids, double stamp);

  /// Stop coasting this track immediately, dropping whatever was remembered of
  /// it. For the case where a detection *did* arrive and was rejected as not
  /// being a person at all — coasting answers "where would they be by now",
  /// which is the wrong question about something that was never there. See the
  /// README.
  void forget(std::int32_t track_id);

  /// Forget tracks past the horizon, so churning ids cannot grow the map.
  void dropStale(double now);

private:
  /// The last sighting of one track, exactly as it was published.
  struct State
  {
    /// Carried whole, so name, reliability and tags survive into the coasted
    /// copy and a consumer keying on `name` sees one continuous person.
    people_msgs::msg::Person person;
    double stamp{ 0.0 };  ///< seconds, from the detection message header
  };

  /// Logger and clock come from it.
  rclcpp::Node& node_;

  std::unordered_map<std::int32_t, State> states_;

  // ── Parameters ──────────────────────────────────────────────────────────────
  double coast_max_s_;
  double coast_process_sigma_;
};

}  // namespace coresense_people_estimation
