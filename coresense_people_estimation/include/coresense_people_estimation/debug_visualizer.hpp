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
#include <memory>
#include <string>
#include <unordered_map>

#include "people_msgs/msg/people.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace coresense_people_estimation
{

/// Draws what the pipeline computed, so the geometry can be checked by eye.
///
/// The output topic carries plausible-looking numbers whether or not the depth
/// read and the velocity fit are right; markers in RViz are what tell the two
/// apart. See the README.
///
/// Not a node: it declares its parameters on the owning node and publishes
/// through it. Off is genuinely off — with `debug` false neither the marker
/// publisher nor the TF broadcaster is created.
class DebugVisualizer
{
public:
  explicit DebugVisualizer(rclcpp::Node& node);

  /// Draws one frame's worth of geometry: a sphere, a look-ahead arrow and a
  /// label per person, in whatever frame `people` carries. No-op unless `debug`
  /// is set.
  void publish(const people_msgs::msg::People& people);

private:
  /// Broadcasts one TF frame per person, positioned at their position and yawed
  /// along their velocity, so a person's heading can be read straight off the
  /// TF display.
  void publishPersonFrames(const people_msgs::msg::People& people);

  /// Appends DELETE markers for ids published into `ns` last frame that this
  /// frame no longer uses, then records `count` as the new high-water mark.
  /// Without this, markers for people who have left the scene linger until
  /// their lifetime expires — or forever, when the lifetime is 0.
  void reapStaleMarkers(visualization_msgs::msg::MarkerArray& array, const std::string& ns, std::size_t count);

  /// Clock, logger, parameters and publishers all come from it.
  rclcpp::Node& node_;

  // ── Parameters ────────────────────────────────────────────────────────────
  bool debug_;
  bool debug_person_frames_;
  /// Seconds of look-ahead an arrow represents: its length is speed * this, so
  /// at 1.0 the tip sits where the person would be one second from now.
  double debug_velocity_scale_;
  double debug_person_radius_;
  rclcpp::Duration debug_marker_lifetime_{ 0, 0 };

  /// Only created when debug person frames are enabled — a node that is not
  /// debugging should not appear as a TF broadcaster at all.
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  /// Highest marker id published per namespace, so stale ones can be deleted.
  std::unordered_map<std::string, std::size_t> debug_marker_counts_;

  /// Null unless `debug` is set.
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_marker_pub_;
};

}  // namespace coresense_people_estimation
