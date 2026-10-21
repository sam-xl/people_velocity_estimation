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

#include <Eigen/Core>
#include <cstddef>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>

#include "people_msgs/msg/people.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace people_velocity_estimation
{

/// Draws what the pipeline computed, so the geometry can be checked by eye.
///
/// The output topic carries plausible-looking numbers whether or not the depth
/// read, the velocity fit and the map transform are right; markers in RViz are
/// what tell them apart. The transformed output and the raw sensor-frame input
/// are drawn together deliberately: where the two disagree localises the fault.
/// See the README.
///
/// Not a node: it declares its parameters on the owning node and publishes
/// through it. Off is genuinely off — with `debug` false neither the marker
/// publisher nor the TF broadcaster is created.
class DebugVisualizer
{
public:
  /// `gravity_aligned` says which convention the people handed to `publish`
  /// follow: true for a z-up target frame, false for the sensor's optical frame.
  /// Only the heading of a person's TF frame depends on it.
  DebugVisualizer(rclcpp::Node& node, bool gravity_aligned);

  /// The robot's own velocity, drawn from the point it turns about. This is the
  /// vector the map transform added to every person, so an arrow pointing the
  /// wrong way explains every wrong person velocity at once.
  struct RobotArrow
  {
    Eigen::Vector3d origin;
    Eigen::Vector3d linear;
  };

  /// True when the untransformed input is worth keeping a copy of, so the
  /// caller pays for that copy only when someone is looking.
  bool showsRaw() const
  {
    return debug_ && debug_show_raw_;
  }

  /// Draws one frame's worth of geometry: a sphere, a look-ahead arrow and a
  /// label per person in `people`, the untransformed `sensor_frame_people`
  /// alongside, and the robot velocity that turned one into the other. No-op
  /// unless `debug` is set.
  void publish(const people_msgs::msg::People& people, const people_msgs::msg::People& sensor_frame_people,
               const std::optional<RobotArrow>& robot);

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

  // ── Injected by the owning node ───────────────────────────────────────────
  /// Set when the map transform is on, so published people are z-up rather than
  /// in the optical frame. Decided once at construction: `target_frame` is not
  /// re-read per message.
  const bool gravity_aligned_;

  // ── Parameters ────────────────────────────────────────────────────────────
  bool debug_;
  bool debug_person_frames_;
  bool debug_show_raw_;
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

}  // namespace people_velocity_estimation
