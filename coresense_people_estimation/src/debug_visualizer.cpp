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

#include "coresense_people_estimation/debug_visualizer.hpp"

#include <Eigen/Core>
#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <memory>
#include <optional>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "people_msgs/msg/people.hpp"
#include "people_msgs/msg/person.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2_eigen/tf2_eigen.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace
{
/// Marker namespaces. Each is independently toggleable in RViz, so keeping
/// positions, velocities, labels and raw input apart is worth the extra ids.
constexpr char kNsPeople[] = "people";
constexpr char kNsVelocity[] = "velocity";
constexpr char kNsLabel[] = "label";
constexpr char kNsRawPeople[] = "raw_people";
constexpr char kNsRawVelocity[] = "raw_velocity";
constexpr char kNsRobot[] = "robot_velocity";

struct Rgba
{
  float r, g, b, a;
};

/// Transformed output is green/cyan, raw sensor-frame input is orange, and the
/// robot's own velocity — the term that turns one into the other — is magenta.
constexpr Rgba kPersonColor{ 0.20F, 0.85F, 0.35F, 0.85F };
constexpr Rgba kVelocityColor{ 0.10F, 0.95F, 0.85F, 0.95F };
constexpr Rgba kLabelColor{ 1.00F, 1.00F, 1.00F, 0.90F };
constexpr Rgba kRawPersonColor{ 1.00F, 0.55F, 0.10F, 0.45F };
constexpr Rgba kRawVelocityColor{ 1.00F, 0.35F, 0.10F, 0.70F };
constexpr Rgba kRobotColor{ 0.90F, 0.20F, 0.90F, 0.90F };

/// Shortest arrow worth drawing, in metres. RViz logs a warning for every
/// zero-length arrow it is handed, once per marker per frame.
constexpr double kMinArrowLength = 1e-3;

/// Arrow thickness as fractions of the person radius, so an arrow stays legible
/// against the sphere it comes out of whatever `debug_person_radius` is.
constexpr double kShaftDiameterFrac = 0.25;
constexpr double kHeadDiameterFrac = 0.60;
constexpr double kHeadLengthFrac = 0.80;

/// Label placement and size as fractions of the person radius, so the text clears
/// the sphere and stays proportionate whatever `debug_person_radius` is.
constexpr double kLabelHeightAboveCentreFrac = 1.5;
constexpr double kLabelTextHeightFrac = 1.2;

/// Enough for " | 12.34 m/s" and the terminator.
constexpr std::size_t kSpeedSuffixSize = 32;

visualization_msgs::msg::Marker makeMarker(const std_msgs::msg::Header& header, const std::string& ns, int id,
                                           int32_t type, const Rgba& color, const rclcpp::Duration& lifetime)
{
  visualization_msgs::msg::Marker marker;
  marker.header = header;
  marker.ns = ns;
  marker.id = id;
  marker.type = type;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.color.r = color.r;
  marker.color.g = color.g;
  marker.color.b = color.b;
  marker.color.a = color.a;
  marker.lifetime = lifetime;
  return marker;
}

/// Lays an ARROW marker out as start point plus displacement, sizing the shaft
/// and head off `radius` so arrows stay legible whatever the person scale is.
/// Returns false when the displacement is too short to draw.
bool setArrow(visualization_msgs::msg::Marker& marker, const Eigen::Vector3d& start,
              const Eigen::Vector3d& displacement, double radius)
{
  if (displacement.norm() < kMinArrowLength)
  {
    return false;
  }

  const Eigen::Vector3d tip = start + displacement;
  marker.points = { tf2::toMsg(start), tf2::toMsg(tip) };
  marker.scale.x = kShaftDiameterFrac * radius;
  marker.scale.y = kHeadDiameterFrac * radius;
  marker.scale.z = kHeadLengthFrac * radius;
  return true;
}

/// "person_3 | 1.24 m/s" — the person's identity plus the number you actually want to
/// sanity-check, without having to echo the topic alongside RViz.
std::string personLabel(const people_msgs::msg::Person& person, std::size_t index, double speed)
{
  std::array<char, kSpeedSuffixSize> buffer{};
  std::snprintf(buffer.data(), buffer.size(), " | %.2f m/s", speed);
  return (person.name.empty() ? "#" + std::to_string(index) : person.name) + buffer.data();
}

/// TF frame ids may not contain arbitrary characters, and a person's name comes
/// from an upstream tracker, so anything unsafe becomes an underscore.
std::string sanitizeFrameId(const std::string& name)
{
  std::string safe = name;
  std::replace_if(safe.begin(), safe.end(), [](unsigned char c) { return std::isalnum(c) == 0 && c != '_'; }, '_');
  return safe;
}
}  // namespace

namespace coresense_people_estimation
{

DebugVisualizer::DebugVisualizer(rclcpp::Node& node, bool gravity_aligned)
  : node_(node)
  , gravity_aligned_(gravity_aligned)
{
  // ── Parameters ────────────────────────────────────────────────────────────
  debug_ = node_.declare_parameter<bool>("debug", false);
  const std::string debug_topic = node_.declare_parameter<std::string>("debug_topic", "~/debug_markers");
  debug_person_frames_ = node_.declare_parameter<bool>("debug_person_frames", true);
  debug_show_raw_ = node_.declare_parameter<bool>("debug_show_raw", true);
  debug_velocity_scale_ = node_.declare_parameter<double>("debug_velocity_scale", 1.0);
  debug_person_radius_ = node_.declare_parameter<double>("debug_person_radius", 0.2);
  debug_marker_lifetime_ =
      rclcpp::Duration::from_seconds(node_.declare_parameter<double>("debug_marker_lifetime", 0.5));

  // Debug publishers are created only when asked for, so a production run costs
  // nothing beyond one branch per message.
  if (!debug_)
  {
    return;
  }

  debug_marker_pub_ = node_.create_publisher<visualization_msgs::msg::MarkerArray>(debug_topic, 10);
  if (debug_person_frames_)
  {
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);
  }

  RCLCPP_INFO(node_.get_logger(), "debug visualisation on | markers=%s | person TF frames=%s | raw input shown=%s",
              debug_marker_pub_->get_topic_name(), debug_person_frames_ ? "yes" : "no", debug_show_raw_ ? "yes" : "no");
}

void DebugVisualizer::publish(const people_msgs::msg::People& people, const people_msgs::msg::People& raw,
                              const std::optional<RobotArrow>& robot)
{
  if (!debug_)
  {
    return;
  }

  visualization_msgs::msg::MarkerArray array;
  // 3 markers per transformed person, 2 per raw one, plus the robot arrow.
  array.markers.reserve(5 * people.people.size() + 1);

  const int type_sphere = visualization_msgs::msg::Marker::SPHERE;
  const int type_arrow = visualization_msgs::msg::Marker::ARROW;
  const int type_text = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;

  std::size_t velocity_count = 0;
  std::size_t raw_velocity_count = 0;

  for (std::size_t i = 0; i < people.people.size(); ++i)
  {
    const auto& person = people.people[i];
    const int id = static_cast<int>(i);

    // Position.
    auto sphere = makeMarker(people.header, kNsPeople, id, type_sphere, kPersonColor, debug_marker_lifetime_);
    sphere.pose.position = person.position;
    sphere.scale.x = sphere.scale.y = sphere.scale.z = 2.0 * debug_person_radius_;
    array.markers.push_back(std::move(sphere));

    // Velocity, as a look-ahead arrow.
    Eigen::Vector3d position;
    tf2::fromMsg(person.position, position);
    Eigen::Vector3d velocity;
    tf2::fromMsg(person.velocity, velocity);
    auto arrow = makeMarker(people.header, kNsVelocity, id, type_arrow, kVelocityColor, debug_marker_lifetime_);
    if (setArrow(arrow, position, debug_velocity_scale_ * velocity, debug_person_radius_))
    {
      // Arrow ids must stay dense from 0 for the stale-marker sweep to work,
      // so a skipped (near-stationary) person does not burn an id.
      arrow.id = static_cast<int>(velocity_count++);
      array.markers.push_back(std::move(arrow));
    }

    // Label, floating above the sphere.
    auto label = makeMarker(people.header, kNsLabel, id, type_text, kLabelColor, debug_marker_lifetime_);
    label.pose.position = person.position;
    label.pose.position.z += kLabelHeightAboveCentreFrac * debug_person_radius_;
    label.scale.z = kLabelTextHeightFrac * debug_person_radius_;
    label.text = personLabel(person, i, velocity.norm());
    array.markers.push_back(std::move(label));
  }

  // The raw input, drawn in the sensor frame it arrived in. Seeing both at once
  // is the whole point: if the two clouds disagree about where a person *is*,
  // the TF chain is wrong; if they only disagree about velocity, the motion
  // compensation is.
  if (debug_show_raw_)
  {
    for (std::size_t i = 0; i < raw.people.size(); ++i)
    {
      const auto& person = raw.people[i];
      const int id = static_cast<int>(i);

      auto sphere = makeMarker(raw.header, kNsRawPeople, id, type_sphere, kRawPersonColor, debug_marker_lifetime_);
      sphere.pose.position = person.position;
      sphere.scale.x = sphere.scale.y = sphere.scale.z = 2.0 * debug_person_radius_;
      array.markers.push_back(std::move(sphere));

      Eigen::Vector3d position;
      tf2::fromMsg(person.position, position);
      Eigen::Vector3d velocity;
      tf2::fromMsg(person.velocity, velocity);
      auto arrow = makeMarker(raw.header, kNsRawVelocity, id, type_arrow, kRawVelocityColor, debug_marker_lifetime_);
      if (setArrow(arrow, position, debug_velocity_scale_ * velocity, debug_person_radius_))
      {
        arrow.id = static_cast<int>(raw_velocity_count++);
        array.markers.push_back(std::move(arrow));
      }
    }
  }

  // The robot's own velocity, drawn from the base origin. This is the vector
  // that was added to every person; an arrow pointing the wrong way here
  // explains every wrong person velocity at once.
  std::size_t robot_count = 0;
  if (robot.has_value())
  {
    auto arrow = makeMarker(people.header, kNsRobot, 0, type_arrow, kRobotColor, debug_marker_lifetime_);
    if (setArrow(arrow, robot->origin, debug_velocity_scale_ * robot->linear, debug_person_radius_))
    {
      array.markers.push_back(std::move(arrow));
      robot_count = 1;
    }
  }

  reapStaleMarkers(array, kNsPeople, people.people.size());
  reapStaleMarkers(array, kNsVelocity, velocity_count);
  reapStaleMarkers(array, kNsLabel, people.people.size());
  reapStaleMarkers(array, kNsRawPeople, debug_show_raw_ ? raw.people.size() : 0);
  reapStaleMarkers(array, kNsRawVelocity, raw_velocity_count);
  reapStaleMarkers(array, kNsRobot, robot_count);

  debug_marker_pub_->publish(array);

  publishPersonFrames(people);
}

void DebugVisualizer::reapStaleMarkers(visualization_msgs::msg::MarkerArray& array, const std::string& ns,
                                       std::size_t count)
{
  std::size_t& previous = debug_marker_counts_[ns];
  for (std::size_t i = count; i < previous; ++i)
  {
    visualization_msgs::msg::Marker deleter;
    deleter.ns = ns;
    deleter.id = static_cast<int>(i);
    deleter.action = visualization_msgs::msg::Marker::DELETE;
    array.markers.push_back(std::move(deleter));
  }
  previous = count;
}

void DebugVisualizer::publishPersonFrames(const people_msgs::msg::People& people)
{
  if (!tf_broadcaster_)
  {
    return;
  }

  std::vector<geometry_msgs::msg::TransformStamped> transforms;
  transforms.reserve(people.people.size());
  // Two tracks sharing a name would otherwise broadcast the same child frame
  // twice in one batch, which tf2 rejects with a TF_REPEATED_DATA warning.
  std::unordered_set<std::string> used_frames;

  for (std::size_t i = 0; i < people.people.size(); ++i)
  {
    const auto& person = people.people[i];

    // A tracker's name is the stable identity, so prefer it — the frame then
    // follows one person across messages instead of jumping with list order.
    std::string suffix = person.name.empty() ? "person_" + std::to_string(i) : sanitizeFrameId(person.name);
    if (!used_frames.insert(suffix).second)
    {
      suffix += "_" + std::to_string(i);
      used_frames.insert(suffix);
    }

    geometry_msgs::msg::TransformStamped tf;
    tf.header = people.header;
    tf.child_frame_id = suffix;
    tf.transform.translation.x = person.position.x;
    tf.transform.translation.y = person.position.y;
    tf.transform.translation.z = person.position.z;

    // Yaw the frame along the direction of travel so heading is readable from
    // the axes alone; a person who is not moving keeps the parent frame's
    // orientation rather than snapping to an arbitrary one. Which plane the
    // heading lives in follows the frame the people arrived in.
    const double forward = gravity_aligned_ ? person.velocity.x : person.velocity.z;
    const double sideways = gravity_aligned_ ? person.velocity.y : person.velocity.x;
    const double heading = std::hypot(forward, sideways) > kMinArrowLength ? std::atan2(sideways, forward) : 0.0;
    tf2::Quaternion q;
    if (gravity_aligned_)
    {
      q.setRPY(0.0, 0.0, heading);
    }
    else
    {
      q.setRPY(0.0, -heading, 0.0);
    }
    tf.transform.rotation = tf2::toMsg(q);

    transforms.push_back(std::move(tf));
  }

  if (!transforms.empty())
  {
    tf_broadcaster_->sendTransform(transforms);
  }
}

}  // namespace coresense_people_estimation
