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

#include <algorithm>
#include <cctype>
#include <cstddef>
#include <memory>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "people_msgs/msg/people.hpp"
#include "people_msgs/msg/person.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "tf2_ros/transform_broadcaster.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace
{
/// Marker namespaces. Each is independently toggleable in RViz, so keeping
/// positions and labels apart is worth the extra ids.
constexpr char kNsPeople[] = "people";
constexpr char kNsLabel[] = "label";

struct Rgba
{
  float r, g, b, a;
};

constexpr Rgba kPersonColor{ 0.20F, 0.85F, 0.35F, 0.85F };
constexpr Rgba kLabelColor{ 1.00F, 1.00F, 1.00F, 0.90F };

/// Label placement and size as fractions of the person radius, so the text clears
/// the sphere and stays proportionate whatever `debug_person_radius` is.
constexpr double kLabelHeightAboveCentreFrac = 1.5;
constexpr double kLabelTextHeightFrac = 1.2;

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

/// "person_3" — the person's identity, without having to echo the topic alongside RViz.
std::string personLabel(const people_msgs::msg::Person& person, std::size_t index)
{
  return person.name.empty() ? "#" + std::to_string(index) : person.name;
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

DebugVisualizer::DebugVisualizer(rclcpp::Node& node)
  : node_(node)
{
  // ── Parameters ────────────────────────────────────────────────────────────
  debug_ = node_.declare_parameter<bool>("debug", false);
  const std::string debug_topic = node_.declare_parameter<std::string>("debug_topic", "~/debug_markers");
  debug_person_frames_ = node_.declare_parameter<bool>("debug_person_frames", true);
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

  RCLCPP_INFO(node_.get_logger(), "debug visualisation on | markers=%s | person TF frames=%s",
              debug_marker_pub_->get_topic_name(), debug_person_frames_ ? "yes" : "no");
}

void DebugVisualizer::publish(const people_msgs::msg::People& people)
{
  if (!debug_)
  {
    return;
  }

  visualization_msgs::msg::MarkerArray array;
  // A sphere and a label per person.
  array.markers.reserve(2 * people.people.size());

  const int type_sphere = visualization_msgs::msg::Marker::SPHERE;
  const int type_text = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;

  for (std::size_t i = 0; i < people.people.size(); ++i)
  {
    const auto& person = people.people[i];
    const int id = static_cast<int>(i);

    // Position.
    auto sphere = makeMarker(people.header, kNsPeople, id, type_sphere, kPersonColor, debug_marker_lifetime_);
    sphere.pose.position = person.position;
    sphere.scale.x = sphere.scale.y = sphere.scale.z = 2.0 * debug_person_radius_;
    array.markers.push_back(std::move(sphere));

    // Label, floating above the sphere.
    auto label = makeMarker(people.header, kNsLabel, id, type_text, kLabelColor, debug_marker_lifetime_);
    label.pose.position = person.position;
    label.pose.position.z += kLabelHeightAboveCentreFrac * debug_person_radius_;
    label.scale.z = kLabelTextHeightFrac * debug_person_radius_;
    label.text = personLabel(person, i);
    array.markers.push_back(std::move(label));
  }

  reapStaleMarkers(array, kNsPeople, people.people.size());
  reapStaleMarkers(array, kNsLabel, people.people.size());

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

    // The frame keeps the parent's orientation: a position carries no heading
    // to align it to.
    tf.transform.rotation.w = 1.0;

    transforms.push_back(std::move(tf));
  }

  if (!transforms.empty())
  {
    tf_broadcaster_->sendTransform(transforms);
  }
}

}  // namespace coresense_people_estimation
