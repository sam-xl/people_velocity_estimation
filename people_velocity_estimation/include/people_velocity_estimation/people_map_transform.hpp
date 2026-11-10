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
#include <memory>
#include <optional>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "people_msgs/msg/people.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"

namespace people_velocity_estimation
{

/// Re-frames `people_msgs/People` from the sensor frame into a world-fixed frame.
///
/// Positions go through the full TF transform; velocities are only rotated, then
/// corrected for the robot's own motion. See docs/map_transform.md.
///
/// Not a node: it declares its parameters on the owning node and logs through
/// it, and takes `cmd_vel` as an argument since that node already latches it.
class PeopleMapTransform
{
public:
  /// The robot's rigid-body motion, expressed in the target frame.
  struct RobotMotion
  {
    /// Velocity of the robot base frame's origin.
    Eigen::Vector3d linear{ Eigen::Vector3d::Zero() };
    /// Angular velocity of the robot base.
    Eigen::Vector3d angular{ Eigen::Vector3d::Zero() };
    /// Position of the robot base frame's origin — the point the angular
    /// velocity turns about, and hence the origin of the lever arm.
    Eigen::Vector3d base_origin{ Eigen::Vector3d::Zero() };
  };

  explicit PeopleMapTransform(rclcpp::Node& node);

  /// Re-frames `people` in place. False means the message should be dropped;
  /// the warning is already logged. A `std::nullopt` `cmd_vel` is fatal only
  /// under `require_cmd_vel`.
  ///
  /// Only what the sensor measured this frame belongs here. Anything already in
  /// `target_frame` — a coasted person — is appended by the caller afterwards,
  /// because transforming it again would apply the robot's pose a second time.
  /// See `TrackCoaster`.
  ///
  /// `motion_out`, when given, receives the robot motion that was subtracted —
  /// the one number that explains every velocity in the output at once, which
  /// is why `DebugVisualizer` draws it.
  bool transform(people_msgs::msg::People& people, const std::optional<geometry_msgs::msg::Twist>& cmd_vel,
                 std::optional<RobotMotion>* motion_out = nullptr);

  /// The reverse of what `transform()` applies: `target_frame` -> `frame`, at `stamp`.
  /// Identity when `target_frame` is empty; false (already warned) when TF cannot answer.
  ///
  /// Public so callers reuse this class's TF buffer rather than adding a second listener.
  /// The frame published people end up in. Empty when the transform is off and
  /// they stay in the sensor's optical frame.
  const std::string& targetFrame() const
  {
    return target_frame_;
  }

  bool lookupFromTarget(const std::string& frame, const rclcpp::Time& stamp,
                        geometry_msgs::msg::TransformStamped& transform);

private:
  /// Looks up `source` -> `target`, falling back to the latest available
  /// transform when `allow_latest_transform_fallback` is set. Returns false
  /// (after a throttled warning) when no usable transform exists.
  bool lookupTransform(const std::string& target_frame, const std::string& source_frame, const rclcpp::Time& stamp,
                       geometry_msgs::msg::TransformStamped& transform);

  /// `cmd_vel`, rotated into the target frame and paired with the base origin it
  /// acts about. `std::nullopt` means no usable command; the caller decides
  /// whether that is fatal.
  std::optional<RobotMotion> robotMotionInTargetFrame(const std::optional<geometry_msgs::msg::Twist>& cmd_vel,
                                                      const rclcpp::Time& stamp);

  /// Clock, logger and parameters all come from it.
  rclcpp::Node& node_;

  // ── Parameters ────────────────────────────────────────────────────────────
  /// Empty disables the transform: the message is left in its incoming frame.
  std::string target_frame_;
  std::string robot_base_frame_;
  /// `rclcpp::Duration` has no public default constructor, so unlike the
  /// parameters above this one cannot be left for the constructor to fill.
  /// The `declare_parameter` call carries the real default.
  rclcpp::Duration transform_timeout_{ 0, 0 };
  bool allow_latest_transform_fallback_;
  bool require_cmd_vel_;

  // ── TF ────────────────────────────────────────────────────────────────────
  /// Null when `target_frame` is empty.
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace people_velocity_estimation
