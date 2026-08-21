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

#include "coresense_people_estimation/people_map_transform.hpp"

#include <memory>
#include <optional>
#include <string>

#include "coresense_people_estimation/parameters.hpp"
#include "tf2/LinearMath/Vector3.hpp"
#include "tf2/exceptions.hpp"
#include "tf2/time.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace coresense_people_estimation
{

PeopleMapTransform::PeopleMapTransform(rclcpp::Node& node)
  : node_(node)
{
  // ── Parameters ────────────────────────────────────────────────────────────
  // Empty is the escape hatch: no TF listener, no transform.
  target_frame_ = node_.declare_parameter<std::string>("target_frame", "map");
  robot_base_frame_ = node_.declare_parameter<std::string>("robot_base_frame", "base_link");

  transform_timeout_ = rclcpp::Duration::from_seconds(node_.declare_parameter<double>("transform_timeout", 0.1));
  allow_latest_transform_fallback_ = node_.declare_parameter<bool>("allow_latest_transform_fallback", true);
  // When false, a missing/stale cmd_vel is treated as "robot is stationary",
  // which is the right call for a robot that only publishes while moving.
  require_cmd_vel_ = node_.declare_parameter<bool>("require_cmd_vel", false);

  // ── TF ────────────────────────────────────────────────────────────────────
  if (!target_frame_.empty())
  {
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_.get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }

  if (target_frame_.empty())
  {
    RCLCPP_WARN(node_.get_logger(),
                "target_frame is empty: people are published in the frame they were measured in, "
                "with no motion compensation.");
  }
  else
  {
    RCLCPP_INFO(node_.get_logger(), "map transform on | target_frame=%s | base_frame=%s", target_frame_.c_str(),
                robot_base_frame_.c_str());
  }
}

bool PeopleMapTransform::transform(people_msgs::msg::People& people,
                                   const std::optional<geometry_msgs::msg::Twist>& cmd_vel,
                                   std::optional<RobotMotion>* motion_out)
{
  if (target_frame_.empty())
  {
    // Nothing is re-framed here, so what the caller appends afterwards is in the
    // same frame as everything else by construction — the sensor frame, which
    // moves with the robot. Coasting is only world-fixed when `target_frame` is
    // set; see the README.
    return true;
  }

  const std::string source_frame = people.header.frame_id;

  if (source_frame.empty())
  {
    RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), kWarnThrottleMs,
                         "People message has an empty frame_id; dropping.");
    return false;
  }

  const rclcpp::Time stamp(people.header.stamp, RCL_ROS_TIME);

  geometry_msgs::msg::TransformStamped source_to_target;
  if (!lookupTransform(target_frame_, source_frame, stamp, source_to_target))
  {
    return false;
  }

  const std::optional<RobotMotion> robot_motion = robotMotionInTargetFrame(cmd_vel, stamp);
  if (motion_out != nullptr)
  {
    *motion_out = robot_motion;
  }
  if (!robot_motion.has_value() && require_cmd_vel_)
  {
    RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), kWarnThrottleMs,
                         "require_cmd_vel is set but no usable robot velocity is available; dropping message.");
    return false;
  }
  // Default-initialised rather than brace-initialised: the members are messages
  // with explicit constructors, which an initializer list would go through.
  RobotMotion stationary;
  const RobotMotion motion = robot_motion.value_or(stationary);
  const tf2::Vector3 v_base(motion.linear.x, motion.linear.y, motion.linear.z);
  const tf2::Vector3 omega(motion.angular.x, motion.angular.y, motion.angular.z);
  const tf2::Vector3 base_origin(motion.base_origin.x, motion.base_origin.y, motion.base_origin.z);

  people.header.frame_id = target_frame_;

  for (auto& person : people.people)
  {
    // Position: full rigid-body transform (rotation + translation).
    geometry_msgs::msg::Point position_in_target;
    tf2::doTransform(person.position, position_in_target, source_to_target);

    // Velocity: rotation only — a velocity is a free vector, so the
    // translation part of the transform must not be applied. The result is
    // still relative to the (moving) robot at this point.
    geometry_msgs::msg::Vector3 v_relative;
    v_relative.x = person.velocity.x;
    v_relative.y = person.velocity.y;
    v_relative.z = person.velocity.z;

    geometry_msgs::msg::Vector3 v_rotated;
    tf2::doTransform(v_relative, v_rotated, source_to_target);

    // Transport theorem: a velocity measured in a frame that both translates
    // and rotates needs both of the robot's motion terms added back.
    //
    //   v_world = v_relative + v_base + omega x (p_person - p_base_origin)
    //
    // The lever arm runs from the *base* origin, not the camera origin: the
    // camera's own lever arm cancels between its linear and angular
    // contributions, since the camera is rigidly attached to the base.
    const tf2::Vector3 lever(position_in_target.x - base_origin.x(), position_in_target.y - base_origin.y(),
                             position_in_target.z - base_origin.z());
    const tf2::Vector3 v_world = tf2::Vector3(v_rotated.x, v_rotated.y, v_rotated.z) + v_base + omega.cross(lever);

    // name, reliability, tagnames and tags carry through untouched; the
    // covariance tags stay in the frame they were measured in. See the README.
    person.position = position_in_target;
    person.velocity.x = v_world.x();
    person.velocity.y = v_world.y();
    person.velocity.z = v_world.z();
  }

  return true;
}

bool PeopleMapTransform::lookupFromTarget(const std::string& frame, const rclcpp::Time& stamp,
                                          geometry_msgs::msg::TransformStamped& transform)
{
  if (target_frame_.empty())
  {
    transform = geometry_msgs::msg::TransformStamped();
    // Default-constructs to an all-zero quaternion, which is not a rotation.
    transform.transform.rotation.w = 1.0;
    return true;
  }
  return lookupTransform(frame, target_frame_, stamp, transform);
}

bool PeopleMapTransform::lookupTransform(const std::string& target_frame, const std::string& source_frame,
                                         const rclcpp::Time& stamp, geometry_msgs::msg::TransformStamped& transform)
{
  try
  {
    transform = tf_buffer_->lookupTransform(target_frame, source_frame, stamp, transform_timeout_);
    return true;
  }
  catch (const tf2::TransformException& ex)
  {
    if (!allow_latest_transform_fallback_)
    {
      RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), kWarnThrottleMs,
                           "Could not transform %s -> %s at the message stamp: %s", source_frame.c_str(),
                           target_frame.c_str(), ex.what());
      return false;
    }
  }

  // The stamped lookup failed (usually extrapolation into the future on a
  // slow TF tree). Fall back to the most recent transform available; for a
  // slowly-moving base the error this introduces is small compared with
  // dropping the detection outright.
  try
  {
    transform = tf_buffer_->lookupTransform(target_frame, source_frame, tf2::TimePointZero);
    RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), kWarnThrottleMs,
                         "No %s -> %s transform at the message stamp; using the latest available one.",
                         source_frame.c_str(), target_frame.c_str());
    return true;
  }
  catch (const tf2::TransformException& ex)
  {
    RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), kWarnThrottleMs, "Could not transform %s -> %s: %s",
                         source_frame.c_str(), target_frame.c_str(), ex.what());
    return false;
  }
}

std::optional<PeopleMapTransform::RobotMotion> PeopleMapTransform::robotMotionInTargetFrame(
    const std::optional<geometry_msgs::msg::Twist>& cmd_vel, const rclcpp::Time& stamp)
{
  if (!cmd_vel.has_value())
  {
    RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), kWarnThrottleMs,
                         "No fresh cmd_vel available; assuming the robot is stationary.");
    return std::nullopt;
  }

  // cmd_vel is expressed in the robot base frame, so both halves of the twist
  // have to be rotated into the target frame before they can be combined with
  // a person's velocity.
  geometry_msgs::msg::TransformStamped base_to_target;
  if (!lookupTransform(target_frame_, robot_base_frame_, stamp, base_to_target))
  {
    return std::nullopt;
  }

  RobotMotion motion;
  tf2::doTransform(cmd_vel->linear, motion.linear, base_to_target);
  tf2::doTransform(cmd_vel->angular, motion.angular, base_to_target);
  // The translation of base -> target *is* the base origin in target coords.
  motion.base_origin = base_to_target.transform.translation;
  return motion;
}

}  // namespace coresense_people_estimation
