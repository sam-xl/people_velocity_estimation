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

#include "coresense_people_estimation/people_estimation_node.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <opencv2/core.hpp>
#include <optional>
#include <sstream>
#include <string>
#include <utility>

#include "coresense_people_estimation/debug_visualizer.hpp"
#include "coresense_people_estimation/depth_reader.hpp"
#include "coresense_people_estimation/keypoints.hpp"
#include "coresense_people_estimation/person_tracker.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "message_filters/synchronizer.hpp"
#include "people_msgs/msg/people.hpp"
#include "people_msgs/msg/person.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "riskam_msgs/msg/person_detection.hpp"
#include "riskam_msgs/msg/person_detections.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace
{
/// Detections arrive at camera rate, so missing-input warnings must be throttled.
constexpr int kWarnThrottleMs = 5000;

std::string formatCovariance(const std::array<double, 9>& covariance)
{
  std::ostringstream out;
  for (std::size_t i = 0; i < covariance.size(); ++i)
  {
    if (i != 0)
    {
      out << ' ';
    }
    out << covariance[i];
  }
  return out.str();
}
}  // namespace

namespace coresense_people_estimation
{

PeopleEstimationNode::PeopleEstimationNode(const rclcpp::NodeOptions& options)
  : rclcpp::Node("people_estimation", options)
  , cmd_vel_stamp_(0, 0, RCL_ROS_TIME)
{
  // ── Topics ────────────────────────────────────────────────────────────────
  const std::string detections_topic = declare_parameter<std::string>("detections_topic", "/riskam/person_detections");
  // Must be registered to the colour frame: the boxes and keypoints are
  // colour-image pixels. See the README.
  const std::string depth_topic =
      declare_parameter<std::string>("depth_topic", "/camera/camera/aligned_depth_to_color/image_raw");
  const std::string camera_info_topic =
      declare_parameter<std::string>("camera_info_topic", "/camera/camera/color/camera_info");
  const std::string cmd_vel_topic = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
  const std::string people_topic = declare_parameter<std::string>("people_topic", "/people");

  const auto sync_queue_size = static_cast<int>(declare_parameter<std::int64_t>("sync_queue_size", 10));
  const double sync_slop = declare_parameter<double>("sync_slop", 0.1);

  // ── Depth read-out ────────────────────────────────────────────────────────
  // The valid-depth band is declared here rather than in `DepthReader`, because
  // both the read-out and the range check in processDetection() judge against it.
  depth_min_m_ = declare_parameter<double>("depth_min_m", 0.1);
  depth_max_m_ = declare_parameter<double>("depth_max_m", 3.0);

  // ── Position noise ────────────────────────────────────────────────────────
  // Feeds the velocity window's consistency test; see positionVariance().
  pixel_sigma_frac_ = declare_parameter<double>("pixel_sigma_frac", 0.25);
  depth_spread_gain_ = declare_parameter<double>("depth_spread_gain", 1.0);
  depth_noise_coeff_ = declare_parameter<double>("depth_noise_coeff", 0.0014);

  cmd_vel_timeout_ = rclcpp::Duration::from_seconds(declare_parameter<double>("cmd_vel_timeout", 0.5));

  // ── Output ────────────────────────────────────────────────────────────────
  publish_covariance_tags_ = declare_parameter<bool>("publish_covariance_tags", false);

  // ── Collaborators ─────────────────────────────────────────────────────────
  // Each declares its own parameters on this node, so they come after the block
  // above. One per README section; see the class comment.
  depth_reader_ = std::make_unique<DepthReader>(*this, depth_min_m_, depth_max_m_);
  tracker_ = std::make_unique<PersonTracker>(*this);
  debug_ = std::make_unique<DebugVisualizer>(*this);

  // ── Wiring ────────────────────────────────────────────────────────────────
  // This message_filters release only exposes the rmw_qos_profile_t overloads
  // of subscribe().
  rmw_qos_profile_t qos_profile = rmw_qos_profile_default;
  qos_profile.depth = static_cast<std::size_t>(sync_queue_size);

  detections_sub_.subscribe(this, detections_topic, qos_profile);
  depth_sub_.subscribe(this, depth_topic, qos_profile);
  sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(sync_queue_size), detections_sub_,
                                                                      depth_sub_);
  sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(sync_slop));
  // The member-function overload: Synchronizer binds any other callable to all
  // nine of Signal9's slots.
  sync_->registerCallback(&PeopleEstimationNode::detectionsCallback, this);

  camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic, 10,
      [this](const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg) { cameraInfoCallback(msg); });
  cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic, 10, [this](const geometry_msgs::msg::Twist::ConstSharedPtr& msg) { cmdVelCallback(msg); });

  people_pub_ = create_publisher<people_msgs::msg::People>(people_topic, 10);

  RCLCPP_INFO(get_logger(), "people_estimation started | detections=%s | depth=%s | camera_info=%s | cmd_vel=%s -> %s",
              detections_topic.c_str(), depth_topic.c_str(), camera_info_topic.c_str(), cmd_vel_topic.c_str(),
              people_topic.c_str());
}

// ── Callbacks ───────────────────────────────────────────────────────────────

void PeopleEstimationNode::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg)
{
  // p[0] and p[5] are fx and fy. fromCameraInfo reports "parameters changed",
  // not "parameters valid", so an uncalibrated message would latch and
  // back-project every pixel to infinity.
  if (msg->p[0] == 0.0 || msg->p[5] == 0.0)
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), kWarnThrottleMs,
                         "camera_info carries no focal length; ignoring it.");
    return;
  }

  const std::lock_guard<std::mutex> lock(camera_info_mutex_);
  camera_model_.fromCameraInfo(*msg);
  camera_info_received_ = true;
}

void PeopleEstimationNode::cmdVelCallback(const geometry_msgs::msg::Twist::ConstSharedPtr& msg)
{
  const std::lock_guard<std::mutex> lock(cmd_vel_mutex_);
  cmd_vel_ = *msg;
  // Twist is unstamped, so arrival time is the best stamp available.
  cmd_vel_stamp_ = now();
  cmd_vel_received_ = true;
}

std::optional<geometry_msgs::msg::Twist> PeopleEstimationNode::latestCmdVel()
{
  const std::lock_guard<std::mutex> lock(cmd_vel_mutex_);
  if (!cmd_vel_received_)
  {
    return std::nullopt;
  }
  // 0 disables the staleness check, for bags with sparse cmd_vel.
  if (cmd_vel_timeout_ > rclcpp::Duration(0, 0) && (now() - cmd_vel_stamp_) > cmd_vel_timeout_)
  {
    return std::nullopt;
  }
  return cmd_vel_;
}

void PeopleEstimationNode::detectionsCallback(const riskam_msgs::msg::PersonDetections::ConstSharedPtr& detections,
                                              const sensor_msgs::msg::Image::ConstSharedPtr& depth_image)
{
  {
    const std::lock_guard<std::mutex> lock(camera_info_mutex_);
    if (!camera_info_received_)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), kWarnThrottleMs,
                           "No camera_info received yet; cannot back-project detections.");
      return;
    }
  }

  // The message outlives this call, so the decoded Mat may alias its buffer.
  const std::optional<cv::Mat> depth_m = decodeDepth(depth_image);
  if (!depth_m.has_value())
  {
    return;
  }

  const double stamp = rclcpp::Time(detections->header.stamp).seconds();

  // ── Integrate the camera's own yaw ────────────────────────────────────────
  // The only thing cmd_vel is used for here: a robot turn bends a stationary
  // person's track, which would otherwise collapse every velocity window during
  // every turn. See the README.
  const std::optional<geometry_msgs::msg::Twist> cmd_vel = latestCmdVel();
  const double omega_z = cmd_vel.has_value() ? cmd_vel->angular.z : 0.0;
  if (previous_detection_stamp_.has_value() && stamp > *previous_detection_stamp_)
  {
    cumulative_yaw_ += omega_z * (stamp - *previous_detection_stamp_);
  }
  previous_detection_stamp_ = stamp;

  // ── Per-detection processing ──────────────────────────────────────────────
  people_msgs::msg::People people;
  people.header = detections->header;

  // Per message, not per process: an ever-growing suffix would name a new debug
  // TF frame every frame, and tf2 never retires a frame id.
  untracked_in_message_ = 0;

  for (const auto& detection : detections->detections)
  {
    std::optional<people_msgs::msg::Person> person = processDetection(*depth_m, detection, stamp);
    if (person.has_value())
    {
      people.people.push_back(std::move(*person));
    }
  }

  tracker_->dropStale(stamp);

  people_pub_->publish(people);

  // Draws exactly what was published, in the frame it was published in.
  debug_->publish(people);
}

// ── Per-detection pipeline ──────────────────────────────────────────────────

std::optional<cv::Mat> PeopleEstimationNode::decodeDepth(const sensor_msgs::msg::Image::ConstSharedPtr& depth_image)
{
  cv::Mat depth_m;
  try
  {
    const auto cv_depth = cv_bridge::toCvShare(depth_image);
    if (depth_image->encoding == sensor_msgs::image_encodings::TYPE_16UC1)
    {
      // uint16 millimetres; the 0 missing-data sentinel scales to 0, below depth_min_m_.
      cv_depth->image.convertTo(depth_m, CV_32F, 1.0 / 1000.0);
    }
    else if (depth_image->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
    {
      depth_m = cv_depth->image;
    }
    else
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), kWarnThrottleMs,
                           "Unsupported depth encoding '%s'; expected 16UC1 or 32FC1.", depth_image->encoding.c_str());
      return std::nullopt;
    }
  }
  catch (const cv_bridge::Exception& e)
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), kWarnThrottleMs, "cv_bridge failed on the depth image: %s",
                         e.what());
    return std::nullopt;
  }
  return depth_m;
}

std::array<double, 3> PeopleEstimationNode::positionVariance(const cv::Point3d& ray, const DepthEstimate& depth,
                                                             double bbox_width, double bbox_height, double fx,
                                                             double fy) const
{
  const double sigma_u = pixel_sigma_frac_ * bbox_width;
  const double sigma_v = pixel_sigma_frac_ * bbox_height;
  const double noise_floor = depth_noise_coeff_ * depth.depth_m * depth.depth_m;
  // On the whole-bbox fallback the spread is the gap to the background, which
  // says nothing about how well this person's distance is known.
  const double sigma_z =
      depth.from_bbox_fallback ? noise_floor : std::max(depth_spread_gain_ * depth.spread_m, noise_floor);

  // P = ray * z, so the depth term lands along the ray and the pixel terms
  // across it. Diagonal only: the window test reads one axis at a time.
  const double du = depth.depth_m / fx * sigma_u;
  const double dv = depth.depth_m / fy * sigma_v;
  const double depth_variance = sigma_z * sigma_z;
  return { { du * du + depth_variance * ray.x * ray.x, dv * dv + depth_variance * ray.y * ray.y,
             depth_variance * ray.z * ray.z } };
}

std::optional<people_msgs::msg::Person> PeopleEstimationNode::processDetection(
    const cv::Mat& depth_m, const riskam_msgs::msg::PersonDetection& detection, double stamp)
{
  const DepthEstimate depth = depth_reader_->read(depth_m, detection);

  // No valid reading means no measurement to publish.
  if (depth.valid_frac <= 0.0 || depth.depth_m < depth_min_m_ || depth.depth_m > depth_max_m_)
  {
    return std::nullopt;
  }

  const cv::Point2d anchor = anchorPixel(detection);
  cv::Point3d ray;
  double fx = 0.0;
  double fy = 0.0;
  {
    const std::lock_guard<std::mutex> lock(camera_info_mutex_);
    ray = camera_model_.projectPixelTo3dRay(anchor);
    fx = camera_model_.fx();
    fy = camera_model_.fy();
  }

  const std::array<double, 3> position{ { ray.x * depth.depth_m, ray.y * depth.depth_m, ray.z * depth.depth_m } };

  const double bbox_width = std::abs(static_cast<double>(detection.bbox[2] - detection.bbox[0]));
  const double bbox_height = std::abs(static_cast<double>(detection.bbox[3] - detection.bbox[1]));
  const std::array<double, 3> variance = positionVariance(ray, depth, bbox_width, bbox_height, fx, fy);

  // ByteTrack reports every unassociated detection as -1; those have no identity
  // to hold a history against, so they pass through as measured, with the velocity
  // reported unknown rather than zero.
  const TrackUpdate update = detection.track_id >= 0 ?
                                 tracker_->update(detection.track_id, stamp, position, variance, cumulative_yaw_) :
                                 TrackUpdate{ position, tracker_->priorFit(), 0.0 };

  // ── Assemble the Person ───────────────────────────────────────────────────
  people_msgs::msg::Person person;
  person.name = detection.track_id >= 0 ? "person_" + std::to_string(detection.track_id) :
                                          "person_untracked_" + std::to_string(untracked_in_message_++);
  person.position.x = update.position[0];
  person.position.y = update.position[1];
  person.position.z = update.position[2];
  person.velocity.x = update.fit.velocity[0];
  person.velocity.y = update.fit.velocity[1];
  person.velocity.z = update.fit.velocity[2];
  // Every detection that got this far is equally believed: there is no model yet
  // of how confident this measurement is.
  person.reliability = 1.0;

  // people_msgs/Person has no field for the depth diagnostics, so tagnames/tags
  // carry them. Always published: dropping a person is diagnosed from these.
  person.tagnames = { "depth_valid_frac", "depth_spread_m" };
  person.tags = { std::to_string(depth.valid_frac), std::to_string(depth.spread_m) };

  if (publish_covariance_tags_)
  {
    // The velocity numbers ride along the same way, in this message's frame.
    // See the README.
    const std::array<double, 9> velocity_covariance{ { update.fit.variance[0], 0.0, 0.0, 0.0, update.fit.variance[1],
                                                       0.0, 0.0, 0.0, update.fit.variance[2] } };
    person.tagnames.insert(person.tagnames.end(), { "velocity_covariance", "n_history_samples", "velocity_window_s" });
    person.tags.insert(person.tags.end(), { formatCovariance(velocity_covariance), std::to_string(update.fit.n_samples),
                                            std::to_string(update.window_s) });
  }

  return person;
}

}  // namespace coresense_people_estimation

RCLCPP_COMPONENTS_REGISTER_NODE(coresense_people_estimation::PeopleEstimationNode)
