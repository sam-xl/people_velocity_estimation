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

#include <array>
#include <cstddef>
#include <memory>
#include <mutex>
#include <opencv2/core.hpp>
#include <optional>

#include "coresense_people_estimation/debug_visualizer.hpp"
#include "coresense_people_estimation/depth_reader.hpp"
#include "coresense_people_estimation/person_tracker.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "image_geometry/pinhole_camera_model.hpp"
#include "message_filters/subscriber.hpp"
#include "message_filters/sync_policies/approximate_time.hpp"
#include "message_filters/synchronizer.hpp"
#include "people_msgs/msg/people.hpp"
#include "people_msgs/msg/person.hpp"
#include "rclcpp/rclcpp.hpp"
#include "riskam_msgs/msg/person_detection.hpp"
#include "riskam_msgs/msg/person_detections.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace coresense_people_estimation
{

/// Turns 2-D person detections into robot-relative `people_msgs/People`.
///
/// This class is the plumbing: it pairs detections with depth, decodes the depth
/// image, latches `camera_info` and `cmd_vel`, and assembles the output message.
/// The metric work is delegated, one collaborator per README section — see
/// `DepthReader`, `PersonTracker` and `DebugVisualizer`.
///
/// Positions and velocities are in the **colour optical frame** and relative to
/// the robot: nothing here re-frames them or takes the robot's own motion out.
class PeopleEstimationNode : public rclcpp::Node
{
public:
  explicit PeopleEstimationNode(const rclcpp::NodeOptions& options);

private:
  typedef message_filters::sync_policies::ApproximateTime<riskam_msgs::msg::PersonDetections, sensor_msgs::msg::Image>
      SyncPolicy;

  // ── Callbacks ─────────────────────────────────────────────────────────────

  void detectionsCallback(const riskam_msgs::msg::PersonDetections::ConstSharedPtr& detections,
                          const sensor_msgs::msg::Image::ConstSharedPtr& depth_image);
  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg);
  void cmdVelCallback(const geometry_msgs::msg::Twist::ConstSharedPtr& msg);

  /// The latched `cmd_vel`, or `std::nullopt` when it is missing or older than
  /// `cmd_vel_timeout_`. Dead-reckons the camera yaw the velocity window is
  /// judged in.
  std::optional<geometry_msgs::msg::Twist> latestCmdVel();

  // ── Per-detection pipeline ────────────────────────────────────────────────

  /// Depth in metres from the incoming image, or `std::nullopt` when the encoding
  /// is unsupported or `cv_bridge` refuses it. Warns, throttled, on the way out.
  std::optional<cv::Mat> decodeDepth(const sensor_msgs::msg::Image::ConstSharedPtr& depth_image);

  /// Per-axis position variance of one back-projected detection, in m^2.
  ///
  /// The diagonal of the Jacobian propagation of `P = ray * z`: the anchor pixel
  /// is uncertain by a fraction of the bounding box, and the depth by the larger
  /// of its measured spread and the sensor noise floor. Only the velocity
  /// window's consistency test reads it, and that test is per axis, so the
  /// cross terms are not carried. See the README.
  std::array<double, 3> positionVariance(const cv::Point3d& ray, const DepthEstimate& depth, double bbox_width,
                                         double bbox_height, double fx, double fy) const;

  /// One detection, all the way to a `Person` — or `std::nullopt` when the depth
  /// is unusable. Mutates the tracker.
  std::optional<people_msgs::msg::Person> processDetection(const cv::Mat& depth_m,
                                                           const riskam_msgs::msg::PersonDetection& detection,
                                                           double stamp);

  // ── Subscriptions and publishers ──────────────────────────────────────────

  message_filters::Subscriber<riskam_msgs::msg::PersonDetections> detections_sub_;
  message_filters::Subscriber<sensor_msgs::msg::Image> depth_sub_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<people_msgs::msg::People>::SharedPtr people_pub_;

  // ── Collaborators ─────────────────────────────────────────────────────────
  // Each declares its own parameters on this node, so all of them are constructed
  // after the node's own declarations.

  std::unique_ptr<DepthReader> depth_reader_;
  std::unique_ptr<PersonTracker> tracker_;
  /// Runs last, on the message that is about to be published.
  std::unique_ptr<DebugVisualizer> debug_;

  // ── State ─────────────────────────────────────────────────────────────────

  std::mutex camera_info_mutex_;
  image_geometry::PinholeCameraModel camera_model_;
  bool camera_info_received_{ false };

  std::mutex cmd_vel_mutex_;
  geometry_msgs::msg::Twist cmd_vel_;
  rclcpp::Time cmd_vel_stamp_;
  bool cmd_vel_received_{ false };
  /// Camera yaw dead-reckoned from cmd_vel; only differences over a window are used.
  double cumulative_yaw_{ 0.0 };
  /// Empty until the first detection message, which has no interval to integrate over.
  std::optional<double> previous_detection_stamp_;

  /// Names untracked detections apart within one message: ByteTrack reports every
  /// one of them as -1. Reset per message, so the set of names — and the debug TF
  /// frames named after them — stays bounded by how many people are seen at once.
  std::size_t untracked_in_message_{ 0 };

  // ── Parameters ────────────────────────────────────────────────────────────
  // Only the ones this class uses itself.

  double depth_min_m_;
  double depth_max_m_;
  rclcpp::Duration cmd_vel_timeout_{ 0, 0 };
  bool publish_covariance_tags_;

  /// Pixel sigma of the anchor point, as a fraction of the bounding box size.
  double pixel_sigma_frac_;
  double depth_spread_gain_;
  /// Quadratic coefficient of the RealSense depth noise model, sigma_z ~ c*z^2.
  /// https://dev.intelrealsense.com/docs/tuning-depth-cameras-for-best-performance
  double depth_noise_coeff_;
};

}  // namespace coresense_people_estimation
