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

#include "people_velocity_estimation/people_estimation_node.hpp"

#include <Eigen/Core>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <opencv2/core.hpp>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "cv_bridge/cv_bridge.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "message_filters/synchronizer.hpp"
#include "people_msgs/msg/people.hpp"
#include "people_msgs/msg/person.hpp"
#include "people_velocity_estimation/debug_visualizer.hpp"
#include "people_velocity_estimation/depth_reader.hpp"
#include "people_velocity_estimation/keypoints.hpp"
#include "people_velocity_estimation/parameters.hpp"
#include "people_velocity_estimation/people_map_transform.hpp"
#include "people_velocity_estimation/person_tracker.hpp"
#include "people_velocity_estimation/plausibility_filter.hpp"
#include "people_velocity_estimation/track_coaster.hpp"
#include "people_velocity_estimation/uncertainty_model.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "riskam_msgs/msg/person_detection.hpp"
#include "riskam_msgs/msg/person_detections.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "tf2_eigen/tf2_eigen.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace
{
std::string formatCovariance(const Eigen::Matrix3d& covariance)
{
  static const Eigen::IOFormat kRowMajorSpaced(Eigen::StreamPrecision, Eigen::DontAlignCols, " ", " ");
  std::ostringstream out;
  out << covariance.format(kRowMajorSpaced);
  return out.str();
}
}  // namespace

namespace people_velocity_estimation
{

PeopleEstimationNode::PeopleEstimationNode(const rclcpp::NodeOptions& options)
  : rclcpp::Node("people_estimation", options)
  , cmd_vel_stamp_(0, 0, RCL_ROS_TIME)
{
  // ── Topics ────────────────────────────────────────────────────────────────
  const std::string detections_topic = declare_parameter<std::string>("detections_topic", "/riskam/person_detections");
  // Must be registered to the colour frame: the boxes and keypoints are
  // colour-image pixels. See docs/plausibility.md.
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
  // three places judge against it: the read-out, the plausibility filter, and
  // the range check in processDetection() below.
  depth_min_m_ = declare_parameter<double>("depth_min_m", 0.1);
  depth_max_m_ = declare_parameter<double>("depth_max_m", 3.0);

  cmd_vel_timeout_ = rclcpp::Duration::from_seconds(declare_parameter<double>("cmd_vel_timeout", 0.5));

  coast_in_view_margin_m_ = declare_parameter<double>("coast_in_view_margin_m", 0.25);

  // ── Output ────────────────────────────────────────────────────────────────
  publish_covariance_tags_ = declare_parameter<bool>("publish_covariance_tags", false);

  // ── Collaborators ─────────────────────────────────────────────────────────
  // Each declares its own parameters on this node, so they come after the block
  // above. One per stage in docs/; see the class comment.
  confirmer_ = std::make_unique<ConfirmationFilter>(*this);
  depth_reader_ = std::make_unique<DepthReader>(*this, depth_min_m_, depth_max_m_);
  plausibility_ = std::make_unique<PlausibilityFilter>(*this, depth_min_m_);
  uncertainty_ = std::make_unique<UncertaintyModel>(*this);
  tracker_ = std::make_unique<PersonTracker>(*this);
  map_transform_ = std::make_unique<PeopleMapTransform>(*this);
  // After the map transform, whose target_frame decides whether what this
  // extrapolates in is world-fixed or not.
  coaster_ = std::make_unique<TrackCoaster>(*this);
  debug_ = std::make_unique<DebugVisualizer>(*this, !map_transform_->targetFrame().empty());

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
  // Read once, reused by the map transform below, so both see the same command.
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

  // Parallel to people.people, so the coaster can key what was published on the
  // ByteTrack id behind it. A detection the pipeline rejected adds to neither.
  std::vector<std::int32_t> track_ids;
  track_ids.reserve(detections->detections.size());

  // Per message, not per process: an ever-growing suffix would name a new debug
  // TF frame every frame, and tf2 never retires a frame id.
  untracked_in_message_ = 0;

  for (const auto& detection : detections->detections)
  {
    std::optional<people_msgs::msg::Person> person = processDetection(*depth_m, detection, stamp);
    if (person.has_value())
    {
      people.people.push_back(std::move(*person));
      track_ids.push_back(detection.track_id);
    }
  }

  confirmer_->dropStale(stamp);
  tracker_->dropStale(stamp);
  plausibility_->dropStale(stamp);

  // The untransformed input, kept only to be drawn next to the output, so the
  // copy is paid for only when someone is looking.
  people_msgs::msg::People sensor_frame_people;
  if (debug_->showsRaw())
  {
    sensor_frame_people = people;
  }

  // Everything above is in the colour optical frame. A message that cannot be
  // re-framed is dropped, not published in the wrong frame.
  std::optional<PeopleMapTransform::RobotMotion> motion;
  if (!map_transform_->transform(people, cmd_vel, &motion))
  {
    return;
  }

  // ── Coasting ──────────────────────────────────────────────────────────────
  // All of it after the transform, so nothing it remembers or forgets is decided
  // by a frame that was never published: a track that could not be re-framed
  // keeps coasting from the last position that could.

  // The anchor the next coasted frame extrapolates from, world-fixed because the
  // transform has already run. Only the detected people are in `people` at this
  // point, so it lines up with `track_ids` one for one.
  coaster_->observe(people, track_ids, stamp);

  // A failed lookup leaves the test empty and coasting runs ungated: not knowing
  // where the camera points is no reason to drop anybody.
  geometry_msgs::msg::TransformStamped target_to_camera;
  TrackCoaster::InViewTest in_view;
  if (map_transform_->lookupFromTarget(detections->header.frame_id, rclcpp::Time(detections->header.stamp),
                                       target_to_camera))
  {
    in_view = [this, &target_to_camera](const geometry_msgs::msg::Point& position) {
      return inCameraView(position, target_to_camera);
    };
  }

  // Already in the published frame, so appended rather than transformed with the
  // rest. After observe(), which must see this frame's measurements alone.
  const std::vector<people_msgs::msg::Person> coasted = coaster_->coast(stamp, track_ids, in_view);
  people.people.insert(people.people.end(), coasted.begin(), coasted.end());

  coaster_->dropStale(stamp);

  people_pub_->publish(people);

  // Draws exactly what was published, plus the input it came from and the robot
  // velocity that separates the two.
  std::optional<DebugVisualizer::RobotArrow> robot;
  if (motion.has_value())
  {
    robot = DebugVisualizer::RobotArrow{ motion->base_origin, motion->linear };
  }
  debug_->publish(people, sensor_frame_people, robot);
}

bool PeopleEstimationNode::inCameraView(const geometry_msgs::msg::Point& position,
                                        const geometry_msgs::msg::TransformStamped& target_to_camera)
{
  geometry_msgs::msg::Point in_camera;
  tf2::doTransform(position, in_camera, target_to_camera);

  // Outside the band no detection could have been made here, so the detector's
  // silence is not evidence of absence. Positive depth_min_m_ also rules out points
  // behind the camera, which would otherwise project back into frame.
  if (in_camera.z < depth_min_m_ || in_camera.z > depth_max_m_)
  {
    return false;
  }

  const std::lock_guard<std::mutex> lock(camera_info_mutex_);
  // Both use P, so this inverts the projectPixelTo3dRay above exactly.
  const cv::Point2d pixel = camera_model_.project3dToPixel(cv::Point3d(in_camera.x, in_camera.y, in_camera.z));
  const cv::Size resolution = camera_model_.fullResolution();
  // Inset, because a person straddling the border is detected unreliably: not finding
  // them there is too weak a signal to drop a track on. In metres at the ghost's own
  // range, since a body spans three times as many pixels at depth_min_m as at
  // depth_max_m. Close in the inset can swallow the image and nothing is ever
  // dropped, which is the safe direction.
  const double margin_x = camera_model_.getDeltaU(coast_in_view_margin_m_, in_camera.z);
  const double margin_y = camera_model_.getDeltaV(coast_in_view_margin_m_, in_camera.z);
  const cv::Rect2d inset(margin_x, margin_y, resolution.width - 2.0 * margin_x, resolution.height - 2.0 * margin_y);
  return inset.contains(pixel);
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

std::optional<people_msgs::msg::Person> PeopleEstimationNode::processDetection(
    const cv::Mat& depth_m, const riskam_msgs::msg::PersonDetection& detection, double stamp)
{
  // A box that was not there before is not evidence of a person. Ahead of the
  // depth read, so an unconfirmed detection costs nothing. See docs/confirmation.md.
  if (!confirmer_->confirmed(detection, stamp))
  {
    return std::nullopt;
  }

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

  // A picture of a person is not a person. See docs/plausibility.md.
  const Plausibility verdict = plausibility_->screen(depth_m, detection, depth.depth_m, stamp);
  if (!verdict.plausible)
  {
    // Suppression has to reach the coaster too. A track already published for
    // the frames it took the filter to make up its mind is sitting in there as
    // somebody worth carrying forward, and coasting it would re-publish the
    // poster for coast_max_s after the filter rejected it. Unlike a dropout,
    // this is not somebody who might still be there. See docs/coasting.md.
    coaster_->forget(detection.track_id);
    return std::nullopt;
  }

  const Eigen::Vector3d position = Eigen::Vector3d(ray.x, ray.y, ray.z) * depth.depth_m;

  const double bbox_width = std::abs(static_cast<double>(detection.bbox[2] - detection.bbox[0]));
  const double bbox_height = std::abs(static_cast<double>(detection.bbox[3] - detection.bbox[1]));
  const Eigen::Matrix3d covariance = uncertainty_->positionCovariance(ray, depth, bbox_width, bbox_height, fx, fy);

  // ByteTrack reports every unassociated detection as -1; those have no identity
  // to hold a history against, so they pass through as measured, with the velocity
  // reported unknown rather than zero.
  const TrackUpdate update = detection.track_id >= 0 ?
                                 tracker_->update(detection.track_id, stamp, position, covariance, cumulative_yaw_) :
                                 TrackUpdate::untracked(position, covariance, tracker_->priorFit());

  // ── Assemble the Person ───────────────────────────────────────────────────
  people_msgs::msg::Person person;
  person.name = detection.track_id >= 0 ? "person_" + std::to_string(detection.track_id) :
                                          "person_untracked_" + std::to_string(untracked_in_message_++);
  person.position = tf2::toMsg(update.position);
  person.velocity = tf2::toMsg(update.fit.velocity);
  // This frame's depth and keypoints, but the filtered covariance: only the
  // position is in doubt. See docs/uncertainty.md.
  person.reliability = uncertainty_->reliability(depth, update.covariance, countValidKeypoints(detection));

  // people_msgs/Person has no field for the per-detection diagnostics, so
  // tagnames/tags carry them. Always published: dropping or holding a person is
  // diagnosed from these, and background_frac lets the plausibility threshold be
  // re-tuned from a bag without rebuilding.
  person.tagnames = { "depth_valid_frac", "depth_spread_m", "position_hold_frames", "background_frac" };
  person.tags = { std::to_string(depth.valid_frac), std::to_string(depth.spread_m), std::to_string(update.hold_frames),
                  std::to_string(verdict.background_frac) };

  if (publish_covariance_tags_)
  {
    // The velocity numbers ride along the same way, in this message's frame.
    // See docs/uncertainty.md.
    const Eigen::Matrix3d velocity_covariance = update.fit.variance.asDiagonal();
    person.tagnames.insert(person.tagnames.end(),
                           { "position_covariance", "velocity_covariance", "n_history_samples", "velocity_window_s" });
    person.tags.insert(person.tags.end(), { formatCovariance(update.covariance), formatCovariance(velocity_covariance),
                                            std::to_string(update.fit.n_samples), std::to_string(update.window_s) });
  }

  return person;
}

}  // namespace people_velocity_estimation

RCLCPP_COMPONENTS_REGISTER_NODE(people_velocity_estimation::PeopleEstimationNode)
