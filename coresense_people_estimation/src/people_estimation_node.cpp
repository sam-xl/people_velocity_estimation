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

#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <opencv2/core.hpp>
#include <optional>
#include <string>
#include <utility>

#include "coresense_people_estimation/debug_visualizer.hpp"
#include "coresense_people_estimation/depth_reader.hpp"
#include "coresense_people_estimation/keypoints.hpp"
#include "cv_bridge/cv_bridge.hpp"
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
}  // namespace

namespace coresense_people_estimation
{

PeopleEstimationNode::PeopleEstimationNode(const rclcpp::NodeOptions& options)
  : rclcpp::Node("people_estimation", options)
{
  // ── Topics ────────────────────────────────────────────────────────────────
  const std::string detections_topic = declare_parameter<std::string>("detections_topic", "/riskam/person_detections");
  // Must be registered to the colour frame: the boxes and keypoints are
  // colour-image pixels. See the README.
  const std::string depth_topic =
      declare_parameter<std::string>("depth_topic", "/camera/camera/aligned_depth_to_color/image_raw");
  const std::string camera_info_topic =
      declare_parameter<std::string>("camera_info_topic", "/camera/camera/color/camera_info");
  const std::string people_topic = declare_parameter<std::string>("people_topic", "/people");

  const auto sync_queue_size = static_cast<int>(declare_parameter<std::int64_t>("sync_queue_size", 10));
  const double sync_slop = declare_parameter<double>("sync_slop", 0.1);

  // ── Depth read-out ────────────────────────────────────────────────────────
  // The valid-depth band is declared here rather than in `DepthReader`, because
  // both the read-out and the range check in processDetection() judge against it.
  depth_min_m_ = declare_parameter<double>("depth_min_m", 0.1);
  depth_max_m_ = declare_parameter<double>("depth_max_m", 3.0);

  // ── Collaborators ─────────────────────────────────────────────────────────
  // Each declares its own parameters on this node, so they come after the block
  // above. One per README section; see the class comment.
  depth_reader_ = std::make_unique<DepthReader>(*this, depth_min_m_, depth_max_m_);
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

  people_pub_ = create_publisher<people_msgs::msg::People>(people_topic, 10);

  RCLCPP_INFO(get_logger(), "people_estimation started | detections=%s | depth=%s | camera_info=%s -> %s",
              detections_topic.c_str(), depth_topic.c_str(), camera_info_topic.c_str(), people_topic.c_str());
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

  // ── Per-detection processing ──────────────────────────────────────────────
  people_msgs::msg::People people;
  people.header = detections->header;

  // Per message, not per process: an ever-growing suffix would name a new debug
  // TF frame every frame, and tf2 never retires a frame id.
  untracked_in_message_ = 0;

  for (const auto& detection : detections->detections)
  {
    std::optional<people_msgs::msg::Person> person = processDetection(*depth_m, detection);
    if (person.has_value())
    {
      people.people.push_back(std::move(*person));
    }
  }

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

std::optional<people_msgs::msg::Person>
PeopleEstimationNode::processDetection(const cv::Mat& depth_m, const riskam_msgs::msg::PersonDetection& detection)
{
  const DepthEstimate depth = depth_reader_->read(depth_m, detection);

  // No valid reading means no measurement to publish.
  if (depth.valid_frac <= 0.0 || depth.depth_m < depth_min_m_ || depth.depth_m > depth_max_m_)
  {
    return std::nullopt;
  }

  const cv::Point2d anchor = anchorPixel(detection);
  cv::Point3d ray;
  {
    const std::lock_guard<std::mutex> lock(camera_info_mutex_);
    ray = camera_model_.projectPixelTo3dRay(anchor);
  }

  // ── Assemble the Person ───────────────────────────────────────────────────
  people_msgs::msg::Person person;
  // ByteTrack reports every unassociated detection as -1, so those all share one
  // id and need a name of their own to be told apart.
  person.name = detection.track_id >= 0 ? "person_" + std::to_string(detection.track_id) :
                                          "person_untracked_" + std::to_string(untracked_in_message_++);
  person.position.x = ray.x * depth.depth_m;
  person.position.y = ray.y * depth.depth_m;
  person.position.z = ray.z * depth.depth_m;
  // Every detection that got this far is equally believed: there is no model yet
  // of how confident this measurement is.
  person.reliability = 1.0;

  // people_msgs/Person has no field for these, so tagnames/tags carry them.
  // See the README.
  person.tagnames = { "depth_valid_frac", "depth_spread_m" };
  person.tags = { std::to_string(depth.valid_frac), std::to_string(depth.spread_m) };

  return person;
}

}  // namespace coresense_people_estimation

RCLCPP_COMPONENTS_REGISTER_NODE(coresense_people_estimation::PeopleEstimationNode)
