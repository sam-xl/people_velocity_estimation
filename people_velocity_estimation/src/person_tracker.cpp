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

#include "people_velocity_estimation/person_tracker.hpp"

#include <Eigen/Dense>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iterator>
#include <opencv2/core/eigen.hpp>

#include "people_velocity_estimation/parameters.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{
/// Rotation about the camera's vertical by a robot yaw of `angle`. The colour
/// optical frame is x-right, y-down, z-forward, so the yaw axis is optical -y
/// and a left turn works out to Ry(+angle). Pitch and roll are ignored.
Eigen::Matrix3d yawRotation(double angle)
{
  return Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitY()).toRotationMatrix();
}
}  // namespace

namespace people_velocity_estimation
{

PersonTracker::PersonTracker(rclcpp::Node& node)
  : node_(node)
{
  velocity_prior_variance_ = node_.declare_parameter<double>("velocity_prior_variance", 1.0);
  velocity_epsilon_ = node_.declare_parameter<double>("velocity_epsilon", 0.05);
  measurement_sigma_scale_ = node_.declare_parameter<double>("measurement_sigma_scale", 0.25);
  gait_sigma_m_ = node_.declare_parameter<double>("gait_sigma_m", 0.06);
  kalman_accel_sigma_ = node_.declare_parameter<double>("kalman_accel_sigma", 0.5);
  kalman_gate_chi2_ = node_.declare_parameter<double>("kalman_gate_chi2", 11.34);
  max_person_accel_ = node_.declare_parameter<double>("max_person_accel", 4.0);

  // See parameters.hpp for why a count is not declared as a plain int64.
  spike_reset_timeout_ = node_.declare_parameter<double>("spike_reset_timeout", 1.0);
  spike_max_holds_ = declareCount(node_, "spike_max_holds", 5);
}

TrackUpdate PersonTracker::update(std::int32_t track_id, double stamp, const Eigen::Vector3d& position,
                                  const Eigen::Matrix3d& covariance, double cumulative_yaw)
{
  const auto [it, is_new] = tracks_.try_emplace(track_id);
  Track& track = it->second;

  const double dt = stamp - track.stamp;
  // A looping bag walks the clock backwards; a long gap leaves nothing worth
  // predicting from.
  if (is_new || dt < 0.0 || dt > spike_reset_timeout_)
  {
    reseed(track, stamp, position, covariance, cumulative_yaw);
    return report(track, 0.0);
  }

  predict(track, stamp, cumulative_yaw);
  const Eigen::Matrix3d measurement_noise = measurementNoise(covariance);
  const double innovation_chi2 = innovationChiSquare(track, position, measurement_noise);

  // At 0 the gate is off, the A/B switch for judging whether it earns its place.
  if (spike_max_holds_ == 0 || innovation_chi2 <= kalman_gate_chi2_)
  {
    cv::Mat measurement;
    cv::eigen2cv(position, measurement);
    cv::eigen2cv(measurement_noise, track.kf.measurementNoiseCov);
    track.kf.correct(measurement);
    capAcceleration(track, dt);
    track.consecutive_holds = 0;
    return report(track, innovation_chi2);
  }

  // Bounded, because gating forever would pin the track to a prediction the
  // person may genuinely have left.
  if (track.consecutive_holds >= spike_max_holds_)
  {
    RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), kWarnThrottleMs,
                         "Track %d has been %.1f chi-square from its prediction for %zu frames running; believing the "
                         "sensor again. Either kalman_gate_chi2 is too tight, or this track id was handed to a "
                         "different person.",
                         track_id, innovation_chi2, track.consecutive_holds);
    reseed(track, stamp, position, covariance, cumulative_yaw);
    return report(track, innovation_chi2);
  }

  ++track.consecutive_holds;
  RCLCPP_DEBUG(node_.get_logger(), "Gating out track %d (chi-square %.1f, hold %zu)", track_id, innovation_chi2,
               track.consecutive_holds);
  return report(track, innovation_chi2);
}

void PersonTracker::reseed(Track& track, double stamp, const Eigen::Vector3d& position,
                           const Eigen::Matrix3d& covariance, double cumulative_yaw) const
{
  track.kf.init(6, 3, 0, CV_64F);
  track.kf.measurementMatrix = cv::Mat::eye(3, 6, CV_64F);

  State state = State::Zero();
  state.head<3>() = position;
  StateMatrix state_covariance = StateMatrix::Zero();
  state_covariance.topLeftCorner<3, 3>() = covariance;
  state_covariance.bottomRightCorner<3, 3>() = priorVelocityCovariance();
  cv::eigen2cv(state, track.kf.statePost);
  cv::eigen2cv(state_covariance, track.kf.errorCovPost);

  track.stamp = stamp;
  track.cumulative_yaw = cumulative_yaw;
  track.yaw_rate = 0.0;
  track.consecutive_holds = 0;
}

void PersonTracker::predict(Track& track, double stamp, double cumulative_yaw) const
{
  const double dt = stamp - track.stamp;
  const double delta_yaw = cumulative_yaw - track.cumulative_yaw;

  StateMatrix constant_velocity = StateMatrix::Identity();
  constant_velocity.topRightCorner<3, 3>() = dt * Eigen::Matrix3d::Identity();
  // The camera's turn is taken out explicitly, so a robot rotation does not
  // read as the person manoeuvring. See the README.
  StateMatrix camera_turn = StateMatrix::Zero();
  camera_turn.topLeftCorner<3, 3>() = yawRotation(delta_yaw);
  camera_turn.bottomRightCorner<3, 3>() = camera_turn.topLeftCorner<3, 3>();

  cv::eigen2cv(StateMatrix(camera_turn * constant_velocity), track.kf.transitionMatrix);
  cv::eigen2cv(processNoise(dt), track.kf.processNoiseCov);
  track.kf.predict();

  if (dt > 0.0)
  {
    track.yaw_rate = delta_yaw / dt;
  }
  track.stamp = stamp;
  track.cumulative_yaw = cumulative_yaw;
}

void PersonTracker::capAcceleration(Track& track, double dt) const
{
  const double max_change = max_person_accel_ * dt;
  if (max_change <= 0.0)
  {
    return;
  }
  StateMatrix predicted_covariance;
  cv::cv2eigen(track.kf.errorCovPre, predicted_covariance);
  if (predicted_covariance.bottomRightCorner<3, 3>().diagonal().maxCoeff() > max_change * max_change)
  {
    return;
  }

  State predicted;
  State corrected;
  cv::cv2eigen(track.kf.statePre, predicted);
  cv::cv2eigen(track.kf.statePost, corrected);
  const Eigen::Vector3d change = corrected.tail<3>() - predicted.tail<3>();
  if (change.norm() <= max_change)
  {
    return;
  }
  // Estimate projection onto the constraint; the covariance is left as the
  // filter computed it.
  // https://doi.org/10.1049/iet-cta.2009.0032
  corrected.tail<3>() = predicted.tail<3>() + change * (max_change / change.norm());
  cv::eigen2cv(corrected, track.kf.statePost);
}

Eigen::Matrix3d PersonTracker::measurementNoise(const Eigen::Matrix3d& covariance) const
{
  return measurement_sigma_scale_ * measurement_sigma_scale_ * covariance +
         gait_sigma_m_ * gait_sigma_m_ * Eigen::Matrix3d::Identity();
}

PersonTracker::StateMatrix PersonTracker::processNoise(double dt) const
{
  const double accel_var = kalman_accel_sigma_ * kalman_accel_sigma_;
  const Eigen::Matrix3d identity = Eigen::Matrix3d::Identity();
  StateMatrix noise;
  noise << std::pow(dt, 4) / 4.0 * identity, std::pow(dt, 3) / 2.0 * identity,  //
      std::pow(dt, 3) / 2.0 * identity, dt * dt * identity;
  return accel_var * noise;
}

double PersonTracker::innovationChiSquare(const Track& track, const Eigen::Vector3d& position,
                                          const Eigen::Matrix3d& measurement_noise) const
{
  State state;
  StateMatrix state_covariance;
  cv::cv2eigen(track.kf.statePost, state);
  cv::cv2eigen(track.kf.errorCovPost, state_covariance);

  const Eigen::Vector3d innovation = position - state.head<3>();
  const Eigen::Matrix3d innovation_covariance = state_covariance.topLeftCorner<3, 3>() + measurement_noise;
  return innovation.dot(innovation_covariance.ldlt().solve(innovation));
}

TrackUpdate PersonTracker::report(const Track& track, double innovation_chi2) const
{
  State state;
  StateMatrix state_covariance;
  cv::cv2eigen(track.kf.statePost, state);
  cv::cv2eigen(track.kf.errorCovPost, state_covariance);

  const Eigen::Vector3d position = state.head<3>();
  // The map transform expects the apparent velocity in the turning camera frame,
  // which the camera's own turn adds to. See the README.
  Eigen::Vector3d velocity = state.tail<3>() + track.yaw_rate * Eigen::Vector3d::UnitY().cross(position);
  // Under the noise floor there is no direction, only the residual wobble of a
  // person standing still. See the README.
  if (velocity.norm() < velocity_epsilon_)
  {
    velocity.setZero();
  }
  return TrackUpdate{ position,
                      state_covariance.topLeftCorner<3, 3>(),
                      velocity,
                      state_covariance.bottomRightCorner<3, 3>(),
                      track.consecutive_holds,
                      innovation_chi2 };
}

Eigen::Matrix3d PersonTracker::priorVelocityCovariance() const
{
  return velocity_prior_variance_ * Eigen::Matrix3d::Identity();
}

void PersonTracker::dropStale(double now)
{
  // Absolute value because a looping bag walks the clock backwards.
  for (auto it = tracks_.begin(); it != tracks_.end();)
  {
    it = std::abs(now - it->second.stamp) > spike_reset_timeout_ ? tracks_.erase(it) : std::next(it);
  }
}

}  // namespace people_velocity_estimation
