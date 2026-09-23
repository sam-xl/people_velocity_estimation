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
#include <cstdint>
#include <opencv2/video/tracking.hpp>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"

namespace people_velocity_estimation
{

/// What to publish for one track this frame.
struct TrackUpdate
{
  Eigen::Vector3d position;
  Eigen::Matrix3d covariance;
  Eigen::Vector3d velocity;
  Eigen::Matrix3d velocity_covariance;
  /// Zero when this frame's measurement was folded in, else how many consecutive
  /// frames it has been gated out and the prediction published instead.
  std::size_t hold_frames;
  /// Squared Mahalanobis distance of this frame's measurement from the
  /// prediction; zero when there was no prediction to compare against.
  double innovation_chi2;

  /// For a detection carrying no track id: published as measured, never held,
  /// with no history behind the velocity.
  static TrackUpdate untracked(const Eigen::Vector3d& position, const Eigen::Matrix3d& covariance,
                               const Eigen::Matrix3d& velocity_covariance)
  {
    return TrackUpdate{ position, covariance, Eigen::Vector3d::Zero(), velocity_covariance, 0, 0.0 };
  }
};

/// One constant-velocity Kalman filter per tracked person, in the colour
/// optical frame.
///
/// The innovation gate doubles as the depth spike filter: a reading too far
/// from the prediction is not folded in, so it never corrupts the velocity.
/// See the README.
///
/// Not a node: it declares its parameters on the owning node and logs through it.
/// The camera's own yaw is passed in rather than integrated here, because the
/// node already latches the `cmd_vel` it comes from.
class PersonTracker
{
public:
  explicit PersonTracker(rclcpp::Node& node);

  /// Fold this frame's reading into `track_id` and report what to publish for it.
  TrackUpdate update(std::int32_t track_id, double stamp, const Eigen::Vector3d& position,
                     const Eigen::Matrix3d& covariance, double cumulative_yaw);

  /// Forget tracks too old to predict from, so churning ids cannot grow the map.
  void dropStale(double now);

  /// Velocity covariance when there is no velocity evidence. Never zero, which
  /// would assert a standstill with total confidence.
  Eigen::Matrix3d priorVelocityCovariance() const;

private:
  typedef Eigen::Matrix<double, 6, 1> State;
  typedef Eigen::Matrix<double, 6, 6> StateMatrix;

  /// Everything remembered about one tracked person between frames.
  struct Track
  {
    /// State [p; v] and its covariance, as of `stamp`. Never copied: cv::Mat
    /// members copy shallowly.
    cv::KalmanFilter kf;
    /// Seconds, from the detection message header. Advanced by gated frames too,
    /// so how long a track has been gone is judged on when it was last seen.
    double stamp{ 0.0 };
    double cumulative_yaw{ 0.0 };  ///< camera yaw at `stamp`
    /// Camera yaw rate over the last prediction, turning the rotation-free
    /// state velocity back into the apparent one the map transform expects.
    double yaw_rate{ 0.0 };
    std::size_t consecutive_holds{ 0 };  ///< gated frames since, bounded by `spike_max_holds`
  };

  /// Start the track over from this reading, with no velocity evidence.
  void reseed(Track& track, double stamp, const Eigen::Vector3d& position, const Eigen::Matrix3d& covariance,
              double cumulative_yaw) const;

  /// Carry the state forward to `stamp`, turning it with the camera's yaw since.
  void predict(Track& track, double stamp, double cumulative_yaw) const;

  /// Frame-to-frame noise of one reading, rather than its absolute error.
  /// See the README.
  Eigen::Matrix3d measurementNoise(const Eigen::Matrix3d& covariance) const;

  /// Clamp this frame's velocity change to `max_person_accel`, once the velocity
  /// is known well enough that a larger change would be an acceleration rather
  /// than the filter still finding it. See the README.
  void capAcceleration(Track& track, double dt) const;

  /// Discrete white-noise acceleration over `dt`.
  StateMatrix processNoise(double dt) const;

  /// Squared Mahalanobis distance of `position` from the predicted one.
  double innovationChiSquare(const Track& track, const Eigen::Vector3d& position,
                             const Eigen::Matrix3d& measurement_noise) const;

  /// The current state, as published.
  TrackUpdate report(const Track& track, double innovation_chi2) const;

  /// Logger and clock come from it.
  rclcpp::Node& node_;

  std::unordered_map<std::int32_t, Track> tracks_;

  // ── Parameters ──────────────────────────────────────────────────────────────
  double velocity_prior_variance_;
  double velocity_epsilon_;
  double measurement_sigma_scale_;
  double gait_sigma_m_;
  double kalman_accel_sigma_;
  double kalman_gate_chi2_;
  double max_person_accel_;
  double spike_reset_timeout_;
  std::size_t spike_max_holds_;
};

}  // namespace people_velocity_estimation
