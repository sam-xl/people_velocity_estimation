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
#include <deque>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"

namespace coresense_people_estimation
{

/// Fewest samples a velocity can be fitted from: two fit any line exactly, so
/// three is the first count that leaves a residual to judge.
constexpr std::size_t kMinFitSamples = 3;

/// Result of fitting a straight line through a track's position history.
struct VelocityFit
{
  Eigen::Vector3d velocity{ Eigen::Vector3d::Zero() };
  Eigen::Vector3d intercept{ Eigen::Vector3d::Zero() };  ///< position at the window's first stamp
  Eigen::Vector3d variance{ Eigen::Vector3d::Zero() };
  std::size_t n_samples{ 0 };
};

/// What to publish for one track this frame.
struct TrackUpdate
{
  Eigen::Vector3d position;
  Eigen::Matrix3d covariance;
  /// Zero when the values above are this frame's measurement, else how many
  /// consecutive frames they have been held for.
  std::size_t hold_frames;
  VelocityFit fit;
  double window_s;  ///< time span of the samples the fit came from

  /// For a detection carrying no track id: published as measured, never held,
  /// with no history behind the velocity. Named so the call site does not have to
  /// be read as "0 means never held, 0.0 means no window".
  static TrackUpdate untracked(const Eigen::Vector3d& position, const Eigen::Matrix3d& covariance,
                               const VelocityFit& fit)
  {
    return TrackUpdate{ position, covariance, 0, fit, 0.0 };
  }
};

/// Remembers where each tracked person has been, and turns that into a velocity.
///
/// Two jobs that cannot be separated: a reading implying an impossible speed is
/// rejected as a depth error, and rejecting it is also what keeps it out of the
/// history the velocity is differentiated from. See the README.
///
/// Not a node: it declares its parameters on the owning node and logs through it.
/// The camera's own yaw is passed in rather than integrated here, because the
/// node already latches the `cmd_vel` it comes from.
class PersonTracker
{
public:
  explicit PersonTracker(rclcpp::Node& node);

  /// Fold this frame's reading into `track_id` and report what to publish for it:
  /// the measurement or a held one, and the velocity that goes with it.
  TrackUpdate update(std::int32_t track_id, double stamp, const Eigen::Vector3d& position,
                     const Eigen::Matrix3d& covariance, double cumulative_yaw);

  /// Forget tracks too old to contribute, so churning ids cannot grow the map.
  void dropStale(double now);

  /// What to publish when there is no velocity evidence: zero, at
  /// `velocity_prior_variance`. Never a zero variance, which would assert a
  /// standstill with total confidence.
  VelocityFit priorFit() const;

private:
  /// One accepted observation of a track; a held frame is never a sample.
  struct Sample
  {
    double stamp;                ///< seconds, from the detection message header
    Eigen::Vector3d position;    ///< colour optical frame, metres
    Eigen::Matrix3d covariance;  ///< position covariance, m^2
    double cumulative_yaw;       ///< camera yaw integrated from cmd_vel, radians
  };

  /// Everything remembered about one tracked person between frames.
  struct Track
  {
    /// The most recent reading the spike filter accepted, republished while its
    /// successors are rejected.
    double stamp{ 0.0 };                 ///< seconds, from the detection message header
    Eigen::Vector3d position{};          ///< colour optical frame, metres
    Eigen::Matrix3d covariance{};        ///< as published in the tags
    double cumulative_yaw{ 0.0 };        ///< camera yaw when `position` was accepted
    std::size_t consecutive_holds{ 0 };  ///< rejections since, bounded by `spike_max_holds`

    /// Refreshed on every sighting, held frames included, so how long a track has
    /// been gone is judged on when it was last seen rather than last believed.
    double seen_stamp{ 0.0 };

    std::deque<Sample> history;   ///< accepted samples, oldest first, never a held frame
    VelocityFit last_fit;         ///< republished whole while the position is held
    double last_window_s{ 0.0 };  ///< span of the window `last_fit` came from
  };

  /// What to publish for one detection, once the spike filter has had its say.
  struct FilteredPosition
  {
    Eigen::Vector3d position;
    Eigen::Matrix3d covariance;
    std::size_t hold_frames;

    // Named so each call site says which of the two it means, rather than leaving
    // a bare 0 to be the only thing separating a measurement from a held frame.
    static FilteredPosition measured(const Eigen::Vector3d& position, const Eigen::Matrix3d& covariance)
    {
      return FilteredPosition{ position, covariance, 0 };
    }
    static FilteredPosition held(const Eigen::Vector3d& position, const Eigen::Matrix3d& covariance,
                                 std::size_t hold_frames)
    {
      return FilteredPosition{ position, covariance, hold_frames };
    }
  };

  /// One track's velocity, and the window it was measured over.
  struct AdaptiveFit
  {
    VelocityFit fit;
    double window_s{ 0.0 };
  };

  /// Accept this frame's reading for a track, or reject it as a spike and
  /// republish the last accepted one.
  FilteredPosition filterPosition(std::int32_t track_id, double stamp, const Eigen::Vector3d& position,
                                  const Eigen::Matrix3d& covariance, double cumulative_yaw);

  /// Velocity over an adaptively chosen window: as much history as stays
  /// consistent with one constant velocity, and no more.
  AdaptiveFit fitAdaptive(const std::deque<Sample>& history) const;

  /// How many of the newest samples to fit, down to the `velocity_window_min_s`
  /// floor.
  std::size_t selectWindow(const std::deque<Sample>& history) const;

  /// Reduced chi-square of a constant-velocity fit over `window`. About 1 when
  /// the model is right; large when the person did something else.
  double windowChiSquare(const std::deque<Sample>& window, const VelocityFit& fit) const;

  VelocityFit fitVelocity(const std::deque<Sample>& history) const;
  /// Rows of p(t) = v t + b: `[t, 1]`, with t measured from the first sample.
  static Eigen::MatrixX2d designMatrix(const std::deque<Sample>& history);
  /// One row per sample, one column per axis.
  static Eigen::MatrixX3d positions(const std::deque<Sample>& history);
  /// Every sample brought into the newest sample's camera frame.
  std::deque<Sample> derotate(const std::deque<Sample>& history) const;

  /// Drop samples aged out of the longest window, and clear it outright across a
  /// gap long enough that the track may not be the same person any more.
  void pruneHistory(std::deque<Sample>& history, double now) const;

  /// Logger and clock come from it.
  rclcpp::Node& node_;

  std::unordered_map<std::int32_t, Track> tracks_;

  // ── Parameters ──────────────────────────────────────────────────────────────
  double velocity_prior_variance_;

  double velocity_window_max_s_;
  double velocity_window_min_s_;
  double velocity_window_chi2_max_;
  double velocity_window_sigma_scale_;
  double velocity_gait_sigma_m_;
  double velocity_min_span_s_;
  double velocity_epsilon_;
  double velocity_max_gap_s_;
  std::size_t history_maxlen_;
  double max_person_speed_;
  double spike_reset_timeout_;
  std::size_t spike_max_holds_;
};

}  // namespace coresense_people_estimation
