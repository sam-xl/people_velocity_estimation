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
  std::array<double, 3> velocity{ { 0.0, 0.0, 0.0 } };
  std::array<double, 3> variance{ { 0.0, 0.0, 0.0 } };
  std::size_t n_samples{ 0 };
};

/// What to publish for one track this frame.
struct TrackUpdate
{
  std::array<double, 3> position;
  VelocityFit fit;
  double window_s;  ///< time span of the samples the fit came from
};

/// Remembers where each tracked person has been, and turns that into a velocity.
///
/// The velocity is a least-squares slope through the trailing samples of a
/// track, over a window chosen per track and per frame: as much history as still
/// looks like one constant velocity, and no more. See the README.
///
/// Not a node: it declares its parameters on the owning node. The camera's own
/// yaw is passed in rather than integrated here, because the node already
/// latches the `cmd_vel` it comes from.
class PersonTracker
{
public:
  explicit PersonTracker(rclcpp::Node& node);

  /// Fold this frame's reading into `track_id` and report what to publish for it.
  /// `variance` is the per-axis position variance the window's consistency test
  /// judges residuals against; see the README.
  TrackUpdate update(std::int32_t track_id, double stamp, const std::array<double, 3>& position,
                     const std::array<double, 3>& variance, double cumulative_yaw);

  /// Forget tracks too old to contribute, so churning ids cannot grow the map.
  void dropStale(double now);

  /// What to publish when there is no velocity evidence: zero, at
  /// `velocity_prior_variance`. Never a zero variance, which would assert a
  /// standstill with total confidence.
  VelocityFit priorFit() const;

private:
  /// One observation of a track.
  struct Sample
  {
    double stamp;                    ///< seconds, from the detection message header
    std::array<double, 3> position;  ///< colour optical frame, metres
    std::array<double, 3> variance;  ///< per-axis position variance, m^2
    double cumulative_yaw;           ///< camera yaw integrated from cmd_vel, radians
  };

  /// Everything remembered about one tracked person between frames.
  struct Track
  {
    std::deque<Sample> history;  ///< accepted samples, oldest first
  };

  /// One track's velocity, and the window it was measured over.
  struct AdaptiveFit
  {
    VelocityFit fit;
    double window_s{ 0.0 };
  };

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
  /// Every sample brought into the newest sample's camera frame.
  std::deque<Sample> derotate(const std::deque<Sample>& history) const;

  /// Drop samples aged out of the longest window, and clear it outright across a
  /// gap long enough that the track may not be the same person any more.
  void pruneHistory(std::deque<Sample>& history, double now) const;

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
};

}  // namespace coresense_people_estimation
