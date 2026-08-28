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

#include "coresense_people_estimation/person_tracker.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <iterator>

#include "rclcpp/rclcpp.hpp"

namespace
{
/// Fewest samples the reduced chi-square has a degree of freedom left at, given
/// the two the straight line itself costs.
constexpr std::size_t kMinChiSquareSamples = 4;

/// Keeps a residual out of a division by a vanishing variance.
constexpr double kVarianceFloor = 1e-12;

/// Below this the normal equations are singular: every sample shares a timestamp.
constexpr double kSingularDetEpsilon = 1e-12;

double norm3(const std::array<double, 3>& v)
{
  return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

/// Rotation of a point about the camera's vertical, by a robot yaw of `angle`.
/// The colour optical frame is x-right, y-down, z-forward, so the yaw axis is
/// optical -y and a left turn works out to Ry(+angle). Pitch and roll are ignored.
std::array<double, 3> rotateByYaw(const std::array<double, 3>& p, double angle)
{
  const double c = std::cos(angle);
  const double s = std::sin(angle);
  return { { c * p[0] + s * p[2], p[1], -s * p[0] + c * p[2] } };
}
}  // namespace

namespace coresense_people_estimation
{

PersonTracker::PersonTracker(rclcpp::Node& node)
{
  velocity_prior_variance_ = node.declare_parameter<double>("velocity_prior_variance", 1.0);

  velocity_window_max_s_ = node.declare_parameter<double>("velocity_window_max_s", 2.0);
  velocity_window_min_s_ = node.declare_parameter<double>("velocity_window_min_s", 0.6);
  velocity_window_chi2_max_ = node.declare_parameter<double>("velocity_window_chi2_max", 3.0);
  velocity_window_sigma_scale_ = node.declare_parameter<double>("velocity_window_sigma_scale", 0.25);
  velocity_gait_sigma_m_ = node.declare_parameter<double>("velocity_gait_sigma_m", 0.03);
  velocity_min_span_s_ = node.declare_parameter<double>("velocity_min_span_s", 0.3);
  velocity_epsilon_ = node.declare_parameter<double>("velocity_epsilon", 0.05);
  velocity_max_gap_s_ = node.declare_parameter<double>("velocity_max_gap_s", 0.5);
  const auto declared_maxlen = node.declare_parameter<std::int64_t>("history_maxlen", 60);
  if (declared_maxlen < static_cast<std::int64_t>(kMinFitSamples))
  {
    RCLCPP_WARN(node.get_logger(),
                "history_maxlen of %ld cannot hold a fit; raising it to %zu. Below that every velocity is zero.",
                declared_maxlen, kMinFitSamples);
  }
  history_maxlen_ = static_cast<std::size_t>(std::max(declared_maxlen, static_cast<std::int64_t>(kMinFitSamples)));
}

TrackUpdate PersonTracker::update(std::int32_t track_id, double stamp, const std::array<double, 3>& position,
                                  const std::array<double, 3>& variance, double cumulative_yaw)
{
  Track& track = tracks_[track_id];

  // A gap wide enough that this may not be the same person clears the history
  // rather than having a slope fitted across it.
  pruneHistory(track.history, stamp);
  track.history.push_back(Sample{ stamp, position, variance, cumulative_yaw });
  while (track.history.size() > history_maxlen_)
  {
    track.history.pop_front();
  }

  const AdaptiveFit adaptive = fitAdaptive(track.history);
  return TrackUpdate{ position, adaptive.fit, adaptive.window_s };
}

VelocityFit PersonTracker::priorFit() const
{
  VelocityFit fit;
  fit.variance = { { velocity_prior_variance_, velocity_prior_variance_, velocity_prior_variance_ } };
  return fit;
}

PersonTracker::AdaptiveFit PersonTracker::fitAdaptive(const std::deque<Sample>& history) const
{
  AdaptiveFit result;
  // Below a minimum baseline there is no velocity worth reporting; see the README.
  if (history.size() < kMinFitSamples || (history.back().stamp - history.front().stamp) < velocity_min_span_s_)
  {
    result.fit = priorFit();
    result.fit.n_samples = history.size();
    return result;
  }

  const std::size_t n = selectWindow(history);
  std::deque<Sample> window(history.end() - static_cast<std::ptrdiff_t>(n), history.end());
  result.window_s = window.back().stamp - window.front().stamp;
  result.fit = fitVelocity(window);

  // Under the noise floor there is no direction, only the residual wobble of a
  // person standing still. See the README.
  if (norm3(result.fit.velocity) < velocity_epsilon_)
  {
    result.fit.velocity = { { 0.0, 0.0, 0.0 } };
  }
  return result;
}

std::size_t PersonTracker::selectWindow(const std::deque<Sample>& history) const
{
  // Judged on de-rotated samples but fitted on the raw ones, so that a robot turn
  // neither collapses the window nor moves the published, body-fixed value.
  // See the README.
  const std::deque<Sample> judged = derotate(history);

  std::size_t n = history.size();
  while (n > kMinFitSamples)
  {
    std::deque<Sample> window(judged.end() - static_cast<std::ptrdiff_t>(n), judged.end());
    // Never shorten below the floor: about one gait oscillation.
    if ((window.back().stamp - window.front().stamp) <= velocity_window_min_s_)
    {
      break;
    }
    if (windowChiSquare(window, fitVelocity(window)) <= velocity_window_chi2_max_)
    {
      break;
    }
    --n;
  }
  return n;
}

double PersonTracker::windowChiSquare(const std::deque<Sample>& window, const VelocityFit& fit) const
{
  // Reduced chi-square of the constant-velocity model, per axis and worst of the
  // three: a pooled statistic buries a one-axis manoeuvre. See the README.

  // Zero reads as "consistent", which is the right answer here: with no degree
  // of freedom left there is nothing to test, so the window must not shorten.
  if (window.size() < kMinChiSquareSamples)
  {
    return 0.0;
  }
  const auto n = static_cast<double>(window.size());
  const double t0 = window.front().stamp;

  // Residuals are judged against frame-to-frame repeatability rather than the
  // absolute position error, with gait as the floor under it. See the README.
  const double gait_var = velocity_gait_sigma_m_ * velocity_gait_sigma_m_;
  const double sigma_scale_sq = velocity_window_sigma_scale_ * velocity_window_sigma_scale_;

  // Intercept per axis, from the same normal equations fitVelocity solved.
  std::array<double, 3> mean_p{ { 0.0, 0.0, 0.0 } };
  double mean_t = 0.0;
  for (const Sample& sample : window)
  {
    mean_t += (sample.stamp - t0) / n;
    for (std::size_t axis = 0; axis < 3; ++axis)
    {
      mean_p[axis] += sample.position[axis] / n;
    }
  }

  double worst = 0.0;
  for (std::size_t axis = 0; axis < 3; ++axis)
  {
    double chi2 = 0.0;
    for (const Sample& sample : window)
    {
      const double t = sample.stamp - t0;
      const double model = mean_p[axis] + fit.velocity[axis] * (t - mean_t);
      const double residual = sample.position[axis] - model;
      const double variance = sigma_scale_sq * sample.variance[axis] + gait_var;
      chi2 += residual * residual / std::max(kVarianceFloor, variance);
    }
    worst = std::max(worst, chi2 / (n - 2.0));
  }
  return worst;
}

std::deque<PersonTracker::Sample> PersonTracker::derotate(const std::deque<Sample>& history) const
{
  std::deque<Sample> rotated = history;
  if (rotated.empty())
  {
    return rotated;
  }
  const double yaw_now = rotated.back().cumulative_yaw;
  for (Sample& sample : rotated)
  {
    const double angle = yaw_now - sample.cumulative_yaw;
    sample.position = rotateByYaw(sample.position, angle);

    // The uncertainty turns with the point: rotateByYaw is Ry, so only the x-z
    // block moves and y is untouched. The variances are per-axis, so the turned
    // pair is a mix of the two — exact only while the x-z cross-covariance is
    // negligible, which is why it is an approximation here. See the README.
    const double c = std::cos(angle);
    const double s = std::sin(angle);
    const double xx = sample.variance[0];
    const double zz = sample.variance[2];
    sample.variance[0] = c * c * xx + s * s * zz;
    sample.variance[2] = s * s * xx + c * c * zz;
  }
  return rotated;
}

VelocityFit PersonTracker::fitVelocity(const std::deque<Sample>& history) const
{
  VelocityFit fit;
  fit.n_samples = history.size();
  if (history.size() < 2)
  {
    fit.variance = priorFit().variance;
    return fit;
  }

  // Least squares for p(t) = v t + b, per axis. A = [t, 1], so
  // (A^T A)^-1 = 1/det * [[n, -S_t], [-S_t, S_tt]] with det = n S_tt - S_t^2,
  // and Var(v) = sigma^2 * n / det.
  const double t0 = history.front().stamp;
  const auto n = static_cast<double>(history.size());
  double s_t = 0.0;
  double s_tt = 0.0;
  std::array<double, 3> s_p{ { 0.0, 0.0, 0.0 } };
  std::array<double, 3> s_tp{ { 0.0, 0.0, 0.0 } };

  for (const Sample& sample : history)
  {
    const double t = sample.stamp - t0;
    s_t += t;
    s_tt += t * t;
    for (std::size_t axis = 0; axis < 3; ++axis)
    {
      s_p[axis] += sample.position[axis];
      s_tp[axis] += t * sample.position[axis];
    }
  }

  const double det = n * s_tt - s_t * s_t;
  if (std::abs(det) < kSingularDetEpsilon)
  {
    // Every sample shares a timestamp: no time base, so no slope.
    fit.variance = priorFit().variance;
    return fit;
  }

  for (std::size_t axis = 0; axis < 3; ++axis)
  {
    fit.velocity[axis] = (n * s_tp[axis] - s_t * s_p[axis]) / det;
  }

  if (history.size() < kMinFitSamples)
  {
    // Two samples fit exactly, so there is no residual to estimate a variance from.
    fit.variance = priorFit().variance;
    return fit;
  }

  std::array<double, 3> residual_sq{ { 0.0, 0.0, 0.0 } };
  for (std::size_t axis = 0; axis < 3; ++axis)
  {
    const double intercept = (s_tt * s_p[axis] - s_t * s_tp[axis]) / det;
    for (const Sample& sample : history)
    {
      const double t = sample.stamp - t0;
      const double r = fit.velocity[axis] * t + intercept - sample.position[axis];
      residual_sq[axis] += r * r;
    }
    const double sigma_sq = residual_sq[axis] / (n - 2.0);
    fit.variance[axis] = sigma_sq * n / det;
  }
  return fit;
}

void PersonTracker::pruneHistory(std::deque<Sample>& history, double now) const
{
  // A gap this long means the track was not observed for most of a window.
  if (!history.empty() && (now - history.back().stamp) > velocity_max_gap_s_)
  {
    history.clear();
    return;
  }
  while (!history.empty() && (now - history.front().stamp) > velocity_window_max_s_)
  {
    history.pop_front();
  }
}

void PersonTracker::dropStale(double now)
{
  // A track with nothing left inside the longest window contributes nothing but
  // a map entry. Absolute value because a looping bag walks the clock backwards.
  for (auto it = tracks_.begin(); it != tracks_.end();)
  {
    const std::deque<Sample>& history = it->second.history;
    const bool stale = history.empty() || std::abs(now - history.back().stamp) > velocity_window_max_s_;
    it = stale ? tracks_.erase(it) : std::next(it);
  }
}

}  // namespace coresense_people_estimation
