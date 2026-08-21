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

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <iterator>

#include "coresense_people_estimation/parameters.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{
/// Fewest samples the reduced chi-square has a degree of freedom left at, given
/// the two the straight line itself costs.
constexpr std::size_t kMinChiSquareSamples = 4;

/// Keeps a residual out of a division by a vanishing variance.
constexpr double kVarianceFloor = 1e-12;

/// Rotation about the camera's vertical by a robot yaw of `angle`. The colour
/// optical frame is x-right, y-down, z-forward, so the yaw axis is optical -y
/// and a left turn works out to Ry(+angle). Pitch and roll are ignored.
Eigen::Matrix3d yawRotation(double angle)
{
  return Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitY()).toRotationMatrix();
}
}  // namespace

namespace coresense_people_estimation
{

PersonTracker::PersonTracker(rclcpp::Node& node)
  : node_(node)
{
  velocity_prior_variance_ = node_.declare_parameter<double>("velocity_prior_variance", 1.0);

  velocity_window_max_s_ = node_.declare_parameter<double>("velocity_window_max_s", 2.0);
  velocity_window_min_s_ = node_.declare_parameter<double>("velocity_window_min_s", 0.6);
  velocity_window_chi2_max_ = node_.declare_parameter<double>("velocity_window_chi2_max", 3.0);
  velocity_window_sigma_scale_ = node_.declare_parameter<double>("velocity_window_sigma_scale", 0.25);
  velocity_gait_sigma_m_ = node_.declare_parameter<double>("velocity_gait_sigma_m", 0.03);
  velocity_min_span_s_ = node_.declare_parameter<double>("velocity_min_span_s", 0.3);
  velocity_epsilon_ = node_.declare_parameter<double>("velocity_epsilon", 0.05);
  velocity_max_gap_s_ = node_.declare_parameter<double>("velocity_max_gap_s", 0.5);
  const auto declared_maxlen = node_.declare_parameter<std::int64_t>("history_maxlen", 60);
  if (declared_maxlen < static_cast<std::int64_t>(kMinFitSamples))
  {
    RCLCPP_WARN(node_.get_logger(),
                "history_maxlen of %ld cannot hold a fit; raising it to %zu. Below that every velocity is zero.",
                declared_maxlen, kMinFitSamples);
  }
  history_maxlen_ = static_cast<std::size_t>(std::max(declared_maxlen, static_cast<std::int64_t>(kMinFitSamples)));

  // See parameters.hpp for why a count is not declared as a plain int64.
  max_person_speed_ = node_.declare_parameter<double>("max_person_speed", 2.5);
  spike_reset_timeout_ = node_.declare_parameter<double>("spike_reset_timeout", 1.0);
  spike_max_holds_ = declareCount(node_, "spike_max_holds", 5);
}

TrackUpdate PersonTracker::update(std::int32_t track_id, double stamp, const Eigen::Vector3d& position,
                                  const Eigen::Matrix3d& covariance, double cumulative_yaw)
{
  // A reading that implies an impossible speed is a depth error, not motion.
  const FilteredPosition filtered = filterPosition(track_id, stamp, position, covariance, cumulative_yaw);

  Track& track = tracks_[track_id];

  VelocityFit fit;
  double window_s = 0.0;
  if (filtered.hold_frames > 0)
  {
    // Hold the velocity with the position, variance included: a rejected reading
    // is no evidence that the person stopped.
    fit = track.last_fit;
    window_s = track.last_window_s;
  }
  else
  {
    const AdaptiveFit adaptive = fitAdaptive(track.history);
    fit = adaptive.fit;
    window_s = adaptive.window_s;
  }

  // A held frame republishes exactly what was published last time.
  if (filtered.hold_frames == 0)
  {
    track.last_fit = fit;
    track.last_window_s = window_s;
  }

  return TrackUpdate{ filtered.position, filtered.covariance, filtered.hold_frames, fit, window_s };
}

PersonTracker::FilteredPosition PersonTracker::filterPosition(std::int32_t track_id, double stamp,
                                                              const Eigen::Vector3d& position,
                                                              const Eigen::Matrix3d& covariance, double cumulative_yaw)
{
  // Accepting a reading is also what feeds the velocity fit, so a spike never
  // becomes a sample and a hold never becomes a stationary one.
  const auto accept = [&](Track& state) {
    state.stamp = stamp;
    state.seen_stamp = stamp;
    state.position = position;
    state.covariance = covariance;
    state.cumulative_yaw = cumulative_yaw;
    state.consecutive_holds = 0;
    pruneHistory(state.history, stamp);
    state.history.push_back(Sample{ stamp, position, covariance.diagonal(), covariance(0, 2), cumulative_yaw });
    while (state.history.size() > history_maxlen_)
    {
      state.history.pop_front();
    }
    return FilteredPosition::measured(position, covariance);
  };

  // This reading is not a successor of the stored one, so every sample in front of
  // it is the history of somewhere else: differentiating across the discontinuity
  // would report the discontinuity itself as motion. See the README.
  const auto reseed = [&](Track& state) {
    state.history.clear();
    state.last_fit = priorFit();
    state.last_window_s = 0.0;
    return accept(state);
  };

  const auto it = tracks_.find(track_id);
  if (it == tracks_.end())
  {
    return accept(tracks_[track_id]);
  }
  Track& state = it->second;
  state.seen_stamp = stamp;

  // The A/B switch for judging whether the filter earns its place: at 0 the
  // tracker behaves exactly as it did before the filter existed, which means
  // accepting every reading rather than re-seeding on each one.
  if (spike_max_holds_ == 0)
  {
    return accept(state);
  }

  const double dt = stamp - state.stamp;
  // The clock walked backwards, as a looping bag does, so the stored position and
  // the history behind it are from the future.
  if (dt < 0.0)
  {
    return reseed(state);
  }
  // Same stamp, so there is no time base to test a speed against.
  if (dt <= 0.0)
  {
    return accept(state);
  }
  // Too old to compare against: the stored position is the worse guess, and the
  // history behind it is older still.
  if (dt > spike_reset_timeout_)
  {
    return reseed(state);
  }

  // Both readings in this frame's camera pose before they are subtracted. A robot
  // turn sweeps a stationary person through the optical frame at omega*r, which at
  // the far end of the depth band clears max_person_speed on its own.
  const Eigen::Vector3d previous = yawRotation(cumulative_yaw - state.cumulative_yaw) * state.position;
  const double speed = (position - previous).norm() / dt;
  // Flat bound on what is left once the robot's own rotation is out. See the README.
  if (speed <= max_person_speed_)
  {
    return accept(state);
  }
  // Bounded, because rejecting forever would pin the track at a position the
  // person may genuinely have left.
  if (state.consecutive_holds >= spike_max_holds_)
  {
    RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), kWarnThrottleMs,
                         "Track %d has implied %.1f m/s for %zu frames running; believing the sensor again. Either "
                         "max_person_speed is too tight, or this track id was handed to a different person.",
                         track_id, speed, state.consecutive_holds);
    // The jump is taken as real, so the track starts over from this reading.
    return reseed(state);
  }

  ++state.consecutive_holds;
  RCLCPP_DEBUG(node_.get_logger(), "Holding track %d at its previous position (%.1f m/s implied, hold %zu)", track_id,
               speed, state.consecutive_holds);
  // The stored covariance, not this frame's: the measured one is centred on the
  // reading just rejected. See the README.
  return FilteredPosition::held(state.position, state.covariance, state.consecutive_holds);
}

VelocityFit PersonTracker::priorFit() const
{
  VelocityFit fit;
  fit.variance.setConstant(velocity_prior_variance_);
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
  if (result.fit.velocity.norm() < velocity_epsilon_)
  {
    result.fit.velocity.setZero();
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

  Eigen::Matrix<double, 2, 3> coefficients;
  coefficients << fit.velocity.transpose(), fit.intercept.transpose();
  const Eigen::MatrixX3d residual = positions(window) - designMatrix(window) * coefficients;

  // Residuals are judged against repeatability rather than the localisation
  // uncertainty the uncertainty model reports, with gait as the floor under it.
  // See the README.
  const double gait_var = velocity_gait_sigma_m_ * velocity_gait_sigma_m_;
  const double sigma_scale_sq = velocity_window_sigma_scale_ * velocity_window_sigma_scale_;
  Eigen::MatrixX3d variance(window.size(), 3);
  for (std::size_t i = 0; i < window.size(); ++i)
  {
    variance.row(i) = (sigma_scale_sq * window[i].variance.array() + gait_var).max(kVarianceFloor);
  }

  const Eigen::RowVector3d chi2 = (residual.array().square() / variance.array()).colwise().sum();
  return chi2.maxCoeff() / (n - 2.0);
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
    sample.position = yawRotation(angle) * sample.position;

    // The uncertainty turns with the point: yawRotation is Ry, so only the x-z
    // block moves and y is untouched. See the README.
    const double c = std::cos(angle);
    const double s = std::sin(angle);
    const double xx = sample.variance[0];
    const double zz = sample.variance[2];
    const double xz = sample.covariance_xz;
    sample.variance[0] = c * c * xx + 2.0 * c * s * xz + s * s * zz;
    sample.variance[2] = s * s * xx - 2.0 * c * s * xz + c * c * zz;
    sample.covariance_xz = (c * c - s * s) * xz + c * s * (zz - xx);
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

  // Least squares for p(t) = v t + b, one column per axis.
  const Eigen::MatrixX2d design = designMatrix(history);
  const Eigen::MatrixX3d observed = positions(history);
  const Eigen::ColPivHouseholderQR<Eigen::MatrixX2d> qr(design);
  if (qr.rank() < 2)
  {
    // Every sample shares a timestamp: no time base, so no slope.
    fit.variance = priorFit().variance;
    return fit;
  }
  const Eigen::Matrix<double, 2, 3> coefficients = qr.solve(observed);
  fit.velocity = coefficients.row(0);
  fit.intercept = coefficients.row(1);

  if (history.size() < kMinFitSamples)
  {
    // Two samples fit exactly, so there is no residual to estimate a variance from.
    fit.variance = priorFit().variance;
    return fit;
  }

  // Var(v) = sigma^2 (A^T A)^-1_00, with sigma^2 the residual variance at n - 2
  // degrees of freedom.
  const auto n = static_cast<double>(history.size());
  const Eigen::Vector3d residual_sq = (observed - design * coefficients).colwise().squaredNorm();
  const double slope_gain = (design.transpose() * design).inverse()(0, 0);
  fit.variance = residual_sq / (n - 2.0) * slope_gain;
  return fit;
}

Eigen::MatrixX2d PersonTracker::designMatrix(const std::deque<Sample>& history)
{
  Eigen::MatrixX2d design(history.size(), 2);
  const double t0 = history.front().stamp;
  for (std::size_t i = 0; i < history.size(); ++i)
  {
    design.row(i) << history[i].stamp - t0, 1.0;
  }
  return design;
}

Eigen::MatrixX3d PersonTracker::positions(const std::deque<Sample>& history)
{
  Eigen::MatrixX3d observed(history.size(), 3);
  for (std::size_t i = 0; i < history.size(); ++i)
  {
    observed.row(i) = history[i].position;
  }
  return observed;
}

void PersonTracker::pruneHistory(std::deque<Sample>& history, double now) const
{
  // A gap this long means the track was not observed for most of a window.
  // Absolute value because a looping bag walks the clock backwards.
  if (!history.empty() && std::abs(now - history.back().stamp) > velocity_max_gap_s_)
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
  // A track is forgotten once neither of the two things it remembers is usable any
  // more: the accepted position the spike filter compares against, and the history
  // the velocity is fitted over. Whichever outlives the other sets the lifetime,
  // so neither parameter silently truncates the other. Judged on when the track
  // was last seen, which a held frame refreshes and `stamp` deliberately does not.
  // Absolute value because a looping bag walks the clock backwards.
  const double retain_s = std::max(spike_reset_timeout_, velocity_window_max_s_);
  for (auto it = tracks_.begin(); it != tracks_.end();)
  {
    it = std::abs(now - it->second.seen_stamp) > retain_s ? tracks_.erase(it) : std::next(it);
  }
}

}  // namespace coresense_people_estimation
