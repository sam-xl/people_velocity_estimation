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
#include <opencv2/core.hpp>

#include "people_velocity_estimation/depth_reader.hpp"
#include "rclcpp/rclcpp.hpp"

namespace people_velocity_estimation
{

/// How much to distrust one back-projected detection.
///
/// Turns the pixel and depth noise into a position covariance, and folds that
/// together with the depth read-out's own quality into the single scalar nav2's
/// social layer consumes. See docs/uncertainty.md.
///
/// Not a node: it declares its parameters on the owning node.
class UncertaintyModel
{
public:
  explicit UncertaintyModel(rclcpp::Node& node);

  /// Σ_P = J diag(σu², σv², σz²) Jᵀ for P = ray * depth.
  Eigen::Matrix3d positionCovariance(const cv::Point3d& ray, const DepthEstimate& depth, double bbox_width,
                                     double bbox_height, double fx, double fy) const;

  /// Presence and localisation confidence for nav2's social layer; see docs/uncertainty.md.
  double reliability(const DepthEstimate& depth, const Eigen::Matrix3d& covariance,
                     std::size_t n_valid_keypoints) const;

private:
  // ── Parameters ──────────────────────────────────────────────────────────────
  double pixel_sigma_frac_;
  double depth_spread_gain_;
  /// Quadratic coefficient of the RealSense depth noise model, sigma_z ~ c*z^2.
  /// https://dev.intelrealsense.com/docs/tuning-depth-cameras-for-best-performance
  double depth_noise_coeff_;

  double reliability_valid_frac_full_;
  double reliability_keypoints_full_;
  double reliability_sigma_scale_;
};

}  // namespace people_velocity_estimation
