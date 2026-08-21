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

#include "coresense_people_estimation/uncertainty_model.hpp"

#include <Eigen/Core>
#include <algorithm>
#include <cstddef>
#include <opencv2/core.hpp>

#include "coresense_people_estimation/depth_reader.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{
/// Keeps a user-supplied denominator off zero. These are all "full marks at"
/// scales, so 0 means "always full marks" rather than a division by zero.
constexpr double kDenominatorFloor = 1e-9;
}  // namespace

namespace coresense_people_estimation
{

UncertaintyModel::UncertaintyModel(rclcpp::Node& node)
{
  pixel_sigma_frac_ = node.declare_parameter<double>("pixel_sigma_frac", 0.25);
  depth_spread_gain_ = node.declare_parameter<double>("depth_spread_gain", 1.0);
  depth_noise_coeff_ = node.declare_parameter<double>("depth_noise_coeff", 0.0014);

  reliability_valid_frac_full_ = node.declare_parameter<double>("reliability_valid_frac_full", 0.5);
  reliability_keypoints_full_ = node.declare_parameter<double>("reliability_keypoints_full", 8.0);
  reliability_sigma_scale_ = node.declare_parameter<double>("reliability_sigma_scale", 0.25);
}

Eigen::Matrix3d UncertaintyModel::positionCovariance(const cv::Point3d& ray, const DepthEstimate& depth,
                                                     double bbox_width, double bbox_height, double fx, double fy) const
{
  const double sigma_u = pixel_sigma_frac_ * bbox_width;
  const double sigma_v = pixel_sigma_frac_ * bbox_height;
  const double noise_floor = depth_noise_coeff_ * depth.depth_m * depth.depth_m;
  // On the whole-bbox fallback the spread is the gap to the background, which
  // says nothing about how well this person's distance is known.
  const double sigma_z =
      depth.from_bbox_fallback ? noise_floor : std::max(depth_spread_gain_ * depth.spread_m, noise_floor);

  // P = ray * z, so J = [ z/fx * ex , z/fy * ey , ray ] and
  // Sigma = J diag(su^2, sv^2, sz^2) J^T.
  const double depth_variance = sigma_z * sigma_z;
  Eigen::Matrix3d covariance;
  covariance << depth_variance * ray.x * ray.x, depth_variance * ray.x * ray.y, depth_variance * ray.x * ray.z,
      depth_variance * ray.y * ray.x, depth_variance * ray.y * ray.y, depth_variance * ray.y * ray.z,
      depth_variance * ray.z * ray.x, depth_variance * ray.z * ray.y, depth_variance * ray.z * ray.z;
  const double du = depth.depth_m / fx * sigma_u;
  const double dv = depth.depth_m / fy * sigma_v;
  covariance(0, 0) += du * du;
  covariance(1, 1) += dv * dv;
  return covariance;
}

double UncertaintyModel::reliability(const DepthEstimate& depth, const Eigen::Matrix3d& covariance,
                                     std::size_t n_valid_keypoints) const
{
  // Presence and localisation, deliberately not velocity confidence. See the README.
  const double r_valid =
      std::clamp(depth.valid_frac / std::max(kDenominatorFloor, reliability_valid_frac_full_), 0.0, 1.0);
  const double r_keypoints = std::clamp(
      static_cast<double>(n_valid_keypoints) / std::max(kDenominatorFloor, reliability_keypoints_full_), 0.0, 1.0);
  const double trace = covariance(0, 0) + covariance(1, 1) + covariance(2, 2);
  const double r_sigma = reliability_sigma_scale_ / (reliability_sigma_scale_ + std::max(0.0, trace));
  return std::clamp(r_valid * r_keypoints * r_sigma, 0.0, 1.0);
}

}  // namespace coresense_people_estimation
